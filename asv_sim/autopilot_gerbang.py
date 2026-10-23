#!/usr/bin/env python3
"""Autopilot gerbang ASV: kamera -> pelampung -> gerbang bernomor -> mesin status misi -> /cmd_vel."""
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty, String

from asv_sim.gerbang import cari_gerbang, kotak_ke_bola, titik_sederhana
from asv_sim.misi import Misi, Status

# ---- Samakan dengan SDF ----
FOV_HORIZONTAL = 1.396    # rad, sama dengan <horizontal_fov> kamera
# (DIAMETER_BOLA dan LEBAR_GERBANG ada di gerbang.py)

# ---- Rentang warna HSV (skala OpenCV: H 0-180) ----
MERAH = [((0, 120, 70), (8, 255, 255)), ((172, 120, 70), (180, 255, 255))]
HIJAU = [((40, 120, 50), (85, 255, 255))]
LUAS_MIN = 8              # piksel persegi; blob lebih kecil dianggap noise
BOLA_MAKS = 6             # paling banyak sekian blob per warna


def cari_semua_bola(hsv, rentang):
    """Kotak (x, y, w, h) semua blob pada rentang warna, terbesar (= terdekat) dulu."""
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for bawah, atas in rentang:
        mask |= cv2.inRange(hsv, np.array(bawah), np.array(atas))
    kontur, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    kotak = [cv2.boundingRect(k) for k in kontur if cv2.contourArea(k) >= LUAS_MIN]
    kotak.sort(key=lambda b: b[2] * b[3], reverse=True)
    return kotak[:BOLA_MAKS]


class AutopilotGerbang(Node):
    def __init__(self):
        super().__init__("autopilot_gerbang")
        self.declare_parameter("auto_mulai", True)           # False = tunggu /misi/mulai
        self.declare_parameter("pilih_target", "lama")       # "lama" atau "pasangan"
        self.pilih_target = self.get_parameter("pilih_target").value
        self.misi = Misi(log=self.get_logger().info,
                         auto_mulai=self.get_parameter("auto_mulai").value)
        self.ringkas_lalu = ""
        self.estop = False
        self.pose_gt = None
        self.create_subscription(Image, "/kamera_depan", self.proses, 1)
        self.create_subscription(Empty, "/misi/mulai", lambda _: self.misi.mulai(), 1)
        self.create_subscription(Bool, "/safety/estop", self.terima_estop, 10)
        self.create_subscription(Empty, "/safety/reset", self.terima_reset, 10)
        self.create_subscription(Odometry, "/odom_gt", self.terima_odom, 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_debug = self.create_publisher(Image, "/debug/gambar", 1)
        self.pub_status = self.create_publisher(String, "/misi/status", 10)
        self.pub_terlihat = self.create_publisher(String, "/misi/terlihat", 10)
        self.get_logger().info(f"Pemilihan target: {self.pilih_target}")

    def terima_odom(self, pesan):
        """Pose benar dari Gazebo, hanya untuk memeriksa perkiraan kamera."""
        p = pesan.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose_gt = (p.position.x, p.position.y, yaw)

    def teks_pose(self):
        if self.pose_gt is None:
            return "belum ada"
        x, y, yaw = self.pose_gt
        return f"x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f}"

    def terima_estop(self, pesan):
        if pesan.data and not self.estop:
            self.estop = True
            self.get_logger().warn("EMERGENCY STOP: misi dihentikan")
            # misi baru yang menunggu START: hitungan gerbang kembali ke 0
            self.misi = Misi(log=self.get_logger().info, auto_mulai=False)

    def terima_reset(self, _):
        if self.estop:
            self.estop = False
            self.get_logger().warn("Emergency stop dilepas. Kirim /misi/mulai untuk mulai lagi")

    def waktu(self, msg):
        """Waktu simulasi dari header gambar (ikut RTF Gazebo); cadangan: jam node."""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        return t

    @staticmethod
    def ringkas(daftar, nomor_awal):
        """Teks gerbang yang terlihat, mis. 'G2 2.4m lengkap; G3 5.1m lengkap'."""
        bagian = []
        for i, g in enumerate(daftar[:3]):
            jenis = "lengkap" if g.lengkap else ("merah saja" if g.merah else "hijau saja")
            bagian.append(f"G{nomor_awal + i} {g.titik[0]:.1f}m {jenis}")
        return "; ".join(bagian)

    def proses(self, msg):
        if self.estop:
            self.pub_cmd.publish(Twist())
            self.pub_status.publish(String(data="ESTOP"))
            return
        lebar, tinggi = msg.width, msg.height
        fx = (lebar / 2.0) / math.tan(FOV_HORIZONTAL / 2.0)
        gambar = np.frombuffer(msg.data, np.uint8).reshape(tinggi, lebar, 3).copy()
        hsv = cv2.cvtColor(gambar, cv2.COLOR_RGB2HSV)

        merah = [kotak_ke_bola(k, lebar, fx) for k in cari_semua_bola(hsv, MERAH)]
        hijau = [kotak_ke_bola(k, lebar, fx) for k in cari_semua_bola(hsv, HIJAU)]
        daftar = cari_gerbang(merah, hijau)          # gerbang terurut dari yang terdekat

        # Titik bidik (maju, kiri) di kerangka kapal. Merah = sisi kiri gerbang.
        if self.pilih_target == "pasangan":
            titik = daftar[0].titik if daftar else None
        else:
            titik = titik_sederhana(merah, hijau)

        # Keputusan diambil oleh mesin status misi
        status_lalu = self.misi.status
        xy = self.pose_gt[:2] if self.pose_gt is not None else None
        perintah = self.misi.langkah(self.waktu(msg), titik, xy)
        cmd = Twist()
        cmd.linear.x = perintah.maju
        cmd.angular.z = perintah.putar
        self.pub_cmd.publish(cmd)
        self.pub_status.publish(String(data=self.misi.teks_status()))

        # Gerbang terdekat yang terlihat = gerbang berikutnya (saat LEWAT: yang sesudahnya)
        nomor_awal = self.misi.gerbang_lewat + 1 + (1 if self.misi.status == Status.LEWAT else 0)
        ringkas = self.ringkas(daftar, nomor_awal)
        self.pub_terlihat.publish(String(data=ringkas))
        if self.misi.status != status_lalu:
            self.get_logger().info(f"  terlihat sebelum: {self.ringkas_lalu or '-'} | "
                                   f"sekarang: {ringkas or '-'} | pose benar: {self.teks_pose()}")
        self.ringkas_lalu = ringkas

        self.kirim_debug(msg, gambar, daftar, nomor_awal, titik, fx, cmd)

    def kirim_debug(self, msg, gambar, daftar, nomor_awal, titik, fx, cmd):
        for i, g in enumerate(daftar[:4]):
            warna = (255, 255, 0) if i == 0 else (170, 170, 170)
            kotak_ada = [b.kotak for b in (g.merah, g.hijau) if b is not None]
            for x, y, w, h in kotak_ada:
                cv2.rectangle(gambar, (x, y), (x + w, y + h), warna, 1)
            if g.lengkap:
                x1, y1, w1, h1 = g.merah.kotak
                x2, y2, w2, h2 = g.hijau.kotak
                cv2.line(gambar, (x1 + w1 // 2, y1 + h1 // 2),
                         (x2 + w2 // 2, y2 + h2 // 2), warna, 1)
            x0, y0 = kotak_ada[0][0], kotak_ada[0][1]
            cv2.putText(gambar, f"G{nomor_awal + i}", (x0, max(y0 - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, warna, 1)
        if titik:
            u = int(msg.width / 2.0 - fx * titik[1] / max(titik[0], 0.1))
            cv2.line(gambar, (u, 0), (u, msg.height), (255, 255, 255), 1)
        cv2.putText(gambar, f"v={cmd.linear.x:.2f} w={cmd.angular.z:+.2f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(gambar, self.misi.teks_status(), (10, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        out = Image()
        out.header = msg.header
        out.height, out.width = msg.height, msg.width
        out.encoding = "rgb8"
        out.step = msg.width * 3
        out.data = gambar.tobytes()
        self.pub_debug.publish(out)


def main():
    rclpy.init()
    node = AutopilotGerbang()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()