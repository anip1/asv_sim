#!/usr/bin/env python3
"""Merapat ke bola biru di dermaga finish (pedoman 2026), hanya dari kamera.

Aktif kalau salah satu terpenuhi:
  1. /misi/status memuat salah satu teks `pemicu_status` (default DOCKING atau SELESAI,
     tahap-tahap misi.py sesudah gerbang ke-10) DAN kapal sudah dekat ujung lintasan
     (x >= x_aktif - 6 m), atau
  2. posisi x kapal >= `x_aktif` (sudah melewati gerbang terakhir; jaring pengaman
     bila hitungan gerbang di autopilot macet), atau
  3. parameter `paksa_aktif` true.
Kembali MENUNGGU bila kapal melompat (reset Gazebo) atau /safety/estop ditekan.
Sesudah aktif, node ini mengirim /cmd_vel_dock yang didahulukan oleh node
keselamatan atas /cmd_vel_auto.

Status: MENUNGGU -> CARI -> DEKATI -> SANDAR -> SELESAI
Posisi bola dari file konfigurasi HANYA dipakai untuk mencetak galat akhir
(penilaian); kendali memakai kamera saja.
"""
import math
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


def batas(x, lo, hi):
    return max(lo, min(hi, x))


def citra_bgr(msg):
    """sensor_msgs/Image (rgb8 atau bgr8) -> array BGR, atau None."""
    if msg.encoding not in ('rgb8', 'bgr8'):
        return None
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    a = a[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return np.ascontiguousarray(a[:, :, ::-1] if msg.encoding == 'rgb8' else a)


class DockBiru(Node):
    def __init__(self):
        super().__init__('dock_biru')
        d = self.declare_parameter
        d('pemicu_status', ['DOCKING', 'SELESAI'])
        d('x_aktif', 1.0e9)
        d('paksa_aktif', False)
        d('bola_xy', [0.0, 0.0])        # hanya untuk penilaian akhir
        d('diameter_bola', 0.30)        # m
        d('fov', 1.396)                 # rad, horizontal
        d('hsv_bawah', [100, 200, 170])  # biru jenuh & terang (OpenCV: H 0-179)
        d('hsv_atas', [130, 255, 255])
        d('jarak_henti', 0.40)          # m, kamera ke pusat bola
        d('v_maks', 0.35)
        d('v_min', 0.06)
        d('k_putar', 0.9)
        d('w_maks', 0.5)
        d('w_cari', 0.3)
        self.p = lambda n: self.get_parameter(n).value

        self.state = 'MENUNGGU'
        self.bola = None            # (waktu, e, jarak, px)
        self.d_akhir, self.t_akhir = 9.9, -1e9
        self.t_state = time.monotonic()
        self.t_stabil = None
        self.status_misi = ''
        self.pose = None            # x, y, yaw
        self.warn_air = 0.0
        self.status_basi = None     # status misi saat reset; diabaikan sampai berubah

        self.create_subscription(Image, '/kamera_depan', self.on_citra, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom_gt', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(String, '/misi/status', self.on_misi, 10)
        self.create_subscription(Bool, '/safety/estop', self.on_estop, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel_dock', 10)
        self.pub_status = self.create_publisher(String, '/dock/status', 10)
        self.create_timer(0.1, self.tick)
        self.create_timer(1.0, self.lapor)

    # ---------- masukan ----------
    def on_misi(self, msg):
        if self.status_basi is not None and msg.data != self.status_basi:
            self.status_basi = None
        self.status_misi = msg.data

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if self.pose is not None and math.hypot(x - self.pose[0], y - self.pose[1]) > 2.0:
            self.setelah_lompat()
        self.pose = (x, y, yaw)

    def setelah_lompat(self):
        """Kapal tiba-tiba pindah >2 m (reset simulasi): mulai dari awal lagi."""
        self.get_logger().warn('Lompatan posisi (reset simulasi?): docking kembali MENUNGGU.')
        self.siaga_lagi()

    def on_estop(self, msg):
        if msg.data:
            self.get_logger().info('E-STOP: docking kembali MENUNGGU.')
            self.siaga_lagi()

    def siaga_lagi(self):
        self.bola, self.d_akhir, self.t_akhir = None, 9.9, -1e9
        self.status_basi = self.status_misi        # jangan aktif lagi gara-gara status lama
        if self.state != 'MENUNGGU':
            self.ganti('MENUNGGU')

    def on_citra(self, msg):
        if self.state == 'MENUNGGU':
            return
        img = citra_bgr(msg)
        if img is None:
            return
        h, w = img.shape[:2]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(self.p('hsv_bawah')), np.array(self.p('hsv_atas')))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        now = time.monotonic()
        if mask.mean() / 255.0 > 0.25 and now - self.warn_air > 5.0:
            self.warn_air = now
            self.get_logger().warn('>25% piksel terbaca biru (air?). Naikkan hsv_bawah[1]/[2].')
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        terbaik = None
        for c in cnts:
            luas = cv2.contourArea(c)
            if luas < 20:
                continue
            (cx, cy), r = cv2.minEnclosingCircle(c)
            if luas < 0.40 * math.pi * r * r:      # bukan bulat
                continue
            if terbaik is None or luas > terbaik[0]:
                terbaik = (luas, cx, r)
        if terbaik is None:
            return
        _, cx, r = terbaik
        fx = (w / 2.0) / math.tan(self.p('fov') / 2.0)
        jarak = fx * self.p('diameter_bola') / (2.0 * r)
        e = (cx - w / 2.0) / (w / 2.0)             # + = bola di kanan
        self.bola = (now, e, jarak, 2 * r)
        self.d_akhir, self.t_akhir = jarak, now

    # ---------- kendali ----------
    def ganti(self, baru):
        self.get_logger().info(f'Dock: {self.state} -> {baru}')
        self.state, self.t_state, self.t_stabil = baru, time.monotonic(), None

    def aktif(self):
        if self.p('paksa_aktif'):
            return True
        if self.pose is None:
            return False
        x, x_aktif = self.pose[0], self.p('x_aktif')
        dari_misi = (self.status_basi is None
                     and any(t in self.status_misi for t in self.p('pemicu_status')))
        return x >= x_aktif or (dari_misi and x >= x_aktif - 6.0)

    def tick(self):
        now = time.monotonic()
        if self.state == 'MENUNGGU':
            if self.aktif():
                self.ganti('CARI')
            return

        b = self.bola if self.bola and now - self.bola[0] < 1.2 else None
        v = w = 0.0
        k, wm = self.p('k_putar'), self.p('w_maks')
        henti = self.p('jarak_henti')

        if self.state == 'CARI':
            if b:
                self.ganti('DEKATI')
            else:   # berputar terus ke satu arah: satu putaran penuh pasti melihat bola
                w = self.p('w_cari')

        elif self.state == 'DEKATI':
            if b:
                _, e, jarak, _ = b
                w = batas(-k * e, -wm, wm)
                v = batas(0.35 * (jarak - henti), self.p('v_min'), self.p('v_maks'))
                v *= max(0.0, 1.0 - 1.5 * abs(e))      # belum lurus -> pelan/berputar saja
                if jarak <= henti + 0.05 and abs(e) < 0.2:
                    self.ganti('SANDAR')
            else:
                hilang = now - self.t_akhir
                if self.d_akhir < 1.0 and hilang < 3.0:
                    v = self.p('v_min')                # lurus buta: bola terlalu dekat
                elif self.d_akhir < 1.0:
                    self.ganti('SELESAI')
                    self.get_logger().warn('Bola hilang dari pandangan di jarak dekat.')
                elif hilang > 1.5:
                    self.ganti('CARI')

        elif self.state == 'SANDAR':       # tahan posisi di jarak_henti
            if b:
                _, e, jarak, _ = b
                gal = jarak - henti
                w = batas(-k * e, -0.3, 0.3)
                if abs(gal) > 0.05:
                    v = batas(0.5 * gal, -0.10, 0.10)
                if gal < 0.07 and abs(e) < 0.15:     # lebih dekat dari titik henti juga dianggap sandar
                    self.t_stabil = self.t_stabil or now
                    if now - self.t_stabil > 3.0:
                        self.ganti('SELESAI')
                        self.nilai()
                else:
                    self.t_stabil = None

        # MENUNGGU tidak sampai sini; SELESAI terus mengirim nol agar kapal tetap diam
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.pub.publish(cmd)

    def nilai(self):
        """Galat akhir terhadap bola (data kebenaran Gazebo, bukan untuk kendali)."""
        if self.pose is None:
            return
        bx, by = self.p('bola_xy')
        x, y, yaw = self.pose
        dx, dy = bx - x, by - y
        depan = math.cos(yaw) * dx + math.sin(yaw) * dy
        samping = -math.sin(yaw) * dx + math.cos(yaw) * dy
        self.get_logger().info(
            f'SELESAI. Bola {depan:.2f} m di depan titik asal kapal, simpangan samping '
            f'{samping * 100:.0f} cm (kamera ada 0,4 m di depan titik asal).')

    def lapor(self):
        teks = self.state
        if self.state in ('DEKATI', 'SANDAR') and self.bola:
            teks += f' jarak={self.d_akhir:.2f}m'
        self.pub_status.publish(String(data=teks))


def main():
    rclpy.init()
    node = DockBiru()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
