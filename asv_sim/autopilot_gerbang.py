#!/usr/bin/env python3
"""Autopilot gerbang ASV: kamera -> deteksi pelampung merah/hijau -> /cmd_vel."""
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

# ---- Samakan dengan SDF dan pedoman ----
FOV_HORIZONTAL = 1.396    # rad, sama dengan <horizontal_fov> kamera
DIAMETER_BOLA = 0.24      # m
LEBAR_GERBANG = 2.0       # m

# ---- Kendali ----
V_JELAJAH = 0.5           # m/s
V_MIN = 0.2               # m/s
GAIN_BELOK = 1.5
BELOK_MAKS = 0.8          # rad/s

# ---- Rentang warna HSV (skala OpenCV: H 0-180) ----
MERAH = [((0, 120, 70), (8, 255, 255)), ((172, 120, 70), (180, 255, 255))]
HIJAU = [((40, 120, 50), (85, 255, 255))]
LUAS_MIN = 8              # piksel persegi; blob lebih kecil dianggap noise


def cari_bola(hsv, rentang):
    """Kotak (x, y, w, h) blob terbesar (= terdekat) pada rentang warna, atau None."""
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for bawah, atas in rentang:
        mask |= cv2.inRange(hsv, np.array(bawah), np.array(atas))
    kontur, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    terbaik = None
    for k in kontur:
        if cv2.contourArea(k) < LUAS_MIN:
            continue
        kotak = cv2.boundingRect(k)
        if terbaik is None or kotak[2] * kotak[3] > terbaik[2] * terbaik[3]:
            terbaik = kotak
    return terbaik


class AutopilotGerbang(Node):
    def __init__(self):
        super().__init__("autopilot_gerbang")
        self.create_subscription(Image, "/kamera_depan", self.proses, 1)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_debug = self.create_publisher(Image, "/debug/gambar", 1)

    @staticmethod
    def ke_kerangka_kapal(kotak, lebar_gambar, fx):
        """Piksel -> (maju, kiri) dalam meter, memakai ukuran bola yang diketahui."""
        x, _, w, _ = kotak
        maju = fx * DIAMETER_BOLA / max(w, 1)
        kanan = (x + w / 2.0 - lebar_gambar / 2.0) * maju / fx
        return maju, -kanan

    def proses(self, msg):
        lebar, tinggi = msg.width, msg.height
        fx = (lebar / 2.0) / math.tan(FOV_HORIZONTAL / 2.0)
        gambar = np.frombuffer(msg.data, np.uint8).reshape(tinggi, lebar, 3).copy()
        hsv = cv2.cvtColor(gambar, cv2.COLOR_RGB2HSV)

        merah = cari_bola(hsv, MERAH)
        hijau = cari_bola(hsv, HIJAU)

        # Titik bidik (maju, kiri) di kerangka kapal. Merah = sisi kiri gerbang
        # (sesuai dunia buatan skrip). Cocokkan dengan Petunjuk Teknis.
        titik = None
        if merah and hijau:
            m = self.ke_kerangka_kapal(merah, lebar, fx)
            h = self.ke_kerangka_kapal(hijau, lebar, fx)
            titik = ((m[0] + h[0]) / 2.0, (m[1] + h[1]) / 2.0)
        elif merah:
            m = self.ke_kerangka_kapal(merah, lebar, fx)
            titik = (m[0], m[1] - LEBAR_GERBANG / 2.0)
        elif hijau:
            h = self.ke_kerangka_kapal(hijau, lebar, fx)
            titik = (h[0], h[1] + LEBAR_GERBANG / 2.0)

        cmd = Twist()
        if titik is None:
            cmd.linear.x = V_MIN                      # tak ada pelampung: lurus pelan
        else:
            galat = math.atan2(titik[1], max(titik[0], 0.1))   # + = target di kiri
            cmd.angular.z = max(-BELOK_MAKS, min(BELOK_MAKS, GAIN_BELOK * galat))
            faktor = max(0.0, 1.0 - abs(galat) / 0.8)           # makin tajam, makin pelan
            cmd.linear.x = V_MIN + (V_JELAJAH - V_MIN) * faktor
        self.pub_cmd.publish(cmd)

        self.kirim_debug(msg, gambar, merah, hijau, titik, fx, cmd)

    def kirim_debug(self, msg, gambar, merah, hijau, titik, fx, cmd):
        for kotak in (merah, hijau):
            if kotak:
                x, y, w, h = kotak
                cv2.rectangle(gambar, (x, y), (x + w, y + h), (255, 255, 0), 1)
        if titik:
            u = int(msg.width / 2.0 - fx * titik[1] / max(titik[0], 0.1))
            cv2.line(gambar, (u, 0), (u, msg.height), (255, 255, 255), 1)
        cv2.putText(gambar, f"v={cmd.linear.x:.2f} w={cmd.angular.z:+.2f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
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
