#!/usr/bin/env python3
"""Mixer: /cmd_vel (Twist) -> gaya thruster kiri/kanan (Newton) di simulasi."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# Konstanta ini turunan dari model Gazebo (hambatan xU, xAbsU, nR, jarak thruster 0.2 m).
# Di kapal asli nilainya harus dikalibrasi ulang.
K_LINIER = 4.0      # N per (m/s)
K_KUADRAT = 6.0     # N per (m/s)^2
K_PUTAR = 25.0      # selisih gaya kanan-kiri (N) per (rad/s)
GAYA_MAKS = 20.0    # N per thruster
BATAS_WAKTU = 0.5   # detik; tanpa perintah baru, thruster dimatikan (fail-safe)


class MixerThruster(Node):
    def __init__(self):
        super().__init__("mixer_thruster")
        self.create_subscription(Twist, "/cmd_vel", self.terima_perintah, 10)
        self.pub_kiri = self.create_publisher(Float64, "/thruster/kiri", 10)
        self.pub_kanan = self.create_publisher(Float64, "/thruster/kanan", 10)
        self.vx = 0.0
        self.wz = 0.0
        self.terakhir = self.get_clock().now()
        self.create_timer(0.05, self.kirim)

    def terima_perintah(self, pesan):
        self.vx = pesan.linear.x
        self.wz = pesan.angular.z
        self.terakhir = self.get_clock().now()

    def kirim(self):
        umur = (self.get_clock().now() - self.terakhir).nanoseconds / 1e9
        vx, wz = (self.vx, self.wz) if umur <= BATAS_WAKTU else (0.0, 0.0)

        total = K_LINIER * vx + K_KUADRAT * vx * abs(vx)
        selisih = K_PUTAR * wz          # gaya kanan - gaya kiri; wz positif = belok kiri
        kiri = (total - selisih) / 2.0
        kanan = (total + selisih) / 2.0

        kiri = max(-GAYA_MAKS, min(GAYA_MAKS, kiri))
        kanan = max(-GAYA_MAKS, min(GAYA_MAKS, kanan))
        self.pub_kiri.publish(Float64(data=kiri))
        self.pub_kanan.publish(Float64(data=kanan))


def main():
    rclpy.init()
    node = MixerThruster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
