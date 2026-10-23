#!/usr/bin/env python3
"""Mixer: /cmd_vel (Twist) -> gaya thruster kiri/kanan (Newton) di simulasi.

Pengaman:
- Batas waktu: tanpa /cmd_vel baru selama `batas_waktu` detik, thruster dimatikan.
- Emergency stop: True di /safety/estop mengunci thruster di 0. Kunci HANYA lepas
  lewat pesan kosong di /safety/reset, tidak lepas sendiri saat sinyal kembali.
- /safety/status melaporkan OK, TIMEOUT, atau ESTOP (1 Hz dan setiap berubah).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty, Float64, String

# Konstanta ini turunan dari model Gazebo (hambatan xU, xAbsU, nR, jarak thruster 0.2 m).
# Di kapal asli nilainya harus dikalibrasi ulang.
K_LINIER = 4.0      # N per (m/s)
K_KUADRAT = 6.0     # N per (m/s)^2
K_PUTAR = 25.0      # selisih gaya kanan-kiri (N) per (rad/s)
GAYA_MAKS = 20.0    # N per thruster


class MixerThruster(Node):
    def __init__(self):
        super().__init__("mixer_thruster")
        # detik; tanpa perintah baru, thruster dimatikan (fail-safe)
        self.declare_parameter("batas_waktu", 1.0)
        self.batas_waktu = self.get_parameter("batas_waktu").value
        self.create_subscription(Twist, "/cmd_vel", self.terima_perintah, 10)
        self.create_subscription(Bool, "/safety/estop", self.terima_estop, 10)
        self.create_subscription(Empty, "/safety/reset", self.terima_reset, 10)
        self.pub_kiri = self.create_publisher(Float64, "/thruster/kiri", 10)
        self.pub_kanan = self.create_publisher(Float64, "/thruster/kanan", 10)
        self.pub_status = self.create_publisher(String, "/safety/status", 10)
        self.vx = 0.0
        self.wz = 0.0
        self.estop = False
        self.status = ""
        self.terakhir = self.get_clock().now()
        self.create_timer(0.05, self.kirim)
        self.create_timer(1.0, self.lapor)

    def terima_perintah(self, pesan):
        self.vx = pesan.linear.x
        self.wz = pesan.angular.z
        self.terakhir = self.get_clock().now()

    def terima_estop(self, pesan):
        if pesan.data and not self.estop:
            self.estop = True
            self.get_logger().warn("EMERGENCY STOP: thruster dikunci di 0 sampai /safety/reset")

    def terima_reset(self, _):
        if self.estop:
            self.estop = False
            self.get_logger().warn("Emergency stop dilepas lewat /safety/reset")

    def lapor(self):
        self.pub_status.publish(String(data=self.status))

    def kirim(self):
        umur = (self.get_clock().now() - self.terakhir).nanoseconds / 1e9
        if self.estop:
            status, vx, wz = "ESTOP", 0.0, 0.0
        elif umur > self.batas_waktu:
            status, vx, wz = "TIMEOUT", 0.0, 0.0
        else:
            status, vx, wz = "OK", self.vx, self.wz
        if status != self.status:
            self.status = status
            self.lapor()
            if status == "TIMEOUT":
                self.get_logger().warn(f"Tidak ada /cmd_vel selama {self.batas_waktu:.1f} s: thruster dimatikan")

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