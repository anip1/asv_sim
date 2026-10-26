#!/usr/bin/env python3
"""Pengaman ASV (pedoman 2026): emergency stop, geofence, fail-safe sinyal hilang.

Posisi di rantai perintah:
    autopilot  -> /cmd_vel_auto --+
                                  +--> [keselamatan] --> /cmd_vel --> mixer --> thruster
    dock_biru  -> /cmd_vel_dock --+

Aturan: kalau ada alasan berhenti (ESTOP, PAGAR, LINK_HILANG, POSISI_HILANG),
keluaran selalu nol. Kalau aman, perintah dari dock_biru didahulukan
(selama masih segar), lalu dari autopilot.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger


def batas(x, m):
    return max(-m, min(m, x))


class Keselamatan(Node):
    def __init__(self):
        super().__init__('keselamatan')
        d = self.declare_parameter
        d('pagar', [-4.0, 45.0, -5.0, 5.0])  # x_min, x_maks, y_min, y_maks (m)
        d('margin_pagar', 1.0)      # m dari tepi pagar: kecepatan maju dibatasi
        d('v_dekat_pagar', 0.15)
        d('v_maks', 0.5)
        d('w_maks', 0.8)
        d('batas_link', 2.0)        # detik tanpa heartbeat -> fail-safe
        d('tahan_pulih', 1.0)       # detik heartbeat stabil sebelum boleh jalan lagi
        d('batas_cmd', 0.5)         # detik umur maksimum perintah autopilot
        d('batas_odom', 1.0)        # detik umur maksimum data posisi
        d('laju_v', 1.0)            # m/s^2: laju perubahan maju maksimum (0 = mati)
        d('laju_w', 2.5)            # rad/s^2: laju perubahan putar maksimum (0 = mati)
        d('wajib_link', True)
        d('wajib_odom', True)
        self.p = lambda n: self.get_parameter(n).value

        self.cmd = {'dock': (None, -1e9), 'auto': (None, -1e9)}
        self.t_hb = -1e9
        self.t_segar = None
        self.estop_tombol = False
        self.estop_kunci = False
        self.pagar_kunci = False
        self.pos = None
        self.t_odom = -1e9
        self.alasan = None
        self.t_status = 0.0
        self.v_lalu = self.w_lalu = 0.0
        self.t_lalu = time.monotonic()

        self.create_subscription(Twist, '/cmd_vel_auto', lambda m: self.terima('auto', m), 10)
        self.create_subscription(Twist, '/cmd_vel_dock', lambda m: self.terima('dock', m), 10)
        self.create_subscription(Empty, '/heartbeat', self.on_hb, 10)
        self.create_subscription(Bool, '/estop', self.on_estop, 10)
        self.create_subscription(Odometry, '/odom_gt', self.on_odom, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/keselamatan/status', 10)
        self.create_service(Trigger, '/keselamatan/reset', self.on_reset)
        self.create_timer(0.05, self.tick)
        self.get_logger().info('Keselamatan siap. Menunggu heartbeat di /heartbeat '
                               'dan posisi di /odom_gt.')

    # ---------- callback ----------
    def terima(self, sumber, msg):
        self.cmd[sumber] = (msg, time.monotonic())

    def on_hb(self, _):
        self.t_hb = time.monotonic()

    def on_estop(self, msg):
        self.estop_tombol = msg.data
        if msg.data and not self.estop_kunci:
            self.estop_kunci = True
            self.get_logger().error('E-STOP DITEKAN: thruster dimatikan.')

    def on_odom(self, msg):
        baru = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        lompat = self.pos is not None and math.hypot(baru[0] - self.pos[0], baru[1] - self.pos[1]) > 2.0
        self.pos = baru
        self.t_odom = time.monotonic()
        if lompat and self.pagar_kunci and self.di_dalam():   # reset simulasi: kapal kembali ke dalam pagar
            self.pagar_kunci = False
            self.get_logger().info('Posisi kembali ke dalam pagar (reset simulasi): kunci PAGAR dilepas.')

    def on_reset(self, _req, res):
        if self.estop_tombol:
            res.success, res.message = False, 'Lepas dulu tombol E-STOP (/estop false).'
        elif self.pagar_kunci and not self.di_dalam():
            res.success, res.message = False, 'Kapal masih di luar pagar; pindahkan dulu.'
        else:
            self.estop_kunci = self.pagar_kunci = False
            res.success, res.message = True, 'Reset berhasil.'
            self.get_logger().info('Reset pengaman.')
        return res

    # ---------- logika ----------
    def di_dalam(self):
        x0, x1, y0, y1 = self.p('pagar')
        x, y = self.pos
        return x0 <= x <= x1 and y0 <= y <= y1

    def dekat_tepi(self):
        x0, x1, y0, y1 = self.p('pagar')
        x, y = self.pos
        return min(x - x0, x1 - x, y - y0, y1 - y) < self.p('margin_pagar')

    def pilih(self, now):
        msg, t = self.cmd['dock']
        if msg is not None and now - t < 0.3:
            return 'dock', msg
        msg, t = self.cmd['auto']
        if msg is not None and now - t < self.p('batas_cmd'):
            return 'auto', msg
        return 'diam', None

    def tick(self):
        now = time.monotonic()

        # posisi + geofence
        odom_ok = self.pos is not None and now - self.t_odom < self.p('batas_odom')
        if odom_ok and not self.pagar_kunci and not self.di_dalam():
            self.pagar_kunci = True
            self.get_logger().error('GEOFENCE dilanggar: thruster dimatikan.')

        # sinyal (heartbeat)
        if now - self.t_hb < self.p('batas_link'):
            if self.t_segar is None:
                self.t_segar = now
            link_ok = now - self.t_segar >= self.p('tahan_pulih')
        else:
            self.t_segar, link_ok = None, False

        if self.estop_kunci:
            alasan = 'ESTOP'
        elif self.pagar_kunci:
            alasan = 'PAGAR'
        elif self.p('wajib_link') and not link_ok:
            alasan = 'LINK_HILANG'
        elif self.p('wajib_odom') and not odom_ok:
            alasan = 'POSISI_HILANG'
        else:
            alasan = 'OK'

        out = Twist()
        sumber = '-'
        dt = min(max(now - self.t_lalu, 0.0), 0.2)
        self.t_lalu = now
        v = w = 0.0
        if alasan == 'OK':
            sumber, cmd = self.pilih(now)
            if cmd is not None:
                v = batas(cmd.linear.x, self.p('v_maks'))
                w = batas(cmd.angular.z, self.p('w_maks'))
                if odom_ok and self.dekat_tepi():
                    v = min(v, self.p('v_dekat_pagar'))
            # haluskan lompatan mendadak (mis. saat autopilot ganti gerbang)
            if self.p('laju_v') > 0:
                dv = self.p('laju_v') * dt
                v = self.v_lalu + batas(v - self.v_lalu, dv)
            if self.p('laju_w') > 0:
                dw = self.p('laju_w') * dt
                w = self.w_lalu + batas(w - self.w_lalu, dw)
        # alasan berhenti: nol seketika, tanpa dihaluskan
        self.v_lalu, self.w_lalu = v, w
        out.linear.x, out.angular.z = float(v), float(w)
        self.pub.publish(out)

        teks = alasan if alasan != 'OK' else f'OK ({sumber})'
        if alasan != self.alasan:
            if self.alasan is not None:
                self.get_logger().warn(f'Status pengaman: {self.alasan} -> {alasan}')
            self.alasan = alasan
            self.t_status = 0.0
        if now - self.t_status > 0.5:
            self.t_status = now
            self.pub_status.publish(String(data=teks))


def main():
    rclpy.init()
    node = Keselamatan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
