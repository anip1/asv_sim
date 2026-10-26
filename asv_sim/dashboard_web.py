#!/usr/bin/env python3
"""Dashboard web ASV (pedoman 2026): posisi, heading, kecepatan, gambar permukaan
dan bawah air real-time, status misi/pengaman, serta tombol E-STOP.

Buka di peramban: http://localhost:8080  (host 0.0.0.0, jadi bisa diakses dari
perangkat lain di jaringan yang sama; untuk akses panitia lewat internet pakai
tunnel, mis. cloudflared/ngrok).

Node ini juga berperan sebagai "remote": ia menerbitkan /heartbeat 5 Hz. Kalau
tombol "putus sinyal" ditekan (atau dashboard mati), node keselamatan masuk
fail-safe LINK_HILANG dan mematikan thruster.
"""
import json
import math
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger

HALAMAN = """<!doctype html><html lang="id"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>ASV Dashboard</title>
<style>
body{margin:0;background:#101418;color:#e6e6e6;font:14px system-ui,sans-serif}
header{padding:10px 16px;background:#1a2129}
main{display:grid;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));gap:12px;padding:12px}
.k{background:#1a2129;border-radius:8px;padding:10px}
img,canvas{width:100%;border-radius:6px;background:#000;display:block;margin-top:6px}
td{padding:2px 12px 2px 0}
button{padding:9px 13px;margin:8px 6px 0 0;border:0;border-radius:6px;font-weight:700;cursor:pointer;color:#fff;background:#445}
.m{background:#d33}.ok{color:#6c6}.bad{color:#f66}
</style>
<header><b>ASV KKI 2026 &mdash; Dashboard</b></header>
<main>
<div class=k>Kamera depan (permukaan)<img src="/cam/depan"></div>
<div class=k>Kamera bawah air<img src="/cam/bawah"></div>
<div class=k><table id=tel></table>
<button class=m onclick="aksi('estop')">E-STOP</button>
<button onclick="aksi('reset')">RESET</button>
<button onclick="aksi('link')">Putus/sambung sinyal (simulasi)</button>
<button onclick="aksi('jejak')">Hapus jejak</button>
<button onclick="aksi('ulang')">Mulai ulang misi</button></div>
<div class=k>Peta<canvas id=peta width=640 height=300></canvas></div>
</main>
<script>
const $=i=>document.getElementById(i);
function aksi(a){fetch('/aksi/'+a,{method:'POST'})}
function peta(S){
  const c=$('peta'),g=c.getContext('2d'),[x0,x1,y0,y1]=S.peta.pagar,W=c.width,H=c.height,m=14;
  const k=Math.min((W-2*m)/(x1-x0),(H-2*m)/(y1-y0));
  const X=x=>m+(x-x0)*k,Y=y=>H-m-(y-y0)*k;
  g.fillStyle='#0b2a3d';g.fillRect(0,0,W,H);
  g.strokeStyle='#fc3';g.setLineDash([6,4]);g.strokeRect(X(x0),Y(y1),(x1-x0)*k,(y1-y0)*k);g.setLineDash([]);
  g.fillStyle='#889';for(const d of S.peta.dermaga){g.fillRect(X(d[0]-d[2]/2),Y(d[1]+d[3]/2),d[2]*k,d[3]*k)}
  for(const p of S.peta.pel){g.fillStyle=p[2]?'#e44':'#4c4';g.beginPath();g.arc(X(p[0]),Y(p[1]),3,0,7);g.fill()}
  g.fillStyle='#38f';g.beginPath();g.arc(X(S.peta.bola[0]),Y(S.peta.bola[1]),4,0,7);g.fill();
  g.strokeStyle='#fff';g.beginPath();S.jejak.forEach((p,i)=>i?g.lineTo(X(p[0]),Y(p[1])):g.moveTo(X(p[0]),Y(p[1])));g.stroke();
  const t=S.telemetri,a=t.heading*Math.PI/180;g.save();g.translate(X(t.x),Y(t.y));g.rotate(-a);
  g.fillStyle='#fff';g.beginPath();g.moveTo(9,0);g.lineTo(-6,5);g.lineTo(-6,-5);g.fill();g.restore();
}
async function ambil(){try{
  const S=await (await fetch('/state.json')).json(),t=S.telemetri;
  const aman=S.keselamatan.startsWith('OK');
  const b=[['Posisi',`x ${t.x.toFixed(2)} &nbsp; y ${t.y.toFixed(2)} m`],['Heading',t.heading.toFixed(0)+'&deg;'],
   ['Kecepatan',t.kecepatan.toFixed(2)+' m/s'],['Misi',S.misi||'-'],['Docking',S.dock||'-'],
   ['Keselamatan',`<span class="${aman?'ok':'bad'}">${S.keselamatan||'-'}</span>`],
   ['Sinyal remote',S.link?'<span class=ok>tersambung</span>':'<span class=bad>PUTUS (simulasi)</span>']];
  $('tel').innerHTML=b.map(r=>`<tr><td>${r[0]}</td><td><b>${r[1]}</b></td></tr>`).join('');
  peta(S);}catch(e){}}
setInterval(ambil,250);
</script></html>"""


class Dashboard(Node):
    def __init__(self):
        super().__init__('dashboard_web')
        d = self.declare_parameter
        d('port', 8080)
        d('host', '0.0.0.0')
        d('fps_maks', 8.0)
        d('pagar', [-4.0, 45.0, -5.0, 5.0])
        d('pelampung_x', [0.0])
        d('pelampung_y', [0.0])
        d('pelampung_merah', [1])
        d('dermaga_awal', [0.0, 0.0, 1.0, 1.0])     # cx, cy, sx, sy
        d('dermaga_finish', [0.0, 0.0, 1.0, 1.0])
        d('bola_xy', [0.0, 0.0])
        self.p = lambda n: self.get_parameter(n).value

        self.tel = {'x': 0.0, 'y': 0.0, 'heading': 0.0, 'kecepatan': 0.0}
        self.teks = {'misi': '', 'dock': '', 'keselamatan': ''}
        self.jejak = deque(maxlen=600)
        self.jpeg, self.t_jpeg = {}, {}
        self.t_v = None
        self.link_aktif = True
        self.antrian = deque()
        self.jadwal = []            # (waktu, fungsi) untuk urutan aksi bertunda

        self.create_subscription(Odometry, '/odom_gt', self.on_odom, qos_profile_sensor_data)
        for nama, topik in (('depan', '/kamera_depan'), ('bawah', '/kamera_bawah')):
            self.create_subscription(Image, topik, lambda m, n=nama: self.on_citra(n, m),
                                     qos_profile_sensor_data)
        for kunci, topik in (('misi', '/misi/status'), ('dock', '/dock/status'),
                             ('keselamatan', '/keselamatan/status')):
            self.create_subscription(String, topik, lambda m, k=kunci: self.teks.update({k: m.data}), 10)
        self.pub_hb = self.create_publisher(Empty, '/heartbeat', 10)
        self.pub_estop = self.create_publisher(Bool, '/estop', 10)
        # jalur pengaman milik mixer/autopilot yang sudah ada
        self.pub_safety_estop = self.create_publisher(Bool, '/safety/estop', 10)
        self.pub_safety_reset = self.create_publisher(Empty, '/safety/reset', 10)
        self.pub_mulai = self.create_publisher(Empty, '/misi/mulai', 10)
        self.cli_reset = self.create_client(Trigger, '/keselamatan/reset')
        self.create_timer(0.2, lambda: self.link_aktif and self.pub_hb.publish(Empty()))
        self.create_timer(0.05, self.proses_aksi)

        server = ThreadingHTTPServer((self.p('host'), self.p('port')), self.buat_handler())
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.server = server
        self.get_logger().info(f'Dashboard: http://localhost:{self.p("port")}')

    # ---------- ROS ----------
    def on_odom(self, msg):
        q, pos = msg.pose.pose.orientation, msg.pose.pose.position
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        t = time.monotonic()
        if self.jejak and math.hypot(pos.x - self.jejak[-1][0], pos.y - self.jejak[-1][1]) > 2.0:
            self.jejak.clear()                       # lompatan posisi = reset simulasi
            self.t_v, self.tel['kecepatan'] = None, 0.0
        if self.t_v is None:
            self.t_v = (t, pos.x, pos.y)
        elif t - self.t_v[0] >= 0.3:
            dt = t - self.t_v[0]
            self.tel['kecepatan'] = math.hypot(pos.x - self.t_v[1], pos.y - self.t_v[2]) / dt
            self.t_v = (t, pos.x, pos.y)
        self.tel.update(x=pos.x, y=pos.y, heading=math.degrees(yaw))
        if not self.jejak or math.hypot(pos.x - self.jejak[-1][0], pos.y - self.jejak[-1][1]) > 0.1:
            self.jejak.append((round(pos.x, 2), round(pos.y, 2)))

    def on_citra(self, nama, msg):
        t = time.monotonic()
        if t - self.t_jpeg.get(nama, 0.0) < 1.0 / self.p('fps_maks'):
            return
        if msg.encoding not in ('rgb8', 'bgr8'):
            return
        self.t_jpeg[nama] = t
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        a = a[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            a = a[:, :, ::-1]
        ok, buf = cv2.imencode('.jpg', np.ascontiguousarray(a), [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            self.jpeg[nama] = (self.jpeg.get(nama, (0, b''))[0] + 1, buf.tobytes())

    def jadwalkan(self, *langkah):
        t = time.monotonic()
        self.jadwal += [(t + tunda, fn) for tunda, fn in langkah]

    def panggil_reset(self):
        if self.cli_reset.service_is_ready():
            fut = self.cli_reset.call_async(Trigger.Request())
            fut.add_done_callback(lambda f: self.get_logger().info(f'Reset pengaman: {f.result().message}'))

    def proses_aksi(self):
        P = self
        while self.antrian:
            a = self.antrian.popleft()
            if a == 'estop':
                self.jadwalkan((0, lambda: P.pub_estop.publish(Bool(data=True))),
                               (0, lambda: P.pub_safety_estop.publish(Bool(data=True))))
            elif a == 'reset':      # lepas semua kunci; misi tetap menunggu (pakai Mulai ulang)
                self.jadwalkan((0, lambda: P.pub_estop.publish(Bool(data=False))),
                               (0, lambda: P.pub_safety_reset.publish(Empty())),
                               (0.3, self.panggil_reset))
            elif a == 'ulang':      # misi baru dari gerbang 1 (setelah reset Gazebo, E-STOP, atau selesai)
                self.jejak.clear()
                self.jadwalkan((0, lambda: P.pub_safety_estop.publish(Bool(data=True))),   # autopilot: misi baru
                               (0.5, lambda: P.pub_estop.publish(Bool(data=False))),
                               (0.5, lambda: P.pub_safety_reset.publish(Empty())),
                               (0.8, self.panggil_reset),
                               (1.2, lambda: P.pub_mulai.publish(Empty())))
                self.get_logger().info('Mulai ulang misi.')
            elif a == 'jejak':
                self.jejak.clear()
            elif a == 'link':
                self.link_aktif = not self.link_aktif
                self.get_logger().warn('Heartbeat ' + ('DISAMBUNG' if self.link_aktif else 'DIPUTUS (simulasi)'))
        if self.jadwal:
            sekarang = time.monotonic()
            siap = [fn for t, fn in self.jadwal if t <= sekarang]
            self.jadwal = [(t, fn) for t, fn in self.jadwal if t > sekarang]
            for fn in siap:
                fn()

    # ---------- HTTP ----------
    def keadaan(self):
        px, py, pm = self.p('pelampung_x'), self.p('pelampung_y'), self.p('pelampung_merah')
        return {
            'telemetri': self.tel, 'link': self.link_aktif, **self.teks,
            'jejak': list(self.jejak),
            'peta': {'pagar': list(self.p('pagar')),
                     'pel': [[x, y, m] for x, y, m in zip(px, py, pm)],
                     'dermaga': [list(self.p('dermaga_awal')), list(self.p('dermaga_finish'))],
                     'bola': list(self.p('bola_xy'))}}

    def buat_handler(self):
        node = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def kirim(self, kode, tipe, isi):
                self.send_response(kode)
                self.send_header('Content-Type', tipe)
                self.send_header('Content-Length', str(len(isi)))
                self.end_headers()
                self.wfile.write(isi)

            def do_GET(self):
                if self.path == '/':
                    self.kirim(200, 'text/html; charset=utf-8', HALAMAN.encode())
                elif self.path == '/state.json':
                    self.kirim(200, 'application/json', json.dumps(node.keadaan()).encode())
                elif self.path in ('/cam/depan', '/cam/bawah'):
                    self.stream(self.path[5:])
                else:
                    self.kirim(404, 'text/plain', b'tidak ada')

            def do_POST(self):
                if self.path.startswith('/aksi/'):
                    node.antrian.append(self.path[6:])
                    self.kirim(200, 'text/plain', b'ok')
                else:
                    self.kirim(404, 'text/plain', b'tidak ada')

            def stream(self, nama):
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                terakhir = -1
                try:
                    while rclpy.ok():
                        seq, jpg = node.jpeg.get(nama, (0, b''))
                        if not jpg or seq == terakhir:
                            time.sleep(0.05)
                            continue
                        terakhir = seq
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                         + str(len(jpg)).encode() + b'\r\n\r\n' + jpg + b'\r\n')
                except (BrokenPipeError, ConnectionResetError):
                    pass

        return H


def main():
    rclpy.init()
    node = Dashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
