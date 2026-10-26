#!/usr/bin/env python3
"""Tambahkan elemen pedoman ASV 2026 ke dunia Gazebo yang sudah ada.

Membaca worlds/kolam_asv.sdf (hasil buat_dunia.py), lalu menulis ulang file itu
dengan tambahan:
  - dermaga awal, dermaga finish + bola biru (letak dihitung dari pelampung terakhir)
  - garis geofence (visual saja, tanpa tabrakan)
  - beberapa objek bawah air
  - kamera bawah air di link kapal yang sama dengan kamera depan
Juga menambah /kamera_bawah ke config/jembatan.yaml dan membuat config/pedoman.yaml
(parameter untuk node keselamatan, dock_biru, dashboard_web).

Aman dijalankan berulang: sumber asli disimpan di worlds/kolam_asv.sdf.asli.
Kalau kolam_asv.sdf dibuat ulang oleh buat_dunia.py, jalankan skrip ini lagi.
"""
import argparse
import copy
import os
import shutil
import statistics
import sys
import xml.etree.ElementTree as ET

PENANDA = 'dermaga_awal'


def angka(teks, n=6):
    try:
        v = [float(t) for t in (teks or '').split()]
    except ValueError:
        v = []
    return (v + [0.0] * n)[:n]


def daftar(v):
    return '[' + ', '.join(f'{a:.3f}' if isinstance(a, float) else str(a) for a in v) + ']'


# ---------------------------------------------------------------- membaca dunia
def cari_pelampung(world, nama_kapal):
    """Semua visual bola selain milik kapal: (x, y, z, merah?)."""
    hasil = []
    for m in world.findall('model'):
        if m.get('name') == nama_kapal:
            continue
        mx, my, mz = angka(m.findtext('pose'))[:3]
        for link in m.findall('link'):
            lx, ly, lz = angka(link.findtext('pose'))[:3]
            for vis in link.findall('visual'):
                if vis.find('geometry/sphere') is None:
                    continue
                vx, vy, vz = angka(vis.findtext('pose'))[:3]
                r, g, b = angka(vis.findtext('material/diffuse'), 4)[:3]
                if b > r and b > g:          # lewati bola biru yang sudah ada
                    continue
                hasil.append((mx + lx + vx, my + ly + vy, mz + lz + vz, r > g))
    return hasil


def ukuran_kapal(kapal):
    """(x_min, x_maks, panjang, lebar, massa); None kalau bentuk tidak terbaca."""
    massa, x0, x1, y0, y1 = 0.0, 1e9, -1e9, 1e9, -1e9
    for link in kapal.findall('link'):
        try:
            massa += float(link.findtext('inertial/mass') or 0)
        except ValueError:
            pass
        lx, ly, _ = angka(link.findtext('pose'))[:3]
        for e in link.findall('collision') + link.findall('visual'):
            kotak = e.find('geometry/box/size')
            if kotak is None:
                continue
            sx, sy, _ = angka(kotak.text)[:3]
            px, py, _ = angka(e.findtext('pose'))[:3]
            x0, x1 = min(x0, lx + px - sx / 2), max(x1, lx + px + sx / 2)
            y0, y1 = min(y0, ly + py - sy / 2), max(y1, ly + py + sy / 2)
    if x1 < x0:
        return None, None, None, None, massa
    return x0, x1, x1 - x0, y1 - y0, massa


# ---------------------------------------------------------------- pembuat model
def model_kotak(nama, x, y, z, sx, sy, sz, warna, kolisi=True, pancar=False):
    r, g, b = warna
    ukuran = f'{sx:.3f} {sy:.3f} {sz:.3f}'
    kol = (f'<collision name="c"><geometry><box><size>{ukuran}</size></box></geometry></collision>'
           if kolisi else '')
    em = f'<emissive>{r} {g} {b} 1</emissive>' if pancar else ''
    return ET.fromstring(
        f'<model name="{nama}"><static>true</static><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>'
        f'<link name="l">{kol}<visual name="v"><geometry><box><size>{ukuran}</size></box></geometry>'
        f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>{em}</material>'
        f'</visual></link></model>')


def model_bola_biru(x, y, z, radius, panjang_tiang):
    return ET.fromstring(
        f'<model name="bola_biru"><static>true</static><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>'
        f'<link name="l">'
        f'<collision name="c"><geometry><sphere><radius>{radius}</radius></sphere></geometry></collision>'
        f'<visual name="v"><geometry><sphere><radius>{radius}</radius></sphere></geometry>'
        f'<material><ambient>0 0.2 1 1</ambient><diffuse>0 0.2 1 1</diffuse>'
        f'<emissive>0 0.2 1 1</emissive></material></visual>'
        f'<visual name="tiang"><pose>{panjang_tiang / 2:.3f} 0 0 0 1.5708 0</pose>'
        f'<geometry><cylinder><radius>0.012</radius><length>{panjang_tiang:.3f}</length></cylinder></geometry>'
        f'<material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material></visual>'
        f'</link></model>')


def model_objek_bawah(i, x, y, z, warna):
    r, g, b = warna
    return ET.fromstring(
        f'<model name="objek_bawah_{i}"><static>true</static><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>'
        f'<link name="l"><visual name="v"><geometry><cylinder><radius>0.10</radius>'
        f'<length>0.50</length></cylinder></geometry><material><ambient>{r} {g} {b} 1</ambient>'
        f'<diffuse>{r} {g} {b} 1</diffuse></material></visual></link></model>')


def kotak_dermaga(nama, x_muka, dalam, lebar, z_atas, warna):
    """Dermaga dengan muka depan di x_muka (kapal datang dari x lebih kecil)."""
    tinggi = z_atas + 0.3            # dasar di z = -0.3
    return model_kotak(nama, x_muka + dalam / 2, 0.0, (z_atas - 0.3) / 2, dalam, lebar, tinggi, warna)


# ---------------------------------------------------------------- utama
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--paket', default=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    ap.add_argument('--kapal', default='kapal_anif')
    ap.add_argument('--jarak-finish', type=float, default=7.0, help='pelampung terakhir -> muka dermaga finish (m)')
    ap.add_argument('--diameter-bola', type=float, default=0.30)
    ap.add_argument('--pagar', type=float, nargs=4, metavar=('X0', 'X1', 'Y0', 'Y1'))
    ap.add_argument('--x-akhir', type=float, help='x pelampung terakhir (kalau tak terbaca otomatis)')
    ap.add_argument('--z-bola', type=float, help='tinggi pusat bola biru (default: setinggi pelampung, minimal di atas air)')
    ap.add_argument('--rate-bawah', type=float, default=1.0, help='laju kamera bawah air (Hz)')
    ap.add_argument('--tanpa-kamera-bawah', action='store_true')
    ap.add_argument('--turun-kamera', type=float, default=0.30, help='kamera bawah air = kamera depan dikurangi ini (m)')
    a = ap.parse_args()

    dunia_path = os.path.join(a.paket, 'worlds', 'kolam_asv.sdf')
    asli_path = dunia_path + '.asli'
    if not os.path.exists(dunia_path):
        sys.exit(f'Tidak ada {dunia_path}')
    with open(dunia_path, encoding='utf-8') as fh:
        isi = fh.read()
    if f'name="{PENANDA}"' in isi:               # sudah pernah diproses -> pakai sumber asli
        if not os.path.exists(asli_path):
            sys.exit('kolam_asv.sdf sudah diproses tapi kolam_asv.sdf.asli hilang; buat ulang dengan buat_dunia.py.')
        with open(asli_path, encoding='utf-8') as fh:
            isi = fh.read()
    else:
        shutil.copyfile(dunia_path, asli_path)

    pohon = ET.ElementTree(ET.fromstring(isi))
    world = pohon.getroot().find('world')
    if world is None:
        sys.exit('Elemen <world> tidak ditemukan.')

    kapal = next((m for m in world.findall('model') if m.get('name') == a.kapal), None)
    if kapal is None:
        sys.exit(f'Model "{a.kapal}" tidak ada di kolam_asv.sdf (pakai --kapal NAMA).')
    bx, by = angka(kapal.findtext('pose'))[:2]

    # --- ukuran & massa kapal (cek pedoman) ---
    x0k, _, pj, lb, massa = ukuran_kapal(kapal)
    print('== Cek ukuran kapal (pedoman: minimal 60 x 20 cm, minimal 7 kg termasuk baterai)')
    if pj is None:
        print('   bentuk lambung bukan <box>; ukuran tidak bisa dibaca otomatis, cek manual.')
        x0k = -0.4
    else:
        print(f'   panjang {pj * 100:.0f} cm, lebar {lb * 100:.0f} cm', '(OK)' if pj >= 0.6 and lb >= 0.2 else '<-- KURANG')
    print(f'   massa total di SDF: {massa:.2f} kg', '(OK)' if massa >= 7 else '<-- KURANG dari 7 kg')

    # --- letak pelampung ---
    pel = cari_pelampung(world, a.kapal)
    if a.x_akhir is None and len(pel) < 4:
        sys.exit('Pelampung (visual bola) tidak terbaca. Beri --x-akhir dan --z-bola.')
    x_akhir = a.x_akhir if a.x_akhir is not None else max(p[0] for p in pel)
    z_ukur = a.z_bola if a.z_bola is not None else statistics.median(p[2] for p in pel)
    z_bola = a.z_bola if a.z_bola is not None else max(z_ukur, a.diameter_bola / 2 + 0.03)   # seluruh bola di atas air
    print(f'== Pelampung terbaca: {len(pel)}; x terakhir {x_akhir:.2f}; z pusat pelampung {z_ukur:.2f}; z bola biru {z_bola:.2f}')

    # --- tata letak ---
    r_bola = a.diameter_bola / 2
    x_muka = x_akhir + a.jarak_finish
    x_bola = x_muka - 0.25
    pagar = a.pagar or [bx - 4.0, x_muka + 2.5, by - 5.0, by + 5.0]
    muka_awal = bx + x0k - 0.2
    elemen = [
        kotak_dermaga(PENANDA, muka_awal - 1.2, 1.2, 2.0, 0.25, (0.55, 0.45, 0.35)),
        kotak_dermaga('dermaga_finish', x_muka, 1.6, 3.0, z_bola + 0.35, (0.55, 0.55, 0.5)),
        model_bola_biru(x_bola, 0.0, z_bola, r_bola, x_muka - x_bola),
    ]
    x0, x1, y0, y1 = pagar
    kuning = (1.0, 0.8, 0.0)
    for i, (cx, cy, sx, sy) in enumerate([((x0 + x1) / 2, y1, x1 - x0, 0.06), ((x0 + x1) / 2, y0, x1 - x0, 0.06),
                                          (x0, (y0 + y1) / 2, 0.06, y1 - y0), (x1, (y0 + y1) / 2, 0.06, y1 - y0)]):
        elemen.append(model_kotak(f'pagar_{i}', cx, cy, 0.03, sx, sy, 0.06, kuning, kolisi=False, pancar=True))
    xs = sorted({round(p[0], 2) for p in pel})
    warna_obj = [(1, 0.5, 0), (1, 1, 0), (1, 0, 1), (0, 1, 1)]
    for n, i in enumerate((1, 3, 5, 7)):
        if i + 1 < len(xs):
            elemen.append(model_objek_bawah(n + 1, (xs[i] + xs[i + 1]) / 2, 1.0 if n % 2 == 0 else -1.0,
                                            -0.5, warna_obj[n]))

    # --- kamera bawah air ---
    fov = 1.396
    if not a.tanpa_kamera_bawah:
        sensor, link_kamera = None, None
        for link in kapal.findall('link'):
            for s in link.findall('sensor'):
                if s.get('type') in ('camera', 'rgbd_camera'):
                    sensor, link_kamera = s, link
                    break
            if sensor is not None:
                break
        if sensor is None:
            sys.exit('Sensor kamera di model kapal tidak ketemu. Jalankan dengan --tanpa-kamera-bawah dan kirim isi buat_dunia.py.')
        fov = float(sensor.findtext('camera/horizontal_fov') or fov)
        px, py, pz = angka(sensor.findtext('pose'))[:3]
        baru = copy.deepcopy(sensor)
        baru.set('name', 'kamera_bawah')
        for tag, nilai in (('pose', f'{px:.3f} {py:.3f} {pz - a.turun_kamera:.3f} 0 0.45 0'),
                           ('topic', '/kamera_bawah'), ('update_rate', f'{a.rate_bawah:g}')):
            el = baru.find(tag)
            if el is None:
                el = ET.SubElement(baru, tag)
            el.text = nilai
        for tag, nilai in (('camera/image/width', '320'), ('camera/image/height', '240')):
            el = baru.find(tag)
            if el is not None:
                el.text = nilai
        link_kamera.append(baru)
        print(f'== Kamera bawah air ditambahkan di link "{link_kamera.get("name")}" '
              f'(z = {pz - a.turun_kamera:.2f}, miring 0,45 rad, 320x240 @ {a.rate_bawah:g} Hz)')

    for e in elemen:
        world.append(e)
    ET.indent(pohon, space='  ')
    pohon.write(dunia_path, encoding='utf-8', xml_declaration=True)
    print(f'== Dunia ditulis: {dunia_path}')

    # --- jembatan ---
    if not a.tanpa_kamera_bawah:
        jp = os.path.join(a.paket, 'config', 'jembatan.yaml')
        with open(jp, encoding='utf-8') as fh:
            j = fh.read()
        if '/kamera_bawah' in j:
            print('== jembatan.yaml sudah memuat /kamera_bawah')
        else:
            if not [l for l in j.splitlines() if l.strip() and not l.lstrip().startswith('#')][0].startswith('- '):
                sys.exit('jembatan.yaml bukan daftar "- ros_topic_name: ..."; tambahkan /kamera_bawah manual.')
            with open(jp, 'a', encoding='utf-8') as fh:
                fh.write('\n' if not j.endswith('\n') else '')
                fh.write('- ros_topic_name: "/kamera_bawah"\n  gz_topic_name: "/kamera_bawah"\n'
                         '  ros_type_name: "sensor_msgs/msg/Image"\n  gz_type_name: "gz.msgs.Image"\n'
                         '  direction: GZ_TO_ROS\n')
            print('== jembatan.yaml: /kamera_bawah ditambahkan')

    # --- parameter ROS ---
    cfg = os.path.join(a.paket, 'config', 'pedoman.yaml')
    ys, xs_p = [p[1] for p in pel], [p[0] for p in pel]
    with open(cfg, 'w', encoding='utf-8') as fh:
        fh.write(f'''# Dibuat oleh tools/tambah_pedoman.py - jangan diedit tangan, jalankan ulang skripnya.
keselamatan:
  ros__parameters:
    pagar: {daftar(pagar)}
dock_biru:
  ros__parameters:
    x_aktif: {x_akhir + 0.5:.3f}
    bola_xy: {daftar([x_bola, 0.0])}
    diameter_bola: {a.diameter_bola}
    fov: {fov}
dashboard_web:
  ros__parameters:
    pagar: {daftar(pagar)}
    pelampung_x: {daftar([round(v, 3) for v in xs_p]) if pel else '[0.0]'}
    pelampung_y: {daftar([round(v, 3) for v in ys]) if pel else '[0.0]'}
    pelampung_merah: {daftar([int(p[3]) for p in pel]) if pel else '[1]'}
    dermaga_awal: {daftar([muka_awal - 0.6, 0.0, 1.2, 2.0])}
    dermaga_finish: {daftar([x_muka + 0.8, 0.0, 1.6, 3.0])}
    bola_xy: {daftar([x_bola, 0.0])}
''')
    print(f'== Parameter ditulis: {cfg}')
    print(f'== Arena geofence: x {x0:.1f}..{x1:.1f}, y {y0:.1f}..{y1:.1f} (pastikan kolam cukup luas)')
    print(f'== Bola biru di x={x_bola:.2f}, z={z_bola:.2f}; dermaga finish muka di x={x_muka:.2f}')


if __name__ == '__main__':
    main()
