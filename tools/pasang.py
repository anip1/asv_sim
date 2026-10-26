#!/usr/bin/env python3
"""Pasang tambahan pedoman ASV 2026 ke paket asv_sim.

Dijalankan dari dalam paket:  python3 tools/pasang.py [opsi tambah_pedoman.py]
Langkah: cek paket -> daftarkan node di setup.py -> cek dependensi ->
         cek tipe /cmd_vel di autopilot -> proses dunia + jembatan + parameter.
"""
import os
import re
import shutil
import subprocess
import sys

PAKET = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ENTRI = ['keselamatan = asv_sim.keselamatan:main',
         'dock_biru = asv_sim.dock_biru:main',
         'dashboard = asv_sim.dashboard_web:main']


def sekarang(teks):
    print(f'\n== {teks}')


def main():
    setup = os.path.join(PAKET, 'setup.py')
    modul = os.path.join(PAKET, 'asv_sim')
    if not (os.path.exists(setup) and os.path.isdir(modul)):
        sys.exit(f'Ini bukan folder paket asv_sim: {PAKET}')
    for f in ('keselamatan.py', 'dock_biru.py', 'dashboard_web.py'):
        if not os.path.exists(os.path.join(modul, f)):
            sys.exit(f'asv_sim/{f} belum ada. Ekstrak arsip ke folder paket dulu.')

    sekarang('setup.py: daftarkan node')
    with open(setup, encoding='utf-8') as fh:
        teks = fh.read()
    baru = [e for e in ENTRI if e.split(' = ')[0] + ' =' not in teks]
    if not baru:
        print('sudah terdaftar')
    else:
        m = re.search(r'''['"]console_scripts['"]\s*:\s*\[''', teks)
        if not m:
            sys.exit('Blok console_scripts tidak ketemu di setup.py; tambahkan tiga baris ini manual:\n  '
                     + '\n  '.join(f"'{e}'," for e in baru))
        sisip = ''.join(f"\n            '{e}'," for e in baru)
        with open(setup, 'w', encoding='utf-8') as fh:
            fh.write(teks[:m.end()] + sisip + teks[m.end():])
        print('ditambahkan:', ', '.join(e.split(' = ')[0] for e in baru))
    if 'glob' not in teks or 'config' not in teks:
        print('PERINGATAN: data_files di setup.py tampaknya tidak memakai glob untuk folder config/;'
              ' pastikan config/pedoman.yaml ikut terpasang.')

    sekarang('Dependensi Python')
    for mod, paket in (('cv2', 'python3-opencv'), ('numpy', 'python3-numpy'), ('yaml', 'python3-yaml')):
        try:
            __import__(mod)
            print(f'{mod}: ada')
        except ImportError:
            print(f'{mod}: TIDAK ADA  -> sudo apt install {paket}')

    sekarang('Tipe perintah di autopilot')
    ap = os.path.join(modul, 'autopilot_gerbang.py')
    if os.path.exists(ap):
        src = open(ap, encoding='utf-8').read()
        if 'TwistStamped' in src:
            print('PERINGATAN: autopilot memakai TwistStamped; node baru memakai Twist. Beri tahu saya.')
        elif 'Twist' in src:
            print('autopilot memakai geometry_msgs/Twist (cocok)')
        else:
            print('tipe Twist tidak terlihat di autopilot_gerbang.py; cek manual.')

    sekarang('Dunia, jembatan, parameter')
    r = subprocess.run([sys.executable, os.path.join(PAKET, 'tools', 'tambah_pedoman.py'),
                        '--paket', PAKET] + sys.argv[1:])
    if r.returncode:
        sys.exit(r.returncode)
    if shutil.which('gz'):
        sekarang('Validasi SDF dengan gz sdf -k')
        subprocess.run(['gz', 'sdf', '-k', os.path.join(PAKET, 'worlds', 'kolam_asv.sdf')])

    print('\nSelesai. Berikutnya:\n  cd ~/ros2_ws && colcon build --symlink-install --packages-select asv_sim'
          '\n  source install/setup.bash\n  ros2 launch asv_sim pedoman.launch.py')


if __name__ == '__main__':
    main()
