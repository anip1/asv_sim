"""Pencocokan pelampung menjadi gerbang (murni Python, tanpa ROS dan OpenCV).

Masalah yang diselesaikan: kalau hanya blob terbesar tiap warna yang dipakai,
pelampung merah gerbang ini bisa berpasangan dengan pelampung hijau gerbang
berikutnya. Di sini semua pelampung dipasangkan berdasarkan lebar gerbang.
"""
import math
from dataclasses import dataclass
from typing import Optional, Tuple

# ---- Samakan dengan SDF dan pedoman ----
DIAMETER_BOLA = 0.24      # m
LEBAR_GERBANG = 2.0       # m

# ---- Syarat pasangan merah-hijau dianggap satu gerbang ----
BEDA_DALAM_DASAR = 1.2    # m: selisih jarak maju maksimum (gerbang miring)
BEDA_DALAM_PER_M = 0.12   # tambahan per meter jarak (ukuran blob makin kasar di jauh)
GALAT_LEBAR_DASAR = 0.4   # m: selisih lebar terukur vs LEBAR_GERBANG yang masih diterima
GALAT_LEBAR_PER_M = 0.10  # tambahan per meter jarak


@dataclass(frozen=True)
class Bola:
    maju: float                 # m, di depan kapal
    kiri: float                 # m, + = di kiri kapal
    kotak: Optional[tuple] = None   # (x, y, w, h) piksel, untuk gambar debug


@dataclass(frozen=True)
class Gerbang:
    titik: Tuple[float, float]  # (maju, kiri) titik bidik di tengah gerbang
    merah: Optional[Bola] = None
    hijau: Optional[Bola] = None

    @property
    def lengkap(self):
        return self.merah is not None and self.hijau is not None


def kotak_ke_bola(kotak, lebar_gambar, fx, diameter=DIAMETER_BOLA):
    """Piksel -> Bola (maju, kiri) dalam meter, memakai ukuran bola yang diketahui."""
    x, _, w, _ = kotak
    maju = fx * diameter / max(w, 1)
    kanan = (x + w / 2.0 - lebar_gambar / 2.0) * maju / fx
    return Bola(maju, -kanan, kotak)


def cari_gerbang(merah, hijau, lebar=LEBAR_GERBANG):
    """Pasangkan pelampung merah dan hijau menjadi gerbang.

    merah, hijau: daftar Bola. Mengembalikan daftar Gerbang terurut dari yang
    terdekat. Pelampung yang tidak punya pasangan menjadi gerbang tunggal dengan
    titik bidik digeser setengah lebar gerbang (merah = sisi kiri gerbang).
    """
    kandidat = []
    for i, r in enumerate(merah):
        for j, h in enumerate(hijau):
            if r.kiri <= h.kiri:                 # merah harus di sebelah kiri hijau
                continue
            terdekat = min(r.maju, h.maju)
            beda_dalam = abs(r.maju - h.maju)
            lebar_ukur = math.hypot(beda_dalam, r.kiri - h.kiri)
            galat = abs(lebar_ukur - lebar)
            if beda_dalam > BEDA_DALAM_DASAR + BEDA_DALAM_PER_M * terdekat:
                continue
            if galat > GALAT_LEBAR_DASAR + GALAT_LEBAR_PER_M * terdekat:
                continue
            kandidat.append((galat, i, j))
    kandidat.sort()

    pakai_m, pakai_h = set(), set()
    hasil = []
    for _, i, j in kandidat:
        if i in pakai_m or j in pakai_h:
            continue
        pakai_m.add(i)
        pakai_h.add(j)
        r, h = merah[i], hijau[j]
        hasil.append(Gerbang(((r.maju + h.maju) / 2.0, (r.kiri + h.kiri) / 2.0), r, h))
    for i, r in enumerate(merah):
        if i not in pakai_m:
            hasil.append(Gerbang((r.maju, r.kiri - lebar / 2.0), merah=r))
    for j, h in enumerate(hijau):
        if j not in pakai_h:
            hasil.append(Gerbang((h.maju, h.kiri + lebar / 2.0), hijau=h))
    hasil.sort(key=lambda g: g.titik[0])
    return hasil


def titik_sederhana(merah, hijau, lebar=LEBAR_GERBANG):
    """Cara lama: pelampung merah dan hijau terdekat dianggap satu pasangan."""
    m = min(merah, key=lambda b: b.maju) if merah else None
    h = min(hijau, key=lambda b: b.maju) if hijau else None
    if m and h:
        return ((m.maju + h.maju) / 2.0, (m.kiri + h.kiri) / 2.0)
    if m:
        return (m.maju, m.kiri - lebar / 2.0)
    if h:
        return (h.maju, h.kiri + lebar / 2.0)
    return None