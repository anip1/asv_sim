"""Mesin status misi ASV (murni Python, tanpa ROS) supaya mudah diuji.

Urutan: MENUNGGU (START) -> GERBANG x10 -> IMAGING_PERMUKAAN
        -> IMAGING_BAWAH_AIR -> DOCKING -> SELESAI (FINISH)
Di tengah gerbang ada dua status bantu: LEWAT (lurus melewati gerbang)
dan CARI (pelampung hilang dari pandangan).
"""
import math
from dataclasses import dataclass
from enum import Enum

# ---- Misi ----
JUMLAH_GERBANG = 10

# ---- Kendali menuju gerbang (dipindah dari autopilot_gerbang.py) ----
V_JELAJAH = 0.5           # m/s
V_MIN = 0.2               # m/s
GAIN_BELOK = 1.5
BELOK_MAKS = 0.8          # rad/s

# ---- Melewati gerbang ----
JARAK_LEWAT = 2.5         # m: pelampung hilang/melonjak di bawah jarak ini = gerbang terlewati
LONJAKAN_LEWAT = 1.0      # m: target mendadak sejauh ini lebih jauh = gerbang terlewati
JARAK_KAMERA_KE_BURITAN = 0.8   # m, SESUAIKAN dengan kapalmu
V_LEWAT = 0.4             # m/s, lurus tanpa melihat saat melewati gerbang
FRAME_MIN_LIHAT = 3       # frame: gerbang harus terlihat sekian frame sebelum boleh dianggap terlewati
BATAS_LEWAT = 3.0         # kali durasi rencana: cadangan bila kapal tidak bergerak maju

# ---- Pelampung hilang ----
TOLERANSI_HILANG = 0.6    # s: hilang sesaat dianggap kedip, lurus saja
LURUS_AWAL = 6.0          # s: belum pernah melihat pelampung, lurus dulu
V_CARI = 0.15             # m/s
PUTAR_CARI = 0.4          # rad/s
T_SAPU = 4.0              # s: ganti arah putar tiap sekian detik
FRAME_DAPAT = 3           # frame beruntun terlihat sebelum dianggap ketemu lagi
PERINGATAN_CARI = 15.0    # s: beri peringatan bila mencari terlalu lama

# ---- Tahap yang belum dibuat ----
DURASI_TAHAP_KOSONG = 2.0  # s: berhenti sebentar lalu lanjut ke tahap berikutnya


@dataclass(frozen=True)
class Perintah:
    maju: float = 0.0     # m/s  -> linear.x
    putar: float = 0.0    # rad/s -> angular.z (+ = belok kiri)


BERHENTI = Perintah(0.0, 0.0)


class Status(Enum):
    MENUNGGU = "MENUNGGU"
    GERBANG = "GERBANG"
    LEWAT = "LEWAT"
    CARI = "CARI"
    IMAGING_PERMUKAAN = "IMAGING_PERMUKAAN"
    IMAGING_BAWAH_AIR = "IMAGING_BAWAH_AIR"
    DOCKING = "DOCKING"
    SELESAI = "SELESAI"


URUTAN_LANJUTAN = [Status.IMAGING_PERMUKAAN, Status.IMAGING_BAWAH_AIR,
                   Status.DOCKING, Status.SELESAI]


def kendali_ke_titik(titik):
    """Sama seperti kendali lama: titik = (maju, kiri) dalam meter."""
    galat = math.atan2(titik[1], max(titik[0], 0.1))       # + = target di kiri
    putar = max(-BELOK_MAKS, min(BELOK_MAKS, GAIN_BELOK * galat))
    faktor = max(0.0, 1.0 - abs(galat) / 0.8)               # makin tajam, makin pelan
    return Perintah(V_MIN + (V_JELAJAH - V_MIN) * faktor, putar)


class Misi:
    def __init__(self, log=print, auto_mulai=True):
        self.log = log
        self.status = Status.MENUNGGU
        self.gerbang_lewat = 0          # jumlah gerbang yang sudah terlewati
        self._boleh_mulai = auto_mulai
        self._t_masuk = 0.0
        self._perintah = Perintah(V_MIN, 0.0)
        self._jarak = None              # jarak ke gerbang pada frame terakhir terlihat
        self._jumlah_lihat = 0          # berapa frame gerbang ini terlihat
        self._sisi = 1.0                # + = target terakhir terlihat di kiri
        self._terakhir_terlihat = 0.0
        self._durasi_lewat = 0.0
        self._jarak_lewat = 0.0         # jarak lurus yang direncanakan saat melewati gerbang
        self._pose = None               # (x, y) kapal bila odometri tersedia
        self._pose_awal_lewat = None
        self._beruntun = 0
        self._arah = 1.0
        self._sudah_peringatan = False

    # ---- antarmuka ----
    def mulai(self):
        """Dipanggil saat sinyal START diterima."""
        self._boleh_mulai = True

    def teks_status(self):
        if self.status in (Status.GERBANG, Status.LEWAT, Status.CARI):
            ke = min(self.gerbang_lewat + 1, JUMLAH_GERBANG)
            return f"{self.status.value} gerbang {ke}/{JUMLAH_GERBANG}"
        return self.status.value

    def langkah(self, t, titik, pose=None):
        """t: waktu (detik). titik: (maju, kiri) dalam meter, atau None bila
        tidak ada pelampung terlihat. pose: (x, y) kapal dalam meter bila ada
        (odometri atau GPS), dipakai untuk mengukur jarak tempuh.
        Mengembalikan Perintah."""
        self._pose = pose
        if self.status == Status.MENUNGGU:
            if not self._boleh_mulai:
                return BERHENTI
            self.log("START: misi dimulai")
            self._masuk(Status.GERBANG, t)
        if self.status == Status.GERBANG:
            return self._gerbang(t, titik)
        if self.status == Status.LEWAT:
            return self._lewat(t)
        if self.status == Status.CARI:
            return self._cari(t, titik)
        if self.status in URUTAN_LANJUTAN[:-1]:
            return self._tahap_kosong(t)
        return BERHENTI                  # SELESAI

    # ---- internal ----
    def _masuk(self, status, t):
        self.status = status
        self._t_masuk = t
        if status == Status.GERBANG:
            self._jarak = None
            self._jumlah_lihat = 0
            self._terakhir_terlihat = t
            self._perintah = Perintah(V_MIN, 0.0)
        elif status == Status.CARI:
            self._beruntun = 0
            self._arah = self._sisi
            self._sudah_peringatan = False
        elif status == Status.SELESAI:
            self.log("FINISH: misi selesai")

    def _gerbang(self, t, titik):
        if titik is not None:
            maju, kiri = titik
            cukup = self._jumlah_lihat >= FRAME_MIN_LIHAT
            lonjakan = (cukup and self._jarak is not None and self._jarak <= JARAK_LEWAT
                        and maju >= self._jarak + LONJAKAN_LEWAT)
            if lonjakan:                 # gerbang di depan sudah lewat, yang terlihat gerbang berikutnya
                return self._mulai_lewat(t, f"target melonjak ke {maju:.2f} m")
            self._terakhir_terlihat = t
            self._jumlah_lihat += 1
            self._jarak = maju
            self._sisi = 1.0 if kiri >= 0 else -1.0
            self._perintah = kendali_ke_titik(titik)
            return self._perintah

        hilang = t - self._terakhir_terlihat
        if hilang < TOLERANSI_HILANG:    # kedip sesaat: lurus dengan kecepatan terakhir
            return Perintah(self._perintah.maju, 0.0)
        if self._jarak is not None and self._jumlah_lihat < FRAME_MIN_LIHAT:
            self._jarak = None           # penampakan sesaat saja: anggap noise
        if self._jarak is None:          # belum pernah melihat pelampung
            if hilang < LURUS_AWAL:
                return Perintah(V_MIN, 0.0)
            return self._mulai_cari(t)
        if self._jarak <= JARAK_LEWAT:   # hilang saat sudah dekat = lewat gerbang
            return self._mulai_lewat(t, "pelampung hilang")
        return self._mulai_cari(t)       # hilang saat masih jauh = cari lagi

    def _mulai_lewat(self, t, alasan):
        self._jarak_lewat = self._jarak + JARAK_KAMERA_KE_BURITAN
        self._durasi_lewat = self._jarak_lewat / V_LEWAT
        self._pose_awal_lewat = self._pose
        self.log(f"Gerbang {self.gerbang_lewat + 1}/{JUMLAH_GERBANG}: terlewati "
                 f"({alasan}; jarak terakhir {self._jarak:.2f} m)")
        self._masuk(Status.LEWAT, t)
        return Perintah(V_LEWAT, 0.0)

    def _lewat(self, t):
        lama = t - self._t_masuk
        tempuh = None
        if self._pose is not None and self._pose_awal_lewat is not None:
            # dengan odometri: berhenti setelah jarak tempuh cukup, berapa pun kecepatan kapal
            tempuh = math.hypot(self._pose[0] - self._pose_awal_lewat[0],
                                self._pose[1] - self._pose_awal_lewat[1])
            selesai = tempuh >= self._jarak_lewat or lama >= BATAS_LEWAT * self._durasi_lewat
        else:
            selesai = lama >= self._durasi_lewat
        if not selesai:
            return Perintah(V_LEWAT, 0.0)
        self.gerbang_lewat += 1
        if tempuh is None:
            self.log(f"Gerbang {self.gerbang_lewat}/{JUMLAH_GERBANG} selesai")
        else:
            self.log(f"Gerbang {self.gerbang_lewat}/{JUMLAH_GERBANG} selesai "
                     f"(tempuh {tempuh:.1f} m dari rencana {self._jarak_lewat:.1f} m, {lama:.1f} s)")
        if self.gerbang_lewat >= JUMLAH_GERBANG:
            self._masuk(URUTAN_LANJUTAN[0], t)
            self.log(f"Semua gerbang selesai, lanjut ke {self.status.value}")
            return BERHENTI
        self._masuk(Status.GERBANG, t)
        return Perintah(V_MIN, 0.0)

    def _mulai_cari(self, t):
        jarak = "belum pernah terlihat" if self._jarak is None else f"{self._jarak:.2f} m"
        self.log(f"Pelampung hilang (jarak terakhir: {jarak}), mulai mencari")
        self._masuk(Status.CARI, t)
        return self._cari(t, None)

    def _cari(self, t, titik):
        if titik is not None:
            self._beruntun += 1
            if self._beruntun >= FRAME_DAPAT:
                self.log("Pelampung ketemu lagi")
                self._masuk(Status.GERBANG, t)
                return self._gerbang(t, titik)
        else:
            self._beruntun = 0
        lama = t - self._t_masuk
        if lama > PERINGATAN_CARI and not self._sudah_peringatan:
            self._sudah_peringatan = True
            self.log(f"Sudah mencari {lama:.0f} detik tanpa hasil")
        arah = self._arah if int(lama // T_SAPU) % 2 == 0 else -self._arah
        return Perintah(V_CARI, arah * PUTAR_CARI)

    def _tahap_kosong(self, t):
        """Tahap yang sensornya belum dibuat: berhenti sebentar lalu lewati."""
        if t - self._t_masuk >= DURASI_TAHAP_KOSONG:
            berikut = URUTAN_LANJUTAN[URUTAN_LANJUTAN.index(self.status) + 1]
            self.log(f"{self.status.value} belum diimplementasi, dilewati -> {berikut.value}")
            self._masuk(berikut, t)
        return BERHENTI