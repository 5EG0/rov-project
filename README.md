# ROV Project - Monorepo

Repositori utama untuk pengembangan sistem perangkat lunak **Remotely Operated Vehicle (ROV)** yang mencakup antarmuka pengguna (GUI), *Computer Vision*, dan kontrol *Thruster*.

## 📂 Struktur Direktori

```text
├── gui/                # Kode antarmuka operator (Ground Control Station)
├── computer-vision/    # Kode pemrosesan gambar & deteksi objek bawah air
├── thruster-control/   # Kode kontrol motor, sensor, & mikrokontroler
└── .gitignore          # Daftar file/folder yang diabaikan oleh Git
```

---

## 🛠️ Persiapan Awal (Dilakukan Sekali)

### 1. Clone Repositori
Pastikan SSH Key sudah terdaftar di akun GitHub Anda, lalu jalankan perintah berikut di terminal:
```bash
git clone git@github.com:USERNAME_ANDA/NAMA_REPO.git
cd NAMA_REPO
```

### 2. Pengaturan Virtual Environment (`venv`) untuk Python
Bagi divisi yang menggunakan Python (misalnya **Computer Vision** atau **GUI**), wajib membuat `venv` secara lokal agar versi *library* seragam:

1. Masuk ke folder divisi terkait (contoh: `cd computer-vision`).
2. Buat `venv`:
   * **Windows:** `python -m venv venv`
   * **Linux/macOS:** `python3 -m venv venv`
3. Aktifkan `venv`:
   * **Windows (CMD):** `venv\Scripts\activate`
   * **Windows (PowerShell):** `.\venv\Scripts\Activate.ps1`
   * **Linux/macOS/Git Bash:** `source venv/bin/activate`
4. Instal seluruh *library* yang dibutuhkan (jika file `requirements.txt` sudah tersedia):
   ```bash
   pip install -r requirements.txt
   ```

> **Catatan:** Jangan pernah meng-upload folder `venv/` ke GitHub. Jika Anda menginstal *library* baru, perbarui daftar paket dengan perintah: `pip freeze > requirements.txt`.

---

## 🚀 Panduan Push Kode & Kolaborasi Tim

Branch `main` dilindungi (*protected*). **Dilarang melakukan `git push` langsung ke `main`.** Ikuti langkah berikut setiap kali menambahkan atau mengubah kode:

### Langkah 1: Pastikan Kode Lokal Terbaru
Selalu ambil pembaruan terbaru dari `main` sebelum mulai bekerja:
```bash
git checkout main
git pull origin main
```

### Langkah 2: Buat Branch Baru
Buat *branch* baru sesuai dengan divisi dan fitur yang sedang dikerjakan. Gunakan awalan (`gui/`, `cv/`, atau `thruster/`):
```bash
# Contoh penamaan branch:
git checkout -b cv/deteksi-objek
git checkout -b gui/indikator-kedalaman
git checkout -b thruster/kalibrasi-pwm
```

### Langkah 3: Simpan Perubahan (Commit)
Setelah selesai menulis dan menguji kode di laptop masing-masing:
1. Cek file apa saja yang berubah:
   ```bash
   git status
   ```
2. Tambahkan file ke dalam *staging area* (pastikan tidak ada file sampah/log sementara):
   ```bash
   git add .
   ```
3. Buat pesan *commit* yang jelas dengan format `tipe(divisi): deskripsi singkat`:
   ```bash
   git commit -m "feat(cv): menambahkan filter warna bawah air"
   ```
   *(Gunakan awalan seperti `feat` untuk fitur baru, `fix` untuk perbaikan bug, atau `docs` untuk dokumentasi).*

### Langkah 4: Push ke GitHub
Kirim *branch* Anda ke repositori GitHub:
```bash
git push origin nama-branch-anda
# Contoh: git push origin cv/deteksi-objek
```

### Langkah 5: Buat Pull Request (PR)
1. Buka halaman repositori ini di browser (GitHub).
2. Akan muncul notifikasi kuning bertuliskan **Compare & pull request** pada *branch* yang baru saja Anda *push*. Klik tombol tersebut.
3. Tuliskan ringkasan perubahan yang Anda buat, lalu klik **Create pull request**.
4. Minta anggota tim lain untuk meninjau (*review*) kode Anda sebelum digabungkan (**Merge**) ke branch `main`.
