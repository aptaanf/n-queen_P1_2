# Panduan Proyek & Aturan Untuk AI / Antigravity (`agy`)

> **Cara pakai untuk anggota tim:** Saat membuka repo `n-queen_p1_2` di Antigravity (`agy`), kirim prompt pertama:
> *"Baca `Solver/berkah/AI_HANDOFF.md` dan `README.md` dulu sebelum mulai mengerjakan tugas."*

---

## 1. Gambaran Proyek & Identitas Tim
- **Mata Kuliah:** KOM1303 Analisis Algoritma (Ganjil 2026/2027) — PBL Akhir (**bobot 40%** dari nilai akhir: Laporan Akhir 50%, Repositori GitHub 30%, PPT & Video 20%).
- **Tim:** **Berkah dunia akhirat** — **Paralel P1, Kelompok 2**
  - Ayubi Fathan (`M0403241050`)
  - Apta Adi Nur Fiansah (`M0403241003`)
  - Syahwali Khan Habibi Harahap (`M0403241128`)
- **Repo Tim:** https://github.com/aptaanf/n-queen_p1_2
- **Repo Upstream (Local App):** https://github.com/ghiffaribraviah/n-queen-analgor-ganjil2627
- **Web Leaderboard:** https://nqueen.ghiffaribraviah.xyz
- **Jadwal Penting:**
  - Kompetisi Leaderboard: **23 Nov – 6 Des 2026** (leaderboard dimatikan **H-3** sebelum penutupan).
  - Batas Pengumpulan Berkas: **13 Des 2026 (23.59 WIB)**.

---

## 2. Aturan Ketat Repositori & Pengumpulan
1. **HANYA boleh mengubah folder `Solver/berkah/` (dan `README.md` bila perlu).**
   - **DILARANG KERAS** mengubah file lain (`Tester/`, `driver/`, `.gitignore`, `package.json`, dll.).
   - Jangan commit file instruksi pribadi (`AGENTS.md`, dll.) di root repo agar struktur repo tetap bersih sesuai aturan upstream.
2. **Batas Submission Leaderboard:** Maksimal **2 submission / case / hari / tim**.
   - **Wajib uji di local app terlebih dahulu** (pastikan 0 invalid, 0 duplikat, jumlah solusi benar, waktu cepat, dan memori ringan) sebelum submit ke web.
3. **Sync Upstream:** Sebelum menguji aturan secara lokal, pastikan sudah `git pull` dari repo upstream (`ghiffaribraviah/n-queen-analgor-ganjil2627`) untuk mendapatkan *automated rule checker* dan *memory probe* terbaru.

---

## 3. Syarat Ketat Kode Program (Wajib — Dicek Otomatis!)
Setiap file solver `.cpp` **wajib** mematuhi 5 syarat berikut (dicek otomatis di web & upstream local app):
1. **Dilarang mendefinisikan sendiri** `operator new`/`delete`, fungsi pustaka C (`malloc`, `free`, `memcpy`, `memset`, dll.), atau bagian dari `std` *(memakainya tetap diperbolehkan)*.
2. **Dilarang menggunakan `#line`.**
3. **Tidak boleh ada variabel global.** Di luar fungsi hanya boleh konstanta (angka, karakter, deretannya), `struct`/`class`, dan fungsi. *(Gunakan variabel lokal di stack di dalam fungsi solver, dan hindari juga variabel lokal `static` mutable.)*
4. **Hanya boleh menggunakan library standar C++ dan `"nqueens.h"`.**
5. **Dilarang pre-computing** penempatan ratu (utuh maupun sebagian) dalam bentuk apa pun — termasuk menyimpan kombinasi ratu sebelum runtime atau mencari kombinasi saat compile time (`constexpr`). Algoritma wajib murni mencari solusi saat runtime.

---

## 4. Spesifikasi 4 Kasus & Status Progres Saat Ini

Board berukuran `N = 4..12`, cutoff **15 detik** per run per test case (Normal: **9 test cases**; Pre-placed, Holes, Walled: masing-masing **27 test cases**). Semua kasus wajib mengembalikan **SEMUA solusi valid tanpa duplikat**.

**Konvensi File Final (`Solver/berkah/`):**
Gunakan satu file final yang cepat, **100% patuh 5 syarat kode**, dan **mudah dijelaskan di Laporan Akhir** untuk setiap kasus:

| Kasus | File Final | Signature Fungsi (`nqueens.h`) | Status | Catatan |
| :--- | :--- | :--- | :--- | :--- |
| **1. Normal** | `Solver/berkah/final-normal.cpp` | `std::vector<std::vector<int>> solveNQueens(int n)` | **Sudah Selesai** | Memakai Backtracking + Bitmask $O(1)$ + Simetri Dihedral $D_4$ (8-fold) + Greedy MRV & Degree Heuristic + Forward Checking + Inline Unit Propagation + Direct Emit (0 global, tanpa `fundBuf`/tabel besar). *(File lama `bitmask-normal.cpp` & `d4-normal.cpp` adalah arsip eksperimen yang masih memakai variabel global — jangan disubmit.)* |
| **2. Pre-placed** | `Solver/berkah/final-preplaced.cpp` | `std::vector<std::vector<int>> solvePreplaced(int n, const std::vector<std::pair<int, int>>& queens)` | **Belum (Prioritas Berikutnya)** | Mulai dengan validasi/pre-fill bitmask ratu terkunci (`queens`), lalu Backtracking + Bitmask + Forward Checking + Greedy MRV untuk baris yang masih kosong. |
| **3. Holes** | `Solver/berkah/final-holes.cpp` | `std::vector<std::vector<int>> solveHoles(int n, const std::vector<std::string>& grid)` | **Belum** | Ratu tidak boleh diletakkan di sel `'H'`, **tetapi serangan ratu tetap menembus `'H'`**. Gunakan `rowAllow[r]` (bitmask kolom non-`'H'`) + Backtracking Bitmask + Forward Checking + Greedy MRV. |
| **4. Walled** | `Solver/berkah/final-walled.cpp` | `std::vector<std::vector<std::pair<int, int>>> solveWalled(int n, const std::vector<std::string>& grid)` | **Belum** | Dinding `'W'` memutus serangan (bisa > 1 ratu per baris/kolom/diagonal jika dipisah `'W'`). Kembalikan **SEMUA papan dengan jumlah ratu maksimal** → gunakan **Branch & Bound** dengan *bounding function* yang ketat + bitmask/segmen. |

---

## 5. Keterkaitan Materi Kuliah KOM1303 (Penting untuk Laporan Akhir!)
Kode yang dibuat **tidak boleh terlalu rumit/aneh** sampai susah dianalisis di laporan akhir. Pastikan setiap solver bisa dipetakan jelas ke materi kuliah:
- **Complete Search & Backtracking (Pertemuan 11):** pola `choose -> explore -> undo`, pengecekan konflik $O(1)$ (bitmask), dan **Soundness of Pruning** (*Forward Checking* / *Unit Propagation*).
- **Greedy (Pertemuan 10):** *selection rule* & *feasibility*, misalnya heuristik **MRV (*Minimum Remaining Values*)** dan **Degree Heuristic** untuk memilih urutan variabel di dalam Backtracking.
- **Branch & Bound (Pertemuan 11):** *upper bound* $B(X)$ yang *sound* untuk masalah optimasi (**Walled N-Queens**).
- **Pemisahan di Laporan:** Pisahkan jelas antara **paradigma algoritma (teori kuliah)** dan **optimasi teknis implementasi C++/WASM** (misalnya menghindari alokasi heap berulang atau buffer stack besar).

### Catatan Optimasi Performa & Memori (WASM)
- **Waktu (*Fastest*):** Diukur dari *best time* dari 8 run berulang (1 warmup + 7 timed) dalam Worker terpisah. Hindari fungsi WASM tunggal raksasa (`> 30–50 KB`) agar compiler JIT V8 (*TurboFan*) cepat naik level.
- **Memori (*Lightest*):** Tester terbaru (`Tester/Execution/memory-probe.ts`) mengukur **seluruh byte yang ditulis di stack + heap + static**, bukan hanya heap `sbrk`. Karena itu:
  - Hindari mengalokasikan buffer array besar di stack (mis. `32 KB`) jika tidak benar-benar perlu.
  - Gunakan tipe data sekecil mungkin (`uint16_t` untuk `N <= 12`, array lokal seperlunya), dan hindari alokasi `std::vector` sementara di dalam rekursi.

---

## 6. Perintah Build & Pengujian Lokal
Jalankan dari root folder repo `n-queen_p1_2`:
- **Build semua solver (Docker default):**
  ```bash
  npm run build:solvers
  ```
- **Build semua solver (Native `emsdk` di Windows):**
  ```cmd
  cmd /c "call C:\emsdk\emsdk_env.bat && npm run build:solvers -- --native"
  ```
- **Jalankan UI Tester lokal:**
  ```bash
  npm run dev
  ```
  Lalu buka URL yang muncul (`http://localhost:5173/`), pilih `berkah / <nama-solver>`, dan uji pada berbagai ukuran `N = 4..12` (untuk Normal, cocokkan jumlah solusi dengan OEIS A000170: `2, 10, 4, 40, 92, 352, 724, 2680, 14200`; untuk Pre-placed/Holes/Walled saat `P=0`/`H=0`/`W=0`, hasilnya harus sama dengan Normal).

---

## 7. Transparansi Penggunaan AI
Setiap kali AI membantu merancang algoritma, melakukan optimasi, atau memperbaiki bug, catat ringkas perannya (tanggal, bagian yang dibantu, dan alasan perubahan) karena penggunaan AI **wajib dilaporkan secara transparan pada Laporan Akhir**.
