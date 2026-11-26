#include <algorithm>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "nqueens.h"

namespace {

// Solver Walled N-Queens (Final - Tim Berkah dunia akhirat / P1-2)
//
// Paradigma Algoritma (KOM1303 Analisis Algoritma):
// 1. Dekomposisi Komponen Terhubung (Graf Konflik Segmen):
//    - Dinding 'W' membagi setiap baris, kolom, diagonal utama (r-c), dan anti-diagonal (r+c)
//      menjadi segmen-segmen maksimal sel bebas '.'.
//    - Dua sel bebas saling menyerang jika dan hanya jika berbagi minimal satu segmen.
//    - Sel terisolasi (derajat 0) pasti masuk ke setiap solusi optimal; komponen terhubung
//      lain diselesaikan secara independen lalu digabung via produk Kartesius.
// 2. Branch & Bound (Pertemuan 11):
//    - Batas bawah awal (bestCount) dicari dengan heuristik greedy min-degree sebelum B&B
//      (tanpa menyimpan solusi heuristik agar tidak terjadi duplikasi/kehilangan solusi).
//    - Fungsi batas atas B(X) yang sound (mengambil minimum dari 3 batas):
//        (a) 4-Dimension Active Segment Bound: min(#rowSeg, #colSeg, #d1Seg, #d2Seg) aktif.
//        (b) Extended Segment Clique Cover: partisi klik dari segmen terpanjang + perluasan klik.
//        (c) Bipartite Matching Bound: maximum matching (rowSeg <-> colSeg) dan (d1Seg <-> d2Seg).
//    - Pemangkasan aman (sound pruning): pangkas hanya jika placed + UB < bestCount (strict).
// 3. Complete Search, Backtracking & Greedy MRV (Pertemuan 10 & 11):
//    - Reduksi sel terisolasi (derajat 0) dan Zero-Slack Unit Propagation (segmen berukuran 1
//      pada dimensi dengan slack == 0) langsung ditempatkan tanpa percabangan.
//    - Memilih segmen pada dimensi dengan jumlah segmen aktif paling sedikit (slack terkecil),
//      lalu memilih segmen dengan kandidat paling sedikit (MRV) dan derajat konflik tertinggi.
//    - Urutan kandidat diurutkan berdasarkan derajat terkecil agar solusi optimal cepat ditemukan.
//
// Optimasi Memori & WASM:
// - 0 variabel global, 0 static mutable, 0 pre-computing, hanya stdlib C++ dan "nqueens.h".
// - Representasi bitset kompak 64-bit (1..4 word tergantung ukuran komponen), stack sangat kecil,
//   0 alokasi std::vector di dalam rekursi pencarian utama.

template <int W>
struct Bits {
  uint64_t w[W];

  inline void clear() {
    for (int i = 0; i < W; ++i) w[i] = 0;
  }
  inline bool any() const {
    uint64_t acc = 0;
    for (int i = 0; i < W; ++i) acc |= w[i];
    return acc != 0;
  }
  inline bool test(int i) const { return ((w[i >> 6] >> (i & 63)) & 1ULL) != 0; }
  inline void set(int i) { w[i >> 6] |= (1ULL << (i & 63)); }
  inline void reset(int i) { w[i >> 6] &= ~(1ULL << (i & 63)); }
  inline int count() const {
    int s = 0;
    for (int i = 0; i < W; ++i) s += __builtin_popcountll(w[i]);
    return s;
  }
  inline int ctz() const {
    for (int i = 0; i < W; ++i) {
      if (w[i]) return (i << 6) + __builtin_ctzll(w[i]);
    }
    return -1;
  }
  inline bool intersects(const Bits& o) const {
    for (int i = 0; i < W; ++i)
      if (w[i] & o.w[i]) return true;
    return false;
  }
  inline int andCount(const Bits& o) const {
    int s = 0;
    for (int i = 0; i < W; ++i) s += __builtin_popcountll(w[i] & o.w[i]);
    return s;
  }
  inline Bits operator&(const Bits& o) const {
    Bits r;
    for (int i = 0; i < W; ++i) r.w[i] = w[i] & o.w[i];
    return r;
  }
  inline Bits andNot(const Bits& o) const {
    Bits r;
    for (int i = 0; i < W; ++i) r.w[i] = w[i] & ~o.w[i];
    return r;
  }
  inline void orEq(const Bits& o) {
    for (int i = 0; i < W; ++i) w[i] |= o.w[i];
  }
  inline void andEq(const Bits& o) {
    for (int i = 0; i < W; ++i) w[i] &= o.w[i];
  }
  inline void andNotEq(const Bits& o) {
    for (int i = 0; i < W; ++i) w[i] &= ~o.w[i];
  }
};

struct CellInfo {
  uint8_t r;
  uint8_t c;
  uint16_t seg[4];  // 0: row, 1: col, 2: d1 (r-c), 3: d2 (r+c)
};

struct CompResult {
  int optCount = 0;
  std::vector<uint8_t> flatBoards;  // setiap papan berukuran optCount (ID sel global)
};

template <int W>
class ComponentSolver {
 public:
  static constexpr int kMaxCells = W * 64;
  // Jumlah segmen per dimensi di dalam komponen berukuran <= kMaxCells tidak melebihi kMaxCells.
  static constexpr int kSegWords = (W <= 3) ? W : 3;

  int m = 0;
  const uint8_t* globalIds = nullptr;
  Bits<W> adj[kMaxCells];
  uint16_t cellSeg[kMaxCells][4];

  int segCount[4] = {0, 0, 0, 0};
  int segBase[4] = {0, 0, 0, 0};
  int totalSegs = 0;
  std::vector<Bits<W>> segMasks;

  int bestCount = 0;
  std::vector<uint8_t> flatBoards;
  uint8_t curBoard[kMaxCells];

  void init(int compSize, const int* compCells, const std::vector<CellInfo>& globalInfo) {
    m = compSize;
    for (int i = 0; i < m; ++i) adj[i].clear();

    int maxGlobalSeg = 0;
    for (int i = 0; i < m; ++i) {
      const CellInfo& ci = globalInfo[compCells[i]];
      for (int d = 0; d < 4; ++d) {
        if (static_cast<int>(ci.seg[d]) + 1 > maxGlobalSeg) {
          maxGlobalSeg = static_cast<int>(ci.seg[d]) + 1;
        }
      }
    }

    std::vector<int> segMap(static_cast<size_t>(maxGlobalSeg), -1);
    totalSegs = 0;
    for (int d = 0; d < 4; ++d) {
      segBase[d] = totalSegs;
      std::fill(segMap.begin(), segMap.end(), -1);
      int localCnt = 0;
      for (int i = 0; i < m; ++i) {
        const int gSeg = globalInfo[compCells[i]].seg[d];
        if (segMap[gSeg] < 0) segMap[gSeg] = localCnt++;
        cellSeg[i][d] = static_cast<uint16_t>(segMap[gSeg]);
      }
      segCount[d] = localCnt;
      totalSegs += localCnt;
    }

    segMasks.assign(static_cast<size_t>(totalSegs), Bits<W>{});
    for (int s = 0; s < totalSegs; ++s) segMasks[s].clear();
    for (int i = 0; i < m; ++i) {
      for (int d = 0; d < 4; ++d) {
        segMasks[static_cast<size_t>(segBase[d] + cellSeg[i][d])].set(i);
      }
    }

    for (int i = 0; i < m; ++i) {
      Bits<W> mask;
      mask.clear();
      for (int d = 0; d < 4; ++d) {
        mask.orEq(segMasks[static_cast<size_t>(segBase[d] + cellSeg[i][d])]);
      }
      mask.reset(i);
      adj[i] = mask;
    }
  }

  // Bangun batas bawah awal via greedy min-degree (tanpa menyimpan ke flatBoards).
  int greedyLowerBound(Bits<W> avail) const {
    int best = 0;
    for (int mode = 0; mode < 2; ++mode) {
      Bits<W> rem = avail;
      int cnt = 0;
      while (rem.any()) {
        int pick = -1;
        int bestScore = 1000000;
        Bits<W> it = rem;
        while (it.any()) {
          const int u = it.ctz();
          it.reset(u);
          const int deg = rem.andCount(adj[u]);
          const int score = (mode == 0) ? deg : (deg * 256 - u);
          if (score < bestScore) {
            bestScore = score;
            pick = u;
            if (deg == 0) break;
          }
        }
        ++cnt;
        rem.andNotEq(adj[pick]);
        rem.reset(pick);
      }
      if (cnt > best) best = cnt;
    }
    return best;
  }

  // Batas atas (b): Extended Segment Clique Cover pada sel tersisa.
  int cliqueCoverBound(Bits<W> rem) const {
    int cliques = 0;
    while (rem.any()) {
      ++cliques;
      const int u = rem.ctz();
      Bits<W> bestSeg;
      int bestSize = 0;
      for (int d = 0; d < 4; ++d) {
        const Bits<W> hit = rem & segMasks[static_cast<size_t>(segBase[d] + cellSeg[u][d])];
        const int sz = hit.count();
        if (sz > bestSize) {
          bestSize = sz;
          bestSeg = hit;
        }
      }
      Bits<W> common = rem.andNot(bestSeg);
      Bits<W> memb = bestSeg;
      while (memb.any() && common.any()) {
        const int v = memb.ctz();
        memb.reset(v);
        common.andEq(adj[v]);
      }
      while (common.any()) {
        const int w = common.ctz();
        common.reset(w);
        bestSeg.set(w);
        common.andEq(adj[w]);
      }
      rem.andNotEq(bestSeg);
    }
    return cliques;
  }

  // Batas atas (c): Maximum Bipartite Matching antara dimensi dL dan dR.
  bool bipartiteCanReach(const Bits<W>& avail, int dL, int dR, int need) const {
    const int nL = segCount[dL];
    const int nR = segCount[dR];
    if (need <= 0) return true;
    if (nL < need || nR < need) return false;

    Bits<kSegWords> lAdj[kMaxCells];
    int16_t lToIdx[kMaxCells];
    int16_t matchR[kMaxCells];
    uint8_t unmatchedL[kMaxCells];

    int activeCount = 0;
    const int baseL = segBase[dL];
    for (int l = 0; l < nL; ++l) {
      if (avail.intersects(segMasks[static_cast<size_t>(baseL + l)])) {
        lToIdx[l] = static_cast<int16_t>(activeCount);
        lAdj[activeCount].clear();
        ++activeCount;
      } else {
        lToIdx[l] = -1;
      }
    }
    if (activeCount < need) return false;

    Bits<W> it = avail;
    while (it.any()) {
      const int u = it.ctz();
      it.reset(u);
      const int idx = lToIdx[cellSeg[u][dL]];
      lAdj[idx].set(cellSeg[u][dR]);
    }

    for (int r = 0; r < nR; ++r) matchR[r] = -1;

    int matched = 0;
    for (int i = 0; i < activeCount; ++i) {
      unmatchedL[i] = 1;
      for (int w = 0; w < kSegWords; ++w) {
        uint64_t bits = lAdj[i].w[w];
        while (bits) {
          const int b = __builtin_ctzll(bits);
          const int r = (w << 6) + b;
          if (matchR[r] < 0) {
            matchR[r] = static_cast<int16_t>(i);
            unmatchedL[i] = 0;
            ++matched;
            w = kSegWords;
            break;
          }
          bits &= bits - 1ULL;
        }
      }
    }
    if (matched >= need) return true;

    int remLeft = activeCount - matched;
    if (matched + remLeft < need) return false;

    auto dfs = [&](auto& self, int u, Bits<kSegWords>& vis) -> bool {
      for (int w = 0; w < kSegWords; ++w) {
        uint64_t availR = lAdj[u].w[w] & ~vis.w[w];
        while (availR) {
          const int b = __builtin_ctzll(availR);
          availR &= availR - 1ULL;
          vis.w[w] |= (1ULL << b);
          const int r = (w << 6) + b;
          if (matchR[r] < 0 || self(self, matchR[r], vis)) {
            matchR[r] = static_cast<int16_t>(u);
            return true;
          }
        }
      }
      return false;
    };

    for (int i = 0; i < activeCount; ++i) {
      if (!unmatchedL[i]) continue;
      Bits<kSegWords> vis;
      vis.clear();
      if (dfs(dfs, i, vis)) {
        ++matched;
        if (matched >= need) return true;
      }
      --remLeft;
      if (matched + remLeft < need) return false;
    }
    return matched >= need;
  }

  inline void recordBoard(int placed) {
    if (placed > bestCount) {
      bestCount = placed;
      flatBoards.clear();
    }
    if (placed == bestCount) {
      for (int i = 0; i < placed; ++i) {
        flatBoards.push_back(globalIds[curBoard[i]]);
      }
    }
  }

  // Evaluasi cepat batas atas tanpa memilih segmen cabang (dipakai untuk cabang skip).
  bool quickUpperOk(const Bits<W>& avail, int need) const {
    if (need <= 0) return true;
    int act[4];
    for (int d = 0; d < 4; ++d) {
      const int cnt = segCount[d];
      const int base = segBase[d];
      int a = 0;
      for (int s = 0; s < cnt; ++s) {
        if (avail.intersects(segMasks[static_cast<size_t>(base + s)])) ++a;
      }
      if (a < need) return false;
      act[d] = a;
    }
    if (need >= 2 && cliqueCoverBound(avail) < need) return false;
    if (need >= 3) {
      if (act[0] <= need + 2 && act[1] <= need + 2 && !bipartiteCanReach(avail, 0, 1, need)) {
        return false;
      }
      if (act[2] <= need + 2 && act[3] <= need + 2 && !bipartiteCanReach(avail, 2, 3, need)) {
        return false;
      }
    }
    return true;
  }

  void search(Bits<W> avail, int placed) {
    while (true) {
      // 1. Reduksi sel terisolasi (derajat 0 pada avail): wajib dipilih tanpa percabangan.
      Bits<W> it = avail;
      while (it.any()) {
        const int u = it.ctz();
        it.reset(u);
        if (!avail.intersects(adj[u])) {
          curBoard[placed++] = static_cast<uint8_t>(u);
          avail.reset(u);
        }
      }

      if (!avail.any()) {
        recordBoard(placed);
        return;
      }

      const int need = bestCount - placed;

      // 2. Hitung batas segmen 4 arah & cari dimensi paling ketat + segmen MRV.
      int minDimCnt = 1000000;
      int bestDim = 0;
      int act[4] = {0, 0, 0, 0};
      int minSegSize[4] = {1000, 1000, 1000, 1000};
      Bits<W> bestHitPerDim[4];
      bool zeroSlackUnit = false;
      int unitCell = -1;

      for (int d = 0; d < 4; ++d) {
        const int cnt = segCount[d];
        const int base = segBase[d];
        int a = 0;
        int bestSz = 1000;
        Bits<W> bestHit;
        for (int s = 0; s < cnt; ++s) {
          const Bits<W> hit = avail & segMasks[static_cast<size_t>(base + s)];
          const int sz = hit.count();
          if (sz == 0) continue;
          ++a;
          if (sz < bestSz) {
            bestSz = sz;
            bestHit = hit;
          }
        }
        if (a < need) return;  // Bound 1: pangkas
        act[d] = a;
        minSegSize[d] = bestSz;
        bestHitPerDim[d] = bestHit;
        if (a == need && bestSz == 1) {
          zeroSlackUnit = true;
          unitCell = bestHit.ctz();
          break;
        }
        if (a < minDimCnt || (a == minDimCnt && bestSz < minSegSize[bestDim])) {
          minDimCnt = a;
          bestDim = d;
        }
      }

      // Zero-Slack Unit Propagation: jika dimensi tanpa slack memiliki segmen berukuran 1,
      // sel tersebut wajib ditempatkan langsung tanpa percabangan maupun pengecekan mahal.
      if (zeroSlackUnit) {
        curBoard[placed++] = static_cast<uint8_t>(unitCell);
        avail.andNotEq(adj[unitCell]);
        avail.reset(unitCell);
        continue;
      }

      // 3. Cek batas atas lanjutan hanya bila belum dipangkas oleh segmen.
      if (need >= 2 && cliqueCoverBound(avail) < need) return;
      if (need >= 3) {
        if (act[0] <= need + 2 && act[1] <= need + 2 && !bipartiteCanReach(avail, 0, 1, need)) {
          return;
        }
        if (act[2] <= need + 2 && act[3] <= need + 2 && !bipartiteCanReach(avail, 2, 3, need)) {
          return;
        }
      }

      // 4. Tie-break MRV di dalam bestDim menggunakan Degree Heuristic bila ada beberapa segmen
      //    dengan ukuran minimum yang sama.
      const int targetSz = minSegSize[bestDim];
      Bits<W> bestHit = bestHitPerDim[bestDim];
      if (targetSz >= 1) {
        const int cnt = segCount[bestDim];
        const int base = segBase[bestDim];
        int bestDegSum = -1;
        for (int s = 0; s < cnt; ++s) {
          const Bits<W> hit = avail & segMasks[static_cast<size_t>(base + s)];
          if (hit.count() != targetSz) continue;
          int degSum = 0;
          Bits<W> hIt = hit;
          while (hIt.any()) {
            const int u = hIt.ctz();
            hIt.reset(u);
            degSum += avail.andCount(adj[u]);
          }
          if (degSum > bestDegSum) {
            bestDegSum = degSum;
            bestHit = hit;
          }
        }
      }

      // 5. Urutkan kandidat pada segmen terpilih berdasarkan derajat konflik terkecil.
      uint8_t cand[16];
      uint8_t candDeg[16];
      int k = 0;
      Bits<W> hitIt = bestHit;
      while (hitIt.any()) {
        const int u = hitIt.ctz();
        hitIt.reset(u);
        cand[k] = static_cast<uint8_t>(u);
        candDeg[k] = static_cast<uint8_t>(avail.andCount(adj[u]));
        ++k;
      }
      for (int i = 1; i < k; ++i) {
        const uint8_t cu = cand[i];
        const uint8_t cd = candDeg[i];
        int j = i - 1;
        while (j >= 0 && candDeg[j] > cd) {
          cand[j + 1] = cand[j];
          candDeg[j + 1] = candDeg[j];
          --j;
        }
        cand[j + 1] = cu;
        candDeg[j + 1] = cd;
      }

      // Cabang 1..k: tempatkan ratu pada salah satu sel di segmen terpilih.
      for (int i = 0; i < k; ++i) {
        const int u = cand[i];
        curBoard[placed] = static_cast<uint8_t>(u);
        Bits<W> nextAvail = avail.andNot(adj[u]);
        nextAvail.reset(u);
        search(nextAvail, placed + 1);
      }

      // Cabang skip: hanya dieksplorasi jika tanpa segmen ini masih bisa mencapai bestCount.
      if (act[bestDim] - 1 >= bestCount - placed) {
        const Bits<W> skipAvail = avail.andNot(bestHit);
        if (quickUpperOk(skipAvail, bestCount - placed)) {
          search(skipAvail, placed);
        }
      }
      return;
    }
  }

  CompResult solve(int compSize, const int* compCells, const uint8_t* gIds,
                   const std::vector<CellInfo>& globalInfo) {
    globalIds = gIds;
    init(compSize, compCells, globalInfo);

    Bits<W> allAvail;
    allAvail.clear();
    for (int i = 0; i < m; ++i) allAvail.set(i);

    bestCount = greedyLowerBound(allAvail);
    flatBoards.clear();
    search(allAvail, 0);

    CompResult res;
    res.optCount = bestCount;
    res.flatBoards = std::move(flatBoards);
    return res;
  }
};

CompResult solveComponentDispatch(int compSize, const int* compCells, const uint8_t* gIds,
                                  const std::vector<CellInfo>& globalInfo) {
  if (compSize <= 64) {
    ComponentSolver<1> s;
    return s.solve(compSize, compCells, gIds, globalInfo);
  }
  if (compSize <= 128) {
    ComponentSolver<2> s;
    return s.solve(compSize, compCells, gIds, globalInfo);
  }
  if (compSize <= 192) {
    ComponentSolver<3> s;
    return s.solve(compSize, compCells, gIds, globalInfo);
  }
  ComponentSolver<4> s;
  return s.solve(compSize, compCells, gIds, globalInfo);
}

}  // namespace

std::vector<std::vector<std::pair<int, int>>> solveWalled(
    int n, const std::vector<std::string>& grid) {
  std::vector<std::vector<std::pair<int, int>>> solutions;
  if (n <= 0 || n > 16 || static_cast<int>(grid.size()) != n) return solutions;

  // 1. Kumpulkan seluruh sel bebas '.'
  std::vector<int> cellId(static_cast<size_t>(n * n), -1);
  std::vector<CellInfo> cells;
  cells.reserve(static_cast<size_t>(n * n));
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c) {
      if (grid[r][c] != 'W') {
        cellId[static_cast<size_t>(r * n + c)] = static_cast<int>(cells.size());
        CellInfo info{};
        info.r = static_cast<uint8_t>(r);
        info.c = static_cast<uint8_t>(c);
        cells.push_back(info);
      }
    }
  }

  const int numFree = static_cast<int>(cells.size());
  if (numFree == 0) {
    solutions.emplace_back();  // 1 papan kosong dengan 0 ratu
    return solutions;
  }

  // 2. Bangun segmen maksimal pada 4 arah (Row, Col, D1, D2).
  {
    int seg = 0;
    for (int r = 0; r < n; ++r) {
      bool inSeg = false;
      for (int c = 0; c < n; ++c) {
        const int id = cellId[static_cast<size_t>(r * n + c)];
        if (id >= 0) {
          if (!inSeg) {
            inSeg = true;
            ++seg;
          }
          cells[static_cast<size_t>(id)].seg[0] = static_cast<uint16_t>(seg - 1);
        } else {
          inSeg = false;
        }
      }
    }
    seg = 0;
    for (int c = 0; c < n; ++c) {
      bool inSeg = false;
      for (int r = 0; r < n; ++r) {
        const int id = cellId[static_cast<size_t>(r * n + c)];
        if (id >= 0) {
          if (!inSeg) {
            inSeg = true;
            ++seg;
          }
          cells[static_cast<size_t>(id)].seg[1] = static_cast<uint16_t>(seg - 1);
        } else {
          inSeg = false;
        }
      }
    }
    seg = 0;
    for (int diff = -(n - 1); diff <= n - 1; ++diff) {
      bool inSeg = false;
      const int rStart = (diff > 0) ? diff : 0;
      const int rEnd = (diff < 0) ? (n + diff) : n;
      for (int r = rStart; r < rEnd; ++r) {
        const int c = r - diff;
        const int id = cellId[static_cast<size_t>(r * n + c)];
        if (id >= 0) {
          if (!inSeg) {
            inSeg = true;
            ++seg;
          }
          cells[static_cast<size_t>(id)].seg[2] = static_cast<uint16_t>(seg - 1);
        } else {
          inSeg = false;
        }
      }
    }
    seg = 0;
    for (int sum = 0; sum <= 2 * (n - 1); ++sum) {
      bool inSeg = false;
      const int rStart = (sum >= n) ? (sum - n + 1) : 0;
      const int rEnd = (sum < n) ? (sum + 1) : n;
      for (int r = rStart; r < rEnd; ++r) {
        const int c = sum - r;
        const int id = cellId[static_cast<size_t>(r * n + c)];
        if (id >= 0) {
          if (!inSeg) {
            inSeg = true;
            ++seg;
          }
          cells[static_cast<size_t>(id)].seg[3] = static_cast<uint16_t>(seg - 1);
        } else {
          inSeg = false;
        }
      }
    }
  }

  // 3. Bangun daftar ketetanggaan graf konflik & dekomposisi komponen terhubung.
  std::vector<std::vector<int>> adjList(static_cast<size_t>(numFree));
  for (int i = 0; i < numFree; ++i) {
    for (int j = i + 1; j < numFree; ++j) {
      if (cells[i].seg[0] == cells[j].seg[0] ||
          cells[i].seg[1] == cells[j].seg[1] ||
          cells[i].seg[2] == cells[j].seg[2] ||
          cells[i].seg[3] == cells[j].seg[3]) {
        adjList[i].push_back(j);
        adjList[j].push_back(i);
      }
    }
  }

  std::vector<uint8_t> visited(static_cast<size_t>(numFree), 0);
  std::vector<std::pair<int, int>> forcedQueens;
  std::vector<CompResult> compResults;

  std::vector<int> bfsQueue;
  std::vector<uint8_t> compGlobalIds;
  for (int i = 0; i < numFree; ++i) {
    if (visited[i]) continue;
    visited[i] = 1;
    if (adjList[i].empty()) {
      forcedQueens.emplace_back(cells[i].r, cells[i].c);
      continue;
    }

    bfsQueue.clear();
    bfsQueue.push_back(i);
    for (size_t head = 0; head < bfsQueue.size(); ++head) {
      const int u = bfsQueue[head];
      for (int v : adjList[u]) {
        if (!visited[v]) {
          visited[v] = 1;
          bfsQueue.push_back(v);
        }
      }
    }

    const int compSize = static_cast<int>(bfsQueue.size());
    compGlobalIds.resize(static_cast<size_t>(compSize));
    for (int k = 0; k < compSize; ++k) {
      compGlobalIds[k] = static_cast<uint8_t>(bfsQueue[k]);
    }
    compResults.push_back(
        solveComponentDispatch(compSize, bfsQueue.data(), compGlobalIds.data(), cells));
  }

  // 4. Gabungkan hasil semua komponen via produk Kartesius.
  if (compResults.empty()) {
    solutions.push_back(std::move(forcedQueens));
    return solutions;
  }

  size_t totalBoards = 1;
  int totalQueens = static_cast<int>(forcedQueens.size());
  for (const auto& cr : compResults) {
    const size_t cnt = cr.flatBoards.size() / static_cast<size_t>(cr.optCount);
    totalBoards *= cnt;
    totalQueens += cr.optCount;
  }

  solutions.reserve(totalBoards);
  std::vector<std::pair<int, int>> current = forcedQueens;
  current.reserve(static_cast<size_t>(totalQueens));

  auto emitCartesian = [&](auto& self, size_t idx) -> void {
    if (idx == compResults.size()) {
      solutions.push_back(current);
      return;
    }
    const CompResult& cr = compResults[idx];
    const int k = cr.optCount;
    const size_t cnt = cr.flatBoards.size() / static_cast<size_t>(k);
    const size_t baseLen = current.size();
    for (size_t b = 0; b < cnt; ++b) {
      const uint8_t* ptr = cr.flatBoards.data() + b * static_cast<size_t>(k);
      for (int j = 0; j < k; ++j) {
        const CellInfo& ci = cells[ptr[j]];
        current.emplace_back(ci.r, ci.c);
      }
      self(self, idx + 1);
      current.resize(baseLen);
    }
  };
  emitCartesian(emitCartesian, 0);

  return solutions;
}
