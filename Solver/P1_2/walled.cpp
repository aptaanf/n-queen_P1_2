#include <algorithm>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "nqueens.h"

namespace {

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
  inline bool equals(const Bits& o) const {
    for (int i = 0; i < W; ++i) {
      if (w[i] != o.w[i]) return false;
    }
    return true;
  }
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
    for (int i = 0; i < W; ++i) {
      if (w[i] & o.w[i]) return true;
    }
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
  uint16_t seg[4];
};

struct SubResult {
  int optCount = 0;
  std::vector<uint8_t> flatBoards;
};

template <int W>
class ComponentSolver {
 public:
  static constexpr int kMaxCells = W * 64;
  static constexpr int kSegWords = (W == 1) ? 1 : 2;
  static constexpr int kMaxSegs = kSegWords * 64;

  int m = 0;
  std::vector<Bits<W>> adj;
  std::vector<uint16_t> cellSeg;
  std::vector<Bits<W>> segMasks;
  std::vector<uint8_t> globalIds;
  int segCount[4] = {0, 0, 0, 0};
  int segBase[4] = {0, 0, 0, 0};

  int bestCount = 0;
  std::vector<uint8_t> flatBoards;
  uint8_t curBoard[kMaxCells];

  void buildComponent(int compSize, const int* compCells, const std::vector<CellInfo>& cells) {
    m = compSize;
    globalIds.resize(static_cast<size_t>(m));
    for (int i = 0; i < m; ++i) globalIds[i] = static_cast<uint8_t>(compCells[i]);

    int maxGlobalSeg = 0;
    for (int i = 0; i < m; ++i) {
      const CellInfo& ci = cells[compCells[i]];
      for (int d = 0; d < 4; ++d) {
        const int s = static_cast<int>(ci.seg[d]) + 1;
        if (s > maxGlobalSeg) maxGlobalSeg = s;
      }
    }

    std::vector<int16_t> segMap(static_cast<size_t>(maxGlobalSeg), -1);
    cellSeg.resize(static_cast<size_t>(m) * 4);
    int totalSegs = 0;

    for (int d = 0; d < 4; ++d) {
      segBase[d] = totalSegs;
      std::fill(segMap.begin(), segMap.end(), -1);
      int16_t localCnt = 0;
      for (int i = 0; i < m; ++i) {
        const int gSeg = cells[compCells[i]].seg[d];
        if (segMap[gSeg] < 0) segMap[gSeg] = localCnt++;
        cellSeg[static_cast<size_t>(i) * 4 + d] = static_cast<uint16_t>(segMap[gSeg]);
      }
      segCount[d] = localCnt;
      totalSegs += localCnt;
    }

    segMasks.resize(static_cast<size_t>(totalSegs));
    for (int s = 0; s < totalSegs; ++s) segMasks[s].clear();
    for (int i = 0; i < m; ++i) {
      for (int d = 0; d < 4; ++d) {
        segMasks[static_cast<size_t>(segBase[d] + cellSeg[static_cast<size_t>(i) * 4 + d])].set(i);
      }
    }

    adj.resize(static_cast<size_t>(m));
    for (int i = 0; i < m; ++i) {
      Bits<W> mask;
      mask.clear();
      for (int d = 0; d < 4; ++d) {
        mask.orEq(segMasks[static_cast<size_t>(segBase[d] + cellSeg[static_cast<size_t>(i) * 4 + d])]);
      }
      mask.reset(i);
      adj[i] = mask;
    }
  }

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

  int cliqueCoverBound(Bits<W> rem, int limit) const {
    int cliques = 0;
    while (rem.any()) {
      ++cliques;
      if (cliques >= limit) return cliques;
      const int u = rem.ctz();
      Bits<W> bestSeg;
      int bestSize = 0;
      for (int d = 0; d < 4; ++d) {
        const Bits<W> hit = rem & segMasks[static_cast<size_t>(segBase[d] + cellSeg[static_cast<size_t>(u) * 4 + d])];
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

  bool bipartiteCanReach(const Bits<W>& avail, int dL, int dR, int need) const {
    const int nL = segCount[dL];
    const int nR = segCount[dR];
    if (need <= 0) return true;
    if (nL < need || nR < need || nL > kMaxSegs || nR > kMaxSegs) return nL >= need && nR >= need;

    Bits<kSegWords> lAdj[kMaxSegs];
    int16_t lToIdx[kMaxSegs];
    int16_t matchR[kMaxSegs];
    uint8_t unmatchedL[kMaxSegs];

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
      const int idx = lToIdx[cellSeg[static_cast<size_t>(u) * 4 + dL]];
      lAdj[idx].set(cellSeg[static_cast<size_t>(u) * 4 + dR]);
    }

    for (int r = 0; r < nR; ++r) matchR[r] = -1;

    int matched = 0;
    for (int i = 0; i < activeCount; ++i) {
      unmatchedL[i] = 1;
      for (int rw = 0; rw < kSegWords; ++rw) {
        uint64_t bits = lAdj[i].w[rw];
        while (bits) {
          const int b = __builtin_ctzll(bits);
          const int r = (rw << 6) + b;
          if (matchR[r] < 0) {
            matchR[r] = static_cast<int16_t>(i);
            unmatchedL[i] = 0;
            ++matched;
            rw = kSegWords;
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
      for (int rw = 0; rw < kSegWords; ++rw) {
        uint64_t availR = lAdj[u].w[rw] & ~vis.w[rw];
        while (availR) {
          const int b = __builtin_ctzll(availR);
          availR &= availR - 1ULL;
          vis.w[rw] |= (1ULL << b);
          const int r = (rw << 6) + b;
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

  int segmentBound(const Bits<W>& avail) const {
    int minAct = 1000000;
    for (int d = 0; d < 4; ++d) {
      const int cnt = segCount[d];
      const int base = segBase[d];
      int act = 0;
      for (int s = 0; s < cnt; ++s) {
        if (avail.intersects(segMasks[static_cast<size_t>(base + s)])) ++act;
      }
      if (act < minAct) minAct = act;
    }
    return minAct;
  }

  Bits<W> extractConnectedComponent(const Bits<W>& avail) const {
    Bits<W> comp;
    comp.clear();
    const int start = avail.ctz();
    if (start < 0) return comp;
    comp.set(start);
    Bits<W> frontier = comp;
    while (frontier.any()) {
      const int u = frontier.ctz();
      frontier.reset(u);
      Bits<W> next = (avail & adj[u]).andNot(comp);
      comp.orEq(next);
      frontier.orEq(next);
    }
    return comp;
  }

  void recordLocalBoard(int outBase, int placed) {
    if (placed > bestCount) {
      bestCount = placed;
      flatBoards.clear();
    }
    if (placed == bestCount) {
      for (int i = 0; i < placed; ++i) {
        flatBoards.push_back(curBoard[outBase + i]);
      }
    }
  }

  void emitSubCartesian(int outBase, int placed, const std::vector<SubResult>& subs, size_t idx) {
    if (idx == subs.size()) {
      recordLocalBoard(outBase, placed);
      return;
    }
    const SubResult& sr = subs[idx];
    const int k = sr.optCount;
    const size_t cnt = sr.flatBoards.size() / static_cast<size_t>(k);
    for (size_t b = 0; b < cnt; ++b) {
      const uint8_t* ptr = sr.flatBoards.data() + b * static_cast<size_t>(k);
      for (int j = 0; j < k; ++j) curBoard[outBase + placed + j] = ptr[j];
      emitSubCartesian(outBase, placed + k, subs, idx + 1);
    }
  }

  SubResult solveIsolatedSub(const Bits<W>& subAvail, int minNeeded, int newOutBase) {
    const int savedBest = bestCount;
    std::vector<uint8_t> savedBoards = std::move(flatBoards);

    const int lb = greedyLowerBound(subAvail);
    bestCount = (minNeeded > lb) ? minNeeded : lb;
    flatBoards.clear();

    search(subAvail, 0, newOutBase, true);

    SubResult res;
    res.optCount = bestCount;
    res.flatBoards = std::move(flatBoards);

    bestCount = savedBest;
    flatBoards = std::move(savedBoards);
    return res;
  }

  void search(Bits<W> avail, int placed, int outBase, bool allowSplit) {
    while (true) {
      Bits<W> it = avail;
      while (it.any()) {
        const int u = it.ctz();
        it.reset(u);
        if (!avail.intersects(adj[u])) {
          curBoard[outBase + placed++] = static_cast<uint8_t>(u);
          avail.reset(u);
        }
      }

      if (!avail.any()) {
        recordLocalBoard(outBase, placed);
        return;
      }

      const int need = bestCount - placed;

      if (need == 1 && cliqueCoverBound(avail, 2) == 1) {
        Bits<W> leafIt = avail;
        while (leafIt.any()) {
          const int u = leafIt.ctz();
          leafIt.reset(u);
          curBoard[outBase + placed] = static_cast<uint8_t>(u);
          recordLocalBoard(outBase, placed + 1);
        }
        return;
      }

      int bestDim = 0;
      int minAct = 1000000;
      int bestMinSz = 1000;
      int bestSingleSeg = -1;
      int bestTieCnt = 0;
      int act[4] = {0, 0, 0, 0};
      bool zeroSlackUnit = false;
      int unitCell = -1;

      for (int d = 0; d < 4; ++d) {
        const int cnt = segCount[d];
        const int base = segBase[d];
        int a = 0;
        int bestSz = 1000;
        int firstSeg = -1;
        int tieCnt = 0;
        for (int s = 0; s < cnt; ++s) {
          const Bits<W> hit = avail & segMasks[static_cast<size_t>(base + s)];
          const int sz = hit.count();
          if (sz == 0) continue;
          ++a;
          if (sz < bestSz) {
            bestSz = sz;
            firstSeg = s;
            tieCnt = 1;
          } else if (sz == bestSz) {
            ++tieCnt;
          }
        }
        if (a < need) return;
        act[d] = a;
        if (a == need && bestSz == 1) {
          zeroSlackUnit = true;
          unitCell = (avail & segMasks[static_cast<size_t>(base + firstSeg)]).ctz();
          break;
        }
        if (a < minAct || (a == minAct && bestSz < bestMinSz)) {
          minAct = a;
          bestMinSz = bestSz;
          bestDim = d;
          bestSingleSeg = firstSeg;
          bestTieCnt = tieCnt;
        }
      }

      if (zeroSlackUnit) {
        curBoard[outBase + placed++] = static_cast<uint8_t>(unitCell);
        avail.andNotEq(adj[unitCell]);
        avail.reset(unitCell);
        continue;
      }

      if (minAct > need) {
        if (need >= 2 && cliqueCoverBound(avail, need) < need) return;
        if (need >= 3) {
          if (act[0] <= need + 2 && act[1] <= need + 2 && !bipartiteCanReach(avail, 0, 1, need)) {
            return;
          }
          if (act[2] <= need + 2 && act[3] <= need + 2 && !bipartiteCanReach(avail, 2, 3, need)) {
            return;
          }
        }

        if (allowSplit && (placed & 3) == 0 && avail.count() >= 12) {
          const Bits<W> firstComp = extractConnectedComponent(avail);
          if (!firstComp.equals(avail)) {
            std::vector<Bits<W>> parts;
            std::vector<int> partUbs;
            Bits<W> rem = avail;
            int ubSum = 0;
            while (rem.any()) {
              const Bits<W> c = parts.empty() ? firstComp : extractConnectedComponent(rem);
              rem.andNotEq(c);
              int ub = segmentBound(c);
              if (ub >= 2) {
                const int cc = cliqueCoverBound(c, ub);
                if (cc < ub) ub = cc;
              }
              parts.push_back(c);
              partUbs.push_back(ub);
              ubSum += ub;
            }
            if (placed + ubSum < bestCount) return;

            std::vector<size_t> order(parts.size());
            for (size_t i = 0; i < order.size(); ++i) order[i] = i;
            std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
              return partUbs[a] < partUbs[b];
            });

            std::vector<SubResult> subs;
            subs.reserve(parts.size());
            int optSum = 0;
            int remUb = ubSum;
            for (size_t idx : order) {
              remUb -= partUbs[idx];
              const int subMinNeed = (bestCount - placed) - optSum - remUb;
              SubResult sr = solveIsolatedSub(parts[idx], subMinNeed, outBase + placed);
              if (sr.flatBoards.empty() || sr.optCount < subMinNeed) return;
              optSum += sr.optCount;
              subs.push_back(std::move(sr));
            }
            if (placed + optSum >= bestCount) {
              emitSubCartesian(outBase, placed, subs, 0);
            }
            return;
          }
        }
      }

      const int base = segBase[bestDim];
      Bits<W> bestHit = avail & segMasks[static_cast<size_t>(base + bestSingleSeg)];
      if (bestTieCnt > 1) {
        const int cnt = segCount[bestDim];
        int bestDegSum = -1;
        for (int s = 0; s < cnt; ++s) {
          const Bits<W> hit = avail & segMasks[static_cast<size_t>(base + s)];
          if (hit.count() != bestMinSz) continue;
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

      uint8_t cand[16];
      uint8_t candDeg[16];
      int k = 0;
      Bits<W> hitIt = bestHit;
      while (hitIt.any()) {
        const int u = hitIt.ctz();
        hitIt.reset(u);
        cand[k++] = static_cast<uint8_t>(u);
      }
      if (k > 1) {
        for (int i = 0; i < k; ++i) candDeg[i] = static_cast<uint8_t>(avail.andCount(adj[cand[i]]));
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
      }

      for (int i = 0; i < k; ++i) {
        const int u = cand[i];
        curBoard[outBase + placed] = static_cast<uint8_t>(u);
        Bits<W> nextAvail = avail.andNot(adj[u]);
        nextAvail.reset(u);
        search(nextAvail, placed + 1, outBase, allowSplit);
      }

      if (act[bestDim] - 1 >= bestCount - placed) {
        avail.andNotEq(bestHit);
        continue;
      }
      return;
    }
  }

  SubResult solve(int compSize, const int* compCells, const std::vector<CellInfo>& cells) {
    buildComponent(compSize, compCells, cells);
    Bits<W> allAvail;
    allAvail.clear();
    for (int i = 0; i < m; ++i) allAvail.set(i);

    bestCount = greedyLowerBound(allAvail);
    flatBoards.clear();

    search(allAvail, 0, 0, true);

    for (uint8_t& id : flatBoards) {
      id = globalIds[id];
    }

    SubResult res;
    res.optCount = bestCount;
    res.flatBoards = std::move(flatBoards);
    return res;
  }
};

SubResult solveComponentDispatch(int compSize, const int* compCells,
                                 const std::vector<CellInfo>& cells) {
  if (compSize <= 64) {
    ComponentSolver<1> s;
    return s.solve(compSize, compCells, cells);
  }
  if (compSize <= 128) {
    ComponentSolver<2> s;
    return s.solve(compSize, compCells, cells);
  }
  if (compSize <= 192) {
    ComponentSolver<3> s;
    return s.solve(compSize, compCells, cells);
  }
  ComponentSolver<4> s;
  return s.solve(compSize, compCells, cells);
}

}

std::vector<std::vector<std::pair<int, int>>> solveWalled(
    int n, const std::vector<std::string>& grid) {
  std::vector<std::vector<std::pair<int, int>>> solutions;
  if (n <= 0 || n > 16 || static_cast<int>(grid.size()) != n) return solutions;

  int16_t cellId[256];
  for (int i = 0; i < n * n; ++i) cellId[i] = -1;

  std::vector<CellInfo> cells;
  cells.reserve(static_cast<size_t>(n * n));
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c) {
      if (grid[r][c] != 'W') {
        cellId[r * n + c] = static_cast<int16_t>(cells.size());
        CellInfo info{};
        info.r = static_cast<uint8_t>(r);
        info.c = static_cast<uint8_t>(c);
        cells.push_back(info);
      }
    }
  }

  const int numFree = static_cast<int>(cells.size());
  if (numFree == 0) {
    solutions.emplace_back();
    return solutions;
  }

  {
    int seg = 0;
    for (int r = 0; r < n; ++r) {
      bool inSeg = false;
      for (int c = 0; c < n; ++c) {
        const int id = cellId[r * n + c];
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
        const int id = cellId[r * n + c];
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
        const int id = cellId[r * n + c];
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
        const int id = cellId[r * n + c];
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

  const int gWords = (numFree + 63) >> 6;
  std::vector<uint64_t> globalAdj(static_cast<size_t>(numFree * gWords), 0ULL);
  for (int i = 0; i < numFree; ++i) {
    for (int j = i + 1; j < numFree; ++j) {
      if (cells[i].seg[0] == cells[j].seg[0] ||
          cells[i].seg[1] == cells[j].seg[1] ||
          cells[i].seg[2] == cells[j].seg[2] ||
          cells[i].seg[3] == cells[j].seg[3]) {
        globalAdj[static_cast<size_t>(i * gWords + (j >> 6))] |= (1ULL << (j & 63));
        globalAdj[static_cast<size_t>(j * gWords + (i >> 6))] |= (1ULL << (i & 63));
      }
    }
  }

  uint64_t unvisited[4] = {0, 0, 0, 0};
  for (int i = 0; i < numFree; ++i) unvisited[i >> 6] |= (1ULL << (i & 63));

  std::vector<std::pair<int, int>> forcedQueens;
  std::vector<SubResult> compResults;
  std::vector<int> compCells;

  for (int start = 0; start < numFree; ++start) {
    if (!((unvisited[start >> 6] >> (start & 63)) & 1ULL)) continue;
    unvisited[start >> 6] &= ~(1ULL << (start & 63));

    bool hasNeighbor = false;
    const uint64_t* sRow = globalAdj.data() + start * gWords;
    for (int w = 0; w < gWords; ++w) {
      if (sRow[w]) {
        hasNeighbor = true;
        break;
      }
    }
    if (!hasNeighbor) {
      forcedQueens.emplace_back(cells[start].r, cells[start].c);
      continue;
    }

    compCells.clear();
    compCells.push_back(start);
    for (size_t head = 0; head < compCells.size(); ++head) {
      const uint64_t* row = globalAdj.data() + compCells[head] * gWords;
      for (int w = 0; w < gWords; ++w) {
        uint64_t bits = row[w] & unvisited[w];
        while (bits) {
          const int b = __builtin_ctzll(bits);
          bits &= bits - 1ULL;
          unvisited[w] &= ~(1ULL << b);
          compCells.push_back((w << 6) + b);
        }
      }
    }
    compResults.push_back(
        solveComponentDispatch(static_cast<int>(compCells.size()), compCells.data(), cells));
  }

  if (compResults.empty()) {
    solutions.push_back(std::move(forcedQueens));
    return solutions;
  }

  if (compResults.size() == 1) {
    const SubResult& sr = compResults[0];
    const int k = sr.optCount;
    const size_t cnt = sr.flatBoards.size() / static_cast<size_t>(k);
    const size_t fSz = forcedQueens.size();
    solutions.resize(cnt);
    for (size_t b = 0; b < cnt; ++b) {
      solutions[b].reserve(fSz + static_cast<size_t>(k));
      for (size_t f = 0; f < fSz; ++f) solutions[b].push_back(forcedQueens[f]);
      const uint8_t* ptr = sr.flatBoards.data() + b * static_cast<size_t>(k);
      for (int j = 0; j < k; ++j) {
        const CellInfo& ci = cells[ptr[j]];
        solutions[b].emplace_back(ci.r, ci.c);
      }
    }
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
    const SubResult& cr = compResults[idx];
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
