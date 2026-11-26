#include <algorithm>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "nqueens.h"

namespace {

struct Mask {
  uint64_t w[4];

  inline void clear(int words) {
    for (int i = 0; i < words; ++i) w[i] = 0;
  }
  inline bool any(int words) const {
    uint64_t acc = 0;
    for (int i = 0; i < words; ++i) acc |= w[i];
    return acc != 0;
  }
  inline bool equals(const Mask& o, int words) const {
    for (int i = 0; i < words; ++i) {
      if (w[i] != o.w[i]) return false;
    }
    return true;
  }
  inline void set(int i) { w[i >> 6] |= (1ULL << (i & 63)); }
  inline void reset(int i) { w[i >> 6] &= ~(1ULL << (i & 63)); }
  inline int count(int words) const {
    int s = 0;
    for (int i = 0; i < words; ++i) s += __builtin_popcountll(w[i]);
    return s;
  }
  inline int ctz(int words) const {
    for (int i = 0; i < words; ++i) {
      if (w[i]) return (i << 6) + __builtin_ctzll(w[i]);
    }
    return -1;
  }
  inline bool intersects(const Mask& o, int words) const {
    for (int i = 0; i < words; ++i) {
      if (w[i] & o.w[i]) return true;
    }
    return false;
  }
  inline int andCount(const Mask& o, int words) const {
    int s = 0;
    for (int i = 0; i < words; ++i) s += __builtin_popcountll(w[i] & o.w[i]);
    return s;
  }
  inline Mask bitAnd(const Mask& o, int words) const {
    Mask r;
    for (int i = 0; i < words; ++i) r.w[i] = w[i] & o.w[i];
    return r;
  }
  inline Mask andNot(const Mask& o, int words) const {
    Mask r;
    for (int i = 0; i < words; ++i) r.w[i] = w[i] & ~o.w[i];
    return r;
  }
  inline void orEq(const Mask& o, int words) {
    for (int i = 0; i < words; ++i) w[i] |= o.w[i];
  }
  inline void andEq(const Mask& o, int words) {
    for (int i = 0; i < words; ++i) w[i] &= o.w[i];
  }
  inline void andNotEq(const Mask& o, int words) {
    for (int i = 0; i < words; ++i) w[i] &= ~o.w[i];
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

class WalledBranchAndBound {
 public:
  int m = 0;
  int words = 1;
  std::vector<Mask> adj;
  std::vector<uint16_t> cellSeg;
  std::vector<Mask> segMasks;
  std::vector<uint8_t> globalIds;
  int segCount[4] = {0, 0, 0, 0};
  int segBase[4] = {0, 0, 0, 0};

  int bestCount = 0;
  std::vector<uint8_t> flatBoards;
  uint8_t curBoard[256];

  void buildComponent(int compSize, const int* compCells, const std::vector<CellInfo>& cells) {
    m = compSize;
    words = (m + 63) >> 6;
    globalIds.resize(static_cast<size_t>(m));
    for (int i = 0; i < m; ++i) globalIds[i] = static_cast<uint8_t>(compCells[i]);

    int maxGlobalSeg = 0;
    for (int i = 0; i < m; ++i) {
      const CellInfo& ci = cells[compCells[i]];
      for (int d = 0; d < 4; ++d) {
        if (static_cast<int>(ci.seg[d]) + 1 > maxGlobalSeg) {
          maxGlobalSeg = static_cast<int>(ci.seg[d]) + 1;
        }
      }
    }

    std::vector<int> segMap(static_cast<size_t>(maxGlobalSeg), -1);
    cellSeg.resize(static_cast<size_t>(m) * 4);
    int totalSegs = 0;

    for (int d = 0; d < 4; ++d) {
      segBase[d] = totalSegs;
      std::fill(segMap.begin(), segMap.end(), -1);
      int localCnt = 0;
      for (int i = 0; i < m; ++i) {
        const int gSeg = cells[compCells[i]].seg[d];
        if (segMap[gSeg] < 0) segMap[gSeg] = localCnt++;
        cellSeg[static_cast<size_t>(i) * 4 + d] = static_cast<uint16_t>(segMap[gSeg]);
      }
      segCount[d] = localCnt;
      totalSegs += localCnt;
    }

    segMasks.resize(static_cast<size_t>(totalSegs));
    for (int s = 0; s < totalSegs; ++s) segMasks[s].clear(words);
    for (int i = 0; i < m; ++i) {
      for (int d = 0; d < 4; ++d) {
        const int sIdx = segBase[d] + cellSeg[static_cast<size_t>(i) * 4 + d];
        segMasks[sIdx].set(i);
      }
    }

    adj.resize(static_cast<size_t>(m));
    for (int i = 0; i < m; ++i) {
      Mask mask;
      mask.clear(words);
      for (int d = 0; d < 4; ++d) {
        const int sIdx = segBase[d] + cellSeg[static_cast<size_t>(i) * 4 + d];
        mask.orEq(segMasks[sIdx], words);
      }
      mask.reset(i);
      adj[i] = mask;
    }
  }

  int greedyLowerBound(Mask avail) const {
    int best = 0;
    for (int mode = 0; mode < 2; ++mode) {
      Mask rem = avail;
      int cnt = 0;
      while (rem.any(words)) {
        int pick = -1;
        int bestScore = 1000000;
        Mask it = rem;
        while (it.any(words)) {
          const int u = it.ctz(words);
          it.reset(u);
          const int deg = rem.andCount(adj[u], words);
          const int score = (mode == 0) ? deg : (deg * 256 - u);
          if (score < bestScore) {
            bestScore = score;
            pick = u;
            if (deg == 0) break;
          }
        }
        ++cnt;
        rem.andNotEq(adj[pick], words);
        rem.reset(pick);
      }
      if (cnt > best) best = cnt;
    }
    return best;
  }

  int cliqueCoverBound(Mask rem) const {
    int cliques = 0;
    while (rem.any(words)) {
      ++cliques;
      const int u = rem.ctz(words);
      Mask bestSeg;
      int bestSize = 0;
      for (int d = 0; d < 4; ++d) {
        const int sIdx = segBase[d] + cellSeg[static_cast<size_t>(u) * 4 + d];
        const Mask hit = rem.bitAnd(segMasks[sIdx], words);
        const int sz = hit.count(words);
        if (sz > bestSize) {
          bestSize = sz;
          bestSeg = hit;
        }
      }
      Mask common = rem.andNot(bestSeg, words);
      Mask memb = bestSeg;
      while (memb.any(words) && common.any(words)) {
        const int v = memb.ctz(words);
        memb.reset(v);
        common.andEq(adj[v], words);
      }
      while (common.any(words)) {
        const int w = common.ctz(words);
        common.reset(w);
        bestSeg.set(w);
        common.andEq(adj[w], words);
      }
      rem.andNotEq(bestSeg, words);
    }
    return cliques;
  }

  bool bipartiteCanReach(const Mask& avail, int dL, int dR, int need) const {
    const int nL = segCount[dL];
    const int nR = segCount[dR];
    if (need <= 0) return true;
    if (nL < need || nR < need) return false;

    const int rWords = (nR + 63) >> 6;
    uint64_t lAdj[128 * 2];
    int16_t lToIdx[128];
    int16_t matchR[128];
    uint8_t unmatchedL[128];

    int activeCount = 0;
    const int baseL = segBase[dL];
    for (int l = 0; l < nL; ++l) {
      if (avail.intersects(segMasks[baseL + l], words)) {
        lToIdx[l] = static_cast<int16_t>(activeCount);
        uint64_t* row = lAdj + activeCount * rWords;
        for (int rw = 0; rw < rWords; ++rw) row[rw] = 0;
        ++activeCount;
      } else {
        lToIdx[l] = -1;
      }
    }
    if (activeCount < need) return false;

    Mask it = avail;
    while (it.any(words)) {
      const int u = it.ctz(words);
      it.reset(u);
      const int idx = lToIdx[cellSeg[static_cast<size_t>(u) * 4 + dL]];
      const int rSeg = cellSeg[static_cast<size_t>(u) * 4 + dR];
      lAdj[idx * rWords + (rSeg >> 6)] |= (1ULL << (rSeg & 63));
    }

    for (int r = 0; r < nR; ++r) matchR[r] = -1;

    int matched = 0;
    for (int i = 0; i < activeCount; ++i) {
      unmatchedL[i] = 1;
      const uint64_t* row = lAdj + i * rWords;
      for (int rw = 0; rw < rWords; ++rw) {
        uint64_t bits = row[rw];
        while (bits) {
          const int b = __builtin_ctzll(bits);
          const int r = (rw << 6) + b;
          if (matchR[r] < 0) {
            matchR[r] = static_cast<int16_t>(i);
            unmatchedL[i] = 0;
            ++matched;
            rw = rWords;
            break;
          }
          bits &= bits - 1ULL;
        }
      }
    }
    if (matched >= need) return true;

    int remLeft = activeCount - matched;
    if (matched + remLeft < need) return false;

    auto dfs = [&](auto& self, int u, uint64_t* vis) -> bool {
      const uint64_t* row = lAdj + u * rWords;
      for (int rw = 0; rw < rWords; ++rw) {
        uint64_t availR = row[rw] & ~vis[rw];
        while (availR) {
          const int b = __builtin_ctzll(availR);
          availR &= availR - 1ULL;
          vis[rw] |= (1ULL << b);
          const int r = (rw << 6) + b;
          if (matchR[r] < 0 || self(self, matchR[r], vis)) {
            matchR[r] = static_cast<int16_t>(u);
            return true;
          }
        }
      }
      return false;
    };

    uint64_t vis[2];
    for (int i = 0; i < activeCount; ++i) {
      if (!unmatchedL[i]) continue;
      for (int rw = 0; rw < rWords; ++rw) vis[rw] = 0;
      if (dfs(dfs, i, vis)) {
        ++matched;
        if (matched >= need) return true;
      }
      --remLeft;
      if (matched + remLeft < need) return false;
    }
    return matched >= need;
  }

  int segmentBound(const Mask& avail) const {
    int minAct = 1000000;
    for (int d = 0; d < 4; ++d) {
      const int cnt = segCount[d];
      const int base = segBase[d];
      int act = 0;
      for (int s = 0; s < cnt; ++s) {
        if (avail.intersects(segMasks[base + s], words)) ++act;
      }
      if (act < minAct) minAct = act;
    }
    return minAct;
  }

  bool upperBoundOk(const Mask& avail, int need, int minAct) const {
    if (need <= 0) return true;
    if (minAct < need) return false;
    if (minAct == need) return true;
    if (need >= 2 && cliqueCoverBound(avail) < need) return false;
    if (need >= 3) {
      if (segCount[0] <= 128 && segCount[1] <= 128 && !bipartiteCanReach(avail, 0, 1, need)) {
        return false;
      }
      if (segCount[2] <= 128 && segCount[3] <= 128 && !bipartiteCanReach(avail, 2, 3, need)) {
        return false;
      }
    }
    return true;
  }

  Mask extractConnectedComponent(const Mask& avail) const {
    Mask comp;
    comp.clear(words);
    const int start = avail.ctz(words);
    if (start < 0) return comp;
    comp.set(start);
    Mask frontier = comp;
    while (frontier.any(words)) {
      const int u = frontier.ctz(words);
      frontier.reset(u);
      Mask next = avail.bitAnd(adj[u], words);
      next.andNotEq(comp, words);
      comp.orEq(next, words);
      frontier.orEq(next, words);
    }
    return comp;
  }

  void recordLocalBoard(int placed) {
    if (placed > bestCount) {
      bestCount = placed;
      flatBoards.clear();
    }
    if (placed == bestCount) {
      for (int i = 0; i < placed; ++i) {
        flatBoards.push_back(curBoard[i]);
      }
    }
  }

  void emitSubCartesian(int placed, const std::vector<SubResult>& subs, size_t idx) {
    if (idx == subs.size()) {
      recordLocalBoard(placed);
      return;
    }
    const SubResult& sr = subs[idx];
    const int k = sr.optCount;
    const size_t cnt = sr.flatBoards.size() / static_cast<size_t>(k);
    for (size_t b = 0; b < cnt; ++b) {
      const uint8_t* ptr = sr.flatBoards.data() + b * static_cast<size_t>(k);
      for (int j = 0; j < k; ++j) curBoard[placed + j] = ptr[j];
      emitSubCartesian(placed + k, subs, idx + 1);
    }
  }

  SubResult solveIsolatedSub(Mask subAvail, int minNeeded) {
    const int savedBest = bestCount;
    std::vector<uint8_t> savedBoards = std::move(flatBoards);
    uint8_t savedCur[256];
    for (int i = 0; i < m; ++i) savedCur[i] = curBoard[i];

    const int lb = greedyLowerBound(subAvail);
    bestCount = (minNeeded > lb) ? minNeeded : lb;
    flatBoards.clear();
    search(subAvail, 0, true);

    SubResult res;
    res.optCount = bestCount;
    res.flatBoards = std::move(flatBoards);

    bestCount = savedBest;
    flatBoards = std::move(savedBoards);
    for (int i = 0; i < m; ++i) curBoard[i] = savedCur[i];
    return res;
  }

  void search(Mask avail, int placed, bool allowSplit) {
    while (true) {
      Mask it = avail;
      while (it.any(words)) {
        const int u = it.ctz(words);
        it.reset(u);
        if (!avail.intersects(adj[u], words)) {
          curBoard[placed++] = static_cast<uint8_t>(u);
          avail.reset(u);
        }
      }

      if (!avail.any(words)) {
        recordLocalBoard(placed);
        return;
      }

      const int need = bestCount - placed;

      int bestDim = 0;
      int minAct = 1000000;
      int bestMinSz = 1000;
      int act[4] = {0, 0, 0, 0};
      int minSzDim[4] = {1000, 1000, 1000, 1000};
      bool zeroSlackUnit = false;
      int unitCell = -1;

      for (int d = 0; d < 4; ++d) {
        const int cnt = segCount[d];
        const int base = segBase[d];
        int a = 0;
        int bestSz = 1000;
        int bestSeg = -1;
        for (int s = 0; s < cnt; ++s) {
          const Mask hit = avail.bitAnd(segMasks[base + s], words);
          const int sz = hit.count(words);
          if (sz == 0) continue;
          ++a;
          if (sz < bestSz) {
            bestSz = sz;
            bestSeg = s;
          }
        }
        if (a < need) return;
        act[d] = a;
        minSzDim[d] = bestSz;
        if (a == need && bestSz == 1) {
          zeroSlackUnit = true;
          unitCell = avail.bitAnd(segMasks[base + bestSeg], words).ctz(words);
          break;
        }
        if (a < minAct || (a == minAct && bestSz < bestMinSz)) {
          minAct = a;
          bestMinSz = bestSz;
          bestDim = d;
        }
      }

      if (zeroSlackUnit) {
        curBoard[placed++] = static_cast<uint8_t>(unitCell);
        avail.andNotEq(adj[unitCell], words);
        avail.reset(unitCell);
        continue;
      }

      if (!upperBoundOk(avail, need, minAct)) return;

      if (allowSplit && avail.count(words) >= 12) {
        const Mask firstComp = extractConnectedComponent(avail);
        if (!firstComp.equals(avail, words)) {
          std::vector<Mask> parts;
          std::vector<int> partUbs;
          Mask rem = avail;
          int ubSum = 0;
          while (rem.any(words)) {
            const Mask c = (parts.empty()) ? firstComp : extractConnectedComponent(rem);
            rem.andNotEq(c, words);
            int ub = segmentBound(c);
            if (ub >= 2) {
              const int cc = cliqueCoverBound(c);
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
            SubResult sr = solveIsolatedSub(parts[idx], subMinNeed);
            if (sr.flatBoards.empty() || sr.optCount < subMinNeed) return;
            optSum += sr.optCount;
            subs.push_back(std::move(sr));
          }
          if (placed + optSum >= bestCount) {
            emitSubCartesian(placed, subs, 0);
          }
          return;
        }
      }

      const int cnt = segCount[bestDim];
      const int base = segBase[bestDim];
      const int targetSz = minSzDim[bestDim];
      Mask bestHit;
      bestHit.clear(words);
      int bestDegSum = -1;
      for (int s = 0; s < cnt; ++s) {
        const Mask hit = avail.bitAnd(segMasks[base + s], words);
        if (hit.count(words) != targetSz) continue;
        int degSum = 0;
        Mask hIt = hit;
        while (hIt.any(words)) {
          const int u = hIt.ctz(words);
          hIt.reset(u);
          degSum += avail.andCount(adj[u], words);
        }
        if (degSum > bestDegSum) {
          bestDegSum = degSum;
          bestHit = hit;
        }
      }

      uint8_t cand[16];
      uint8_t candDeg[16];
      int k = 0;
      Mask hitIt = bestHit;
      while (hitIt.any(words)) {
        const int u = hitIt.ctz(words);
        hitIt.reset(u);
        cand[k] = static_cast<uint8_t>(u);
        candDeg[k] = static_cast<uint8_t>(avail.andCount(adj[u], words));
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

      for (int i = 0; i < k; ++i) {
        const int u = cand[i];
        curBoard[placed] = static_cast<uint8_t>(u);
        Mask nextAvail = avail.andNot(adj[u], words);
        nextAvail.reset(u);
        search(nextAvail, placed + 1, allowSplit);
      }

      if (act[bestDim] - 1 >= bestCount - placed) {
        const Mask skipAvail = avail.andNot(bestHit, words);
        const int skipMinAct = segmentBound(skipAvail);
        if (upperBoundOk(skipAvail, bestCount - placed, skipMinAct)) {
          search(skipAvail, placed, allowSplit);
        }
      }
      return;
    }
  }

  SubResult solve(int compSize, const int* compCells, const std::vector<CellInfo>& cells) {
    buildComponent(compSize, compCells, cells);
    Mask allAvail;
    allAvail.clear(words);
    for (int i = 0; i < m; ++i) allAvail.set(i);

    bestCount = greedyLowerBound(allAvail);
    flatBoards.clear();
    search(allAvail, 0, true);

    for (uint8_t& id : flatBoards) {
      id = globalIds[id];
    }

    SubResult res;
    res.optCount = bestCount;
    res.flatBoards = std::move(flatBoards);
    return res;
  }
};

}

std::vector<std::vector<std::pair<int, int>>> solveWalled(
    int n, const std::vector<std::string>& grid) {
  std::vector<std::vector<std::pair<int, int>>> solutions;
  if (n <= 0 || n > 16 || static_cast<int>(grid.size()) != n) return solutions;

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
    solutions.emplace_back();
    return solutions;
  }

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

  const int gWords = (numFree + 63) >> 6;
  std::vector<Mask> globalAdj(static_cast<size_t>(numFree));
  for (int i = 0; i < numFree; ++i) globalAdj[i].clear(gWords);
  for (int i = 0; i < numFree; ++i) {
    for (int j = i + 1; j < numFree; ++j) {
      if (cells[i].seg[0] == cells[j].seg[0] ||
          cells[i].seg[1] == cells[j].seg[1] ||
          cells[i].seg[2] == cells[j].seg[2] ||
          cells[i].seg[3] == cells[j].seg[3]) {
        globalAdj[i].set(j);
        globalAdj[j].set(i);
      }
    }
  }

  Mask unvisited;
  unvisited.clear(gWords);
  for (int i = 0; i < numFree; ++i) unvisited.set(i);

  std::vector<std::pair<int, int>> forcedQueens;
  std::vector<SubResult> compResults;
  std::vector<int> compCells;
  WalledBranchAndBound solver;

  while (unvisited.any(gWords)) {
    const int start = unvisited.ctz(gWords);
    unvisited.reset(start);
    if (!globalAdj[start].any(gWords)) {
      forcedQueens.emplace_back(cells[start].r, cells[start].c);
      continue;
    }

    compCells.clear();
    compCells.push_back(start);
    for (size_t head = 0; head < compCells.size(); ++head) {
      Mask nbrs = unvisited.bitAnd(globalAdj[compCells[head]], gWords);
      while (nbrs.any(gWords)) {
        const int v = nbrs.ctz(gWords);
        nbrs.reset(v);
        unvisited.reset(v);
        compCells.push_back(v);
      }
    }
    compResults.push_back(solver.solve(static_cast<int>(compCells.size()), compCells.data(), cells));
  }

  if (compResults.empty()) {
    solutions.push_back(std::move(forcedQueens));
    return solutions;
  }

  if (compResults.size() == 1 && forcedQueens.empty()) {
    const SubResult& sr = compResults[0];
    const int k = sr.optCount;
    const size_t cnt = sr.flatBoards.size() / static_cast<size_t>(k);
    solutions.resize(cnt);
    for (size_t b = 0; b < cnt; ++b) {
      solutions[b].reserve(static_cast<size_t>(k));
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
