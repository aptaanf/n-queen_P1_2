#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "nqueens.h"

namespace {


__attribute__((always_inline)) inline void pushReserved(
    std::vector<std::vector<int>>& dst, const int* p, int n) {
  if (dst.size() == dst.capacity()) __builtin_unreachable();
  dst.emplace_back(p, p + n);
}

template <int N>
__attribute__((noinline)) void emitDirectFixed(
    std::vector<std::vector<int>>& dst, const uint8_t* raw, uint8_t mask) {
  constexpr int last = N - 1;
  int s[N];
  int t[N];

  if (__builtin_expect(mask == 0xFFu, 1)) {
    if (__builtin_expect(dst.size() + 8 > dst.capacity(), 0)) {
      const size_t nextCap = dst.capacity() ? dst.capacity() * 2 : 16;
      dst.reserve(nextCap < dst.size() + 8 ? dst.size() + 8 : nextCap);
    }
#pragma clang loop unroll(full)
    for (int r = 0; r < N; ++r) {
      const int c = raw[r];
      s[r] = c;
      t[c] = r;
    }
    pushReserved(dst, s, N);

#pragma clang loop unroll(full)
    for (int r = 0; r < N; ++r) s[r] = last - raw[r];
    pushReserved(dst, s, N);

#pragma clang loop unroll(full)
    for (int r = 0; r < N; ++r) s[last - r] = raw[r];
    pushReserved(dst, s, N);

#pragma clang loop unroll(full)
    for (int r = 0; r < N; ++r) s[last - r] = last - raw[r];
    pushReserved(dst, s, N);

    pushReserved(dst, t, N);

#pragma clang loop unroll(full)
    for (int r = 0; r < N; ++r) s[r] = last - t[r];
    pushReserved(dst, s, N);

#pragma clang loop unroll(full)
    for (int r = 0; r < N; ++r) s[last - r] = t[r];
    pushReserved(dst, s, N);

#pragma clang loop unroll(full)
    for (int r = 0; r < N; ++r) s[last - r] = last - t[r];
    pushReserved(dst, s, N);
    return;
  }

  for (int r = 0; r < N; ++r) t[raw[r]] = r;
  if (mask & 0x01u) { for (int r = 0; r < N; ++r) s[r] = raw[r]; dst.emplace_back(s, s + N); }
  if (mask & 0x02u) { for (int r = 0; r < N; ++r) s[r] = last - raw[r]; dst.emplace_back(s, s + N); }
  if (mask & 0x04u) { for (int r = 0; r < N; ++r) s[last - r] = raw[r]; dst.emplace_back(s, s + N); }
  if (mask & 0x08u) { for (int r = 0; r < N; ++r) s[last - r] = last - raw[r]; dst.emplace_back(s, s + N); }
  if (mask & 0x10u) dst.emplace_back(t, t + N);
  if (mask & 0x20u) { for (int r = 0; r < N; ++r) s[r] = last - t[r]; dst.emplace_back(s, s + N); }
  if (mask & 0x40u) { for (int r = 0; r < N; ++r) s[last - r] = t[r]; dst.emplace_back(s, s + N); }
  if (mask & 0x80u) { for (int r = 0; r < N; ++r) s[last - r] = last - t[r]; dst.emplace_back(s, s + N); }
}

__attribute__((noinline)) void emitDirectGeneric(
    std::vector<std::vector<int>>& dst, const uint8_t* raw, int n, uint8_t mask) {
  const int last = n - 1;
  int s[12];
  int t[12];
  for (int r = 0; r < n; ++r) t[raw[r]] = r;
  if (mask & 0x01u) { for (int r = 0; r < n; ++r) s[r] = raw[r]; dst.emplace_back(s, s + n); }
  if (mask & 0x02u) { for (int r = 0; r < n; ++r) s[r] = last - raw[r]; dst.emplace_back(s, s + n); }
  if (mask & 0x04u) { for (int r = 0; r < n; ++r) s[last - r] = raw[r]; dst.emplace_back(s, s + n); }
  if (mask & 0x08u) { for (int r = 0; r < n; ++r) s[last - r] = last - raw[r]; dst.emplace_back(s, s + n); }
  if (mask & 0x10u) dst.emplace_back(t, t + n);
  if (mask & 0x20u) { for (int r = 0; r < n; ++r) s[r] = last - t[r]; dst.emplace_back(s, s + n); }
  if (mask & 0x40u) { for (int r = 0; r < n; ++r) s[last - r] = t[r]; dst.emplace_back(s, s + n); }
  if (mask & 0x80u) { for (int r = 0; r < n; ++r) s[last - r] = last - t[r]; dst.emplace_back(s, s + n); }
}

struct D4MrvSolver {
  int n;
  int last;
  int curC0;
  bool curCase1;
  uint32_t fullMask;
  uint8_t pos[12];
  std::vector<std::vector<int>>* out;

  inline void emit(uint8_t mask) const {
    if (n == 12) emitDirectFixed<12>(*out, pos, mask);
    else if (n == 11) emitDirectFixed<11>(*out, pos, mask);
    else if (n == 10) emitDirectFixed<10>(*out, pos, mask);
    else emitDirectGeneric(*out, pos, n, mask);
  }

  __attribute__((noinline)) void recordCase1() const {
    uint8_t t[12];
    for (int r = 0; r < n; ++r) t[pos[r]] = static_cast<uint8_t>(r);
    const int cmp = std::memcmp(pos, t, static_cast<size_t>(n));
    if (cmp > 0) return;
    emit((cmp == 0) ? 0x0Fu : 0xFFu);
  }

  __attribute__((noinline)) void recordCase2() const {
    const int c0 = curC0;
    if (pos[c0] != static_cast<uint8_t>(last) &&
        pos[last - c0] != 0u &&
        pos[last] != static_cast<uint8_t>(last - c0)) {
      emit(0xFFu);
      return;
    }

    uint8_t t[12];
    for (int r = 0; r < n; ++r) t[pos[r]] = static_cast<uint8_t>(r);
    uint8_t sym[8][12];
    for (int r = 0; r < n; ++r) {
      const uint8_t br = pos[r], tr = t[r];
      sym[0][r] = br;
      sym[1][r] = static_cast<uint8_t>(last - br);
      sym[2][last - r] = br;
      sym[3][last - r] = static_cast<uint8_t>(last - br);
      sym[4][r] = tr;
      sym[5][r] = static_cast<uint8_t>(last - tr);
      sym[6][last - r] = tr;
      sym[7][last - r] = static_cast<uint8_t>(last - tr);
    }

    const uint8_t c0u = static_cast<uint8_t>(c0);
    if (sym[3][0] == c0u && std::memcmp(sym[3], sym[0], static_cast<size_t>(n)) < 0) return;
    if (sym[5][0] == c0u && std::memcmp(sym[5], sym[0], static_cast<size_t>(n)) < 0) return;
    if (sym[6][0] == c0u && std::memcmp(sym[6], sym[0], static_cast<size_t>(n)) < 0) return;

    uint8_t mask = 1u;
    for (int k = 1; k < 8; ++k) {
      bool dup = false;
      for (int j = 0; j < k; ++j) {
        if ((mask & (1u << j)) && std::memcmp(sym[k], sym[j], static_cast<size_t>(n)) == 0) {
          dup = true;
          break;
        }
      }
      if (!dup) mask |= static_cast<uint8_t>(1u << k);
    }
    emit(mask);
  }

  inline void record() const {
    if (curCase1) recordCase1();
    else recordCase2();
  }

  void search(int branchRow, uint32_t branchMask,
              uint32_t remRows, uint32_t remCols, int remCount, const uint16_t* dom) {
    uint16_t curDom[12];
    const int lastIdx = last;

    while (branchMask) {
      int pRow = branchRow;
      uint32_t pBit = branchMask & (~branchMask + 1u);
      branchMask ^= pBit;

      uint32_t rRows = remRows;
      uint32_t rCols = remCols;
      int rCnt = remCount;
      const uint16_t* srcDom = dom;

      while (true) {
        pos[pRow] = static_cast<uint8_t>(__builtin_ctz(pBit));
        rRows ^= (1u << pRow);
        rCols ^= pBit;
        --rCnt;

        if (__builtin_expect(rCnt <= 2, 0)) {
          if (rCnt == 2) {
            const int r1 = __builtin_ctz(rRows);
            const int r2 = __builtin_ctz(rRows & (rRows - 1u));
            const int d1 = (r1 > pRow) ? (r1 - pRow) : (pRow - r1);
            const int d2 = (r2 > pRow) ? (r2 - pRow) : (pRow - r2);
            uint32_t m1 = static_cast<uint32_t>(srcDom[r1]) & ~(pBit | (pBit << d1) | (pBit >> d1));
            const uint32_t m2 = static_cast<uint32_t>(srcDom[r2]) & ~(pBit | (pBit << d2) | (pBit >> d2));
            if (m1 && m2 && ((m1 | m2) == rCols)) {
              const int d12 = r2 - r1;
              while (m1) {
                const uint32_t b1 = m1 & (~m1 + 1u);
                m1 ^= b1;
                const uint32_t b2 = m2 & ~(b1 | (b1 << d12) | (b1 >> d12));
                if (b2) {
                  pos[r1] = static_cast<uint8_t>(__builtin_ctz(b1));
                  pos[r2] = static_cast<uint8_t>(__builtin_ctz(b2));
                  record();
                }
              }
            }
          } else if (rCnt == 1) {
            const int rLast = __builtin_ctz(rRows);
            const int d = (rLast > pRow) ? (rLast - pRow) : (pRow - rLast);
            const uint32_t m = static_cast<uint32_t>(srcDom[rLast]) & ~(pBit | (pBit << d) | (pBit >> d));
            if (m) {
              pos[rLast] = static_cast<uint8_t>(__builtin_ctz(m));
              record();
            }
          } else {
            record();
          }
          break;
        }

        uint32_t tmp = rRows;
        uint32_t seen1 = 0u;
        uint32_t bestKey = 0xFFFFFFFFu;
        bool ok = true;

        while (tmp) {
          const int r = __builtin_ctz(tmp);
          tmp &= tmp - 1u;
          const int d = (r > pRow) ? (r - pRow) : (pRow - r);
          const uint32_t m = static_cast<uint32_t>(srcDom[r]) & ~(pBit | (pBit << d) | (pBit >> d));
          if (!m) {
            ok = false;
            break;
          }
          curDom[r] = static_cast<uint16_t>(m);
          seen1 |= m;
          const int distCtr = (2 * r >= lastIdx) ? (2 * r - lastIdx) : (lastIdx - 2 * r);
          const uint32_t key = (static_cast<uint32_t>(__builtin_popcount(m)) << 24) |
                               (static_cast<uint32_t>(distCtr) << 20) |
                               (static_cast<uint32_t>(r) << 16) | m;
          if (key < bestKey) bestKey = key;
        }
        if (!ok || seen1 != rCols) break;

        const int bestRow = static_cast<int>((bestKey >> 16) & 15u);
        const uint32_t bestMask = bestKey & 0xFFFFu;

        if ((bestKey >> 24) == 1u) {
          pRow = bestRow;
          pBit = bestMask;
          srcDom = curDom;
          continue;
        }

        search(bestRow, bestMask, rRows, rCols, rCnt, curDom);
        break;
      }
    }
  }

  void launch(uint32_t remRows, uint32_t remCols, int remCnt, const uint16_t* dom) {
    uint32_t tmp = remRows;
    uint32_t seen1 = 0u;
    uint32_t bestKey = 0xFFFFFFFFu;
    while (tmp) {
      const int r = __builtin_ctz(tmp);
      tmp &= tmp - 1u;
      const uint32_t m = dom[r];
      if (!m) return;
      seen1 |= m;
      const int distCtr = (2 * r >= last) ? (2 * r - last) : (last - 2 * r);
      const uint32_t key = (static_cast<uint32_t>(__builtin_popcount(m)) << 24) |
                           (static_cast<uint32_t>(distCtr) << 20) |
                           (static_cast<uint32_t>(r) << 16) | m;
      if (key < bestKey) bestKey = key;
    }
    if (seen1 != remCols) return;
    search(static_cast<int>((bestKey >> 16) & 15u), bestKey & 0xFFFFu,
           remRows, remCols, remCnt, dom);
  }
};

}

std::vector<std::vector<int>> solveNQueens(int n) {
  std::vector<std::vector<int>> solutions;
  if (n <= 0 || n > 12) return solutions;
  if (n == 1) {
    solutions.emplace_back(1, 0);
    return solutions;
  }
  if (n < 4) return solutions;

  D4MrvSolver solver;
  solver.n = n;
  solver.last = n - 1;
  solver.fullMask = (1u << n) - 1u;
  solver.out = &solutions;

  const int last = n - 1;
  const uint32_t fullMask = solver.fullMask;
  const uint32_t rightBit = 1u << last;
  const uint32_t edgeMask = 1u | rightBit;

  solver.curCase1 = true;
  solver.pos[0] = 0;
  for (int c1 = 2; c1 < n; ++c1) {
    const uint32_t b1 = 1u << c1;
    solver.pos[1] = static_cast<uint8_t>(c1);
    uint16_t dom[12];
    for (int r = 2; r < n; ++r) {
      const uint32_t allow = (r < c1) ? (fullMask & ~2u) : fullMask;
      const int d1 = r - 1;
      dom[r] = static_cast<uint16_t>(allow & ~(1u | (1u << r) | b1 | (b1 << d1) | (b1 >> d1)));
    }
    solver.launch(fullMask ^ 3u, fullMask ^ (1u | b1), n - 2, dom);
  }

  solver.curCase1 = false;
  for (int c0 = 1; c0 <= (n - 2) / 2; ++c0) {
    solver.curC0 = c0;
    const uint32_t b0 = 1u << c0;
    solver.pos[0] = static_cast<uint8_t>(c0);
    const uint32_t lastMask = ((1u << (n - c0)) - 1u) & ~((1u << (c0 + 1)) - 1u);
    uint16_t dom[12];
    for (int r = 1; r < n; ++r) {
      uint32_t allow = (r == last) ? lastMask : fullMask;
      if (r != last) {
        if (r < c0 || r >= n - c0) allow &= ~edgeMask;
        if (r == c0) allow &= ~1u;
        if (r == last - c0) allow &= ~rightBit;
      }
      dom[r] = static_cast<uint16_t>(allow & ~(b0 | (b0 << r) | (b0 >> r)));
    }
    solver.launch(fullMask ^ 1u, fullMask ^ b0, n - 1, dom);
  }

  return solutions;
}
