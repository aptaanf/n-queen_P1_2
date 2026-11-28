#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>
#include "nqueens.h"

namespace {

struct BufState {
  uint32_t bits[12];
  size_t count, used, total;
  uint8_t* buf;
};

template <int N>
__attribute__((noinline)) void emitBatch(std::vector<std::vector<int>>& dst, const uint8_t* __restrict buf, size_t count) {
  const int last = N - 1;
  for (size_t i = 0; i < count; ++i) {
    const uint8_t mask = buf[i * (N + 1)];
    const uint8_t* __restrict raw = buf + i * (N + 1) + 1;
    int s0[N], s1[N], s2[N], s3[N], s4[N], s5[N], s6[N], s7[N];
#pragma clang loop unroll(full)
    for (int r = 0; r < N; ++r) {
      const int br = raw[r];
      s0[r] = br; s1[r] = last - br; s2[last - r] = br; s3[last - r] = last - br; s4[br] = r;
    }
#pragma clang loop unroll(full)
    for (int r = 0; r < N; ++r) {
      const int tr = s4[r];
      s5[r] = last - tr; s6[last - r] = tr; s7[last - r] = last - tr;
    }
    int* ps[8] = {s0, s1, s2, s3, s4, s5, s6, s7};
    if (__builtin_expect(mask == 0xFFu, 1)) {
      for (int k = 0; k < 8; ++k) { if (dst.size() == dst.capacity()) __builtin_unreachable(); dst.emplace_back(ps[k], ps[k] + N); }
    } else {
      for (int k = 0; k < 8; ++k) if (mask & (1u << k)) dst.emplace_back(ps[k], ps[k] + N);
    }
  }
}

template <int N>
void saveRec(BufState& st, const uint8_t* b, uint8_t mask, size_t cnt) {
  st.buf[st.used] = mask;
  std::memcpy(st.buf + st.used + 1, b, static_cast<size_t>(N));
  st.used += N + 1;
  ++st.count;
  st.total += cnt;
}

template <int N>
__attribute__((noinline)) void record1(BufState& st) {
  uint8_t b[N], t[N];
  for (int r = 0; r < N; ++r) { b[r] = static_cast<uint8_t>(__builtin_ctz(st.bits[r])); t[b[r]] = static_cast<uint8_t>(r); }
  const int cmp = std::memcmp(b, t, static_cast<size_t>(N));
  if (cmp <= 0) saveRec<N>(st, b, (cmp == 0) ? 0x0Fu : 0xFFu, (cmp == 0) ? 4u : 8u);
}

template <int N, int C0>
__attribute__((noinline)) void record2(BufState& st) {
  const int last = N - 1;
  uint8_t b[N], mask = 0xFFu;
  size_t cnt = 8u;
  for (int r = 0; r < N; ++r) b[r] = static_cast<uint8_t>(__builtin_ctz(st.bits[r]));
  if (b[C0] == static_cast<uint8_t>(last) || b[last - C0] == 0u || b[last] == static_cast<uint8_t>(last - C0)) {
    uint8_t t[N], sym[8][N];
    for (int r = 0; r < N; ++r) t[b[r]] = static_cast<uint8_t>(r);
    for (int r = 0; r < N; ++r) {
      const uint8_t br = b[r], tr = t[r];
      sym[0][r] = br; sym[1][r] = static_cast<uint8_t>(last - br); sym[2][last - r] = br; sym[3][last - r] = static_cast<uint8_t>(last - br);
      sym[4][r] = tr; sym[5][r] = static_cast<uint8_t>(last - tr); sym[6][last - r] = tr; sym[7][last - r] = static_cast<uint8_t>(last - tr);
    }
    const uint8_t c0u = static_cast<uint8_t>(C0);
    if ((sym[3][0] == c0u && std::memcmp(sym[3], sym[0], static_cast<size_t>(N)) < 0) ||
        (sym[5][0] == c0u && std::memcmp(sym[5], sym[0], static_cast<size_t>(N)) < 0) ||
        (sym[6][0] == c0u && std::memcmp(sym[6], sym[0], static_cast<size_t>(N)) < 0)) return;
    mask = 1u; cnt = 1u;
    for (int k = 1; k < 8; ++k) {
      bool dup = false;
      for (int j = 0; j < k; ++j) if ((mask & (1u << j)) && std::memcmp(sym[k], sym[j], static_cast<size_t>(N)) == 0) { dup = true; break; }
      if (!dup) { mask |= static_cast<uint8_t>(1u << k); ++cnt; }
    }
  }
  saveRec<N>(st, b, mask, cnt);
}

template <int N, int R>
__attribute__((always_inline)) inline void dfs1(uint32_t avail, uint32_t cols, uint32_t d1, uint32_t d2, int c1, BufState& st) {
  const uint32_t fullMask = (1u << N) - 1u;
  if constexpr (R == N - 2) {
    while (avail) {
      const uint32_t bit = avail & (~avail + 1u); avail ^= bit;
      if (const uint32_t la = fullMask & ~((cols | bit) | ((d1 | bit) << 1) | ((d2 | bit) >> 1))) { st.bits[R] = bit; st.bits[N - 1] = la; record1<N>(st); }
    }
  } else {
    const uint32_t allowNext = (R + 1 < c1) ? (fullMask & ~2u) : fullMask;
    while (avail) {
      const uint32_t bit = avail & (~avail + 1u); avail ^= bit;
      const uint32_t nc = cols | bit, nd1 = (d1 | bit) << 1, nd2 = (d2 | bit) >> 1;
      if (const uint32_t nextAvail = allowNext & ~(nc | nd1 | nd2)) { st.bits[R] = bit; dfs1<N, R + 1>(nextAvail, nc, nd1, nd2, c1, st); }
    }
  }
}

template <int N, int C0, int R>
__attribute__((always_inline)) inline void dfs2(uint32_t avail, uint32_t cols, uint32_t d1, uint32_t d2, const uint32_t* __restrict allow, BufState& st) {
  const int last = N - 1;
  const uint32_t rightBit = 1u << last, lastMask = ((1u << (N - C0)) - 1u) & ~((1u << (C0 + 1)) - 1u);
  if constexpr (R == N - 2) {
    while (avail) {
      const uint32_t bit = avail & (~avail + 1u); avail ^= bit;
      if (const uint32_t la = allow[last] & ~((cols | bit) | ((d1 | bit) << 1) | ((d2 | bit) >> 1))) { st.bits[R] = bit; st.bits[last] = la; record2<N, C0>(st); }
    }
  } else {
    const int nr = R + 1;
    const uint32_t allowNext = allow[nr];
    while (avail) {
      const uint32_t bit = avail & (~avail + 1u); avail ^= bit;
      const uint32_t nc = cols | bit;
      if constexpr (C0 + 1 < last - C0) if ((nc & lastMask) == lastMask) continue;
      const uint32_t nd1 = (d1 | bit) << 1, nd2 = (d2 | bit) >> 1;
      uint32_t nextAvail = allowNext & ~(nc | nd1 | nd2);
      if constexpr (nr == N - 2 - C0) { if (!(nc & rightBit)) nextAvail &= rightBit; }
      else if constexpr (nr == N - 1 - C0) { if (!(nc & 1u)) nextAvail &= 1u; }
      if (nextAvail) { st.bits[R] = bit; dfs2<N, C0, nr>(nextAvail, nc, nd1, nd2, allow, st); }
    }
  }
}

template <int N, int C0>
__attribute__((noinline)) void runOneC0(BufState& st) {
  const int last = N - 1;
  const uint32_t fullMask = (1u << N) - 1u, rightBit = 1u << last, edgeMask = 1u | rightBit, b0 = 1u << C0;
  uint32_t allow[N];
  for (int r = 1; r < last; ++r) {
    uint32_t m = fullMask;
    if (r < C0 || r >= N - C0) m &= ~edgeMask;
    if (r == C0) m &= ~1u;
    if (r == last - C0) m &= ~rightBit;
    if constexpr (C0 + 1 == last - C0) {
      const int fc = C0 + 1, dist = last - r;
      m &= ~(1u << fc);
      if (fc >= dist) m &= ~(1u << (fc - dist));
      if (fc + dist < N) m &= ~(1u << (fc + dist));
    }
    allow[r] = m;
  }
  allow[last] = ((1u << (N - C0)) - 1u) & ~((1u << (C0 + 1)) - 1u);
  st.bits[0] = b0;
  uint32_t av1 = allow[1] & ~(b0 | (b0 << 1) | (b0 >> 1));
  if constexpr (1 == N - 2 - C0) av1 &= rightBit;
  if (av1) dfs2<N, C0, 1>(av1, b0, b0 << 1, b0 >> 1, allow, st);
}

template <int N, int C0, int MaxC0>
inline void runCase2All(BufState& st) {
  if constexpr (C0 <= MaxC0) { runOneC0<N, C0>(st); runCase2All<N, C0 + 1, MaxC0>(st); }
}

template <int N>
std::vector<std::vector<int>> runN() {
  std::vector<std::vector<int>> solutions;
  uint8_t rawBuf[N == 12 ? 23500 : (N == 11 ? 4500 : 1200)];
  BufState st{{1u}, 0u, 0u, 0u, rawBuf};
  const uint32_t fullMask = (1u << N) - 1u;
  for (int c1 = 2; c1 < N; ++c1) {
    const uint32_t b1 = 1u << c1; st.bits[1] = b1;
    const uint32_t c2 = 1u | b1, d1 = (2u | b1) << 1, d2 = b1 >> 1;
    if (const uint32_t av2 = ((2 < c1) ? (fullMask & ~2u) : fullMask) & ~(c2 | d1 | d2)) dfs1<N, 2>(av2, c2, d1, d2, c1, st);
  }
  runCase2All<N, 1, ((N - 2) >> 1)>(st);
  solutions.reserve(st.total);
  emitBatch<N>(solutions, rawBuf, st.count);
  return solutions;
}

}

std::vector<std::vector<int>> solveNQueens(int n) {
  switch (n) {
    case 1: return {{0}};
    case 4: return runN<4>(); case 5: return runN<5>(); case 6: return runN<6>();
    case 7: return runN<7>(); case 8: return runN<8>(); case 9: return runN<9>();
    case 10: return runN<10>(); case 11: return runN<11>(); case 12: return runN<12>();
    default: return {};
  }
}
