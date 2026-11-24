#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "nqueens.h"

namespace {

constexpr size_t kFundBufBytes = 65536;
uint8_t g_fundBuf[kFundBufBytes];
uint32_t g_boardBits[32];
size_t g_fundCount = 0;
size_t g_totalBuffered = 0;
size_t g_bufUsed = 0;
bool g_overflowed = false;
bool g_debugMode = false;
std::vector<std::vector<int>>* g_out = nullptr;

__attribute__((always_inline)) inline void pushPtrReserved(
    std::vector<std::vector<int>>& dst, const int* __restrict p, int n) {
  if (dst.size() == dst.capacity()) __builtin_unreachable();
  dst.emplace_back(p, p + n);
}

template <int N>
__attribute__((noinline)) void emitRecordFallback(
    std::vector<std::vector<int>>& dst, const uint8_t* __restrict raw, uint8_t mask) {
  asm volatile("");
  constexpr int last = N - 1;
  int s0[N], s1[N], s2[N], s3[N], s4[N], s5[N], s6[N], s7[N];
#pragma clang loop vectorize(disable) unroll(disable)
  for (int r = 0; r < N; ++r) {
    const int br = raw[r];
    s0[r] = br;
    s1[r] = last - br;
    s2[last - r] = br;
    s3[last - r] = last - br;
    s4[br] = r;
  }
#pragma clang loop vectorize(disable) unroll(disable)
  for (int r = 0; r < N; ++r) {
    const int tr = s4[r];
    s5[r] = last - tr;
    s6[last - r] = tr;
    s7[last - r] = last - tr;
  }
  if (mask & 0x01u) dst.emplace_back(s0, s0 + N);
  if (mask & 0x02u) dst.emplace_back(s1, s1 + N);
  if (mask & 0x04u) dst.emplace_back(s2, s2 + N);
  if (mask & 0x08u) dst.emplace_back(s3, s3 + N);
  if (mask & 0x10u) dst.emplace_back(s4, s4 + N);
  if (mask & 0x20u) dst.emplace_back(s5, s5 + N);
  if (mask & 0x40u) dst.emplace_back(s6, s6 + N);
  if (mask & 0x80u) dst.emplace_back(s7, s7 + N);
}

template <int N>
__attribute__((noinline)) void emitBatchFixed(
    std::vector<std::vector<int>>& dst, const uint8_t* __restrict buf, size_t count) {
  asm volatile("");
  constexpr int last = N - 1;
  constexpr size_t stride = static_cast<size_t>(N) + 1;

  for (size_t i = 0; i < count; ++i) {
    const uint8_t* __restrict rec = buf + i * stride;
    const uint8_t mask = rec[0];
    const uint8_t* __restrict raw = rec + 1;

    if (__builtin_expect(mask == 0xFFu, 1)) {
      int s0[N], s1[N], s2[N], s3[N];
      int s4[N], s5[N], s6[N], s7[N];
#pragma clang loop unroll(full)
      for (int r = 0; r < N; ++r) {
        const int br = raw[r];
        s0[r] = br;
        s1[r] = last - br;
        s2[last - r] = br;
        s3[last - r] = last - br;
        s4[br] = r;
      }
#pragma clang loop unroll(full)
      for (int r = 0; r < N; ++r) {
        const int tr = s4[r];
        s5[r] = last - tr;
        s6[last - r] = tr;
        s7[last - r] = last - tr;
      }
      pushPtrReserved(dst, s0, N);
      pushPtrReserved(dst, s1, N);
      pushPtrReserved(dst, s2, N);
      pushPtrReserved(dst, s3, N);
      pushPtrReserved(dst, s4, N);
      pushPtrReserved(dst, s5, N);
      pushPtrReserved(dst, s6, N);
      pushPtrReserved(dst, s7, N);
    } else {
      emitRecordFallback<N>(dst, raw, mask);
    }
  }
}

template <int N>
__attribute__((always_inline)) inline void storeRecordFixed(const uint8_t* b, uint8_t mask, size_t cnt) {
  constexpr size_t recBytes = static_cast<size_t>(N) + 1;
  if (__builtin_expect(!g_overflowed && g_bufUsed + recBytes <= kFundBufBytes, 1)) {
    g_fundBuf[g_bufUsed] = mask;
    std::memcpy(g_fundBuf + g_bufUsed + 1, b, static_cast<size_t>(N));
    g_bufUsed += recBytes;
    ++g_fundCount;
    g_totalBuffered += cnt;
  } else {
    g_overflowed = true;
    emitRecordFallback<N>(*g_out, b, mask);
  }
  if (__builtin_expect(g_debugMode, 0)) {
    std::vector<std::pair<int, int>> qs;
    qs.reserve(static_cast<size_t>(N));
    for (int r = 0; r < N; ++r) qs.emplace_back(r, static_cast<int>(b[r]));
    snapshot(qs, "fundamental #" + std::to_string(g_fundCount) + " (x" + std::to_string(cnt) + ")");
  }
}

template <int N>
__attribute__((noinline)) void recordCase1() {
  asm volatile("");
  uint8_t b[N];
  uint8_t t[N];
#pragma clang loop unroll(full)
  for (int r = 0; r < N; ++r) {
    const uint8_t c = static_cast<uint8_t>(__builtin_ctz(g_boardBits[r]));
    b[r] = c;
    t[c] = static_cast<uint8_t>(r);
  }
  const int cmp = std::memcmp(b, t, static_cast<size_t>(N));
  if (cmp > 0) return;
  storeRecordFixed<N>(b, (cmp == 0) ? 0x0Fu : 0xFFu, (cmp == 0) ? 4 : 8);
}

template <int N>
__attribute__((noinline)) void recordCase2(int c0) {
  asm volatile("");
  constexpr int last = N - 1;
  uint8_t b[N];
#pragma clang loop unroll(full)
  for (int r = 0; r < N; ++r) {
    b[r] = static_cast<uint8_t>(__builtin_ctz(g_boardBits[r]));
  }
  if (b[c0] != static_cast<uint8_t>(last) &&
      b[last - c0] != 0u &&
      b[last] != static_cast<uint8_t>(last - c0)) {
    storeRecordFixed<N>(b, 0xFFu, 8);
    return;
  }

  uint8_t t[N];
#pragma clang loop unroll(full)
  for (int r = 0; r < N; ++r) t[b[r]] = static_cast<uint8_t>(r);

  uint8_t sym[8][N];
#pragma clang loop vectorize(disable) unroll(disable)
  for (int r = 0; r < N; ++r) {
    const uint8_t br = b[r];
    const uint8_t tr = t[r];
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
  if (sym[3][0] == c0u && std::memcmp(sym[3], sym[0], static_cast<size_t>(N)) < 0) return;
  if (sym[5][0] == c0u && std::memcmp(sym[5], sym[0], static_cast<size_t>(N)) < 0) return;
  if (sym[6][0] == c0u && std::memcmp(sym[6], sym[0], static_cast<size_t>(N)) < 0) return;

  uint8_t mask = 1u;
  size_t cnt = 1;
  for (int k = 1; k < 8; ++k) {
    bool dup = false;
    for (int j = 0; j < k; ++j) {
      if ((mask & (1u << j)) && std::memcmp(sym[k], sym[j], static_cast<size_t>(N)) == 0) {
        dup = true;
        break;
      }
    }
    if (!dup) {
      mask |= static_cast<uint8_t>(1u << k);
      ++cnt;
    }
  }
  storeRecordFixed<N>(b, mask, cnt);
}

template <int N, int C0, int ROW>
constexpr uint32_t computeRowAllow() {
  constexpr uint32_t fullMask = (1u << N) - 1u;
  constexpr int last = N - 1;
  constexpr uint32_t edgeMask = 1u | (1u << last);
  if constexpr (ROW == last) {
    return ((1u << (N - C0)) - 1u) & ~((1u << (C0 + 1)) - 1u);
  } else {
    uint32_t mask = fullMask;
    if constexpr (ROW < C0 || ROW >= N - C0) mask &= ~edgeMask;
    if constexpr (ROW == C0) mask &= ~1u;
    if constexpr (ROW == last - C0) mask &= ~(1u << last);
    if constexpr (C0 + 1 == last - C0) {
      constexpr int forcedCol = C0 + 1;
      constexpr int dist = last - ROW;
      mask &= ~(1u << forcedCol);
      if constexpr (forcedCol - dist >= 0) mask &= ~(1u << (forcedCol - dist));
      if constexpr (forcedCol + dist < N) mask &= ~(1u << (forcedCol + dist));
    }
    return mask;
  }
}

template <int N, int ROW>
__attribute__((always_inline)) inline void dfsCase1(
    uint32_t avail, uint32_t cols, uint32_t d1, uint32_t d2, int c1) {
  constexpr uint32_t fullMask = (1u << N) - 1u;
  constexpr int last = N - 1;
  constexpr int secondLast = N - 2;

  if constexpr (ROW == secondLast) {
    while (avail) {
      const uint32_t bit = avail & (~avail + 1u);
      avail ^= bit;
      const uint32_t lastAvail = fullMask & ~((cols | bit) | ((d1 | bit) << 1) | ((d2 | bit) >> 1));
      if (lastAvail) {
        g_boardBits[ROW] = bit;
        g_boardBits[last] = lastAvail;
        recordCase1<N>();
      }
    }
  } else {
    constexpr int nextRow = ROW + 1;
    const uint32_t allowNext = (nextRow < c1) ? (fullMask & ~2u) : fullMask;
    while (avail) {
      const uint32_t bit = avail & (~avail + 1u);
      avail ^= bit;
      const uint32_t nextCols = cols | bit;
      const uint32_t nextD1 = (d1 | bit) << 1;
      const uint32_t nextD2 = (d2 | bit) >> 1;
      const uint32_t nextAvail = allowNext & ~(nextCols | nextD1 | nextD2);
      if (nextAvail) {
        g_boardBits[ROW] = bit;
        dfsCase1<N, nextRow>(nextAvail, nextCols, nextD1, nextD2, c1);
      }
    }
  }
}

template <int N>
__attribute__((noinline)) void runCase1All() {
  asm volatile("");
  constexpr uint32_t fullMask = (1u << N) - 1u;
  g_boardBits[0] = 1u;
  for (int c1 = 2; c1 < N; ++c1) {
    const uint32_t bit1 = 1u << c1;
    g_boardBits[1] = bit1;
    const uint32_t cols2 = 1u | bit1;
    const uint32_t d1_2 = (2u | bit1) << 1;
    const uint32_t d2_2 = bit1 >> 1;
    const uint32_t allow2 = (2 < c1) ? (fullMask & ~2u) : fullMask;
    const uint32_t avail2 = allow2 & ~(cols2 | d1_2 | d2_2);
    if (avail2) {
      dfsCase1<N, 2>(avail2, cols2, d1_2, d2_2, c1);
    }
  }
}

template <int N, int C0, int ROW>
__attribute__((always_inline)) inline void dfsCase2(
    uint32_t avail, uint32_t cols, uint32_t d1, uint32_t d2) {
  constexpr int last = N - 1;
  constexpr int secondLast = N - 2;
  constexpr uint32_t rightBit = 1u << last;
  constexpr uint32_t lastMask = ((1u << (N - C0)) - 1u) & ~((1u << (C0 + 1)) - 1u);
  constexpr bool multiLast = (C0 + 1 < last - C0);

  if constexpr (ROW == secondLast) {
    constexpr uint32_t allowLast = computeRowAllow<N, C0, last>();
    while (avail) {
      const uint32_t bit = avail & (~avail + 1u);
      avail ^= bit;
      const uint32_t lastAvail = allowLast & ~((cols | bit) | ((d1 | bit) << 1) | ((d2 | bit) >> 1));
      if (lastAvail) {
        g_boardBits[ROW] = bit;
        g_boardBits[last] = lastAvail;
        recordCase2<N>(C0);
      }
    }
  } else {
    constexpr int nextRow = ROW + 1;
    constexpr uint32_t allowNext = computeRowAllow<N, C0, nextRow>();
    while (avail) {
      const uint32_t bit = avail & (~avail + 1u);
      avail ^= bit;
      const uint32_t nextCols = cols | bit;
      if constexpr (multiLast) {
        if ((nextCols & lastMask) == lastMask) continue;
      }
      const uint32_t nextD1 = (d1 | bit) << 1;
      const uint32_t nextD2 = (d2 | bit) >> 1;
      uint32_t nextAvail = allowNext & ~(nextCols | nextD1 | nextD2);
      if constexpr (nextRow == N - 2 - C0) {
        if (!(nextCols & rightBit)) nextAvail &= rightBit;
      } else if constexpr (nextRow == N - 1 - C0) {
        if (!(nextCols & 1u)) nextAvail &= 1u;
      }
      if (nextAvail) {
        g_boardBits[ROW] = bit;
        dfsCase2<N, C0, nextRow>(nextAvail, nextCols, nextD1, nextD2);
      }
    }
  }
}

// Each C0 subtree is an isolated ~2-4 KB WASM function so V8 compiles each in < 0.3 ms.
template <int N, int C0>
__attribute__((noinline)) void runOneC0() {
  asm volatile("");
  constexpr int last = N - 1;
  constexpr uint32_t rightBit = 1u << last;
  constexpr uint32_t bit0 = 1u << C0;
  g_boardBits[0] = bit0;

  constexpr uint32_t cols1 = bit0;
  constexpr uint32_t d1_1 = bit0 << 1;
  constexpr uint32_t d2_1 = bit0 >> 1;
  constexpr uint32_t allow1 = computeRowAllow<N, C0, 1>();
  uint32_t avail1 = allow1 & ~(cols1 | d1_1 | d2_1);
  if constexpr (1 == N - 2 - C0) {
    avail1 &= rightBit;
  }
  if (avail1) {
    dfsCase2<N, C0, 1>(avail1, cols1, d1_1, d2_1);
  }
}

template <int N, int C0, int MaxC0>
inline void runCase2All() {
  if constexpr (C0 <= MaxC0) {
    runOneC0<N, C0>();
    runCase2All<N, C0 + 1, MaxC0>();
  }
}

template <int N>
__attribute__((noinline)) std::vector<std::vector<int>> solveFixed() {
  asm volatile("");
  std::vector<std::vector<int>> solutions;

  g_fundCount = 0;
  g_totalBuffered = 0;
  g_bufUsed = 0;
  g_overflowed = false;
  g_debugMode = snapshotsEnabled();
  g_out = &solutions;

  runCase1All<N>();
  runCase2All<N, 1, ((N - 2) >> 1)>();

  if (g_totalBuffered > 0) {
    solutions.reserve(solutions.size() + g_totalBuffered);
    emitBatchFixed<N>(solutions, g_fundBuf, g_fundCount);
  }

  return solutions;
}

__attribute__((noinline)) std::vector<std::vector<int>> solveSmall(int n) {
  asm volatile("");
  std::vector<std::vector<int>> solutions;
  const uint32_t fullMask = (n == 32) ? ~0u : ((1u << n) - 1u);
  const int lastRow = n - 1;
  const int secondLast = n - 2;
  uint8_t board[32];
  size_t halfCount = 0;
  const bool dbg = snapshotsEnabled();

  auto dfs = [&](auto& self, int row, uint32_t cols, uint32_t d1, uint32_t d2) -> void {
    uint32_t avail = fullMask & ~(cols | d1 | d2);
    if (row == secondLast) {
      while (avail) {
        const uint32_t bit = avail & (~avail + 1u);
        avail ^= bit;
        const uint32_t lastAvail = fullMask & ~((cols | bit) | ((d1 | bit) << 1) | ((d2 | bit) >> 1));
        if (lastAvail) {
          board[row] = static_cast<uint8_t>(__builtin_ctz(bit));
          board[lastRow] = static_cast<uint8_t>(__builtin_ctz(lastAvail));
          if (halfCount * static_cast<size_t>(n) + static_cast<size_t>(n) <= kFundBufBytes) {
            std::memcpy(g_fundBuf + halfCount * static_cast<size_t>(n), board, static_cast<size_t>(n));
            ++halfCount;
          } else {
            int s1[32], s2[32];
            for (int r = 0; r < n; ++r) {
              s1[r] = board[r];
              s2[r] = lastRow - board[r];
            }
            solutions.emplace_back(s1, s1 + n);
            solutions.emplace_back(s2, s2 + n);
          }
          if (__builtin_expect(dbg, 0)) {
            std::vector<std::pair<int, int>> qs;
            for (int r = 0; r < n; ++r) qs.emplace_back(r, static_cast<int>(board[r]));
            snapshot(qs, "half #" + std::to_string(halfCount));
          }
        }
      }
      return;
    }
    while (avail) {
      const uint32_t bit = avail & (~avail + 1u);
      avail ^= bit;
      board[row] = static_cast<uint8_t>(__builtin_ctz(bit));
      self(self, row + 1, cols | bit, (d1 | bit) << 1, (d2 | bit) >> 1);
    }
  };

  const int half = n >> 1;
  for (int c0 = 0; c0 < half; ++c0) {
    const uint32_t bit0 = 1u << c0;
    board[0] = static_cast<uint8_t>(c0);
    dfs(dfs, 1, bit0, bit0 << 1, bit0 >> 1);
  }
  if (n & 1) {
    const int mid = half;
    const uint32_t bit0 = 1u << mid;
    board[0] = static_cast<uint8_t>(mid);
    for (int c1 = 0; c1 <= mid - 2; ++c1) {
      const uint32_t bit1 = 1u << c1;
      board[1] = static_cast<uint8_t>(c1);
      dfs(dfs, 2, bit0 | bit1, ((bit0 << 1) | bit1) << 1, ((bit0 >> 1) | bit1) >> 1);
    }
  }

  if (halfCount > 0) {
    solutions.reserve(solutions.size() + halfCount * 2);
    int s1[32], s2[32];
    for (size_t i = 0; i < halfCount; ++i) {
      const uint8_t* src = g_fundBuf + i * static_cast<size_t>(n);
      for (int r = 0; r < n; ++r) {
        s1[r] = src[r];
        s2[r] = lastRow - src[r];
      }
      pushPtrReserved(solutions, s1, n);
      pushPtrReserved(solutions, s2, n);
    }
  }
  return solutions;
}

}  // namespace

std::vector<std::vector<int>> solveNQueens(int n) {
  if (n <= 0 || n > 32) return {};
  switch (n) {
    case 1: return {{0}};
    case 2:
    case 3: return {};
    case 10: return solveFixed<10>();
    case 11: return solveFixed<11>();
    case 12: return solveFixed<12>();
    default: return solveSmall(n);
  }
}
