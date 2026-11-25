#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "nqueens.h"

namespace {

// Static scratch buffer for half-solutions during DFS (512 KB, covers up to N=13 without heap allocation).
// Automatically falls back to direct vector insertion if exceeded.
constexpr size_t kHalfBufBytes = 524288;
uint8_t g_halfBuf[kHalfBufBytes];

struct NormalContext {
  int n;
  int lastRow;
  int secondLastRow;
  uint32_t fullMask;
  uint8_t board[32];
  size_t halfCount;
  size_t bufUsed;
  bool overflowed;
  std::vector<std::vector<int>>* out;

  inline void recordSolution() {
    if (!overflowed && bufUsed + static_cast<size_t>(n) <= kHalfBufBytes) {
      std::memcpy(g_halfBuf + bufUsed, board, static_cast<size_t>(n));
      bufUsed += static_cast<size_t>(n);
      ++halfCount;
    } else {
      overflowed = true;
      auto& s1 = out->emplace_back(n);
      auto& s2 = out->emplace_back(n);
      int* p1 = s1.data();
      int* p2 = s2.data();
      const int lastCol = n - 1;
      for (int r = 0; r < n; ++r) {
        p1[r] = board[r];
        p2[r] = lastCol - board[r];
      }
    }
    if (snapshotsEnabled()) {
      std::vector<std::pair<int, int>> qs;
      qs.reserve(static_cast<size_t>(n));
      for (int r = 0; r < n; ++r) qs.emplace_back(r, static_cast<int>(board[r]));
      snapshot(qs, "half-solution #" + std::to_string(halfCount));
    }
  }

  void dfs(int row, uint32_t cols, uint32_t d1, uint32_t d2) {
    uint32_t avail = fullMask & ~(cols | d1 | d2);
    if (row == secondLastRow) {
      while (avail) {
        const uint32_t bit = avail & (~avail + 1u);
        avail ^= bit;
        board[row] = static_cast<uint8_t>(__builtin_ctz(bit));
        const uint32_t lastAvail = fullMask & ~((cols | bit) | ((d1 | bit) << 1) | ((d2 | bit) >> 1));
        if (lastAvail) {
          board[lastRow] = static_cast<uint8_t>(__builtin_ctz(lastAvail));
          recordSolution();
        }
      }
      return;
    }

    while (avail) {
      const uint32_t bit = avail & (~avail + 1u);
      avail ^= bit;
      board[row] = static_cast<uint8_t>(__builtin_ctz(bit));
      dfs(row + 1, cols | bit, (d1 | bit) << 1, (d2 | bit) >> 1);
    }
  }
};

}  // namespace

std::vector<std::vector<int>> solveNQueens(int n) {
  std::vector<std::vector<int>> solutions;
  if (n <= 0 || n > 32) return solutions;
  if (n == 1) {
    solutions.emplace_back(1, 0);
    return solutions;
  }
  if (n < 4) return solutions;

  NormalContext ctx{};
  ctx.n = n;
  ctx.lastRow = n - 1;
  ctx.secondLastRow = n - 2;
  ctx.fullMask = (n == 32) ? ~0u : ((1u << n) - 1u);
  ctx.halfCount = 0;
  ctx.bufUsed = 0;
  ctx.overflowed = false;
  ctx.out = &solutions;

  const int half = n >> 1;

  // Case A: Row 0 in left half [0 .. half - 1]. Mirror covers [n - half .. n - 1].
  for (int c0 = 0; c0 < half; ++c0) {
    const uint32_t bit0 = 1u << c0;
    ctx.board[0] = static_cast<uint8_t>(c0);
    ctx.dfs(1, bit0, bit0 << 1, bit0 >> 1);
  }

  // Case B (odd N only): Row 0 at center column `half`, Row 1 restricted to [0 .. half - 2].
  // Mirror keeps Row 0 at `half` and maps Row 1 to [half + 2 .. n - 1].
  if (n & 1) {
    const int mid = half;
    const uint32_t bit0 = 1u << mid;
    ctx.board[0] = static_cast<uint8_t>(mid);
    const uint32_t cols1 = bit0;
    const uint32_t d1_1 = bit0 << 1;
    const uint32_t d2_1 = bit0 >> 1;
    for (int c1 = 0; c1 <= mid - 2; ++c1) {
      const uint32_t bit1 = 1u << c1;
      ctx.board[1] = static_cast<uint8_t>(c1);
      ctx.dfs(2, cols1 | bit1, (d1_1 | bit1) << 1, (d2_1 | bit1) >> 1);
    }
  }

  // Materialize buffered half-solutions + their horizontal mirrors with exact capacity reservation.
  const size_t totalBuffered = ctx.halfCount * 2;
  if (totalBuffered > 0) {
    solutions.reserve(solutions.size() + totalBuffered);
    const int lastCol = n - 1;
    for (size_t i = 0; i < ctx.halfCount; ++i) {
      const uint8_t* src = g_halfBuf + i * static_cast<size_t>(n);

      auto& s1 = solutions.emplace_back(n);
      int* p1 = s1.data();
      for (int r = 0; r < n; ++r) p1[r] = src[r];

      auto& s2 = solutions.emplace_back(n);
      int* p2 = s2.data();
      for (int r = 0; r < n; ++r) p2[r] = lastCol - src[r];
    }
  }

  return solutions;
}
