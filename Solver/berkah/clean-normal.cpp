#include <cstdint>
#include <vector>

#include "nqueens.h"

namespace {

struct CleanNormalSolver {
  int n;
  int lastRow;
  uint32_t fullMask;
  uint8_t pos[32];  // pos[r] = kolom ratu pada baris r
  uint8_t* halfBuf;
  size_t halfBufCap;
  size_t halfCount;
  size_t bufUsed;
  std::vector<std::vector<int>>* out;

  void savePair() {
    if (bufUsed + static_cast<size_t>(n) <= halfBufCap) {
      uint8_t* dst = halfBuf + bufUsed;
      for (int r = 0; r < n; ++r) dst[r] = pos[r];
      bufUsed += static_cast<size_t>(n);
      ++halfCount;
    } else {
      int s1[32], s2[32];
      for (int r = 0; r < n; ++r) {
        s1[r] = pos[r];
        s2[r] = lastRow - pos[r];
      }
      out->emplace_back(s1, s1 + n);
      out->emplace_back(s2, s2 + n);
    }
  }

  // Backtracking dengan pengecekan konflik O(1) (bitmask) dan 1-step forward checking.
  void dfs(int row, uint32_t avail, uint32_t cols, uint32_t d1, uint32_t d2) {
    if (row == lastRow) {
      pos[row] = static_cast<uint8_t>(__builtin_ctz(avail));
      savePair();
      return;
    }

    while (avail) {
      const uint32_t bit = avail & (~avail + 1u);  // pilih kandidat kolom terendah
      avail ^= bit;

      const uint32_t nextCols = cols | bit;
      const uint32_t nextD1 = (d1 | bit) << 1;
      const uint32_t nextD2 = (d2 | bit) >> 1;
      const uint32_t nextAvail = fullMask & ~(nextCols | nextD1 | nextD2);

      // Safe pruning: hentikan cabang jika baris berikutnya sudah buntu.
      if (nextAvail) {
        pos[row] = static_cast<uint8_t>(__builtin_ctz(bit));  // choose
        dfs(row + 1, nextAvail, nextCols, nextD1, nextD2);    // explore
      }
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

  // Buffer lokal di stack (bukan variabel global) untuk menampung setengah solusi
  // sebelum dicerminkan. Cukup hingga N=12 (7.100 * 12 = 85.200 byte < 90.000 byte).
  constexpr size_t kHalfBufBytes = 90000;
  uint8_t halfBuf[kHalfBufBytes];

  CleanNormalSolver s{};
  s.n = n;
  s.lastRow = n - 1;
  s.fullMask = (n == 32) ? ~0u : ((1u << n) - 1u);
  s.halfBuf = halfBuf;
  s.halfBufCap = kHalfBufBytes;
  s.out = &solutions;

  const int half = n >> 1;

  // 1. Symmetry breaking (refleksi horizontal): baris 0 cukup telusuri separuh kiri.
  for (int c0 = 0; c0 < half; ++c0) {
    const uint32_t bit0 = 1u << c0;
    s.pos[0] = static_cast<uint8_t>(c0);
    const uint32_t avail1 = s.fullMask & ~(bit0 | (bit0 << 1) | (bit0 >> 1));
    if (avail1) {
      s.dfs(1, avail1, bit0, bit0 << 1, bit0 >> 1);
    }
  }

  // 2. Khusus N ganjil: ratu baris 0 di kolom tengah, baris 1 dibatasi sisi kiri.
  if (n & 1) {
    const int mid = half;
    const uint32_t bit0 = 1u << mid;
    s.pos[0] = static_cast<uint8_t>(mid);
    const uint32_t c1Cols = bit0;
    const uint32_t c1D1 = bit0 << 1;
    const uint32_t c1D2 = bit0 >> 1;
    for (int c1 = 0; c1 <= mid - 2; ++c1) {
      const uint32_t bit1 = 1u << c1;
      s.pos[1] = static_cast<uint8_t>(c1);
      const uint32_t cols2 = c1Cols | bit1;
      const uint32_t d1_2 = (c1D1 | bit1) << 1;
      const uint32_t d2_2 = (c1D2 | bit1) >> 1;
      const uint32_t avail2 = s.fullMask & ~(cols2 | d1_2 | d2_2);
      if (avail2) {
        s.dfs(2, avail2, cols2, d1_2, d2_2);
      }
    }
  }

  // Rekonstruksi pasangan solusi cermin dengan satu kali alokasi kapasitas tepat.
  if (s.halfCount > 0) {
    solutions.reserve(solutions.size() + s.halfCount * 2);
    int s1[32], s2[32];
    const int lastCol = n - 1;
    for (size_t i = 0; i < s.halfCount; ++i) {
      const uint8_t* src = halfBuf + i * static_cast<size_t>(n);
      for (int r = 0; r < n; ++r) {
        s1[r] = src[r];
        s2[r] = lastCol - src[r];
      }
      solutions.emplace_back(s1, s1 + n);
      solutions.emplace_back(s2, s2 + n);
    }
  }

  return solutions;
}
