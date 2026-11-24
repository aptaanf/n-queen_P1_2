#include <algorithm>
#include <array>
#include <cstdint>

#include "nqueens.h"

namespace {

using Mask = std::uint32_t;
using PackedBoard = std::uint64_t;
constexpr int maxBoardSize = 12;

// An empty 12x12 board has 14,200 solutions, the maximum for N = 4..12.
// Holes only remove solutions. Four bits suffice for each column.
constexpr int maxSolutions = 14200;

PackedBoard* search(const Mask* row, const Mask* lastRow, Mask columns,
                    Mask downLeft, Mask downRight, PackedBoard board,
                    PackedBoard* output) {
  // Holes exclude placements in this row. They do not interrupt diagonal attacks.
  Mask available = *row & ~(columns | downLeft | downRight);
  const PackedBoard prefix = board << 4;
  if (row == lastRow) {
    // With N - 1 columns used, the final row has at most one candidate.
    if (available != 0) *output++ = prefix | __builtin_ctz(available);
    return output;
  }

  while (available != 0) {
    const int column = __builtin_ctz(available);
    const Mask queen = Mask{1} << column;
    available ^= queen;
    output = search(row + 1, lastRow, columns | queen, (downLeft | queen) >> 1,
                    (downRight | queen) << 1, prefix | column, output);
  }
  return output;
}

}  // namespace

std::vector<std::vector<int>> solveHoles(int n, const std::vector<std::string>& grid) {
  if (n < 4 || n > maxBoardSize) return {};

  std::array<Mask, maxBoardSize> openCells;
  Mask openColumns = 0;
  bool mirrorSymmetric = true;
  for (int row = 0; row < n; ++row) {
    Mask open = 0;
    Mask reflected = 0;
    for (int column = 0; column < n; ++column) {
      if (grid[row][column] == '.') {
        open |= Mask{1} << column;
        reflected |= Mask{1} << (n - 1 - column);
      }
    }
    if (open == 0) return {};
    openCells[row] = open;
    openColumns |= open;
    mirrorSymmetric &= open == reflected;
  }
  if (openColumns != (Mask{1} << n) - 1) return {};

  // Start from the more constrained edge, comparing successive row widths.
  bool reversed = false;
  for (int row = 0; row < n / 2; ++row) {
    const int top = __builtin_popcount(openCells[row]);
    const int bottom = __builtin_popcount(openCells[n - 1 - row]);
    if (top == bottom) continue;
    reversed = bottom < top;
    break;
  }
  if (reversed) std::reverse(openCells.begin(), openCells.begin() + n);

  std::array<PackedBoard, maxSolutions> packed;
  PackedBoard* end = packed.data();
  const Mask* lastRow = openCells.data() + n - 1;
  if (mirrorSymmetric) {
    const int middleColumn = n / 2;
    const Mask middle = Mask{1} << middleColumn;
    const Mask leftHalf = middle - 1;
    const bool useMiddle = (n % 2 != 0) && (openCells[0] & middle) != 0;
    openCells[0] &= leftHalf;
    end = search(openCells.data(), lastRow, 0, 0, 0, 0, end);
    if (useMiddle) {
      // With the first queen in the middle, the second picks the representative.
      openCells[1] &= leftHalf;
      end = search(openCells.data() + 1, lastRow, middle, middle >> 1,
                   middle << 1, middleColumn, end);
    }
  } else {
    end = search(openCells.data(), lastRow, 0, 0, 0, 0, end);
  }

  // Allocate the outer vector once, after the exact output size is known.
  std::vector<std::vector<int>> solutions;
  solutions.reserve((end - packed.data()) * (mirrorSymmetric ? 2 : 1));
  std::array<int, maxBoardSize> board;
  for (const PackedBoard* current = packed.data(); current != end; ++current) {
    PackedBoard code = *current;
    for (int row = n - 1; row >= 0; --row) {
      board[reversed ? n - 1 - row : row] = static_cast<int>(code & 15);
      code >>= 4;
    }
    solutions.emplace_back(board.begin(), board.begin() + n);
    NQ_SNAPSHOT(nq::queensOf(solutions.back()), "solution " + std::to_string(solutions.size()));
    if (mirrorSymmetric) {
      for (int row = 0; row < n; ++row) board[row] = n - 1 - board[row];
      solutions.emplace_back(board.begin(), board.begin() + n);
      NQ_SNAPSHOT(nq::queensOf(solutions.back()), "solution " + std::to_string(solutions.size()));
    }
  }
  return solutions;
}
