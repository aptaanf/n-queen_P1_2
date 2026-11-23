#include <array>
#include <cstdint>
#include <string>

#include "nqueens.h"

namespace {

using Mask = std::uint32_t;
constexpr int maxBoardSize = 12;

void search(int n, int row, Mask columns, Mask downLeft, Mask downRight,
            const std::array<Mask, maxBoardSize>& allowed,
            std::array<int, maxBoardSize>& board,
            std::vector<std::vector<int>>& solutions) {
  if (row == n) {
    solutions.emplace_back(board.begin(), board.begin() + n);
    NQ_SNAPSHOT(nq::queensOf(solutions.back()), "solution " + std::to_string(solutions.size()));
    return;
  }

  Mask available = allowed[row] & ~(columns | downLeft | downRight);
  while (available != 0) {
    const int column = __builtin_ctz(available);
    const Mask queen = Mask{1} << column;
    available ^= queen;
    board[row] = column;
    search(n, row + 1, columns | queen, (downLeft | queen) >> 1,
           (downRight | queen) << 1, allowed, board, solutions);
  }
}

}  // namespace

std::vector<std::vector<int>> solvePreplaced(
    int n, const std::vector<std::pair<int, int>>& preplaced) {
  if (n < 4 || n > maxBoardSize) return {};

  // The contest bounds N, so search state can live in fixed-size buffers.
  std::array<int, maxBoardSize> board;
  board.fill(-1);
  const Mask fullRow = (Mask{1} << n) - 1;
  std::array<Mask, maxBoardSize> allowed;
  allowed.fill(fullRow);

  // Apply locked queens before searching, including attacks on earlier rows.
  for (const auto& [row, column] : preplaced) {
    const Mask queen = Mask{1} << column;
    if (board[row] != -1 || (allowed[row] & queen) == 0) return {};
    board[row] = column;
    allowed[row] = queen;

    for (int otherRow = 0; otherRow < n; ++otherRow) {
      if (otherRow == row) continue;
      const int distance = otherRow < row ? row - otherRow : otherRow - row;
      const Mask attacks = queen | (queen >> distance) | (queen << distance);
      allowed[otherRow] &= ~attacks;
      if (allowed[otherRow] == 0) return {};
    }
  }

  std::vector<std::vector<int>> solutions;
  search(n, 0, 0, 0, 0, allowed, board, solutions);
  return solutions;
}
