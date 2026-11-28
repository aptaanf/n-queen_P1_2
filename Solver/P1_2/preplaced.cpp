#include "nqueens.h"

#include <algorithm>
#include <cstdint>

namespace {

using Mask = std::uint32_t;
using Board = std::uint64_t;
constexpr int maxSize = 12;

Mask attacks(int r, Mask sums, Mask differences) {
  return (sums >> r) | (differences >> (maxSize - 1 - r));
}

Board packedQueen(int r, Mask queen) {
  return static_cast<Board>(__builtin_ctz(queen)) << (4 * r);
}

void appendBoard(std::vector<std::vector<int>>& solutions, int n, Board board) {
  auto& columns = solutions.emplace_back(n);
  for (int& c : columns) {
    c = static_cast<int>(board & 15);
    board >>= 4;
  }
}

template <bool emit>
unsigned enumerate(Mask rows, Mask columns, Mask sums, Mask differences,
                   Board board, int n, std::vector<std::vector<int>>& answers) {
  if (!rows) {
    if constexpr (emit) appendBoard(answers, n, board);
    return 1;
  }

  int row = __builtin_ctz(rows);
  Mask choices = columns & ~attacks(row, sums, differences);
  const Mask otherRows = rows & (rows - 1);
  if (!otherRows) {
    if (!choices) return 0;
    if constexpr (emit) appendBoard(answers, n, board | packedQueen(row, choices));
    return 1;
  }
  int fewest = __builtin_popcount(choices);
  for (Mask other = otherRows; other && fewest > 1; other &= other - 1) {
    const int r = __builtin_ctz(other);
    const Mask legal = columns & ~attacks(r, sums, differences);
    const int size = __builtin_popcount(legal);
    if (size < fewest) {
      row = r;
      choices = legal;
      fewest = size;
    }
  }
  rows ^= 1u << row;
  unsigned count = 0;
  while (choices) {
    const Mask queen = choices & -choices;
    choices ^= queen;
    Board next = board;
    if constexpr (emit) next |= packedQueen(row, queen);
    count += enumerate<emit>(rows, columns ^ queen, sums | (queen << row),
                             differences | (queen << (maxSize - 1 - row)), next, n, answers);
  }
  return count;
}

char* writeNumber(char* out, unsigned number) {
  if (number >= 10) out = writeNumber(out, number / 10);
  *out++ = static_cast<char>('0' + number % 10);
  return out;
}

}

std::vector<std::vector<int>> solvePreplaced(
    int n, const std::vector<std::pair<int, int>>& preplaced) {
  if (n < 4 || n > maxSize) return {};
  Mask rows = (1u << n) - 1, columns = rows, sums = 0, differences = 0;
  Board board = 0;
  for (const auto& [r, c] : preplaced) {
    if (static_cast<unsigned>(r) >= static_cast<unsigned>(n) ||
        static_cast<unsigned>(c) >= static_cast<unsigned>(n)) {
      return {};
    }
    const Mask row = 1u << r, col = 1u << c;
    if (!(rows & row) || !(columns & col) || (attacks(r, sums, differences) & col)) return {};
    rows ^= row;
    columns ^= col;
    sums |= col << r;
    differences |= col << (maxSize - 1 - r);
    board |= static_cast<Board>(c) << (4 * r);
  }

  std::vector<std::vector<int>> solutions;
  const unsigned count = enumerate<false>(rows, columns, sums, differences, 0, n, solutions);
  if (!count) return solutions;
  solutions.reserve(count);
  enumerate<true>(rows, columns, sums, differences, board, n, solutions);

  if (snapshotsEnabled()) {
    constexpr char prefix[] = "solution ";
    char label[24];
    char* digits = std::copy_n(prefix, sizeof(prefix) - 1, label);
    for (unsigned i = 0; i < solutions.size(); ++i) {
      *writeNumber(digits, i + 1) = '\0';
      snapshot(nq::queensOf(solutions[i]), label);
    }
  }
  return solutions;
}
