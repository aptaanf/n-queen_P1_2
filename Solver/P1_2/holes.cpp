#include <algorithm>
#include <array>
#include <cstdint>

#include "nqueens.h"

namespace {

using Mask = std::uint32_t;
using PackedBoard = std::uint64_t;
using Solutions = std::vector<std::vector<int>>;
constexpr int maxBoardSize = 12;
using Lines = std::array<Mask, maxBoardSize>;
using Board = std::array<int, maxBoardSize>;

struct Direction {
  PackedBoard score;
  bool transposed;
  bool reversed;
  bool mirrorSymmetric;
};

Direction chooseDirection(int n, const Lines& rows, const Lines& columns) {
  Direction best{};
  for (bool transposed : {false, true}) {
    const Lines& lines = transposed ? columns : rows;
    const Lines& cross = transposed ? rows : columns;
    bool mirrorSymmetric = true;
    for (int i = 0; i < n / 2; ++i) {
      mirrorSymmetric &= cross[i] == cross[n - 1 - i];
    }

    PackedBoard forward = 0;
    PackedBoard backward = 0;
    for (int i = 0; i < n; ++i) {
      const int choices = __builtin_popcount(lines[i]);
      forward = (forward << 4) | choices;
      backward |= PackedBoard{static_cast<unsigned>(choices)} << (4 * i);
    }
    const Direction candidate{std::min(forward, backward), transposed,
                              backward < forward, mirrorSymmetric};
    if (best.score == 0 || candidate.mirrorSymmetric > best.mirrorSymmetric ||
        (candidate.mirrorSymmetric == best.mirrorSymmetric && candidate.score < best.score)) {
      best = candidate;
    }
  }
  return best;
}

template <int Bytes>
struct PackedPath {
  std::array<std::uint8_t, Bytes> bytes;

  void store(PackedBoard path) {
    for (int i = 0; i < Bytes; ++i) bytes[i] = static_cast<std::uint8_t>(path >> (8 * i));
  }

  PackedBoard load() const {
    PackedBoard path = 0;
    for (int i = Bytes - 1; i >= 0; --i) path = (path << 8) | bytes[i];
    return path;
  }
};

template <class Code>
Code* search(const Mask* row, const Mask* last, Mask choices, Mask used,
             Mask left, Mask right, PackedBoard path, Code* out) {
  if (row == last) {
    if (choices) (out++)->store(path);
    return out;
  }
  const Mask open = row[1] & ~(used | (left >> 1) | (right << 1));
  if (!open) return out;
  path <<= 4;
  while (choices) {
    const Mask q = choices & (0u - choices);
    choices ^= q;
    const Mask next = open & ~((7u * q) >> 1);
    if (!next) continue;
    const auto code = path | __builtin_ctz(q);
    if (row + 1 == last) (out++)->store(code);
    else out = search(row + 1, last, next, used | q, (left | q) >> 1, (right | q) << 1,
                      code, out);
  }
  return out;
}

void unpack(int n, int top, PackedBoard path, Board& board) {
  board[0] = top;
  Mask missing = ((1u << n) - 1) ^ (1u << top);
  for (int row = n - 2; row >= 1; --row) {
    board[row] = path & 15;
    missing ^= 1u << board[row];
    path >>= 4;
  }
  board[n - 1] = __builtin_ctz(missing);
}

void transformBoard(int n, const Board& board, int image, Board& mapped) {
  for (int i = 0; i < n; ++i) {
    const int row = (image & 4) ? board[i] : i;
    const int col = (image & 4) ? i : board[i];
    mapped[(image & 2) ? n - 1 - row : row] = (image & 1) ? n - 1 - col : col;
  }
}

int composeImages(int outer, int inner) {
  if (outer & 4) inner = (inner & 4) | ((inner & 1) << 1) | ((inner & 2) >> 1);
  return outer ^ inner;
}

unsigned uniqueImages(int n, const Board& board) {
  const int last = n - 1;
  int left = 0, right = 0;
  for (int row = 0; row < n; ++row) {
    if (board[row] == 0) left = row;
    if (board[row] == last) right = row;
  }
  const int top = board[0], bottom = board[last];
  const int first[] = {top, last - top, bottom, last - bottom,
                       left, last - left, right, last - right};
  unsigned images = 255;
  for (int i = 1; i < 8; ++i) {
    if (first[i] != top) continue;
    Board mapped;
    transformBoard(n, board, i, mapped);
    const auto diff = std::mismatch(board.begin(), board.begin() + n, mapped.begin());
    if (diff.first != board.begin() + n) {
      if (*diff.second < *diff.first) return 0;
    } else {
      for (int j = 0; j < 8; ++j) {
        const int equivalent = composeImages(j, i);
        if (equivalent > j) images &= ~(1u << equivalent);
      }
    }
  }
  return images;
}

struct Hole {
  std::uint8_t row;
  std::uint8_t column;
};

struct SparseHoles {
  std::array<Hole, maxBoardSize> cells;
  int count = 0;
};

unsigned legalImages(int n, const Board& board, unsigned images, const SparseHoles& holes) {
  const int last = n - 1;
  for (int i = 0; i < holes.count; ++i) {
    const int r = holes.cells[i].row, c = holes.cells[i].column;
    const int oppositeRow = last - r, oppositeCol = last - c;
    const unsigned blocked =
        (unsigned(board[r] == c) << 0) | (unsigned(board[r] == oppositeCol) << 1) |
        (unsigned(board[oppositeRow] == c) << 2) | (unsigned(board[oppositeRow] == oppositeCol) << 3) |
        (unsigned(board[c] == r) << 4) | (unsigned(board[oppositeCol] == r) << 5) |
        (unsigned(board[c] == oppositeRow) << 6) | (unsigned(board[oppositeCol] == oppositeRow) << 7);
    images &= ~blocked;
    if (!images) break;
  }
  return images;
}

void append(Solutions& solutions, const Board& board, int n) {
  solutions.emplace_back(board.begin(), board.begin() + n);
  NQ_SNAPSHOT(nq::queensOf(solutions.back()), "solution " + std::to_string(solutions.size()));
}

Solutions solveSmall(int n, const Lines& rows) {
  std::array<PackedPath<4>, 92> paths;
  auto* end = search(rows.data(), &rows[n - 1], rows[0], 0, 0, 0, 0, paths.data());
  Solutions solutions;
  solutions.reserve(end - paths.data());
  Board board;
  for (auto* p = paths.data(); p != end; ++p) {
    const PackedBoard path = p->load();
    const int top = static_cast<int>(path >> (4 * (n - 2)));
    unpack(n, top, path, board);
    append(solutions, board, n);
  }
  return solutions;
}

template <bool squareSymmetry>
Solutions solveLarge(int n, Lines lines, const Direction& direction, const SparseHoles& holes) {
  if (direction.reversed) std::reverse(lines.begin(), lines.begin() + n);
  const int last = n - 1;
  const Mask full = (1u << n) - 1;
  const Mask edges = 1u | (1u << last);
  const Mask half = (1u << (n / 2)) - 1;
  constexpr int maxSolutions = 14200;
  constexpr int capacity = squareSymmetry ? maxSolutions / 2 : maxSolutions;
  std::array<PackedPath<5>, capacity> paths;
  std::array<std::uint8_t, squareSymmetry ? capacity : 0> images;
  std::array<int, maxBoardSize> ends{};
  int written = 0, count = 0;
  Board board;

  for (int top = 0; top < n; ++top) {
    ends[top] = written;
    const Mask first = 1u << top;
    if (!(lines[0] & first)) continue;
    if (direction.mirrorSymmetric && 2 * top > last) break;
    auto* begin = paths.data() + written;
    auto* end = begin;
    const Mask left = first >> 1, right = first << 1;
    Mask second = lines[1] & ~(first | left | right);
    if constexpr (squareSymmetry) {
      if (2 * top == last) break;
      Lines open = lines;
      open[last] &= (full >> top) & (full << top);
      for (int r = 1; r < top; ++r) {
        open[r] &= ~edges;
        open[last - r] &= ~edges;
      }
      second &= open[1];
      if (top == 0) {
        int until = 1;
        for (; second; second &= second - 1) {
          const int col = __builtin_ctz(second);
          while (until < col) open[++until] &= ~Mask{2};
          const Mask q = 1u << col;
          const Mask columns = first | q;
          const Mask nextLeft = (left | q) >> 1, nextRight = (right | q) << 1;
          const Mask next = open[2] & ~(columns | nextLeft | nextRight);
          if (next) end = search(&open[2], &open[last], next, columns, nextLeft, nextRight, col, end);
        }
      } else {
        end = search(&open[1], &open[last], second, first, left, right, 0, end);
      }
      for (auto* path = begin; path != end; ++path) {
        unsigned orbit = 255;
        if (top || holes.count) {
          unpack(n, top, path->load(), board);
          if (top) orbit = uniqueImages(n, board);
          if (orbit && holes.count) orbit = legalImages(n, board, orbit, holes);
        }
        if (!orbit) continue;
        paths[written] = *path;
        images[written++] = static_cast<std::uint8_t>(orbit);
        count += __builtin_popcount(orbit);
      }
    } else {
      if (direction.mirrorSymmetric && 2 * top == last) second &= half;
      end = search(&lines[1], &lines[last], second, first, left, right, 0, end);
      const int found = static_cast<int>(end - begin);
      written += found;
      count += direction.mirrorSymmetric ? 2 * found : found;
    }
    ends[top] = written;
  }

  Solutions solutions;
  solutions.reserve(count);
  int index = 0;
  for (int top = 0; index < written; ++top) {
    for (; index < ends[top]; ++index) {
      unpack(n, top, paths[index].load(), board);
      const unsigned orbit = squareSymmetry ? images[index] : direction.mirrorSymmetric ? 3u : 1u;
      for (unsigned rest = orbit; rest; rest &= rest - 1) {
        int image = __builtin_ctz(rest);
        if constexpr (!squareSymmetry) {
          if (direction.reversed) image ^= 2;
          if (direction.transposed) image = composeImages(4, image);
        }
        if (!image) {
          append(solutions, board, n);
        } else {
          Board mapped;
          transformBoard(n, board, image, mapped);
          append(solutions, mapped, n);
        }
      }
    }
  }
  return solutions;
}

}

std::vector<std::vector<int>> solveHoles(int n, const std::vector<std::string>& grid) {
  if (n < 4 || n > maxBoardSize) return {};

  Lines rows{};
  Lines columns{};
  for (int row = 0; row < n; ++row) {
    for (int column = 0; column < n; ++column) {
      if (grid[row][column] == '.') {
        rows[row] |= Mask{1} << column;
        columns[column] |= Mask{1} << row;
      }
    }
    if (rows[row] == 0) return {};
  }
  for (int column = 0; column < n; ++column) {
    if (columns[column] == 0) return {};
  }

  if (n <= 8) return solveSmall(n, rows);
  Direction direction = chooseDirection(n, rows, columns);
  bool squareSymmetry = direction.mirrorSymmetric && rows == columns;
  SparseHoles holes;
  const Mask full = (1u << n) - 1;
  if (!squareSymmetry) {
    int blocked = 0;
    for (int r = 0; r < n; ++r) blocked += __builtin_popcount(full ^ rows[r]);
    if (blocked <= n) {
      for (int r = 0; r < n; ++r) {
        for (Mask bits = full ^ rows[r]; bits; bits &= bits - 1)
          holes.cells[holes.count++] = {static_cast<std::uint8_t>(r),
                                        static_cast<std::uint8_t>(__builtin_ctz(bits))};
      }
      rows.fill(full);
      squareSymmetry = true;
      direction = {0, false, false, true};
    }
  }
  const Lines& lines = direction.transposed ? columns : rows;
  if (squareSymmetry) return solveLarge<true>(n, lines, direction, holes);
  return solveLarge<false>(n, lines, direction, holes);
}
