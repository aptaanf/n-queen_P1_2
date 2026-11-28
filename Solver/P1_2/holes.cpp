#include "nqueens.h"
#include <cstdint>
namespace {
using Mask = unsigned;
using Code = std::uint64_t;
struct Search {
  int n, sideLimit = 0, cornerBound = 0, count = 0, size = 0;
  Mask full, open[12]{};
  unsigned char legal[12][12], stream[14200 * 8];
  Code previous = 0;
  template<bool symmetric, int row>
  void visit(Mask choices, Mask used, Mask left, Mask right, Code path) {
    if (symmetric && row > 1 && sideLimit < 0 && row <= cornerBound) choices &= ~2u;
    if (symmetric && row == sideLimit) {
      const Mask missing = (1u | (1u << (n-1))) & ~used;
      if (missing & (missing-1)) return;
      if (missing) choices &= missing;
    }
    if (!choices) return;
    if (row == n-1) {
      const unsigned last = __builtin_ctz(choices);
      Mask images = 1;
      if constexpr (symmetric) {
        images = 255;
        Code cells = (path << 4) | last;
        for (int r = n-1; r >= 0; --r, cells >>= 4) images &= legal[r][cells & 15];
        if (!images) return;
        stream[size++] = images;
      }
      const Code token = ((path >> 4) << 1) | ((path & 15) > last);
      Code delta = previous ^ token;
      while (delta >= 128) { stream[size++] = 128 | (delta & 127); delta >>= 7; }
      stream[size++] = delta;
      previous = token; count += __builtin_popcount(images);
    } else if constexpr (row < 11) {
      const Mask openNext = open[row+1] & ~(used | (left >> 1) | (right << 1));
      for (; choices; choices &= choices-1) {
        const Mask q = choices & -choices, next = openNext & ~((7u * q) >> 1);
        if constexpr (symmetric && row == 1) cornerBound = __builtin_ctz(q);
        if (next) visit<symmetric, row+1>(next, used | q, (left | q) >> 1, (right | q) << 1, (path << 4) | __builtin_ctz(q));
      }
    }
  }
};
}
std::vector<std::vector<int>> solveHoles(int n, const std::vector<std::string>& grid) {
  if (n < 4 || n > 12) return {};
  Search s;
  s.n = n; s.full = (1u << n) - 1;
  const int m = n-1;
  int holes = n*n;
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c) if (grid[r][c] == '.') s.open[r] |= 1u << c;
    if (!s.open[r]) return {};
    holes -= __builtin_popcount(s.open[r]);
  }
  const bool symmetric = n >= 9 && holes <= 2*n;
  if (!symmetric) s.visit<false, 0>(s.open[0], 0, 0, 0, 0);
  else for (int top = 0; 2*top < m; ++top) {
    s.sideLimit = top ? m-top : -1;
    for (int r = 0; r < n; ++r) for (int c = 0; c < n; ++c) {
      Mask images = 0;
      for (int i = 0; i < 8; ++i) {
        const int y = i & 4 ? c : r, x = i & 4 ? r : c;
        if (grid[i & 2 ? m-y : y][i & 1 ? m-x : x] == '.') images |= 1u << i;
      }
      if (top && r == m && c == m-top) images &= 0x53u;
      if (top && (c == 0 || c == m) && (r == top || r == m-top)) images &= 0x0fu;
      s.legal[r][c] = images;
    }
    for (int r = 0; r < n; ++r) s.open[r] = s.full;
    s.open[m] &= (s.full << top) & (s.full >> top);
    for (int r = 1; r < top; ++r) s.open[r] = s.open[m-r] = s.full & ~(1u | (1u << m));
    const Mask q = 1u << top;
    s.visit<true, 1>(s.open[1] & ~(q | (q >> 1) | (q << 1)), q, q >> 1, q << 1, top);
  }
  std::vector<std::vector<int>> result;
  result.reserve(s.count);
  Code token = 0;
  for (int pos = 0; pos < s.size;) {
    Mask images = symmetric ? s.stream[pos++] : 1;
    for (int shift = 0;; shift += 7) { auto byte = s.stream[pos++]; token ^= Code(byte & 127) << shift; if (byte < 128) break; }
    Code path = token >> 1;
    int layout[12], mapped[12]; Mask unused = s.full;
    for (int r = m-2; r >= 0; --r, path >>= 4) { layout[r] = path & 15; unused ^= 1u << layout[r]; }
    layout[m-1+(token & 1)] = __builtin_ctz(unused);
    layout[m-(token & 1)] = 31-__builtin_clz(unused);
    for (; images; images &= images-1) {
      const int i = __builtin_ctz(images);
      for (int r = 0; r < n; ++r) {
        const int c = layout[r], y = i & 4 ? c : r, x = i & 4 ? r : c;
        mapped[i & 2 ? m-y : y] = i & 1 ? m-x : x;
      }
      result.emplace_back(mapped, mapped+n);
      NQ_SNAPSHOT(nq::queensOf(result.back()), "solution " + std::to_string(result.size()));
    }
  }
  return result;
}
