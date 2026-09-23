#include "inc_dude/hungarian.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace inc_dude {

namespace {

// Rows <= columns. Classic potentials formulation; returns row -> column.
std::vector<int> solveRectangular(const std::vector<std::vector<double>> &a,
                                  int n, int m) {
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> u(n + 1, 0.0);
  std::vector<double> v(m + 1, 0.0);
  std::vector<int> p(m + 1, 0);    // column -> row (1-based, 0 = free)
  std::vector<int> way(m + 1, 0);
  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(m + 1, inf);
    std::vector<bool> used(m + 1, false);
    do {
      used[j0] = true;
      const int i0 = p[j0];
      double delta = inf;
      int j1 = 0;
      for (int j = 1; j <= m; ++j) {
        if (used[j]) {
          continue;
        }
        const double cur = a[i0 - 1][j - 1] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (int j = 0; j <= m; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do {
      const int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }
  std::vector<int> row_to_col(n, -1);
  for (int j = 1; j <= m; ++j) {
    if (p[j] != 0) {
      row_to_col[p[j] - 1] = j - 1;
    }
  }
  return row_to_col;
}

}  // namespace

std::vector<int>
solveAssignment(const std::vector<std::vector<double>> &cost) {
  const int rows = static_cast<int>(cost.size());
  if (rows == 0) {
    return {};
  }
  const int cols = static_cast<int>(cost.front().size());
  std::vector<int> result(rows, -1);
  if (cols == 0) {
    return result;
  }

  // Infeasible pairs get a cost larger than any complete feasible assignment
  // so that the solver never trades a feasible pair for a cheaper one.
  double max_cost = 0.0;
  for (const auto &row : cost) {
    for (double c : row) {
      if (std::isfinite(c)) {
        max_cost = std::max(max_cost, std::abs(c));
      }
    }
  }
  const double big = (max_cost + 1.0) * (rows + cols + 1);

  const bool transpose = rows > cols;
  const int n = transpose ? cols : rows;
  const int m = transpose ? rows : cols;
  std::vector<std::vector<double>> a(n, std::vector<double>(m, big));
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      const double c = std::isfinite(cost[i][j]) ? cost[i][j] : big;
      if (transpose) {
        a[j][i] = c;
      } else {
        a[i][j] = c;
      }
    }
  }

  const std::vector<int> assignment = solveRectangular(a, n, m);
  for (int r = 0; r < n; ++r) {
    const int c = assignment[r];
    if (c < 0) {
      continue;
    }
    const int row = transpose ? c : r;
    const int col = transpose ? r : c;
    if (std::isfinite(cost[row][col])) {
      result[row] = col;
    }
  }
  return result;
}

}  // namespace inc_dude
