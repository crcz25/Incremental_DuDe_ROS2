#pragma once

#include <vector>

namespace inc_dude {

// Minimum-cost one-to-one assignment (Kuhn-Munkres, O(n^2 m)).
// `cost[i][j]` is the cost of assigning row i to column j; entries that are
// not finite mark infeasible pairs. The solver first maximizes the number of
// feasible pairs and then minimizes their total cost. Returns, for each row,
// the assigned column or -1.
std::vector<int>
solveAssignment(const std::vector<std::vector<double>> &cost);

}  // namespace inc_dude
