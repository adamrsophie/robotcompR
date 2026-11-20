# Optimize a pose graph

Minimizes \\\sum e\_{ij}^\top \Omega\_{ij} e\_{ij}\\ over the node poses
with Gauss-Newton iterations and Levenberg-Marquardt damping. For SE(2)
graphs the residual is \\e\_{ij} = (R_z^\top(R_i^\top(t_j - t_i) - t_z),
\mathrm{wrap}(\theta_j - \theta_i - \theta_z))\\, analytic Jacobians
(Grisetti et al., 2010, "A tutorial on graph-based SLAM") and sparse
linear algebra from 'Matrix', and headings are wrapped after each
update. For SE(3) graphs the residual is \\e\_{ij} = \log(Z\_{ij}^{-1}
X_i^{-1} X_j)\\ (translation first), nodes are updated on the manifold
as \\X \exp(\delta)\\, and the Jacobians with respect to \\\delta\\ are
\\J_r(e)^{-1}\\ and \\-J_r(e)^{-1} \mathrm{Ad}(X_j^{-1} X_i)\\, taken in
the chart of the residual returned by the logarithm so that they stay
valid when a residual rotation is near \\\pi\\. Node `fixed` is held
constant to remove the gauge freedom; every node must be connected to it
through edges.

## Usage

``` r
pose_graph_optimize(
  graph,
  fixed = 1,
  max_iterations = 50,
  tol = 1e-12,
  gradient_tol = 1e-06
)
```

## Arguments

- graph:

  A `robot_pose_graph`.

- fixed:

  Index of the anchored node.

- max_iterations:

  Maximum iterations (0 only evaluates the graph).

- tol:

  Relative cost decrease treated as no progress.

- gradient_tol:

  Componentwise stationarity tolerance (see Details).

## Value

A list with the optimized `graph`, `cost` history, `iterations`,
`converged` (`TRUE` only for `status == "converged"`), `status` and
`gradient`, the largest scaled gradient component at the start of the
last iteration.

## Details

Convergence means stationarity, not merely a small change: the optimizer
stops with `status = "converged"` when every residual component is at
roundoff level (about 1000 machine epsilons of an arithmetic bound on
that component's rounding error, computed coordinatewise so a large
world offset along one axis does not excuse an error along another), or
every gradient component is negligible on its own scale: with \\\Omega =
U^\top U\\, each component \\g_k = \sum_e (U J_e)\_k^\top (U e)\\ must
satisfy \\\|g_k\| \le\\ `gradient_tol` \\\sum_e \|U J_e\|\_k^\top \|U
e\|\\, which holds only when its contributions cancel (contributions of
residuals at roundoff level are discounted first). A large irreducible
cost in one coordinate therefore cannot hide an error in another. A
graph with a single node is returned unchanged as converged. Other
outcomes are `"stalled"` (the relative decrease fell below `tol`, or no
damped step decreased the cost, while the gradient is not negligible),
`"max_iterations"`, `"linear_solve_failed"` and `"invalid_graph"` (nodes
not connected to `fixed`; the graph is returned unchanged with a
warning). A noisy graph converges to a positive cost.

## Examples

``` r
g <- robot_pose_graph(rbind(c(0, 0, 0), c(1.2, 0.1, 0.1), c(1.9, 1.2, 1.7)))
g <- pose_graph_add_edge(g, 1, 2, c(1, 0, pi / 2))
g <- pose_graph_add_edge(g, 2, 3, c(1, 0, pi / 2))
res <- pose_graph_optimize(g)
round(res$graph$nodes, 6)
#>      [,1] [,2]      [,3]
#> [1,]    0    0  0.000000
#> [2,]    1    0  1.570796
#> [3,]    1    1 -3.141593
```
