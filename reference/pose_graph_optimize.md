# Optimize an SE(2) pose graph

Minimizes \\\sum e\_{ij}^\top \Omega\_{ij} e\_{ij}\\ over the node poses
with Gauss-Newton iterations and Levenberg-Marquardt damping, using the
relative-pose residual \\e\_{ij} = (R_z^\top(R_i^\top(t_j - t_i) - t_z),
\mathrm{wrap}(\theta_j - \theta_i - \theta_z))\\, analytic Jacobians
(Grisetti et al., 2010, "A tutorial on graph-based SLAM") and sparse
linear algebra from 'Matrix'. Node `fixed` is held constant to remove
the gauge freedom. Headings are wrapped after each update.

## Usage

``` r
pose_graph_optimize(graph, fixed = 1, max_iterations = 50, tol = 1e-12)
```

## Arguments

- graph:

  A `robot_pose_graph`.

- fixed:

  Index of the anchored node.

- max_iterations:

  Maximum iterations.

- tol:

  Stop when the relative decrease of the cost is below `tol`.

## Value

A list with the optimized `graph`, `cost` history, `iterations` and
`converged`.

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
