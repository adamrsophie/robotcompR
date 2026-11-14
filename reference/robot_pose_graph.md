# Pose graphs

`robot_pose_graph()` creates a graph with initial node poses: either a
matrix with one planar pose `c(x, y, theta)` per row (an SE(2) graph) or
a list of SE(3) `robot_pose` objects or 4 by 4 homogeneous matrices (an
SE(3) graph). `pose_graph_add_edge()` adds a relative pose measurement
\\z\_{ij}\\, the pose of node j in the frame of node i, with an
information matrix (inverse covariance). For SE(2) the measurement is
`c(dx, dy, dtheta)` and the information matrix is 3 by 3 (x, y, theta);
for SE(3) the measurement is a `robot_pose` (or 4 by 4 matrix) and the
information matrix is 6 by 6, translation first, matching the tangent
ordering of
[`se3_log()`](https://adamrsophie.github.io/robotcompR/reference/lie_exp_log.md).

## Usage

``` r
robot_pose_graph(nodes)

pose_graph_add_edge(graph, from, to, measurement, information = NULL)
```

## Arguments

- nodes:

  Matrix of initial planar poses, or a list of SE(3) poses.

- graph:

  A `robot_pose_graph`.

- from, to:

  Node indices.

- measurement:

  Relative pose.

- information:

  Information matrix; defaults to the identity.

## Value

A `robot_pose_graph` with `nodes`, `edges` and `group` (`"SE2"` or
`"SE3"`).

## Examples

``` r
g <- robot_pose_graph(rbind(c(0, 0, 0), c(1.1, 0.1, 0.05)))
g <- pose_graph_add_edge(g, 1, 2, c(1, 0, 0), diag(3))
g3 <- robot_pose_graph(list(robot_pose(), transl(1.1, 0.1, 0)))
g3 <- pose_graph_add_edge(g3, 1, 2, transl(1, 0, 0))
```
