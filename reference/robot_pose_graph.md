# SE(2) pose graphs

`robot_pose_graph()` creates a graph with initial node poses (one
`c(x, y, theta)` per row). `pose_graph_add_edge()` adds a relative pose
measurement \\z\_{ij}\\, the pose of node j in the frame of node i, with
a 3 by 3 information matrix (inverse covariance, ordered x, y, theta).

## Usage

``` r
robot_pose_graph(nodes)

pose_graph_add_edge(graph, from, to, measurement, information = diag(3))
```

## Arguments

- nodes:

  Matrix of initial poses.

- graph:

  A `robot_pose_graph`.

- from, to:

  Node indices.

- measurement:

  Relative pose `c(dx, dy, dtheta)`.

- information:

  Information matrix.

## Value

A `robot_pose_graph`.

## Examples

``` r
g <- robot_pose_graph(rbind(c(0, 0, 0), c(1.1, 0.1, 0.05)))
g <- pose_graph_add_edge(g, 1, 2, c(1, 0, 0), diag(3))
```
