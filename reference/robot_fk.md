# Forward kinematics

Computes the pose of a frame in the world frame,
\\{}^{world}T\_{frame}(q) = T\_{base} \prod_k {}^{k-1}T\_{k}(q_k)\\,
along the path from the root to `frame`. With `frame = "all"` the poses
of every link frame are returned as a named list. A matrix `q` with one
configuration per row returns a list with one result per row.
Configurations outside the joint limits are evaluated without error; use
[`robot_within_limits()`](https://adamrsophie.github.io/robotcompR/reference/joint_limits.md)
to check them.

## Usage

``` r
robot_fk(robot, q, frame = NULL)
```

## Arguments

- robot:

  A `robot_model`.

- q:

  Joint configuration (length
  [`robot_dof()`](https://adamrsophie.github.io/robotcompR/reference/robot_introspection.md))
  or a matrix with one configuration per row.

- frame:

  Frame name, `"all"`, or `NULL` for the end effector.

## Value

A `robot_pose`, or a list of poses.

## Examples

``` r
robot <- robot_example("planar2")
robot_fk(robot, c(pi / 2, pi / 2))
#> <robot_pose> SE(3) world <- link2
#>      [,1] [,2] [,3] [,4]
#> [1,]   -1    0    0   -1
#> [2,]    0   -1    0    1
#> [3,]    0    0    1    0
#> [4,]    0    0    0    1
```
