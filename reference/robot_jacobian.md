# Geometric Jacobians

Returns the 6 by n geometric Jacobian of `frame`, mapping joint
velocities to the spatial velocity of the frame. Rows are ordered linear
first, \\(v_x, v_y, v_z, \omega_x, \omega_y, \omega_z)\\, and columns
follow the configuration order of
[`robot_joint_names()`](https://adamrsophie.github.io/robotcompR/reference/robot_introspection.md).
Columns of joints that do not move the frame are zero.

## Usage

``` r
robot_jacobian(
  robot,
  q,
  frame = NULL,
  reference = c("world", "local", "space"),
  point = NULL
)
```

## Arguments

- robot:

  A `robot_model`.

- q:

  Configuration.

- frame:

  Frame name (`NULL` for the end effector).

- reference:

  `"world"`, `"local"`, or `"space"`.

- point:

  Optional point offset in the frame coordinates, for the Jacobian of a
  point fixed to the frame.

## Value

A 6 by n matrix.

## Details

The `reference` argument fixes the coordinates:

- `"world"` (default): \\v\\ is the velocity of the frame origin (or
  `point`) and \\\omega\\ the angular velocity, both expressed in world
  coordinates. This is the Jacobian used for inverse kinematics.

- `"local"`: the same velocities expressed in the axes of `frame` (the
  body Jacobian).

- `"space"`: the spatial twist, whose linear part is the velocity of the
  body point that instantaneously coincides with the world origin (the
  space Jacobian of Lynch and Park).

For a revolute joint with world axis \\z\\ through point \\o\\, the
world column is \\(z \times (p - o), z)\\; for a prismatic joint it is
\\(z, 0)\\. The geometric Jacobian gives angular velocities; it is not
the derivative of Euler angles, which is given by
[`robot_jacobian_analytic()`](https://adamrsophie.github.io/robotcompR/reference/robot_jacobian_analytic.md).

## Examples

``` r
robot <- robot_example("planar2")
robot_jacobian(robot, c(0.2, 0.4))[1:2, ]
#>        joint1     joint2
#> vx -0.7633118 -0.5646425
#> vy  1.8054022  0.8253356
```
