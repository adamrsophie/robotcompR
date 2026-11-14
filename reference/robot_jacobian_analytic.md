# Analytic Jacobians

Maps joint velocities to the time derivative of the position and a
minimal orientation parameterization. With `representation = "rpy"` the
orientation is the roll-pitch-yaw triple \\(r, p, y)\\ with \\R = R_z(y)
R_y(p) R_x(r)\\, and \\\omega = E(r, p, y) (\dot r, \dot p, \dot y)\\
with columns \\E = \[R_z R_y e_x, R_z e_y, e_z\]\\; the analytic
Jacobian is \\J_a = \mathrm{diag}(I, E^{-1}) J\\. `E` is singular when
\\\cos p = 0\\; the function stops there. With
`representation = "rotvec"` the orientation is the rotation vector
\\\phi\\ and \\\dot\phi = J_l(\phi)^{-1}\omega\\.

## Usage

``` r
robot_jacobian_analytic(
  robot,
  q,
  frame = NULL,
  representation = c("rpy", "rotvec"),
  tol = 1e-08
)
```

## Arguments

- robot:

  A `robot_model`.

- q:

  Configuration.

- frame:

  Frame name (`NULL` for the end effector).

- representation:

  `"rpy"` or `"rotvec"`.

- tol:

  Singularity tolerance on \\\|\cos p\|\\.

## Value

A 6 by n matrix.

## Examples

``` r
robot <- robot_example("puma560")
robot_jacobian_analytic(robot, c(0, 0.3, -0.2, 0.1, 0.4, 0.2))
#>          joint1        joint2        joint3        joint4     joint5     joint6
#> x     0.1500500 -5.592750e-01 -4.316694e-01 -1.314306e-19  0.0000000  0.0000000
#> y     0.3896048  2.385641e-17 -1.402801e-18 -1.387779e-17  0.0000000  0.0000000
#> z     0.0000000  3.896048e-01 -2.290948e-02 -1.318705e-20  0.0000000  0.0000000
#> roll  0.0000000 -3.682612e-01 -3.682612e-01 -1.067785e-01 -0.2601764 -0.5250239
#> pitch 0.0000000 -9.455240e-01 -9.455240e-01  3.250100e-02 -0.9731390  0.1186888
#> yaw   1.0000000  1.721395e-01  1.721395e-01  1.044917e+00  0.1315832  1.1231933
```
