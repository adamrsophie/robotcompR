# Task-space (Cartesian) trajectories

Interpolates between two poses with a quintic time scaling \\s(t)\\: the
position moves along the straight line and the orientation along the
shortest great circle
([`quaternion_slerp()`](https://adamrsophie.github.io/robotcompR/reference/quaternion_slerp.md)),
\\R(s) = \exp(s\\\widehat{\phi}) R_0\\ with world-frame rotation vector
\\\phi = \log(R_1 R_0^\top)\\. The spatial velocity of the moving frame,
in world coordinates and linear part first, is therefore \\(v, \omega) =
\dot s\\(p_1 - p_0, \phi)\\ and its acceleration \\\ddot s\\(p_1 - p_0,
\phi)\\; both vanish at the ends.

## Usage

``` r
robot_traj_cartesian(
  from,
  to,
  duration,
  n = 51,
  robot = NULL,
  initial = NULL,
  frame = NULL,
  weights = NULL,
  ...
)
```

## Arguments

- from, to:

  Start and end `robot_pose` objects (SE(3)).

- duration:

  Duration in seconds.

- n:

  Number of samples.

- robot:

  Optional `robot_model` for the joint-space conversion.

- initial:

  Initial guess for the first inverse kinematics solve.

- frame:

  End-effector frame; defaults to the robot's end effector.

- weights:

  Task weights passed to
  [`robot_ik()`](https://adamrsophie.github.io/robotcompR/reference/robot_ik.md)
  and used in the velocity solves (for example `c(1, 1, 0, 0, 0, 1)` for
  a planar arm).

- ...:

  Further arguments passed to
  [`robot_ik()`](https://adamrsophie.github.io/robotcompR/reference/robot_ik.md).

## Value

A list with `time`, `s`, `poses` (a list of `robot_pose`), `velocity`
and `acceleration` (one row per sample, columns
`vx, vy, vz, wx, wy, wz`), and, with `robot`, `joint` (a
`robot_trajectory`) and `converged` (logical, one per sample).

## Details

When `robot` is given, the path is also converted to joints: inverse
kinematics is solved at each sample, warm-started from the previous
solution (from `initial` at the first sample), and joint velocities and
accelerations follow from the weighted least-squares solutions of \\J
\dot q = V\\ and \\J \ddot q = A - \dot J \dot q\\, with the world
Jacobian of `frame` and \\\dot J \dot q\\ from a central difference of
\\J\\ along \\\dot q\\.

## Examples

``` r
ct <- robot_traj_cartesian(robot_pose(), robot_pose(rotz(1), c(1, 0, 0)),
                           duration = 1, n = 3)
pose_translation(ct$poses[[2]])
#> [1] 0.5 0.0 0.0
ct$velocity[2, ]
#>    vx    vy    vz    wx    wy    wz 
#> 1.875 0.000 0.000 0.000 0.000 1.875 
arm <- robot_example("planar2")
a <- robot_fk(arm, c(0.3, 1.2))
b <- robot_pose(translation = c(1.2, 0.6, 0))
jt <- robot_traj_cartesian(a, b, 1, n = 11, robot = arm, initial = c(0.3, 1.2),
                           weights = c(1, 1, 0, 0, 0, 0))
all(jt$converged)
#> [1] TRUE
```
