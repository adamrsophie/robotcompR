# Robot models from Denavit-Hartenberg parameters

Builds a serial `robot_model` whose link frames coincide with the DH
frames. Link `i` is named `"link<i>"` and its frame is DH frame `i`; the
root is `"base"` (DH frame 0). For revolute joints `theta` is the joint
offset and \\q_i\\ is added to it; for prismatic joints `d` is the
offset and \\q_i\\ is added to it. Standard and modified conventions are
never mixed in one model; see
[`dh_transform()`](https://adamrsophie.github.io/robotcompR/reference/dh_transform.md)
for the formulas.

## Usage

``` r
robot_dh(
  a,
  alpha,
  d,
  theta,
  joint_type = "revolute",
  convention = c("standard", "modified"),
  limits = NULL,
  base = robot_pose(),
  tool = NULL,
  mass = NULL,
  com = NULL,
  inertia = NULL,
  name = "dh_robot"
)
```

## Arguments

- a, alpha, d, theta:

  Numeric vectors of equal length with one entry per link.

- joint_type:

  Character vector of `"revolute"` or `"prismatic"`.

- convention:

  `"standard"` or `"modified"`.

- limits:

  Optional matrix with one row `c(lower, upper)` per joint.

- base:

  Optional base pose.

- tool:

  Optional pose of a `"tool"` frame in the last link frame.

- mass, com, inertia:

  Optional inertial parameters.

- name:

  Model name.

## Value

A `robot_model` with extra field `dh` holding the parameters.

## Details

Optional inertial parameters are given per link in the DH link frame:
`mass` is a vector, `com` a matrix with one row per link, and `inertia`
a list of 3 by 3 matrices about the center of mass.

## Examples

``` r
robot <- robot_dh(a = c(1, 1), alpha = c(0, 0), d = c(0, 0),
                  theta = c(0, 0), joint_type = c("revolute", "revolute"))
pose_translation(robot_fk(robot, c(pi / 2, pi / 2)))
#> [1] -1  1  0
```
