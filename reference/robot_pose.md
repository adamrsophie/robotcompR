# Rigid-body poses in SE(2) and SE(3)

A pose is a homogeneous transform \\T = \[R, p; 0, 1\]\\. The pose
\\{}^{a}T\_{b}\\ gives the position and orientation of frame `b`
(`child`) relative to frame `a` (`parent`), and maps coordinates of a
point expressed in `b` into `a`: \\{}^{a}p = {}^{a}T\_{b}\\{}^{b}p\\.
Composition follows matrix multiplication: \\{}^{a}T\_{c} =
{}^{a}T\_{b}\\{}^{b}T\_{c}\\, written `T_ab * T_bc`. When both operands
carry frame names, composition checks that the child of the left operand
equals the parent of the right operand.

## Usage

``` r
robot_pose(
  rotation = NULL,
  translation = NULL,
  matrix = NULL,
  parent = NULL,
  child = NULL,
  tol = 1e-06
)
```

## Arguments

- rotation:

  A `robot_rotation`, rotation matrix, or quaternion. Defaults to the
  identity of the dimension implied by `translation`.

- translation:

  Translation vector of length 2 or 3 (meters).

- matrix:

  Alternatively, a full homogeneous matrix.

- parent, child:

  Optional frame names.

- tol:

  Validation tolerance.

## Value

A `robot_pose` object.

## Examples

``` r
T <- robot_pose(robot_rotation(axis = c(0, 0, 1), angle = pi / 2),
                c(1, 2, 3))
robot_transform_point(T, c(1, 0, 0))
#> [1] 1 3 3
robot_inverse(T) * T
#> <robot_pose> SE(3)
#>      [,1] [,2] [,3] [,4]
#> [1,]    1    0    0    0
#> [2,]    0    1    0    0
#> [3,]    0    0    1    0
#> [4,]    0    0    0    1
```
