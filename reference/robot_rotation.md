# Rotations in two and three dimensions

Creates a validated rotation, an element of SO(2) or SO(3). Exactly one
representation must be supplied:

## Usage

``` r
robot_rotation(
  x = NULL,
  axis = NULL,
  angle = NULL,
  euler = NULL,
  sequence = "ZYX",
  rotvec = NULL,
  quaternion = NULL,
  tol = 1e-06
)
```

## Arguments

- x:

  Optional matrix, `robot_rotation`, or `robot_quaternion`.

- axis:

  Rotation axis (length 3).

- angle:

  Rotation angle in radians.

- euler:

  Three Euler angles in radians.

- sequence:

  Intrinsic axis sequence, one of the 12 valid sequences such as
  `"ZYX"`, `"XYZ"` or `"ZYZ"`.

- rotvec:

  Rotation vector (length 3).

- quaternion:

  Scalar-first quaternion (length 4).

- tol:

  Tolerance for matrix validation.

## Value

A `robot_rotation` object.

## Details

- `x`: a rotation matrix, `robot_rotation`, or `robot_quaternion`;

- `axis` and `angle`: active right-handed rotation by `angle` radians
  about the unit vector `axis` (any nonzero vector is normalized);

- `angle` alone: a planar rotation (SO(2));

- `euler` with `sequence`: intrinsic Euler angles, so `sequence = "ZYX"`
  and `euler = c(a, b, c)` gives \\R_z(a) R_y(b) R_x(c)\\ (yaw, pitch,
  roll). The URDF `rpy = c(r, p, y)` attribute corresponds to
  `sequence = "ZYX"` with `euler = c(y, p, r)`;

- `rotvec`: a rotation vector \\\theta u\\;

- `quaternion`: scalar-first `c(w, x, y, z)`.

Matrices are checked for \\R^\top R = I\\ and \\\det R = 1\\ within
`tol`; use
[`orthonormalize()`](https://adamrsophie.github.io/robotcompR/reference/orthonormalize.md)
first to repair accumulated drift.

## Examples

``` r
R <- robot_rotation(axis = c(0, 0, 1), angle = pi / 2)
R
#> <robot_rotation> SO(3)
#> angle 1.5708 rad about axis (0, 0, 1)
#>      [,1] [,2] [,3]
#> [1,]    0   -1    0
#> [2,]    1    0    0
#> [3,]    0    0    1
robot_rotation(euler = c(0.3, -0.2, 0.1), sequence = "ZYX")
#> <robot_rotation> SO(3)
#> angle 0.381565 rad about axis (0.3379, -0.4807, 0.8092)
#>        [,1]    [,2]    [,3]
#> [1,] 0.9363 -0.3130 -0.1593
#> [2,] 0.2896  0.9447 -0.1538
#> [3,] 0.1987  0.0978  0.9752
robot_rotation(angle = pi / 6)
#> <robot_rotation> SO(2)
#> angle 0.523599 rad
#>       [,1]   [,2]
#> [1,] 0.866 -0.500
#> [2,] 0.500  0.866
```
