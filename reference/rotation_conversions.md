# Convert a rotation to other representations

`as_euler()` returns intrinsic Euler angles for `sequence` (see
[`robot_rotation()`](https://adamrsophie.github.io/robotcompR/reference/robot_rotation.md));
the middle angle lies in \\\[-\pi/2, \pi/2\]\\ for Tait-Bryan sequences
and \\\[0, \pi\]\\ for proper Euler sequences, and the outer angles lie
in \\\[-\pi, \pi)\\. At gimbal lock only the sum or difference of the
outer angles is determined; the third (last) angle is then set to zero.
`as_axis_angle()` returns a list with a unit `axis` and `angle` in
\\\[0, \pi\]\\; the axis is `c(0, 0, 1)` for the identity. `as_rotvec()`
returns \\\theta u\\. For planar rotations `as_axis_angle()` returns the
signed angle.

## Usage

``` r
as_euler(x, sequence = "ZYX")

as_axis_angle(x)

as_rotvec(x)
```

## Arguments

- x:

  A `robot_rotation`, `robot_pose`, `robot_quaternion` or matrix.

- sequence:

  Intrinsic Euler sequence.

## Value

Numeric vector or list.

## Examples

``` r
R <- robot_rotation(euler = c(0.3, -0.2, 0.1), sequence = "ZYX")
as_euler(R, "ZYX")
#> [1]  0.3 -0.2  0.1
as_axis_angle(R)
#> $axis
#> [1]  0.3378807 -0.4807199  0.8091632
#> 
#> $angle
#> [1] 0.3815648
#> 
```
