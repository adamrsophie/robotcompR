# Interpolate between two poses

With `method = "decoupled"` (default) the translation is interpolated
linearly and the rotation by
[`quaternion_slerp()`](https://adamrsophie.github.io/robotcompR/reference/quaternion_slerp.md)
(or linear angle interpolation in 2D). With `method = "screw"` the
interpolation follows the constant twist \\T_1 \exp(s \log(T_1^{-1}
T_2))\\, a screw motion.

## Usage

``` r
robot_pose_interp(a, b, s, method = c("decoupled", "screw"))
```

## Arguments

- a, b:

  Start and end poses.

- s:

  Interpolation parameter(s) in \\\[0, 1\]\\.

- method:

  `"decoupled"` or `"screw"`.

## Value

A `robot_pose` (scalar `s`) or a list of poses.

## Examples

``` r
a <- robot_pose()
b <- robot_pose(robot_rotation(axis = c(0, 0, 1), angle = pi / 2), c(1, 1, 0))
robot_pose_interp(a, b, 0.5)
#> <robot_pose> SE(3)
#>        [,1]    [,2] [,3] [,4]
#> [1,] 0.7071 -0.7071    0  0.5
#> [2,] 0.7071  0.7071    0  0.5
#> [3,] 0.0000  0.0000    1  0.0
#> [4,] 0.0000  0.0000    0  1.0
```
