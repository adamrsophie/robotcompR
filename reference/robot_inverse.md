# Inverse of a rotation or pose

For \\T = \[R, p; 0, 1\]\\ returns \\T^{-1} = \[R^\top, -R^\top p; 0,
1\]\\, computed in closed form. Frame names are swapped.

## Usage

``` r
robot_inverse(x)
```

## Arguments

- x:

  A `robot_pose`, `robot_rotation`, or `robot_quaternion`.

## Value

An object of the same class.

## Examples

``` r
T <- robot_pose(robot_rotation(axis = c(1, 0, 0), angle = 0.5), c(1, 2, 3))
as.matrix(robot_inverse(T) * T)
#>      [,1]        [,2]        [,3] [,4]
#> [1,]    1 0.00000e+00 0.00000e+00    0
#> [2,]    0 1.00000e+00 2.58022e-17    0
#> [3,]    0 2.58022e-17 1.00000e+00    0
#> [4,]    0 0.00000e+00 0.00000e+00    1
```
