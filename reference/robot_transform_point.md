# Transform points and directions

`robot_transform_point(T, p)` returns \\R p + t\\. Directions (free
vectors such as velocities or axes) only rotate:
`robot_transform_direction(T, v)` returns \\R v\\. Points may be one
vector or a matrix with one point per row.

## Usage

``` r
robot_transform_point(x, points)

robot_transform_direction(x, points)
```

## Arguments

- x:

  A `robot_pose` or `robot_rotation`.

- points:

  A vector or matrix with one point per row.

## Value

Transformed points with the same shape as the input.

## Examples

``` r
T <- robot_pose(robot_rotation(axis = c(0, 0, 1), angle = pi / 2), c(1, 2, 3))
robot_transform_point(T, rbind(c(1, 0, 0), c(0, 1, 0)))
#>      [,1] [,2] [,3]
#> [1,]    1    3    3
#> [2,]    0    2    3
robot_transform_direction(T, c(1, 0, 0))
#> [1] 1.110223e-16 1.000000e+00 0.000000e+00
```
