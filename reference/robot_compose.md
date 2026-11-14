# Compose poses

`robot_compose(a, b, c)` returns `a * b * c`, the product of the
homogeneous matrices from left to right.

## Usage

``` r
robot_compose(...)
```

## Arguments

- ...:

  `robot_pose` or `robot_rotation` objects.

## Value

A `robot_pose`.

## Examples

``` r
A <- robot_pose(translation = c(1, 0, 0), parent = "world", child = "a")
B <- robot_pose(robot_rotation(axis = c(0, 0, 1), angle = pi / 2),
                parent = "a", child = "b")
robot_compose(A, B)
#> <robot_pose> SE(3) world <- b
#>      [,1] [,2] [,3] [,4]
#> [1,]    0   -1    0    1
#> [2,]    1    0    0    0
#> [3,]    0    0    1    0
#> [4,]    0    0    0    1
```
