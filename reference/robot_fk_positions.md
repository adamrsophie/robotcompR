# Positions of a frame for many configurations

Evaluates forward kinematics for each row of `Q` and returns the frame
origin positions as a matrix with one row per configuration.

## Usage

``` r
robot_fk_positions(robot, Q, frame = NULL)
```

## Arguments

- robot:

  A `robot_model`.

- Q:

  Matrix with one configuration per row.

- frame:

  Frame name (`NULL` for the end effector).

## Value

A matrix with columns x, y, z.

## Examples

``` r
robot <- robot_example("planar2")
robot_fk_positions(robot, rbind(c(0, 0), c(pi / 2, 0)))
#>                 x y z
#> [1,] 2.000000e+00 0 0
#> [2,] 2.220446e-16 2 0
```
