# Distance between two poses

Returns the Euclidean translation distance and the rotation angle
\\\\\log(R_1^\top R_2)\\\\ in radians. The two parts have different
units and are reported separately.

## Usage

``` r
robot_pose_distance(a, b)
```

## Arguments

- a, b:

  `robot_pose` objects of the same dimension.

## Value

A named numeric vector `c(translation, rotation)`.

## Examples

``` r
robot_pose_distance(robot_pose(), robot_pose(translation = c(1, 0, 0)))
#> translation    rotation 
#>           1           0 
```
