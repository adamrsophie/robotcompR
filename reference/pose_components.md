# Pose components

`pose_rotation()` returns the rotation part as a `robot_rotation`, and
`pose_translation()` the translation vector.

## Usage

``` r
pose_rotation(x)

pose_translation(x)
```

## Arguments

- x:

  A `robot_pose`.

## Value

A `robot_rotation` or numeric vector.

## Examples

``` r
T <- robot_pose(translation = c(1, 2, 3))
pose_translation(T)
#> [1] 1 2 3
```
