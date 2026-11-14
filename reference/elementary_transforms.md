# Homogeneous transforms from elementary motions

`transl()` returns a pure translation and `trotx()`, `troty()` and
`trotz()` pure rotations, all as `robot_pose` objects, so that sequences
of elementary transformations can be composed with `*`.

## Usage

``` r
transl(x = 0, y = 0, z = 0)

trotx(theta)

troty(theta)

trotz(theta)
```

## Arguments

- x, y, z:

  Translation components in meters. `x` may be a length-3 vector.

- theta:

  Angle in radians.

## Value

A `robot_pose`.

## Examples

``` r
T <- transl(1, 0, 0) * trotz(pi / 2) * transl(1, 0, 0)
pose_translation(T)
#> [1] 1 1 0
```
