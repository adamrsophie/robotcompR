# Quaternion operations

`quaternion_conjugate()` returns \\(w, -x, -y, -z)\\, the inverse of a
unit quaternion. `quaternion_rotate()` rotates the columns of a 3 by n
matrix (or one 3-vector). `quaternion_distance()` returns the rotation
angle between two orientations in \\\[0, \pi\]\\, which is invariant to
the sign ambiguity.

## Usage

``` r
quaternion_conjugate(q)

quaternion_rotate(q, v)

quaternion_distance(q1, q2)
```

## Arguments

- q, q1, q2:

  `robot_quaternion` objects or length-4 vectors.

- v:

  A 3-vector or 3 by n matrix.

## Value

A quaternion, matrix, vector or number.

## Examples

``` r
q <- robot_quaternion(robot_rotation(axis = c(0, 0, 1), angle = pi / 2))
quaternion_rotate(q, c(1, 0, 0))
#> [1] 2.220446e-16 1.000000e+00 0.000000e+00
quaternion_distance(q, robot_quaternion(c(1, 0, 0, 0)))
#> [1] 1.570796
```
