# Unit quaternions

Creates a unit quaternion with scalar-first component order \\q = (w, x,
y, z)\\. Quaternions use the Hamilton product and represent active,
right-handed rotations: \\R(q)v = q \otimes (0, v) \otimes q^\*\\.
Because \\q\\ and \\-q\\ represent the same rotation, the constructor
normalizes the input and returns the representative with \\w \ge 0\\.
Composition order matches rotation matrices:
`R(q1 * q2) = R(q1) %*% R(q2)`.

## Usage

``` r
robot_quaternion(x, canonical = TRUE)
```

## Arguments

- x:

  A length-4 numeric vector `c(w, x, y, z)`, a rotation matrix, or a
  `robot_rotation`. Vectors are normalized; a zero vector is an error.

- canonical:

  If `TRUE` (default) return the representative with nonnegative scalar
  part.

## Value

A `robot_quaternion`.

## Examples

``` r
q <- robot_quaternion(c(cos(pi / 4), 0, 0, sin(pi / 4)))
as.matrix(robot_rotation(q))
#>              [,1]          [,2] [,3]
#> [1,] 2.220446e-16 -1.000000e+00    0
#> [2,] 1.000000e+00  2.220446e-16    0
#> [3,] 0.000000e+00  0.000000e+00    1
q * quaternion_conjugate(q)
#> <robot_quaternion> w = 1, (x, y, z) = (0, 0, 0)
```
