# Spherical linear interpolation of rotations

Interpolates between two orientations along the shortest great circle:
\\q(s) = q_1 (q_1^{\*} q_2)^s\\ for \\s \in \[0, 1\]\\. The sign of
\\q_2\\ is flipped when needed so the path is the shorter of the two
arcs that represent the same pair of rotations. For nearly identical
inputs the method falls back to normalized linear interpolation.

## Usage

``` r
quaternion_slerp(q1, q2, s)
```

## Arguments

- q1, q2:

  Start and end orientations: quaternions, rotation matrices or
  `robot_rotation` objects.

- s:

  Interpolation parameter(s) in \\\[0, 1\]\\.

## Value

A `robot_quaternion` when `s` is a scalar, otherwise a list.

## Examples

``` r
qa <- robot_quaternion(c(1, 0, 0, 0))
qb <- robot_quaternion(robot_rotation(axis = c(0, 0, 1), angle = pi / 2))
quaternion_slerp(qa, qb, 0.5)
#> <robot_quaternion> w = 0.92388, (x, y, z) = (0, 0, 0.382683)
```
