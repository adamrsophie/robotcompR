# Finite-difference Jacobian check

Computes the world-frame Jacobian of `frame` by central differences on
the Lie group: column \\i\\ is \\(p(q + h e_i) - p(q - h e_i)) / 2h\\
for the linear part and \\\log(R(q + h e_i) R(q - h e_i)^\top) / 2h\\
for the angular part. This uses forward kinematics only and is
independent of the analytic Jacobian code, so it serves as a validation
reference.

## Usage

``` r
robot_jacobian_numeric(robot, q, frame = NULL, h = 1e-06)
```

## Arguments

- robot:

  A `robot_model`.

- q:

  Configuration.

- frame:

  Frame name (`NULL` for the end effector).

- h:

  Step size.

## Value

A 6 by n matrix.

## Examples

``` r
robot <- robot_example("planar3")
q <- c(0.1, 0.2, 0.3)
max(abs(robot_jacobian(robot, q) - robot_jacobian_numeric(robot, q)))
#> [1] 1.66096e-10
```
