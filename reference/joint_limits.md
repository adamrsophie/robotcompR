# Joint limits checks

`robot_within_limits()` reports whether each configuration satisfies the
position limits within `tol`. `robot_clamp()` projects a configuration
onto the limit box.

## Usage

``` r
robot_within_limits(robot, q, tol = 1e-10)

robot_clamp(robot, q)
```

## Arguments

- robot:

  A `robot_model`.

- q:

  Configuration vector.

- tol:

  Tolerance.

## Value

Logical scalar (`robot_within_limits`) or numeric vector.

## Examples

``` r
robot <- robot_example("planar2")
robot_within_limits(robot, c(0, 4))
#> [1] FALSE
robot_clamp(robot, c(0, 4))
#> [1] 0.000000 3.141593
```
