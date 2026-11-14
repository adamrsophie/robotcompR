# Detect singular configurations

Returns `TRUE` when the selected Jacobian block loses rank within `tol`,
with the manipulability diagnostics attached as an attribute. A warning
is issued when `warn = TRUE`.

## Usage

``` r
robot_is_singular(
  robot,
  q,
  frame = NULL,
  measure = c("translational", "rotational", "full"),
  rows = NULL,
  tol = 1e-08,
  warn = FALSE
)
```

## Arguments

- robot:

  A `robot_model`.

- q:

  Configuration.

- frame:

  Frame name (`NULL` for the end effector).

- measure:

  `"translational"`, `"rotational"`, or `"full"`.

- rows:

  Optional integer vector of Jacobian rows to keep instead of `measure`.

- tol:

  Relative tolerance for rank and singularity decisions.

- warn:

  Issue a warning at singular configurations.

## Value

Logical scalar.

## Examples

``` r
robot <- robot_example("planar2")
robot_is_singular(robot, c(0.4, 0), rows = 1:2)
#> [1] TRUE
#> attr(,"manipulability")
#> <robot_manipulability> custom rows (vx, vy)
#> singular values: 2.2361, 4.0129e-17
#> rank 1, Yoshikawa 8.97308e-17, condition Inf, SINGULAR
```
