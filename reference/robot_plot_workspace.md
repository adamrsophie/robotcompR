# Plot the reachable workspace

Samples `n` random configurations within the joint limits and plots the
positions of `frame`. This is a Monte Carlo picture of the workspace,
not an exact boundary.

## Usage

``` r
robot_plot_workspace(
  robot,
  n = 2000,
  frame = NULL,
  projection = c("xy", "xz", "yz"),
  seed = NULL,
  color_by = NULL,
  col = grDevices::adjustcolor("steelblue", 0.4),
  main = "Workspace samples",
  ...
)
```

## Arguments

- robot:

  A `robot_model`.

- n:

  Number of samples.

- frame:

  Frame (`NULL` for the end effector).

- projection:

  `"xy"`, `"xz"` or `"yz"`.

- seed:

  Optional seed (the global random state is restored).

- color_by:

  Optional function of a configuration returning a number used to color
  points, for example a manipulability measure.

- col:

  Point color when `color_by` is `NULL`.

- main:

  Plot title.

- ...:

  Further arguments to
  [`graphics::plot()`](https://rdrr.io/r/graphics/plot.default.html).

## Value

Invisibly, the sampled positions.

## Examples

``` r
robot <- robot_example("planar2")
robot_plot_workspace(robot, n = 500, seed = 1)
```
