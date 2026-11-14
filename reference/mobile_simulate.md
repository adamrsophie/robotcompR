# Simulate a mobile robot

Applies piecewise-constant controls with the closed-form arc of
[`unicycle_step()`](https://adamrsophie.github.io/robotcompR/reference/unicycle_step.md).
For unicycles and differential drives the controls are `c(v, omega)` per
row; for bicycles `c(v, steer)`, converted to \\\omega =
v\tan\delta/L\\. Controls outside the model limits are an error unless
`saturate = TRUE`, in which case they are clipped.

## Usage

``` r
mobile_simulate(model, pose, controls, dt, saturate = FALSE)
```

## Arguments

- model:

  A `robot_mobile`.

- pose:

  Initial pose `c(x, y, theta)`.

- controls:

  Matrix with one control per row.

- dt:

  Step duration (scalar or one per row).

- saturate:

  Clip controls to the limits.

## Value

A matrix of poses with columns `time`, `x`, `y`, `theta`.

## Examples

``` r
sim <- mobile_simulate(robot_unicycle(), c(0, 0, 0),
                       cbind(1, rep(pi / 2, 4)), dt = 1)
round(sim, 6)
#>      time        x       y     theta
#> [1,]    0  0.00000 0.00000  0.000000
#> [2,]    1  0.63662 0.63662  1.570796
#> [3,]    2  0.00000 1.27324 -3.141593
#> [4,]    3 -0.63662 0.63662 -1.570796
#> [5,]    4  0.00000 0.00000  0.000000
```
