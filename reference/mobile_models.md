# Mobile robot models

`robot_diffdrive()` describes a differential-drive base with wheel
radius \\r\\ and track width \\b\\ (distance between wheels): \\v =
r(\omega_R + \omega_L)/2\\, \\\omega = r(\omega_R - \omega_L)/b\\.
`robot_bicycle()` describes a kinematic bicycle (or Ackermann car) with
wheelbase \\L\\, referenced at the rear axle: \\\omega = v \tan\delta /
L\\. `robot_unicycle()` is the plain unicycle. Optional limits are
checked by
[`mobile_simulate()`](https://adamrsophie.github.io/robotcompR/reference/mobile_simulate.md).

## Usage

``` r
robot_unicycle(max_speed = Inf, max_turn_rate = Inf)

robot_diffdrive(
  wheel_radius = 0.05,
  track_width = 0.3,
  max_speed = Inf,
  max_turn_rate = Inf
)

robot_bicycle(wheelbase = 2.5, max_steer = pi/4, max_speed = Inf)
```

## Arguments

- max_speed, max_turn_rate:

  Optional limits.

- wheel_radius, track_width:

  Differential-drive geometry (m).

- wheelbase:

  Bicycle wheelbase (m).

- max_steer:

  Maximum steering angle (rad).

## Value

A `robot_mobile` object.

## Examples

``` r
dd <- robot_diffdrive(wheel_radius = 0.05, track_width = 0.3)
diffdrive_forward(dd, 10, 12)
#>         v     omega 
#> 0.5500000 0.3333333 
```
