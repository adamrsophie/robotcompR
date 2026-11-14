# Convert between body and world velocities

For a planar robot at heading \\\theta\\, the world velocity of the body
origin is \\(v\cos\theta, v\sin\theta)\\ for body velocity \\(v_x^b,
v_y^b) = (v, 0)\\; in general \\v^w = R(\theta) v^b\\.

## Usage

``` r
body_to_world_velocity(velocity, theta)

world_to_body_velocity(velocity, theta)
```

## Arguments

- velocity:

  Length-2 velocity.

- theta:

  Heading (rad).

## Value

Length-2 velocity in the other frame.

## Examples

``` r
body_to_world_velocity(c(1, 0), pi / 2)
#> [1] 6.123234e-17 1.000000e+00
```
