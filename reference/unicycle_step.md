# Closed-form unicycle motion

Integrates the unicycle model \\\dot x = v\cos\theta\\, \\\dot y =
v\sin\theta\\, \\\dot\theta = \omega\\ in closed form for constant body
velocities over `dt`: along a circular arc of radius \\v/\omega\\,
written in the chord form \\\Delta p = v\\dt\\\mathrm{sinc}(\omega
dt/2)(\cos(\theta + \omega dt/2), \sin(\theta + \omega dt/2))\\, which
stays accurate as \\\omega \to 0\\ (a straight line in the limit).
Equivalently the new pose is \\X \exp(\widehat{(v, 0, \omega)}\\dt)\\ on
SE(2).

## Usage

``` r
unicycle_step(pose, v, omega, dt)
```

## Arguments

- pose:

  Pose `c(x, y, theta)`.

- v:

  Forward speed (m/s).

- omega:

  Turn rate (rad/s).

- dt:

  Duration (s).

## Value

The new pose.

## Examples

``` r
unicycle_step(c(0, 0, 0), v = 1, omega = pi / 2, dt = 1)
#>         x         y     theta 
#> 0.6366198 0.6366198 1.5707963 
```
