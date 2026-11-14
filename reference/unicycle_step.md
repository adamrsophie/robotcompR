# Exact unicycle motion

Integrates the unicycle model \\\dot x = v\cos\theta\\, \\\dot y =
v\sin\theta\\, \\\dot\theta = \omega\\ exactly for constant body
velocities over `dt`: along a circular arc of radius \\v/\omega\\, or a
straight line when \\\|\omega\| \< 10^{-12}\\. Equivalently the new pose
is \\X \exp(\widehat{(v, 0, \omega)}\\dt)\\ on SE(2).

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
