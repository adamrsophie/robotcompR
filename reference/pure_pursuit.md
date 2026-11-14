# Pure pursuit path following

Computes a unicycle command that steers toward the lookahead point: the
point of the polyline `path` an arclength `lookahead` beyond the closest
point to the robot. The commanded curvature is \\\kappa = 2 y_L /
\ell^2\\, the circle through the robot and the target, where \\(x_L,
y_L)\\ is the target in the body frame and \\\ell^2 = x_L^2 + y_L^2\\
its squared distance (Coulter, 1992). On a straight path \\\ell\\ is
close to `lookahead`, but it is shorter when the robot is off the path.
Near the end of the path the last point is used as the goal.

## Usage

``` r
pure_pursuit(pose, path, lookahead, speed)
```

## Arguments

- pose:

  Current pose.

- path:

  Matrix of 2D waypoints (one per row).

- lookahead:

  Lookahead distance \\L_d\\ (m).

- speed:

  Forward speed (m/s).

## Value

A list with `v`, `omega`, the `target` point and `done` (within
`lookahead / 4` of the final point).

## Examples

``` r
path <- cbind(seq(0, 5, 0.5), 1)
pure_pursuit(c(0, 0, 0), path, lookahead = 1, speed = 0.5)$omega
#> [1] 0.5
```
