# Differential-drive kinematics and odometry

`diffdrive_forward()` converts wheel angular speeds (rad/s) into body
velocities `c(v, omega)`; `diffdrive_inverse()` does the reverse.
`diffdrive_odometry()` integrates wheel rotation increments (radians,
for example encoder ticks times radians per tick), treating each
increment as a constant-velocity arc, which is exact when the wheel
speeds are constant over each interval.

## Usage

``` r
diffdrive_forward(model, omega_left, omega_right)

diffdrive_inverse(model, v, omega)

diffdrive_odometry(model, pose, delta_left, delta_right)
```

## Arguments

- model:

  A differential-drive `robot_mobile`.

- omega_left, omega_right:

  Wheel angular speeds (rad/s).

- v, omega:

  Body velocities.

- pose:

  Initial pose.

- delta_left, delta_right:

  Vectors of wheel rotation increments.

## Value

A named vector, or a matrix of poses (first row is `pose`).

## Examples

``` r
dd <- robot_diffdrive(0.05, 0.3)
diffdrive_inverse(dd, 0.5, 1)
#>  left right 
#>     7    13 
diffdrive_odometry(dd, c(0, 0, 0), rep(1, 4), rep(1.2, 4))
#>               x            y      theta
#> [1,] 0.00000000 0.0000000000 0.00000000
#> [2,] 0.05498982 0.0009165818 0.03333333
#> [3,] 0.10991854 0.0036653088 0.06666667
#> [4,] 0.16472514 0.0082431273 0.10000000
#> [5,] 0.21934873 0.0146449511 0.13333333
```
