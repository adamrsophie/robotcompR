# Elementary rotation matrices

`rotx()`, `roty()` and `rotz()` return 3 by 3 rotation matrices for an
active, right-handed rotation by `theta` radians about the x, y and z
axes. `rot2()` returns the 2 by 2 planar rotation.

## Usage

``` r
rotx(theta)

roty(theta)

rotz(theta)

rot2(theta)
```

## Arguments

- theta:

  Angle in radians.

## Value

A numeric matrix.

## Examples

``` r
rotz(pi / 2) %*% c(1, 0, 0)
#>              [,1]
#> [1,] 6.123234e-17
#> [2,] 1.000000e+00
#> [3,] 0.000000e+00
```
