# Exponential and logarithm maps of SO(2) and SE(2)

`so2_exp(theta)` returns the 2 by 2 rotation matrix; `so2_log()` returns
the angle in \\(-\pi, \pi\]\\. `se2_exp()` maps a planar twist
`c(vx, vy, w)` to a 3 by 3 homogeneous transform, and `se2_log()`
inverts it.

## Usage

``` r
so2_exp(x)

so2_log(x, tol = 1e-06)

se2_exp(x)

se2_log(x, tol = 1e-06)
```

## Arguments

- x:

  Angle or twist (exp), or a matrix or planar object (log).

- tol:

  Tolerance for validating group elements.

## Value

A matrix (exp) or numeric value or vector (log).

## Examples

``` r
se2_log(se2_exp(c(1, 0, pi / 2)))
#> [1]  1.000000e+00 -3.168556e-17  1.570796e+00
```
