# Exponential and logarithm maps of SO(3) and SE(3)

`so3_exp()` maps a rotation vector \\\omega = \theta u\\ to the rotation
matrix \\\exp(\hat\omega)\\ using the Rodrigues formula, with Taylor
expansions of its coefficients when \\\theta \< 10^{-5}\\. `so3_log()`
returns the rotation vector with \\\theta \in \[0, \pi\]\\. The angle is
computed with [`atan2()`](https://rdrr.io/r/base/Trig.html) instead of
[`acos()`](https://rdrr.io/r/base/Trig.html), and for angles within
\\10^{-3}\\ of \\\pi\\ the axis is recovered from the symmetric part of
the matrix, so the map stays accurate near the cut locus where the
antisymmetric part vanishes. At exactly \\\pi\\ the two vectors \\\pm\pi
u\\ are equally valid; the one returned is deterministic but arbitrary.

## Usage

``` r
so3_exp(x)

so3_log(x, tol = 1e-06)

se3_exp(x)

se3_log(x, tol = 1e-06)
```

## Arguments

- x:

  Rotation vector or twist (for `exp`), or a rotation matrix,
  homogeneous transform, `robot_rotation` or `robot_pose` (for `log`).

- tol:

  Tolerance for validating the input group element.

## Value

A matrix (exp) or numeric vector (log).

## Details

`se3_exp()` maps a linear-first twist \\\xi = (v, \omega)\\ to a
homogeneous transform \\\[\exp(\hat\omega), J_l(\omega) v; 0, 1\]\\,
where \\J_l\\ is the left Jacobian of SO(3). `se3_log()` inverts it.

## Examples

``` r
R <- so3_exp(c(0, 0, pi / 2))
so3_log(R)
#> [1] 0.000000 0.000000 1.570796
T <- se3_exp(c(1, 0, 0, 0, 0, pi / 2))
se3_log(T)
#> [1] 1.000000e+00 6.558161e-17 0.000000e+00 0.000000e+00 0.000000e+00
#> [6] 1.570796e+00
```
