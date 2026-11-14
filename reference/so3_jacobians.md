# Jacobians of SO(3)

The left Jacobian \\J_l(\omega) = I + \frac{1 - \cos\theta}{\theta^2}
\hat\omega + \frac{\theta - \sin\theta}{\theta^3}\hat\omega^2\\
satisfies \\\exp(\widehat{\omega + \delta}) \approx \exp(\widehat{J_l
\delta}) \exp(\hat\omega)\\ for small \\\delta\\. The right Jacobian is
\\J_r(\omega) = J_l(-\omega)\\. Inverses use closed forms with Taylor
expansions near zero. The inverses are singular at \\\theta = 2\pi k\\,
\\k \ne 0\\; rotation vectors returned by
[`so3_log()`](https://adamrsophie.github.io/robotcompR/reference/lie_exp_log.md)
have \\\theta \le \pi\\ and never reach that singularity.

## Usage

``` r
so3_jacobian_left(x)

so3_jacobian_right(x)

so3_jacobian_left_inverse(x)

so3_jacobian_right_inverse(x)
```

## Arguments

- x:

  Rotation vector.

## Value

A 3 by 3 matrix.

## Examples

``` r
w <- c(0.1, -0.2, 0.3)
so3_jacobian_left(w) %*% so3_jacobian_left_inverse(w)
#>              [,1]          [,2]          [,3]
#> [1,] 1.000000e+00 -5.326896e-17 -1.032605e-16
#> [2,] 1.083719e-16  1.000000e+00  3.292370e-17
#> [3,] 2.750556e-17  9.087790e-17  1.000000e+00
```
