# Hat and vee operators

`so3_hat()` maps a 3-vector to the skew-symmetric matrix with
`so3_hat(w) %*% v == cross(w, v)`; `so3_vee()` is its inverse.
`se3_hat()` maps a linear-first twist `c(v, w)` to the 4 by 4 matrix
`[[hat(w), v], [0, 0]]`, and `se3_vee()` inverts it. `so2_hat()`,
`se2_hat()` and their vee operators are the planar analogues, with
planar twists ordered `c(vx, vy, w)`.

## Usage

``` r
so3_hat(x)

so3_vee(x, tol = 1e-08)

se3_hat(x)

se3_vee(x, tol = 1e-08)

so2_hat(x)

so2_vee(x, tol = 1e-08)

se2_hat(x)

se2_vee(x, tol = 1e-08)
```

## Arguments

- x:

  Numeric vector (hat) or matrix (vee).

- tol:

  Tolerance used by vee operators to check skew symmetry.

## Value

A matrix (hat) or numeric vector (vee).

## Examples

``` r
W <- so3_hat(c(1, 2, 3))
so3_vee(W)
#> [1] 1 2 3
se3_vee(se3_hat(c(1, 0, 0, 0, 0, 1)))
#> [1] 1 0 0 0 0 1
```
