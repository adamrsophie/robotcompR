# Project a matrix onto the nearest rotation

Returns the rotation matrix closest to `x` in the Frobenius norm,
computed from the singular value decomposition \\x = U S V^\top\\ as \\U
\mathrm{diag}(1, \ldots, \det(UV^\top)) V^\top\\. Use it to remove drift
that accumulates after many compositions.

## Usage

``` r
orthonormalize(x)
```

## Arguments

- x:

  A square 2 by 2 or 3 by 3 numeric matrix.

## Value

A rotation matrix.

## Examples

``` r
R <- so3_exp(c(0.1, 0.2, 0.3)) + 1e-4
is_rotation_matrix(R, tol = 1e-8)
#> [1] FALSE
is_rotation_matrix(orthonormalize(R), tol = 1e-12)
#> [1] TRUE
```
