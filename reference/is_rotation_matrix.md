# Test whether a matrix is a rotation matrix

Checks \\R^\top R = I\\ and \\\det R = 1\\ within `tol`.

## Usage

``` r
is_rotation_matrix(x, tol = 1e-06)
```

## Arguments

- x:

  A numeric matrix or `robot_rotation`.

- tol:

  Absolute tolerance.

## Value

`TRUE` or `FALSE`.

## Examples

``` r
is_rotation_matrix(diag(3))
#> [1] TRUE
is_rotation_matrix(diag(c(1, 1, -1)))
#> [1] FALSE
```
