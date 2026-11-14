# Adjoint representation of SE(3) and SE(2)

For \\T = (R, p)\\, the adjoint maps a linear-first twist expressed in
the child frame to the same twist expressed in the parent frame: \\Ad_T
= \[R, \hat p R; 0, R\]\\. For SE(2) with twists `c(vx, vy, w)`, \\Ad_T
= \[R, (p_y, -p_x)^T; 0, 1\]\\.

## Usage

``` r
se3_adjoint(x)

se2_adjoint(x)
```

## Arguments

- x:

  A homogeneous transform or `robot_pose`.

## Value

A 6 by 6 (SE(3)) or 3 by 3 (SE(2)) matrix.

## Examples

``` r
se3_adjoint(se3_exp(c(1, 2, 3, 0.1, 0.2, 0.3)))
#>            [,1]       [,2]        [,3]       [,4]       [,5]        [,6]
#> [1,]  0.9357548 -0.2831650  0.21019171 -1.2698783 -2.5970727  2.15467457
#> [2,]  0.3029327  0.9505806 -0.06803132  2.9878045 -0.9768295 -0.34471519
#> [3,] -0.1805401  0.1273346  0.97529031 -1.5685769  1.5169105 -0.48841473
#> [4,]  0.0000000  0.0000000  0.00000000  0.9357548 -0.2831650  0.21019171
#> [5,]  0.0000000  0.0000000  0.00000000  0.3029327  0.9505806 -0.06803132
#> [6,]  0.0000000  0.0000000  0.00000000 -0.1805401  0.1273346  0.97529031
```
