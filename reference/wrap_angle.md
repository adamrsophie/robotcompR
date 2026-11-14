# Wrap angles to the interval from minus pi to pi

Maps angles in radians to the half-open interval \\\[-\pi, \pi)\\.

## Usage

``` r
wrap_angle(theta)
```

## Arguments

- theta:

  Numeric vector of angles in radians.

## Value

Numeric vector of wrapped angles.

## Examples

``` r
wrap_angle(c(0, 3 * pi / 2, -3 * pi / 2, 7))
#> [1]  0.0000000 -1.5707963  1.5707963  0.7168147
```
