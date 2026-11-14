# Convert between degrees and radians

All angles in robotcompR are radians. These helpers make conversions
explicit; no function in the package interprets degrees silently.

## Usage

``` r
deg2rad(x)

rad2deg(x)
```

## Arguments

- x:

  Numeric vector.

## Value

Numeric vector.

## Examples

``` r
deg2rad(180)
#> [1] 3.141593
rad2deg(pi / 2)
#> [1] 90
```
