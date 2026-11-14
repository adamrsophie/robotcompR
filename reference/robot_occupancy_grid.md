# Occupancy grid maps

Creates a 2D occupancy grid covering `xlim` by `ylim` with square cells
of side `resolution`. Cells store log-odds of occupancy, initialized
from the prior probability. Cell `[i, j]` covers `x` in
`xlim[1] + (j - 1, j) * resolution` and `y` in
`ylim[1] + (i - 1, i) * resolution` (rows are y, columns are x).

## Usage

``` r
robot_occupancy_grid(xlim, ylim, resolution, prior = 0.5, occupied = NULL)
```

## Arguments

- xlim, ylim:

  World extents (m).

- resolution:

  Cell size (m).

- prior:

  Prior occupancy probability.

- occupied:

  Optional logical matrix to initialize a known map (`TRUE` is occupied)
  with probabilities 0.99 and 0.01.

## Value

A `robot_occupancy_grid`.

## Examples

``` r
g <- robot_occupancy_grid(c(0, 5), c(0, 3), resolution = 0.1)
g
#> <robot_occupancy_grid> 30 by 50 cells of 0.1 m covering [0, 5] x [0, 3]
```
