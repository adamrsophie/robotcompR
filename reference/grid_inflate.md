# Inflate obstacles

Marks every cell within `radius` of an occupied cell as occupied, for
planning a disc-shaped robot as a point.

## Usage

``` r
grid_inflate(grid, radius, threshold = 0.65)
```

## Arguments

- grid:

  A `robot_occupancy_grid`.

- radius:

  Inflation radius (m).

- threshold:

  Occupancy threshold.

## Value

A logical matrix of inflated occupancy.

## Examples

``` r
occ <- matrix(FALSE, 10, 10); occ[5, 5] <- TRUE
g <- robot_occupancy_grid(c(0, 1), c(0, 1), 0.1, occupied = occ)
sum(grid_inflate(g, 0.15))
#> [1] 9
```
