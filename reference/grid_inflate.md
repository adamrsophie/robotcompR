# Inflate obstacles

Marks every cell whose center lies closer than `radius` to an occupied
cell (treated as a solid square, not a point) as occupied, so a
disc-shaped robot of that radius placed at any remaining free cell
center does not overlap an occupied cell. Plan the robot as a point on
the result.

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
