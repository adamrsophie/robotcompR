# Grid coordinate conversion and probabilities

`grid_world_to_cell()` maps world points to `c(row, col)` indices (`NA`
outside the map); `grid_cell_to_world()` returns cell centers.
`grid_probability()` returns the occupancy probability matrix.
`grid_occupied()` thresholds it.

## Usage

``` r
grid_world_to_cell(grid, points)

grid_cell_to_world(grid, cells)

grid_probability(grid)

grid_occupied(grid, threshold = 0.65)
```

## Arguments

- grid:

  A `robot_occupancy_grid`.

- points:

  Matrix of world points (one per row) or one point.

- cells:

  Matrix of `c(row, col)` indices.

- threshold:

  Probability threshold.

## Value

Matrices.

## Examples

``` r
g <- robot_occupancy_grid(c(0, 1), c(0, 1), 0.25)
grid_world_to_cell(g, c(0.3, 0.9))
#>      row col
#> [1,]   4   2
grid_cell_to_world(g, cbind(1, 1))
#>          x     y
#> [1,] 0.125 0.125
```
