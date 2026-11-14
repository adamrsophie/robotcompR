# A\* search on an occupancy grid

Finds a shortest 8-connected path between two world points through free
cells, with Euclidean step costs and the Euclidean distance as an
admissible heuristic. Diagonal moves are not allowed to cut corners of
occupied cells.

## Usage

``` r
grid_astar(grid, start, goal, occupied = NULL)
```

## Arguments

- grid:

  A `robot_occupancy_grid`.

- start, goal:

  World points.

- occupied:

  Optional logical occupancy matrix (for example from
  [`grid_inflate()`](https://adamrsophie.github.io/robotcompR/reference/grid_inflate.md));
  defaults to
  [`grid_occupied()`](https://adamrsophie.github.io/robotcompR/reference/grid_coordinates.md).

## Value

A matrix of world waypoints (cell centers), or `NULL` when the goal is
unreachable on the grid.

## Examples

``` r
occ <- matrix(FALSE, 20, 20); occ[3:18, 10] <- TRUE
g <- robot_occupancy_grid(c(0, 2), c(0, 2), 0.1, occupied = occ)
path <- grid_astar(g, c(0.25, 1), c(1.75, 1))
nrow(path)
#> [1] 19
```
