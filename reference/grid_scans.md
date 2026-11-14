# Simulate and integrate range scans

`grid_raycast()` returns, for each beam angle (relative to the robot
heading), the range to the first cell whose occupancy probability
exceeds `threshold`, or `max_range`. `grid_update_scan()` performs the
standard inverse sensor model update in log-odds: cells traversed by a
beam before the hit receive `l_free`, the hit cell receives `l_occ`;
beams at `max_range` only clear space. Log-odds are clamped to `clamp`.

## Usage

``` r
grid_raycast(grid, pose, angles, max_range, threshold = 0.65)

grid_update_scan(
  grid,
  pose,
  angles,
  ranges,
  max_range,
  l_occ = 0.85,
  l_free = -0.4,
  clamp = c(-5, 5)
)
```

## Arguments

- grid:

  A `robot_occupancy_grid`.

- pose:

  Sensor pose `c(x, y, theta)`.

- angles:

  Beam angles (rad) relative to the heading.

- max_range:

  Maximum sensor range.

- threshold:

  Occupancy threshold for ray casting.

- ranges:

  Measured ranges.

- l_occ, l_free:

  Log-odds increments.

- clamp:

  Log-odds bounds.

## Value

Ranges (`grid_raycast`) or an updated grid.

## Examples

``` r
occ <- matrix(FALSE, 20, 20)
occ[, 15] <- TRUE
truth <- robot_occupancy_grid(c(0, 2), c(0, 2), 0.1, occupied = occ)
grid_raycast(truth, c(0.5, 1, 0), 0, max_range = 3)
#> [1] 0.9
```
