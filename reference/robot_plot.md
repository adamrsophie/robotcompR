# Plot a robot configuration

Draws the kinematic skeleton of a robot with base graphics: links as
segments between consecutive frame origins, joints as points and the end
effector as a filled marker. Planar projections (`"xy"`, `"xz"`, `"yz"`)
are exact orthographic views; `"3d"` uses a perspective view set by
`theta` and `phi` as in
[`graphics::persp()`](https://rdrr.io/r/graphics/persp.html). Several
configurations can be overlaid with `add = TRUE`. Works on any graphics
device, including headless ones.

## Usage

``` r
robot_plot(
  robot,
  q,
  projection = c("xy", "xz", "yz", "3d"),
  frames = FALSE,
  frame_scale = NULL,
  add = FALSE,
  col = "steelblue4",
  lwd = 3,
  xlim = NULL,
  ylim = NULL,
  theta = 30,
  phi = 20,
  main = NULL,
  ...
)
```

## Arguments

- robot:

  A `robot_model`.

- q:

  Configuration.

- projection:

  `"xy"`, `"xz"`, `"yz"`, or `"3d"`.

- frames:

  Draw coordinate axes of each link frame (red x, green y, blue z).

- frame_scale:

  Length of the frame axes.

- add:

  Add to the current plot.

- col:

  Link color.

- lwd:

  Link line width.

- xlim, ylim:

  Optional axis limits for planar projections.

- theta, phi:

  Viewing angles for `"3d"`.

- main:

  Plot title.

- ...:

  Further arguments passed to
  [`graphics::plot()`](https://rdrr.io/r/graphics/plot.default.html).

## Value

Invisibly, the matrix of projected link origins.

## Examples

``` r
robot <- robot_example("planar2")
robot_plot(robot, c(pi / 4, pi / 4))
robot_plot(robot, c(-pi / 4, pi / 2), add = TRUE, col = "grey50")
```
