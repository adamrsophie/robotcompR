# Collision and visual geometry

Constructors for geometric primitives. Each is defined in its own frame,
placed by `origin` relative to the frame it is attached to (a link or
the world):

## Usage

``` r
geom_sphere(radius, origin = robot_pose())

geom_box(size, origin = robot_pose())

geom_cylinder(radius, length, origin = robot_pose())

geom_capsule(radius, length, origin = robot_pose())

geom_convex(vertices, origin = robot_pose())

geom_mesh(filename, scale = c(1, 1, 1), origin = robot_pose())
```

## Arguments

- radius, length:

  Dimensions in meters.

- origin:

  Pose of the geometry frame.

- size:

  Box edge lengths `c(x, y, z)`.

- vertices:

  Matrix with one 3D vertex per row.

- filename:

  Mesh file name (not read).

- scale:

  Mesh scale factors.

## Value

A `robot_geometry` object.

## Details

- `geom_sphere(radius)`: centered at the origin.

- `geom_box(size)`: an axis-aligned box in its own frame with edge
  lengths `size`, centered at the origin. A rotated `origin` gives an
  oriented box.

- `geom_cylinder(radius, length)`: axis along z, centered at the origin.

- `geom_capsule(radius, length)`: a segment of length `length` along z,
  centered at the origin, swept by a sphere of `radius`.

- `geom_convex(vertices)`: the convex hull of the rows of `vertices`.

- `geom_mesh(filename, scale)`: a reference to a mesh file, kept as
  metadata only. Mesh files are never opened, and meshes are ignored by
  collision checking.

## Examples

``` r
geom_box(c(0.2, 0.1, 0.1), origin = transl(0.5, 0, 0))
#> <robot_geometry> box: size (0.2, 0.1, 0.1), at (0.5, 0, 0)
geom_capsule(0.05, 0.4)
#> <robot_geometry> capsule: radius 0.05, length 0.4, at (0, 0, 0)
```
