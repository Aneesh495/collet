import collet/geometry/math_ffi
import collet/geometry/point.{type Point2D, Point2D}
import gleam/list

// Planar polygon defined by an ordered list of 2D vertices.
// In standard winding convention, outer boundaries are oriented counter-clockwise.
pub type Polygon {
  Polygon(vertices: List(Point2D))
}

// Axis-Aligned Bounding Box (AABB) for broad-phase spatial culling.
pub type AABB {
  AABB(min_x: Float, min_y: Float, max_x: Float, max_y: Float)
}

// Oriented Bounding Box (OBB) representing a rectangular robot physical footprint.
// Length is measured along the vehicle heading, width is perpendicular to heading.
pub type OrientedBoundingBox {
  OrientedBoundingBox(
    center: Point2D,
    width: Float,
    length: Float,
    heading: Float,
  )
}

// Creates a new polygon from a list of vertices.
pub fn new(vertices: List(Point2D)) -> Polygon {
  Polygon(vertices: vertices)
}

// Computes the tight axis-aligned bounding box enclosing all vertices of the polygon.
pub fn compute_aabb(p: Polygon) -> Result(AABB, Nil) {
  case p.vertices {
    [] -> Error(Nil)
    [first, ..rest] -> {
      let init_box =
        AABB(
          min_x: first.x,
          min_y: first.y,
          max_x: first.x,
          max_y: first.y,
        )
      let final_box =
        list.fold(rest, init_box, fn(box, v) {
          AABB(
            min_x: math_ffi.min_float(box.min_x, v.x),
            min_y: math_ffi.min_float(box.min_y, v.y),
            max_x: math_ffi.max_float(box.max_x, v.x),
            max_y: math_ffi.max_float(box.max_y, v.y),
          )
        })
      Ok(final_box)
    }
  }
}

// Checks if two axis-aligned bounding boxes overlap.
pub fn aabb_intersects(a: AABB, b: AABB) -> Bool {
  let disjoint_x = a.max_x <. b.min_x || b.max_x <. a.min_x
  let disjoint_y = a.max_y <. b.min_y || b.max_y <. a.min_y
  !disjoint_x && !disjoint_y
}

// Checks if an axis-aligned bounding box contains a point.
pub fn aabb_contains_point(box: AABB, pt: Point2D) -> Bool {
  pt.x >=. box.min_x
  && pt.x <=. box.max_x
  && pt.y >=. box.min_y
  && pt.y <=. box.max_y
}

// Computes the signed area of the polygon using the Shoelace formula.
// Positive area indicates counter-clockwise winding; negative indicates clockwise.
pub fn signed_area(p: Polygon) -> Float {
  let n = list.length(p.vertices)
  case n < 3 {
    True -> 0.0
    False -> {
      let edges = get_edges(p.vertices)
      let sum =
        list.fold(edges, 0.0, fn(acc, edge) {
          let #(v1, v2) = edge
          acc +. { v1.x *. v2.y -. v2.x *. v1.y }
        })
      sum /. 2.0
    }
  }
}

// Computes the positive geometric area of the polygon in square meters.
pub fn area(p: Polygon) -> Float {
  math_ffi.abs_float(signed_area(p))
}

// Checks if the polygon has a counter-clockwise vertex winding order.
pub fn is_counter_clockwise(p: Polygon) -> Bool {
  signed_area(p) >. 0.0
}

// Point-in-polygon containment test using the even-odd ray casting algorithm.
// Casts a horizontal ray from pt toward +infinity along the X axis.
pub fn contains_point(p: Polygon, pt: Point2D) -> Bool {
  let edges = get_edges(p.vertices)
  let crossings =
    list.fold(edges, 0, fn(count, edge) {
      let #(v1, v2) = edge
      let cond1 = v1.y <=. pt.y && pt.y <. v2.y
      let cond2 = v2.y <=. pt.y && pt.y <. v1.y
      case cond1 || cond2 {
        True -> {
          let x_intersect =
            v1.x +. { pt.y -. v1.y } /. { v2.y -. v1.y } *. { v2.x -. v1.x }
          case pt.x <. x_intersect {
            True -> count + 1
            False -> count
          }
        }
        False -> count
      }
    })
  // If number of ray crossings is odd, point is inside the polygon
  crossings % 2 != 0
}

// Computes the centroid (center of mass) of the planar polygon.
pub fn centroid(p: Polygon) -> Result(Point2D, Nil) {
  let s_area = signed_area(p)
  case math_ffi.abs_float(s_area) <. 0.000000001 {
    True -> Error(Nil)
    False -> {
      let edges = get_edges(p.vertices)
      let factor = 1.0 /. { 6.0 *. s_area }
      let #(cx_sum, cy_sum) =
        list.fold(edges, #(0.0, 0.0), fn(acc, edge) {
          let #(acc_x, acc_y) = acc
          let #(v1, v2) = edge
          let common = v1.x *. v2.y -. v2.x *. v1.y
          let dx = { v1.x +. v2.x } *. common
          let dy = { v1.y +. v2.y } *. common
          #(acc_x +. dx, acc_y +. dy)
        })
      Ok(Point2D(x: cx_sum *. factor, y: cy_sum *. factor))
    }
  }
}

// Converts an oriented bounding box into a 4-vertex convex quadrilateral polygon.
pub fn obb_to_polygon(obb: OrientedBoundingBox) -> Polygon {
  let half_len = obb.length /. 2.0
  let half_wid = obb.width /. 2.0
  let cos_h = math_ffi.cos(obb.heading)
  let sin_h = math_ffi.sin(obb.heading)

  // Local coordinate offsets:
  // Forward-Right (+half_len, -half_wid)
  // Forward-Left  (+half_len, +half_wid)
  // Rear-Left     (-half_len, +half_wid)
  // Rear-Right    (-half_len, -half_wid)
  let rotate_offset = fn(dx: Float, dy: Float) -> Point2D {
    Point2D(
      x: obb.center.x +. cos_h *. dx -. sin_h *. dy,
      y: obb.center.y +. sin_h *. dx +. cos_h *. dy,
    )
  }

  let p1 = rotate_offset(half_len, half_wid)
  let p2 = rotate_offset(0.0 -. half_len, half_wid)
  let p3 = rotate_offset(0.0 -. half_len, 0.0 -. half_wid)
  let p4 = rotate_offset(half_len, 0.0 -. half_wid)

  Polygon(vertices: [p1, p2, p3, p4])
}

// Tests if two convex polygons intersect using the Separating Axis Theorem (SAT).
pub fn polygons_intersect_sat(poly_a: Polygon, poly_b: Polygon) -> Bool {
  let axes_a = get_polygon_normals(poly_a.vertices)
  let axes_b = get_polygon_normals(poly_b.vertices)
  let all_axes = list.append(axes_a, axes_b)

  // If there exists any separating axis where projections do not overlap, they do not intersect
  has_no_separating_axis(all_axes, poly_a.vertices, poly_b.vertices)
}

// Expands vehicle footprint dimensions by a safety margin.
pub fn minkowski_expand_dimensions(
  width: Float,
  length: Float,
  safety_margin: Float,
) -> #(Float, Float) {
  let expanded_width = width +. 2.0 *. safety_margin
  let expanded_length = length +. 2.0 *. safety_margin
  #(expanded_width, expanded_length)
}

// Helper: extracts cyclic sequential edges (v_i, v_{i+1}) including wrap-around.
fn get_edges(vertices: List(Point2D)) -> List(#(Point2D, Point2D)) {
  case vertices {
    [] -> []
    [_single] -> []
    [first, ..rest] -> {
      let rolled = list.append(rest, [first])
      list.zip(vertices, rolled)
    }
  }
}

// Helper: extracts outward unit normal vectors for each edge of a polygon.
fn get_polygon_normals(vertices: List(Point2D)) -> List(Point2D) {
  let edges = get_edges(vertices)
  list.filter_map(edges, fn(edge) {
    let #(v1, v2) = edge
    let edge_vec = point.sub(v2, v1)
    // Perpendicular normal: (-edge_vec.y, edge_vec.x)
    let normal = Point2D(x: 0.0 -. edge_vec.y, y: edge_vec.x)
    point.normalize(normal)
  })
}

// Checks if projections overlap across all axes.
fn has_no_separating_axis(
  axes: List(Point2D),
  verts_a: List(Point2D),
  verts_b: List(Point2D),
) -> Bool {
  case axes {
    [] -> True
    [axis, ..rest] -> {
      let #(min_a, max_a) = project_onto_axis(verts_a, axis)
      let #(min_b, max_b) = project_onto_axis(verts_b, axis)
      // Check if separated
      case max_a <. min_b || max_b <. min_a {
        True -> False
        False -> has_no_separating_axis(rest, verts_a, verts_b)
      }
    }
  }
}

// Projects all vertices of a polygon onto a unit normal axis and finds [min, max] projection.
fn project_onto_axis(vertices: List(Point2D), axis: Point2D) -> #(Float, Float) {
  case vertices {
    [] -> #(0.0, 0.0)
    [first, ..rest] -> {
      let init_proj = point.dot(first, axis)
      list.fold(rest, #(init_proj, init_proj), fn(acc, v) {
        let #(acc_min, acc_max) = acc
        let p = point.dot(v, axis)
        #(math_ffi.min_float(acc_min, p), math_ffi.max_float(acc_max, p))
      })
    }
  }
}
