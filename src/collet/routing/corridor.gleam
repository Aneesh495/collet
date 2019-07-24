import collet/geometry/point.{type Point2D, Point2D}
import collet/geometry/polygon.{type Polygon, Polygon}
import collet/reservation/table.{type TimeInterval, TimeInterval}
import collet/routing/space_time_astar.{type TimedRouteNode}
import collet/topology/map.{type WarehouseMap}
import gleam/list
import gleam/option.{None, Some}

// Lateral boundary segment bounding an AMR's trajectory.
// Enables onboard local obstacle avoidance within guaranteed safe geometric corridors.
pub type CorridorSegment {
  CorridorSegment(
    start_point: Point2D,
    end_point: Point2D,
    boundary: Polygon,
    time_window: TimeInterval,
    half_width_meters: Float,
  )
}

// Full continuous safe navigation tube along an executing multi-agent route.
pub type NavigationCorridor {
  NavigationCorridor(
    segments: List(CorridorSegment),
    total_length_meters: Float,
  )
}

// Synthesizes a collision-free polygonal corridor around a planned route.
pub fn synthesize_corridor(
  route: List(TimedRouteNode),
  wmap: WarehouseMap,
  default_half_width_meters: Float,
) -> NavigationCorridor {
  let segments =
    build_corridor_segments(route, wmap, default_half_width_meters)
  let total_len =
    list.fold(segments, 0.0, fn(acc, s) {
      acc +. point.distance(s.start_point, s.end_point)
    })
  NavigationCorridor(segments: segments, total_length_meters: total_len)
}

// Verifies if the instantaneous vehicle position and timestamp are inside the corridor.
pub fn is_within_corridor(
  corridor: NavigationCorridor,
  position: Point2D,
  current_time_sec: Float,
) -> Bool {
  list.any(corridor.segments, fn(seg) {
    let in_time =
      current_time_sec >=. seg.time_window.start_sec -. 0.5
      && current_time_sec <=. seg.time_window.end_sec +. 0.5
    case in_time {
      False -> False
      True -> polygon.contains_point(seg.boundary, position)
    }
  })
}

// Extracts all boundary polygons of the corridor for visual rendering and SAT broad-phase.
pub fn to_polygons(corridor: NavigationCorridor) -> List(Polygon) {
  list.map(corridor.segments, fn(s) { s.boundary })
}

// Internal recursive builder pairing sequential route nodes.
fn build_corridor_segments(
  route: List(TimedRouteNode),
  wmap: WarehouseMap,
  half_width: Float,
) -> List(CorridorSegment) {
  case route {
    [] -> []
    [_single] -> []
    [u, ..rest] -> {
      case list.first(rest) {
        Error(Nil) -> []
        Ok(v) -> {
          case map.get_node(wmap, u.node_id), map.get_node(wmap, v.node_id) {
            Ok(node_u), Ok(node_v) -> {
              let seg_half_w = case v.traversing_edge_id {
                None -> half_width
                Some(eid) ->
                  case map.get_edge(wmap, eid) {
                    Ok(e) -> e.corridor_width /. 2.0
                    Error(Nil) -> half_width
                  }
              }

              let seg_polygon =
                construct_segment_polygon(
                  node_u.position,
                  node_v.position,
                  seg_half_w,
                )
              let time_win =
                TimeInterval(u.arrival_time_sec, v.arrival_time_sec)
              let corridor_seg =
                CorridorSegment(
                  start_point: node_u.position,
                  end_point: node_v.position,
                  boundary: seg_polygon,
                  time_window: time_win,
                  half_width_meters: seg_half_w,
                )
              [
                corridor_seg,
                ..build_corridor_segments(rest, wmap, half_width)
              ]
            }
            _, _ -> build_corridor_segments(rest, wmap, half_width)
          }
        }
      }
    }
  }
}

// Constructs a 4-vertex rectangular polygon centered along a line segment with width 2 * half_w.
fn construct_segment_polygon(
  p_start: Point2D,
  p_end: Point2D,
  half_w: Float,
) -> Polygon {
  let dir = point.sub(p_end, p_start)
  case point.normalize(dir) {
    Error(Nil) -> {
      // Degenerate segment: box centered at start
      Polygon(vertices: [
        Point2D(p_start.x -. half_w, p_start.y -. half_w),
        Point2D(p_start.x +. half_w, p_start.y -. half_w),
        Point2D(p_start.x +. half_w, p_start.y +. half_w),
        Point2D(p_start.x -. half_w, p_start.y +. half_w),
      ])
    }
    Ok(unit_dir) -> {
      // Perpendicular normal: (-unit_dir.y, unit_dir.x)
      let normal = Point2D(0.0 -. unit_dir.y, unit_dir.x)
      let offset = point.scale(normal, half_w)

      let v1 = point.add(p_start, offset)
      let v2 = point.add(p_end, offset)
      let v3 = point.sub(p_end, offset)
      let v4 = point.sub(p_start, offset)

      Polygon(vertices: [v1, v2, v3, v4])
    }
  }
}
