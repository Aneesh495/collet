import collet/geometry/point.{type Point2D}
import collet/topology/edge.{type Edge, type EdgeId}
import collet/topology/node.{type Node, type NodeId}
import gleam/dict.{type Dict}
import gleam/list

// Unique identifier for a floor map topology version.
pub type MapId =
  String

// Complete topological warehouse navigation graph.
// Contains spatial nodes, directed edges, and bidirectional adjacency indices.
pub type WarehouseMap {
  WarehouseMap(
    id: MapId,
    name: String,
    nodes: Dict(NodeId, Node),
    edges: Dict(EdgeId, Edge),
    // Outgoing edge index: StartNodeId -> List(EdgeId)
    outgoing: Dict(NodeId, List(EdgeId)),
    // Incoming edge index: EndNodeId -> List(EdgeId)
    incoming: Dict(NodeId, List(EdgeId)),
  )
}

// Initializes an empty warehouse topological map.
pub fn new(id: MapId, name: String) -> WarehouseMap {
  WarehouseMap(
    id: id,
    name: name,
    nodes: dict.new(),
    edges: dict.new(),
    outgoing: dict.new(),
    incoming: dict.new(),
  )
}

// Inserts a node into the topological graph.
pub fn add_node(map: WarehouseMap, node: Node) -> WarehouseMap {
  let updated_nodes = dict.insert(map.nodes, node.id, node)
  WarehouseMap(..map, nodes: updated_nodes)
}

// Inserts a directed edge and updates the adjacency indices.
pub fn add_edge(map: WarehouseMap, edge: Edge) -> WarehouseMap {
  let updated_edges = dict.insert(map.edges, edge.id, edge)

  let current_out = case dict.get(map.outgoing, edge.start_node_id) {
    Ok(list) -> list
    Error(Nil) -> []
  }
  let updated_out =
    dict.insert(map.outgoing, edge.start_node_id, [edge.id, ..current_out])

  let current_in = case dict.get(map.incoming, edge.end_node_id) {
    Ok(list) -> list
    Error(Nil) -> []
  }
  let updated_in =
    dict.insert(map.incoming, edge.end_node_id, [edge.id, ..current_in])

  WarehouseMap(
    ..map,
    edges: updated_edges,
    outgoing: updated_out,
    incoming: updated_in,
  )
}

// Inserts a bidirectional edge (creates forward edge and reverse edge).
pub fn add_bidirectional_edge(
  map: WarehouseMap,
  forward_edge: Edge,
  reverse_id: EdgeId,
) -> WarehouseMap {
  let rev_edge = edge.reverse(forward_edge, reverse_id)
  map
  |> add_edge(forward_edge)
  |> add_edge(rev_edge)
}

// Looks up a node by its unique identifier.
pub fn get_node(map: WarehouseMap, id: NodeId) -> Result(Node, Nil) {
  dict.get(map.nodes, id)
}

// Looks up an edge by its unique identifier.
pub fn get_edge(map: WarehouseMap, id: EdgeId) -> Result(Edge, Nil) {
  dict.get(map.edges, id)
}

// Retrieves all outgoing neighbor nodes and traversing edges from a source node.
pub fn get_neighbors(
  map: WarehouseMap,
  node_id: NodeId,
) -> List(#(Node, Edge)) {
  case dict.get(map.outgoing, node_id) {
    Error(Nil) -> []
    Ok(edge_ids) ->
      list.filter_map(edge_ids, fn(eid) {
        case dict.get(map.edges, eid) {
          Error(Nil) -> Error(Nil)
          Ok(e) ->
            case e.is_blocked {
              True -> Error(Nil)
              False ->
                case dict.get(map.nodes, e.end_node_id) {
                  Error(Nil) -> Error(Nil)
                  Ok(target_node) -> Ok(#(target_node, e))
                }
            }
        }
      })
  }
}

// Finds the nearest topological node to a continuous spatial coordinate query.
pub fn nearest_node(map: WarehouseMap, query: Point2D) -> Result(Node, Nil) {
  let all_nodes = dict.values(map.nodes)
  case all_nodes {
    [] -> Error(Nil)
    [first, ..rest] -> {
      let init_dist = point.distance(query, first.position)
      let #(closest, _) =
        list.fold(rest, #(first, init_dist), fn(acc, n) {
          let #(_curr_node, curr_dist) = acc
          let d = point.distance(query, n.position)
          case d <. curr_dist {
            True -> #(n, d)
            False -> acc
          }
        })
      Ok(closest)
    }
  }
}

// Validates the structural integrity of the topological graph.
// Returns a list of error strings describing any broken edge endpoints or isolated nodes.
pub fn validate(map: WarehouseMap) -> List(String) {
  let edge_errors =
    dict.values(map.edges)
    |> list.filter_map(fn(e) {
      let start_missing = case dict.get(map.nodes, e.start_node_id) {
        Error(Nil) -> True
        Ok(_) -> False
      }
      let end_missing = case dict.get(map.nodes, e.end_node_id) {
        Error(Nil) -> True
        Ok(_) -> False
      }
      case start_missing, end_missing {
        True, True ->
          Ok("Edge " <> e.id <> " references non-existent start and end nodes")
        True, False ->
          Ok("Edge " <> e.id <> " references non-existent start node " <> e.start_node_id)
        False, True ->
          Ok("Edge " <> e.id <> " references non-existent end node " <> e.end_node_id)
        False, False -> Error(Nil)
      }
    })

  edge_errors
}
