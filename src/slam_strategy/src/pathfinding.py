"""
Range of pathfinding algorithms.

In each case, the given graph should be graphs.Graph subclass, implementing its
interface.
"""

def _rebuild_path(path_map, end):
    path = [end]
    current_source = path_map[end]
    while current_source is not None:
        path.insert(0, current_source)
        current_source = path_map[current_source]
    return path

def breadth_first_search(graph, source, end):
    """Breadth-First-Search algorithm."""
    to_visit = set([source])
    came_from = {source: None}
    while to_visit:
        vertex = to_visit.pop()
        if vertex == end:
            return _rebuild_path(came_from, end)
        for neighbor_vertex in graph.neighbors(vertex):
            if neighbor_vertex not in came_from:
                came_from[neighbor_vertex] = vertex
                to_visit.add(neighbor_vertex)

def dijkstra(graph, source, end):
    """Dijkstra's algorithm."""
    to_visit = set([source])
    came_from = {source: None}
    dist_to = {source: 0}
    for vertex in graph.vertices():
        if vertex != source:
            dist_to[vertex] = float('inf')
            came_from[vertex] = None
        to_visit.add(vertex)
    while to_visit:
        items = [(dist_to[v], v) for v in to_visit]
        vertex = min(to_visit, key=lambda v: dist_to[v])
        to_visit.remove(vertex)
        if vertex == end:
            return _rebuild_path(came_from, end)
        for n_vertex in graph.neighbors(vertex):
            alt = dist_to[vertex] + graph.transition_cost(vertex, n_vertex)
            if dist_to[n_vertex] is None or alt < dist_to[n_vertex]:
                dist_to[n_vertex] = alt
                came_from[n_vertex] = vertex

def astar(graph, source, end, heuristic):
    """A* algorithm."""
    open_set = set([source])
    came_from = {source: None}
    closed_set = set()
    g_score = {source: 0}
    f_score = {source: g_score[source] + heuristic(source, end)}
    while open_set:
        items = [(f_score[v], v) for v in open_set]
        vertex = min(open_set, key=lambda v: f_score[v])
        if vertex == end:
            return _rebuild_path(came_from, end)
        open_set.remove(vertex)
        closed_set.add(vertex)
        for n_vertex in graph.neighbors(vertex):
            if n_vertex in closed_set:
                continue
            alt_g_score = g_score[vertex] + graph.transition_cost(vertex, n_vertex)
            if n_vertex not in open_set or alt_g_score < g_score[n_vertex]:
                came_from[n_vertex] = vertex
                g_score[n_vertex] = alt_g_score
                f_score[n_vertex] = g_score[n_vertex] + heuristic(n_vertex, end)
                if n_vertex not in open_set:
                    open_set.add(n_vertex)
