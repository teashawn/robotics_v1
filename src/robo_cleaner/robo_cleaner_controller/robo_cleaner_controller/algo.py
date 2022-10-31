from robo_cleaner_controller import models
import math
import sys

# Credits: https://math.stackexchange.com/a/3448361
def GenSpiral(x, y):
    for n in range(sys.maxsize):
        k = math.ceil((math.sqrt(n) - 1) / 2.0)
        t = 2 * k + 1
        m = t ** 2
        t = t - 1
        if n >= m - t:
            yield x + k - (m - n), y - k
        else:
            m = m - t
        if n >= m - t:
            yield x + -k, y -k + (m - n)
        else:
            m = m - t
        if n >= m - t:
            yield x -k + (m - n), y + k
        else:
            yield x + k, y + k - (m - n - t)

"""
orientation: target -> rotations

1-st level keys designate the current robot direction
2-nd level keys are the desired direction of the robot

The innermost values are number of turns that need
to be performed in order to reach the target direction
"""
REORIENTATION_COSTS = {
    models.ROBOT_DIRECTION.UP: {
        models.ROBOT_DIRECTION.UP: 0,
        models.ROBOT_DIRECTION.RIGHT: 1,
        models.ROBOT_DIRECTION.DOWN: 2,
        models.ROBOT_DIRECTION.LEFT: 1,
    },
    models.ROBOT_DIRECTION.RIGHT: {
        models.ROBOT_DIRECTION.UP: 1,
        models.ROBOT_DIRECTION.RIGHT: 0,
        models.ROBOT_DIRECTION.DOWN: 1,
        models.ROBOT_DIRECTION.LEFT: 2,
    },
    models.ROBOT_DIRECTION.DOWN: {
        models.ROBOT_DIRECTION.UP: 2,
        models.ROBOT_DIRECTION.RIGHT: 1,
        models.ROBOT_DIRECTION.DOWN: 0,
        models.ROBOT_DIRECTION.LEFT: 1,
    },
    models.ROBOT_DIRECTION.LEFT: {
        models.ROBOT_DIRECTION.UP: 1,
        models.ROBOT_DIRECTION.RIGHT: 2,
        models.ROBOT_DIRECTION.DOWN: 1,
        models.ROBOT_DIRECTION.LEFT: 0,
    },
}

# Credits: original version: https://networkx.org/documentation/stable/_modules/networkx/algorithms/shortest_paths/astar.html
# Below is a modified version to consider turns cost
"""Shortest paths and path lengths using the A* ("A star") algorithm.
"""
from heapq import heappop, heappush
from itertools import count

import networkx as nx
from networkx.algorithms.shortest_paths.weighted import _weight_function

__all__ = ["astar_path", "astar_path_length"]

def astar_path(G, source, target, cur_dir : models.ROBOT_DIRECTION, heuristic=None, weight="weight"):
    """Returns a list of nodes in a shortest path between source and target
    using the A* ("A-star") algorithm.

    There may be more than one shortest path.  This returns only one.

    Parameters
    ----------
    G : NetworkX graph

    source : node
       Starting node for path

    target : node
       Ending node for path

    cur_dir : models.ROBOT_DIRECTION
        Current robot direction. Used for calculating turn cost for edges.

    heuristic : function
       A function to evaluate the estimate of the distance
       from the a node to the target.  The function takes
       two nodes arguments and must return a number.
       If the heuristic is inadmissible (if it might
       overestimate the cost of reaching the goal from a node),
       the result may not be a shortest path.
       The algorithm does not support updating heuristic
       values for the same node due to caching the first
       heuristic calculation per node.

    weight : string or function
       If this is a string, then edge weights will be accessed via the
       edge attribute with this key (that is, the weight of the edge
       joining `u` to `v` will be ``G.edges[u, v][weight]``). If no
       such edge attribute exists, the weight of the edge is assumed to
       be one.
       If this is a function, the weight of an edge is the value
       returned by the function. The function must accept exactly three
       positional arguments: the two endpoints of an edge and the
       dictionary of edge attributes for that edge. The function must
       return a number.

    Raises
    ------
    NetworkXNoPath
        If no path exists between source and target.

    Examples
    --------
    >>> G = nx.path_graph(5)
    >>> print(nx.astar_path(G, 0, 4))
    [0, 1, 2, 3, 4]
    >>> G = nx.grid_graph(dim=[3, 3])  # nodes are two-tuples (x,y)
    >>> nx.set_edge_attributes(G, {e: e[1][0] * 2 for e in G.edges()}, "cost")
    >>> def dist(a, b):
    ...     (x1, y1) = a
    ...     (x2, y2) = b
    ...     return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
    >>> print(nx.astar_path(G, (0, 0), (2, 2), heuristic=dist, weight="cost"))
    [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2)]


    See Also
    --------
    shortest_path, dijkstra_path

    """
    if source not in G or target not in G:
        msg = f"Either source {source} or target {target} is not in G"
        raise nx.NodeNotFound(msg)

    if heuristic is None:
        # The default heuristic is h=0 - same as Dijkstra's algorithm
        def heuristic(u, v):
            return 0

    push = heappush
    pop = heappop
    weight = _weight_function(G, weight)

    # The queue stores priority, node, cost to reach, and parent.
    # Uses Python heapq to keep in priority order.
    # Add a counter to the queue to prevent the underlying heap from
    # attempting to compare the nodes themselves. The hash breaks ties in the
    # priority and is guaranteed unique for all nodes in the graph.
    c = count()
    # priority, node, cost to reach, parent, node direction
    queue = [(0, next(c), source, 0, None, cur_dir)]

    # Maps enqueued nodes to distance of discovered paths and the
    # computed heuristics to target. We avoid computing the heuristics
    # more than once and inserting the node into the queue too many times.
    enqueued = {}
    # Maps explored nodes to parent closest to the source.
    explored = {}

    while queue:
        # Pop the smallest item from queue.
        # priority, node, cost to reach, parent, node direction
        _, __, curnode, dist, parent, curnodedir = pop(queue)

        if curnode == target:
            path = [curnode]
            node = parent
            while node is not None:
                path.append(node)
                node = explored[node]
            path.reverse()
            return path

        if curnode in explored:
            # Do not override the parent of starting node
            if explored[curnode] is None:
                continue

            # Skip bad paths that were enqueued before finding a better one
            qcost, h = enqueued[curnode]
            if qcost < dist:
                continue

        explored[curnode] = parent

        for neighbor, w in G[curnode].items():
            neighbor_dir, turn_aware_cost = get_turn_aware_cost(curnode, neighbor, curnodedir)
            ncost = dist + turn_aware_cost
            if neighbor in enqueued:
                qcost, h = enqueued[neighbor]
                # if qcost <= ncost, a less costly path from the
                # neighbor to the source was already determined.
                # Therefore, we won't attempt to push this neighbor
                # to the queue
                if qcost <= ncost:
                    continue
            else:
                h = heuristic(neighbor, target)
            enqueued[neighbor] = ncost, h
            # priority, node, cost to reach, parent, node direction
            push(queue, (ncost + h, next(c), neighbor, ncost, curnode, neighbor_dir))

    raise nx.NetworkXNoPath(f"Node {target} not reachable from {source}")

def get_turn_aware_cost(source, destination, source_dir):
    cost = 1

    if source.row == destination.row:
        # same row
        if source.column < destination.column:
            # right
            cost += REORIENTATION_COSTS[source_dir][models.ROBOT_DIRECTION.RIGHT]
            return models.ROBOT_DIRECTION.RIGHT, cost
        elif source.column > destination.column:
            # left
            cost += REORIENTATION_COSTS[source_dir][models.ROBOT_DIRECTION.LEFT]
            return models.ROBOT_DIRECTION.LEFT, cost
    elif source.row > destination.row:
        # warning: skipping checks for column, because the algorithm
        # is only used for grid map without diagonal moves!

        # down
        cost += REORIENTATION_COSTS[source_dir][models.ROBOT_DIRECTION.DOWN]
        return models.ROBOT_DIRECTION.DOWN, cost
    elif source.row < destination.row:
        # warning: skipping checks for column, because the algorithm
        # is only used for grid map without diagonal moves!

        # up
        cost += REORIENTATION_COSTS[source_dir][models.ROBOT_DIRECTION.UP]
        return models.ROBOT_DIRECTION.UP, cost