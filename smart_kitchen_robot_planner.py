import networkx as nx
import matplotlib.pyplot as plt
from queue import Queue, PriorityQueue
import time

# Define actions
actions = {
    "Fetch vegetables": {"pre": [], "effect": (1,0,0,0,0,0), "cost": 2},
    "Wash vegetables": {"pre": [0], "effect": (0,1,0,0,0,0), "cost": 3},
    "Chop vegetables": {"pre": [1], "effect": (0,0,1,0,0,0), "cost": 4},
    "Boil water": {"pre": [], "effect": (0,0,0,1,0,0), "cost": 5},
    "Add vegetables": {"pre": [2,3], "effect": (0,0,0,0,1,0), "cost": 1},
    "Cook soup": {"pre": [4], "effect": (0,0,0,0,0,1), "cost": 6}
}

initial_state = (0,0,0,0,0,0)
goal_state = (1,1,1,1,1,1)

def is_goal(state):
    return state == goal_state

def apply_action(state, action):
    pre = actions[action]["pre"]
    for i in pre:
        if state[i] == 0:
            return None
    effect = actions[action]["effect"]
    new_state = tuple(state[i] | effect[i] for i in range(len(state)))
    return new_state

# BFS Implementation
def bfs(initial):
    queue = Queue()
    queue.put((initial, [], 0))  # state, path, cost
    visited = set()
    nodes_expanded = 0

    G = nx.DiGraph()
    G.add_node(initial)

    while not queue.empty():
        state, path, cost = queue.get()
        if state in visited:
            continue
        visited.add(state)
        nodes_expanded += 1

        if is_goal(state):
            return path, cost, nodes_expanded, G

        for action in actions:
            new_state = apply_action(state, action)
            if new_state and new_state not in visited:
                queue.put((new_state, path + [(action, new_state)], cost + actions[action]["cost"]))
                G.add_node(new_state)
                G.add_edge(state, new_state, label=action)
    return None, 0, nodes_expanded, G

# A* Implementation
def heuristic(state):
    remaining = sum(1 for i in range(len(state)) if state[i] == 0)
    min_cost = min([actions[a]["cost"] for a in actions])
    return remaining * min_cost

def a_star(initial):
    pq = PriorityQueue()
    pq.put((0 + heuristic(initial), 0, initial, []))  # f(n), g(n), state, path
    visited = {}
    nodes_expanded = 0

    G = nx.DiGraph()
    G.add_node(initial)

    while not pq.empty():
        f, g, state, path = pq.get()
        if state in visited and visited[state] <= g:
            continue
        visited[state] = g
        nodes_expanded += 1

        if is_goal(state):
            return path, g, nodes_expanded, G

        for action in actions:
            new_state = apply_action(state, action)
            if new_state:
                g_new = g + actions[action]["cost"]
                f_new = g_new + heuristic(new_state)
                pq.put((f_new, g_new, new_state, path + [(action, new_state)]))
                G.add_node(new_state)
                G.add_edge(state, new_state, label=action)
    return None, 0, nodes_expanded, G

# Visualization function
def draw_graph(G, path, filename="graph.png", initial_state=None):
    """Draw the graph to a PNG file.

    - path is a list of (action, to_state) tuples (as used in the searches).
    - initial_state (optional) is used so the first edge on the path is highlighted.
    - Node labels are shortened to s0, s1... to keep the plot readable.
    """
    pos = nx.spring_layout(G, seed=42)
    plt.figure(figsize=(12,8))

    # Map full state tuples to short labels (s0, s1, ...)
    nodes = list(G.nodes())
    label_map = {nodes[i]: f"s{i}" for i in range(len(nodes))}

    # Draw nodes with short labels
    nx.draw(G, pos, labels=label_map, with_labels=True, node_size=1000, node_color='lightblue')
    edge_labels = nx.get_edge_attributes(G, 'label')
    # Replace edge label keys with short names for readability in the plot
    short_edge_labels = {}
    for (u, v), lbl in edge_labels.items():
        short_u = label_map.get(u, str(u))
        short_v = label_map.get(v, str(v))
        short_edge_labels[(u, v)] = lbl
    nx.draw_networkx_edge_labels(G, pos, edge_labels=short_edge_labels, font_size=8)

    # Build the list of edges that form the path. Use initial_state to include first edge.
    path_edges = []
    if path:
        prev_state = initial_state
        # If initial_state not provided, try to infer from graph (best-effort)
        if prev_state is None:
            # If path contains nodes, attempt to find a node in G that is not the first 'to' node
            # This is a fallback and may not always work — providing initial_state is best.
            prev_state = None
        for action, new_state in path:
            if prev_state is not None:
                path_edges.append((prev_state, new_state))
            prev_state = new_state

    # Draw highlighted path edges (if any) in red. Use short labels for selecting edges.
    if path_edges:
        # Ensure edges exist in the graph before drawing
        valid_edges = [e for e in path_edges if G.has_edge(e[0], e[1])]
        nx.draw_networkx_edges(G, pos, edgelist=valid_edges, edge_color='red', width=2)

    # Save and close
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    plt.close()

# Run BFS
start_time = time.time()
bfs_path, bfs_cost, bfs_nodes, bfs_graph = bfs(initial_state)
bfs_time = time.time() - start_time

print("BFS Path:", [a for a,_ in bfs_path])
print("BFS Total Cost:", bfs_cost)
print("BFS Nodes Expanded:", bfs_nodes)
print("BFS Execution Time (s):", round(bfs_time,6))
draw_graph(bfs_graph, bfs_path, "bfs_graph.png", initial_state=initial_state)

# Run A*
start_time = time.time()
astar_path, astar_cost, astar_nodes, astar_graph = a_star(initial_state)
astar_time = time.time() - start_time

print("A* Path:", [a for a,_ in astar_path])
print("A* Total Cost:", astar_cost)
print("A* Nodes Expanded:", astar_nodes)
print("A* Execution Time (s):", round(astar_time,6))
draw_graph(astar_graph, astar_path, "astar_graph.png", initial_state=initial_state)

# Comparison Table
print("\nComparison of BFS vs A*:")
print("{:<10} {:<10} {:<15} {:<15}".format("Algorithm","Cost","Nodes Expanded","Time (s)"))
print("{:<10} {:<10} {:<15} {:<15}".format("BFS", bfs_cost, bfs_nodes, round(bfs_time,6)))
print("{:<10} {:<10} {:<15} {:<15}".format("A*", astar_cost, astar_nodes, round(astar_time,6)))
