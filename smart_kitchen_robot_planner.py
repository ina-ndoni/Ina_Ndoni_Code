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
def draw_graph(G, path, filename="graph.png"):
    pos = nx.spring_layout(G, seed=42)
    plt.figure(figsize=(12,8))
    nx.draw(G, pos, with_labels=True, node_size=1000, node_color='lightblue')
    edge_labels = nx.get_edge_attributes(G,'label')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=8)

    path_edges = []
    if path:
        prev_state = None
        for action, new_state in path:
            if prev_state is not None:
                path_edges.append((prev_state, new_state))
            prev_state = new_state
    nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='red', width=2)
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
draw_graph(bfs_graph, bfs_path, "bfs_graph.png")

# Run A*
start_time = time.time()
astar_path, astar_cost, astar_nodes, astar_graph = a_star(initial_state)
astar_time = time.time() - start_time

print("A* Path:", [a for a,_ in astar_path])
print("A* Total Cost:", astar_cost)
print("A* Nodes Expanded:", astar_nodes)
print("A* Execution Time (s):", round(astar_time,6))
draw_graph(astar_graph, astar_path, "astar_graph.png")

# Comparison Table
print("\nComparison of BFS vs A*:")
print("{:<10} {:<10} {:<15} {:<15}".format("Algorithm","Cost","Nodes Expanded","Time (s)"))
print("{:<10} {:<10} {:<15} {:<15}".format("BFS", bfs_cost, bfs_nodes, round(bfs_time,6)))
print("{:<10} {:<10} {:<15} {:<15}".format("A*", astar_cost, astar_nodes, round(astar_time,6)))
