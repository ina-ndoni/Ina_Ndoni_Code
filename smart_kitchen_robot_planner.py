"""
Smart Kitchen Robot Planner
=============================
This module implements two graph search algorithms (BFS and A*) to plan a sequence 
of kitchen actions that transforms the initial state to the goal state.

State representation:
  - A tuple of 6 binary values (0 or 1), where each index corresponds to:
    0: vegetables_fetched
    1: vegetables_washed
    2: vegetables_chopped
    3: water_boiled
    4: vegetables_added_to_pot
    5: soup_cooked

Each action has:
  - "pre": list of state indices that must be 1 (satisfied preconditions) to apply the action
  - "effect": tuple of 6 values (0 or 1) that are OR'ed with the current state
  - "cost": numeric cost to apply the action

The goal is to find a sequence of actions that transform initial_state (all 0s) 
to goal_state (all 1s), minimizing total cost.
"""

import networkx as nx
import matplotlib.pyplot as plt
import logging
from queue import Queue, PriorityQueue
import time

# ===== Problem Definition =====
# Define available actions for the kitchen robot planner.
# Each action specifies preconditions (state indices that must be 1), 
# effects (state bits to set to 1), and a cost.
actions = {
    "Fetch vegetables": {"pre": [], "effect": (1,0,0,0,0,0), "cost": 2},
    "Wash vegetables": {"pre": [0], "effect": (0,1,0,0,0,0), "cost": 3},
    "Chop vegetables": {"pre": [1], "effect": (0,0,1,0,0,0), "cost": 4},
    "Boil water": {"pre": [], "effect": (0,0,0,1,0,0), "cost": 5},
    "Add vegetables": {"pre": [2,3], "effect": (0,0,0,0,1,0), "cost": 1},
    "Cook soup": {"pre": [4], "effect": (0,0,0,0,0,1), "cost": 6}
}

# Initial state: no ingredients prepared, no water boiled, no actions taken.
initial_state = (0,0,0,0,0,0)

# Goal state: all steps completed, soup is cooked.
goal_state = (1,1,1,1,1,1)

# Configure logging for debugging and tracing execution.
logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

# ===== Goal Test =====
def is_goal(state):
    """Check if the current state matches the goal state."""
    return state == goal_state


# ===== Action Application =====
def apply_action(state, action):
    """
    Try to apply an action to the current state.
    
    Returns:
      - new_state: a tuple representing the state after the action (if preconditions met)
      - None: if the action's preconditions are not satisfied
    
    Logic:
      1. Check all preconditions (state indices that must be 1).
      2. If any precondition is not met (state value is 0), return None.
      3. Otherwise, compute new_state by OR'ing the effect with the current state.
    """
    pre = actions[action]["pre"]
    # Check all preconditions
    for i in pre:
        if state[i] == 0:
            return None  # Precondition not satisfied; action cannot be applied
    
    # All preconditions met; compute new state by applying effects
    effect = actions[action]["effect"]
    new_state = tuple(state[i] | effect[i] for i in range(len(state)))
    return new_state


# ===== Breadth-First Search (BFS) =====
def bfs(initial):
    """
    Breadth-First Search: explores states level by level, guaranteeing the shortest path 
    (in number of steps, not cost). Suitable for unweighted or uniformly-weighted graphs.
    
    Returns:
      - path: list of (action_name, new_state) tuples representing the plan
      - cost: total cost of the path
      - nodes_expanded: number of states visited
      - G: networkx.DiGraph representing the search tree
    """
    queue = Queue()
    queue.put((initial, [], 0))  # Each item: (state, path, cumulative_cost)
    visited = set()  # Track visited states to avoid cycles
    nodes_expanded = 0

    # Initialize graph to track the search tree
    G = nx.DiGraph()
    G.add_node(initial)

    while not queue.empty():
        state, path, cost = queue.get()
        
        # Skip if already visited (to avoid re-processing states)
        if state in visited:
            continue
        visited.add(state)
        nodes_expanded += 1

        # Goal test: check if we reached the goal
        if is_goal(state):
            return path, cost, nodes_expanded, G

        # Expand: try all possible actions from this state
        for action in actions:
            new_state = apply_action(state, action)
            # Only enqueue new unvisited states
            if new_state and new_state not in visited:
                queue.put((new_state, path + [(action, new_state)], cost + actions[action]["cost"]))
                G.add_node(new_state)
                G.add_edge(state, new_state, label=action)
    
    # No solution found
    return None, 0, nodes_expanded, G


# ===== A* Heuristic and Search =====
def heuristic(state):
    """
    Admissible heuristic for A*: estimates the minimum cost to reach the goal from state.
    
    Heuristic: count remaining unsatisfied goals (state bits that are 0),
    multiply by the minimum action cost. This is admissible because any action costs 
    at least min_cost, and we need at least one action per unsatisfied goal.
    
    Returns:
      - estimated cost to goal
    """
    remaining = sum(1 for i in range(len(state)) if state[i] == 0)
    min_cost = min([actions[a]["cost"] for a in actions])
    return remaining * min_cost

def a_star(initial):
    """
    A* Search: uses f(n) = g(n) + h(n) to guide search, typically faster than BFS 
    for cost-sensitive problems.
    
    f(n) = g(n) + h(n), where:
      - g(n): actual cost from initial state to current state
      - h(n): heuristic estimate of cost from current state to goal
    
    Returns:
      - path: list of (action_name, new_state) tuples representing the plan
      - cost: total cost of the path (= g value at goal)
      - nodes_expanded: number of states visited
      - G: networkx.DiGraph representing the search tree
    """
    pq = PriorityQueue()
    # Each item: (f_value, g_value, state, path)
    pq.put((0 + heuristic(initial), 0, initial, []))
    visited = {}  # Maps state -> minimum g value seen for that state
    nodes_expanded = 0

    # Initialize graph to track the search tree
    G = nx.DiGraph()
    G.add_node(initial)

    while not pq.empty():
        f, g, state, path = pq.get()
        
        # Skip if we've seen this state with a lower cost
        if state in visited and visited[state] <= g:
            continue
        visited[state] = g
        nodes_expanded += 1

        # Goal test
        if is_goal(state):
            return path, g, nodes_expanded, G

        # Expand: try all possible actions from this state
        for action in actions:
            new_state = apply_action(state, action)
            if new_state:
                g_new = g + actions[action]["cost"]
                f_new = g_new + heuristic(new_state)
                pq.put((f_new, g_new, new_state, path + [(action, new_state)]))
                G.add_node(new_state)
                G.add_edge(state, new_state, label=action)
    
    # No solution found
    return None, 0, nodes_expanded, G

# ===== Graph Visualization =====
def draw_graph(G, path, filename="graph.png", initial_state=None):
    """Draw the graph to a PNG file.

    Args:
      - G: networkx.DiGraph containing the search tree
      - path: list of (action, to_state) tuples representing the solution path
      - filename: output PNG filename
      - initial_state: the starting state (used to highlight the first edge in the path)

    Features:
      - Node labels are shortened (s0, s1, ...) for readability
      - Edges labeled with action names
      - Solution path highlighted in red
    """
    pos = nx.spring_layout(G, seed=42)
    plt.figure(figsize=(12,8))

    # Map full state tuples to short labels (s0, s1, ...) for readability
    nodes = list(G.nodes())
    label_map = {nodes[i]: f"s{i}" for i in range(len(nodes))}

    # Draw nodes with short labels
    nx.draw(G, pos, labels=label_map, with_labels=True, node_size=1000, node_color='lightblue')
    
    # Extract and display edge labels (action names)
    edge_labels = nx.get_edge_attributes(G, 'label')
    short_edge_labels = {}
    for (u, v), lbl in edge_labels.items():
        short_u = label_map.get(u, str(u))
        short_v = label_map.get(v, str(v))
        short_edge_labels[(u, v)] = lbl
    nx.draw_networkx_edge_labels(G, pos, edge_labels=short_edge_labels, font_size=8)

    # Build the list of edges that form the solution path
    path_edges = []
    if path:
        prev_state = initial_state
        # Reconstruct path edges: connect prev_state to each new_state in the path
        if prev_state is None:
            prev_state = None
        for action, new_state in path:
            if prev_state is not None:
                path_edges.append((prev_state, new_state))
            prev_state = new_state

    # Highlight the solution path in red
    if path_edges:
        # Ensure edges exist in the graph before drawing
        valid_edges = [e for e in path_edges if G.has_edge(e[0], e[1])]
        nx.draw_networkx_edges(G, pos, edgelist=valid_edges, edge_color='red', width=2)

    # Save and close
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    plt.close()


# ===== Main Execution =====

# Run BFS and measure execution time
start_time = time.time()
bfs_path, bfs_cost, bfs_nodes, bfs_graph = bfs(initial_state)
bfs_time = time.time() - start_time

# Print BFS results
print("BFS Path:", [a for a,_ in bfs_path])
print("BFS Total Cost:", bfs_cost)
print("BFS Nodes Expanded:", bfs_nodes)
print("BFS Execution Time (s):", round(bfs_time,6))
draw_graph(bfs_graph, bfs_path, "bfs_graph.png", initial_state=initial_state)

# Run A* and measure execution time
start_time = time.time()
astar_path, astar_cost, astar_nodes, astar_graph = a_star(initial_state)
astar_time = time.time() - start_time

# Print A* results
print("A* Path:", [a for a,_ in astar_path])
print("A* Total Cost:", astar_cost)
print("A* Nodes Expanded:", astar_nodes)
print("A* Execution Time (s):", round(astar_time,6))
draw_graph(astar_graph, astar_path, "astar_graph.png", initial_state=initial_state)

# Print comparison table
print("\nComparison of BFS vs A*:")
print("{:<10} {:<10} {:<15} {:<15}".format("Algorithm","Cost","Nodes Expanded","Time (s)"))
print("{:<10} {:<10} {:<15} {:<15}".format("BFS", bfs_cost, bfs_nodes, round(bfs_time,6)))
print("{:<10} {:<10} {:<15} {:<15}".format("A*", astar_cost, astar_nodes, round(astar_time,6)))

