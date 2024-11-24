# Artificial Intelligence Algorithms Repository

This repository contains the implementation of key search algorithms commonly used in Artificial Intelligence for pathfinding and graph traversal. The algorithms implemented are:

- **A\* (A-Star) Search Algorithm**
- **BFS (Breadth-First Search)**
- **DFS (Depth-First Search)**
- **GBFS (Greedy Best First Search)**

All these algorithms have been applied to a graph of cities where the goal is to find the optimal or feasible path between two locations, using various strategies.

## Algorithms

### 1. A\* (A-Star) Search
A\* is a popular and efficient pathfinding algorithm that uses a heuristic to guide its search. It balances between exploring the shortest path and reducing the distance to the goal. It uses the following:
- **g(n)**: The actual cost from the start node to the current node.
- **h(n)**: A heuristic function that estimates the cost from the current node to the goal.
- **f(n) = g(n) + h(n)**: The total cost estimate used to guide the search.

**Usage:**
```python
path = A_star(graph, start_node, target_node)
```

### 2. BFS (Breadth-First Search)
BFS is an uninformed search algorithm that explores all possible paths from the starting node layer by layer. It guarantees finding the shortest path in terms of the number of edges, but not necessarily the least cost in terms of weights.

**Usage:**
```python
path = BFS(graph, start_node, target_node)
```

### 3. DFS (Depth-First Search)
DFS is an uninformed search algorithm that explores as far as possible along each branch before backtracking. It does not guarantee the shortest or optimal path but is efficient in memory usage.

**Usage:**
```python
path = DFS(graph, start_node, target_node)
```

### 4. GBFS (Greedy Best First Search)
Greedy Best First Search is a heuristic-based search algorithm that expands the node closest to the goal, as measured by a heuristic function. It does not guarantee finding the optimal solution but can be faster in some cases due to its heuristic-driven approach.

**Usage:**
```python
path = GBFS(graph, start_node, target_node)
```

## File Structure

```bash
Artificial_Intelligence/
│
├── A_star.ipynb               # A* Search algorithm implementation
├── BFS.ipynb                  # Breadth-First Search algorithm implementation
├── DFS.ipynb                  # Depth-First Search algorithm implementation
├── GBFS.ipynb                 # Greedy Best First Search algorithm implementation
├── RandomGraph.py             # Python script to generate random graphs
├── complete_graph.csv         # Graph data containing distances between cities
├── random_connected_graph.csv # Random graph for testing algorithms
├── random_graph.ipynb         # Random graph generation notebook
└── README.md                  # This file
```

## How to Use

1. Clone the repository:
```bash
git clone https://github.com/8MISHRA/Best_Path_Between_Two_Districts.git
cd Best_Path_Between_Two_Districts
```

2. Install necessary libraries (if not already installed):
```bash
pip install pandas networkx matplotlib
```

3. Run the algorithms:
   - Open the Jupyter notebooks (`.ipynb` files) and execute the code cells.
   - You can test the algorithms on different cities by modifying the `start_node` and `target_node` values in each notebook.

4. The `RandomGraph.py` script can be used to generate random connected graphs for testing the algorithms.

## Graph Data

The `complete_graph.csv` file contains the graph data used for these algorithms. The graph consists of cities as nodes and the distances between them as edges. The file also includes heuristic distance values used for A\* and Greedy Best First Search algorithms.

## Example

Here’s an example of how to find the path between two cities using the A* algorithm:
```python
start_node = "Lucknow"
target_node = "Mirzapur"

path = A_star(graph, start_node, target_node)
print(path)
```

## Contribution

Feel free to submit pull requests or open issues for improvements or bug fixes.
