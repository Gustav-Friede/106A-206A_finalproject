import numpy as np


def prim_np(weights, node1, node2):
    """Prim's algorithm is a greedy algorithm that 
    finds a minimum spanning tree for a weighted undirected graph.
    1. Initialize a tree with a single vertex,
       chosen arbitrarily from the graph.
    2. Grow the tree by one edge: of the edges that connect the tree to 
       vertices not yet in the tree, find the minimum-weight edge,
       and transfer it to the tree.
    3. Repeat step 2 (until all vertices are in the tree).
    
    Inputs:
        weights, heads, tails: list of numbers (not np arrays)
    """
    w_indexes = np.array([i for i in range(len(weights))])
    edges_in = [False] * len(weights)
    
    # Using sets for storing the vectors of the tree
    rem_nodes = set(list(node1)+list(node2))
    has_nodes = set([node1[0]])
    rem_nodes.remove(node1[0])
    
    while len(rem_nodes)>0:
        # Selecting the edges between the tree and the remaining vertices.
        has_node1 = np.isin(node1, list(has_nodes))
        has_node2 = np.isin(node2, list(has_nodes))
        between_edges = np.not_equal(has_node1,has_node2)
        # Selecting the minimum weight edge.
        w = weights[between_edges]
        w_i = w_indexes[between_edges]
        n_edge = w_i[np.argmin(w)]
        # Select the new node and add it to the tree vertices.
        n_node = node2[n_edge] if node1[n_edge] in has_nodes else node1[n_edge]
        has_nodes.add(n_node)
        rem_nodes.remove(n_node)
        edges_in[n_edge] = True
    return edges_in


def build_arrays(V, G):
    """Build arrays for np array prim from graph:
    Input:
        V: Number of nodes
        G: edges [h,t,w]: head - tail, weight
    Output:
        weights: list of weights
        heads: list of first nodes
        tails: list of last nodes"""
    Gn = np.array(G)
    return Gn[:,2], Gn[:,0], Gn[:,1]
