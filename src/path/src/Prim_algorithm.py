import numpy as np


def prim_np(w, n1, n2):
    """Prim's algorithm is a greedy algorithm that 
    finds a minimum spanning tree for a weighted undirected graph.
    1. Initialize a tree with a single vertex,
       chosen arbitrarily from the graph.
    2. Grow the tree by one edge: of the edges that connect the tree to 
       vertices not yet in the tree, find the minimum-weight edge,
       and transfer it to the tree.
    3. Repeat step 2 (until all vertices are in the tree).
    
    Inputs:
        w = weights, 
        heads, 
        tails: list of numbers (not np arrays)
    """
    # w_ind takes care of index of each w, which is helpful to keep track of starting position of w 
    w_ind = np.array([i for i in range(len(w))])
    # by default, it is set to false only when the edge is not considered for Minimum search tree, 
    # but ended up true when you include that edge in minimum search tree
    
    include_edge = [False] * len(w) 
    
    # Using sets for storing the vectors of the tree
    left_node = set(list(n1)+list(n2)) # nodes that are not included in MST
    current_node = set([n1[0]]) # this node is a part of MST 
    left_node.remove(n1[0]) # removes the first nodes 
    
    while len(left_node)>0: # keep adding all nodes to MST and figure out which node has less weights
        # Selecting the edges between the tree and the remaining vertices.
        has_n1 = np.isin(n1, list(current_node)) # checks or node 1 if that node is in MST
        has_n2 = np.isin(n2, list(current_node)) # checks or node 2 if that node is in MST
        between_edges = np.not_equal(has_n1,has_n2) # compare node1 and node 2
        # Selecting the minimum weight edge.
        w = w[between_edges] # compare nodes/vertex that are not equal
        w_i = w_ind[between_edges]
        # main step to figure out which edge is smaller between two nodes
        # the one thats smaller edge(using argmin) will be processed into MST
        n_edge = w_i[np.argmin(w)] 
        
        # Select the new node and add it to the tree vertices.
        n_node = n2[n_edge] if n1[n_edge] in current_node else n1[n_edge]
        current_node.add(n_node)
        left_node.remove(n_node)
        include_edge[n_edge] = True
    return include_edge


def build_arrays(V, G):
    """Build arrays for np array experimental from graph:
    Input:
        V: Number of nodes
        G: edges [h,t,w]: head - tail, weight
    Output:
        w: list of w
        heads: list of first nodes
        tails: list of last nodes"""
    Gn = np.array(G)
    return Gn[:,2], Gn[:,0], Gn[:,1]
