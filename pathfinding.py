import numpy as np
import heapq
import networkx as nx

def ed(a, b):
    #Calculate the Euclidean distance between two points.
    return np.linalg.norm(np.array(a) - np.array(b))

def findpath(graph, start_node, end_node):

    open_list = []
    heapq.heappush(open_list, (0, start_node))
    came_from = {}
    g_score = {node: float('inf') for node in graph.nodes}
    g_score[start_node] = 0
    f_score = {node: float('inf') for node in graph.nodes}
    f_score[start_node] = ed(graph.nodes[start_node]['pos'], graph.nodes[end_node]['pos'])

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == end_node:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start_node)
            path.reverse()
            return path

        for neighbor in graph.neighbors(current):
            tentative_g_score = g_score[current] + graph[current][neighbor]['weight']
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + ed(graph.nodes[neighbor]['pos'], graph.nodes[end_node]['pos'])
                heapq.heappush(open_list, (f_score[neighbor], neighbor))

    return None  # No path found

def find_nearest_node(graph, point):
    min_dist = float('inf')
    nearest_node = None
    for node, data in graph.nodes(data=True):
        dist = np.linalg.norm(np.array(data['pos']) - np.array(point))
        if dist < min_dist:
            min_dist = dist
            nearest_node = node
    return nearest_node
