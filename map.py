import networkx as nx
from svgpathtools import svg2paths, CubicBezier, QuadraticBezier
import matplotlib.pyplot as plt
import numpy as np

paths, attributes = svg2paths('map.svg')

#Scale
scale_factor = 0.1

#Create an empty graph
graph = nx.Graph()

def makegraph (paths, num_samples=10):
    node_positions = {}  # Dictionary to store node positions and their unique IDs

    for path in paths:
        # Loop through each segment in the path (assuming they represent roads)
        for segment in path:
            # For straight lines (lines and cubic Bézier curves)
            if isinstance(segment, CubicBezier) or isinstance(segment, QuadraticBezier):
                # Sample points along the curve
                sampled_points = sample_curve(segment, num_samples)
                
                # Add sampled points as nodes
                for i, point in enumerate(sampled_points):
                    scaled_point = (point.real * scale_factor, point.imag * scale_factor)
                    if scaled_point not in node_positions:
                        node_positions[scaled_point] = len(node_positions)
                        graph.add_node(node_positions[scaled_point], pos=scaled_point)
                    
                    # If it's not the first point, connect it to the previous point with an edge
                    if i > 0:
                        prev_point = sampled_points[i-1]
                        prev_scaled_point = (prev_point.real * scale_factor, prev_point.imag * scale_factor)
                        edge_weight = np.linalg.norm(np.array(scaled_point) - np.array(prev_scaled_point))
                        graph.add_edge(node_positions[prev_scaled_point], node_positions[scaled_point], weight=edge_weight)

            else:  # Handle straight lines or other types of paths
                start = (segment.start.real * scale_factor, segment.start.imag * scale_factor)
                end = (segment.end.real * scale_factor, segment.end.imag * scale_factor)
                if start not in node_positions:
                    node_positions[start] = len(node_positions)
                    graph.add_node(node_positions[start], pos=start)
                if end not in node_positions:
                    node_positions[end] = len(node_positions)
                    graph.add_node(node_positions[end], pos=end)
                edge_weight = np.linalg.norm(np.array(start) - np.array(end))
                graph.add_edge(node_positions[start], node_positions[end], weight=edge_weight)

#Function to sample points along a Bézier curve
def sample_curve(curve, num_samples=10):
    points = []
    for t in np.linspace(0, 1, num_samples):
        points.append(curve.point(t))
    return points

#Add paths from the SVG to the graph
makegraph(paths)

#Get node positions from the graph for plotting
pos = nx.get_node_attributes(graph, 'pos')

#Plot the graph using matplotlib
plt.figure(figsize=(8, 8))
nx.draw(graph, pos, with_labels=True, node_color='lightblue', node_size=800, edge_color='gray')

#Draw edge labels showing the weight (distance)
labels = nx.get_edge_attributes(graph, 'weight')
nx.draw_networkx_edge_labels(graph, pos, edge_labels=labels)

plt.show()
