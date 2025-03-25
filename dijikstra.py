import networkx as nx
from svgpathtools import svg2paths, CubicBezier, QuadraticBezier
import matplotlib.pyplot as plt
import math
import numpy as np
import heapq

# Show animation flag
show_animation = True

class SVGMapDijkstra:
    def __init__(self, svg_file, scale_factor=0.1, num_samples=10):
        """
        Initialize path planner with an SVG map
        
        svg_file: path to SVG file
        scale_factor: scale factor for SVG coordinates
        num_samples: number of samples to take along curves
        """
        self.scale_factor = scale_factor
        self.num_samples = num_samples
        
        # Load the SVG file
        self.paths, self.attributes = svg2paths(svg_file)
        
        # Create the graph
        self.graph, self.node_positions = self.create_graph_from_svg()
        
        # Create reverse mapping from positions to node IDs
        self.pos_to_node = {pos: node for node, pos in self.node_positions.items()}
        
    def sample_curve(self, curve, num_samples=10):
        """Sample points along a Bézier curve at evenly spaced intervals."""
        points = []
        for t in np.linspace(0, 1, num_samples):
            points.append(curve.point(t))
        return points
    
    def create_graph_from_svg(self):
        """Convert SVG paths to a networkx graph."""
        graph = nx.Graph()
        node_positions = {}  # Dictionary to store node positions
        node_id = 0  # Counter for assigning unique node IDs
        
        for path_idx, path in enumerate(self.paths):
            for segment in path:
                # For curves (Bézier curves)
                if isinstance(segment, (CubicBezier, QuadraticBezier)):
                    # Sample points along the curve
                    sampled_points = self.sample_curve(segment, self.num_samples)
                    
                    # Add sampled points as nodes and connect them
                    prev_node_id = None
                    for point in sampled_points:
                        scaled_point = (round(point.real * self.scale_factor, 3), 
                                        round(point.imag * self.scale_factor, 3))
                        
                        # Check if this point already exists
                        if scaled_point not in self.pos_to_node if hasattr(self, 'pos_to_node') else True:
                            current_node_id = node_id
                            node_positions[current_node_id] = scaled_point
                            graph.add_node(current_node_id, pos=scaled_point)
                            node_id += 1
                        else:
                            current_node_id = self.pos_to_node[scaled_point]
                        
                        # Connect to previous point
                        if prev_node_id is not None:
                            prev_pos = node_positions[prev_node_id]
                            curr_pos = scaled_point
                            # Calculate Euclidean distance
                            weight = math.sqrt((prev_pos[0] - curr_pos[0])**2 + 
                                               (prev_pos[1] - curr_pos[1])**2)
                            graph.add_edge(prev_node_id, current_node_id, weight=weight)
                        
                        prev_node_id = current_node_id
                else:
                    # Handle straight lines
                    start = (round(segment.start.real * self.scale_factor, 3), 
                             round(segment.start.imag * self.scale_factor, 3))
                    end = (round(segment.end.real * self.scale_factor, 3), 
                           round(segment.end.imag * self.scale_factor, 3))
                    
                    # Check if these points already exist
                    if start not in self.pos_to_node if hasattr(self, 'pos_to_node') else True:
                        start_node_id = node_id
                        node_positions[start_node_id] = start
                        graph.add_node(start_node_id, pos=start)
                        node_id += 1
                    else:
                        start_node_id = self.pos_to_node[start]
                    
                    if end not in self.pos_to_node if hasattr(self, 'pos_to_node') else True:
                        end_node_id = node_id
                        node_positions[end_node_id] = end
                        graph.add_node(end_node_id, pos=end)
                        node_id += 1
                    else:
                        end_node_id = self.pos_to_node[end]
                    
                    # Calculate Euclidean distance
                    weight = math.sqrt((start[0] - end[0])**2 + (start[1] - end[1])**2)
                    graph.add_edge(start_node_id, end_node_id, weight=weight)
        
        return graph, node_positions
    
    def find_closest_node(self, x, y):
        """Find the closest node in the graph to the given coordinates."""
        closest_node = None
        min_distance = float('inf')
        
        for node, pos in self.node_positions.items():
            distance = math.sqrt((x - pos[0])**2 + (y - pos[1])**2)
            if distance < min_distance:
                min_distance = distance
                closest_node = node
        
        return closest_node, min_distance
    
    def planning(self, start_x, start_y, goal_x, goal_y):
        """
        Find the shortest path from start to goal using Dijkstra's algorithm
        
        Args:
            start_x, start_y: Starting position
            goal_x, goal_y: Goal position
            
        Returns:
            rx, ry: Lists of x and y coordinates of the path
        """
        # Find closest nodes to start and goal
        start_node, _ = self.find_closest_node(start_x, start_y)
        goal_node, _ = self.find_closest_node(goal_x, goal_y)
        
        if start_node is None or goal_node is None:
            print("Start or goal position not reachable!")
            return [], []
        
        # Use NetworkX's Dijkstra algorithm to find the shortest path
        try:
            path = nx.dijkstra_path(self.graph, start_node, goal_node, weight='weight')
            path_length = nx.dijkstra_path_length(self.graph, start_node, goal_node, weight='weight')
            print(f"Path found with length: {path_length}")
        except nx.NetworkXNoPath:
            print("No path exists between start and goal!")
            return [], []
        
        # Extract x and y coordinates
        rx = []
        ry = []
        for node in path:
            pos = self.node_positions[node]
            rx.append(pos[0])
            ry.append(pos[1])
        
        return rx, ry
    
    def find_path_to_ducks(self, start_x, start_y, duck_positions):
        """
        Find optimal path to collect all ducks
        
        Args:
            start_x, start_y: Starting position
            duck_positions: List of (x,y) positions of ducks
            
        Returns:
            Complete path coordinates (rx, ry)
        """
        # Start with the robot's position
        current_x, current_y = start_x, start_y
        complete_path_x, complete_path_y = [], []
        remaining_ducks = duck_positions.copy()
        
        while remaining_ducks:
            # Find closest duck
            closest_duck = None
            min_distance = float('inf')
            
            for duck in remaining_ducks:
                duck_x, duck_y = duck
                distance = math.hypot(current_x - duck_x, current_y - duck_y)
                if distance < min_distance:
                    min_distance = distance
                    closest_duck = duck
            
            # Plan path to the closest duck
            duck_x, duck_y = closest_duck
            rx, ry = self.planning(current_x, current_y, duck_x, duck_y)
            
            if not rx:  # No path found
                print(f"No path found to duck at ({duck_x}, {duck_y})")
                remaining_ducks.remove(closest_duck)
                continue
                
            # Add this path segment to the complete path
            if complete_path_x:
                # Skip the first point as it's the same as the last point of the previous segment
                complete_path_x.extend(rx[1:])
                complete_path_y.extend(ry[1:])
            else:
                complete_path_x = rx
                complete_path_y = ry
            
            # Update current position and remove the duck from remaining ducks
            current_x, current_y = duck_x, duck_y
            remaining_ducks.remove(closest_duck)
        
        return complete_path_x, complete_path_y
    
    def visualize_map(self):
        """Visualize the network map."""
        plt.figure(figsize=(10, 10))
        
        # Draw the network
        pos = self.node_positions
        nx.draw(self.graph, pos, node_size=20, node_color='lightblue')
        
        plt.title("SVG Map Network")
        plt.grid(True)
        plt.axis('equal')
    
    def visualize_path(self, rx, ry, start_pos, duck_positions):
        """Visualize the path with the network map."""
        plt.figure(figsize=(10, 10))
        
        # Draw the network
        pos = self.node_positions
        nx.draw(self.graph, pos, node_size=10, node_color='lightgray', edge_color='gray', alpha=0.5)
        
        # Draw the path
        plt.plot(rx, ry, "-r", linewidth=2, label="Path")
        
        # Draw start and ducks
        plt.plot(start_pos[0], start_pos[1], "og", markersize=10, label="Start")
        
        # Plot ducks
        duck_x = [duck[0] for duck in duck_positions]
        duck_y = [duck[1] for duck in duck_positions]
        plt.plot(duck_x, duck_y, "oy", markersize=15, label="Ducks")
        
        plt.title("Duck Transport Robot Path Planning")
        plt.grid(True)
        plt.axis('equal')
        plt.legend()


def main():
    print("Duck transport robot path planning start!!")
    
    # SVG map file
    svg_file = 'map.svg'
    
    # Start position
    sx = 0.0
    sy = 0.0
    
    # Duck positions - adjust these according to your map
    duck_positions = [
        (10.0, 10.0),
        (30.0, 15.0),
        (15.0, 30.0),
        (45.0, 25.0)
    ]
    
    # Create planner with SVG map
    planner = SVGMapDijkstra(svg_file, scale_factor=0.1, num_samples=10)
    
    # First, visualize the map
    if show_animation:
        planner.visualize_map()
        plt.pause(0.01)
    
    # Find path to collect all ducks
    rx, ry = planner.find_path_to_ducks(sx, sy, duck_positions)
    
    # Visualize the path
    if show_animation and rx:
        planner.visualize_path(rx, ry, (sx, sy), duck_positions)
        plt.pause(0.01)
        plt.show()
    
    print("Planning complete!")
    
    return rx, ry


if __name__ == '__main__':
    main()