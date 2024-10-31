import heapq
#Task 1:

class Graph:
    def __init__(self):
        self.vertices = {}

    def add_vertex(self, vertex):
        if vertex not in self.vertices:
            self.vertices[vertex] = {}

    def add_edge(self, source, destination, weight):
        if source in self.vertices and destination in self.vertices:
            self.vertices[source][destination] = weight
            self.vertices[destination][source] = weight  # For undirected graph

    def get_neighbors(self, vertex):
        return self.vertices.get(vertex, {})


#Task 2:


def dijkstra(graph, start):
    # Priority queue to store (distance, vertex)
    priority_queue = [(0, start)]
    # Dictionary to store the shortest path to each vertex
    shortest_paths = {vertex: float('inf') for vertex in graph.vertices}
    shortest_paths[start] = 0
    # Dictionary to store the path taken to each vertex
    previous_vertices = {vertex: None for vertex in graph.vertices}

    while priority_queue:
        current_distance, current_vertex = heapq.heappop(priority_queue)

        # Nodes can only be popped once in Dijkstra's algorithm
        if current_distance > shortest_paths[current_vertex]:
            continue

        for neighbor, weight in graph.get_neighbors(current_vertex).items():
            distance = current_distance + weight

            # Only consider this new path if it's better
            if distance < shortest_paths[neighbor]:
                shortest_paths[neighbor] = distance
                previous_vertices[neighbor] = current_vertex
                heapq.heappush(priority_queue, (distance, neighbor))

    return shortest_paths, previous_vertices

def reconstruct_path(previous_vertices, start, end):
    path = []
    current_vertex = end
    while current_vertex is not None:
        path.append(current_vertex)
        current_vertex = previous_vertices[current_vertex]
    path.reverse()
    return path if path[0] == start else []

#Task 3:

graph = Graph()
graph.add_vertex('A')
graph.add_vertex('B')
graph.add_vertex('C')
graph.add_edge('A', 'B', 5)
graph.add_edge('B', 'C', 3)
graph.add_edge('A', 'C', 10)

shortest_paths, previous_vertices = dijkstra(graph, 'A')
print("Shortest paths:", shortest_paths)
print("Path from A to C:", reconstruct_path(previous_vertices, 'A', 'C'))

#Task 4:
#Time complexity of Dijkstra's algorithm is O((V + E) log V) where V is the number of vertices and E is the number of edges in the graph.
#Space complexity of Dijkstra's algorithm is O(V) where V is the number of vertices in the graph.
#Optimizations like using Fibonacci heap can reduce the time complexity to O(V log V + E) and space complexity to O(V + E) where V is the number of vertices and E is the number of edges in the graph.
