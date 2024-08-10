class GBFS:
    def __init__(self, graph, root, target):
        self.graph = graph
        self.visited = dict()
        self.priority_queue = [] 
        self.counter = 0
        self.target = target

        root.heuristic = self.calculate_heuristic(root)
        self.visited[root.UID] = root
        self.priority_queue.append((root.heuristic, root))

    def calculate_heuristic(self, node):

    def run(self):
        while len(self.priority_queue) > 0:
            self.counter += 1
            _, current_state = self.priority_queue.pop(0)
            self.visited[current_state.UID] = current_state

            if current_state.is_equal(self.target):
                depth = current_state.step
                return True, self.counter, depth

            neighbor_nodes = self.graph.reveal_neighbors(current_state)

            for neighbor_node in neighbor_nodes:
                if neighbor_node.UID not in self.visited.keys():
                    neighbor_node.heuristic = self.calculate_heuristic(neighbor_node)
                    self.priority_queue.append((neighbor_node.heuristic, neighbor_node))

            self.priority_queue.sort(key=lambda x: x[0])

        return False, 0, 0
