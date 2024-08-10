class DLS:
    def __init__(self, graph, root, depth_limit):
        self.graph = graph
        self.visited = dict()
        self.stack = list()
        self.counter = 0
        self.depth_limit = depth_limit
        self.visited[root.UID] = root
        self.stack.append((root, 0))

    def run(self, target):
        while len(self.stack) > 0:
            self.counter += 1
            current_state, current_depth = self.stack.pop()

            if current_state.is_equal(target):
                depth = current_state.step
                return True, self.counter, depth

            if current_depth < self.depth_limit:
                neighbor_nodes = self.graph.reveal_neighbors(current_state)

                for neighbor_node in neighbor_nodes:
                    if neighbor_node.UID not in self.visited.keys():
                        self.stack.append((neighbor_node, current_depth + 1))
                        self.visited[neighbor_node.UID] = neighbor_node

        return False, 0, 0
