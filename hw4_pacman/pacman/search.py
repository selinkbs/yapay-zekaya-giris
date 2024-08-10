"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """

    from util import Stack

    open_list = Stack()

    visited_list = []
    path = []
    action_cost = 0  # Cost of each movement.

    start_position = problem.getStartState()

    open_list.push((start_position, path, action_cost))

    while not open_list.isEmpty():

        current_node = open_list.pop()
        position = current_node[0]
        path = current_node[1]

        if position not in visited_list:
            visited_list.append(position)

        if problem.isGoalState(position):
            return path

        successors = problem.getSuccessors(position)

        for item in successors:
            if item[0] not in visited_list:

                new_position = item[0]
                new_path = path + [item[1]]
                open_list.push((new_position, new_path, item[2]))

    util.raiseNotDefined()


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    from util import Queue
    open_list = Queue()

    visited_list = []
    path = []
    action_cost = 0  # Cost of each movement.

    start_position = problem.getStartState()
    open_list.push((start_position, path, action_cost))

    while not open_list.isEmpty():

        current_node = open_list.pop()
        position = current_node[0]
        path = current_node[1]

        if position not in visited_list:
            visited_list.append(position)

        if problem.isGoalState(position):
            return path

        successors = problem.getSuccessors(position)
        for item in successors:
            if item[0] not in visited_list and item[0] not in (node[0] for node in open_list.list):

                new_position = item[0]
                new_path = path + [item[1]]
                open_list.push((new_position, new_path, item[2]))

    util.raiseNotDefined()


def uniformCostSearch(problem):

    from util import PriorityQueue
    open_list = PriorityQueue()

    visited_list = []
    path = []
    priority = 0    # Initializes the priority to 0.

    start_position = problem.getStartState()

    open_list.push((start_position, path), priority)

    while not open_list.isEmpty():

        current_node = open_list.pop()
        position = current_node[0]
        path = current_node[1]

        if position not in visited_list:
            visited_list.append(position)

        if problem.isGoalState(position):
            return path

        successors = problem.getSuccessors(position)

        def getPriorityOfNode(priority_queue, node):
            for item in priority_queue.heap:
                if item[2][0] == node:
                    return problem.getCostOfActions(item[2][1])
        for item in successors:
            if item[0] not in visited_list and (item[0] not in (node[2][0] for node in open_list.heap)):
                new_path = path + [item[1]]
                new_priority = problem.getCostOfActions(new_path)
                open_list.push((item[0], new_path), new_priority)

            elif item[0] not in visited_list and (item[0] in (node[2][0] for node in open_list.heap)):
                old_priority = getPriorityOfNode(open_list, item[0])
                new_priority = problem.getCostOfActions(new_path)

                if old_priority > new_priority:
                    new_path = path + [item[1]]
                    open_list.update((item[0], new_path), new_priority)

    util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):

    from util import PriorityQueue

    open_list = PriorityQueue()

    visited_list = []
    path = []
    priority = 0    

    start_position = problem.getStartState()

    open_list.push((start_position, path), priority)

    while not open_list.isEmpty():

        current_node = open_list.pop()
        position = current_node[0]
        path = current_node[1]

        if problem.isGoalState(position):
            return path

        if position not in visited_list:
            visited_list.append(position)

            successors = problem.getSuccessors(position)

            for item in successors:
                if item[0] not in visited_list:
                    new_position = item[0]
                    new_path = path + [item[1]]

                  
                    """ g(n): Current cost from start state to the current position. """
                    g = problem.getCostOfActions(new_path)

                    """ h(n): Estimate of the lowest cost from the current position to the goal state. """
                    h = heuristic(new_position, problem)

                    """ f(n): Estimate of the lowest cost of the solution path
                              from start state to the goal state passing through the current position """
                    f = g + h

                    new_priority = f
                    open_list.push((new_position, new_path), new_priority)

    util.raiseNotDefined()

bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
