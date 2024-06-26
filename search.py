# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


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

    def expand(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (child,
        action, stepCost), where 'child' is a child to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that child.
        """
        util.raiseNotDefined()

    def getActions(self, state):
        """
          state: Search state

        For a given state, this should return a list of possible actions.
        """
        util.raiseNotDefined()

    def getActionCost(self, state, action, next_state):
        """
          state: Search state
          action: action taken at state.
          next_state: next Search state after taking action.

        For a given state, this should return the cost of the (s, a, s') transition.
        """
        util.raiseNotDefined()

    def getNextState(self, state, action):
        """
          state: Search state
          action: action taken at state

        For a given state, this should return the next state after taking action from state.
        """
        util.raiseNotDefined()

    def getCostOfActionSequence(self, actions):
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
    explorednodes = set()
    fringe = util.Stack()
    startstate = problem.getStartState()
    startnode = (startstate, [])
    fringe.push(startnode)
    while not fringe.isEmpty():
        state, path = fringe.pop()
        if problem.isGoalState(state):
            return path
        if state not in explorednodes:
            explorednodes.add(state)
            for successor, action, stepCost in problem.expand(state):
                if successor not in explorednodes:
                    fringe.push((successor, path + [action]))
    return []
    util.raiseNotDefined()


def breadthFirstSearch(problem):
    explorednodes = set()
    fringe = util.Queue()
    startstate = problem.getStartState()
    startnode = (startstate, [])
    fringe.push(startnode)
    while not fringe.isEmpty():
        state, path = fringe.pop()
        if problem.isGoalState(state):
            return path
        if state not in explorednodes:
            explorednodes.add(state)
            for successor, action, stateCost in problem.expand(state):
                if successor not in explorednodes:
                    fringe.push((successor, path + [action]))
    return []
    util.raiseNotDefined()


def uniformCostSearch(problem: SearchProblem):
    explorednodes = set()
    fringe = util.PriorityQueue()
    startState = problem.getStartState()
    fringe.push((startState, []), 0)
    while not fringe.isEmpty():
        state, path = fringe.pop()
        if problem.isGoalState(state):
            return path
        if state not in explorednodes:
            explorednodes.add(state)
            for successor, action, stepCost in problem.expand(state):
                if successor not in explorednodes:
                    newPath = path + [action]
                    totalCost = problem.getCostOfActionSequence(newPath)
                    fringe.push((successor, newPath), totalCost)
    return []
    util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def manhattan(state, problem):
    current_location = state  
    goal_location = [1,1]  
    manhattan_distance = abs(current_location[0] - goal_location[0]) + abs(current_location[1] - goal_location[1])
    return manhattan_distance


def aStarSearch(problem, heuristic=manhattan):
    explorednodes = set()
    fringe = util.PriorityQueue()
    startState = problem.getStartState()
    fringe.push((startState, []), heuristic(startState, problem))

    while not fringe.isEmpty():
        state, path = fringe.pop()
        if problem.isGoalState(state):
            return path
        if state not in explorednodes:
            explorednodes.add(state)
            for successor, action, stepCost in problem.expand(state):
                if successor not in explorednodes:
                    newPath = path + [action]
                    totalCost = problem.getCostOfActionSequence(
                        newPath) + heuristic(successor, problem)
                    fringe.push((successor, newPath), totalCost)

    return []
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
ucs = uniformCostSearch
astar = aStarSearch



