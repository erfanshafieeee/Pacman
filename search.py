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
    return  [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    fringe = util.Stack()
    exploredNodes = []
    startState = problem.getStartState()
    startNode = (startState , [])
    fringe.push(startNode)
    while not fringe.isEmpty():
        currentState , actions = fringe.pop()
        if currentState not in exploredNodes:
            exploredNodes.append(currentState)
            if problem.isGoalState(currentState):
                return actions
            else:
                successors = problem.getSuccessors(currentState)
                for succState , succAction , succCost in successors:
                    newAction = actions + [succAction]
                    newNode = (succState , newAction)
                    fringe.push(newNode)
    return actions                


def breadthFirstSearch(problem):
    fringe = util.Queue()
    exploredNodes = []
    startState = problem.getStartState()
    startNode = (startState , [] , 0)
    fringe.push(startNode)
    while not fringe.isEmpty():
        currentState , actions , currentCost = fringe.pop()
        if currentState not in exploredNodes:
            exploredNodes.append(currentState)
            if problem.isGoalState(currentState):
                return actions
            else:
                successors = problem.getSuccessors(currentState)
                for succState , succAction , succCost in successors:
                    newAction = actions + [succAction]
                    newCost = currentCost + succCost
                    newNode = (succState , newAction , newCost)

                    fringe.push(newNode)
    return actions

def uniformCostSearch(problem):
    frontier = util.PriorityQueue()
    exploredNodes = {}
    startState = problem.getStartState()
    startNode = (startState, [], 0) 
    frontier.push(startNode, 0)
    while not frontier.isEmpty(): 
        currentState, actions, currentCost = frontier.pop()
        if (currentState not in exploredNodes) or (currentCost < exploredNodes[currentState]):   
            exploredNodes[currentState] = currentCost
            if problem.isGoalState(currentState):
                return actions
            else:
                successors = problem.getSuccessors(currentState)
                for succState, succAction, succCost in successors:
                    newAction = actions + [succAction]
                    newCost = currentCost + succCost
                    newNode = (succState, newAction, newCost)
                    frontier.update(newNode, newCost)

    return actions

# def nullHeuristic(state, problem=None):
#     """
#     A heuristic function estimates the cost from the current state to the nearest
#     goal in the provided SearchProblem.  This heuristic is trivial.
#     """
#     return 0

def MANHATTANHeuristic(state, problem):
    current_location = state  
    goal_location = problem.goalState  
    manhattan_distance = abs(current_location[0] - goal_location[0]) + abs(current_location[1] - goal_location[1])
    return manhattan_distance


def aStarSearch(problem, heuristic=MANHATTANHeuristic):
    explored = set() 
    frontier = util.PriorityQueue() 
    startState = problem.getStartState()
    frontier.push((startState, []), heuristic(startState, problem)) 

    while not frontier.isEmpty():
        state, path = frontier.pop()
        if problem.isGoalState(state):
            return path 
        if state not in explored:
            explored.add(state)  
            for successor, action,stepCost in problem.expand(state):
                if successor not in explored:
                    newPath = path + [action]
                    totalCost = problem.getCostOfActionSequence(newPath) + heuristic(successor, problem)
                    frontier.push((successor, newPath), totalCost)

    return [] 
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
