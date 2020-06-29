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
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    frontier = util.Stack()
    frontier.push(start)
    explored = set()
    actions = {}
    actions[start] = []

    while not frontier.isEmpty():
        node = frontier.pop()
        explored.add(node)

        if problem.isGoalState(node):
            return actions[node]

        for x in problem.getSuccessors(node):
            nextNode = x[0]
            nextAction = x[1]
            
            if nextNode not in explored:
                frontier.push(nextNode)
                actions[nextNode] = actions[node] + [nextAction]
                
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    frontier = util.Queue()
    frontier.push(start)
    frontierSet = set()
    explored = set()
    actions = {}
    actions[start] = []

    while not frontier.isEmpty():
        node = frontier.pop()
        explored.add(node)

        if problem.isGoalState(node):
            return actions[node]

        for x in problem.getSuccessors(node):
            nextNode = x[0]
            nextAction = x[1]

            if nextNode not in explored and nextNode not in frontierSet:
                frontier.push(nextNode)
                actions[nextNode] = actions[node] + [nextAction]
                frontierSet.add(nextNode)

    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    frontier = util.PriorityQueue()
    explored = []

    frontier.push((start, [], 0), 0)

    while not frontier.isEmpty():
        c_state, c_actions, c_cost = frontier.pop()
        node = (c_state, c_cost)
        explored.append(node)

        if problem.isGoalState(c_state):
            return c_actions

        else:
            for x in problem.getSuccessors(c_state):
                s_state = x[0]
                s_action = x[1]
                nextAction = c_actions + [s_action]
                nextCost = problem.getCostOfActions(nextAction)
                nextNode = (s_state, nextAction, nextCost)

                has_explored = False

                for e in explored:
                    e_state, e_cost = e

                    if (x[0] == e_state) and (nextCost > e_cost):
                        has_explored = True

                if not has_explored:
                    frontier.push(nextNode, nextCost)
                    explored.append((s_state, nextCost))
                    
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    frontier = util.PriorityQueue()
    explored = []

    frontier.push((start, [], 0), 0)

    while not frontier.isEmpty():
        c_state, c_action, c_cost = frontier.pop()
        c_node = (c_state, c_cost)
        explored.append(c_node)

        if problem.isGoalState(c_state):
            return c_action

        else:
            for x in problem.getSuccessors(c_state):
                s_state = x[0]
                s_action = x[1]
                nextAction = c_action + [s_action]
                nextCost = problem.getCostOfActions(nextAction)
                nextNode = (s_state, nextAction, nextCost)

                has_explored = False

                for e in explored:
                    e_state, e_cost = e

                    if (x[0] == e_state) and (nextCost >= e_cost):
                        has_explored = True

                if not has_explored:
                    frontier.push(nextNode, nextCost + heuristic(s_state, problem))
                    s_node = (s_state, nextCost)
                    explored.append(s_node)

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
