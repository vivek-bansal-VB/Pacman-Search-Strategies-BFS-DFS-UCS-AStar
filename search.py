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
import searchAgents

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
    Approach :
    1) Take a stack and push InitialState into it.
    2) Pop a node from stack and check if it is a goal state. If Yes, then return the sequences of moves to reach that goal state.
    3) Else, find all the successors of curr_state. If it is not visited push it into fringelist stack and mark that 
       node as visited. 
    4) Keep doing this until the stack becomes empty.
    """
    FringeList = util.Stack()
    Visited_States = []
    FringeList.push((problem.getStartState(),[]))                  # push starting state into stack
    
    while not FringeList.isEmpty():
        curr_state, moves = FringeList.pop()
        if problem.isGoalState(curr_state):
            return moves                                # if goal state return sequence of moves to reach that state
        Visited_States.append(curr_state)
        successors = problem.getSuccessors(curr_state)
        for succ in successors:
            succ_state = succ[0]
            succ_action = succ[1]
            succ_cost = succ[2]
            if not succ_state in Visited_States:
                move = moves + [succ_action]
                FringeList.push((succ_state, move)) # not using succ_cost as we consider cost to be same for all edges in DFS
                #Visited_States.append(curr_state)
    return []
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    
    """
    Approach :
    1) Take a Queue and enqueue InitialState into it and mark that state as visited.
    2) Deque a node from Queue and check if it is a goal state. If Yes, then return the sequences of moves to reach that goal state.
    3) Else, Append the dequeued node from the queue to the visited list. Find all the successors of dequeued node and enqueue them into Queue. 
    4) Keep doing this until the queue becomes empty.
    """
    FringeList = util.Queue()
    Visited_States = []
    FringeList.push((problem.getStartState(),[]))              # push initial state into queue
    
    while not FringeList.isEmpty():
        curr_state, moves = FringeList.pop()
        if not curr_state in Visited_States:
            Visited_States.append(curr_state)
            if problem.isGoalState(curr_state):
                return moves                        # if reached goal state then return sequence of moves
            for succ in problem.getSuccessors(curr_state):
                succ_state = succ[0]
                succ_action = succ[1]
                if not succ_state in Visited_States:
                    move = moves + [succ_action]
                    FringeList.push((succ_state, move)) #enqueue into fringelist if not visited
                    #Visited_States.append(curr_state)
    return []
    util.raiseNotDefined()

def uniformCostSearch(problem):
    
    """
    Approach :
    1) Take a Priority Queue and enqueue InitialState into it.
    2) Deque a node from Queue and check if it is a goal state. If Yes, then return the sequences of moves to reach that goal state.
    3) Find all the successors of dequeued node and enqueue them into Queue with their costs computed. If not visited
       push it into priority queue.
    4) Keep doing this until the queue becomes empty.
    """
    FringeList = util.PriorityQueue();          # priority queue
    Visited_States = [];
    FringeList.push( (problem.getStartState(),[],0),0);    #last argument denotes priority
    
    while not FringeList.isEmpty():
        curr_state, moves, curr_cost = FringeList.pop();
        if not curr_state in Visited_States:
            Visited_States.append(curr_state)
            if (problem.isGoalState(curr_state)):
                return moves;
        #Visited.append(curr_state)
            for succ in problem.getSuccessors(curr_state):
                succ_state = succ[0];
                succ_action = succ[1];
                succ_cost = succ[2];                        # succ_cost is the cost to reach a successor from a current_state
                if(succ_state not in Visited_States):
                    move = moves + [succ_action]
                    FringeList.push((succ_state, move, curr_cost + succ_cost), curr_cost + succ_cost)
                #Visited.append(succ_state)
    return []
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    
    """
    Approach :
    1) Take a Priority Queue and enqueue InitialState into it and mark that state as visited.
    2) Deque a node from Queue and check if it is a goal state. If Yes, then return the sequences of moves to reach that goal state.
    3) Find all the successors of dequeued node and enqueue them into Queue with their total costs computed 
       including the heuristic value as well. Priority is decided by the total cost(f) = cost to reach this state(g)+ heuristic value at this state(h).
    4) Keep doing this until the queue becomes empty.
    """
    FringeList = util.PriorityQueue();              # priority queue
    Visited_States = [];
    FringeList.push( (problem.getStartState(),[],0),heuristic(problem.getStartState(),problem)); # push initial state with its heuristic
    
    while not FringeList.isEmpty():
        curr_state, moves, curr_cost = FringeList.pop();
        if not curr_state in Visited_States:
            Visited_States.append(curr_state)
            if (problem.isGoalState(curr_state)):
                return moves;                       # reach the goal state so return that state
            #Visited.append(curr_state)
            for succ in problem.getSuccessors(curr_state):
                succ_state = succ[0];
                succ_action = succ[1];
                succ_cost = succ[2];
                succ_hval = heuristic(succ_state, problem);  # heuristic value at this state
                if(succ_state not in Visited_States):
                    move = moves + [succ_action]
                    FringeList.push((succ_state, move, curr_cost + succ_cost), curr_cost + succ_cost + succ_hval);
                #Visited.append(succ_state)
    return []
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
