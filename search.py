# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
"""

import util
import time

class SearchProblem:
  """
  This class outlines the structure of a search problem, but doesn't implement
  any of the methods (in object-oriented terminology: an abstract class).
  
  You do not need to change anything in this class, ever.
  """
  
  def getStartState(self):
     """
     Returns the start state for the search problem 
     """
     util.raiseNotDefined()
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take
 
     This method returns the total cost of a particular sequence of actions.  The sequence must
     be composed of legal moves
     """
     util.raiseNotDefined()
           

def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first
  [2nd Edition: p 75, 3rd Edition: p 87]
  
  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm 
  [2nd Edition: Fig. 3.18, 3rd Edition: Fig 3.7].
  
  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:
  
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  """
  "*** YOUR CODE HERE ***"
  # use a list to remember the current path
  # use a list to remember the places where there was a branch
  optimalPath = []
  currentState = problem.getStartState()
  prevBranch = []
  traversed = []
  lastNode = None
  traversed.append(currentState)

  # while we didn't find the solution, get the possible moves, 
  # remove the already traversed moves from the possible moves,
  # if there's only one remaining possible move, then push that move to list
  # if there's more than one remaining possible move, push the choices 
  while problem.isGoalState(currentState) == False:
    choices = problem.getSuccessors(currentState)

    # get a list of the nodes that have not been traversed yet
    new_choices = []
    for choice in choices:
      if choice[0] not in traversed:
        new_choices.append(choice)
  
    # if there's only one node that have not been traversed yet, move in that direction
    if len(new_choices) == 1:
      currentState = new_choices[0][0]
      optimalPath.append(new_choices[0])
      traversed.append(new_choices[0][0])

    # if there's more than one node that have not been traversed yet
    # push the node into the prevBranch list
    elif len(new_choices) > 1:
      currentState = new_choices[0][0]
      optimalPath.append(new_choices[0])
      traversed.append(new_choices[0][0])
      prevBranch.append(new_choices)
    
    # when there are no more choices, go back up the tree and pop nodes off optimalPath
    # until reaching the last branch, and traverse one untraversed node in that branch
    else:
      if len(optimalPath) == 0:
        currentState = problem.getStartState()
        continue


      if len(prevBranch) > 0:
        lastBranch = prevBranch.pop() 
      
      while currentState != lastBranch[0][0]:
        lastNode = optimalPath.pop()
        currentState = lastNode[0]
      
      currentState = optimalPath[-1][0]

  # return the directions
  directions = []
  for node in optimalPath:
    directions.append(node[1])

  return directions
      

def breadthFirstSearch(problem):
  """
  Search the shallowest nodes in the search tree first.
  [2nd Edition: p 73, 3rd Edition: p 82]
  """
  "*** YOUR CODE HERE ***"
  # a queue to store the nodes in current layer
  # a paths dictionary to know which nodes are where
  current_layer = util.Queue()
  paths = {}
  traversed = []

  current_layer.push(problem.getStartState())
  shortest_path = []
  traversed.append(problem.getStartState())


  # go through all the nodes in current_layer
  while current_layer.isEmpty() == False:
    current_state = current_layer.pop()
    if problem.isGoalState(current_state) == True:
      shortest_path = paths[current_state]
      break

    current_path = []
    if current_state in paths:
      current_path = paths.pop(current_state, None)

    # print "current state", current_state
    # print "current path", current_path
    # print "current layer", current_layer.printSelf()
    # print "traversed", traversed

    
    choices = problem.getSuccessors(current_state)
    new_choices = []
    for choice in choices:
      if choice[0] not in traversed:
        new_choices.append(choice)
        traversed.append(choice[0])

    if len(new_choices) == 0:
      continue
    else:
      for new_choice in new_choices:
        new_path = current_path[:]
        new_path.append(new_choice)
        paths[new_choice[0]] = new_path
        current_layer.push(new_choice[0])

    # print "new_choices", new_choices
    # print "paths after", paths
    # print
    
    
    

  directions = []
  for node in shortest_path:
    directions.append(node[1])
  return directions
      
    
    
    

  # while problem.isGoalState(currentState) == False:
  #   choices = problem.getSuccessors(currentState)
  #   new_choices = []
  #   for choice in choices:
  #     if choice not in traversed:
  #       new_choices.append(choice)
    
  #   if len(new_choices) == 1:
  #     optimalPath.append(new_choices[0])
  #     currentState = new_choices[0][0]
  #     traversed.append(currentState)
      
  #   elif len(new_choices) > 1:
  #     current_layer.extend(new_choices)





  util.raiseNotDefined()
      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"
  util.raiseNotDefined()

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"
  util.raiseNotDefined()
    
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
