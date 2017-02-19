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
  corners_problem = False

  start_state = problem.getStartState()
  print start_state
  if not isinstance(start_state[1], int):
    corners_problem = True


  if not corners_problem:
    current_layer.push(start_state)
    shortest_path = []
    traversed.append(start_state)

    # go through all the nodes in current_layer, until current_layer is empty
    while current_layer.isEmpty() == False:
      current_state = current_layer.pop()

      # if we found the goal state, then get the current path and break
      if problem.isGoalState(current_state):
        print "current state", current_state
        shortest_path = paths[current_state]
        break

      # otherwise, get the current path for that node
      current_path = []
      if current_state in paths:
        current_path = paths.pop(current_state, None)

      # get the untraversed possible moves for current node
      choices = problem.getSuccessors(current_state)
      new_choices = []
      for choice in choices:
        if choice[0] not in traversed:
          new_choices.append(choice)
          traversed.append(choice[0])
      
      # if there's no more untraversed possible moves, 
      # then terminate search on that path
      # otherwise, make deep copy of current path, push new node onto the copy
      # and put new path into paths dictionary
      if len(new_choices) == 0:
        continue
      else:
        for new_choice in new_choices:
          new_path = current_path[:]
          new_path.append(new_choice)
          paths[new_choice[0]] = new_path
          current_layer.push(new_choice[0])

  
  # if problem is corners problem
  else: 
    corners_state = start_state[1][:]
    # print corners_state
    current_layer.push(start_state[0])
    shortest_path = []
    traversed.append(start_state[0])

    # find shortest path to first corner
    while current_layer.isEmpty() == False:
      current_state = current_layer.pop()
      

      # if we traversed to a corner in current start_state
      # if traversed all corners, then add current path to shortest_path and return iter
      # otherwise, reset Queue, traversed, and paths and add current path to shortest_path
      if len(problem.isGoalState(current_state)) > len(corners_state):
        if len(problem.isGoalState(current_state)) == 4:
          shortest_path += paths[current_state]
          break
        else:
          corners_state = problem.isGoalState(current_state)[:]
          shortest_path += paths[current_state]
          paths = {}
          traversed = []
          traversed.append(current_state)
          current_layer = util.Queue()
          current_layer.push(current_state)

      # otherwise, get the current path for that node
      current_path = []
      if current_state in paths:
        current_path = paths.pop(current_state, None)

      # get the untraversed possible moves for current node
      choices = problem.getSuccessors(current_state)
      new_choices = []
      for choice in choices:
        if choice[0] not in traversed:
          new_choices.append(choice)
          traversed.append(choice[0])
      
      # if there's no more untraversed possible moves, 
      # then terminate search on that path
      # otherwise, make deep copy of current path, push new node onto the copy
      # and put new path into paths dictionary
      if len(new_choices) == 0:
        continue
      else:
        for new_choice in new_choices:
          new_path = current_path[:]
          new_path.append(new_choice)
          paths[new_choice[0]] = new_path
          current_layer.push(new_choice[0])

    # get directions in "South", "West", "East", and "North" form
    directions = [node[1] for node in shortest_path]
    return directions


      


      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"
  # use a priority queue to store the current layer
  # use a traversed list for nodes that are already traversed
  frontier = util.PriorityQueue()
  traversed = []


  # perform the first loop:
  # get start node and its successors, compute value and push into frontier
  current_node = problem.getStartState()
  traversed.append(current_node)
  actions = problem.getSuccessors(current_node)

  for action in actions:
    actions_list = []
    path = []
    path.append(action)
    actions_list.append(action[1])
    cost = problem.getCostOfActions(actions_list)
    frontier.push(path, cost)
    traversed.append(action[0])

  # while the frontier isn't empty, pop and examine the next path with the smallest cost
  while frontier.isEmpty() == False:
    current_path = frontier.pop()
    current_node = current_path[-1]

    # if we reached the goal, break and return shortest path
    if problem.isGoalState(current_node[0]):
      shortest_path = [node[1] for node in current_path]
      return shortest_path
      break

    # get the new_choices and new traversed list
    choices = problem.getSuccessors(current_node[0])
    new_choices, traversed = getNewChoices(choices, traversed)     

    # if no more choices, then end looking into this path
    if len(new_choices) == 0:
      continue

    # concat current path to new choice, get cost, and push into frontier
    for new_choice in new_choices:
      new_path = current_path[:]
      new_path.append(new_choice)
      new_cost_compute = [node[1] for node in new_path]
      new_cost = problem.getCostOfActions(new_cost_compute)
      frontier.push(new_path, new_cost)


def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  """
  Search the node that has the lowest combined cost and heuristic first. 
  total cost = cost of actions + distance from goal according to manhattan heuristic
  """
  # use a priority queue to store the current layer
  # use a traversed list for nodes that are already traversed
  frontier = util.PriorityQueue()
  traversed = []


  # perform the first loop:
  # get start node and its successors, compute value and push into frontier
  current_node = problem.getStartState()
  traversed.append(current_node)
  actions = problem.getSuccessors(current_node)

  for action in actions:
    actions_list = []
    path = []
    path.append(action)
    actions_list.append(action[1])
    cost = problem.getCostOfActions(actions_list)
    cost += heuristic(current_node, problem)
    frontier.push(path, cost)
    traversed.append(action[0])

  # while the frontier isn't empty, pop and examine the next path with the smallest cost
  while frontier.isEmpty() == False:
    current_path = frontier.pop()
    current_node = current_path[-1]

    # if we reached the goal, break and return shortest path
    if problem.isGoalState(current_node[0]):
      shortest_path = [node[1] for node in current_path]
      return shortest_path
      break

    # get the new_choices and new traversed list
    choices = problem.getSuccessors(current_node[0])
    new_choices, traversed = getNewChoices(choices, traversed)     

    # if no more choices, then end looking into this path
    if len(new_choices) == 0:
      continue

    # concat current path to new choice, get cost, and push into frontier
    for new_choice in new_choices:
      new_path = current_path[:]
      new_path.append(new_choice)
      new_cost_compute = [node[1] for node in new_path]
      new_cost = problem.getCostOfActions(new_cost_compute) + heuristic(current_node[0], problem)
      frontier.push(new_path, new_cost)
  util.raiseNotDefined()

def getNewChoices(choices, traversed):
  """
    input: a list of choices and a traversed list
    output: a list of the choices that are not in the traversed list
  """
  new_choices = []
  for choice in choices:
    if choice[0] not in traversed:
      new_choices.append(choice)
      traversed.append(choice[0])
  return new_choices, traversed
    
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
