# multiAgents.py
# --------------
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


from util import manhattanDistance
from game import Directions
import random, util

from game import Agent

class ReflexAgent(Agent):
    """
      A reflex agent chooses an action at each choice point by examining
      its alternatives via a state evaluation function.

      The code below is provided as a guide.  You are welcome to change
      it in any way you see fit, so long as you don't touch our method
      headers.
    """


    def getAction(self, gameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {North, South, West, East, Stop}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices) # Pick randomly among the best

        "Add more of your code here if you want to"

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState, action):
        """
        Design a better evaluation function here.

        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.

        Print out these variables to see what you're getting, then combine them
        to create a masterful evaluation function.
        """
        # Useful information you can extract from a GameState (pacman.py)
        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        newGhostStates = successorGameState.getGhostStates()
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]
        newGhostPositions = successorGameState.getGhostPositions()

        # if next state is win or lose
        if successorGameState.isWin():
            return float("inf") - 20
        if successorGameState.isLose():
          return float("-inf") +20

        score = 0

        # minimum distance to nearest food
        next_food = 1000
        for (ind_r, ind_c) in newFood.asList():
            next_food = min(next_food, manhattanDistance((ind_r, ind_c), newPos))

        # if we eat a food in next step, increase score
        if (currentGameState.getNumFood() > successorGameState.getNumFood()):
            score += next_food * 7

        if action == "Stop":
          score -= 3
        score -= next_food * 2
        score -= random.randint(0, 10) * 0.2

        # avoid ghosts within 3 steps
        for ghost in newGhostPositions:
            md = manhattanDistance(ghost, newPos)
            if md == 0:
              score -= next_food * 10
            if md == 1:
                score -= next_food * 5
            elif md == 2:
                score -= next_food * 3
            elif md == 3:
              score -= next_food * 2

        # optimize for eating capsules 
        capsuleplaces = currentGameState.getCapsules()
        if successorGameState.getPacmanPosition() in capsuleplaces:
            score += next_food * 5

        return score

def scoreEvaluationFunction(currentGameState):
    """
      This default evaluation function just returns the score of the state.
      The score is the same one displayed in the Pacman GUI.

      This evaluation function is meant for use with adversarial search agents
      (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
      This class provides some common elements to all of your
      multi-agent searchers.  Any methods defined here will be available
      to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

      You *do not* need to make any changes here, but you can if you want to
      add functionality to all your adversarial search agents.  Please do not
      remove anything, however.

      Note: this is an abstract class: one that should not be instantiated.  It's
      only partially specified, and designed to be extended.  Agent (game.py)
      is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
    """
      Your minimax agent (question 2)
    """

    def getAction(self, gameState, current_depth=0):
        """
          Returns the minimax action from the current gameState using self.depth
          and self.evaluationFunction.

          Here are some method calls that might be useful when implementing minimax.

          gameState.getLegalActions(agentIndex):
            Returns a list of legal actions for an agent
            agentIndex=0 means Pacman, ghosts are >= 1

          gameState.generateSuccessor(agentIndex, action):
            Returns the successor game state after an agent takes an action

          gameState.getNumAgents():
            Returns the total number of agents in the game
        """
        # get each agent's legal actions
        actions = gameState.getLegalActions(0)
        num_agents = gameState.getNumAgents()
        agent_actions = []
        for agent_index in range(num_agents):
          agent_actions.append(gameState.getLegalActions(agent_index))

        

        # for the new_game_state, find best move for agent
        if current_depth == self.depth: 
          # for all ghost agents, get the best possible move
          best_ghost_moves = []

          for agent_index in range(num_agents-1):
            possible_choices = agent_actions[agent_index+1][:]
            scores = []
            
            for choice in possible_choices:
              new_game_state = gameState.generateSuccessor(agent_index+1, choice)
              scores.append(self.evaluationFunction(new_game_state))
            best_ghost_moves.append(possible_choices[scores.index(min(scores))])

          new_game_state = gameState
          
          for ghost_move_index in range(len(best_ghost_moves)):
            new_game_state = new_game_state.generateSuccessor(ghost_move_index+1, best_ghost_moves[ghost_move_index])


          agent_choice_scores = []
          for choice in agent_actions[0]:
            new_agent_game = new_game_state.generateSuccessor(0, choice)
            agent_choice_scores.append(self.evaluationFunction(new_agent_game))
          
          print agent_choice_scores
          index_best_move = agent_choice_scores.index(max(agent_choice_scores))
          return agent_actions[0][index_best_move]
        
        else:
          best_game_state
          return self.getAction()

              



            


        # successors = gameState.generateSuccessor(0, )

        
        # for each depth, get successors
        current_depth = 0
        


        
        

class AlphaBetaAgent(MultiAgentSearchAgent):
    """
      Your minimax agent with alpha-beta pruning (question 3)
    """

    def getAction(self, gameState):
        """
          Returns the minimax action using self.depth and self.evaluationFunction
        """
        "*** YOUR CODE HERE ***"
        util.raiseNotDefined()

class ExpectimaxAgent(MultiAgentSearchAgent):
    """
      Your expectimax agent (question 4)
    """

    def getAction(self, gameState):
        """
          Returns the expectimax action using self.depth and self.evaluationFunction

          All ghosts should be modeled as choosing uniformly at random from their
          legal moves.
        """
        "*** YOUR CODE HERE ***"
        util.raiseNotDefined()

def betterEvaluationFunction(currentGameState):
    """
      Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
      evaluation function (question 5).

      DESCRIPTION: <write something here so we know what you did>
    """
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

# Abbreviation
better = betterEvaluationFunction

