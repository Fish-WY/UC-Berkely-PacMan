# myTeam.py
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

import math
from captureAgents import CaptureAgent
import random, time, util ,sys
from game import Directions
import game
from util import nearestPoint

#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first = 'DefensiveMCT', second = 'AStarOffensive'):
    #DefensiveMCT
    #DefensiveReflexAgent
  """
  This function should return a list of two agents that will form the
  team, initialized using firstIndex and secondIndex as their agent
  index numbers.  isRed is True if the red team is being created, and
  will be False if the blue team is being created.

  As a potentially helpful development aid, this function can take
  additional string-valued keyword arguments ("first" and "second" are
  such arguments in the case of this function), which will come from
  the --redOpts and --blueOpts command-line arguments to capture.py.
  For the nightly contest, however, your team will be created without
  any extra arguments, so you should make sure that the default
  behavior is what you want for the nightly contest.
  """

  # The following line is an example only; feel free to change it.
  return [eval(first)(firstIndex), eval(second)(secondIndex)]

##########
# Agents #
##########
class ReflexCaptureAgent(CaptureAgent):
        """
        A base class for reflex agents that chooses score-maximizing actions
        """

        def registerInitialState(self, gameState):
            self.start = gameState.getAgentPosition(self.index)
            CaptureAgent.registerInitialState(self, gameState)
            self.midWidth = gameState.data.layout.width / 2
            self.midHeight = (gameState.data.layout.height - 2) / 2
            # get legal centreposition of the map
            self.centrePosition = nearestPoint((self.midWidth, self.midHeight))
            x, y = self.centrePosition
            i = 1
            if self.red:
                i = -1
                x = x + i;
            while gameState.hasWall(x, y):
                x = x + i
            self.centrePosition = (x, y)
            self.debugDraw(self.centrePosition, [1, 0, 0], True)
            self.location = self.centrePosition;
            self.preFood = self.getFoodYouAreDefending(gameState).asList()
            self.opponentOffensiveAgent = []
            #self.noisy =[]
            self.noisy2 = [0,0,0,0]
            self.myPos2 = (0,0)
            #self.noisy[self.index] = gameState.getAgentDistances()
            # Get the legal positions that agents could be in.
            self.legalPositions = [p for p in gameState.getWalls().asList(False) if p[1] > 1]

        def chooseAction(self, gameState):
            """
            Picks among the actions with the highest Q(s,a).
            """
            actions = gameState.getLegalActions(self.index)

            # You can profile your evaluation time by uncommenting these lines
            # start = time.time()
            values = [self.evaluate(gameState, a) for a in actions]
            # print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)

            maxValue = max(values)
            bestActions = [a for a, v in zip(actions, values) if v == maxValue]

            return random.choice(bestActions)

        def getSuccessor(self, gameState, action):
            """
            Finds the next successor which is a grid position (location tuple).
            """
            successor = gameState.generateSuccessor(self.index, action)
            pos = successor.getAgentState(self.index).getPosition()
            if pos != nearestPoint(pos):
                # Only half a grid position was covered
                return successor.generateSuccessor(self.index, action)
            else:
                return successor

        def evaluate(self, gameState, action):
            """
            Computes a linear combination of features and feature weights
            """
            features = self.getFeatures(gameState, action)
            weights = self.getWeights(gameState, action)
            return features * weights

        def getFeatures(self, gameState, action):
            """
            Returns a counter of features for the state
            """
            features = util.Counter()
            successor = self.getSuccessor(gameState, action)
            features['successorScore'] = self.getScore(successor)
            return features

        def getWeights(self, gameState, action):
            """
            Normally, weights do not depend on the gamestate.  They can be either
            a counter or a dictionary.
            """
            return {'successorScore': 1.0}
#####
#offensive
#####
class AStarOffensive(ReflexCaptureAgent):
  """
  A Dummy agent to serve as an example of the necessary agent structure.
  You should look at baselineTeam.py for more details about how to
  create an agent as this is the bare minimum.
  """

  def registerInitialState(self, gameState):
    """
    This method handles the initial setup of the
    agent to populate useful fields (such as what team
    we're on).

    A distanceCalculator instance caches the maze distances
    between each pair of positions, so your agents can use:
    self.distancer.getDistance(p1, p2)

    IMPORTANT: This method may run for at most 15 seconds.
    """
    CaptureAgent.registerInitialState(self, gameState)
    self.start = gameState.getAgentPosition(self.index)
    foodList = self.getFood(gameState).asList()
    safeFood = self.getSafeFood(foodList,gameState)
    if self.red:
        centralX = (gameState.data.layout.width - 2) / 2
    else:
        centralX = ((gameState.data.layout.width - 2) / 2) + 1
    self.noWallSpots = []
    for i in range(1, gameState.data.layout.height - 1):
        if not gameState.hasWall(centralX, i):
            self.noWallSpots.append((centralX, i))
    #self.debugDraw(self.noWallSpots, [1, 0, 0], True)
    self.stopLastTime = False

  def getSafeFood(self, foodList, gameState):
    #print len(foodList)
    safeFood = []
    for x, y in foodList:
      if self.checkFood(x, y, gameState):
        safeFood.append((x, y))
    return safeFood

  def checkFood(self,x,y,gameState):
    k = 0
    if gameState.hasWall(x + 1,y)  or gameState.hasFood(x + 1, y):
      k = k + 1
    if gameState.hasWall(x - 1,y)  or gameState.hasFood(x - 1, y):
      k = k + 1
    if gameState.hasWall(x, y + 1) or gameState.hasFood(x, y + 1):
      k = k + 1
    if gameState.hasWall(x, y - 1)  or gameState.hasFood(x, y - 1):
      k = k + 1
    if k <= 2 :
      return True
    return False



  def chooseAction(self, gameState):
    """
    Picks among actions randomly.
    """
    startactions = gameState.getLegalActions(self.index)

    '''
    You should change this in your own agent.
    '''
    # You can profile your evaluation time by uncommenting these lines
    start = time.time()

    self.noisy2 = gameState.getAgentDistances()
    myState = gameState.getAgentState(self.index)
    myPos = myState.getPosition()
    self.myPos2 =myPos

    foodList = self.getFood(gameState).asList()
    self.leftFood = len(foodList)
    capList = self.getCapsules(gameState)
    self.leftCap = len(capList)
    safeFood = self.getSafeFood(foodList,gameState)
    safeFood = safeFood + capList
    #self.debugDraw(safeFood, [1, 0, 0], True)
    self.leftSafeFood = len(safeFood)
    myState = gameState.getAgentState(self.index)
    self.run = False
    self.risk = False
    if self.leftFood < 3:
      self.run = True
      #print 'go home now'
    elif self.Ghostdistance(gameState) <= 4:
      self.risk = True
      if gameState.getAgentState(self.index).numCarrying > 3:
        self.run = True
        #print 'go home now'

    visitedNode = []
    PQueue = util.PriorityQueue()
    initialState = gameState
    PQueue.push((initialState, [], 0), 0)

    while not PQueue.isEmpty():
      currentState, actionList, ppp = PQueue.pop()
      if time.time() - start >0.95:
        #print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)
        #print len(visitedNode)
        break;
      if self.isGoalState(currentState):
        if len(actionList) == 0:
          continue
        else:
          return actionList[0]
      if currentState not in visitedNode :
        visitedNode.append(currentState)
        nextactions = currentState.getLegalActions(self.index)
        for act in nextactions:
          nextState = self.getSuccessor(currentState, act)
          if self.Ghostdistance(nextState) > 1 and nextState.getAgentState(self.index).getPosition() != self.start :
            PQueue.push((nextState, actionList + [act], ppp + 1),ppp + 1 + self.heuristic(nextState))

    #print 'no way'
    # TODO choose a safe position
    safeAction = []
    if myState.isPacman:
      for act in startactions:
        nextState = self.getSuccessor(gameState, act)
        if self.Ghostdistance(nextState) > 1 and nextState.getAgentState(self.index).getPosition() != self.start :
           safeAction.append(act)
    if len(safeAction) == 1 and safeAction[0] == Directions.STOP and self.stopLastTime:
        self.stopLastTime = False
        return random.choice(startactions)
    if len(safeAction) == 1 and safeAction[0] == Directions.STOP:
        self.stopLastTime = True
        return Directions.STOP
    if len(safeAction) > 0:
        randomAction = random.choice(safeAction)
        return randomAction
    return random.choice(startactions)

  def heuristic(self,successor):

    if self.run:
      myState = successor.getAgentState(self.index)
      myPos = myState.getPosition()
      dists = [self.getMazeDistance(myPos, a) for a in self.noWallSpots]
      return min(dists)
    elif self.risk and self.leftSafeFood > 0:
      myState = successor.getAgentState(self.index)
      myPos = myState.getPosition()
      foodList = self.getFood(successor).asList()
      capList = self.getCapsules(successor)
      safefood = self.getSafeFood(foodList, successor)
      safefood= safefood + capList
      if self.leftSafeFood > len(safefood) or self.leftCap > len(capList):
        return 0
      else:
        minDistance = min([self.getMazeDistance(myPos, food) for food in safefood])
        return minDistance
    else:
      myState = successor.getAgentState(self.index)
      myPos = myState.getPosition()
      foodList = self.getFood(successor).asList()
      capList = self.getCapsules(successor)
      if self.leftFood > len(foodList) or self.leftCap > len(capList):
        return 0
      else:
        minDistance = min([self.getMazeDistance(myPos, food) for food in foodList])
        return minDistance

  def Ghostdistance(self, successor):
    myState = successor.getAgentState(self.index)
    if myState.isPacman == False:
      return 99;
    myPos = myState.getPosition()
    enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
    ghosts = [a for a in enemies if a.isPacman == False and a.getPosition() != None and a.scaredTimer < 3]

    ghostsDistance = 99
    if len(ghosts) > 0:
      dists = [self.getMazeDistance(myPos, a.getPosition()) for a in ghosts]
      ghostsDistance = min(dists)
    #if ghostsDistance < 99: print 'ghostDistance = %d' %ghostsDistance
    return  ghostsDistance

  def isGoalState(self, gameState):
    if self.run == True:
      myState = gameState.getAgentState(self.index)
      if myState.isPacman:
        return False
      else:
        return True
    elif self.risk and self.leftSafeFood > 0:
      foodList = self.getFood(gameState).asList()
      capList = self.getCapsules(gameState)
      safefood = self.getSafeFood(foodList, gameState)
      safefood = safefood + capList
      if self.leftSafeFood > len(safefood) or self.leftCap > len(capList):
        return True
      else:
        return False
    else:
      foodList = self.getFood(gameState).asList()
      capList = self.getCapsules(gameState)
      if self.leftFood > len(foodList) or self.leftCap > len(capList):
        return True
      else:
        return False

  def getSuccessor(self, gameState, action):
    """
    Finds the next successor which is a grid position (location tuple).
    """
    successor = gameState.generateSuccessor(self.index, action)
    pos = successor.getAgentState(self.index).getPosition()
    if pos != nearestPoint(pos):
      # Only half a grid position was covered
      return successor.generateSuccessor(self.index, action)
    else:
      return successor


##########
# Defence #
##########




class DefensiveReflexAgent(ReflexCaptureAgent):

    def getFeatures(self, gameState, action):
        features = util.Counter()
        successor = self.getSuccessor(gameState, action)

        myState = successor.getAgentState(self.index)
        myPos = myState.getPosition()

        # Computes whether we're on defense (1) or offense (0)
        features['onDefense'] = 1
        if myState.isPacman: features['onDefense'] = 0

        # Computes distance to invaders we can see
        enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
        for i in self.getOpponents(successor):
            if successor.getAgentState(i).isPacman and i not in self.opponentOffensiveAgent:
                self.opponentOffensiveAgent.append(i);
                # print i
        #get noisy distance
        noisy = gameState.getAgentDistances()
        print noisy
        for i in self.getOpponents(successor):
            pop =[]
            for a in self.legalPositions:
                if noisy[i] == util.manhattanDistance(myPos, a) and self.noisy2[i] == util.manhattanDistance(self.myPos2, a):
                    pop.append(a)
            self.debugDraw(pop,[0,1,0],True)

        invaders = [a for a in enemies if a.isPacman and a.getPosition() != None]
        features['numInvaders'] = len(invaders)
        # search to locate a invader
        pacmans = [a for a in enemies if a.isPacman]
        if len(pacmans) == 0:
            if len(self.opponentOffensiveAgent) == 1 and successor.getAgentState(self.opponentOffensiveAgent[0]).getPosition() != None:
                (x , y) = successor.getAgentState(self.opponentOffensiveAgent[0]).getPosition()
                x = int(x)
                y = int(y)
                if self.red:
                    while gameState.hasWall(x, y) or x > self.midWidth :
                        x = x - 1
                else:
                    while gameState.hasWall(x, y) or x < self.midWidth :
                        x = x + 1
                self.location = (x,y)
            else :
                self.location = self.centrePosition
                # print "no invender"
        currentFood = self.getFoodYouAreDefending(gameState).asList()
        # print currentFood
        if len(currentFood) < len(self.preFood):
            # print "food missed"
            for i in self.preFood:
                if i not in currentFood:
                    self.location = i
                    # print self.location
        #self.debugDraw(self.location, [1, 0, 0], True)
        self.preFood = currentFood
        # print self.preFood

        features['invaderDistance'] = 60
        features['centreDistance'] = 60
        if len(invaders) > 0:
            dists = [self.getMazeDistance(myPos, a.getPosition()) for a in invaders]
            features['invaderDistance'] = min(dists)
            features['centreDistance'] = 0;
        else:
            features['centreDistance'] = self.getMazeDistance(myPos, self.location)

        if action == Directions.STOP: features['stop'] = 1
        if action == Directions.SOUTH or action == Directions.NORTH: features['vertical'] = 1
        rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
        if action == rev: features['reverse'] = 1

        return features

    def getWeights(self, gameState, action):
        return {'numInvaders': -10000, 'onDefense': 1000, 'invaderDistance': -10, 'centreDistance': -10, 'stop': -100,
                'reverse': -2, 'vertical': 5}


######
# montecarloteam
######

class DefensiveMCT(ReflexCaptureAgent):
    def chooseAction(self, gameState):
        # You can profile your evaluation time by uncommenting these lines
        # start = time.time()
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        invaders = [a for a in enemies if a.isPacman and a.getPosition() != None]
        self.leftinvaders = len(invaders)
        actions = gameState.getLegalActions(self.index)
        actions.remove(Directions.STOP)
        values = []
        for a in actions:
            nextState = gameState.generateSuccessor(self.index, a)
            tmp = 0
            for i in range(1, 31):
                tmp += self.randomSimulation(10, nextState)
            values.append(tmp)
        maxValue = max(values)
        bestActions = [a for a, v in zip(actions, values) if v == maxValue]

        return random.choice(bestActions)

    def randomSimulation(self, depth, gameState):

      nextState = gameState.deepCopy()
      while depth > 0:
        actions = nextState.getLegalActions(self.index)
        actions.remove(Directions.STOP)
        reversed_direction = Directions.REVERSE[nextState.getAgentState(self.index).configuration.direction]
        if reversed_direction in actions and len(actions) > 1:
          actions.remove(reversed_direction)
        a = random.choice(actions)
        nextState = nextState.generateSuccessor(self.index, a)
        enemies = [nextState.getAgentState(i) for i in self.getOpponents(nextState)]
        invaders = [a for a in enemies if a.isPacman and a.getPosition() != None]
        if len(invaders) < self.leftinvaders:
            break
        depth -= 1
      return self.evaluate(nextState, Directions.STOP)

    def getFeatures(self, gameState, action):
        features = util.Counter()
        successor = self.getSuccessor(gameState, action)

        myState = successor.getAgentState(self.index)
        myPos = myState.getPosition()

        # Computes whether we're on defense (1) or offense (0)
        features['onDefense'] = 1
        if myState.isPacman: features['onDefense'] = 0

        # Computes distance to invaders we can see
        enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
        for i in self.getOpponents(successor):
          if successor.getAgentState(i).isPacman and i not in self.opponentOffensiveAgent:
            self.opponentOffensiveAgent.append(i);
            #print i


        invaders = [a for a in enemies if a.isPacman and a.getPosition() != None]
        features['numInvaders'] = len(invaders)
        # search to locate a invader
        pacmans = [a for a in enemies if a.isPacman]
        if len(pacmans) == 0:
          self.location = self.centrePosition
          #print "no invender"
        currentFood = self.getFoodYouAreDefending(gameState).asList()
        #print currentFood
        if len(currentFood)<len(self.preFood):
          #print "food missed"
          for i in self.preFood:
            if i not in currentFood:
              self.location = i
              #print self.location
        self.preFood = currentFood
        #print self.preFood

        features['invaderDistance'] = 60
        features['centreDistance'] = 60
        if len(invaders) > 0:
          dists = [self.getMazeDistance(myPos, a.getPosition()) for a in invaders]
          features['invaderDistance'] = min(dists)
          features['centreDistance'] = 0;
        else:
            features['centreDistance'] = self.getMazeDistance(myPos,self.location)

        return features

    def getWeights(self, gameState, action):
        return {'numInvaders': -1000, 'onDefense': 1000, 'invaderDistance': -10,'centreDistance':-10 }


