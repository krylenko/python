# (c) daniel ford, daniel.jb.ford@gmail.com

############################################################################
# blackbox optimization maze solver for infomax robot controller
############################################################################

__author__ = 'Daniel Ford, dford@email.arizona.edu'

from scipy import array
from pybrain.rl.learners.valuebased import ActionValueTable

# import optimization frameworks and agent class
from pybrain.optimization import *
from pybrain.rl.agents import OptimizationAgent
from pybrain.rl.experiments import EpisodicExperiment

# import neural net stuff
from pybrain.structure.networks.network import Network
from pybrain.tools.shortcuts import buildNetwork

# import tasks and environments
from pybrain.rl.environments.mazes import Maze
from pybrain.rl.environments.cartpole.balancetask import BalanceTask
from policyTask import policyTask
from policyEnv import policyEnv
from MDPMazeTaskEpisodic import MDPMazeTaskEpisodic
from mazeEnv import mazeEnv

# comment out "DEBUG = False" to run debug script
DEBUG = True
DEBUG = False

NUM_EPISODES = 8
AT_GOAL = False
# action enum, basically
NORTH = 0; EAST = 1; SOUTH = 2; WEST = 3

# actual code in progress
if not DEBUG:

  maxEvals = 1000

  # set up maze and state/action params
  structure = 	array([[1, 1, 1, 1, 1, 1, 1, 1, 1],
                         [1, 0, 0, 0, 0, 0, 0, 0, 1],
                         [1, 0, 0, 0, 0, 1, 1, 1, 1],
                         [1, 0, 0, 0, 0, 0, 0, 0, 1],
                         [1, 0, 0, 0, 0, 0, 0, 0, 1],
                         [1, 0, 0, 0, 0, 0, 0, 0, 1],
                         [1, 1, 1, 1, 1, 1, 0, 0, 1],
                         [1, 0, 0, 0, 0, 0, 0, 0, 1],
                         [1, 1, 1, 1, 1, 1, 1, 1, 1]])

  side = 9
  goal = 3,2

  env = mazeEnv(structure, goal)   #use maze environment for now; note pos is Y,X

  # our own task and environment for later
  #env = policyEnv()
  thetask = MDPMazeTaskEpisodic(env)

  # create neural net; create and train agent
  theparams = buildNetwork(thetask.outdim, thetask.indim, bias=False)
  agent = OptimizationAgent(theparams, CMAES())
  exp = EpisodicExperiment(thetask, agent)

  # train agent        
  exp.doEpisodes(NUM_EPISODES)
  print "\ntotal reward = ",thetask.getTotalReward()

  #print "\n"
  #print "initial weights: "; print theparams.params
  print "\n"
  print "NOTE positions below are (Y,X)"

  print "\n"
  print "getting observation 1"
  print "robot = ",thetask.getObservation()
  print "goal  = ",goal
  print "reward: ", thetask.getReward()

  print "\n"
  print "performing action 1"
  thetask.performAction([NORTH])
  print "reward: ", thetask.getReward()

  print "\n"
  print "getting observation 2"
  print "robot = ",thetask.getObservation()
  print "goal  = ",goal
  print "reward: ", thetask.getReward()
  print "\n"

  print "the maze"
  print thetask.env.__str__()

# test code below *******************************************************************
if DEBUG:

  # Simple pole-balancing task: we learn the weights of a neural network controller.

  thetask = BalanceTask()
  theparams = buildNetwork(thetask.outdim, thetask.indim, bias=False)

  agent = OptimizationAgent(theparams, CMAES())
  exp = EpisodicExperiment(thetask, agent)
  exp.doEpisodes(NUM_EPISODES)

  # test printouts for debugging
  print "initial weights: " + str(theparams.params)
  print "\n"
  print "getting observation 1"
  print thetask.getObservation()
  print "reward: " + str(thetask.getReward())
  print "\n"
  print "performing action 1"
  thetask.performAction([.2])
  print "reward: " + str(thetask.getReward())
  print "\n"
  print "getting observation 2"
  print thetask.getObservation()
  print "reward: " + str(thetask.getReward())
  print "\n"





