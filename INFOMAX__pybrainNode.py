# (c) daniel ford, daniel.jb.ford@gmail.com

# learning node for infomax robot controller

__author__ = 'Daniel Ford, dford@email.arizona.edu'

# import utility functions
import datetime, pickle, sys
from scipy import *
from scipy import array
from scipy.linalg import norm, pinv, inv
from numpy import *
from numpy.random import randn

# import optimization frameworks and agent class
from pybrain.optimization import *
from pybrain.rl.agents import OptimizationAgent
from pybrain.rl.experiments import EpisodicExperiment

# import neural net stuff
from pybrain.structure.networks.network import Network
from pybrain.tools.shortcuts import buildNetwork
from pybrain.structure.modules import SoftmaxLayer
from pybrain.structure import FeedForwardNetwork, LinearLayer, FullConnection
#from RBFlayer import RBFLayer

# import tasks and environments
from mazeTask import MDPMazeTaskEpisodic
from mazeEnv import mazeEnv

class PybrainNode():

	def __init__(self, option):

		# examine command-line argument for learning algorithm
		if option[1] == "PGPE":
			self._PGPE = True; self._CMAES = False	
		elif option[1] == "CMAES":
			self._PGPE = False; self._CMAES = True
		else: self._PGPE = True

		# init RBF variables
		self.numCenters = 10
		self.beta = 8.
		self.centers = []

	def _createRBF(self):

        	# choose random centers on map
		for i in range(self.numCenters):        	
			self.centers.append(self.env._randomInitPose())

		# create an RBF network
		params = FeedForwardNetwork()
		
		inLayer = LinearLayer(self.task.outdim)
		hiddenLayer = RBFLayer(self.numCenters, self.centers)
		#inLayer = RBFLayer(self.numCenters, self.centers)
		outLayer = LinearLayer(self.task.indim)

		params.addInputModule(inLayer)
		params.addModule(hiddenLayer)
		params.addOutputModule(outLayer)

		in_to_hidden = FullConnection(inLayer,hiddenLayer)
		hidden_to_out = FullConnection(hiddenLayer,outLayer)
		params.addConnection(in_to_hidden)
		params.addConnection(hidden_to_out)

		params.sortModules()

		return params

	def run(self, size, goal, initPose, mapSelect, envSelect, maxSteps, goalTol, randomizeInitPose):

		choose = 0

		self.env = mazeEnv(size, goal, initPose, mapSelect, envSelect, randomizeInitPose)
		self.task = MDPMazeTaskEpisodic(self.env, maxSteps, goalTol)
		self.task.reset()

		# load network if we're just running, not training
		self.params = pickle.load(open('policyNet.pkl'))
		self.params.sorted = False
		self.params.sortModules()

		print "\ngoal = ",self.task.env.goal
		print "initial pose = ",self.task.env.initPose
		print "\n"

		while not self.task.isFinished():
	
			# get initial observation of agent's position
			p = self.task.getObservation()
			print "pose_pre : ",p

			# interpret and print new direction based on network output (max output wins)
			out = self.params.activate(p)

			temp = min(out)
			for num in range(len(out)):	
				if out[num] > temp:
					temp = out[num]
					choose = num
			if choose == 0: print "north"
			if choose == 1: print "east"
			if choose == 2: print "south"
			if choose == 3: print "west"

			# send network output to task to perform action
			self.task.performAction(out) 

			print self.task.getReward()
			p = self.task.getObservation()
			print "pose_post: ",p
			print "\n"

		print "total reward =",self.task.getTotalReward()
		print "\n" 

	def train(self, size, goal, initPose, mapSelect, envSelect, episodes, maxSteps, goalTol, randomizeInitPose):
 	
		avgReward = 0

		# set up environment and task
		self.env = mazeEnv(size, goal, initPose, mapSelect, envSelect, randomizeInitPose)
		self.task = MDPMazeTaskEpisodic(self.env, maxSteps, goalTol)

		# create neural net and learning agent
		self.params = buildNetwork(self.task.outdim, 48, self.task.indim, \
		bias=True, outclass=SoftmaxLayer)

		if self._PGPE:
			self.agent = OptimizationAgent(self.params, PGPE(minimize=True,verbose=False))
		elif self._CMAES:
			self.agent = OptimizationAgent(self.params, CMAES(minimize=True,verbose=False))

		# init experiment
		exp = EpisodicExperiment(self.task, self.agent)

		for i in range(0, episodes):        
			exp.doEpisodes()
			avgReward += self.task.getTotalReward()
			print "reward episode ",i,self.task.getTotalReward()

		# print initial info
		print "\naverage reward over training = ",avgReward/episodes

		# import weights into network and save network
		if self._PGPE:
			for i in range(len(self.params.params)):
				self.params.params[i] = self.agent.learner.current[i]
			pickle.dump(self.params, open('policyNet.pkl','w'))

		elif self._CMAES:

			################ following code came from WWInfoMaxCMAES.py script from ICDL 2010 paper
			arz = randn(self.agent.learner.numParameters, self.agent.learner.batchSize)
			arx = tile(self.agent.learner.center.reshape(self.agent.learner.numParameters, 1),\
			(1, self.agent.learner.batchSize)) + \
			self.agent.learner.stepSize * dot(dot(self.agent.learner.B, self.agent.learner.D), arz)
			# Go through the parameters and pick the current best 
			arfitness = zeros(self.agent.learner.batchSize)
			for k in xrange(self.agent.learner.batchSize):
			  self.agent.learner.wrappingEvaluable._setParameters(arx[:, k]);
			  arfitness[k] = self.agent.learner._BlackBoxOptimizer__evaluator\
			(self.agent.learner.wrappingEvaluable)

			# Sort by fitness and compute weighted mean into center
			tmp = sorted(map(lambda (x, y): (y, x), enumerate(ravel(arfitness))))
			arfitness = array(map(lambda x: x[0], tmp))
			arindex = array(map(lambda x: int(x[1]), tmp))

			arz = arz[:, arindex]
			curparams = arx[:, arindex[0]];

			# update network weights with selected parameters
			for i in range(len(self.params.params)):
				self.params.params[i] = curparams[i]
			# save trained network
			pickle.dump(self.params, open('policyNet.pkl','w'))
