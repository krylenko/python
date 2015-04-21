# (c) daniel ford, daniel.jb.ford@gmail.com

# maze solver task for infomax robot controller learning node

__author__ = 'Daniel Ford, dford@email.arizona.edu'

from pybrain.rl.environments import EpisodicTask
from scipy import array, clip
import math

class MDPMazeTaskEpisodic(EpisodicTask):
	""" This is a MDP task for the MazeEnvironment. The state is fully observable,
	giving the agent the current position of perseus. Reward is given on reaching
	the goal, otherwise no reward. """

	def __init__(self,environment, maxSteps, goalTolerance):
		EpisodicTask.__init__(self,environment)
		self.env = environment
		self.count = 0
		self.atGoal = False
		self.MAX_STEPS = maxSteps
		self.GOAL_TOLERANCE = goalTolerance
		self.oldDist = 0
		self.reward = 0

	def reset(self):
		EpisodicTask.reset(self)
		self.count = 0
		self.oldDist = 0.
		self.atGoal = False
		self.reward = 0		

	def performAction(self, action):
	
		self.oldDist = self.__getDistance()		# copy current distance for shaping reward
		EpisodicTask.performAction(self, action)

		self.count += 1

	def isFinished(self):
		# continue task until goal is reached, we hit the edge, or we've moved MAX_STEPS times
		if self.atGoal or self.count >= self.MAX_STEPS or self.env.outbounds == True:	
			return True
		else:
			return False

	def __getDistance(self):
		# return Euclidean distance between agent and goal
		distance = math.hypot( (self.env.goal[0]-self.env.perseus[0]), (self.env.goal[1]-self.env.perseus[1]) )
		return distance

	def __getShaping(self):
		# calculate shaping reward based on distance delta between steps

		newDist = self.__getDistance()
		delta = newDist - self.oldDist 
		self.oldDist = newDist

		if delta < 0: return delta
		else: return 0

		#return delta

	def getReward(self):
		# compute and return the current reward (i.e. corresponding to the last action performed)
				
		reward = 0	
		scaling = 0	# scaling factor for shaping reward
					
		# goal is "reached" if the agent is within a certain distance of the goal
		if self.__getDistance() <= self.GOAL_TOLERANCE:
			reward = -500.	
			self.atGoal = True

		# make reward worse if agent hits map bounds
		elif self.env.outbounds == True:	
			reward = 6.21			
			
		# penalize each step around the map a little
		else: 
			reward = 1.1

		# update reward with shaping factor
		reward += (self.__getShaping() * scaling)

		return reward

	@property
	def indim(self):
		return self.env.indim

	@property
	def outdim(self):
		return self.env.outdim   


