#!/usr/bin/env python3
# coding: utf-8

# Author : vledoze
# Date   : 25/05/2018

### IMPORTS =======================================================
# Import standard lib
import math
import numpy
import random
import rospy
import sys
import time

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

import matplotlib
import matplotlib.pyplot as plt

from collections import namedtuple
from torch.autograd import Variable

# Import local lib

# ROS messages
from std_msgs.msg import Empty as Msg_Empty
from std_msgs.msg import Float32 as Msg_Float32
from sensor_msgs.msg import Imu as Msg_Imu
from chroma_gazebo_plugins_msgs.msg import Rotorspeed as Msg_Rotorspeed
from chroma_gazebo_plugins_msgs.msg import BodyGroundTruth as Msg_BodyGroundTruth

# ROS services
from std_srvs.srv import Empty as Srv_Empty

### CONSTANTS =========================================================
# Constant on the action choice
EPS_START = 1.0
EPS_END = 0.0
EPS_DECAY = 200

# Learning constant
GAMMA = 0.999

# Batch size
BATCH_SIZE = 100

# One episode time limit
EPISODE_STEPS_LIMIT = 100

# Memory size
REPLAY_MEMORY = 1000000

# Memory tuple
TRANSITION = namedtuple(
    'Transition',
    ('state', 'action', 'next_state', 'reward'))

# Function reward type
# Possibilities : "position" / "gradient (default)"
REWARD_FUNC = "gradient"

# Default command
DOWN = 50.0
UP = 85.0
UP_CMD = [UP, UP, UP, UP]
DOWN_CMD = [DOWN, DOWN, DOWN, DOWN]

### CLASSES ============================================================
class ReplayMemory(object):
    """
    Class that allows saving, accessing and replaying memory
    composed of episodes "state, action, next_state, reward"
    """

    def __init__(self, capacity):
        """
        Initialization
        """
        self.capacity = capacity #Nombre max d'épisode a memoriser
        self.memory = []
        self.position = 0

    def push(self, *args):
        """
        Save an episode. If the maximum memory size is
        reached, the oldest episodes are overwritten.
        """
        if len(self.memory) < self.capacity:
            self.memory.append(None)
        self.memory[self.position] = TRANSITION(*args)
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size):
        """
        Returns an episode from memory
        """
        return random.sample(self.memory, batch_size)

    def __len__(self):
        """
        Number of memorized episodes
        """
        return len(self.memory)

class DQN(nn.Module):
    """ Q-Network """

    def __init__(self):
        """
        Inputs :
            altitude
            imu
            rotorspeed
        Outputs :
            Q(s, add_thrust)
            Q(s, rmv_thrust)
        """
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(3, 10)
        self.fc2 = nn.Linear(10, 50)
        self.fc3 = nn.Linear(50, 10)
        self.head = nn.Linear(10, 2)

    def forward(self, x):
        x = F.leaky_relu(self.fc1(x))
        x = F.leaky_relu(self.fc2(x))
        x = F.leaky_relu(self.fc3(x))
        return F.softmax(self.head(x), dim=1)

class DroneReinforcementLearning(object):
    """
    """

    # Initialization functions -----------------------------------------
    def __init__(self):
        """
        """
        # Learning attributes
        self.__init__learning_attributes()
        # Episodes attributes
        self.__init__episodes_attributes()
        # ROS services
        self.__init__services()
        # ROS subscribers
        self.__init__subscribers()
        # ROS publishers
        self.__init__publishers()

    def __init__learning_attributes(self):
        """
        Fixed attributes during learning
        """
        # Drone memory
        self.memory = ReplayMemory(
            rospy.get_param("~replay_memory", REPLAY_MEMORY))
        # Episodes counter
        self.episodes = 0
        # Storage for plotting
        self.cumu_reward_list = []
        self.mean_loss_list = []
        # Networks
        self.policy_net = DQN().cuda()
        self.target_net = DQN().cuda()
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.target_net.eval()
        # Optimizer
        self.optimizer = optim.RMSprop(
            self.policy_net.parameters(),
            lr=rospy.get_param("~learning_rate", 0.01))
        # Batch size
        self.batch_size = rospy.get_param("~batch_size", BATCH_SIZE)
        # Reward function choice
        self.reward_func = rospy.get_param("~reward_func", REWARD_FUNC)
        # Cost calculation method
        self.loss_method = "method_2"

    def __init__episodes_attributes(self):
        """
        Attributes that will be reset with each new episode
        """
        # Flag : drone OK
        self.drone_ok = True
        # Flag : learning OK
        self.learning_ok = True
        # Initial state
        self.altitude = 0.0126
        self.verticalspeed = 0.0
        self.rotorspeed = 0.0
        self.observations = torch.FloatTensor([[
            self.altitude,
            self.verticalspeed,
            self.rotorspeed]])
        # Number of time steps per episode
        self.episode_steps = 0
        # In process action
        self.action = torch.LongTensor([[0]])
        # Reward
        self.reward = torch.FloatTensor([[0.0]])
        # Cumulative reward
        self.cumu_reward = 0.0
        # Cost
        self.loss_list = []
        # Number of episodes
        self.episodes += 1
        self.has_no_trace = True

    def __init__publishers(self):
        """
        ROS publishers initialization
        """
        self.pub_rotorspeed = rospy.Publisher(
            "command/rotorspeed", Msg_Rotorspeed, queue_size=10)

    def __init__subscribers(self):
        """
        ROS subscribers initialization
        """
        rospy.Subscriber("imu", Msg_Imu, self.callback_imu)
        rospy.Subscriber(
            "groundtruth", Msg_BodyGroundTruth, self.callback_groundtruth)
        rospy.Subscriber(
            "rotorspeed/0", Msg_Float32, self.callback_rotorspeed_0)
        rospy.Subscriber('start_learning', Msg_Empty, self.callback_start_learning)
        rospy.Subscriber('stop_learning', Msg_Empty, self.callback_stop_learning)

    def __init__services(self):
        """
        ROS services initialization
        """

    # Public functions ----------------------------------------------
    def run(self):
        """
        """
        rate = rospy.Rate(rospy.get_param("~freq", 10.0)) # 10Hz command
        while not rospy.is_shutdown():
            rate.sleep()
            self.step()

    def step(self):
        """
        Learning step
        """
        if self.drone_ok & self.learning_ok:
            # Observation vector
            observations = torch.FloatTensor([[
                self.altitude,
                self.verticalspeed,
                self.rotorspeed]])
            # Reward function
            if (self.reward_func == "position"):
                self.reward_function_position(observations)
            else:
                self.reward_function_gradient(observations)
            # Action
            self.select_action(observations)
            # Motor message
            msg_rotorspeed = Msg_Rotorspeed()
            msg_rotorspeed.values = DOWN_CMD if (int(self.action)==0) \
                else UP_CMD #Rotation velocity of the 4 motors (rad/s)
            self.pub_rotorspeed.publish(msg_rotorspeed)
        if (self.episodes > 1) & (self.has_no_trace):
            #Mean loss and cumulative reward plot
            plt.clf()
            plt.title('Training...')
            ax = plt.subplot(111)
            ax.plot(self.cumu_reward_list, label="cumulative_reward", color="blue")
            ax.set_xlabel('Episode')
            ax.set_ylabel('reward')
            ax.legend(loc=0)
            ax1 = ax.twinx()
            ax1.set_ylabel('loss')
            ax1.plot(self.mean_loss_list, label="mean_loss", color="green")
            ax1.legend(loc=1)
            plt.pause(0.001)  # Pause a bit so that plots are updated
            #
            self.has_no_trace = False

    def select_action(self, observations):
        """
        Choose an action based on an observation
        Actions = Rotation speeds of the 4 motors (rad/s)
        """
        # Store the transition in memory
        self.memory.push(self.observations, self.action, observations, self.reward)
        # Perform one step of the optimization (on the target network)
        self.optimize_model()
        # Conditions to use the policy net
        trigger = random.random()
        threshold = EPS_END + (EPS_START - EPS_END) * \
            math.exp(-1. * self.episodes / EPS_DECAY)
        if trigger > threshold:
            self.action = self.policy_net(Variable(observations, volatile=True).cuda()).data.max(1)[1].view(1, 1).cpu()
        # Otherwise choose random actions
        else:
            self.action = torch.LongTensor([[random.randrange(2)]])
        # Memorized state
        self.observations = observations

    def reward_function_position(self, observations):
        """
        Calculate a reward based on the current state
        """
        if observations[0][0] < 0.1:
            reward = 0.0
        else:
            ecart = observations[0][0] - 1.0 # Constant altitude flight = 1.0m
            ecart = abs(ecart*10.0)
            # ecart = min(ecart, 1.0) # Limit of the gap 1.0m
            reward = 1.0/max(ecart, 1.0) # Important reward if low gap.
        self.reward = torch.FloatTensor([[reward]])
        self.cumu_reward += math.pow(GAMMA, self.episode_steps)*reward
        self.episode_steps += 1

    def reward_function_gradient(self, observations):
        """
        Calculate a reward based on the current state
        """
        dist_cur = abs(observations[0][0] - 1.0)
        dist_old = abs(self.observations[0][0] - 1.0)
        if dist_old > (dist_cur + 0.01):
            reward = 1.0
        else:
            reward = 0.0
        self.reward = torch.FloatTensor([[reward]])
        self.cumu_reward += math.pow(GAMMA, self.episode_steps)*reward
        self.episode_steps += 1

    def optimize_model(self):
        # Optimization only if enough memory
        if len(self.memory) < self.batch_size:
            return
        # Recovery of n-transitions
        transitions = self.memory.sample(self.batch_size)
        # Transpose the batch
        # (see http://stackoverflow.com/a/19343/3343043 for detailed explanation).
        batch = TRANSITION(*zip(*transitions))
        state_batch = Variable(torch.cat(batch.state)).cuda()
        action_batch = Variable(torch.cat(batch.action)).cuda()
        next_state_batch = Variable(torch.cat(batch.next_state)).cuda()
        reward_batch = Variable(torch.cat(batch.reward)).cuda()
        # Compute the Loss
        if (self.loss_method == "Bellman"):
            # Compute Q(s_t, a)
            # The model computes Q(s_t), then we select the columns of actions taken
            state_action_values = self.policy_net(state_batch).gather(1, action_batch.long())
            # Compute V(s_{t+1}) for all next states.
            next_state_value = self.target_net(next_state_batch).max(1)[0].view(self.batch_size, 1)
            # Compute the expected Q values
            expected_state_action_values = Variable(((next_state_value * GAMMA) + reward_batch).data)
            # Compute Huber loss
            loss = F.smooth_l1_loss(state_action_values, expected_state_action_values)# # Compute Q(s_t, a)
        elif (self.loss_method == "method_1"):
            # The model computes Q(s_t), then we select the columns of actions taken
            state_action_values = self.policy_net(state_batch).gather(1, action_batch.long())
            # Loss = Q(s,t) - R(s, a)
            loss = F.smooth_l1_loss(state_action_values, reward_batch)
        else:
            # A(s, t), vecteur des probabilités d'actions
            policy_output = self.policy_net(state_batch)
            # R_bar : inverse des reward (si r=1, r_bar=0)
            reward_bar = torch.abs(reward_batch - 1.0)
            # Expected output = 1 for the best action and 0 otherwise
            expected_output = torch.cat([reward_bar, reward_bar], dim=1)
            for i in range(self.batch_size):
                expected_output[i, action_batch.long().data[i][0]] = reward_batch[i, 0]
            # Loss = A(s, t) - R(s, a)
            loss = F.l1_loss(policy_output, expected_output)
        self.loss_list.append(float(loss.data))
        # Optimize the model
        self.optimizer.zero_grad()
        loss.backward()
        for param in self.policy_net.parameters():
            param.grad.data.clamp_(-1, 1)
        self.optimizer.step()


    # Callback functions -----------------------------------------------
    def callback_rotorspeed_0(self, rotor_data):
        """
        Rotors speed callback
        """
        self.rotorspeed = rotor_data.data

    def callback_imu(self, imu_data):
        """
        IMU message callback
        Message structure : http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
        """
        if ((imu_data.linear_acceleration.z < 0.0) | \
           (imu_data.linear_acceleration.z > 1000.0)) & \
           (self.drone_ok == True):
           # Drone crash possibility : stop learning
           self.drone_ok = False

    def callback_groundtruth(self, gt_data):
        """
        Groundtruth callback
        Message structure : chroma_gazebo_plugins_msgs/msg/BodyGroundTruth
        Function called at each time step of the Gazebo simulation
        """
        self.altitude = gt_data.z_enu
        self.verticalspeed = gt_data.vz_enu

    # Private funtions -------------------------------------------------
    def callback_stop_learning(self, req):
        """
        Function called by the stop_learning service
        """
        # Learning on stand-by
        self.learning_ok = False
        # Null commands to stop the rotors
        msg_rotorspeed = Msg_Rotorspeed()
        msg_rotorspeed.values = [0.0, 0.0, 0.0, 0.0]
        self.pub_rotorspeed.publish(msg_rotorspeed)
        # Mean reward
        self.cumu_reward_list.append(self.cumu_reward)
        # Mean loss
        if len(self.loss_list) > 0:
            self.mean_loss_list.append(numpy.mean(self.loss_list))
        else:
            self.mean_loss_list.append(1.0)

    def callback_start_learning(self, req):
        """
        Function called by the start_learning service
        """
        # Attributes reset
        self.__init__episodes_attributes()
        self.target_net.load_state_dict(self.policy_net.state_dict())
        # Learning restart
        self.learning_ok = True
        return Srv_Empty()



### MAIN ===============================================================
if __name__=="__main__":
    rospy.init_node("node", anonymous=False)
    node = DroneReinforcementLearning()
    plt.figure()
    node.run()
