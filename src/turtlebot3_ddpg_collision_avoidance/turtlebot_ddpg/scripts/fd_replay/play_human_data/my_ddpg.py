#! /usr/bin/env python

# this network can change both heading and velocity

import numpy as np
from keras.models import Sequential, Model
from keras.layers import Dense, Dropout, Input, merge
from keras.layers.merge import Add, Concatenate
from keras.optimizers import Adam
import keras.backend as K
import tensorflow
import random
from collections import deque
import os.path
import timeit
import csv
import math
import time
import matplotlib.pyplot as plt
import scipy.io as sio
from priortized_replay_buffer import PrioritizedReplayBuffer


import rospy
import rospkg
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion

import threading
import time

from keras.models import Sequential, Model
from keras.layers import Dense, Dropout, Input, merge
from keras.layers.merge import Add, Concatenate
from keras.optimizers import Adam
import gym

from std_srvs.srv import Empty


class InfoGetter(object):
    def __init__(self):
        #event that will block until the info is received
        self._event = threading.Event()
        #attribute for storing the rx'd message
        self._msg = None

    def __call__(self, msg):
        #Uses __call__ so the object itself acts as the callback
        #save the data, trigger the event
        self._msg = msg
        self._event.set()

    def get_msg(self, timeout=None):
        """Blocks until the data is rx'd with optional timeout
        Returns the received message
        """
        self._event.wait(timeout)
        return self._msg

def batch_stack_samples(samples):
	array = np.array(samples)
	#before_current_states = np.stack(array[:,0])
	current_states = np.stack(array[:,0]).reshape((array.shape[0],-1))
	actions = np.stack(array[:,1]).reshape((array.shape[0],-1))
	rewards = np.stack(array[:,2]).reshape((array.shape[0],-1))
	new_states = np.stack(array[:,3]).reshape((array.shape[0],-1))
	dones = np.stack(array[:,4]).reshape((array.shape[0],-1))
	weights = np.stack(array[:,5]).reshape((array.shape[0],-1))
	indices = np.stack(array[:,6]).reshape((array.shape[0],-1))
	eps_d = np.stack(array[:,7]).reshape((array.shape[0],-1))



	return current_states, actions, rewards, new_states, dones, weights, indices, eps_d

class GameState:

    def __init__(self):
        self.talker_node = rospy.init_node('talker', anonymous=True)
        self.pose_ig = InfoGetter()
        self.laser_ig = InfoGetter()
        self.collision_ig = InfoGetter()
        

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.position = Point()
        self.move_cmd = Twist()

        self.laser_info = rospy.Subscriber("/scan", LaserScan, self.laser_ig)
        # self.bumper_info = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.processBump)

        rospy.on_shutdown(self.shutdown)


        # tf
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")
        (self.position, self.rotation) = self.get_odom()
        self.init_position_x = self.position.x
        self.init_position_y = self.position.y
  
        self.rate = rospy.Rate(100) # 100hz

        # Create a Twist message and add linear x and angular z values
        self.move_cmd = Twist()
        self.move_cmd.linear.x = 0.6 #linear_x
        self.move_cmd.angular.z = 0.2 #angular_z

        # crush default value
        self.crash_indicator = 0

        # observation_space and action_space
        self.state_num = 28 #685                 # when you change this value, remember to change the reset default function as well
        self.action_num = 2
        self.observation_space = np.empty(self.state_num)
        self.action_space = np.empty(self.action_num)
        # self.state_input1_space =  np.empty(1)
        # self.state_input2_space =  np.empty(1)

        self.laser_reward = 0
        # set target position
        self.target_x = 2 + self.init_position_x
        self.target_y = -2 + self.init_position_y

        # set turtlebot index in gazebo world
        self.model_index = 10 #25

        ##### need modification
        # self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)


    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])


    def shutdown(self):
        self.pub.publish(Twist())
        rospy.exceptions.ROSInterruptException("ROS shutdown request")
        rospy.sleep(1)


    def print_odom(self):
        (self.position, self.rotation) = self.get_odom()
        print("position is %s, %s, %s, ", self.position.x, self.position.y, self.position.z)
        # print("rotation is %s, ", self.rotation)


    def reset(self):
        index_list = [-1, 1]
        index_x = random.choice(index_list)
        index_y = random.choice(index_list)
        index_turtlebot_y = random.choice(index_list)
        self.target_x = 2 + self.init_position_x
        self.target_y = -2 + self.init_position_y
        random_turtlebot_y = (np.random.random())*5 #+ index_turtlebot_y


        self.crash_indicator = 0

        self.init_pose_x = self.get_odom()



        initial_state = np.ones(self.state_num)
        #initial_state[self.state_num-2] = 0
        initial_state[self.state_num-1] = 0
        initial_state[self.state_num-2] = 0
        initial_state[self.state_num-3] = 0
        initial_state[self.state_num-4] = 0

        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.pub.publish(self.move_cmd)
        time.sleep(1)
        self.pub.publish(self.move_cmd)
        self.rate.sleep()


        return initial_state


    def turtlebot_is_crashed(self, laser_values, range_limit):
        self.laser_crashed_value = 0
        self.laser_crashed_reward = 0

        # print(laser_values)
        for i in range(len(laser_values)):
            if (laser_values[i] < 2*range_limit):
                self.laser_crashed_reward = -80
            if (laser_values[i] < range_limit):
                self.laser_crashed_value = 1
                self.laser_crashed_reward = -200
                print("turtlebot crashed")
                self.shutdown()
                self.reset()
                time.sleep(1)
                break
        return self.laser_crashed_reward


    def game_step(self, time_step=0.1, linear_x=0.8, angular_z=0.3):
        start_time = time.time()
        record_time = start_time
        record_time_step = 0
        self.move_cmd.linear.x = linear_x*0.26
        self.move_cmd.angular.z = angular_z
        self.rate.sleep()


        (self.position, self.rotation) = self.get_odom()
        # print("position x: {}, y: {}".format(self.position.x, self.position.y))
        turtlebot_x_previous = self.position.x
        turtlebot_y_previous = self.position.y


        while (record_time_step < time_step) and (self.crash_indicator==0):
            self.pub.publish(self.move_cmd)
            self.rate.sleep()
            record_time = time.time()
            record_time_step = record_time - start_time
        (self.position, self.rotation) = self.get_odom()
        turtlebot_x = self.position.x
        turtlebot_y = self.position.y
        angle_turtlebot = self.rotation
        # make input, angle between the turtlebot and the target
        angle_turtlebot_target = atan2(self.target_y - turtlebot_y, self.target_x- turtlebot_x)
        
        if angle_turtlebot < 0:
            angle_turtlebot = angle_turtlebot + 2*math.pi

        if angle_turtlebot_target < 0:
            angle_turtlebot_target = angle_turtlebot_target + 2*math.pi


        angle_diff = angle_turtlebot_target - angle_turtlebot
        if angle_diff < -math.pi:
            angle_diff = angle_diff + 2*math.pi
        if angle_diff > math.pi:
            angle_diff = angle_diff - 2*math.pi


        # prepare the normalized laser value and check if it is crash
        laser_msg = self.laser_ig.get_msg()
        laser_values = laser_msg.ranges
        #print('turtlebot laser_msg.ranges is %s', laser_msg.ranges)
        #print('turtlebot laser data is %s', laser_values)
        normalized_laser = [(x)/3.5 for x in (laser_msg.ranges)]
        normalized_laser = [1.0 if (normalized_laser[int(math.floor(-90.0 + float(x) *(180.0/23.0)))]) == 0 else (normalized_laser[int(math.floor(-90.0 + float(x) *(180.0/23.0)))]) for x in range(24)]


        # prepare state
        #state = np.append(normalized_laser, angle_diff)
        #state = np.append(normalized_laser,self.target_x- turtlebot_x)
        current_distance_turtlebot_target = math.sqrt((self.target_x - turtlebot_x)**2 + (self.target_y - turtlebot_y)**2)

        state = np.append(normalized_laser, current_distance_turtlebot_target)
        state = np.append(state, angle_diff)
        state = np.append(state, linear_x*0.26)
        state = np.append(state, angular_z)
        # print("angle_turtlebot and angle_diff are %s %s", angle_turtlebot*180/math.pi, angle_diff*180/math.pi)
        # print("position x is %s position y is %s", turtlebot_x, turtlebot_y)
        # print("command angular is %s", angular_z*1.82)
        # print("command linear is %s", linear_x*0.26)
        #print("state is %s", state)

        state = state.reshape(1, self.state_num)


        # make distance reward
        (self.position, self.rotation) = self.get_odom()
        turtlebot_x = self.position.x
        turtlebot_y = self.position.y
        distance_turtlebot_target_previous = math.sqrt((self.target_x - turtlebot_x_previous)**2 + (self.target_y - turtlebot_y_previous)**2)
        distance_turtlebot_target = math.sqrt((self.target_x - turtlebot_x)**2 + (self.target_y - turtlebot_y)**2)
        distance_reward = distance_turtlebot_target_previous - distance_turtlebot_target
        self.laser_crashed_reward = self.turtlebot_is_crashed(normalized_laser, range_limit=0.06)
        self.laser_reward = sum(normalized_laser)-24
        self.collision_reward = self.laser_crashed_reward + self.laser_reward


        self.angular_punish_reward = 0
        self.linear_punish_reward = 0

        if angular_z > 0.8:
            self.angular_punish_reward = -1
        if angular_z < -0.8:
            self.angular_punish_reward = -1

        if linear_x < 0.2:
            self.linear_punish_reward = -2


        self.arrive_reward = 0
        if distance_turtlebot_target<1:
            self.arrive_reward = 100
            print("Turtlebot arrival")
            self.shutdown()
            # self.reset()
            time.sleep(1)


 

        reward  = distance_reward*(5/time_step)*1.2*7 + self.arrive_reward + self.collision_reward + self.angular_punish_reward + self.linear_punish_reward
        # print("laser_reward is %s", self.laser_reward)
        # print("laser_crashed_reward is %s", self.laser_crashed_reward)
        # print("arrive_reward is %s", self.arrive_reward)
        # print("distance reward is : %s", distance_reward*(5/time_step)*1.2*7)


        return reward, state, self.laser_crashed_value


# determines how to assign values to each state, i.e. takes the state
# and action (two-input model) and determines the corresponding value
class ActorCritic:
	def __init__(self, env, sess):
		self.env  = env
		self.sess = sess

		self.learning_rate = 0.0001
		self.epsilon = .9
		self.epsilon_decay = .99995
		self.gamma = .90
		self.tau   = .01


		self.buffer_size = 1000000
		self.batch_size = 512

		self.hyper_parameters_lambda3 = 0.2
		self.hyper_parameters_eps = 0.2
		self.hyper_parameters_eps_d = 0.4

		self.demo_size = 1000

		# ===================================================================== #
		#                               Actor Model                             #
		# Chain rule: find the gradient of chaging the actor network params in  #
		# getting closest to the final value network predictions, i.e. de/dA    #
		# Calculate de/dA as = de/dC * dC/dA, where e is error, C critic, A act #
		# ===================================================================== #

		self.memory = PrioritizedReplayBuffer() #deque(maxlen=40000)
		self.actor_state_input, self.actor_model = self.create_actor_model()
		_, self.target_actor_model = self.create_actor_model()

		self.actor_critic_grad = tensorflow.placeholder(tensorflow.float32,
			[None, self.env.action_space.shape[0]]) # where we will feed de/dC (from critic)

		actor_model_weights = self.actor_model.trainable_weights
		self.actor_grads = tensorflow.gradients(self.actor_model.output,
			actor_model_weights, -self.actor_critic_grad) # dC/dA (from actor)
		grads = zip(self.actor_grads, actor_model_weights)
		self.optimize = tensorflow.train.AdamOptimizer(self.learning_rate).apply_gradients(grads)

		# ===================================================================== #
		#                              Critic Model                             #
		# ===================================================================== #

		self.critic_state_input, self.critic_action_input, \
			self.critic_model = self.create_critic_model()
		_, _, self.target_critic_model = self.create_critic_model()

		self.critic_grads = tensorflow.gradients(self.critic_model.output,
			self.critic_action_input) # where we calcaulte de/dC for feeding above

		# Initialize for later gradient calculations
		self.sess.run(tensorflow.initialize_all_variables())

	# ========================================================================= #
	#                              Model Definitions                            #
	# ========================================================================= #

	def create_actor_model(self):
		state_input = Input(shape=self.env.observation_space.shape)
		h1 = Dense(500, activation='relu')(state_input)
		#h2 = Dense(1000, activation='relu')(h1)
		h2 = Dense(500, activation='relu')(h1)
		h3 = Dense(500, activation='relu')(h2)
		delta_theta = Dense(1, activation='tanh')(h3) 
		speed = Dense(1, activation='sigmoid')(h3) # sigmoid makes the output to be range [0, 1]

		#output = Dense(self.env.action_space.shape[0], activation='tanh')(h3)
		#output = Concatenate()([delta_theta])#merge([delta_theta, speed],mode='concat')
		output = Concatenate()([delta_theta, speed])
		model = Model(input=state_input, output=output)
		adam  = Adam(lr=0.0001)
		model.compile(loss="mse", optimizer=adam)
		return state_input, model

	def create_critic_model(self):
		state_input = Input(shape=self.env.observation_space.shape)
		state_h1 = Dense(500, activation='relu')(state_input)
		#state_h2 = Dense(1000)(state_h1)

		action_input = Input(shape=self.env.action_space.shape)
		action_h1    = Dense(500)(action_input)

		merged    = Concatenate()([state_h1, action_h1])
		merged_h1 = Dense(500, activation='relu')(merged)
		merged_h2 = Dense(500, activation='relu')(merged_h1)
		output = Dense(1, activation='linear')(merged_h2)
		model  = Model(input=[state_input,action_input], output=output)

		adam  = Adam(lr=0.0001)
		model.compile(loss="mse", optimizer=adam)
		return state_input, action_input, model

	# ========================================================================= #
	#                               Model Training                              #
	# ========================================================================= #

	def remember(self, cur_state, action, reward, new_state, done):
		
		indice = len(self.memory.memory_data())

		target_actions = self.target_actor_model.predict(new_state)
		future_rewards = self.target_critic_model.predict([new_state, target_actions])
		rewards = reward + self.gamma* future_rewards * (1 - done)

		# get critic_loss_element_wise and actor_loss_element
		critic_values = self.critic_model.predict([cur_state, action])
		critic_loss_element = np.power((critic_values-rewards), 2)
		predicted_action = self.actor_model.predict(cur_state)
		actor_loss_element = self.critic_model.predict([cur_state, predicted_action])

		new_priorities  = critic_loss_element
		new_priorities += self.hyper_parameters_lambda3 * np.power(actor_loss_element,2)
		new_priorities += self.hyper_parameters_eps
		# new_priorities += self.hyper_parameters_eps_d

		self.memory.add(cur_state, action, reward, new_state, done, indice, new_priorities)  # add to buffer, instead of sampling batch



	def read_human_data(self):
		mat_contents = sio.loadmat('my_human_data.mat')
		a = mat_contents['data']
		
		for i in range(self.demo_size):
			cur_state = a[i][0:28]
			action = a[i][28:30]
			reward = a[i][30]
			new_state = a[i][31:59]
			done = a[i][59]
			cur_state = cur_state.reshape(1,28)
			action = action.reshape(1,2)
			#array_reward = np.array(reward)
			#reward = self.array_reward.reshape(1,1)
			new_state = new_state.reshape(1,28)
			indice = i
			new_priorities = 1
			action[0][1] = action[0][1]/0.26
			# print("angular velocity recorded is %s", action[0][0])
			# print("linear velocity recorded is %s", action[0][1])
			self.memory.add(cur_state, action, reward, new_state, done, indice, new_priorities)


	def _train_critic_actor(self, samples):
 
   		# 1, sample to get states, actions, rewards, new_states, dones
   		# 2, calculate weights, indices, eps_d
   		# 3, get critic_loss_element_wise
   		# 4, train critic based on weights
   		# 5, train actor based on weights
   		# 6, update target network?
   		# 7, update priorities for sampling


   		# 1, sample
		cur_states, actions, rewards, new_states, dones, weights, indices, eps_d = samples #batch_stack_samples(samples)
		target_actions = self.target_actor_model.predict(new_states)
		future_rewards = self.target_critic_model.predict([new_states, target_actions])
		rewards = rewards + self.gamma* future_rewards * (1 - dones)


		# 4, train critic based on weights
		_sample_weight = weights #(rewards/rewards).flatten()
		# print("_sample_weight is %s", _sample_weight)
		evaluation = self.critic_model.fit([cur_states, actions], rewards, verbose=0, sample_weight=_sample_weight)
		# print('\nhistory dict:', evaluation.history)


		# 5, train actor based on weights
		predicted_actions = self.actor_model.predict(cur_states)
		grads = self.sess.run(self.critic_grads, feed_dict={
			self.critic_state_input:  cur_states,
			self.critic_action_input: predicted_actions
		})[0]



		#calculate grads_weight for changing the actor model weight?
		grads_weight = grads
		for i in range(0, len(grads)):
			grads_weight[i][0] = grads[i][0]*_sample_weight[i]
			grads_weight[i][1] = grads[i][1]*_sample_weight[i]
		grads = grads_weight
		self.sess.run(self.optimize, feed_dict={
			self.actor_state_input: cur_states,
			self.actor_critic_grad: grads
		})
		# print("grads*weights is %s", grads)
		


		# 3, get critic_loss_element_wise
		critic_values = self.critic_model.predict([cur_states, actions])
		critic_loss_element = np.power((critic_values-rewards), 2)

		# 7, update priorities for sampling
		actor_loss_element = self.critic_model.predict([cur_states, predicted_actions])
		# print("actor_loss_element is %s", actor_loss_element)

		new_priorities  = critic_loss_element
		new_priorities += self.hyper_parameters_lambda3 * np.power(actor_loss_element,2)
		new_priorities += self.hyper_parameters_eps
		new_priorities += self.hyper_parameters_eps_d
		# print("new_priorities is %s", new_priorities)


		######################################################################
		# update priority of sampled transitions, batch_size.
		self.memory.update_priorities(indices, new_priorities)



	def read_Q_values(self, cur_states, actions):
		critic_values = self.critic_model.predict([cur_states, actions])
		return critic_values

	def train(self):
		batch_size = self.batch_size
		if len(self.memory.memory_data()) < batch_size: #batch_size:
			return
		#samples = random.sample(self.memory.memory_data(), batch_size)    # what is deque, what is random.sample? self.mempory begins with self.memory.append
		samples = self.memory.sample(1, batch_size)
		self.samples = samples
		# print("samples is %s", samples)
		# print("samples [1] is %s", samples[1])
		# print("length of memory is %s", len(self.memory.memory_data()))
		# print("samples shape is %s", samples.shape)
		self._train_critic_actor(samples)


	# ========================================================================= #
	#                         Target Model Updating                             #
	# ========================================================================= #

	def _update_actor_target(self):
		actor_model_weights  = self.actor_model.get_weights()
		actor_target_weights = self.target_actor_model.get_weights()
		
		for i in range(len(actor_target_weights)):
			actor_target_weights[i] = actor_model_weights[i]*self.tau + actor_target_weights[i]*(1-self.tau)
		self.target_actor_model.set_weights(actor_target_weights)

	def _update_critic_target(self):
		critic_model_weights  = self.critic_model.get_weights()
		critic_target_weights = self.target_critic_model.get_weights()
		
		for i in range(len(critic_target_weights)):
			critic_target_weights[i] = critic_model_weights[i]*self.tau + critic_target_weights[i]*(1-self.tau)
		self.target_critic_model.set_weights(critic_target_weights)

	def update_target(self):
		self._update_actor_target()
		self._update_critic_target()

	# ========================================================================= #
	#                              Model Predictions                            #
	# ========================================================================= #

	def act(self, cur_state):  # this function returns action, which is predicted by the model. parameter is epsilon
		#self.epsilon *= self.epsilon_decay
		self.epsilon = 0.9
		eps = self.epsilon
		action = self.actor_model.predict(cur_state)
		if np.random.random() < self.epsilon:
			action[0][0] = action[0][0] + (np.random.random()-0.5)*0.4
			action[0][1] = action[0][1] + np.random.random()*0.4
			return action, eps	
		else:
			action[0][0] = (np.random.random()-0.5)*2   # angular velocity
			action[0][1] = np.random.random()   # linear velocity
			return action, eps
		

	# ========================================================================= #
	#                              save weights                            #
	# ========================================================================= #

	def save_weight(self, num_trials, trial_len):
		self.actor_model.save_weights('actormodel' + '-' +  str(num_trials) + '-' + str(trial_len) + '.h5', overwrite=True)
		self.critic_model.save_weights('criticmodel' + '-' + str(num_trials) + '-' + str(trial_len) + '.h5', overwrite=True)#("criticmodel.h5", overwrite=True)

	def play(self, cur_state):
		return self.actor_model.predict(cur_state)


def main():
	
    sess = tensorflow.Session()
    K.set_session(sess)

    ########################################################
    game_state= GameState()   # game_state has frame_step(action) function
    actor_critic = ActorCritic(game_state, sess)
    ########################################################
    num_trials = 10000
    trial_len  = 500
    train_indicator = 0

    current_state = game_state.reset()

    # actor_critic.read_human_data()

    step_reward = [0,0]
    step_Q = [0,0]
    step = 0
    
    for i in range(num_trials):
        print("trial:" + str(i))
        current_state = game_state.reset()
        
        print("=====================================================")
        print("init position x: {}, y: {}".format(game_state.init_position_x, game_state.init_position_y))
        print("target position x: {}, y: {}".format(game_state.target_x, game_state.target_y))
        print("=====================================================")
        

        actor_critic.actor_model.load_weights("/home/seonghun/RL_project/src/turtlebot3_ddpg_collision_avoidance/turtlebot_ddpg/scripts/fd_replay/play_human_data/saved_no_obstacles/actormodel-50-500.h5")
        actor_critic.critic_model.load_weights("/home/seonghun/RL_project/src/turtlebot3_ddpg_collision_avoidance/turtlebot_ddpg/scripts/fd_replay/play_human_data/saved_no_obstacles/criticmodel-50-500.h5")
        ##############################################################################################
        total_reward = 0
        
        for j in range(trial_len):
            ###########################################################################################
            current_state = current_state.reshape((1, game_state.observation_space.shape[0]))

            start_time = time.time()
            action = actor_critic.play(current_state)  # need to change the network input output, do I need to change the output to be [0, 2*pi]
            action = action.reshape((1, game_state.action_space.shape[0]))
            end_time = time.time()
            # print(1/(end_time - start_time), "fps for calculating next step")
            
            reward, new_state, crashed_value = game_state.game_step(0.1, action[0][1], action[0][0]) # we get reward and state here, then we need to calculate if it is crashed! for 'dones' value
            if j %10 == 9:
                game_state.print_odom()
            total_reward = total_reward + reward
            ###########################################################################################

            if j == (trial_len - 1):
                crashed_value = 1
                print("this is reward:", total_reward)
                

            # if (j % 5 == 0):
            # 	actor_critic.train()
            # 	actor_critic.update_target()   
            
            new_state = new_state.reshape((1, game_state.observation_space.shape[0]))
            # actor_critic.remember(cur_state, action, reward, new_state, done)   # remember all the data using memory, memory data will be samples to samples automatically.
            # cur_state = new_state

            ##########################################################################################
            #actor_critic.remember(current_state, action, reward, new_state, crashed_value)
            current_state = new_state

            ##########################################################################################[]



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("somthing went wrong")
        pass
