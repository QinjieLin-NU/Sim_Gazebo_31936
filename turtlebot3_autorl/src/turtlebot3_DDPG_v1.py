#!/usr/bin/env python
"""
Note: This is a updated version from my previous code,
for the target network, I use moving average to soft replace target parameters instead using assign function.
By doing this, it has 20% speed up on my machine (CPU).

Deep Deterministic Policy Gradient (DDPG), Reinforcement Learning.
DDPG is Actor Critic based algorithm.
Pendulum example.

View more on my tutorial page: https://morvanzhou.github.io/tutorials/

Using:
tensorflow 1.0
gym 0.8.0
"""

import tensorflow as tf
import numpy as np
import gym
import time
import os
from environment_PPO_v2 import Env
import rospy

from util_io import saveModelInfo,writeData

###############################  DDPG  ####################################


class DDPG(object):
    def __init__(self, a_dim, s_dim, a_bound,LR_A,LR_C,GAMMA,TAU,MEMORY_CAPACITY,BATCH_SIZE,gpu_options):
        self.LR_A = LR_A
        self.LR_C = LR_C
        self.GAMMA = GAMMA
        self.TAU = TAU
        self.MEMORY_CAPACITY = MEMORY_CAPACITY
        self.BATCH_SIZE = BATCH_SIZE

        self.memory = np.zeros((self.MEMORY_CAPACITY, s_dim * 2 + a_dim + 1), dtype=np.float32)
        self.pointer = 0
        self.sess = tf.Session(config=tf.ConfigProto(gpu_options=gpu_options))

        self.a_dim, self.s_dim, self.a_bound = a_dim, s_dim, a_bound,
        self.S = tf.placeholder(tf.float32, [None, s_dim], 's')
        self.S_ = tf.placeholder(tf.float32, [None, s_dim], 's_')
        self.R = tf.placeholder(tf.float32, [None, 1], 'r')

        self.a = self._build_a(self.S,)
        q = self._build_c(self.S, self.a, )
        a_params = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope='Actor')
        c_params = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope='Critic')
        ema = tf.train.ExponentialMovingAverage(decay=1 - self.TAU)          # soft replacement

        def ema_getter(getter, name, *args, **kwargs):
            return ema.average(getter(name, *args, **kwargs))

        target_update = [ema.apply(a_params), ema.apply(c_params)]      # soft update operation
        a_ = self._build_a(self.S_, reuse=True, custom_getter=ema_getter)   # replaced target parameters
        q_ = self._build_c(self.S_, a_, reuse=True, custom_getter=ema_getter)

        a_loss = - tf.reduce_mean(q)  # maximize the q
        self.atrain = tf.train.AdamOptimizer(self.LR_A).minimize(a_loss, var_list=a_params)

        with tf.control_dependencies(target_update):    # soft replacement happened at here
            q_target = self.R + self.GAMMA * q_
            td_error = tf.losses.mean_squared_error(labels=q_target, predictions=q)
            self.ctrain = tf.train.AdamOptimizer(self.LR_C).minimize(td_error, var_list=c_params)

        self.sess.run(tf.global_variables_initializer())
        self.saver = tf.train.Saver(max_to_keep=20)#qinjielin 

    def choose_action(self, s):
        return self.sess.run(self.a, {self.S: s[np.newaxis, :]})[0]

    def learn(self):
        indices = np.random.choice(self.MEMORY_CAPACITY, size=self.BATCH_SIZE)
        bt = self.memory[indices, :]
        bs = bt[:, :self.s_dim]
        ba = bt[:, self.s_dim: self.s_dim + self.a_dim]
        br = bt[:, -self.s_dim - 1: -self.s_dim]
        bs_ = bt[:, -self.s_dim:]

        self.sess.run(self.atrain, {self.S: bs})
        self.sess.run(self.ctrain, {self.S: bs, self.a: ba, self.R: br, self.S_: bs_})

    def store_transition(self, s, a, r, s_):
        transition = np.hstack((s, a, [r], s_))
        index = self.pointer % self.MEMORY_CAPACITY  # replace the old memory with new memory
        self.memory[index, :] = transition
        self.pointer += 1

    def _build_a(self, s, reuse=None, custom_getter=None):
        trainable = True if reuse is None else False
        with tf.variable_scope('Actor', reuse=reuse, custom_getter=custom_getter):
            net = tf.layers.dense(s, 30, activation=tf.nn.relu, name='l1', trainable=trainable)
            a = tf.layers.dense(net, self.a_dim, activation=tf.nn.tanh, name='a', trainable=trainable)
            return tf.multiply(a, self.a_bound, name='scaled_a')

    def _build_c(self, s, a, reuse=None, custom_getter=None):
        trainable = True if reuse is None else False
        with tf.variable_scope('Critic', reuse=reuse, custom_getter=custom_getter):
            n_l1 = 30
            w1_s = tf.get_variable('w1_s', [self.s_dim, n_l1], trainable=trainable)
            w1_a = tf.get_variable('w1_a', [self.a_dim, n_l1], trainable=trainable)
            b1 = tf.get_variable('b1', [1, n_l1], trainable=trainable)
            net = tf.nn.relu(tf.matmul(s, w1_s) + tf.matmul(a, w1_a) + b1)
            return tf.layers.dense(net, 1, trainable=trainable)  # Q(s,a)

    def save_model(self,fileLoc,filename,step):
        # os_path = os.path.dirname(os.path.abspath(__file__))
        # cpath = os_path + '/saved_model/'+filename+'/PPO_model-'+str(step)+'.ckpt'
        cpath = fileLoc+filename+'/RL_model-'+str(step)+'.ckpt'
        save_path = self.saver.save(self.sess, cpath)#qinjielin 
        print("Model saved in path: %s" % save_path)
        return

    def load_model(self,fileLoc,filename,step):
        # os_path = os.path.dirname(os.path.abspath(__file__))
        # cpath = os_path+'/saved_model/'+filename+'/PPO_model-'+str(step)+'.ckpt'
        cpath = fileLoc+filename+'/RL_model-'+str(step)+'.ckpt'
        self.saver.restore(self.sess,cpath)#qinjielin
        print("model restore")
        return


if __name__ == "__main__":
    rospy.init_node('turtlebot3_DDPG_v1')
    trainFlag = rospy.get_param('/train')
    loadFalg =  rospy.get_param('/load')
    filename = rospy.get_param('/filename')
    load_ep = rospy.get_param('/load_ep')
    ep_len = rospy.get_param('/ep_len')
    itimateFlag = rospy.get_param('/itimateFlag')
    laserStep = rospy.get_param('/laser_step')
    # complexNN = rospy.get_param('/complex_nn')
    fileLoc = os.path.dirname(os.path.abspath(__file__))+"/../../../../../../clever/saved_model_ddpg/"
    if( not loadFalg):
        if(trainFlag):
            saveModelInfo(fileLoc,filename)

#####################  hyper parameters  ####################

    MAX_EPISODES = 300
    MAX_EP_STEPS = int(ep_len)
    LR_A = 0.001    # learning rate for actor
    LR_C = 0.002    # learning rate for critic
    GAMMA = 0.9     # reward discount
    TAU = 0.01      # soft replacement
    MEMORY_CAPACITY = 10000
    BATCH_SIZE = 32

    var =None # control exploration
    if(trainFlag):
        var = 3  
    else:
        var = 0.1#3 * (0.9995 ** load_ep) #0.1
    model_gap = 25

###############################  training  ####################################

    gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.02)
    env =Env()
    #s_dim = 10
    #a_dim = 2
    s_dim, a_dim = int(360/laserStep)+4, 2  #QinjieLin 16,2
    a_bound = np.asarray([2.])

    ddpg = DDPG(a_dim, s_dim, a_bound,LR_A,LR_C,GAMMA,TAU,MEMORY_CAPACITY,BATCH_SIZE,gpu_options)
    if( loadFalg):
        ddpg.load_model(fileLoc,filename,load_ep)

    t1 = time.time()
    all_ep_r = []
    for i in range(MAX_EPISODES):
        ep = i
        s = env.reset()
        ep_reward = 0
        for j in range(MAX_EP_STEPS):

            # Add exploration noise
            # print("state before:",s)
            a = ddpg.choose_action(s)
            if(trainFlag):
                a = np.clip(np.random.normal(a, var), -2, 2)    # add randomness to action selection for exploration
            s_, r, done = env.step(a)
            # print("state after:",s_)

            if(trainFlag):

                ddpg.store_transition(s, a, r / 10, s_)

                if ddpg.pointer > MEMORY_CAPACITY:
                    var *= .9995    # decay the action randomness
                    ddpg.learn()

            if done:
                s_ = env.reset()
            s = s_
            ep_reward += r
            if j == MAX_EP_STEPS-1:
                print('Episode:', i, ' Reward: %i' % int(ep_reward), 'Explore: %.2f' % var, )
                # if ep_reward > -300:RENDER = True
                break

        if(trainFlag):
            saveEp = ep
            if(loadFalg): 
                load_ep = load_ep + 1
                saveEp = load_ep
            if((saveEp%model_gap)==0):
                ddpg.save_model(fileLoc,filename,saveEp)

        if ep == 0: all_ep_r.append(ep_reward)
        else: all_ep_r.append(all_ep_r[-1]*0.9 + ep_reward*0.1)

        #save ep reward data
        if(trainFlag):
            dataEp = ep
            if(loadFalg):
                dataEp = load_ep
            writeData(fileLoc,filename,dataEp,all_ep_r[-1],env.numSuccess)
        print('Ep: %i' % ep,
            "model is saved in %s"% filename,
            "success time: %i" %env.numSuccess)
            

    print('Running time: ', time.time() - t1)