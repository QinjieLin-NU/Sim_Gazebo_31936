#!/usr/bin/env python
import os
import numpy as np
import rospy
import torch
import torch.nn as nn
#from mpi4py import MPI

from torch.optim import Adam
from collections import deque

from model.net import MLPPolicy, CNNPolicy
#from circle_world import StageWorld
from turtlebot_world_v2 import TurtlebotWorld
# from turtlebot_world_v1 import TurtlebotWorld
from model.ppo import generate_action_no_sampling, transform_buffer
import copy
from respawnGoal import Respawn




MAX_EPISODES = 5000
LASER_BEAM = 360
LASER_HIST = 3
HORIZON = 200
GAMMA = 0.99
LAMDA = 0.95
BATCH_SIZE = 512
EPOCH = 3
COEFF_ENTROPY = 5e-4
CLIP_VALUE = 0.1
NUM_ENV = 50
OBS_SIZE = 360
ACT_SIZE = 2
LEARNING_RATE = 5e-5


def enjoy(env, policy, action_bound,respawn_goal):


    # if env.index == 0:
    #     env.reset_world()

    #env.reset_pose()

    env.generate_goal_point()
    step = 1
    terminal = False

    obs = env.get_laser_observation()
    #print("obs",obs)
    obs_stack = deque([obs, obs, obs])
    #print("get obs_stack")
    # goals = raw_input("Press Enter to continue...")
    goals = respawn_goal.getPosition()
    env.goal_point=[(goals[0]),(goals[1])]
    goal = np.asarray(env.get_local_goal())
    # print("goal",goal)
    # print("goal.shape",goal.shape)
    # print("goal.shape[0]",goal.shape[0])
    speed = np.asarray(env.get_self_speed())
    #print("speed",speed)
    state = [obs_stack, goal, speed]

    while not rospy.is_shutdown():
        # get next state
        s_next = env.get_laser_observation()
        left = obs_stack.popleft()
        obs_stack.append(s_next)
 
        if terminal == True:
            real_action = [0,0]
            env.control_vel(real_action)  
            # goals = raw_input("Press Enter to continue...")
            # goals = goals.split(" ")
            goals = respawn_goal.getPosition() 
            env.goal_point=[(goals[0]),(goals[1])]

        goal_next = np.asarray(env.get_local_goal())
        #print("goal_next",goal_next)
        speed_next = np.asarray(env.get_self_speed())
        state_next = [obs_stack, goal_next, speed_next]


        state = state_next

        #state_list = comm.gather(state, root=0)
        state_list = state

        # generate actions at rank==0
        mean, scaled_action =generate_action_no_sampling(env=env, state_list=state_list,
                                            policy=policy, action_bound=action_bound)      
        # execute actions
        real_action = copy.deepcopy(scaled_action[0])
        #real_action[1]=real_action[1]
        #real_action = comm.scatter(scaled_action, root=0)

        print("real_action",real_action)
        env.control_vel(real_action)
        # rate.sleep()
        rospy.sleep(0.05)
        # get informtion
        r, terminal, result = env.get_reward_and_terminate(step)
        step += 1






if __name__ == '__main__':

    #comm = MPI.COMM_WORLD
    #rank = comm.Get_rank()
    #size = comm.Get_size()
    rospy.init_node("turtlebot3_ppo_v3", anonymous=None)
    filename = rospy.get_param('/filename',"autoRL_2")
    load_ep = rospy.get_param('/load_ep',800)
    policy_path =    os.path.dirname(os.path.abspath(__file__))+"/../../../../../../clever/saved_model_ppo/"
    file = policy_path + filename+'/Stage1_%d'%load_ep
    print("filename:",file)
    respawn_goal = Respawn()

    print("begin to load world")
    env = TurtlebotWorld(OBS_SIZE, index=1, num_env=NUM_ENV)
    print("generate the world")
    reward = None
    action_bound = [[0, -3], [1, 3]]


    policy_path = 'policy'
    # policy = MLPPolicy(obs_size, act_size)
    policy = CNNPolicy(frames=LASER_HIST, action_space=2)
    policy.cuda()
    opt = Adam(policy.parameters(), lr=LEARNING_RATE)
    mse = nn.MSELoss()

    if not os.path.exists(policy_path):
        os.makedirs(policy_path)

    # file = policy_path + '/Stage1_800'
    if os.path.exists(file):
        print ('####################################')
        print ('############Loading Model###########')
        print ('####################################')
        state_dict = torch.load(file)
        policy.load_state_dict(state_dict)
    else:
        print ('Error: Policy File Cannot Find')
        exit()





    try:
        enjoy(env=env, policy=policy, action_bound=action_bound,respawn_goal=respawn_goal)
    except KeyboardInterrupt:
        import traceback
        traceback.print_exc()
