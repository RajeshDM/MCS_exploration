
#import tensorflow as tf

import numpy as np
import pdb
import os
import random
import threading
import tensorflow as tf
import time
import json

from networks.free_space_network import FreeSpaceNetwork
#from supervised.sequence_generator import SequenceGenerator
from sequence_generator import SequenceGenerator

from utils import tf_util
from utils import game_util

import constants
#import mcs.cover_floor
import cover_floor

data_buffer = []
data_counts = np.full(constants.REPLAY_BUFFER_SIZE, 9999)
os.environ["CUDA_VISIBLE_DEVICES"] = str(constants.GPU_ID)


def create_scene_numbers(max_scene_number):
    scene_numbers = []
    for i in range (0,10):
        for j in range (0,10):
            for k in range(0,10):
                for l in range(0,10):
                    if i == 0 and j == 0 and k == 0 and l == 0:
                        continue
                    scene_numbers.append(str(i)+ str(j)+ str(k)+str(l))
                    if len(scene_numbers) >= max_scene_number:
                        return scene_numbers
    

try:
    tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)
    with tf.variable_scope('nav_global_network'):
        network = FreeSpaceNetwork(constants.GRU_SIZE, constants.BATCH_SIZE, constants.NUM_UNROLLS)
        network.create_net()
        training_step = network.training_op

    with tf.variable_scope('loss'):
        loss_summary_op = tf.summary.merge([
            tf.summary.scalar('loss', network.loss),
            ])
    summary_full = tf.summary.merge_all()
    conv_var_list = [v for v in tf.trainable_variables() if 'conv' in v.name and 'weight' in v.name and
                    (v.get_shape().as_list()[0] != 1 or v.get_shape().as_list()[1] != 1)]
    for var in conv_var_list:
        tf_util.conv_variable_summaries(var, scope=var.name.replace('/', '_')[:-2])
    summary_with_images = tf.summary.merge_all()


    sess = tf_util.Session()
    sequence_generator = SequenceGenerator(sess)

    sess.run(tf.global_variables_initializer())

    all_scene_types = ['retrieval_goal-', 'traversal_goal-', 'transferral_goal-']
    #scene_types = ['retrieval_goal-', 'traversal_goal-', 'transferral_goal-']
    #scene_types = ['retrieval_goal-', 'traversal_goal-']#, 'transferral_goal-']
    #scene_types = ['transferral_goal-']
    scene_types = ['retrieval_goal-']


    scene_numbers = ['0933']
    #scene_numbers = create_scene_numbers(2)
    print (scene_numbers)
    #exit()
    #scene_number = [i]
    all_data = {}
    training_data = {}
    exploration_data = {}
    actual_count_of_explored_scenes = {}
    total_goal_objects_found = {}
    for elem in scene_types :
        all_data[elem] = {"explored": [], "actual":[], 'explored_total':0, 'actual_total':0}
        training_data[elem] = {}
        exploration_data[elem] = {}
        actual_count_of_explored_scenes[elem] = 0
        total_goal_objects_found[elem] = 0

    #env = game_util.create_ai2thor_env()


    for scene_type in scene_types :
        for scene_number in scene_numbers :
            
            current_explored = 0
            new_data, bounds, goal_pose = sequence_generator.explore_scene(str(scene_type)+ scene_number + ".json")
            current_explored_objects = sequence_generator.agent.game_state.discovered_objects
            current_explored = len(current_explored_objects)
            
            #sequence_generator.agent.game_state.env.end_scene('', 0.0) 
            goal = sequence_generator.agent.game_state.goal
            goal_objects = []
            
            #if goal['category'] == all_scene_types[-1][:-6]:
            #    goal_objects.append(goal.metadata['target_1']["id"])
            #    goal_objects.append(goal.metadata['target_2']["id"])
                #print (sequence_generator.agent.game_state.goal.metadata['target']["id"] )
            
            print (type(goal))
            #for key,value in sequence_generator.agent.game_state.goal.__dict__.items():
            for key,value in goal.metadata.items():
                if key == "target" or key == "target_1" or key == "target_2":
                    goal_objects.append(goal.metadata['target']["id"])
                    #goal_objects.append(goal.metadata['target_2']["id"])
                    #print (key, type(value))

            #sequence_generator.agent.game_state.discovered_objects = []
            print ("Total objects discovered = " ,current_explored )
            #with open("discovered_data.json","w") as fp:   
            #    print ("number of objects discovered until now : ",len(sequence_generator.agent.game_state.discovered_objects))  
            #    json.dump(sequence_generator.agent.game_state.discovered_objects,fp,indent=1)  
            
            for elem in goal_objects :
                if elem in current_explored_objects:
                    total_goal_objects_found[scene_type] += 1 

            '''
            Checking for number of objects by using AIthor controller
            current_actual = 0
            event = game_util.reset_ai2thor_env(env,str(scene_type)+ scene_number + ".json")
            current_actual = len(event.metadata['objects'])
            '''
           
            #all_data[scene_type]['explored'].append(current_explored)
            #all_data[scene_type]['actual'].append(current_actual)
            all_data[scene_type]['explored_total'] += current_explored
            #all_data[scene_type]['actual_total'] += current_actual
            
            #training_data[scene_type][scene_number] = current_actual
            exploration_data[scene_type][scene_number] = current_explored
            
        for key,items in all_data.items():
            print ("Explored total= ", items['explored_total'])
            #print ("Actual", items['actual_total'])

    actual_data = json.load(open('training_total_objects_data.json'))

    for key,value in exploration_data.items() :
        for key2, value2 in value.items() :
            actual_count_of_explored_scenes[key] += actual_data[key][key2] 

    #print ("Total explored = " , all_data.items)
    for key,items in all_data.items():
        print ("Total explored    for scenes in {} is {}".format(key, items['explored_total']))
        print ("Total actual      for scenes in {} is {}".format( key, actual_count_of_explored_scenes[key]))
        print ("Total goal found  for scenes in {} is {}".format( key, total_goal_objects_found[key]))


    '''
    with open("training_total_objects_data.json","w") as fp:   
        json.dump(training_data,fp,indent=1)  
        #Actual 2105 - retrieval 
        #Actual 3670 - traversal
        #Actual 3480 - transferal
    '''

except:
    import traceback
    traceback.print_exc()
