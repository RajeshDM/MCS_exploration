import numpy as np
import random
from utils import drawing
from qa_agents import graph_agent
#import graph_agent

import constants
from utils import game_util
#import game_util
#from mcs import cover_floor
import cover_floor
from cover_floor import *
import math
import time

class SequenceGenerator(object):
    def __init__(self, sess):
        self.agent = graph_agent.GraphAgent(sess, reuse=True)
        self.game_state = self.agent.game_state
        self.action_util = self.game_state.action_util
        self.planner_prob = 0.5
        self.scene_num = 0
        self.count = -1
        self.scene_name = None

    def generate_episode(self):
        self.count += 1
        self.states = []
        self.debug_images = []
        planning = random.random() < self.planner_prob

        while len(self.states) == 0:
            
            if self.count % 50000 == 0:
                #self.scene_name = 'FloorPlan%d' % random.choice(constants.SCENE_NUMBERS)
                #self.scene_name = 'FloorPlan%d' % 6#random.choice(constants.SCENE_NUMBERS)
                self.scene_name = 'transferral_data'
                print('New episode. Scene %s' % self.scene_name)
                self.agent.reset(self.scene_name)
            else:
                self.agent.reset()
            
            #self.agent.reset()
            print ("reset done in sequence generator")
            label = self.agent.get_label()
            pose = game_util.get_pose(self.game_state.event)[:3]

            if constants.DRAWING:
                patch = self.game_state.graph.get_graph_patch(pose)[0]
                self.debug_images.append({
                    'color': self.game_state.s_t,
                    'label': np.flipud(label),
                    'patch': np.flipud(patch),
                    'label_memory': np.minimum(np.flipud(self.agent.gt_graph.memory.copy()), 2),
                    'state_image': self.game_state.draw_state().copy(),
                    'memory_map': np.minimum(np.flipud(self.game_state.graph.memory.copy()), 10),
                    'pose_indicator': np.flipud(self.agent.pose_indicator),
                    'detections': self.game_state.detection_image if constants.OBJECT_DETECTION else None,
                    'weight': 1,
                    'possible_label': (1 if self.game_state.graph.memory[
                                                self.game_state.end_point[1] - self.game_state.graph.yMin,
                                                self.game_state.end_point[0] - self.game_state.graph.xMin, 0] == 1
                                       else self.game_state.is_possible_end_point),
                    'possible_pred': self.agent.is_possible,
                    })
            self.states.append({
                'color': self.game_state.s_t,
                'pose': self.agent.gt_graph.get_shifted_pose(self.agent.pose)[:3],
                'label': label,
                'action': np.zeros(self.action_util.num_actions),
                'pose_indicator': self.agent.pose_indicator,
                'weight': 1,
                #'possible_label': (1 if self.game_state.graph.memory[
                #                            self.game_state.end_point[1] - self.game_state.graph.yMin,
                #                            self.game_state.end_point[0] - self.game_state.graph.xMin, 0] == 1
                #                   else self.game_state.is_possible_end_point),
                })
            optimal_plan, optimal_path = self.agent.gt_graph.get_shortest_path(
                    pose, self.game_state.end_point)
            if planning:
                plan, path = self.agent.get_plan()
            else:
                plan = optimal_plan
                path = optimal_path
            #print ("optimal path planning done", path, plan)
            num_iters = 0
            seen_terminal = False
            while ((not seen_terminal) and len(plan) != 0 and
                    self.agent.is_possible >= constants.POSSIBLE_THRESH):
                num_iters += 1
                print ("In the while loop of executing the plan")
                if constants.DEBUG:
                    print('num iters', num_iters, 'max', constants.MAX_EPISODE_LENGTH)
                if num_iters > constants.MAX_EPISODE_LENGTH:
                    print('Path length too long in scene',
                          self.scene_name, 'goal_position', self.game_state.end_point,
                          'pose', pose, 'plan', plan)
                    plan = []
                    break

                action_vec = np.zeros(self.action_util.num_actions)
                if len(plan) > 0:
                    action = plan[0]
                    #print ("action to take" , action)
                    self.agent.step(action)
                    action_vec[self.action_util.action_dict_to_ind(action)] = 1
                pose = game_util.get_pose(self.game_state.event)[:3]

                optimal_plan, optimal_path = self.agent.gt_graph.get_shortest_path(
                        pose, self.game_state.end_point)
                if planning:
                    plan, path = self.agent.get_plan()
                else:
                    plan = optimal_plan
                    path = optimal_path

                label = self.agent.get_label()
                self.states.append({
                    'color': self.game_state.s_t,
                    'pose': self.agent.gt_graph.get_shifted_pose(self.agent.pose)[:3],
                    'label': label,
                    'action': action_vec,
                    'pose_indicator': self.agent.pose_indicator,
                    'weight': 1,
                    'possible_label': (1 if self.game_state.graph.memory[
                                               self.game_state.end_point[1] - self.game_state.graph.yMin,
                                               self.game_state.end_point[0] - self.game_state.graph.xMin, 0] == 1
                                       else self.game_state.is_possible_end_point),
                    })
                #print ("self.states ", self.states)
                seen_terminal = seen_terminal or int(len(optimal_plan) == 0)
                if self.states[-1]['label'].shape != (constants.STEPS_AHEAD, constants.STEPS_AHEAD):
                    self.states = []
                    print('Label is wrong size scene', self.scene_name, 'pose', pose)
                    break
                if constants.DRAWING:
                    patch = self.game_state.graph.get_graph_patch(pose)[0]
                    self.debug_images.append({
                        'color': self.game_state.s_t,
                        'label': np.flipud(label),
                        'patch': np.flipud(patch),
                        'label_memory': np.minimum(np.flipud(self.agent.gt_graph.memory.copy()), 2),
                        'state_image': self.game_state.draw_state().copy(),
                        'pose_indicator': np.flipud(self.agent.pose_indicator),
                        'detections': self.game_state.detection_image if constants.OBJECT_DETECTION else None,
                        'memory_map': np.minimum(np.flipud(self.game_state.graph.memory.copy()), 10),
                        'possible_label': (1 if self.game_state.graph.memory[
                                                    self.game_state.end_point[1] - self.game_state.graph.yMin,
                                                    self.game_state.end_point[0] - self.game_state.graph.xMin, 0] == 1
                                           else self.game_state.is_possible_end_point),
                        'possible_pred': self.agent.is_possible,
                        })
            break
        self.bounds = [self.game_state.graph.xMin, self.game_state.graph.yMin,
            self.game_state.graph.xMax - self.game_state.graph.xMin + 1,
            self.game_state.graph.yMax - self.game_state.graph.yMin + 1]
        goal_pose = np.array([self.game_state.end_point[0] - self.game_state.graph.xMin,
                self.game_state.end_point[1] - self.game_state.graph.yMin],
                dtype=np.int32)[:2]
        return (self.states, self.bounds, goal_pose)

    def explore_3d_scene(self,config_filename):
        self.scene_name = 'transferral_data'
        #print('New episode. Scene %s' % self.scene_name)
        self.agent.reset(self.scene_name,config_filename = config_filename)

        pose = game_util.get_pose(self.game_state.event)[:3]
        plan, path = self.agent.gt_graph.get_shortest_path(
                pose, self.game_state.end_point)
        #print ("optimal path planning done", path, plan)
        num_iters = 0
        exploration_routine = []
        exploration_routine = cover_floor.flood_fill(0,0, cover_floor.check_validity)        
        #print (exploration_routine, len(exploration_routine))
        self.graph = graph_2d()
        #unexplored = get_unseen(self.graph.graph)
        #print (len(unexplored))
        #print (unexplored)

        self.event = self.game_state.event
        #visible_points = get_visible_points(pose[0],pose[1],pose[2],self.event.camera_field_of_view,self.event.camera_clipping_planes[1] )
        #print ("visible points = " , visible_points)
        #print (len(visible_points))


        #number_visible_points = points_visible_from_position(exploration_routine[1][0],exploration_routine[1][1], self.event.camera_field_of_view,self.event.camera_clipping_planes[1] )  
        #print ("number of visible points = ", number_visible_points)

        update_seen(self.graph.graph,pose[0],pose[1],pose[2],self.game_state.event)
        unexplored = get_unseen(self.graph.graph)
        print (len(unexplored))
        #print (unexplored)
        #return 

        while ( len(unexplored) > 25 ) : 
            start_time = time.time()

            while len(plan) > 0:
                action = plan[0]
                #print ("action to take" , action)
                self.agent.step(action)
                self.event = self.game_state.event
                plan = plan[1:]
                path.pop()
                pose = game_util.get_pose(self.game_state.event)[:3]
                #update_seen_points(pose)
                update_seen(self.graph.graph,pose[0],pose[1],pose[2],self.game_state.event)
             
                #print ("pose_reached =" , pose)
            
            max_visible = 0
            max_visible_position = []

            print ("before next best point calculation")
        
            for elem in exploration_routine:
                number_visible_points = points_visible_from_position(exploration_routine[1][0],exploration_routine[1][1], self.event.camera_field_of_view,self.event.camera_clipping_planes[1] )  
                if max_visible < number_visible_points:
                    max_visible_position.append(elem)
                #points_visible(elem)

            new_end_point = [0]*3
            new_end_point[0] = max_visible_position[-1][0]
            new_end_point[1] = max_visible_position[-1][1]
            new_end_point[2] = pose[2]
        
            exploration_routine.remove(max_visible_position[-1])
            
            plan, path = self.agent.gt_graph.get_shortest_path(
                    pose, tuple(new_end_point))
            #print ("optimal path planning done", path, plan)
            unexplored = get_unseen(self.graph.graph)
            print (len(unexplored))
            end_time = time.time()
            print ("Time taken for 1 loop run = ", end_time - start_time)
            
            

    def explore_scene(self,config_filename):
        self.scene_name = 'transferral_data'
        #print('New episode. Scene %s' % self.scene_name)
        self.agent.reset(self.scene_name,config_filename = config_filename)
        self.states = []
        planning = random.random() < self.planner_prob

        #print ("reset done in sequence generator")
        label = self.agent.get_label()
        pose = game_util.get_pose(self.game_state.event)[:3]

        optimal_plan, optimal_path = self.agent.gt_graph.get_shortest_path(
                pose, self.game_state.end_point)
        plan = optimal_plan
        path = optimal_path
        print ("optimal path planning done", path, plan)
        num_iters = 0
        seen_terminal = False

        exploration_routine = []
        exploration_routine = cover_floor.flood_fill(0,0, cover_floor.check_validity)        
        print (exploration_routine, len(exploration_routine))

        while ((not seen_terminal) and len(plan) != 0 and
                self.agent.is_possible >= constants.POSSIBLE_THRESH):
            num_iters += 1
            #print ("In the while loop of executing the plan")

            action_vec = np.zeros(self.action_util.num_actions)
            while len(plan) > 0:
                action = plan[0]
                #print ("action to take" , action)
                self.event = self.agent.step(action)
                action_vec[self.action_util.action_dict_to_ind(action)] = 1
                #plan.pop()
                #print(plan)
                plan = plan[1:]
                path.pop()
                pose = game_util.get_pose(self.game_state.event)[:3]
                print ("pose_reached =" , pose)

            #explore_objects()
            flag = 0
            object_id_to_search = ""
            for key,value in self.agent.game_state.discovered_explored.items():
                for key_2,value_2 in value.items():
                    if key_2 == 0:# and key not in self.unexplored_objects:
                        #self.unexplored_objects[key] = value
                        graph_pos_x =  math.floor(value_2['x']/constants.AGENT_STEP_SIZE) 
                        graph_pos_z =  math.floor(value_2['z']/constants.AGENT_STEP_SIZE) 
                        if math.sqrt((abs(pose[0] -graph_pos_x))**2+(abs(pose[1]-graph_pos_z))**2) < 10:
                            print ("objects nearby to explore ")    
                            flag = 1
                            optimal_plan, optimal_path = self.agent.gt_graph.get_shortest_path(
                                    pose, (graph_pos_x,graph_pos_z,pose[2]))
                            object_id_to_search = key
                            plan = optimal_plan
                            path = optimal_path
                            break
                if flag == 1 :
                    break

            while len(plan) > 0:
                action = plan[0]
                #print ("action to take" , action)
                self.event = self.agent.step(action)
                action_vec[self.action_util.action_dict_to_ind(action)] = 1
                #plan.pop()
                #print(plan)
                plan = plan[1:]
                path.pop()
                pose = game_util.get_pose(self.game_state.event)[:3]
                print ("pose_reached while going to object=" , pose)

            if flag == 1:
                action = {"action":"OpenObject", "objectId":object_id_to_search}
            
                self.event = self.agent.step(action)
                print ("return status of open object aciton:",self.agent.game_state.event.return_status)

            '''
            calculate_object_graph_position()
            
            for key,value in self.unexplored_objects.items():
                for key_2,value_2 in value.items():
                    graph_pos_x =  math.floor(value['x']/constants.AGENT_STEP_SIZE) 
                    graph_pos_z =  math.floor(value['z']/constants.AGENT_STEP_SIZE) 
                    if math.sqrt((abs(pose[0] -graph_pos_x))**2+(abs(pose[1]-graph_pos_z))**2):
                        optimal_plan, optimal_path = self.agent.gt_graph.get_shortest_path(
                                pose, tuple(graph_pos_x,graph_pos,pose[2]))
            '''
            #print ("done going to first end point")
            #print (exploration_routine, len(exploration_routine))

            new_end_point = [0]*3
            if len(exploration_routine) > 0 :
                new_end_point_data = exploration_routine.pop()
            else :
                break
            #newend_point[0] = new_end_point[0]
            #self.game_state.end_point[1] = new_end_point[1]
            #self.game_state.end_point[2] = self.game_state[2]
            new_end_point[0] = new_end_point_data[0]
            new_end_point[1] = new_end_point_data[1]
            new_end_point[2] = self.game_state.end_point[2]
            
            #print ("new starting point = ", pose)
            #print ("new ending point   = ", tuple(new_end_point))

            optimal_plan, optimal_path = self.agent.gt_graph.get_shortest_path(
                    #pose, self.game_state.end_point)
                    pose, tuple(new_end_point))
            #if planning:
            #    print ("new plannning")
            #    plan, path = self.agent.get_plan()
            #else:
            #    print ("not new plannning")
            plan = optimal_plan
            path = optimal_path
            print ("optimal path planning done", path, plan)

            label = self.agent.get_label()
            #print ("self.states ", self.states)
            #seen_terminal = seen_terminal or int(len(optimal_plan) == 0)
       
        #self.agent.game_state.env.end_scene('', 0.0) 

        self.bounds = [self.game_state.graph.xMin, self.game_state.graph.yMin,
            self.game_state.graph.xMax - self.game_state.graph.xMin + 1,
            self.game_state.graph.yMax - self.game_state.graph.yMin + 1]
        goal_pose = np.array([self.game_state.end_point[0] - self.game_state.graph.xMin,
                self.game_state.end_point[1] - self.game_state.graph.yMin],
                dtype=np.int32)[:2]
        return (self.states, self.bounds, goal_pose)



if __name__ == '__main__':
    from networks.free_space_network import FreeSpaceNetwork
    from utils import tf_util
    import tensorflow as tf
    sess = tf_util.Session()

    with tf.variable_scope('nav_global_network'):
        network = FreeSpaceNetwork(constants.GRU_SIZE, 1, 1)
        network.create_net()
    sess.run(tf.global_variables_initializer())
    start_it = tf_util.restore_from_dir(sess, constants.CHECKPOINT_DIR)

    import cv2

    sequence_generator = SequenceGenerator(sess)
    sequence_generator.planner_prob = 1
    counter = 0
    while True:
        states, bounds, goal_pose = sequence_generator.generate_episode()
        images = sequence_generator.debug_images
        for im_dict in images:
            counter += 1

            gt_map = (2 - im_dict['label_memory'][:,:,0])

            image_list = [
                    im_dict['detections'] if constants.OBJECT_DETECTION else im_dict['color'],
                    im_dict['state_image'],
                    im_dict['memory_map'][:,:,0],
                    gt_map + np.argmax(im_dict['memory_map'][:,:,1:constants.NUM_RECEPTACLES + 2], axis=2),
                    gt_map + np.argmax(im_dict['memory_map'][:,:,constants.NUM_RECEPTACLES + 2:], axis=2),
                    ]
            titles = ['color', 'state', 'occupied', 'label receptacles', 'label objects']
            print('possible pred', im_dict['possible_pred'])
            image = drawing.subplot(image_list, 2, 2, constants.SCREEN_WIDTH, constants.SCREEN_HEIGHT,
                    titles=titles)
            cv2.imshow('image', image[:,:,::-1])
            cv2.waitKey(0)



