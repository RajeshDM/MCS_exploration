
import constants
import networkx as nx
import math

testing = 0

#max_abs = 5/constants.AGENT_STEP_SIZE
if testing != 1 :
    #AGENT_STEP_SIZE = 0
    max_abs = 5/constants.AGENT_STEP_SIZE
    move_step_size = 6
else :
    move_step_size = 1
    max_abs =1/constants.AGENT_STEP_SIZE
x_range = max_abs
z_range = max_abs

xMin = -x_range
xMax = x_range
zMin = -z_range
zMax = z_range

number_direction = 4
#q = [] 


class graph_2d():
    def __init__(self):#,xMin,xMax,yMin,yMax):
        self.graph = nx.DiGraph() 
        '''
        xMin = int(global xMin)
        yMin = int(global yMin)
        xMax = int(global xMax)
        yMax = int(global yMax)
        
        xMin_local = int(xMin)
        zMin_local = int(zMin)
        xMax_local = int(xMax)
        zMax_local = int(zMax)
        '''
        self.xMin = int(xMin)
        self.yMin = int(zMin)
        self.xMax = int(xMax)
        self.yMax = int(zMax)
        not_seen_points = []
        #for x in range (xMin_local, xMax_local):
        #    for y in range (xMin_local, yMax_local):
       
        for xx in range (self.xMin, self.xMax):
            for yy in range (self.yMin, self.yMax):
                self.graph.add_node((xx,yy), visited=False, seen=False, contains_object = False)
                curr_weight = 1
                if yy != self.yMax:
                    self.graph.add_edge((xx,yy),(xx, yy + 1), weight=curr_weight)
                elif xx != self.xMax:
                    self.graph.add_edge((xx,yy),(xx + 1, yy), weight=curr_weight)
                elif yy != self.yMin:
                    self.graph.add_edge((xx,yy),(xx, yy - 1), weight=curr_weight)
                elif xx != self.xMin:
                    self.graph.add_edge((xx,yy),(xx - 1, yy), weight=curr_weight)
                #not_seen_points.append((x,y))    

        #print (self.graph.nodes[(-4,-3)]['visited'])

'''
Get the set of points visible from a given position and 
dircetion of the agent
''' 
def get_visible_points(x,y,direction,camera_field_of_view,radius):
    '''
    𝑝2.𝑥=𝑐𝑝.𝑥+𝑟∗𝑐𝑜𝑠(𝛼+𝜃)
    𝑝2.𝑦=𝑐𝑝.𝑦+𝑟∗𝑠𝑖𝑛(𝛼+𝜃)
    '''
    visible_points = []
    #radius = event.camera_clipping_planes[1]/constants.AGENT_STEP_SIZE
    #radius = event.camera_clipping_planes[1]#/constants.AGENT_STEP_SIZE
    #radius = event.camera_clipping_planes[1]#/constants.AGENT_STEP_SIZE
    #print ("direction ", direction)
    #print ("radius = ", radius)
    z = y

    #event.camera_field_of_view = 45
    #print (event.camera_field_of_view/2)
    #print (event.rotation)
    #print (event.rotation - (event.camera_field_of_view/2))
    #print (event.rotation + (event.camera_field_of_view/2))
    '''
    angle_pt_1 = math.radians(event.rotation - (event.camera_field_of_view/2))
    angle_pt_2 = math.radians(event.rotation + (event.camera_field_of_view/2))
    '''
    rotation = direction * 90
    angle_pt_1 = math.radians(rotation - (camera_field_of_view/2))
    angle_pt_2 = math.radians(rotation + (camera_field_of_view/2))
    
    p1_x = x + radius * math.cos(angle_pt_1)
    p1_z = z + radius * math.sin(angle_pt_1)

    p2_x = x + radius * math.cos(angle_pt_2)
    p2_z = z + radius * math.sin(angle_pt_2)

    #print (p2_x-x,p2_z-z )
    #print ((p2_x-x)/(p2_z-z))

    #print ("angle between points p2 and x = ", math.degrees(math.atan((p2_x-x)/(p2_z-z))))
    #print ("angle between points p1 and x = ", math.degrees(math.atan((p1_x-x)/(p1_z-z))))
    #print ("angle between points = ", math.degrees(math.atan(((p2_x-p1_x)/(p2_x-p1_x)))))
    #print ("centre of the curve", x,z)
    
    pt_1_angle =  math.degrees(math.atan((p1_x-x)/(p1_z-z)))
    pt_2_angle =  math.degrees(math.atan((p2_x-x)/(p2_z-z)))


    #print ("pt 1 angle = ", pt_1_angle)
    #print ("pt_2_andle = ", pt_2_angle)

    lower_angle = min(pt_1_angle,pt_2_angle)
    higher_angle = max(pt_1_angle,pt_2_angle)

    #print ("first end point of the curve", p1_x, p1_z)
    #print ("second end point of the curve", p2_x,p2_z)
    
    loop_x_min = max(min(x,p1_x,p2_x),xMin)
    loop_z_min = max(min(z,p1_z,p2_z),zMin)

    loop_x_max = min(max(x,p1_x,p2_x),xMax)
    loop_z_max = min(max(z,p1_z,p2_z),zMax)

    #print ("loop range x = ",loop_x_min,loop_x_max)
    #print ("loop range z = ",loop_z_min,loop_z_max)
    
    for i in range(math.floor(loop_x_min), math.ceil(loop_x_max) ): #, math.ceil(max(abs(p1_x),abs(p2_x)))):
        for j in range(math.floor(loop_z_min), math.ceil(loop_z_max)):# math.ceil(max(abs(p1_z),abs(p2_z)))):
            #print (i,j)
            if math.sqrt( (i-x)**2 + (j-y)**2) < radius:
                #print ("points inside ", (i,j))
                if  (j==z):
                    #print ("j is z" , j)
                    continue
                current_point_angle = math.degrees(math.atan((i-x)/(j-z)))
                #print ("j not z")
                if current_point_angle >= lower_angle and current_point_angle <= higher_angle : 
                    visible_points.append((i,j))
                    #print ("No issue" , (i,j), ", angle = ", current_point_angle)
                else :
                    pass 
                    #print ("angle issue",(i,j), ", angle = ", current_point_angle)

    return visible_points

'''
function to get all the unexplored points in the grid
'''
def get_unseen(g):#,xMin,xMax,yMin,yMax):
    
    xMin_local = int(xMin)
    yMin_local = int(zMin)
    xMax_local = int(xMax)
    yMax_local = int(zMax)
    '''
    xMin = int(global xMin)
    yMin = int(global yMin)
    xMax = int(global xMax)
    yMax = int(global yMax)
    '''
    not_seen_points = []
    for x in range (xMin_local, xMax_local):
        for y in range (yMin_local, yMax_local):
            node = g.nodes[(x,y)]
            #if not (node['visited']) and not(node['seen']) and not(node['contains_object']):
            if not (node['visited'] or node['seen'] or node['contains_object']):
                not_seen_points.append((x,y))    

    return not_seen_points


def points_visible_from_position(x,y,camera_field_of_view,radius):
    number_visible_points = 0
    number_directions = 4
    for direction in range (0,number_directions):
        number_visible_points += len(get_visible_points(x,y,direction,camera_field_of_view, radius))

    return number_visible_points

'''
Function to update any new explored points in the grid
'''
def update_seen(g, x, z, direction,event):
    camera_field_of_view = event.camera_field_of_view
    radius = event.camera_clipping_planes[1]
    visible_points = get_visible_points(x,z,direction,camera_field_of_view,radius)
    for elem in visible_points :
        g.nodes[elem]['seen']= True
    #pass

def explore_point(x,y,graph ,agent):

    directions = 4
    event = agent.game_state.event
    #pose = 
    action = action = "RotateLook, rotation=45"
    for i in range (0,directions,0.5):
        agent.step(action)
        #update_seen( graph, event. , event. ,i ,  event  )
    #pass
    return agent
    

def check_validity(x,z,q):
    if x < xMin :
        return False
    elif x >= xMax :
        return False
    elif z < zMin :
        return False 
    elif z >= zMax:
        return False
    if ((x,z)) in q:
        return False
    return True

def flood_fill(x,y, check_validity):
   #//here check_validity is a function that given coordinates of the point tells you whether
   #//the point should be colored or not
   #Queue q
    curr_q = []
    q = []
    q.append((x,y))
    curr_q.append((x,y))
    i = 1
    while (len(curr_q) != 0):
        #(x1,y1) = curr_q.pop()
        (x1,y1) = curr_q[0]
        curr_q = curr_q[1:]
        #print (x1,y1)
        #color(x1,y1)
 
        if (check_validity(x1+move_step_size,y1,q)):
             q.append((x1+move_step_size,y1))
             curr_q.append((x1+move_step_size,y1))
             i += 1
        if (check_validity(x1,y1+move_step_size,q)):
             q.append((x1,y1+move_step_size))
             curr_q.append((x1,y1+move_step_size))
             i += 1
        if (check_validity(x1-move_step_size,y1,q)):
             q.append((x1-move_step_size,y1))
             curr_q.append((x1-move_step_size,y1))
             i += 1
        if (check_validity(x1,y1-move_step_size,q)):
             q.append((x1,y1-move_step_size))
             curr_q.append((x1,y1-move_step_size))
             i += 1
        #print ("i = ", i, x1,y1, len(q))
        #if i > 35 :
        #    break
    return q

if __name__ == '__main__': 
    #q = flood_fill(0,0,check_validity)
    #print (len(q))
    #print (q)
    g = graph(xMin,xMax,zMin,zMax)
