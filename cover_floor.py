
import constants
import networkx as nx
import math
import time 

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


def  pointInTriangle(x1, y1, x2, y2, x3, y3, x, y):

    denominator = ((y2 - y3)*(x1 - x3) + (x3 - x2)*(y1 - y3))
    a = ((y2 - y3)*(x - x3) + (x3 - x2)*(y - y3)) / denominator
    b = ((y3 - y1)*(x - x3) + (x1 - x3)*(y - y3)) / denominator
    c = 1 - a - b;

    #if x == 0 and (y == 1 or y==2):
    #    print ("a,b,c", a,b,c)

    #if c < 0.0000000001 :
    #    c = 0
 
    return 0 <= a and a <= 1 and 0 <= b and b <= 1 and -0.000000000001 <= c and c <= 1;

#def pointInPolygon( x1, y1, x2, y2, x3, y3, x4, y4, x,y):

#def pointInPolygon(nvert, vertx, verty, testx, testy):
'''
def pointInPolygon(nvert, polygonX, polygonY, targetX, targetY):

    #i, j, c = 0;
    
    #for (i = 0, j = nvert-1; i < nvert; j = i++) {
    j = nvert - 1
    c = False
    i = 0
    for i in range (0, nvert): 
        #if ( ((verty[i]>testy) != (verty[j]>testy)) and (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) ):
        #    c = not c 
        #j = i
        #i = i + 1

        if( polygonX[i] < polygonX[(i + 1) % nvert]):
            tempX = polygonX[i];
            tempY = polygonX[(i + 1) % nvert]
        else :
            tempX = polygonX[(i + 1) % nvert]
            tempY = polygonX[i]
            

        #First check if the ray is possible to cross the line
        if (targetX > tempX and targetX <= tempY and (targetY < polygonY[i] or targetY <= polygonY[(i + 1) % nvert])) :
            eps = 0.000001
            #Calculate the equation of the line
            dx = polygonX[(i + 1) % nvert] - polygonX[i];
            dy = polygonY[(i + 1) % nvert] - polygonY[i];

            if (abs(dx) < eps):
                k = 999999999999999999999
            else:
                k = dy / dx

            m = polygonY[i] - k * polygonX[i]
            #Find if the ray crosses the line
            y2 = k * targetX + m;
            if (targetY <= y2) :
                #crossings++;
                c = not c 

    return c
'''

'''
Get the set of points visible from a given position and 
dircetion of the agent
''' 
def get_visible_points_old(x,y,direction,camera_field_of_view,radius):
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
    #radius = radius * 1.2
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

    #TODO check if pt_1_angle == rotation- camera_filedof_view

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

'''
Funcrtion to get all the visible points from a certain point in the 2D grid
'''

def points_visible_from_position(x,y,camera_field_of_view,radius):
    number_visible_points = 0
    number_directions = 4
    #number_directions = 8
    for direction in range (0,number_directions):#,number_directions*2):
    #for direction in range (0,number_directions):
        #print (direction)
        #number_visible_points += len(get_visible_points(x,y,direction/2,camera_field_of_view, radius))
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

    directions = 8
    event = agent.game_state.event
    #pose = 
    action = "RotateLook, rotation=45"
    for direction in range (0,directions):
        agent.game_state.env.step(action)
        update_seen( graph,x , y ,direction/2 ,  event  )
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


def get_visible_points(x,y,direction,camera_field_of_view,radius):
    
    #camera_field_of_view = 90
    #direction = 0.5
    direction = (direction+3) %4
    #print ("dircetion = ", direction)
    rotation = direction * 90
    z = y
    radius = radius * 1.2
    #rotation_rad = math.radians(rotation)

    angle_pt_1 = math.radians(rotation - (camera_field_of_view/2))
    angle_pt_2 = math.radians(rotation + (camera_field_of_view/2))

    #p_x = x + radius* math.cos(rotation_rad)  
    #p_z = z + radius * sine(rotation_rad)

    p1_x = x + radius * math.cos(angle_pt_1)
    p1_z = z + radius * math.sin(angle_pt_1)

    p2_x = x + radius * math.cos(angle_pt_2)
    p2_z = z + radius * math.sin(angle_pt_2)

    #print (x,z)
    #print (p1_x,p1_z)
    #print (p2_x,p2_z)

    return  get_points_in_triangle(x,z,p1_x,p1_z,p2_x,p2_z)
    

def get_points_in_triangle(x,z,p1_x,p1_z,p2_x,p2_z):

    #(𝑥2−𝑥1)(𝑦3−𝑦1)−(𝑦2−𝑦1)(𝑥3−𝑥1)|≠0
    if (p1_x - x )*(p2_z-z) - (p1_z-z)*(p2_x-x) == 0     :
        return ["co linear points"]

    loop_x_min = math.floor(max(min(x,p1_x,p2_x),xMin))
    loop_z_min = math.floor(max(min(z,p1_z,p2_z),zMin))

    loop_x_max = math.ceil(min(max(x,p1_x,p2_x),xMax))
    loop_z_max = math.ceil(min(max(z,p1_z,p2_z),zMax))

    #print ("loop range x = ",loop_x_min,loop_x_max)
    #print ("loop range z = ",loop_z_min,loop_z_max)
    
    true_count = 0
    total_count = 0
    #in_triangle = False 
    visible_points = []

    for i in range(loop_x_min, loop_x_max+1 ): 
        for j in range(loop_z_min, loop_z_max+1):
            if i == xMax or j == zMax:    
                continue
            in_triangle =   pointInTriangle(x,z,p1_x,p1_z,p2_x,p2_z,i,j)
            total_count += 1
            if in_triangle == True :
                visible_points.append((i,j))
                #true_count += 1
    return visible_points

'''

def get_points_in_polygon(x,z,p1_x,p1_z,p2_x,p2_z, p3_x,p4_z):

    #(𝑥2−𝑥1)(𝑦3−𝑦1)−(𝑦2−𝑦1)(𝑥3−𝑥1)|≠0
    if (p1_x - x )*(p2_z-z) - (p1_z-z)*(p2_x-x) == 0     :
        return ["co linear points"]

    loop_x_min = math.floor(max(min(x,p1_x,p2_x,p3_x),xMin))
    loop_z_min = math.floor(max(min(z,p1_z,p2_z,p3_z),zMin))

    loop_x_max = math.ceil(min(max(x,p1_x,p2_x,p3_x),xMax))
    loop_z_max = math.ceil(min(max(z,p1_z,p2_z,p3_z),zMax))

    print ("loop range x = ",loop_x_min,loop_x_max)
    print ("loop range z = ",loop_z_min,loop_z_max)
    
    true_count = 0
    total_count = 0
    #in_triangle = False 
    visible_points = []

    for i in range(loop_x_min, loop_x_max+1 ): 
        for j in range(loop_z_min, loop_z_max+1):
            if i == xMax or j == zMax:    
                #print ("hitting max value ")
                continue
            #in_triangle =   pointInTriangle(x,z,p1_x,p1_z,p2_x,p2_z,i,j)
            in_polygon = pointInPolygon(4, [x,p1_x,p2_x,p3_x], [z,p1_z,p2_z,p3_z],i,j)
            print ("points being checked" , i,j, in_polygon)
            
            total_count += 1
            if in_polygon == True :
                visible_points.append((i,j))
                true_count += 1

    print ("points inside = ", true_count)
    print ("total checked = ", total_count)
    
    return visible_points
'''

if __name__ == '__main__': 
    #q = flood_fill(0,0,check_validity)
    #print (len(q))
    #print (q)
    #g = graph(xMin,xMax,zMin,zMax)

    x,z = -0.234,-0.476
    x,z = -12,17
    x,z = 0,0
    #p1_x,p1_z = -0.07890,5.1276
    #p1_x,p1_z = 0,-5.1276
    #p1_x,p1_z = 0.0000001,5
    p1_x, p1_z = 0,5
    p2_x,p2_z = 5,0
    p3_x,p3_z = 5,5

    start_time = time.time()
    #for number_points_to_check in range(0,1):
    #    for directions in range(0,1):
    #print (get_points_in_triangle(x,z,p1_x,p1_z,p2_x,p2_z))
    #print (get_points_in_polygon(x,z,p1_x,p1_z,p2_x,p2_z, p3_x,p3_z))
    print (len(get_visible_points (x,z,1 , 45, 40)))
    #print (len(get_visible_points_old (x,z,0 , 45, 40)))

    end_time = time.time()

    print ("processing time = ", end_time-start_time)

