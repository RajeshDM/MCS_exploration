
#import constants

testing = 0

#max_abs = 5/constants.AGENT_STEP_SIZE
if testing != 1 :
    AGENT_STEP_SIZE = 0.1
    max_abs = 5/AGENT_STEP_SIZE
    move_step_size = 10
else :
    move_step_size = 1
    max_abs =1
x_range = max_abs
z_range = max_abs

xMin = -x_range
xMax = x_range
zMin = -z_range
zMax = z_range

#q = [] 

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
        (x1,y1) = curr_q.pop()
        #print (x1,y1)
        #color(x1,y1)
 
        if (check_validity(x1+move_step_size,y1,q)):
             q.append((x1+move_step_size,y1))
             curr_q.append((x1+move_step_size,y1))
             i += 1
        if (check_validity(x1-move_step_size,y1,q)):
             q.append((x1-move_step_size,y1))
             curr_q.append((x1-move_step_size,y1))
             i += 1
        if (check_validity(x1,y1+move_step_size,q)):
             q.append((x1,y1+move_step_size))
             curr_q.append((x1,y1+move_step_size))
             i += 1
        if (check_validity(x1,y1-move_step_size,q)):
             q.append((x1,y1-move_step_size))
             curr_q.append((x1,y1-move_step_size))
             i += 1
        #print ("i = ", i, x1,y1, len(q))
        #if i > 35 :
        #    break
    return q

#if __main__
if __name__ == '__main__': 
    q = flood_fill(0,0,check_validity)
    print (len(q))
    print (q)
