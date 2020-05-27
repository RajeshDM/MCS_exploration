import constants
import numpy as np
import json

max_abs = 5
x_range = max_abs
z_range = max_abs

xMin = -x_range
xMax = x_range
zMin = -z_range
zMax = z_range

main_data = {"allPoints":[],   "receptaclePivotPoints": [],
  "receptaclePoints": [],
  "screenHeight": constants.SCREEN_HEIGHT,
  "screenWidth": constants.SCREEN_WIDTH} 

#for i in range (xMin,xMax,constants.AGENT_STEP_SIZE):
#    for j in range (zMin,zMax,constants.AGENT_STEP_SIZE):
for i in np.arange(xMin,xMax, constants.AGENT_STEP_SIZE):
    for j in np.arange(zMin,zMax, constants.AGENT_STEP_SIZE):
        object_to_append = {"horizon": 0.0,"openReceptacle": False,"pivotId": 0,"receptacleObjectId": "","rotation": 0.0,
                           "x": i,
                           "y": 1.0,
                           "z": j}
        
        main_data["allPoints"].append(object_to_append) 


with open("transferral_data-layout_" + str(constants.AGENT_STEP_SIZE)+ ".json","w") as fp:   
    #print (len(self.discovered_objects))   
    #json.dump(self.discovered_objects,fp,indent=1)
    json.dump(main_data,fp,indent=1)
    print (len(main_data['allPoints']))
    
