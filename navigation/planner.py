from visibility_road_map import VisibilityRoadMap,ObstaclePolygon,IncrementalVisibilityRoadMap
import random
import math
import matplotlib.pyplot as plt
from fov import FieldOfView

show_animation = True
random.seed(1)

class Planner:

	# pose is a triplet x,y,theta (heading)
	def __init__(self, pose, obst=[], robot_radius=5, eps=0.05,maxStep=10):
		self.obstacles = obst
		self.radius = robot_radius
		self.agentX = pose[0]
		self.agentY = pose[1]
		self.agentH = pose[2]
		self.epsilon = eps
		self.maxStep = maxStep
		self.roadmap = IncrementalVisibilityRoadMap(self.radius, do_plot=False)


	def addObstacle(self, obst):
		self.obstacles.append(obst)
		self.roadmap.addObstacle(obst)

	
	def closedLoopPlannerFast(self, goal):

		obstacleCnt = -1
		i = 1

		while math.sqrt((self.agentX-goal[0])**2 + (self.agentY-goal[1])**2) > self.epsilon:
			

			# compute full plan given our current knowledge
			if len(self.obstacles) != obstacleCnt:
			#	print("replan")
				obstacleCnt = len(self.obstacles)
				pathX, pathY = self.roadmap.planning(self.agentX, self.agentY, goal[0], goal[1])
				i = 1

			#print(i)
			# execute a small step along that plan by
			# turning to face the first waypoint
			#print ("From one step move, path found by path finder ",pathX,pathY)
			dX = pathX[i]-self.agentX
			dY = pathY[i]-self.agentY
			angleFromAxis = math.atan2(dX, dY)
			
			#taking at most a step of size 0.1
			distToFirstWaypoint = math.sqrt((self.agentX-pathX[i])**2 + (self.agentY-pathY[i])**2)
			stepSize = min(self.maxStep, distToFirstWaypoint)
			
			if distToFirstWaypoint < self.epsilon:
				i+=1

			yield (stepSize, angleFromAxis)

	def closedLoopPlanner(self, goal):

		roadmap = VisibilityRoadMap(self.radius, do_plot=False)
		obstacleCnt = -1
		i = 1

		while math.sqrt((self.agentX-goal[0])**2 + (self.agentY-goal[1])**2) > self.epsilon:
			
			# compute full plan given our current knowledge
			if len(self.obstacles) != obstacleCnt:
			#	print("replan")
				obstacleCnt = len(self.obstacles)
				pathX, pathY = roadmap.planning(self.agentX, self.agentY, goal[0], goal[1], self.obstacles)
				i = 1

			#print(i)
			# execute a small step along that plan by
			# turning to face the first waypoint
			dX = pathX[i]-self.agentX
			dY = pathY[i]-self.agentY
			angleFromAxis = math.atan2(dX, dY)
			
			#taking at most a step of size 0.1
			distToFirstWaypoint = math.sqrt((self.agentX-pathX[i])**2 + (self.agentY-pathY[i])**2)
			stepSize = min(self.maxStep, distToFirstWaypoint)
			
			if distToFirstWaypoint == 0:
				i+=1

			yield (stepSize, angleFromAxis)
			



def genRandomRectangle():
    width = random.randrange(5,50)
    height = random.randrange(5,50)
    botLeftX = random.randrange(1,100)
    botRightX = random.randrange(1,100)
    theta = random.random()*2*math.pi

    x = [random.randrange(1,50)]
    y = [random.randrange(1,50)]

    x.append(x[-1]+width)
    y.append(y[-1])

    x.append(x[-1])
    y.append(y[-1]+height)

    x.append(x[-1]-width)
    y.append(y[-1])

    for i in range(4):
        tx = x[i]*math.cos(theta) - y[i]*math.sin(theta)
        ty = x[i]*math.sin(theta) + y[i]*math.cos(theta)
        x[i] = tx
        y[i] = ty

    return ObstaclePolygon(x,y)

def main():
	print(__file__ + " start!!")


	for i in range(1):
		# start and goal position
		sx, sy = random.randrange(-100,-80), random.randrange(-100,-80)  # [m]
		gx, gy = random.randrange(80,100), random.randrange(80,100)  # [m]
		#sx, sy = -60,-80#random.randrange(-100,-80), random.randrange(-100,-80)  # [m]
		#gx, gy = -60,80#random.randrange(80,100), random.randrange(80,100)  # [m]

		robot_radius = 5.0  # [m]

		cnt = 1
		obstacles=[]
		for i in range(cnt):
			obstacles.append(genRandomRectangle())
		visible = [False]*cnt

		if show_animation:  # pragma: no cover
			plt.xlim((-100, 100))
			plt.ylim((-100, 100))
			plt.plot(sx, sy, "or")
			plt.plot(gx, gy, "ob")
			for ob in obstacles:
				ob.plot()
			plt.axis("equal")
			
			#plt.pause(0.1)

		#create a planner and initalize it with the agent's pose
		plan = Planner( [sx,sy,0], [])


		fov = FieldOfView( [sx,sy,0], 60/180.0*math.pi, obstacles)
			
		for stepSize, heading in plan.closedLoopPlannerFast([gx,gy]):
			
			#needs to be replaced with turning the agent to the appropriate heading in the simulator, then stepping.
			#the resulting agent position / heading should be used to set plan.agent* values.
			plan.agentH = heading
			plan.agentX = plan.agentX + stepSize*math.sin(plan.agentH)
			plan.agentY = plan.agentY + stepSize*math.cos(plan.agentH)

			#any new obstacles that were observed during the step should be added to the planner
			for i in range(len(obstacles)):
				if not visible[i] and obstacles[i].minDistanceToVertex(plan.agentX, plan.agentY) < 30:
					plan.addObstacle(obstacles[i])
					visible[i] = True

			fov.agentX = plan.agentX
			fov.agentY = plan.agentY
			fov.agentH = plan.agentH
			poly = fov.getFoVPolygon(100)
			#print(poly.bounds)# = fov.getFoVPolygon(100)
			

			if show_animation:
				plt.cla()
				plt.xlim((-100, 100))
				plt.ylim((-100, 100))
				plt.gca().set_xlim((-100, 100))
				plt.gca().set_ylim((-100, 100))

				plt.plot(plan.agentX, plan.agentY, "or")
				plt.plot(gx, gy, "ob")
				poly.plot("-r")
			
				for i in range(len(obstacles)):
					if visible[i]:
						obstacles[i].plot("-g")
					else:
						obstacles[i].plot("-k")
				
				plt.axis("equal")
				plt.pause(0.1)

    
    #if show_animation:  # pragma: no cover
    #    plt.plot(rx, ry, "-r")
    #    plt.pause(0.1)
    #    plt.show()


if __name__ == '__main__':
    main()
