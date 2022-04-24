#!/usr/bin/env python
import rospy
import math
import heapq
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class Node:

    def __init__(self, i, j, endI, endJ, theta):
        self.i = i
        self.j = j
        self.theta = theta
        self.costToCome = 0.0
        self.costToGo = 2.5 * (math.sqrt((i - endI) ** 2 + (j - endJ) ** 2))
        self.cost = None
        self.neighbours = {}
        self.valid_actions = {}
        self.parent = None

    def __lt__(self, other):
        return self.cost < other.cost


class Graph:
    def __init__(self, start, end, RPM1, RPM2, RADIUS, CLEARANCE):
        self.visited = {}
        self.endI = end.i
        self.endJ = end.j
        self.RPM1 = RPM1
        self.RPM2 = RPM2
        self.RADIUS = RADIUS
        self.CLEARANCE = CLEARANCE

    def getNeighbours(self, currentNode):
        i, j, theta = currentNode.i, currentNode.j, currentNode.theta
        neighbours = {}
        valid_actions = {}
        actions = [[0, self.RPM1], [self.RPM1, 0], [self.RPM1, self.RPM1], [0, self.RPM2], [self.RPM2, 0],
                   [self.RPM2, self.RPM2], [self.RPM1, self.RPM2], [self.RPM2, self.RPM1]]
        for UL, UR in actions:
            x, y, newTheta, distance, lin_vel, ang_vel = self.getNewCoordiantes(i, j, theta, UL, UR)
            if (not self.isOutsideArena(x, y)) and (not self.isAnObstacle(x, y)):
                newNode = Node(x, y, self.endI, self.endJ, newTheta)
                neighbours[newNode] = distance
                valid_actions[newNode] = [lin_vel, ang_vel]
        return neighbours, valid_actions

    def getNewCoordiantes(self, i, j, theta, UL, UR):
        t = 0
        
        # Robot Dimensions
        r = 0.105
        L = 0.16
        dt = 0.1

        UL = 3.14 * (UL / 30)
        UR = 3.14 * (UR / 30)
        ang_vel = (r / L) * (UR - UL)
        lin_vel = 0.5 * r * (UL + UR)
        newI = i
        newJ = j
        newTheta = 3.14 * theta / 180
        D = 0

        while t < 1:
            t = t + dt
            Delta_Xn = 0.5 * r * (UL + UR) * math.cos(newTheta) * dt
            Delta_Yn = 0.5 * r * (UL + UR) * math.sin(newTheta) * dt
            newI += Delta_Xn
            newJ += Delta_Yn
            newTheta += (r / L) * (UR - UL) * dt
            D = D + math.sqrt(math.pow(Delta_Xn, 2) + math.pow(Delta_Yn, 2))
        newTheta = 180 * newTheta / 3.14

        if newTheta > 0:
            newTheta = newTheta % 360
        elif newTheta < 0:
            newTheta = (newTheta + 360) % 360

        newI = self.getRoundedNumber(newI)
        newJ = self.getRoundedNumber(newJ)
        return newI, newJ, newTheta, D, lin_vel, ang_vel


    def getRoundedNumber(self, i):
        i = 50 * i
        i = int(i)
        i = float(i) / 50.0
        return i

    def performAStar(self, start, end):

        if self.isAnObstacle(start.i, start.j) and self.isAnObstacle(end.i, end.j):
            rospy.loginfo("Starting and ending point are inside the obstacle! Check clearances!")
            return

        if self.isAnObstacle(start.i, start.j):
            rospy.loginfo("Starting point is inside the obstacle! Check clearances!")
            return

        if self.isAnObstacle(end.i, end.j):
            rospy.loginfo("Ending point is inside the obstacle! Check clearances!")
            return

        if self.isOutsideArena(start.i, start.j):
            rospy.loginfo("Starting point is outside the arena! Check clearances!")
            return

        if self.isOutsideArena(end.i, end.j):
            rospy.loginfo("Ending point is outside the arena! Check clearances!")
            return

        rospy.loginfo("Finding path...")
        priorityQueue = []
        visited_list = {}
        heapq.heappush(priorityQueue, (start.cost, start))
        while len(priorityQueue):
            currentNode = heapq.heappop(priorityQueue)
            currentNode = currentNode[1]
            if self.isInTargetArea(currentNode.i, currentNode.j):
                self.backTrack(currentNode)
                print("Found a path!")
                print("Distance Required to reach from start to end is:", currentNode.costToCome)
                return True

            if tuple([currentNode.i, currentNode.j]) in visited_list:
                continue
            visited_list[tuple([currentNode.i, currentNode.j])] = True

            currentDistance = currentNode.costToCome
            neighbours, valid_actions = self.getNeighbours(currentNode)
            currentNode.neighbours = neighbours
            currentNode.valid_actions = valid_actions
            for neighbourNode, newDistance in neighbours.items():
                neighbourNode.costToCome = currentDistance + newDistance
                neighbourNode.cost = neighbourNode.costToCome + neighbourNode.costToGo
                neighbourNode.parent = currentNode
                heapq.heappush(priorityQueue, (neighbourNode.cost, neighbourNode))
        print("OH NO!! -Path Not Found.")
        return False

    def isInTargetArea(self, i, j):
        if (i - self.endI) ** 2 + (j - self.endJ) ** 2 - 0.01 <= 0:
            return True
        else:
            return False

    def backTrack(self, child):
        while child != None:
            path.append(child)
            child = child.parent
        return True

    def isAnObstacle(self, x, y):
        if (x < 0) or (x > 10) or (y < 0) or (y > 10): 
            return True

        elif (x-2)**2 + (y-8)**2 - (1+self.CLEARANCE)**2 <= 0:   
            return True

        elif x >= 0.25-self.CLEARANCE and x <= 1.75+self.CLEARANCE and y >= 4.25-self.CLEARANCE and y <= 5.75+self.CLEARANCE: 
            return True

        elif x >= 3.75-self.CLEARANCE and x <= 6.25+self.CLEARANCE and y >= 4.25-self.CLEARANCE and y <= 5.75+self.CLEARANCE:      
            return True

        elif (x-2)**2 + (y-2)**2 - (1+self.CLEARANCE)**2 <= 0:                
            return True

        elif x >= 7.25-self.CLEARANCE and x <= 8.75+self.CLEARANCE and y >= 2-self.CLEARANCE and y <= 4+self.CLEARANCE:      
            return True

        else:
            return False 
        


    def isOutsideArena(self, x, y):
        return True if x < self.CLEARANCE or y < self.CLEARANCE or x > 10 - self.CLEARANCE or y > 10 - self.CLEARANCE else False


def publisher(action):
    msg = Twist()
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    now = rospy.get_rostime()
    duration = rospy.Duration(1)
    end_time = now + duration
    while(rospy.get_rostime() < end_time):
        if not rospy.is_shutdown():
            msg.linear.x = action[0]
            
            msg.angular.z = action[1]
            pub.publish(msg)
    msg.linear.x = 0       
    msg.angular.z = 0
    pub.publish(msg)
    return


if __name__ == '__main__':
    try:
        x1 = float(input("Enter start x: "))
        y1 = float(input("Enter start y: "))
        thetaStart = int(input("Enter start theta: "))
        print("--")

        x2 = float(input("Enter start x: "))
        y2 = float(input("Enter start y: "))
        print("--")
        RPM1 = float(input("Enter the RPM 1: "))
        RPM2 = float(input("Enter the RPM 2: "))

        RADIUS = float(input("Enter the radius of the robot:  "))
        CLEARANCE = float(input("Enter the clearance:  "))
        print("--")

        end = Node(x2, y2, x2, y2, 0)
        start = Node(x1, y1, x2, y2, thetaStart)
        start.costToCome = 0
        robot = Graph(start, end, RPM1, RPM2, RADIUS, CLEARANCE)
        path = []

        if robot.performAStar(start, end):
            path.reverse()
            trail = 0
            while trail<5:
                publisher([0.0, 0.0])
                trail += 1
            for index in range(len(path) - 1):
                node = path[index]
                print((node.i,node.j))
                action = node.valid_actions[path[index + 1]]
                publisher(action)
            trail = 0
            while trail<5:
                publisher([0.0, 0.0])
                trail += 1
    except rospy.ROSInterruptException:
        pass

