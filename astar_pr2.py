#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import openravepy
from sympy import *  
from copy import deepcopy 


if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

class Node:
    def __init__(self, point3d):
        self.point = point3d;
        self.parent = None
        self.H = 0
        self.G = 0

    def __eq__(self, other):
        return self.point.x == other.point.x and self.point.y == other.point.y

    def __hash__(self):
        new_hash = hash((self.point.x, self.point.y))
        return new_hash

class Point3D:
    def __init__(self, array3d):
        self.x = array3d[0]
        self.y = array3d[1]
        self.z = array3d[2]

    def __repr__(self):
        return "(%s, %s, %s)"%(self.x, self.y, self.z)

    def __len__(self):
        return 0#len(self.x) + len(self.y) +  len(self.z) 

def checkNode(point1, point2):
    if (abs(point1.point.x-point2.point.x)<0.01)&(abs(point1.point.y-point2.point.y)<0.01):#&(abs(point1.point.z-point2.point.z)<0.01):
        # point1.parent = point2
        return True
    else:
        return False

def check_collision(temp_neibhour, env, robot):
    point = [temp_neibhour.point.x,temp_neibhour.point.y,temp_neibhour.point.z]
    robot.SetActiveDOFValues(point)
    return env.CheckCollision(robot)


# def getNeibhour(currentNode, env, robot, handles):
#     step_size = 0.1
#     possibleMoves = [[0,step_size,0],[0,-step_size,0],[step_size,0,0],[-step_size,0,0]]
#     valid_neibhour = set()
#     for i in possibleMoves:
#         temp_neibhour = Node(Point3D([currentNode.point.x + i[0],currentNode.point.y + i[1],currentNode.point.z + i[2]]))
#         if(check_collision(temp_neibhour, env, robot)):
#             point = [temp_neibhour.point.x,temp_neibhour.point.y,temp_neibhour.point.z]
#             handles.append(env.plot3(points=array(point),pointsize=4.0,colors=array(((1,0,0)))))
            
#         else:   
#             point = [temp_neibhour.point.x,temp_neibhour.point.y,temp_neibhour.point.z]
#             handles.append(env.plot3(points=array(point),pointsize=4.0,colors=array(((0,0,1)))))
#             valid_neibhour.add(temp_neibhour)
  
#     return valid_neibhour

def getNeibhour(currentNode, env, robot, handles):
    step_size = 0.1
    diagonal_ssize = step_size*1.4
    possibleMoves = [[0,step_size,0],[0,-step_size,0],[step_size,0,0],[-step_size,0,0],[step_size,step_size,0],[-step_size,step_size,0],[step_size,-step_size,0],[-step_size,-step_size,0]]
    valid_neibhour = set()
    for i in possibleMoves:
        temp_neibhour = Node(Point3D([currentNode.point.x + i[0],currentNode.point.y + i[1],currentNode.point.z + i[2]]))
        if(check_collision(temp_neibhour, env, robot)):
            point = [temp_neibhour.point.x,temp_neibhour.point.y,temp_neibhour.point.z]
            handles.append(env.plot3(points=array(point),pointsize=4.0,colors=array(((1,0,0)))))
            
        else:   
            point = [temp_neibhour.point.x,temp_neibhour.point.y,temp_neibhour.point.z]
            handles.append(env.plot3(points=array(point),pointsize=4.0,colors=array(((0,0,1)))))
            valid_neibhour.add(temp_neibhour)
  
    return valid_neibhour

def calculate_H(point1,point2):  
    #manhattan
    return abs(point1.point.x-point2.point.x) + abs(point1.point.y-point2.point.y) #+ abs(point1.point.z-point2.point.z)

    #EUCLEDEAN
    #return ((point1.point.x-point2.point.x)**2 + (point1.point.y-point2.point.y)**2 + (point1.point.z-point2.point.z)**2)**0.5

def checkclosedSet(point1, closedSet):
    present = 0
    for point2 in closedSet:
        if (abs(point1.point.x-point2.point.x)<0.01)&(abs(point1.point.y-point2.point.y)<0.01):#&(abs(point1.point.z-point2.point.z)<0.01):
            present = 1

    return present

def checktoPass(point1, closedSet, new_cost):
    present = 1
    #print '.'
    # for point2 in closedSet:
    #     if ((point1.point.x==point2.point.x))&((point1.point.y==point2.point.y))&((point1.point.z==point2.point.z)):
    #         if(new_cost < point2.G):
    #             present = 1
    #         else:
    #             present = 0
    # return present
    for point in closedSet:
        if point == point1:
            if new_cost < point.G:
                present = 1
            else:
                present = 0
    return present


def astar(start, goal, env, robot):
    with env:
        
        startNode =  Node(Point3D(start))
        goalNode  =  Node(Point3D(goal))

        #set instead of list will increase the efficiency
        openSet   =  set() 
        closedSet =  set()

        #initialise:: G ||| H ||| currentNode ||| addtoopenSet  
        startNode.H =  calculate_H(startNode, goalNode)
        startNode.G =  0

        currentNode =  startNode
        openSet.add(currentNode)

        step_cost = 0.05
        count = 0
        while(len(openSet)!=0):
            #get current node, minimum value node from opennode
            currentNode = min(openSet, key=lambda gg:gg.H + gg.G)

            #check if current node is goal
            if(checkNode(currentNode, goalNode)):
                # currentNode.parent = goalNode
                break

            #get valid neibhours
            neibhourSet = set()
            neibhourSet = getNeibhour(currentNode, env, robot, handles)

            openSet.remove(currentNode)
            closedSet.add(currentNode)            
            #point = [currentNode.point.x,currentNode.point.y,currentNode.point.z]
            #handles.append(env.plot3(points=array(point),pointsize=4.0,colors=array(((0,0,0)))))
            
            for neighborNode in neibhourSet:
                #calculate the cost and get the min of openset
                # if(checkclosedSet(neighborNode, closedSet)):
                #     continue
                # tentative_g = currentNode.G + step_cost    
    
                # if len(openSet) == 0:
                #     neighborNode.G = tentative_g
                #     neighborNode.parent = currentNode
                #     neighborNode.H = calculate_H(neighborNode, goalNode)
                #     openSet.add(deepcopy(neighborNode))
                #     print 'check'
                # else:
                #     if not nodeIsOpenSet(neighborNode,openSet):
                #         neighborNode.H = calculate_H(neighborNode, goalNode)
                #         neighborNode.parent = currentNode
                #         if tentative_g < neighborNode.G:
                #             neighborNode.G = tentative_g
                #         openSet.add(deepcopy(neighborNode))
                new_cost = currentNode.G + step_cost
                if(checktoPass(neighborNode, closedSet, new_cost)):
                    neighborNode.G = new_cost
                    neighborNode.H = calculate_H(neighborNode, goalNode)
                    neighborNode.parent = currentNode
                    openSet.add(neighborNode)
                    count = count + 1
                    #print 'count :', count
        #robot.SetActiveDOFValues(start)
        print count
    return currentNode




if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);
    handles =[]

    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
        # startconfig =[-3.4, -1.4, 0*pi]
        goalconfig = [2.6,-1.3,-pi/2]
       
        startconfig = robot.GetActiveDOFValues()
        #### Implement the A* algorithm to compute a path for the robot's base starting from the current configuration of the robot and ending at goalconfig. The robot's base DOF have already been set as active. It may be easier to implement this as a function in a separate file and call it here.
        #temp = robot.GetTransform()
        #temp_coordinates = numpy.asarray(temp[[0,2],[3]], dtype=float)
        #xy_startconfig = [temp_coordinates]
        #xy_goalconfig = numpy.asarray([goalconfig[0:2],0.05], dtype=float)
        #print xy_startconfig, xy_goalconfig
        path = set()
        xy_startconfig =[-3.4, -1.4, .05]
        xy_goalconfig = goalconfig

        startConfig = robot.GetActiveDOFValues()
        # startNode = Node(startConfig
        final = astar(startConfig, goalconfig, env, robot)
        # path_point = set()
        # for point in path:
        #     getattr(item, 'point3d')

       
        # handles.append(env.plot3(points=array((-1.5,-1,0)),pointsize=25.0,colors=array(((0,0,1,0.2)))))
        # #### Draw the X and Y components of the configurations explored by A*
        print final.point
        # final = goalconfig
        final_path = []
        currentNODE = final
        currentNODE.point = [currentNODE.point.x,currentNODE.point.y,-pi/2]#currentNODE.point.z]
        final_path.append(currentNODE.point)
        #print currentNODE.point
        print final_path
        previousNODE = currentNODE 
        point_prev = deepcopy(previousNODE.point)
        print point_prev #.x,previousNODE.point.y]
        print type(point_prev)
        passfor1 = 0
        print '------------------------------------------------------------------------'
        while(True):

            previousNODE = currentNODE 
            point_prev = deepcopy(previousNODE.point)
            print point_prev #.x,previousNODE.point.y]
            print type(point_prev)
            # print point_prev.point.x
            #print point_prev
            currentNODE = currentNODE.parent
            #,math.atan2(currentNODE.point.y,currentNODE.point.x)]
            point_curr = [currentNODE.point.x ,currentNODE.point.y ] #math.atan2(currentNODE.point.y,currentNODE.point.x)]
            point = [currentNODE.point.x,currentNODE.point.y,math.atan2(currentNODE.point.y,currentNODE.point.x)]
            final_path.append(point)
            # print currentNODE.point
            point12 = [currentNODE.point.x,currentNODE.point.y,0.05]
            handles.append(env.plot3(points=array(point12),pointsize=4.0,colors=array(((0,0,0)))))
            if xy_startconfig[0] == currentNODE.point.x and xy_startconfig[1] == currentNODE.point.y:
                print 'done'    
                break
        print final_path

        print '------------------------------------------------------------------------'
        #create openrave trajectory                
        traj = RaveCreateTrajectory(env,'')
        traj.Init(robot.GetActiveConfigurationSpecification())
        # traj.Insert(0,robot.GetActiveDOFValues())
        # print robot.GetActiveDOFValues()
        # print type(robot.GetActiveDOFValues())
        # #traj.Insert(1,goalvalues)
        # planningutils.RetimeActiveDOFTrajectory(traj,robot,hastimestamps=False,maxvelmult=1)
        #print 'duration',traj.GetDuration()        
        
        for step in final_path:
            #a = tuple(step)
            #print a
            #print type(a)
            traj.Insert(0,(step))
            #print step
            #print robot.GetAffineDOF()
        # #    i = 1
        # #    a = tuple(step)
            
        #     # print step
        # #    i += 1
        # # print step
        # # print type(step)
        # # a = numpy.asarray(step)
        # # print a
        # # print type(a)
            

        planningutils.RetimeActiveDOFTrajectory(traj,robot,hastimestamps=False,maxvelmult=1)
        print 'duration',traj.GetDuration()
        print robot.GetActiveDOFValues()
        robot.SetActiveDOFValues(goalconfig)
    robot.GetController().SetPath(traj)
    #robot_final_location = [[1,0,0,2.4],[0,1,0,-1.3],[0,0,-pi/2,0.05],[0,0,0,1]]
    #robot.SetTransform(numpy.array(robot_final_location))
    robot.WaitForController(0)
    print robot.GetActiveDOFValues()
    # with env:
    #     robot.SetActiveDOFValues(goalconfig)
    
        #         def display_path(final_node):
        #  node = final_node
        #  while node is not initialconfig:
        #   Path.append(node)
        #   print "Path : node:",node
        #   astar_path.append(env.plot3(points=array(((node[0],node[1],0.05))),
        #                            pointsize=0.05,
        #                            colors=array(((0,0,0))),
        #                            drawstyle=1))
        #   node = Parent[node]
        
    waitrobot(robot)

    raw_input("Press enter to exit...")

