#!/usr/bin/env python3

import numpy as np
from dataclasses import dataclass
import queue
from enum import Enum
import json
import rospy
from std_msgs.msg import String




rospy.init_node('putanja_node')
pub = rospy.Publisher('turtlebot_motion',String, queue_size=100)

param_name = '/suma_matrica'
#param_value = "Hello, ROS!"

while not rospy.get_param(param_name, False):
    rospy.loginfo("Waiting for the file to be available...")
    rospy.sleep(1) 

matrica = rospy.get_param(param_name)

matrica_array = np.array(matrica).reshape(36,36)

print(matrica_array)





maze = matrica_array
print("uspelo je")


class Path(Enum):
    gore = 0
    dole = 1
    levo = 2
    desno = 3
    stop = 4
    gripper_zatvori = 5
    gripper_otvori = 6

path_sequence = []
grid_queue0 = queue.Queue()
grid_queue1 = queue.Queue()
orijentacija = "dole" 
stanje = 0 

max_x = maze.shape[0]
max_y =maze.shape[1]

print(max_x,max_y)

def findTargetCenter(target_num, maze):
   
    start_x, end_x, start_y, end_y = None, None, None, None
   
   
    for i in range(max_x): 
        for j in range(max_y ):
            if maze[i][j] == target_num:
                if start_x is None:
                    start_x, start_y = j, i
                end_x, end_y = j, i

   
    if start_x is not None:
        mid_x = (start_x + end_x) // 2
        mid_y = (start_y + end_y) // 2
        return mid_x, mid_y
    else:
        return None

def kretanje_levo(p):
    global orijentacija

    if(p == "levo"):
        if(orijentacija == "dole"):
            p = "r65"
        elif(orijentacija == "levo"):
            p = "f65"
        elif(orijentacija == "gore"):
            p = "l65"
        elif(orijentacija == "desno"):
            p = "p65"
        orijentacija = "levo"
        slanje_komadni(p)

def kretanje_desno(p):

    global orijentacija

    if(p == "desno"):
        if(orijentacija == "dole"):
            p = "l65"
        elif(orijentacija == "levo"):
            p = "p65"
        elif(orijentacija == "gore"):
            p = "r65"
        elif(orijentacija == "desno"):
            p = "f65"
        orijentacija = "desno"
        slanje_komadni(p)


def kretanje_unapred(p):

    global orijentacija
    if(p == "gore"):
        if(orijentacija == "dole"):
            p = "p65"
        elif(orijentacija == "levo"):
            p = "r65"
        elif(orijentacija == "gore"):
            p = "f65"
        elif(orijentacija == "desno"):
            p = "l65"
        orijentacija = "gore"
    slanje_komadni(p)


def kretanje_unazad(p):

    global orijentacija
    if(p == "dole"):
        if(orijentacija == "dole"):
            p = "f65"
        elif(orijentacija == "levo"):
            p = "l65"
        elif(orijentacija == "gore"):
            p = "p65"
        elif(orijentacija == "desno"):
            p = "r65"
        orijentacija = "dole"
        slanje_komadni(p)
    

def zatvaranje_gripper(p):

    global orijentacija
    if(p == "gripper_zatvori"):
        p = "g10"
        slanje_komadni(p)

def otvaranje_gripper(p):

    global orijentacija
    if(p == "gripper_otvori"):
        p = "o15"    
        slanje_komadni(p)
    
def slanje_komadni(p):
    global path_sequence
    global orijentacija

    path_sequence.append(p) 
    with open("/home/uros/Documents/putanja_kretanja.txt", "a") as file_object:
        file_object.write(str(p) + "\n")



def check_neighbours(grid, x,y, trajectory):

    global orijentacija
    global path_sequence
    
    p = Path.stop 
    
    
    if(grid[y][x+1] == grid[y][x] - 1 and grid[y][x+1] != 0): 
        if(grid[y][x] != 1): 
            p = Path.desno
            kretanje_desno(p.name)
            check_neighbours(grid, x+1, y, trajectory)
    elif(grid[y][x-1] == grid[y][x] - 1 and grid[y][x-1] != 0):
        if(grid[y][x] != 1):
            p = Path.levo
            kretanje_levo(p.name)
            check_neighbours(grid, x-1, y, trajectory)
    elif(grid[y-1][x] == grid[y][x] - 1 and grid[y-1][x] != 0):
        if(grid[y][x] != 1):
            p = Path.gore
            kretanje_unapred(p.name)
            check_neighbours(grid, x, y-1,trajectory)   
    elif(grid[y+1][x] == grid[y][x] - 1 and grid[y+1][x] != 0):
        if(grid[y][x] != 1):
            p = Path.dole
            kretanje_unazad(p.name)
            check_neighbours(grid, x, y+1,trajectory)
   
    trajectory[y][x] = 6
    
    return trajectory


def stanje_grippera(stanje):
    if stanje == 0:  
        p = Path.gripper_zatvori
        zatvaranje_gripper(p.name)
        stanje = 1
    else:
        p = Path.gripper_otvori
        otvaranje_gripper(p.name)
        stanje= 0
    return stanje

#MATRICA POTENCIJALA OD ROBOTA DO OBJEKTA

robot_x, robot_y = findTargetCenter(4, maze)
finish_center_x, finish_center_y = findTargetCenter(2, maze)
object_center_x, object_center_y = findTargetCenter(3, maze)
print(robot_x,robot_y)
print(finish_center_x,finish_center_y)
print(object_center_x,object_center_y)

def matrix_potential(active_x,active_y,stop_x,stop_y, grid_queue):
    max_x = maze.shape[0]
    max_y =maze.shape[1]

    visited= np.where((maze == 2)  | (maze == 3) |  (maze ==4), 0, maze)

   
   
    grid = np.where(visited == 0, 1, np.where(visited == 1, 0, visited))
   
    visited[active_x][active_y] = 1
    

    grid_queue.put((active_x,active_y))

    while(active_x != stop_x or active_y != stop_y ): 
    
        active_x,active_y = grid_queue.get() 

        if(active_x-1 > 0 and visited[active_y][active_x-1] == 0): 
            grid[active_y][active_x-1]+=grid[active_y][active_x] 
            visited[active_y][active_x-1] = 1
            grid_queue.put((active_x-1,active_y)) 

        if(active_x+1 < max_x and visited[active_y][active_x+1] == 0): 
            grid[active_y][active_x+1]+=grid[active_y][active_x] 
            visited[active_y][active_x+1] = 1
            grid_queue.put((active_x+1,active_y)) 

        if(active_y-1 > 0 and visited[active_y-1][active_x] == 0): 
            grid[active_y-1][active_x]+=grid[active_y][active_x]
            visited[active_y-1][active_x] = 1
            grid_queue.put((active_x,active_y-1))

        if(active_y+1 < max_y and visited[active_y+1][active_x] == 0):
            grid[active_y+1][active_x]+=grid[active_y][active_x]
            visited[active_y+1][active_x] = 1
            grid_queue.put((active_x,active_y+1))
    return grid

with open("/home/uros/Documents/putanja_kretanja.txt", "w") as file_object: 
    file_object.write("start\n")

grid2 = matrix_potential(object_center_x,object_center_y,robot_x,robot_y,grid_queue0)
#grid1= matrix_potential(finish_center_x,finish_center_y,object_center_x,object_center_y,grid_queue1)
maze = check_neighbours(grid2, robot_x, robot_y, maze)

grip = stanje_grippera(0)

grid1= matrix_potential(finish_center_x,finish_center_y,object_center_x,object_center_y,grid_queue1)
maze = check_neighbours(grid1, object_center_x, object_center_y,maze)
grip = stanje_grippera(1)

with open("/home/uros/Documents/putanja_kretanja.txt", "a") as file_object:
    file_object.write("end\n")



i = len(path_sequence)-1
while(i >= 0):
    pub.publish(str(path_sequence[i]))
    i=i-1
print("salje")
print(path_sequence[i])