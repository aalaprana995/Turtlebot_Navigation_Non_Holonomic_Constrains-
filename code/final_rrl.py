# -*- coding: utf-8 -*-
"""
Created on Sun Apr 28 00:33:10 2019

@author: Aalap
"""


# -*- coding: utf-8 -*
"""
Created on Thu Mar 28 18:47:25 2019

@author: Aalap
"""

import numpy as np
import matplotlib.pyplot as plt
import math

class Node:
    def __init__(self, nodex, nodey,nodetheta, cost, parentnode,vx,vy,vt):
        self.nodex = nodex
        self.nodey = nodey
        self.nodetheta=nodetheta
        self.cost = cost
        self.parentnode = parentnode
        self.vx=vx
        self.vy=vy
        self.vt=vt
    def get_nodex(self):
        return self.nodex
    def get_nodey(self):
        return self.nodey
    def get_nodetheta(self):
        return self.nodetheta
    def get_vx(self):
        return vx
    def get_vy(self):
        return vy
    def get_vt(self):
        return vt

def motion(current_node,ur,ul,time):
    r=3.8
    l=23
    ur=0.104666667*ur
    ul=0.104666667*ul
    
    
    thetadot=(r/l)*(ur-ul)
    newnodetheta=thetadot*time+current_node.nodetheta
    xdot=(r/2)*(ur+ul)*(math.cos(current_node.nodetheta))
    ydot=(r/2)*(ur+ul)*(math.sin(current_node.nodetheta))
    d=math.sqrt((ydot)**2+(xdot)**2)
    #delta_x=d*math.cos(newnodetheta)
    #delta_y=d*math.sin(newnodetheta)
    cost=math.sqrt((xdot*time)**2+(ydot*time)**2)
    newcost=round(cost+current_node.cost)
    newnodex=round(xdot*time+current_node.nodex)
    newnodey=round(ydot*time+current_node.nodey)
    xvelocity=(ur)
    yvelocity=(ul)
    thetavelocity=thetadot
    newnodex,newnodey,newnodetheta,newcost,xvelocity,yvelocity,thetavelocity
    

    return newnodex,newnodey,newnodetheta,newcost,xvelocity,yvelocity,thetavelocity





def shortest_path(goalnode, visited, reso):
    #shortest path found until parent id is -1
    path_x = []#stroes path x coordinates
    path_y = []#stroes path x coordinates
    xvelocity = []
    yvelocity = []
    thetavelocity =[]
    path_x.append((goalnode.nodex))
    path_y.append((goalnode.nodey))
    xvelocity.append((goalnode.vx))
    yvelocity.append((goalnode.vy))
    thetavelocity.append((goalnode.vt))
    p = goalnode.parentnode
    
    print(p)
    while (p != -1):
        print('lll')
        tracknode = visited[p]
        path_x.append((tracknode.nodex))
        path_y.append((tracknode.nodey))
        xvelocity.append((tracknode.vx))
        yvelocity.append((tracknode.vy))
        thetavelocity.append((tracknode.vt))
        p = tracknode.parentnode
    return path_x, path_y,xvelocity,yvelocity,thetavelocity

def node_key(node):
    node_key = (node.nodex) * 250 + node.nodey#unique key generation by equation
    return node_key

def hd(node,goalnode):
    d=math.sqrt((node.nodex-goalnode.nodex)**2+(node.nodey-goalnode.nodey)**2)#cost to go
    return d                

def check_node(node,obsmap,obs_x,obs_y):
    #check of node correctness
    if (node.nodex < (min(obs_x)) or node.nodex > (max(obs_x)) or node.nodey < (min(obs_y)) or node.nodey > (max(obs_y))):
        return False
    if (obsmap[node.nodex][node.nodey]):
        return False
    if (node.nodex < 0):
        return False
    if (node.nodex > 1110):
        return False
    if (node.nodey < 0):
        return False
    if (node.nodey > 1011):
        return False
    return True

def check_goal_node(node,goalnode):
    d=math.sqrt((node.nodex-goalnode.nodex)**2+(node.nodey-goalnode.nodey)**2)
    
    if(d<10):
        #check goalnode reached
        return True

def obstacle_map(obs_x, obs_y):
    max_x = round(max(obs_x))
    max_y = round(max(obs_y))
    min_x = round(min(obs_x))
    min_y = round(min(obs_y))

    obsmap = np.zeros((1111,1011))#make a world space which is all false   
    for i in range(min_x,max_x):
        for j in range(min_y,max_y):
            obsmap[i][j]=False#make a obstacle space that is all false
    for index,i in enumerate(obs_x):
        obsmap[obs_x[index]][obs_y[index]] = True#update the obstacle space at points where there is obstacle to true
    return obsmap

def obstacle_space(r,c):
    points=[]#stores points of obstacle space
    obs_x=[]#stores x coordinates of obstacle space
    obs_y=[]#stores y coordinates of obstacle space
    e=r+c
    
    ##circular obstacle space
    print("computing circle1 obstacle")
    k = 40.5 + (r) + c
    for i in range(e,(1111-e)):
        for j in range(e,(1011-e)):
            if (((i - 390) ** 2 + (j - 45) ** 2 - (k ** 2)) <= 0):
                obs_x.append(i)
                obs_y.append(j)
                points.append([i,j])
    print("circle1 obstacle computed")
    #print("c1x",obs_x)
    #print("c1y",obs_y)                
    
    print("computing circle2 obstacle")
    k = 40.5 + (r) + c
    for i in range(e,(1111-e)):
        for j in range(e,(1011-e)):
            if (((i - 438) ** 2 + (j - 274) ** 2 - (k ** 2)) <= 0):
                obs_x.append(i)
                obs_y.append(j)
                points.append([i,j])
    print("circle2 obstacle computed")
    #print("c2x",obs_x)
    #print("c2y",obs_y)                
    
    print("computing circle3 obstacle")
    k = 40.5 + (r) + c
    for i in range(e,(1111-e)):
        for j in range(e,(1011-e)):
            if (((i - 438) ** 2 + (j - 736) ** 2 - (k ** 2)) <= 0):
                obs_x.append(i)
                obs_y.append(j)
                points.append([i,j])
    print("circle3 obstacle computed")
    #print("c3x",obs_x)
    #print("c3y",obs_y)                
    
    
    print("computing circle4 obstacle")
    k = 40.5 + (r) + c
    for i in range(e,(1111-e)):
        for j in range(e,(1011-e)):
            if (((i - 390) ** 2 + (j - 965) ** 2 - (k ** 2)) <= 0):
                obs_x.append(i)
                obs_y.append(j)
                points.append([i,j])
    print("circle4 obstacle computed")
    #print("c4x",obs_x)
    #print("c4y",obs_y)                
    
    print("computing rectangle1 obstacle")
    for i in range(e,1111-e):
        for j in range(e,1011-e):
            if ((i - 1110-r-c <= 0) & (j - 35+r+c >= 0) & (j - 111-r-c <= 0) &(i -927+r+c >= 0)):
                obs_x.append(i)
                obs_y.append(j)
                points.append([i, j])
    print("computed rectangle1 obstacle")
    
    print("computing rectangle2 obstacle")
    for i in range(e,1111-e):
        for j in range(e,1011-e):
            if ((i - 896-r-c <= 0) & (j - 35+r+c >= 0) & (j - 93-r-c <= 0) &(i -779+r+c >= 0)):
                obs_x.append(i)
                obs_y.append(j)
                points.append([i, j])
    print("computed rectangle2 obstacle")
    
    print("computing rectangle3 obstacle")
    for i in range(e,1111-e):
        for j in range(e,1011-e):
            if ((i - 748-r-c <= 0) & (j - 35+r+c >= 0) & (j - 187-r-c <= 0) &(i -474+r+c >= 0)):
                obs_x.append(i)
                obs_y.append(j)
                points.append([i, j])
    print("computed rectangle3 obstacle")
    
    print("computing rectangle4 obstacle")
    for i in range(e,1111-e):
        for j in range(e,1011-e):
            if ((i - 1110-r-c <= 0) & (j - 621+r+c >= 0) & (j - 697-r-c <= 0) &(i -744+r+c >= 0)):
                obs_x.append(i)
                obs_y.append(j)
                points.append([i, j])
    print("computed rectangle4 obstacle")
    
    print("computing rectangle5 obstacle")
    for i in range(e,1111-e):
        for j in range(e,1011-e):
            if ((i - 1110-r-c <= 0) & (j - 448.5+r+c >= 0) & (j - 565.5-r-c <= 0) &(i -1052+r+c >= 0)):
                obs_x.append(i)
                obs_y.append(j)
                points.append([i, j])
    print("computed rectangle5 obstacle")
    
    print("computing rectangle6 obstacle")
    for i in range(e,1111-e):
        for j in range(e,1011-e):
            if ((i - 1110-r-c <= 0) & (j - 362.5+r+c >= 0) & (j - 448.5-r-c <= 0) &(i -1019+r+c >= 0)):
                obs_x.append(i)
                obs_y.append(j)
                points.append([i, j])
    print("computed rectangle6 obstacle")
    
    print("computing rectangle7 obstacle")
    for i in range(e,1111-e):
        for j in range(e,1011-e):
            if ((i - 1110-r-c <= 0) & (j - 178.25+r+c >= 0) & (j - 295.25-r-c <= 0) &(i -1052+r+c >= 0)):
                obs_x.append(i)
                obs_y.append(j)
                points.append([i, j])
    print("computed rectangle7 obstacle")
    
    print("computing rectangle8 obstacle")
    for i in range(e,1111-e):
        for j in range(e,1011-e):
            if ((i - 529-r-c <= 0) & (j - 314.5+r+c >= 0) & (j - 497.5-r-c <= 0) &(i -438+r+c >= 0)):
                obs_x.append(i)
                obs_y.append(j)
                points.append([i, j])
    print("computed rectangle8 obstacle")
    
    print("computing rectangle9 obstacle")
    for i in range(e,1111-e):
        for j in range(e,1011-e):
            if ((i - 712-r-c <= 0) & (j - 256+r+c >= 0) & (j - 332-r-c <= 0) &(i -529+r+c >= 0)):
                obs_x.append(i)
                obs_y.append(j)
                points.append([i, j])
    print("computed rectangle9 obstacle")
    
    print("computing rectangle10 obstacle")
    for i in range(e,1111-e):
        for j in range(e,1011-e):
            if ((i -1026 -r-c <= 0) & (j -919+r+c >= 0) & (j - 1010-r-c <= 0) &(i -983+r+c >= 0)):
                obs_x.append(i)
                obs_y.append(j)
                points.append([i, j])
    print("computed rectangle10 obstacle")
    
    print("computing rectangle11 obstacle")
    for i in range(e,1111-e):
        for j in range(e,1011-e):
            if ((i -918 -r-c <= 0) & (j -827+r+c >= 0) & (j - 1010-r-c <= 0) &(i -832+r+c >= 0)):
                obs_x.append(i)
                obs_y.append(j)
                points.append([i, j])
    print("computed rectangle11 obstacle")
    
    print("computing rectangle12 obstacle")
    for i in range(e,1111-e):
        for j in range(e,1011-e):
            if ((i -1110 -r-c <= 0) & (j -0+r+c >= 0) & (j - 58-r-c <= 0) &(i -585+r+c >= 0)):
                obs_x.append(i)
                obs_y.append(j)
                points.append([i, j])
    print("computed rectangle12 obstacle")
    
    
    print("computing rectangle13 obstacle")
    for i in range(e,1111-e):
        for j in range(e,1011-e):
            if ((i -936 -r-c <= 0) & (j -267+r+c >= 0) & (j - 384-r-c <= 0) &(i -784+r+c >= 0)):
                obs_x.append(i)
                obs_y.append(j)
                points.append([i, j])
    print("computed rectangle13 obstacle")
    
    
    
    
    print("computing rectangle14 obstacle")
    for i in range(e,1111-e):
        for j in range(e,1011-e):
            if ((i -309 -r-c <= 0) & (j -750+r+c >= 0) & (j - 910-r-c <= 0) &(i -150+r+c >= 0)):
                obs_x.append(i)
                obs_y.append(j)
                points.append([i, j])
    print("computed rectangle14 obstacle")
    
    #semi circle
    print("computing semicircle5 obstacle")
    k = 80 + (r) + c
    for i in range(e,(1111-e)):
        for j in range(e,(1011-e)):
            if (((i - 150) ** 2 + (j - 830) ** 2 - (k ** 2)) <= 0):
                obs_x.append(i)
                obs_y.append(j)
                points.append([i,j])
    print("semicircle5 obstacle computed")
    
    print("computing semicircle6 obstacle")
    k = 80 + (r) + c
    for i in range(e,(1111-e)):
        for j in range(e,(1011-e)):
            if (((i - 310) ** 2 + (j - 830) ** 2 - (k ** 2)) <= 0):
                obs_x.append(i)
                obs_y.append(j)
                points.append([i,j])
    print("semicircle6 obstacle computed")
    #boundary obstacle space
    print("computing boundary ")
    if(r==0 and c==0):
        for i in range(1111):
            for j in range(1011):
                if(i==0 or i==1110 or j==1010 or j==0):
                    obs_x.append(i)
                    obs_y.append(j)
                    points.append([i,j])
    else:
        
        e=r+c
        for i in range(e,1111-e):
            for j in range(e,1011-e):
                if(i==r+c or i==1110-r-c or j==1010-r-c or j==r+c):
                    obs_x.append(i)
                    obs_y.append(j)
                    points.append([i,j])
    print("boundary computed")
    print(min(obs_x))
    print(max(obs_x))
    print(min(obs_y))
    print(max(obs_y))                
    return obs_x,obs_y
    

def a_algo(startx,starty,starttheta,goalx,goaly,goaltheta,reso,r,c,time):
    show=True
    lx = []#used to store all explored node x
    ly = []#used to store all explored node y
    flag=0
    unvisited=dict()#dictionary to storedunvisited node
    visited=dict()#dictionary to stored visited node for back tracking
    moves = [[60, 0], [40, 0], [60, 40], [40, 60], [60, 60], [40, 40],
             [0,60], [0, 40]]#all possible moves allowed

    startnode = Node(round(startx / reso), round(starty / reso), 0,0, -1,0,0,0)#start node formation
    goalnode = Node(round(goalx / reso), round(goaly / reso), 0,1000, 0,0,0,0)#goal node formation
    obs_x, obs_y = obstacle_space(r, c)#obstacle space fromed 
    #obstacle space in discretized formate
    obs_x = [round(x / reso) for x in obs_x]
    obs_y = [round(y / reso) for y in obs_y]
    #obstacle space converted to true false obstacle map 
    obsmap= obstacle_map(obs_x,obs_y)
    #checking if the startnode or  goalnode  is not in obstacle or out of world space
    if not(startnode.nodex < min(obs_x) or startnode.nodex > max(obs_x) or startnode.nodey < min(obs_y) or startnode.nodey > max(obs_y)):
        if not(goalnode.nodex < min(obs_x) or goalnode.nodex > max(obs_x) or goalnode.nodey < min(obs_y) or goalnode.nodey > max(obs_y)):
            if not obsmap[startnode.nodex][startnode.nodey] and not obsmap[goalnode.nodex][goalnode.nodey]:
                flag = 1
            
    unvisited[node_key(startnode)] = startnode
    while (flag):
        current_node_id = min(unvisited, key=lambda o: unvisited[o].cost+hd(goalnode,unvisited[o]))#finding minimum cost node
        current_node = unvisited[current_node_id]#making it the current node
        visited[current_node_id] = current_node#putting current node to visited dictionary
        del unvisited[current_node_id]#removing current node from unvisited dictionary
        for i, _ in enumerate(moves):#node exploration
            newnodex,newnodey,newnodetheta,newcost,xvelocity,yvelocity,thetavelocity = motion(current_node , moves[i][0], moves[i][1],time)
            node=Node(newnodex,newnodey,newnodetheta,newcost,current_node_id,xvelocity,yvelocity,thetavelocity)
            lx.append(Node.get_nodex(node))#used get node to store new nodex in lx
            ly.append(Node.get_nodey(node))#used get node to store new nodey in ly
            
            if (len(lx)%1000==0):
                if(show):
                    plt.plot(lx,ly,".r")
                    plt.plot(obs_x, obs_y,".k")#obstacle space
                    plt.show()
                    plt.grid()
    
            if (check_goal_node(node, goalnode)):
                goalnode.nodex=node.nodex
                goalnode.parentnode=node.parentnode
                goalnode.nodey=node.nodey
                goalnode.cost=node.cost
                goalnode.vt=node.vt
                goalnode.vx=node.vx
                goalnode.vy=node.vy
                goalnode.nodetheta=node.nodetheta
                print(node.parentnode,"sdaadsas")
                
                flag=False
                break
            f = node_key(node)
            if not check_node(node, obsmap,obs_x,obs_y):#check the new node is not in obstacle
                continue
            if f in visited:#check new node in visited
                continue
            if f in unvisited:#check node in unvisited and update the parameters
                if (unvisited[f].cost > node.cost):
                    unvisited[f].cost = node.cost
                    unvisited[f].parentnode = node.parentnode
            else:
                unvisited[f] = node#add new node to unvisited dictionary
    print(visited)    
    a, b,xvelocity,yvelocity,thetavelocity = shortest_path(goalnode, visited, reso)#return shortest path
    
    if(flag):
        print("shortest path aaya")
    else:
        print("end")        
    return a, b, obs_x, obs_y, lx,ly,xvelocity,yvelocity,thetavelocity

            
                
                
            

    






def main():
    print( "astar algorithm start!!")
    show=True#flag used to display the result
    
    startx = 50.0  # startx coordinate
    starty = 50.0  # starty coordinate
    starttheta=0
    goalx = 250.0  # goalx coordinate
    goaly = 250.0  # goaly coordinate
    goaltheta=0
    reso =  1 # resolution
    r = 24  #robot radius
    c= 0# clearance
    time=1
    if show:
        plt.plot(startx/reso, starty/reso, "xc")
        plt.plot(goalx/reso, goaly/reso, "xb")
    a,b, obs_x, obs_y, lx,ly,xvelocity,yvelocity,thetavelocity =a_algo(startx,starty,starttheta,goalx,goaly,goaltheta,reso,r,c,time)
    print(a)
    print(b)
    print(xvelocity)
    print(yvelocity)
    print(thetavelocity)
   
    
    
    
    if show:
#displaying the result
#if input or output is incorrect then only obstacle and start and goal is displayed        
        print("final output for astar!!!!")
        plt.plot(lx,ly,".g")#node explored
        plt.plot(obs_x, obs_y,".k")#obstacle space
        plt.plot(a, b, "-r")#shortest path
        plt.grid()
        plt.show()
        
        
        
        
        
        
        
if __name__ == '__main__':
    main()# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

