
"""
Created on Mon Apr 24 00:57:05 2023

@author: mothish raj satish
"""

import math
import numpy as np
import sys
import time
import pygame



class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        

    

class RRTconnect:

    def __init__(self,start,goal,rand_area,sampleRate = 0.05, iteration=3000):
        self.start = Node(start[0], start[1]) 
        self.end = Node(goal[0], goal[1])  
        self.mid = Node(45,45)
        self.sampleRate = sampleRate   
        self.iteration = iteration                    
        self.start_node_list = [self.start]
        self.end_node_list = [self.end]
        self.mide_node_list = [self.mid]
        self.isS_M=False
        self.isE_M=False
        self.path_coord_SM=[]
        self.path_coord_EM=[]
        self.algorithm()




    def algorithm(self):
        screen_scale = 10                

        pygame.init()
        screen = pygame.display.set_mode((screen_scale*100, screen_scale*100))

        screen.fill((25,25,25))
        pygame.draw.polygon(screen, (100,100,100), ((screen_scale*30, screen_scale*40), (screen_scale*30, screen_scale*20), (screen_scale*10, screen_scale*20) ,(screen_scale*10, screen_scale*40)))
        pygame.draw.circle(screen, (100,100,100), (screen_scale*75, screen_scale*25), screen_scale*10)
        pygame.draw.circle(screen, (100,100,100), (screen_scale*75, screen_scale*75), screen_scale*12)
        pygame.draw.polygon(screen, (100,100,100), ((screen_scale*30, screen_scale*90), (screen_scale*30, screen_scale*70), (screen_scale*10, screen_scale*70) ,(screen_scale*10, screen_scale*90)))


        pygame.draw.circle(screen, (255,255,0), (screen_scale*self.end.x, screen_scale*100-(screen_scale*self.end.y)), 14)
        pygame.draw.circle(screen, (100,255,100), (screen_scale*self.start.x, screen_scale*100-(screen_scale*self.start.y)), 14)
        pygame.draw.circle(screen, (255,100,0), (screen_scale*self.mid.x, screen_scale*100-(screen_scale*self.mid.y)), 14)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        
        iter_count = 0 
        start_time = time.time()
        for i in range(self.iteration):
            time.sleep(0.009)
            iter_count = iter_count+1
            start_node_random = self.getRandomNode() 
            start_node_near = self.getNeighborNode(self.start_node_list,start_node_random)                 
            is_obstacle_start,start_new_node = self.action_set_End(start_node_near, start_node_random)      
            
            if is_obstacle_start == False :
                is_obstacle_start,start_new_node =self.action_set(start_node_near, start_node_random)
                
            if is_obstacle_start:
                self.start_node_list.append(start_new_node)  
                pygame.draw.line(screen, (100,255,100), (screen_scale*start_node_near.x, screen_scale*100 - screen_scale*start_node_near.y), (screen_scale*start_new_node.x, screen_scale*100 - screen_scale*start_new_node.y),3)


            end_random_node = self.getRandomNode() 
            end_near_node = self.getNeighborNode(self.end_node_list,end_random_node)     
            is_obstacle_end,end_new_node = self.action_set_Start(end_near_node, end_random_node) 

          
            if is_obstacle_end == False:     
                is_obstacle_end,end_new_node = self.action_set(end_near_node, end_random_node)     
                
            if is_obstacle_end:
                
                self.end_node_list.append(end_new_node)   
                pygame.draw.line(screen, (255,255,0), (screen_scale*end_near_node.x, screen_scale*100 - screen_scale*end_near_node.y), (screen_scale*end_new_node.x, screen_scale*100 - screen_scale*end_new_node.y),3)
            

            distance_S_E, _ = self.getDistanceAngle(end_new_node,start_new_node)


            

            if distance_S_E < 1.5:
                print("Goal Reached!!")
                print("Time to Find Path with neighbor: ",time.time() - start_time, "seconds")
                print("Iteration Count : ",iter_count)
                
                path_coord = self.backTrack(start_new_node, end_new_node)
                total_path =path_coord
                for point in path_coord:
                    pygame.draw.circle(screen, (128, 0, 128), (screen_scale*point[0], 100*screen_scale-screen_scale*point[1]), 5)
                    pygame.display.update()
                time.sleep(10)                                
                return True
            

                
            pygame.display.update()
    
        return False
                        

    

    def action_set(self, neighborNode, randomNode):
            
        result_node = Node(neighborNode.x, neighborNode.y)
        dist, theta = self.getDistanceAngle(result_node, randomNode) 
        step_size = 1
        min_d = min(dist, step_size)
        t=0
        dt = 0.1  

        while t<min_d:
            t = t + dt
            result_node.x += dt  * math.cos(theta)
            result_node.y += dt * math.sin(theta)
            if self.isObstacle(result_node.x,result_node.y)==False:
                return (False,result_node)

        result_node.parent = neighborNode

        return(True,result_node)
    

    def action_set_End(self, neighborNode, randomNode):
            
        result_node = Node(neighborNode.x, neighborNode.y)
        dist, theta = self.getDistanceAngle(result_node, randomNode)
        
        b,alpha = self.getDistanceAngle(result_node,self.end)
        k=0.4
        
        step_size = 0.09
        min_d = min(dist, step_size)
        t=0
        dt = 0.05

        while t<min_d:
            t = t + dt
            result_node.x += k * math.cos(alpha) + (dt*math.cos(theta)) + ((1-k)*math.sin(alpha))
            result_node.y += k * math.sin(alpha) + (dt*math.sin(theta)) + ((1-k)*math.cos(alpha))
            if self.isObstacle(result_node.x,result_node.y)==False:
                return (False,result_node)

        result_node.parent = neighborNode

        return(True,result_node)
    
    def action_set_Start(self, neighborNode, randomNode):
            
        result_node = Node(neighborNode.x, neighborNode.y)
        dist, theta = self.getDistanceAngle(result_node, randomNode) 
        
        b,alpha = self.getDistanceAngle(result_node,self.start)
        k=0.4

        step_size = 0.09
        min_d = min(dist, step_size)
        t=0
        dt = 0.05   
        
        while t<min_d:
            t = t + dt
            result_node.x += k * math.cos(alpha) + ( dt*math.cos(theta)) + ((1-k)*math.sin(alpha))
            result_node.y += k * math.sin(alpha) + ( dt*math.sin(theta)) + ((1-k)*math.cos(alpha))
            if self.isObstacle(result_node.x,result_node.y)==False:
                return (False,result_node)
            
        result_node.parent = neighborNode

        return(True,result_node)

       
    def backTrack(self,start_new_node, end_new_node):
        path_start = [(start_new_node.x, start_new_node.y)]
        node_now = start_new_node

        while node_now.parent is not None:
            node_now = node_now.parent
            path_start.append((node_now.x, node_now.y))

        path_end = [(end_new_node.x, end_new_node.y)]
        node_now = end_new_node

        while node_now.parent is not None:
            node_now = node_now.parent
            path_end.append((node_now.x, node_now.y))
    
        return (list(list(reversed(path_start)) + path_end))
    


    def getRandomNode(self):
        if  np.random.random()> self.sampleRate:        
            return Node(np.random.randint(0, 100), np.random.randint(0, 100))
        else: 
            return self.end


     
    def getDistanceAngle(self,fromN, toN):
        distance = math.hypot(toN.x - fromN.x, toN.y - fromN.y)
        angle = math.atan2(toN.y - fromN.y,toN.x - fromN.x)      
        return(distance, angle)
    
   

    def getNeighborNode(self,nodeList, randNode):
        distanceList =  [(node.x - randNode.x)**2 + (node.y - randNode.y)**2
                         for node in nodeList]
        minIndex = distanceList.index(min(distanceList))

        return(nodeList[minIndex])
    
    
    def isObstacle(self,x,y):
          
        if (x-75)**2 + (y-25)**2 - (12)**2 <= 0:

            return False

        elif (x-75)**2 + (y-75)**2 - (10)**2 <= 0:
            return False
            
        elif x >= 10 and x <= 30 and y >= 60 and y <=80 :      
            return False
        
        elif x >= 10 and x <= 30 and y >= 10 and y <=30 :      
            return False

        return True    
         



def main():
    rrt = RRTconnect(start = [5,5], goal = [95,95], rand_area=[0, 1000])

   
    
if __name__ == '__main__':
    main() 
    

    
