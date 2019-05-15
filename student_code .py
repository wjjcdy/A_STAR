# -*- coding: UTF-8 -*-
import math
class Pose_node:
    def __init__(self, Node,G,H,P):
        self.map_node = Node
        self._g = G
        self._f = H + G
        self._parent = P

def shortest_path(M,start,goal):
    print("shortest path called")
    init_node = Pose_node(start,0,0,-1)        #初始起始节点
    goal_node = Pose_node(goal,0,0,-1)
    Explored = {}                                #初始已探索集合,集合为字典
    frontier = {init_node.map_node:init_node}    #初始边缘点集合
    while len(frontier):
        curr_node_key = min(frontier, key=lambda x:frontier[x]._f) #从边缘节点中找代价最小的节点key
        curr_node = frontier[curr_node_key]                        #获取最小代价节点
        if curr_node.map_node == goal:                             #最小代价的节点为goal
            goal_node = curr_node
            break

        Explored[curr_node.map_node] = curr_node                   #最小节点应放入已探索集合中
        del frontier[curr_node.map_node]                           #此节点扩展新节点，故也不再是边缘节点                        

        for road in M.roads[curr_node.map_node]:                    #遍历此节点所有链接的节点
            dx = M.intersections[curr_node.map_node][0] - M.intersections[road][0]     
            dy = M.intersections[curr_node.map_node][1] - M.intersections[road][1]
            dis = math.sqrt(dx**2 + dy**2)                         #计算到新链接节点的这一步代价，即距离
            g = curr_node._g + dis                                  #计算从起点到达新节点的累计代价

            dx = M.intersections[goal][0] - M.intersections[road][0]     
            dy = M.intersections[goal][1] - M.intersections[road][1]
            h = math.sqrt(dx**2 + dy**2)                           #计算新链接节点到目标点代价，即启发函数

            next_node = Pose_node(road,g,h,curr_node.map_node)     #构建新边缘节点
            
            if road in Explored:                                   #如果next节点在已经探索集合中，则不处理
                continue

            if road in frontier:                                   #判断新的链接节点是否已经是边缘节点，若是，判断代价是否小于已有累计代价，若是则替换
                if g < frontier[road]._g:
                    frontier[road] = next_node                     #更新边缘节点代价
            else:
                frontier[road] = next_node                         #增加新的边缘节点
    node = goal_node
    path = []
    path.append(goal_node.map_node)
    while  node._parent != -1:                                     #从目标节点开始查找上一节点，直到起始节点
        node = Explored[node._parent]                              #获取上一节点
        path.append(node.map_node)                                 #将节点位置放入路径中
    path.reverse()    
    return path