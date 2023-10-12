import cv2
import numpy as np
import math
from .import config

class bot_pathplanner():
    def __init__(self) -> None:
        self.dfs=DFS()
        self.dijkstra=dijkstra()
        self.a_star=A_star()
        self.path_to_goal = []
        self.img_shortest_path = []
        self.choosen_route = []

    @staticmethod
    def cords_to_pts(cords):
      return [cord[::-1] for cord in cords]
    
    
    def draw_path_on_maze(self,maze,shortest_path_pts,method):
        
        maze_bgr = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)
        self.choosen_route = np.zeros_like(maze_bgr)

        rang = list(range(0,254,25))
        
        depth = maze.shape[0]
        for i in range(len(shortest_path_pts)-1):
            per_depth = (shortest_path_pts[i][1])/depth

            # Blue : []   [0 1 2 3 251 255 251 3 2 1 0] 0-depthperc-0
            # Green :[]     depthperc
            # Red :  [] 100-depthperc
            color = ( 
                      int(255 * (abs(per_depth+(-1*(per_depth>0.5)))*2) ),
                      int(255 * per_depth),
                      int(255 * (1-per_depth))
                    )
            cv2.line(maze_bgr,shortest_path_pts[i] , shortest_path_pts[i+1], color)
            cv2.line(self.choosen_route,shortest_path_pts[i] , shortest_path_pts[i+1], color,3)

        img_str = "maze (Found Path) [" +method +"]"
        if config.debug and config.debug_pathplanning:
            cv2.namedWindow(img_str,cv2.WINDOW_FREERATIO)
            cv2.imshow(img_str, maze_bgr)

        if method == "dijkstra":
            self.dijkstra.shortest_path_overlayed = maze_bgr
        elif method == "a_star":
            self.a_star.shortest_path_overlayed = maze_bgr
            
        self.img_shortest_path = maze_bgr.copy()

    def find_path_nd_display(self,graph,start,end,maze,method = "DFS"):

        Path_str = "Path"
        
        if method=="DFS":
            paths = self.dfs.get_paths(graph, start, end)
            path_to_display = paths[0]
        elif(method=="DFS_SHORTEST"):
            path_n_costs=self.dfs.get_paths_cost(graph,start,end)
            paths=path_n_costs[0]
            costs=path_n_costs[1]
            min_cost=min(costs)
            path_to_display=paths[costs.index(min_cost)]

        elif (method == "dijkstra"):
            
            if not self.dijkstra.shortest_path_found:
                print("Finding Shortest ROutes")
                self.dijkstra.find_best_routes(graph, start, end)
            
            path_to_display = self.dijkstra.shortest_path
            Path_str = "Shortest "+ Path_str

        elif (method == "a_star"):
            
            if not self.dijkstra.shortest_path_found:
                print("Finding Shortest ROutes")
                self.a_star.find_best_routes(graph,start,end)
            
            path_to_display = self.a_star.shortest_path
            Path_str = "Shortest "+ Path_str


        pathpts_to_display=self.cords_to_pts(path_to_display)
        self.path_to_goal=pathpts_to_display
        print(f"Found path_pts={pathpts_to_display}")
        self.draw_path_on_maze(maze,pathpts_to_display,method)
        # cv2.waitKey(0)

        
class Heap():
    def __init__(self) -> None:
        self.array=[]
        self.vertexpos=[]
        self.size=0

    def new_minHeap_node(self,v,dist):
        return ([v,dist])
    
    def swap_nodes(self,a,b):
        temp=self.array[a]
        self.array[a]=self.array[b]
        self.array[b]=temp
    
    def minHeapify(self,node_idx):
        smallest=node_idx
        left_node=2*node_idx+1
        right_node=2*node_idx+2

        if(left_node<self.size and self.array[left_node][1]<self.array[smallest][1]):
            smallest=left_node
        if(right_node<self.size and self.array[right_node][1]<self.array[smallest][1]):
            smallest=right_node

        if(smallest!=node_idx):
            self.vertexpos[self.array[node_idx][0]]=smallest
            self.vertexpos[self.array[smallest][0]]=node_idx

            self.swap_nodes(smallest,node_idx)

            self.minHeapify(smallest)

    def extract_min(self):
        if self.size==0:
            return
        
        root=self.array[0]
        last_node=self.array[self.size-1]

        self.array[0]=last_node

        self.vertexpos[root[0]]=self.size-1

        self.vertexpos[last_node[0]]=0

        self.size-=1

        self.minHeapify(0)


        return root
    

    def decrease_key(self,vertx,dist):
        idxvertex=self.vertexpos[vertx]
        self.array[idxvertex][1]=dist 

        while(idxvertex>0 and self.array[idxvertex][1]<self.array[(idxvertex-1)//2 ][1]):

            self.vertexpos[self.array[idxvertex][0]]=(idxvertex-1)//2
            self.vertexpos[self.array[(idxvertex-1)//2][0]]=idxvertex

            self.swap_nodes(idxvertex,(idxvertex-1)//2)

            idxvertex=(idxvertex-1)//2

        
    def isinheap(self,vertx):
        if self.vertexpos[vertx]<self.size:
            return True
        return False
    

class dijkstra():

        def __init__(self) -> None:
            self.shortest_path=[]

            self.minheap=Heap()

            self.idx2vrtx={}
            self.vrtx2idx={}

            self.djikstra_nodes_visited=0
            self.shortest_path_found=False
            self.shortest_path_overlayed=[]


        def ret_shortestroute(self,parent,start,end,route):
        
        # Keep updating the shortest route from end to start by visiting closest vertices starting fron end
            route.append(self.idx2vrtx[end])
        
        # Once we have reached the start (maze_entry) => Stop! We found the shortest route
            if (end==start):
                return
        
        # Visit closest vertex to each node
            end = parent[end]
        # Recursively call function with new end point until we reach start
            self.ret_shortestroute(parent, start,end, route)

        def find_best_routes(self,graph,start,end):
            
            start_idx=[idx for idx,key in enumerate(graph.items()) if key[0]==start][0]
        
                
            dist=[]

            parent=[]
            
            self.minheap.size=len(graph.keys())

            for idx,v in enumerate(graph.keys()):
                
                dist.append(1e7)

                self.minheap.array.append(self.minheap.new_minHeap_node(idx,dist[idx]))
                self.minheap.vertexpos.append(idx)

                parent.append(-1)

                self.idx2vrtx[idx]=v
                self.vrtx2idx[v]=idx

            dist[start_idx]=0
            self.minheap.decrease_key(start_idx,dist[start_idx])

            while(self.minheap.size!=0):
                self.djikstra_nodes_visited+=1

                curr_top=self.minheap.extract_min()
                u_idx=curr_top[0]
                u=self.idx2vrtx[u_idx]

                for v in graph[u]:
                    if v!='case':

                        print(f"vertex adjacent to {u} is {v}")
                        v_idx=self.vrtx2idx[v]
                        if(self.minheap.isinheap(v_idx) and dist[u_idx]!=1e7 and dist[u_idx]+graph[u][v]['cost']<dist[v_idx]):
                            dist[v_idx]=dist[u_idx]+graph[u][v]['cost']
                            self.minheap.decrease_key(v_idx,dist[v_idx])
                            parent[v_idx]=u_idx

                if u==end:
                    break

            shortest_path=[]
            self.ret_shortestroute(parent,start_idx,self.vrtx2idx[end],shortest_path)

            self.shortest_path=shortest_path[::-1]
            
            self.shortest_path_found=True


class DFS():


    @staticmethod
    def get_paths(graph,start,end,path=[]):
        path=path+[start]

        if (start==end):
            return path
        if start not in graph.keys():
            return []

        paths=[]            
        for node in graph[start].keys():
            if(node not in path and node!='case'):
                new_paths=DFS.get_paths(graph,node,end,path)
                for p in new_paths:
                    paths.append(p)


        return paths
    
    @staticmethod
    def get_paths_cost(graph,start,end,path=[],cost=0,trav_cost=0):
        path=path+[start]
        cost=cost+trav_cost

        if (start==end):
            return [path],[cost]
        if start not in graph.keys():
            return []

        paths=[]
        costs=[]            
        for node in graph[start].keys():
            if(node not in path and node!="case"):
                new_paths,new_costs=DFS.get_paths_cost(graph,node,end,path,cost,graph[start][node]["cost"])
                for p in new_paths:
                    paths.append(p)
                for c in new_costs:
                    costs.append(c)

        return paths,costs


class A_star(dijkstra):

    def __init__(self) -> None:
        
        super().__init__()
        self.astar_nodes_visited=0
        self.shortest_path_overlayed=[]

    @staticmethod
    def euc_dist(a,b):
        return math.sqrt(pow(a[0]-b[0],2)+pow(a[1]-b[1],2))
    
    def ret_shortestroute(self,parent,start,end,route):
        
        # Keep updating the shortest route from end to start by visiting closest vertices starting fron end
            route.append(self.idx2vrtx[end])
        
        # Once we have reached the start (maze_entry) => Stop! We found the shortest route
            if (end==start):
                return
        
        # Visit closest vertex to each node
            end = parent[end]
        # Recursively call function with new end point until we reach start
            self.ret_shortestroute(parent,start,end, route)
    
    def find_best_routes(self,graph,start,end):

        start_idx=[idx for idx,key in enumerate(graph.items()) if key[0]==start][0]

        cost2node=[]

        parent=[]

        dist=[]
        self.minheap.size=len(graph.keys())
        for idx,v in enumerate(graph.keys()):
            cost2node.append(1e7)
            dist.append(1e7)
            print(f"Vertex:{v}")
            self.minheap.array.append(self.minheap.new_minHeap_node(idx,dist[idx]))
            self.minheap.vertexpos.append(idx)

            parent.append(-1)

            self.vrtx2idx[v]=idx
            self.idx2vrtx[idx]=v

        cost2node[start_idx]=0
        dist[start_idx]=cost2node[start_idx]+self.euc_dist(start,end)
        self.minheap.decrease_key(start_idx,dist[start_idx])
        print(self.minheap.size)


        while(self.minheap.size!=0):
            self.astar_nodes_visited+=1
            print("Chut")
            curr_top=self.minheap.extract_min()
            u_idx=curr_top[0]
            u=self.idx2vrtx[u_idx]

            for v in graph[u]:
                if v!='case':
                    v_idx=self.vrtx2idx[v]
                    print(f"Vertex adjacent to {u} is {v}")
                    if(self.minheap.isinheap(v_idx) and dist[u_idx]!=1e7 and cost2node[u_idx]+graph[u][v]['cost']<cost2node[v_idx]):
                        cost2node[v_idx]=cost2node[u_idx]+graph[u][v]['cost']
                        dist[v_idx]=cost2node[v_idx]+self.euc_dist(v,end)
                        self.minheap.decrease_key(v_idx,dist[v_idx])
                        parent[v_idx]=u_idx

            if (u==end):
                break

            shortest_path=[]

            shortest_path=self.ret_shortestroute(parent,start_idx,self.vrtx2idx[end],shortest_path)

            self.shortest_path=shortest_path[::-1]

            self.shortest_path_found=True

        






    
