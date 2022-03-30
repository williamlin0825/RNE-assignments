from asyncio.windows_events import NULL
import cv2
import sys
sys.path.append("..")
import PathPlanning.utils as utils
from PathPlanning.planner import Planner

class PlannerAStar(Planner):
    def __init__(self, m, inter=10):
        super().__init__(m)
        self.inter = inter
        self.initialize()

    def initialize(self):
        self.queue = []
        self.parent = {}
        self.h = {} # Distance from start to node
        self.g = {} # Distance from node to goal
        self.goal_node = None

    def planning(self, start=(100,200), goal=(375,520), inter=None, img=None):
        if inter is None:
            inter = self.inter
        start = (int(start[0]), int(start[1]))
        goal = (int(goal[0]), int(goal[1]))
        # Initialize 
        self.initialize()
        self.queue.append(start)
        self.parent[start] = None
        self.g[start] = 0
        self.h[start] = utils.distance(start, goal)
        while(1):
            for x in range(-1, 2):
                for y in range(-1, 2):
                    temp_location = (int(start[0] - inter * x), int(start[1]- inter * y))
                    if temp_location not in self.queue and temp_location != start:
                        line = utils.Bresenham(start[0], temp_location[0], start[1], temp_location[1])
                        collision = False
                        for pts in line:
                            if self.map[int(pts[1]),int(pts[0])]<0.5:
                                collision = True
                        if not collision:
                            self.g[temp_location] = 10 + self.g[start]
                            self.h[temp_location] = utils.distance(temp_location, goal)
                            self.parent[temp_location] = start
                            self.queue.append(temp_location)
            self.g[start] = 9999
            self.h[start] = 9999
            self.queue.sort(key = lambda s: self.g[s] + self.h[s])
            start = self.queue[0]
            

            if img is not None:
                cv2.circle(img, temp_location, 1, (255, 0, 0), 1)
                # Draw Image
                img_ = cv2.flip(img,0)
                cv2.imshow("Path Planning",img_)
                k = cv2.waitKey(1)
                if k == 27:
                    break

            if utils.distance(start, goal) <= inter:
                self.goal_node = start
                break

        # Extract path
        path = []
        p = self.goal_node
        if p is None:
            return path
        while(True):
            path.insert(0,p)
            if self.parent[p] is None:
                break
            p = self.parent[p]
        if path[-1] != goal:
            path.append(goal)
        return path
