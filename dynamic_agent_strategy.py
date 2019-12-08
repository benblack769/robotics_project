import coord_math
import random
from collections import Counter,defaultdict
import tempfile
import heapq
import math

def disc(coord):
    x,y = coord
    x = int(round((x-2)/5))*5+2
    y = int(round((y-2)/5))*5+2
    return x,y

class DynamicAgentStrategy:
    NEEDS_EXPLORING_DATA=False
    def __init__(self, start, env_data, reward_points, path_finder):
        '''
        pathfinder takes in  (point_weights,goals)
        '''
        self.coord = start
        self.libvis = libvis
        self.path_finder = path_finder
        self.guards_seen = []
        self.cost_map = defaultdict(int)

    def move(self):
        return (1,0)#direction of movement

    def get_coord(self):
        return self.coord

    def moved(self,move_vector):
        self.coord = coord_math.add(self.coord,move_vector)

    def on_guard_sight(self,guard_point):
        # check guard position
        self.guards_seen.append((time,guard_point))
        pass

    def on_object_sight(self,object_point):
        # doesn't use seen objects at all
        return
