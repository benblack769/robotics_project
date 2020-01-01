import coord_math
from coord_math import *

class Follower:
    NEEDS_EXPLORING_DATA = False
    def __init__(self,path,start_coord,fixed_time_step=False):
        self.path = path
        #init_path_idx = closest(path,start_coord)
        self.coord = start_coord
        self.next_point = 0
        self.fixed_time_step = fixed_time_step

    def move(self):
        next_coord = self.path[self.next_point]
        self.next_point = (self.next_point+1)%len(self.path)
        new_coord = self.path[self.next_point]
        if coord_math.distc(next_coord,new_coord) > 5.01:
            print("jumped!!!!!!!!!!!")
        # while distc(next_coord,self.coord) < 1.2:
        #     self.next_point = (self.next_point+1)%len(self.path)
        #     next_coord = self.path[self.next_point]

        return coord_math.sub(new_coord,self.coord)

    def get_coord(self):
        return self.coord

    def moved(self,move_vector):
        self.coord = coord_math.add(self.coord,move_vector)

    def on_guard_sight(self,guard_point):
        # doesn't use seen guards at all
        return

    def on_object_sight(self,object_point):
        #print(object_point)
        # doesn't use seen objects at all
        return
