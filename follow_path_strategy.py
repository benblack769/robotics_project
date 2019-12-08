import coord_math
from coord_math import *

class Follower:
    NEEDS_EXPLORING_DATA = False
    def __init__(self,path,start_coord):
        self.path = path
        init_path_idx = closest(path,start_coord)
        self.coord = start_coord
        self.next_point = init_path_idx

    def move(self):
        next_coord = self.path[self.next_point]
        while distc(next_coord,self.coord) < 3:
            self.next_point = (self.next_point+1)%len(self.path)
            next_coord = self.path[self.next_point]

        '''trajectories = get_rays((0,0),19,1.0)
        best_dir = trajectories[closest(trajectories,sub(next_coord,self.coord))]
        print(best_dir)
        print(self.coord)'''
        best_dir = coord_math.sub(next_coord,self.get_coord())
        return best_dir

    def get_coord(self):
        return self.coord

    def moved(self,move_vector):
        self.coord = coord_math.add(self.coord,move_vector)

    def on_guard_sight(self,guard_point):
        # doesn't use seen guards at all
        return

    def on_reward_sight(self,reward_loc):
        # doesn't use reward seen at all
        return

    def on_reward_collected(self,reward_loc):
        # doesn't use reward seen at all
        return

    def on_object_sight(self,object_point,is_block):
        #print(object_point)
        # doesn't use seen objects at all
        return
