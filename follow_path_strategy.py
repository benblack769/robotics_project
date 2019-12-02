import coord_math
from coord_math import *

class Follower:
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

        trajectories = get_rays((0,0),19,1.0)
        best_dir = trajectories[closest(trajectories,sub(next_coord,self.coord))]
        print(best_dir)
        print(self.coord)
        return best_dir

    def get_coord(self):
        return self.coord

    def moved(self,new_pos):
        self.coord = new_pos
