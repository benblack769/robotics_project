import coord_math
import random
from collections import Counter,defaultdict
import tempfile
import heapq
import math
import time
from exploring_agent import PriorityQueue

def disc(coord):
    x,y = coord
    x = int(x)#int(round((x-2)/5))*5+2
    y = int(y)#int(round((y-2)/5))*5+2
    return x,y

STATIC_BLOCKED = "B"
STATIC_OPEN = "O"

def sqr(x):
    return x * x

COORDS_AROUND = ((-1,0),(1,0),(1,-1),(-1,-1),(1,1),(1,-1),(0,1),(0,-1))

def get_sight_locs(rad,cen,static_map):
    num_rays = int(rad*6)
    ax,ay = cen
    sight_locs = set()
    for rx,ry in coord_math.get_rays(cen,num_rays,rad):
        sight_coords = coord_math.raytrace2dgrid(ax,ay,rx,ry)
        for sx,sy in sight_coords:
            if static_map[disc((sx,sy))] != STATIC_OPEN:
                break
            sight_locs.add((sx,sy))

    return (sight_locs)

def make_static_map(libvis):
    map = defaultdict(lambda: STATIC_OPEN)
    for x in range(700):
        for y in range(700):
            outpoint = disc((x,y))
            if map[outpoint] == STATIC_OPEN:
                map[outpoint] = STATIC_OPEN if libvis.in_env(outpoint) else STATIC_BLOCKED
                if map[outpoint] is STATIC_BLOCKED:
                    for p in COORDS_AROUND:
                        bp = coord_math.add(p,outpoint)
                        map[bp] = STATIC_BLOCKED
    return map


def Dikstras(start,travel_cost_fn,static_map,should_travel):
    travel_heap = PriorityQueue()
    visited_parents = dict()#{start:0.0}
    travel_heap.push((0.0,start,None))
    res_coord = None
    while not travel_heap.empty():
        val,coord,parent  = travel_heap.pop()
        if coord not in visited_parents:
            visited_parents[coord] = parent
            if coord in should_travel:
                res_coord = coord
                break

            cx,cy = coord
            for ux,uy in ((-1,0),(1,0),(1,-1),(-1,-1),(1,1),(1,-1),(0,1),(0,-1)):
                newc = cx+ux,cy+uy
                mag = coord_math.dist(0,0,ux,uy)
                if newc not in visited_parents and newc in static_map and static_map[newc] == STATIC_OPEN:
                    travel_cost = travel_cost_fn(newc)*mag
                    travel_heap.push((val+travel_cost,newc,coord))

    if res_coord is None:
        return None
    else:
        rpath = []
        finc = res_coord
        while finc is not None:
            rpath.append(finc)
            finc = visited_parents[finc]
        return list(reversed(rpath))


class DynamicAgent:
    NEEDS_EXPLORING_DATA = False
    def __init__(self,start_loc,env_data,libvis,reward_points):
        self.guard_linesight = env_data.guard_linesight
        self.x,self.y = start_loc
        self.timestep = 1
        self.reward_points = [disc(p) for p in reward_points]
        self.static_map = make_static_map(libvis)
        self.timed_guard_sight_posses = []
        self.guard_sight_counts = Counter()
        self.current_path = []

    def get_coord(self):
        return (self.x,self.y)

    def ipos(self):
        return disc(self.get_coord())

    def update_guard_sight(self):
        new_guard_sight = []
        for time,point in self.timed_guard_sight_posses:
            if time < 5:
                new_guard_sight.append((time+1,point))
            if time == 0:
                print("found guard!!")
                for spos in get_sight_locs(self.guard_linesight+5,point,self.static_map):
                    self.guard_sight_counts[spos] += 1

        self.timed_guard_sight_posses = new_guard_sight

    def get_current_guard_sight(self):
        guard_sightings = set()
        for time,sight_pos in self.timed_guard_sight_posses:
            sight_rad = self.guard_linesight+10
            guard_sightings |= get_sight_locs(sight_rad,sight_pos,self.static_map)
        return guard_sightings

    def make_travel_cost(self,cur_guard_sight):
        guard_sight_counts = self.guard_sight_counts
        def travel_cost(pos):
            rpos = pos
            guard_sight_val = 1000000000 if pos in cur_guard_sight else 0
            sight_density = guard_sight_counts[rpos]#/self.timestep
            travel_cost = sqr(sqr(sight_density+0.1))
            return travel_cost + guard_sight_val
        return travel_cost

    def on_object_sight(self,obj_point,is_block):
        pass

    def on_guard_sight(self,guard_point):
        guard_point = disc(guard_point)
        self.timed_guard_sight_posses.append((0,guard_point))

    def on_reward_sight(self,reward_loc):
        # don't need this information
        pass


    def on_reward_collected(self,reward_loc):
        reward_loc = disc(reward_loc)
        if reward_loc in self.reward_points:
            self.reward_points.remove(reward_loc)

    def moved(self,new_coord):
        self.x,self.y = coord_math.add(self.get_coord(),new_coord)
        self.timestep += 1
        self.update_guard_sight()

    def move(self):
        cur_guard_sightings = self.get_current_guard_sight()
        should_travel = list(self.reward_points)
        travel_cost_fn = self.make_travel_cost(cur_guard_sightings)
        start = time.time()
        path_to_goal = Dikstras(self.ipos(),travel_cost_fn,self.static_map,should_travel)
        end = time.time() - start
        print(end)
        #print(len(self.static_map))
        #print(path_targets)
        self.current_path = path_to_goal
        if path_to_goal is not None and len(path_to_goal)> 1:
            next_goal = path_to_goal[1]
            return coord_math.sub(next_goal,self.get_coord())
        else:
            return (0.1,-0.1)# just some random direction
