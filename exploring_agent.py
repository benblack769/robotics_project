import coord_math
import random
from collections import Counter,defaultdict
import tempfile
import heapq
import math

class PriorityQueue:
    def __init__(self):
        self.q = []
    def push(self,key):
        heapq.heappush(self.q,key)
    def top(self):
        return self.q[0]
    def pop(self):
        return heapq.heappop(self.q)
    def empty(self):
        return len(self.q) == 0

def directed_sqr(x):
    return x * abs(x)

def sqr(x):
    return x * x

STATIC_HIDDEN = "H"
STATIC_BLOCKED = "B"
STATIC_REWARD = "R"
STATIC_OPEN = "O"

def get_sight_locs(rad,cen,static_map):
    num_rays = int(rad*6)
    ax,ay = cen
    sight_locs = set()
    for rx,ry in coord_math.get_rays(cen,num_rays,rad):
        sight_coords = coord_math.raytrace2dgrid(ax,ay,rx,ry)
        for sx,sy in sight_coords:
            if static_map[(sx,sy)] != STATIC_OPEN:
                break
            sight_locs.add((sx,sy))

    return (sight_locs)


def get_unseen_neighboring_open(static_map):
    res = []
    for (sx,sy),val in static_map.items():
        if val == STATIC_OPEN:
            for ux,uy in coord_math.COORDS_AROUND:
                rx,ry = sx+ux,sy+uy
                if (rx,ry) not in static_map or static_map[(rx,ry)] == STATIC_HIDDEN or static_map[(rx,ry)] == STATIC_REWARD:
                    res.append((sx,sy))
                    break

    return res

def Dikstras(start,travel_costs,should_travel):
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
            for ux,uy in coord_math.COORDS_AROUND:
                newc = cx+ux,cy+uy
                if newc not in visited_parents and newc in travel_costs:
                    travel_cost = travel_costs[newc]
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

DISC_SIZE = 1

def disc(coord):
    x,y = coord
    return int(x),int(y)

class AgentDecisionMaker:
    NEEDS_EXPLORING_DATA = True
    def __init__(self,start_loc,env_data):
        #self.map = dict()
        self.guard_linesight = env_data.guard_linesight
        self.agent_linesight = env_data.agent_linesight
        self.reward_collect_radius = env_data.reward_collect_radius
        self.x,self.y = start_loc
        self.timestep = 1
        self.reward_points = set()
        self.timed_guard_sight_posses = []
        self.current_clear_sightings = []
        self.guard_sight_counts = Counter()
        #self.sight_counts = Counter()
        self.current_path = []
        self.static_map = defaultdict(lambda:STATIC_HIDDEN)

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
                for spos in get_sight_locs(self.guard_linesight+5,point,self.static_map):
                    self.guard_sight_counts[spos] += 1


        self.timed_guard_sight_posses = new_guard_sight

    def get_current_guard_sight(self):
        guard_sightings = set()
        for time,sight_pos in self.timed_guard_sight_posses:
            sight_rad = self.guard_linesight+10
            guard_sightings |= get_sight_locs(sight_rad,sight_pos,self.static_map)
        return guard_sightings

    def get_travel_costs(self,cur_guard_sight):
        static_map,guard_sight_counts = self.static_map,self.guard_sight_counts
        #index_map = {pos : idx for idx,(pos,val) in enumerate(static_map.items()) if val == STATIC_OPEN}
        #edges = []
        #travel_idxs = [index_map[pos] for pos in should_travel]
        travel_costs = dict()
        for idx,(pos,val) in enumerate(static_map.items()):
            if val == STATIC_BLOCKED:
                continue
            rpos = pos
            guard_sight_val = 1000000000 if pos in cur_guard_sight else 0
            sight_density = guard_sight_counts[rpos]#/self.timestep
            travel_cost = sqr(sqr(sight_density+0.1))
            #print(travel_cost)
            travel_costs[pos] = travel_cost + guard_sight_val
        return travel_costs

    def update_static_map(self):
        for rx,ry in self.current_clear_sightings:
            for spos in coord_math.raytrace2dgrid(self.x,self.y,rx,ry):
                if self.static_map[spos] != STATIC_BLOCKED:
                    self.static_map[spos] = STATIC_OPEN
        self.current_clear_sightings = []

    def on_object_sight(self,obj_point,is_block):
        obj_point = disc(obj_point)
        #if coord_math.distc(self.get_coord(),obj_point) < self.agent_linesight-5:
        if is_block:
            self.static_map[obj_point] = STATIC_BLOCKED

        self.current_clear_sightings.append(obj_point)

    def on_guard_sight(self,guard_point):
        guard_point = disc(guard_point)
        self.timed_guard_sight_posses.append((0,guard_point))

    def on_reward_sight(self,reward_loc):
        self.reward_points.add(disc(reward_loc))


    def on_reward_collected(self,reward_loc):
        reward_loc = disc(reward_loc)
        if reward_loc in self.reward_points:
            self.reward_points.remove(reward_loc)

    def moved(self,new_coord):
        self.x,self.y = coord_math.add(self.get_coord(),new_coord)
        self.timestep += 1
        self.update_static_map()
        self.update_guard_sight()

    def move(self):
        cur_guard_sightings = self.get_current_guard_sight()
        should_travel = get_unseen_neighboring_open(self.static_map) + list(self.reward_points)
        travel_costs = self.get_travel_costs(cur_guard_sightings)
        path_to_goal = Dikstras(self.ipos(),travel_costs,should_travel)
        #print(len(self.static_map))
        #print(path_targets)
        self.current_path = path_to_goal
        if path_to_goal is not None and len(path_to_goal)> 1:
            next_goal = path_to_goal[1]
            return coord_math.sub(next_goal,self.get_coord())
        else:
            return (0.1,-0.1)# just some random direction
