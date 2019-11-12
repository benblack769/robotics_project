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

STATIC_HIDDEN = "H"
STATIC_BLOCKED = "B"
STATIC_REWARD = "R"
STATIC_OPEN = "O"

def get_sight_locs(rad,cen,static_map):
    num_rays = int(rad*3)
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
                if static_map[(rx,ry)] == STATIC_HIDDEN or static_map[(rx,ry)] == STATIC_REWARD:
                    res.append((sx,sy))
                    break

    return res

def format_travel_edges(edges_costs):
    res_lines = []
    for (idx1,idx2),cost in edges_costs:
        res_lines.append("{} {} {}".format(idx1,idx2,cost))
    return "\n".join(res_lines)


def format_travel_idxs(travel_idxs):
    return "\n".join([str(idx) for idx in travel_idxs])

def format_file(edges,travel_idxs):
    return "{}\n{}\n{}\n{}\n".format(
        len(travel_idxs),
        len(edges),
        format_travel_edges(edges),
        format_travel_edges(edges)
    )

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

class AgentDecisionMaker:
    def __init__(self,env_data):
        #self.map = dict()
        self.guard_linesight = env_data['guard_linesight']
        self.agent_linesight = env_data['agent_linesight']
        self.reward_collect_radius = env_data['reward_collect_radius']
        self.x=0
        self.y=0
        self.guard_sight_counts = Counter()
        self.sight_counts = Counter()
        self.static_map = defaultdict(lambda:STATIC_HIDDEN)

    def ipos(self):
        return (int(self.x),int(self.y))

    def update_clear(self,all_clear_posses,end_pos):
        rx,ry = end_pos
        for spos in coord_math.raytrace2dgrid(self.x,self.y,rx,ry):
            if self.static_map[spos] != STATIC_BLOCKED:
                self.static_map[spos] = STATIC_OPEN
                all_clear_posses.add(spos)

    def get_travel_costs(self,cur_guard_sight):
        static_map,sight_counts,guard_sight_counts = self.static_map,self.sight_counts,self.guard_sight_counts
        #index_map = {pos : idx for idx,(pos,val) in enumerate(static_map.items()) if val == STATIC_OPEN}
        #edges = []
        #travel_idxs = [index_map[pos] for pos in should_travel]
        travel_costs = dict()
        for idx,(pos,val) in enumerate(static_map.items()):
            rpos = pos
            guard_sight_val = 1000000000 if pos in cur_guard_sight else 0
            sight_density = guard_sight_counts[rpos]/max(0.00001,sight_counts[rpos])
            travel_cost = 1.0/max(0.00001,(1-math.sqrt(sight_density)))
            travel_costs[pos] = travel_cost + guard_sight_val
        return travel_costs

    def format_sights(self,guard_sight_posses):
        res  = [str(len(self.static_map))]
        for pos in self.static_map:
            res.append("{} {} {} {} {} {}".format(
                pos[0],
                pos[1],
                self.static_map[pos],
                self.sight_counts[pos],
                self.guard_sight_counts[pos],
                1 if pos in guard_sight_posses else 0,
            ))
        return "\n".join(res)

    def update_maps(self,pointcloud):
        all_clear_posses = set()
        guard_sight_posses = set()
        cur_guard_locs = []
        for obj_type,point_offset in pointcloud:
            xoff,yoff = point_offset
            pos = coord_math.add(point_offset,(self.x,self.y))
            pos = (int(pos[0]),int(pos[0]))
            self.update_clear(all_clear_posses,pos)

        for obj_type,point_offset in pointcloud:
            xoff,yoff = point_offset
            pos = coord_math.add(point_offset,(self.x,self.y))
            pos = (int(pos[0]),int(pos[0]))
            if obj_type == "OBSTICAL":
                self.static_map[pos] = STATIC_BLOCKED
            elif obj_type == "GUARD":
                guard_sight = get_sight_locs(self.guard_linesight+3,pos,self.static_map)
                guard_sight_posses |= guard_sight
            elif obj_type == "REWARD":
                self.static_map[pos] = STATIC_REWARD

        for sight_pos in all_clear_posses:
            self.sight_counts[sight_pos] += 1
        for guard_pos in guard_sight_posses:
            self.guard_sight_counts[guard_pos] += 1

        should_travel = get_unseen_neighboring_open(self.static_map)
        travel_costs = self.get_travel_costs(guard_sight_posses)

        path_to_goal = Dikstras(self.ipos(),travel_costs,should_travel)
        #print(len(self.static_map))
        if path_to_goal is not None and len(path_to_goal)> 1:
            return path_to_goal[1],guard_sight_posses
        else:
            return None,None
        #edges,travel_idxs = self.get_travel_costs(should_travel,guard_sight_posses)

        #with tempfile.NamedTemporaryFile() as tmpfile:
        #    fname = tmpfile.name
        #    print(self.format_sights(guard_sight_posses))
        #    tmpfile.write(self.format_sights(guard_sight_posses))
        #    exit(0)

    def on_sight(self,pointcloud):
        map_goal,guard_sight_posses = self.update_maps(pointcloud)
        mx,my = map_goal if map_goal is not None else (0,0)
        cx,cy = mx-self.x,my-self.y

        xweight = random.random()*2-1 + cx*100
        yweight = random.random()*2-1 + cy*100

        for obj_type,point_offset in pointcloud:
            xoff = point_offset[0] #- self.x
            yoff = point_offset[1] #- self.y
            sqrdist = (xoff*xoff + yoff*yoff)
            weight = 10.0/(0.0001+sqrdist)
            #print(point_offset)
            if obj_type == "OBSTICAL":
                xweight -= weight*xoff
                yweight -= weight*yoff
            if obj_type == "GUARD":
                xweight -= 1000*weight*xoff
                yweight -= 1000*weight*yoff
                #print("hithere")
            if obj_type == "REWARD":
                xweight += 10*xoff*weight
                yweight += 10*yoff*weight

        return xweight,yweight


    def on_move(self,delta_pos,delta_reward):
        self.x += delta_pos[0]
        self.y += delta_pos[1]
