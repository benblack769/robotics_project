import heapq
import coord_math

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


def dikstras_dists(start,travel_costs,points,adj_list,goals):
    travel_heap = PriorityQueue()
    visited_parents = dict()#{start:0.0}
    travel_heap.push((0.0,start,None))
    res_coord = None
    goal_dists = [10e10]*len(goals)
    goals_hit = 0
    while not travel_heap.empty():
        val,coord,parent  = travel_heap.pop()
        if coord not in visited_parents:
            visited_parents[coord] = parent
            if coord in goals:
                res_coord = coord
                goal_idx = goals.index(coord)
                goal_dists[goal_idx] = val
                goals_hit += 1
                if goals_hit == len(goals):
                    break

            for newc in adj_list[coord]:
                if newc not in visited_parents:
                    trav_dist =  coord_math.distc(points[newc],points[coord])
                    travel_cost = 0.01 + travel_costs[newc] *trav_dist
                    #print(trav_dist)
                    travel_heap.push((val+travel_cost,newc,coord))

    if goals_hit != len(goals):
        return None
    else:
        return goal_dists

def dikstras(start,travel_costs,points,adj_list,goals):
    travel_heap = PriorityQueue()
    visited_parents = dict()#{start:0.0}
    travel_heap.push((0.0,start,None))
    res_coord = None
    while not travel_heap.empty():
        val,coord,parent  = travel_heap.pop()
        if coord not in visited_parents:
            visited_parents[coord] = parent
            if coord in goals:
                res_coord = coord
                break

            for newc in adj_list[coord]:
                if newc not in visited_parents:
                    trav_dist =  coord_math.distc(points[newc],points[coord])
                    travel_cost = 0.01 + travel_costs[newc] *trav_dist
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
