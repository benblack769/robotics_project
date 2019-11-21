import heapq

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

def dikstras(start,travel_costs,adj_list,goals):
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
