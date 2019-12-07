class DynamicAgentStrategy:
    NEEDS_EXPLORING_DATA=False
    def __init__(self, start, env_data, reward_points, pathing_graph, libvis):
        self.coord = start
        self.libvis = libvis
        self.pathing_graph = pathing_graph
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
#hey this is god_create_bugs
