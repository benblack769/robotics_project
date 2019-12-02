class AgentStrategy:
    def __init__(self, start, path):
        self.coord = start

    def get_coord(self):
        return self.coord
    def moved(self, movevec):
        pass
    def guard_seen(self,guard_offset):
        pass
    def get_move_vector(self):
        pass
