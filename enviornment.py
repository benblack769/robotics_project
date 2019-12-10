import coord_math


def find_block_intersect(libvis,source,dest):
    if coord_math.distc(source,dest) < 3:
        return dest
    midpoint = coord_math.midpoint(source,dest)

    if not libvis.can_see(source,midpoint):
        return find_block_intersect(libvis,source,midpoint)
    else:
        return find_block_intersect(libvis,midpoint,dest)

class EnviornmentCoordinator:
    def __init__(self,libvis,env_values,agent,guards,rewards):
        self.libvis = libvis
        self.agent = agent
        self.guards = guards
        self.rewards = rewards
        self.movers = [agent]+guards
        self.env_values = env_values
        self.agent_found = False
        self.reward_collected = 0

    def step_move(self):
        # set line of sight of movers
        agent = self.agent
        for guard in self.guards:
            if (self.libvis.can_see(agent.get_coord(),guard.get_coord()) and
                    coord_math.distc(agent.get_coord(),guard.get_coord()) < self.env_values.agent_linesight):
                agent.on_guard_sight(guard.get_coord())

        if self.agent.NEEDS_EXPLORING_DATA:
            agent_coord = self.agent.get_coord()
            for ray in coord_math.get_rays(agent_coord,47,self.env_values.agent_linesight):
                if not self.libvis.can_see(agent_coord,ray):
                    self.agent.on_object_sight(find_block_intersect(self.libvis,agent_coord,ray))

        # move movers
        for mover in self.movers:
            move_dir = mover.move()
            move_dist = coord_math.distc((0,0),move_dir)
            #if move_dist > 1.0:
            #    move_dir = coord_math.scalar_mul(move_dir,1.0/move_dist)
            #print(move_dir)
            src_coord = mover.get_coord()
            dest_coord = coord_math.add(src_coord,move_dir)
            if not self.libvis.can_see(src_coord,dest_coord):
                print("mover at {} tried to move though wall!".format(src_coord))
                #exit(0)
            mover.moved(move_dir)

        # check if agent is found by guard
        agent_loc = self.agent.get_coord()
        for guard in self.guards:
            guard_loc = guard.get_coord()
            if (coord_math.distc(agent_loc,guard_loc) < self.env_values.guard_linesight and
                    self.libvis.can_see(agent_loc,guard_loc)):
                self.agent_found = True
                #print(agent_loc)
                #print(guard_loc)
                #print(coord_math.distc(agent_loc,guard_loc))

        # check if agent has collected reward, and execute collection
        new_reward_list = []
        for rew_point in self.rewards:
            if (coord_math.distc(agent_loc,rew_point) < self.env_values.reward_collect_radius and
                    self.libvis.can_see(agent_loc,rew_point) and
                    not self.agent_found):
                self.reward_collected += 1
            else:
                new_reward_list.append(rew_point)
        self.rewards = new_reward_list


    def game_finished(self):
        return self.agent_found or not self.rewards

    def game_result(self):
        return self.reward_collected

    def reward_points(self):
        return self.rewards

    def guard_points(self):
        return [g.get_coord() for g in self.guards]

    def agent_points(self):
        return [self.agent.get_coord()]
