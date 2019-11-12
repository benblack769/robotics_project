from PIL import  Image
import numpy as np
import math
import coord_math


def to_rgb_int(r,g,b):
    return (r*256+g)*256+b

GUARD_PATH_VAL = to_rgb_int(185,122,87)
BLOCK_VAL = to_rgb_int(0,0,0)
REWARD_VAL = to_rgb_int(237,28,36)
BLANK_VAL = to_rgb_int(255,255,255)

class Guard:
    def __init__(self,xstart,ystart):
        self.x = xstart
        self.y = ystart
        self.xprev = self.x
        self.yprev = self.y

    def val_to_move(self,x,y):
        return coord_math.dist(x,y,self.xprev,self.yprev)

    def pos(self):
        return (self.x,self.y)

    def exec_move(self,move):
        GUARD_DECAY_VAL = 0.9
        self.xprev = self.xprev*GUARD_DECAY_VAL + self.x*(1-GUARD_DECAY_VAL)
        self.yprev = self.yprev*GUARD_DECAY_VAL + self.y*(1-GUARD_DECAY_VAL)

        self.x,self.y = move

    def find_move(self, map):
        best_val = -10e50
        best_move = (0,0)
        NUM_MOVE_FINDS = 19
        for angle in range(NUM_MOVE_FINDS):
            theta = 2*math.pi*angle/NUM_MOVE_FINDS
            x = math.cos(theta) + self.x
            y = math.sin(theta) + self.y
            if map[int(y),int(x)] == GUARD_PATH_VAL:
                new_val = self.val_to_move(x,y)
                if best_val < new_val:
                    best_val = new_val
                    best_move = (x,y)

        if best_val < -10e49:
            raise RuntimeError("guard at ({},{}) didn't find a  place to move".format(self.x,self.y))

        return best_move

class Agent:
    def __init__(self,xstart,ystart):
        self.x = xstart
        self.y = ystart
    def pos(self):
        return self.x,self.y

    def recenter_coord(self,coord):
        return (coord[0]-self.x, coord[1]-self.y)


class Enviornment:
    def __init__(self,env_data):
         pix = Image.open(env_data['img_path'])
         pix_vals = np.array(pix).astype(np.uint32)
         self.rgb_vals = to_rgb_int(pix_vals[:,:,0],pix_vals[:,:,1],pix_vals[:,:,2])
         #self.reward_left = np.where(np.equal(self.rgb_vals,REWARD_VAL),True,False)
         self.block_vals = np.where(np.equal(self.rgb_vals,BLOCK_VAL),True,False)


         self.reward_coords = []
         for y in range(len(self.rgb_vals)):
             for x in range(len(self.rgb_vals[0])):
                 if self.rgb_vals[y,x] == REWARD_VAL:
                     self.reward_coords.append((x,y))

         self.guard_linesight = env_data['guard_linesight']
         self.agent_linesight = env_data['agent_linesight']
         self.reward_collect_radius = env_data['reward_collect_radius']
         self.reward_collected = 0
         self.guards = [Guard(x,y) for x,y in env_data['guard_locations']]
         ax,ay = env_data['agent_location']
         self.agent = Agent(ax,ay)

    def rob_coords(self,pos):
        return int(pos[1]),int(pos[0])

    def update_robot_pos(self,pos,move):
        return

    def exec_agent_move(self,agent,dir):
        dir_mag = coord_math.dist(dir[0],dir[1],0,0)
        if dir_mag < 0.000001:
            return
        dir = (dir[0] / dir_mag, dir[1] / dir_mag)

        new_pos = (agent.x+dir[0],agent.y+dir[1])
        #new_pos_int = (int(new_pos[0]),int(new_pos[1]))
        self.update_robot_pos(agent.pos(),new_pos)
        agent.x += dir[0]
        agent.y += dir[1]
        if self.rgb_vals[int(agent.y),int(agent.x)] == BLOCK_VAL:
            raise RuntimeError("agent ran into the wall")

        MIN_DIST_BORDER = self.agent_linesight+3
        map_shape = self.rgb_vals.shape
        if (agent.y < MIN_DIST_BORDER or agent.y >= map_shape[0]-MIN_DIST_BORDER or
            agent.x < MIN_DIST_BORDER or agent.x >= map_shape[1]-MIN_DIST_BORDER):
            raise RuntimeError("agent got too close to border")
        return dir

    def check_game_over(self):
        agent = self.agent
        for guard in self.guards:
            if self.guard_found_agent(agent,guard):
                return "FOUND_AGENT"

        return "GAME_CONTINUED"

    def communicate_line_sight(self,agent):
        NUM_RAYS = self.agent_linesight*8
        NUM_CHECK_ON_RAY = self.agent_linesight*2
        ray_results = []
        for tx,ty in coord_math.get_rays(self.agent.pos(),NUM_RAYS,self.agent_linesight):
            obj,coord = self.find_first_el_blocked(agent.x,agent.y,tx,ty,NUM_CHECK_ON_RAY)
            #print(obj)
            ray_results.append((obj,(coord)))

        ray_results += self.agent_sees_objects()
        recentered = [(obj,self.agent.recenter_coord(coord)) for obj,coord in ray_results]
        return recentered

    def find_first_el_blocked(self,x1,y1,x2,y2,num_check):
        xinc = (x2-x1)/num_check
        yinc = (y2-y1)/num_check
        xidxs = np.arange(0,num_check,dtype=np.float32)*xinc + x1
        yidxs = np.arange(0,num_check,dtype=np.float32)*yinc + y1
        #coords = np.stack([xidxs,yidxs], axis=1)

        #rob_pos = (int(y1),int(x1))
        #orig_val = self.block_vals[rob_pos]
        #self.block_vals[rob_pos] = False
        vals = self.block_vals[yidxs.astype(np.int32),xidxs.astype(np.int32)]
        first_blocked = np.argmax(vals)
        #self.block_vals[rob_pos] = orig_val
        if vals[first_blocked] == False:
            return ("EMPTY",(x2,y2))

        first_coord_blocked = (xidxs[first_blocked],yidxs[first_blocked])
        first_coord_blocked_int = int(yidxs[first_blocked]),int(xidxs[first_blocked])
        is_obstical = self.rgb_vals[first_coord_blocked_int] == BLOCK_VAL

        if is_obstical:
            return ("OBSTICAL",first_coord_blocked)
        '''else:
            if np.any(np.equal(first_coord_blocked_int,self.rob_coords(self.agent.pos()))):
                return "AGENT",first_coord_blocked
            else:
                for guard in self.guards:
                    if np.any(np.equal(first_coord_blocked_int,self.rob_coords(guard.pos()))):
                        return "GUARD",first_coord_blocked'''

        raise RuntimeError("could not find obsitcal type")

    def agent_collect_reward(self):
        agent = self.agent
        reward_collected = 0
        new_reward_l = []
        for x,y in self.reward_coords:
            if self.found_target(agent.x,agent.y,x,y,self.reward_collect_radius):
                reward_collected += 1
            else:
                new_reward_l.append((x,y))

        self.reward_coords = new_reward_l
        self.reward_collected += reward_collected
        return reward_collected

    def agent_sees_objects(self):
        agent = self.agent
        coords_seen = []
        for x,y in self.reward_coords:
            if self.found_target(agent.x,agent.y,x,y,self.agent_linesight):
                coords_seen.append(("REWARD",(x,y)))
        for guard in self.guards:
            if self.found_target(agent.x,agent.y,guard.x,guard.y,self.agent_linesight):
                coords_seen.append(("GUARD",(x,y)))
        return coords_seen

    def found_target(self,sx,sy,tx,ty,max_rad):
        if coord_math.dist(sx,sy,tx,ty) > max_rad:
            return False
        coord_list = coord_math.raytrace2dgrid(sx,sy,tx,ty)
        for x,y in coord_list:
            val = self.rgb_vals[y,x]
            if val == BLOCK_VAL:
                return False
        return True

    def guard_found_agent(self,agent,guard):
        return self.found_target(guard.x,guard.y,agent.x,agent.y,self.guard_linesight)


    def get_img(self):
        #fig_save = self.rgb_vals.copy()
        fig_save = np.where(np.equal(self.rgb_vals,BLOCK_VAL),BLOCK_VAL,BLANK_VAL)

        for rx,ry in self.reward_coords:
            fig_save[ry,rx] = REWARD_VAL

        fig_save[int(self.agent.y),int(self.agent.x)] = to_rgb_int(0,0,255)

        for guard in self.guards:
            fig_save[int(guard.y),int(guard.x)] = to_rgb_int(0,255,0)

        return fig_save
