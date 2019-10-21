import coord_math
import random

def directed_sqr(x):
    return x * abs(x)

class AgentDecisionMaker:
    def __init__(self):
        #self.map = dict()
        self.x=0
        self.y=0
        self.prevx = 0
        self.prevy = 0

    def on_sight(self,pointcloud,coords_seen):
        pointcloud += coords_seen
        xweight = random.random()*2-1
        yweight = random.random()*2-1

        #xweight += directed_sqr(self.x - self.prevx)
        #yweight += directed_sqr(self.y - self.prevy)

        for obj_type,point_offset in pointcloud:
            xoff = point_offset[0] #- self.x
            yoff = point_offset[1] #- self.y
            sqrdist = (xoff*xoff + yoff*yoff)
            weight = 100.0/(0.0001+sqrdist)
            #print(point_offset)
            if obj_type == "OBSTICAL":
                xweight -= weight*xoff
                yweight -= weight*yoff
            if obj_type == "GUARD":
                xweight -= 10000*weight*xoff
                yweight -= 10000*weight*yoff
                print("hithere")
            if obj_type == "REWARD":
                xweight += 10*xoff*weight
                yweight += 10*yoff*weight

        return xweight,yweight


    def on_move(self,delta_pos,delta_reward):
        DECAY_VAL = 0.9
        self.prevx = self.prevx*DECAY_VAL+self.x*(1-DECAY_VAL)
        self.prevy = self.prevy*DECAY_VAL+self.y*(1-DECAY_VAL)
        self.x += delta_pos[0]
        self.y += delta_pos[1]
