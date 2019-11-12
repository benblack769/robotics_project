import coord_math
import random

def directed_sqr(x):
    return x * abs(x)

class AgentDecisionMaker:
    def __init__(self,env_data):
        #self.map = dict()
        self.x=0
        self.y=0

    def on_sight(self,pointcloud):
        xweight = random.random()*2-1
        yweight = random.random()*2-1

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
