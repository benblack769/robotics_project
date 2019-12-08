import math
import numpy as np

def sign(diff):
    return 1 if diff == 0 else (-1,1)[diff>0]

def sqr(x):
    return x*x

def dist(x1,y1,x2,y2):
    return math.sqrt(sqr(x1-x2)+sqr(y1-y2))

def distc(p1,p2):
    return dist(*p1,*p2)

def closest(points,target):
    min_dist = 10e50
    min_idx = None
    tx,ty = target
    for idx,(px,py) in enumerate(points):
        d = dist(tx,ty,px,py)
        if d < min_dist:
            min_dist = d
            min_idx = idx
    return min_idx


COORDS_AROUND = ((-1,0),(0,-1),(1,0),(0,1))

def neg(p):
    x,y = p
    return (-x,-y)

def scalar_mul(p,m):
    x,y = p
    return (x*m,y*m)

def sub(p1,p2):
    return add(p1,neg(p2))

def add(p1,p2):
    ax,ay = p1
    bx,by = p2
    return (ax+bx,ay+by)

def midpoint(source,dest):
    vector = sub(dest,source)
    half_vec = scalar_mul(vector,0.5)
    midpoint = add(source,half_vec)
    return midpoint

def raytrace2dgrid(x1,y1,x2,y2):
    RANGE_LEN = 100
    range = np.arange(RANGE_LEN)/RANGE_LEN
    xvec = (x2-x1)*range + x1
    yvec = (y2-y1)*range + y1
    pointvec = np.stack([xvec,yvec],axis=1)
    #alts = np.concatenate([pointvec
    #,pointvec+np.asarray([1,0])
    #,pointvec+np.asarray([0,1])
    #,pointvec+np.asarray([0.5,0.5])
    #,pointvec+np.asarray([0.5,-0.5])],axis=0)
    tuples = pointvec.astype(np.int32).tolist()
    res = []
    prev = None
    for p in tuples:
        if prev != p:
            res.append(tuple(p))
            prev = p
    res.pop()
    return res
    '''cx = int(x1)
    cy = int(y1)
    tx = int(x2)
    ty = int(y2)
    dirx = sign(x2-x1)
    diry = sign(y2-y1)
    #print(dirx)
    #print(diry)
    ratio = (y2-y1+1e-7)/(x2-x1+1e-7)
    coord_list = []
    def dist_line(nx,ny):
        return abs((y1)+ratio*(nx-(x1)) - ny)
    while not (cx == tx and cy == ty):
        # if abs(y1+ratio*(cx-x1) - y1)
        if dist_line(cx+dirx,cy) < dist_line(cx,cy+diry):
            cx += dirx
        else:
            cy += diry
        #print((cx,cy))
        coord_list.append((cx,cy))
        #if len(coord_list) > 10:
        #    return

    if len(coord_list):
        coord_list.pop()# remove x2,y2 from list

    return coord_list'''

def get_rays(cen,num_rays,radius):
    rays = []
    ax,ay = cen
    for angle in range(num_rays):
        radians = math.pi * 2 * angle / num_rays
        tx = ax + math.cos(radians)*radius
        ty = ay + math.sin(radians)*radius
        rays.append((tx,ty))

    return rays

if __name__=="__main__":
    print(raytrace2dgrid(1.5,1,3,-2))
    print(raytrace2dgrid(0,0,10,-0.2))
    print(raytrace2dgrid(0,0,0,10))
    #exit(0)
    # tot_list = []
    # for x in range(100000):
    #     tot_list.append(raytrace2dgrid(1,1,10,5)[0])
    # print(tot_list[-3])
