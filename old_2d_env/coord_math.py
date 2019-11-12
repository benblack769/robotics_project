import math

def sign(x1,x2):
    diff = x2-x1
    return 0 if diff == 0 else (-1,1)[diff>0]

def sqr(x):
    return x*x

def dist(x1,y1,x2,y2):
    return math.sqrt(sqr(x1-x2)+sqr(y1-y2))

COORDS_AROUND = ((-1,0),(0,-1),(1,0),(0,1))

def add(p1,p2):
    ax,ay = p1
    bx,by = p2
    return (ax+bx,ay+by)

def raytrace2dgrid(x1,y1,x2,y2):
    cx = int(x1)
    cy = int(y1)
    tx = int(x2)
    ty = int(y2)
    dirx = sign(cx,tx)
    diry = sign(cy,ty)
    coord_list = []
    while not (cx == tx and cy == ty):
        if dist(cx+dirx,cy,tx,ty) < dist(cx,cy+diry,tx,ty):
            cx += dirx
        else:
            cy += diry
        coord_list.append((cx,cy))

    if len(coord_list):
        coord_list.pop()# remove x2,y2 from list

    return coord_list

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
    tot_list = []
    for x in range(1000000):
        tot_list.append(raytrace2dgrid(1,1,10,5)[0])
    print(tot_list[-3])
