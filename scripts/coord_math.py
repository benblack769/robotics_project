import math

def sign(x1,x2):
    diff = x2-x1
    return 0 if diff == 0 else (-1,1)[diff>0]

def sqr(x):
    return x*x

def dist(x1,y1,x2,y2):
    return math.sqrt(sqr(x1-x2)+sqr(y1-y2))

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

if __name__=="__main__":
    print(raytrace2dgrid(1.5,1,3,-2))
    tot_list = []
    for x in range(1000000):
        tot_list.append(raytrace2dgrid(1,1,10,5)[0])
    print(tot_list[-3])
