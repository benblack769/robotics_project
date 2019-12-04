import argparse
import json
from visibility import LibVisibility
from struct_ import Struct
import math

def discritize(width,height,space):
    points = []
    for x in range(space//2,width,space):
        for y in range(space//2,height,space):
            points.append((x,y))
    return points

def manhattan(p1,p2):
    x1,y1 = p1
    x2,y2 = p2
    return abs(x1-x2)+abs(y1-y2)

def sqr(x):
    return x * x

def euclid(p1,p2):
    x1,y1 = p1
    x2,y2 = p2
    return math.sqrt(sqr(x1-x2)+sqr(y1-y2))

def points_are_adjacent(p1,p2,space):
    return euclid(p1,p2) < space*1.501

def prune_graph(graph,points,space):
    new_graph = []
    for node,(alist,point) in enumerate(zip(graph,points)):
        new_alist = []
        for edge in alist:
            if points_are_adjacent(point,points[edge],space):
                new_alist.append(edge)
        new_graph.append(new_alist)
    return new_graph

def main():
    parser = argparse.ArgumentParser(description='run ai enviornmnent')
    parser.add_argument('json_fname', type=str, help='enviornment json file')
    args = parser.parse_args()

    env_values = json.load(open(args.json_fname))

    SPACE = 5

    map_parser = Struct(**json.load(open("enviornments/"+env_values['map_fname'])))
    print(map_parser.blocker_polygons)
    discritized_space_points = discritize(map_parser.width,map_parser.height,SPACE)

    libvis = LibVisibility(map_parser.blocker_polygons,map_parser.width,map_parser.height)
    #print(discritized_space_points)
    discritized_space_points = libvis.filter_points(discritized_space_points)

    adj_list = libvis.get_point_visibility_graph(discritized_space_points)
    graph_counts = [len(adj) for adj in adj_list]
    pruned_list = prune_graph(adj_list,discritized_space_points,SPACE)
    print(pruned_list)
    fin_dict = {
        "points":discritized_space_points,
        "counts":graph_counts,
        "adj_list":pruned_list,
    }
    old_fname_parts = args.json_fname.split(".")
    print(old_fname_parts)
    new_fname = ".".join(old_fname_parts[:-1])+".graph.json"
    json.dump(fin_dict,open(new_fname,'w'))

if __name__=="__main__":
    main()
