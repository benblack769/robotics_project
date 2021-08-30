from html.parser import HTMLParser
import pygame
import argparse
import json
import time
import os
import shutil
import subprocess
from visibility import LibVisibility
#from parse_svg import SimpleSVGParser
import coord_math
from enviornment import EnviornmentCoordinator
from follow_path_strategy import Follower
from static_pathing import dikstras
from struct_ import Struct
from gtsp import GTSP,get_path
import numpy as np
import random
from collections import Counter

def renderPolys(screen,polys):
    black = (0,0,0)
    for poly in polys:
        #print(x1,y1)
        pygame.draw.polygon(screen,black,poly,3)

def renderRewards(screen,rewards):
    for (rx,ry),prob in rewards:
        irew = (int(rx),int(ry))
        print( int(prob*255))
        pygame.draw.circle(screen, (255, 0, 0, int(prob*255)), irew, 5)

def negative_circle(cen,map_info,radius):
    poly = []
    poly.append((map_info.width-1,0))
    for point in coord_math.get_rays(cen,29,radius):
        poly.append(point)
    poly.append(coord_math.add((radius,0),cen))
    poly.append((map_info.width-1,0))
    poly.append((map_info.width-1,map_info.height-1))
    poly.append((0,map_info.height-1))
    poly.append((0,0))
    return poly

def renderSight(screen,map_info,poly,cen,radius,color):
    poly_screen = pygame.Surface((map_info.width, map_info.height), pygame.SRCALPHA)  # the size of your rect
    poly_screen.set_alpha(16)

    if poly:
        pygame.draw.polygon(poly_screen,color+(128,),poly)

    #pygame.draw.circle(poly_screen, color, cen, radius)

    blank = (0,0,0,0)
    #if poly:
        #pygame.draw.polygon(screen,color,poly,3)
    #    pygame.draw.polygon(poly_screen,blank,poly)
    neg_circle = negative_circle(cen,map_info,radius)
    pygame.draw.polygon(poly_screen,blank,neg_circle)
    #pygame.draw.circle(poly_screen,blank, cen, radius)

    screen.blit(poly_screen, (0,0))

def renderPossiblePaths(map_info,point_targets,color):
    path_screen = pygame.Surface((map_info.width, map_info.height), pygame.SRCALPHA)  # the size of your rect
    for targs in point_targets:
        path_other = pygame.Surface((map_info.width, map_info.height), pygame.SRCALPHA)  # the size of your rect
        renderPath(path_other,targs,color)
        path_screen.blit(path_other,(0,0))
    return path_screen

def renderPath(screen,point_targets,color=(255,255,0)):
    if point_targets:
        prevp = point_targets[0]
        for nextp in point_targets[1:]:
            pygame.draw.line(screen,color,(prevp),nextp,2)
            prevp = nextp

def intify(coord):
    x,y = coord
    return int(x),int(y)

def discritize(width,height,space):
    points = []
    for x in range(space//2,width,space):
        for y in range(space//2,height,space):
            points.append((x,y))
    return points

def find_path_points(visibilty_info, gtsp,start, goals):
    plist = visibilty_info["points"]
    adj_list = visibilty_info["adj_list"]
    counts = visibilty_info["counts"]
    start_idx = coord_math.closest(plist,start)
    goals_idxs = [coord_math.closest(plist,g) for g in goals]
    ordering = get_path(gtsp,plist,adj_list,counts,start_idx,goals_idxs,5000000)
    old_goal = start_idx
    resulting_path = []
    #print(ordering)
    print("ordering")
    print(ordering)
    for new_goal_idx in ordering:
        new_goal = goals_idxs[new_goal_idx]
        new_path = dikstras(old_goal,counts,plist,adj_list,[new_goal])
        resulting_path += new_path
        old_goal = new_goal
    #print(resulting_path)
    return resulting_path



def save_video(img_dir,out_name):
    ffmpeg_call = [
        "ffmpeg",
        "-y",# overwrite output.mp4 if already there
        "-hide_banner","-loglevel","error", #don't print out unnecessary stuff
        "-pattern_type", "glob","-i","{}data*.png".format(img_dir),# get input from image list
        "-c:v","libx264",#deine output format
        "-r","30", #define output sample rate
        "-pix_fmt","yuv420p",#???
        out_name
    ]
    subprocess.call(ffmpeg_call)
    shutil.rmtree(img_dir)

def draw_density_map(screen,map_info,weightmap,pointlist,index,color,radius):
    accel_surface = pygame.Surface((map_info.width, map_info.height),pygame.SRCALPHA)#,pygame.RLEACCEL)
    for i in range(len(weightmap)):
        path_other = pygame.Surface((map_info.width, map_info.height),pygame.RLEACCEL)  # the size of your rect
        path_other.set_alpha(2)
        point = pointlist[weightmap[i][index]]
        pygame.draw.circle(path_other, color, point, radius)
        accel_surface.blit(path_other,(0,0))
    screen.blit(accel_surface,(0,0))


def sample_path(start,weightmap,adj_list):
    path = [start]
    point = start
    #print(point)
    #proint()
    for t in range(len(weightmap)):
        vals = np.asarray(adj_list[point])
        probs = np.asarray(weightmap[t][point][:len(vals)])
        #print(len(vals))
        #print(len(probs))
        next_point = np.random.choice(a=vals,p=probs)
        next_point = int(next_point)
        path.append(next_point)
        point = next_point
    return path

def sample_path_points(visibilty_info, start_coord,path_mixture):
    plist = visibilty_info['points']
    chosen_path = random.choice(path_mixture)
    res = [plist[c] for c in chosen_path]
    #start_idx = coord_math.closest(plist,start_coord)
    #path_targets = find_path_points(visibilty_info,gtsp,start_coord,map_info.reward_points)
    #path_targets = sample_path(start_idx,weightmap,visibilty_info["adj_list"])
    #graph_points = visibilty_info['points']
    #path_points = [graph_points[x] for x in path_targets]
    return res

def to_point_paths(visibilty_info,paths):
    plist = visibilty_info['points']
    return [[plist[p] for p in path] for path in paths]

def main():
    parser = argparse.ArgumentParser(description='run ai enviornmnent')
    parser.add_argument('json_fname', type=str, help='enviornment json file')
    parser.add_argument('--weightmap-num', required=True, type=int, help='enviornment json file')
    parser.add_argument('--weightmap-dir', default="wm_img_dir/enviornments", type=str, help='output director of c++ program')
    parser.add_argument('-V', '--produce_video', action='store_true',help="produces video of screen")
    parser.add_argument('-D', '--no_display', action='store_true',help="disables drawing to screen")
    parser.add_argument('-N', '--display_num', action='store_true',help="disables drawing to screen")
    args = parser.parse_args()
    print(args.no_display)

    basename = os.path.basename(args.json_fname).split(".")[0]
    img_dir = basename+f"_img_dir_{args.weightmap_num}/"
    video_name = basename+f"_vid.{args.weightmap_num}.mp4"
    if os.path.exists(img_dir):
        shutil.rmtree(img_dir)
    if args.produce_video:
        os.makedirs(img_dir,exist_ok=True)

    #gtsp = GTSP()

    env_values = Struct(**json.load(open(args.json_fname)))

    visibilty_info = json.load(open(env_values.adjacency_list))

    map_info = Struct(**json.load(open(env_values.map_fname)))

    agent_weightmap_path = env_values.agent_weightmap.format(dir=args.weightmap_dir,player="agent",number=args.weightmap_num)
    guard_weightmap_path = env_values.agent_weightmap.format(dir=args.weightmap_dir,player="guard",number=args.weightmap_num)

    agent_weightmap = json.load(open(agent_weightmap_path))
    guard_weightmap = json.load(open(guard_weightmap_path))

    plist = visibilty_info['points']
    start_coord = env_values.agent_location
    start_idx = coord_math.closest(plist,start_coord)
    for path in agent_weightmap:
        cur_idx = start_idx
        for idx in path:
            if coord_math.cdist(plist[idx],plist[cur_idx]) > 5.001:
                print("failed assert")
                print(plist[idx])
                print(plist[cur_idx])
                print(idx)
                print(cur_idx)
                #exit(0)
                break
            cur_idx = idx

    #agent_screen_out = renderPossiblePaths(map_info,to_point_paths(visibilty_info,agent_weightmap),(0,255,0,4))
    #guard_screen_out = renderPossiblePaths(map_info,to_point_paths(visibilty_info,guard_weightmap),(0,0,255,4))
    #print(map_info.blocker_polygons)
    #pygame.image.save(agent_screen_out,"guard_density_maps.png")
    #pygame.image.save(guard_screen_out,"agent_density_maps.png")

    libvis = LibVisibility(map_info.blocker_polygons,map_info.width,map_info.height)

    NUM_ENVS = 20
    all_envs = []
    for _ in range(NUM_ENVS):
        start_coord = env_values.guard_locations
        path_points = sample_path_points(visibilty_info,start_coord,agent_weightmap)
        guard_path_points = sample_path_points(visibilty_info,env_values.agent_location,guard_weightmap)
        #print(path_points)
        agent = Follower(path_points,start_coord,True)
        guards = [Follower(guard_path_points,env_values.guard_locations,True) ]
        enviornment = EnviornmentCoordinator(libvis,env_values,agent,guards,map_info.reward_points)
        all_envs.append(enviornment)

    pygame.init()

    if not args.no_display:
        screen = pygame.display.set_mode([map_info.width, map_info.height])
    else:
        screen = pygame.Surface((map_info.width, map_info.height), pygame.SRCALPHA)

    running = True
    count = 1
    frame_count = 0
    while running:
        # Did the user click the window close button?
        if not args.no_display:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

        if not args.no_display:
            time.sleep(0.05)

        # Fill the background with white
        screen.fill((255, 255, 255))

        #screen.blit(guard_screen_out, (0,0))
        #screen.blit(agent_screen_out, (0,0))
        #draw_density_map(screen,map_info,agent_weightmap,visibilty_info['points'],frame_count,(0,0,255),10)
        #draw_density_map(screen,map_info,agent_weightmap,visibilty_info['points'],frame_count,(0,255,0),env_values.guard_linesight)

        renderPolys(screen,map_info.blocker_polygons)
        # Draw a solid blue circle in the center
        count += 1
        #poly = libvis.get_visibilily_polygon((count, count))
        #print(poly)
        #time.sleep(0.01)
        # counts = visibilty_info['counts']
        # avg_value = sum(counts,0)/len(counts)
        # for point,value in zip(visibilty_info['points'],visibilty_info['counts']):
        #     #print(value)
        #     pygame.draw.circle(screen, (0, 255, 0,128), point, int(value/(0.8*avg_value)))

        for env in all_envs:
            for agent_point in env.agent_points():
                agent_color = (0, 0, 255)
                #print("point: ",agent_point)
                pygame.draw.circle(screen, agent_color, intify(agent_point), 5)
                poly = libvis.get_visibilily_polygon(agent_point)
                renderSight(screen,map_info,poly,intify(agent_point),env_values.reward_collect_radius,agent_color)
                #poly = libvis.get_visibilily_polygon(agent_point)
                #renderSight(screen,map_info,poly,intify(agent_point),env_values.agent_linesight,agent_color)

        for env in all_envs:
            for guard_point in env.guard_points():
                guard_color = (0, 255, 0)
                pygame.draw.circle(screen, guard_color, intify(guard_point), 5)
                poly = libvis.get_visibilily_polygon(guard_point)
                renderSight(screen,map_info,poly,intify(guard_point),env_values.guard_linesight,guard_color)

        rew_points = sum(([tuple(p) for p in env.reward_points()] for env in all_envs),[])
        rew_point_counts = Counter(rew_points)
        rew_point_probs = [(point,count/NUM_ENVS) for point,count in rew_point_counts.items()]
        renderRewards(screen,rew_point_probs)
        #renderSight(screen,map_info,poly)

        #path_targets = find_path_points(visibilty_info,gtsp,(count,count),map_info.reward_points)
        #renderPath(screen,path_points)

        #pygame.draw.line(screen, (0, 0, 255), (250, 250),  (250, 0),3)

        # Flip the display

        if not args.no_display:
            pygame.display.flip()
        SAMPLE_RATE = 1
        if args.produce_video and frame_count % SAMPLE_RATE == 0:
            pygame.image.save(screen, img_dir+"data{0:05d}.png".format(frame_count//SAMPLE_RATE))
        frame_count += 1


        new_all_envs = []
        for env in all_envs:
            env.step_move()
            if not env.game_finished():
                new_all_envs.append(env)
        all_envs = new_all_envs
        if len(all_envs) == 0:
            break

        if frame_count > len(agent_weightmap[0]):
            break

    if args.produce_video:
        save_video(img_dir,video_name)
    # Done! Time to quit.
    pygame.quit()

if __name__=="__main__":
    main()
