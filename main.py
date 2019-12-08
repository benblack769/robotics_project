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
from exploring_agent import AgentDecisionMaker
import exploring_agent

def renderPolys(screen,polys):
    black = (0,0,0)
    for poly in polys:
        #print(x1,y1)
        pygame.draw.polygon(screen,black,poly,3)

def renderRewards(screen,rewards):
    for rx,ry in rewards:
        irew = (int(rx),int(ry))
        pygame.draw.circle(screen, (255, 0, 0), irew, 5)

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
    poly_screen.set_alpha(128)

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

def renderPath(screen,point_targets):
    if point_targets:
        prevp = point_targets[0]
        for nextp in point_targets[1:]:
            pygame.draw.line(screen,(255,255,0),(prevp),nextp,2)
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




def main():
    parser = argparse.ArgumentParser(description='run ai enviornmnent')
    parser.add_argument('json_fname', type=str, help='enviornment json file')
    parser.add_argument('-V', '--produce_video', action='store_true',help="produces video of screen")
    parser.add_argument('-D', '--no_display', action='store_true',help="disables drawing to screen")
    args = parser.parse_args()
    print(args.no_display)

    basename = os.path.basename(args.json_fname).split(".")[0]
    img_dir = basename+"_img_dir/"
    video_name = basename+"_vid.mp4"
    if os.path.exists(img_dir):
        shutil.rmtree(img_dir)
    if args.produce_video:
        os.makedirs(img_dir,exist_ok=True)

    #gtsp = GTSP()

    env_values = Struct(**json.load(open(args.json_fname)))

    visibilty_info = json.load(open("enviornments/"+env_values.adjacency_list))

    map_info = Struct(**json.load(open("enviornments/"+env_values.map_fname)))

    print(map_info.blocker_polygons)

    libvis = LibVisibility(map_info.blocker_polygons,map_info.width,map_info.height)

    start_coord = env_values.agent_location
    #path_targets = find_path_points(visibilty_info,gtsp,start_coord,map_info.reward_points)
    #graph_points = visibilty_info['points']
    #path_points = [graph_points[x] for x in path_targets]
    def make_pathfinder():
        def path_finder(pointweights,goals):
            plist = visibilty_info['points']
            for i in range(len(plist)):
                if plist[i] in pointweights:
                    pass
    #agent = Follower(path_points,start_coord)
    agent = AgentDecisionMaker(start_coord,env_values)
    guards = [Follower(map_info.guard_dests,guard_loc) for guard_loc in env_values.guard_locations]
    enviornment = EnviornmentCoordinator(libvis,env_values,agent,guards,map_info.reward_points)

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

        enviornment.step_move()
        if enviornment.game_finished():
            print("result: {}".format(enviornment.game_result()))
            running = False

        # Fill the background with white
        screen.fill((255, 255, 255))

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

        for agent_point in enviornment.agent_points():
            agent_color = (0, 0, 255)
            print("point: ",agent_point)
            pygame.draw.circle(screen, agent_color, intify(agent_point), 5)
            poly = libvis.get_visibilily_polygon(agent_point)
            renderSight(screen,map_info,poly,intify(agent_point),env_values.agent_linesight,agent_color)

        for guard_point in enviornment.guard_points():
            guard_color = (0, 255, 0)
            pygame.draw.circle(screen, guard_color, intify(guard_point), 5)
            poly = libvis.get_visibilily_polygon(guard_point)
            renderSight(screen,map_info,poly,intify(guard_point),env_values.guard_linesight,guard_color)

        renderRewards(screen,enviornment.reward_points())
        #renderSight(screen,map_info,poly)

        draw_exploring_agent(screen,map_info,agent)
        #path_targets = find_path_points(visibilty_info,gtsp,(count,count),map_info.reward_points)
        #renderPath(screen,visibilty_info,path_targets)

        #pygame.draw.line(screen, (0, 0, 255), (250, 250),  (250, 0),3)

        # Flip the display
        if not args.no_display:
            pygame.display.flip()

        SAMPLE_RATE = 3
        if args.produce_video and frame_count % SAMPLE_RATE == 0:
            pygame.image.save(screen, img_dir+"data{0:05d}.png".format(frame_count//SAMPLE_RATE))
        frame_count += 1


    if args.produce_video:
        save_video(img_dir,video_name)
    # Done! Time to quit.
    pygame.quit()

if __name__=="__main__":
    main()
