from svg.path import parse_path
from html.parser import HTMLParser
import pygame
import argparse
import json
import time
from visibility import LibVisibility
from parse_svg import SimpleSVGParser
import coord_math
from static_pathing import dikstras

def renderPolys(screen,polys):
    black = (0,0,0)
    for poly in polys:
        #print(x1,y1)
        pygame.draw.polygon(screen,black,poly,3)

def renderRewards(screen,rewards):
    for rx,ry in rewards:
        irew = (int(rx),int(ry))
        pygame.draw.circle(screen, (255, 0, 0), irew, 5)

def renderSight(screen,map_parser,poly):
    poly_screen = pygame.Surface((map_parser.width, map_parser.height), pygame.SRCALPHA)  # the size of your rect
    poly_screen.set_alpha(128)

    if poly:
        pygame.draw.polygon(screen,(255,0,0),poly,3)
        pygame.draw.polygon(poly_screen,(255,0,0,128),poly)

    screen.blit(poly_screen, (0,0))

def renderPath(screen,visibilty_info,path_targets):
    if path_targets:
        point_targets = [visibilty_info["points"][t] for t in path_targets]
        prevp = point_targets[0]
        for nextp in point_targets[1:]:
            pygame.draw.line(screen,(255,255,0),(prevp),nextp,2)
            prevp = nextp


def discritize(width,height,space):
    points = []
    for x in range(space//2,width,space):
        for y in range(space//2,height,space):
            points.append((x,y))
    return points

def find_path_points(visibilty_info, start, goals):
    plist = visibilty_info["points"]
    adj_list = visibilty_info["adj_list"]
    counts = visibilty_info["counts"]
    start_idx = coord_math.closest(plist,start)
    goals_idxs = [coord_math.closest(plist,g) for g in goals]
    resulting_path = dikstras(start_idx,counts,adj_list,goals_idxs)
    return resulting_path



def main():
    parser = argparse.ArgumentParser(description='run ai enviornmnent')
    parser.add_argument('json_fname', type=str, help='enviornment json file')
    args = parser.parse_args()

    env_values = json.load(open(args.json_fname))

    visibilty_info = json.load(open("enviornments/"+env_values['adjacency_list']))

    map_parser = SimpleSVGParser()
    map_parser.feed(open("enviornments/"+env_values['svg_fname']).read())
    print(map_parser.blocker_polygons)

    #adj_list = json.load(open(env_values.adjacency_list))
    #discritized_space_points = discritize(map_parser.width,map_parser.height,5)


    libvis = LibVisibility(map_parser.blocker_polygons,map_parser.width,map_parser.height)
    #print(discritized_space_points)
    #vis_counts = libvis.get_point_visibility_counts(discritized_space_points)

    pygame.init()

    screen = pygame.display.set_mode([map_parser.width, map_parser.height])

    running = True
    count = 1
    while running:
        # Did the user click the window close button?
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Fill the background with white
        screen.fill((255, 255, 255))

        renderPolys(screen,map_parser.blocker_polygons)
        # Draw a solid blue circle in the center
        count += 1
        poly = libvis.get_visibilily_polygon((count, count))
        #print(poly)
        time.sleep(0.01)
        counts = visibilty_info['counts']
        avg_value = sum(counts,0)/len(counts)
        for point,value in zip(visibilty_info['points'],visibilty_info['counts']):
            #print(value)
            pygame.draw.circle(screen, (0, 255, 0,128), point, int(value/avg_value))
        pygame.draw.circle(screen, (0, 0, 255), (count, count), 5)
        renderRewards(screen,map_parser.reward_points)
        renderSight(screen,map_parser,poly)

        path_targets = find_path_points(visibilty_info,(count,count),map_parser.reward_points)
        print(path_targets)
        renderPath(screen,visibilty_info,path_targets)

        #pygame.draw.line(screen, (0, 0, 255), (250, 250),  (250, 0),3)

        # Flip the display
        pygame.display.flip()
        #pygame.image.save(screen, "screenshot.jpeg")


    # Done! Time to quit.
    pygame.quit()

if __name__=="__main__":
    main()
