from svg.path import parse_path
from html.parser import HTMLParser
import pygame
import argparse
import json
import visilibity as vis

EPSILON = 0.0000001

class SimpleSVGParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self.blocker_lines = []
        self.guard_dests = []
        self.reward_points = []
        self.width = None
        self.height = None

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if tag == "svg":
            self.width = int(attrs['width'])
            self.height = int(attrs['height'])
        elif tag == "line":
            self.blocker_lines.append((
                float(attrs["x1"]),
                float(attrs["y1"]),
                float(attrs["x2"]),
                float(attrs['y2'])
            ))
        elif tag == "rect":
            self.reward_points.append((float(attrs["x"]),float(attrs["y"])))
        elif tag == "path":
            pathstr = attrs['d']
            parsed = parse_path(pathstr)
            pointed = [p.point(0.0) for p in parsed]
            tupled = [(p.real,p.imag) for p in pointed]
            self.guard_dests += tupled

    def handle_endtag(self, tag):
        pass

    def handle_data(self, data):
        pass


def renderLines(screen,lines):
    black = (0,0,0)
    for x1,y1,x2,y2 in lines:
        #print(x1,y1)
        pygame.draw.line(screen,black,(x1,y1),(x2,y2),3)

def renderRewards(screen,rewards):
    pass

def renderSight(screen):
    pass

def points_to_vis_points(tupled_points):
    return [vis.Point(x,y) for x,y in tupled_points]


def vis_poly_to_pointlist(polygon):
    end_poss = []
    #print ('Points of Polygon: ')
    for i in range(polygon.n()):
        x = polygon[i].x()
        y = polygon[i].y()

        end_poss.append((x,y))

        #print( x,y)

    return end_poss
class LibVisibility:
    def __init__(self,blocker_lines):
        self.lines = []
        for x1,y1,x2,y2 in blocker_lines:
            p1 = vis.Point(x1,y1)
            p2 = vis.Point(x2,y2)
            self.lines.append(vis.Polygon([p1,p2]))
        self.environment = vis.Environment(self.lines)

    def get_point_visibility_counts(self,points):
        vis_points = points_to_vis_points(points)
        graph = vis.Visibility_Graph(vis_points,self.environment,1e-12)
        point_counts = [0]*len(points)
        print(len(points))
        print(graph.n())
        #exit(0)
        #for
    #def

    def get_visibilily_polygon(self,origin):
        ox,oy = origin
        origin_vis = vis.Point(ox,oy)
        isovist = vis.Visibility_Polygon(origin_vis, env, epsilon)
        pointlist = vis_poly_to_pointlist(isovist)
        print(pointlist)

def discritize(width,height,space):
    points = []
    for x in range(space//2,width,space):
        for y in range(space//2,height,space):
            points.append((x,y))
    return points

def main():
    parser = argparse.ArgumentParser(description='run ai enviornmnent')
    parser.add_argument('json_fname', type=str, help='enviornment json file')
    args = parser.parse_args()

    env_values = json.load(open(args.json_fname))

    map_parser = SimpleSVGParser()
    map_parser.feed(open("enviornments/"+env_values['svg_fname']).read())
    print(map_parser.blocker_lines)
    discritized_space_points = discritize(map_parser.width,map_parser.height,5)


    libvis = LibVisibility(map_parser.blocker_lines)
    #libvis.get_point_visibility_counts(discritized_space_points)

    pygame.init()

    screen = pygame.display.set_mode([map_parser.width, map_parser.height])

    running = True
    count = 0
    while running:

        # Did the user click the window close button?
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Fill the background with white
        screen.fill((255, 255, 255))

        renderLines(screen,map_parser.blocker_lines)
        # Draw a solid blue circle in the center
        count += 1
        pygame.draw.circle(screen, (0, 0, 255), (count, count), 5)
        #pygame.draw.line(screen, (0, 0, 255), (250, 250),  (250, 0),3)

        # Flip the display
        pygame.display.flip()

    # Done! Time to quit.
    pygame.quit()

if __name__=="__main__":
    main()
