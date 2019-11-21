from svg.path import parse_path
from html.parser import HTMLParser
import argparse
import json
import time

class SimpleSVGParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self.blocker_polygons = []
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
            self.blocker_polygons.append([
                (float(attrs["x1"]),
                float(attrs["y1"])),
                (float(attrs["x2"]),
                float(attrs['y2']))
            ])
        elif tag == "rect":
            self.reward_points.append((float(attrs["x"]),float(attrs["y"])))
            print(self.reward_points)
        elif tag == "path":
            pathstr = attrs['d']
            color = attrs['stroke']
            parsed = parse_path(pathstr)
            pointed = [p.point(0.0) for p in parsed]
            tupled = [(p.real,p.imag) for p in pointed]
            if color == "#894F3F":
                self.guard_dests += tupled
            elif color == "#000000":
                self.blocker_polygons.append(tupled)
            else:
                raise RuntimeError("bad color for path: {}".format(color))

    def handle_endtag(self, tag):
        pass

    def handle_data(self, data):
        pass
