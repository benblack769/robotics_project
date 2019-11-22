import argparse
import json
from parse_svg import SimpleSVGParser


def main():
    parser = argparse.ArgumentParser(description='takes svg, generates json with the svg information')
    parser.add_argument('svg_fname', type=str, help='enviornment svg file')
    args = parser.parse_args()

    map_parser = SimpleSVGParser()
    map_parser.feed(open(args.svg_fname).read())
    fin_dict = {
        "width":map_parser.width,
        "height":map_parser.height,
        "blocker_polygons":map_parser.blocker_polygons,
        "reward_points":map_parser.reward_points,
        "guard_dests":map_parser.guard_dests,
    }
    new_fname = args.svg_fname + ".json"
    json.dump(fin_dict,open(new_fname,'w'), indent=4)

if __name__=="__main__":
    main()
