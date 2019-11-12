import env_lib
from env_lib import Enviornment
from exploring_agent_ai import AgentDecisionMaker
import sys
from PIL import Image,ImageDraw
import PIL
import os
import shutil
import numpy as np
import subprocess
import json

def draw_color(map,coord,color):
    ax,ay = coord
    draw_rad = 1
    for i in range(-draw_rad,draw_rad+1):
        for j in range(-draw_rad,draw_rad+1):
            map[int(ay)+i,int(ax)+j] = color

def save_video():
    ffmpeg_call = [
        "ffmpeg",
        "-y",# overwrite output.mp4 if already there
        "-hide_banner","-loglevel","error", #don't print out unnecessary stuff
        "-pattern_type", "glob","-i","img_data/data*.png",# get input from image list
        "-c:v","libx264",#deine output format
        "-r","30", #define output sample rate
        "-pix_fmt","yuv420p",#???
        "output.mp4"
    ]
    subprocess.call(ffmpeg_call)


def draw_image(out_fname,rgb_map,agent_loc,guard_locs):
    draw_color(rgb_map,agent_loc,env_lib.to_rgb_int(0,255,0))
    for guard_loc in guard_locs:
        draw_color(rgb_map,guard_loc,env_lib.to_rgb_int(0,0,255))

    Blue = rgb_map & 255
    Green = (rgb_map >> 8) & 255
    Red =   (rgb_map >> 16) & 255
    img_data = np.stack([Red,Green,Blue],axis=2)
    orig_img = Image.fromarray(img_data.astype(np.uint8))
    orig_shape = rgb_map.shape

    big_image = orig_img.resize(size=(orig_shape[0]*2,orig_shape[1]*2),resample=PIL.Image.NEAREST)

    big_image.save(out_fname)


def main(json_data):
    agent_decider = AgentDecisionMaker(json_data)
    env = Enviornment(json_data)
    if os.path.exists("img_data"):
        shutil.rmtree("img_data")
    os.mkdir("img_data")

    for x in range(200):
        for guard in env.guards:
            move = guard.find_move(env.rgb_vals)
            env.update_robot_pos((guard.x,guard.y),move)
            guard.exec_move(move)
        #print("updated")
        ray_results = env.communicate_line_sight(env.agent)
        move_dir = agent_decider.on_sight(ray_results)
        actual_dir = env.exec_agent_move(env.agent,move_dir)
        agent_reward_col = env.agent_collect_reward()

        agent_decider.on_move(actual_dir,agent_reward_col)

        agent_location = (env.agent.x,env.agent.y)
        next_guard_l = []
        for guard in env.guards:
            next_guard_l.append((guard.x,guard.y))

        SAMPLE_RATE = 3
        if x % SAMPLE_RATE == 0:
            img_data = env.get_img()
            draw_image("img_data/data{0:05d}.png".format(x//SAMPLE_RATE),img_data,agent_location,next_guard_l)

        if x % 10 == 0:
            print("executed {}th iteration".format(x))
        #if x % 10 == 0:
        #    env.save_img("data/data{}.png".format(x))
        if env.check_game_over() == "FOUND_AGENT":
            print("guard found agent")
            #env.save_img("datafinal.png")
            break

    print("total reward: ",env.reward_collected)

if __name__ == "__main__":
    assert len(sys.argv) == 2, "needs a command line argument defining the input json file name"
    fname = sys.argv[1]
    json_data = json.load(open(fname))
    try:
        main(json_data)
    finally:
        save_video()
