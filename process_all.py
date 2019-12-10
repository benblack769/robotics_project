import os
import subprocess
import random
import shutil

srcfolder = "wm_img_dir/enviornments/"
destfolder = "wm_img_dir/"
def remove_if(path):
    if os.path.exists(path):
        os.remove(path)

def process(path):
    if "_guard." in path:
        name = path[:path.find("_guard.")]
        num = int(path[path.rfind(".")+1:])
        dest_guard = destfolder+name+"_guard.weightmap.json"
        dest_agent = destfolder+name+"_agent.weightmap.json"
        dest_guard = dest_guard.replace(".json","",1)
        dest_agent = dest_agent.replace(".json","",1)
        real_name = name.replace(".json","",1)
        remove_if(dest_guard)
        remove_if(dest_agent)
        shutil.copy(srcfolder+name+"_guard.weightmap.json.{}".format(num),dest_guard)
        shutil.copy(srcfolder+name+"_agent.weightmap.json.{}".format(num),dest_agent)
        subprocess.check_call(["python","main.py","-D","-V","enviornments/"+real_name+".json"])
        vid_name = real_name+"_vid.mp4"
        os.rename(vid_name,"new_videos_img_dir/"+real_name+"_vid_{}.mp4".format(num))

src_dir = list(os.listdir(srcfolder))
random.shuffle(src_dir)
for path in src_dir:
    process(path)
