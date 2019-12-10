import os
import subprocess
import shutil
srcfolder = "/mnt/disk/robo_bigfiles/wm_img_dir/"
destfolder = "wm_img_dir/"
def remove_if(path):
    if os.path.exists(path):
        os.remove(path)
        
def process(path):
    if "_guard." in path:
        name = path[:path.find("_guard.")]
        num = int(path[path.rfind(".")+1:])
        dest_guard = destfolder+name+"_guard.weightmap.npy"
        dest_agent = destfolder+name+"_agent.weightmap.npy"
        remove_if(dest_guard)
        remove_if(dest_agent)
        os.symlink(srcfolder+name+"_guard.weightmap.npy.{}".format(num),dest_guard)
        os.symlink(srcfolder+name+"_agent.weightmap.npy.{}".format(num),dest_agent)
        subprocess.check_call(["python","main.py","-D","-V","enviornments/"+name+".json"])
        vid_name = name+"_vid.mp4"
        os.rename(vid_name,"bulk_videos_img_dir/"+name+"_vid_{}.mp4".format(num))

for path in os.listdir(srcfolder):
    process(path)
