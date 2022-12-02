from PIL import Image
import numpy as np
import glob
import os

def make_video(path, savename):

	CURR_DIR = os.path.dirname(os.path.realpath(__file__))

	frames = []

	for filename in sorted(glob.glob(CURR_DIR + path), key=os.path.getmtime):
		new_frame = Image.open(filename)
		frames.append(new_frame)

	frames[0].save(savename + ".gif", format="GIF", append_images=frames[1:], save_all=True, loop=0)

if __name__ == "__main__":

	make_video()