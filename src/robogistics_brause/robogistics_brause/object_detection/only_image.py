import cv2
import matplotlib.pyplot as plt
import numpy as np 
import pyrealsense2 as rs
import tkinter as tk

from PIL import Image
from scipy.interpolate import splprep, splev

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Wait for a coherent pair of frames: depth and color
frames = pipeline.wait_for_frames()
color_frame = frames.get_color_frame()
color_image = np.asanyarray(color_frame.get_data())
cv2.imwrite('35.png', color_image)
pipeline.stop()