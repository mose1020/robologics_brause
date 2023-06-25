import cv2
import torch
import matplotlib.pyplot as plt
import numpy as np 
import os
import pyrealsense2 as rs
import tkinter as tk
from tkinter import font
import tkinter.ttk as ttk

from PIL import ImageTk
from PIL import Image
from scipy.interpolate import splprep, splev
from ultralytics import YOLO

# new imports for transfer from docker to local
import docker
import shutil

class ColorSelector:
    
    def __init__(self, color_available):
        # Create the main window
        self.color_available = color_available
        self.root = tk.Tk()
        self.method = "Classic"

        # Set the window title and size
        self.root.title("Color Selector")

        if color_available:
            self.root.geometry("550x780")
        else:
            self.root.geometry("550x820")

        # Center the window on the screen
        self.root.eval('tk::PlaceWindow %s center' % self.root.winfo_toplevel())
        self.root.resizable(False, False)

        button_width = 250
        button_height = 320

        red_image = Image.open("src/robogistics_brause/robogistics_brause/object_detection/images/Himbeere.PNG")# Replace "red_image.png" with the path to your image
        red_image = red_image.resize((button_width, button_height)) 
        self.red_image = ImageTk.PhotoImage(red_image)

        orange_image = Image.open("src/robogistics_brause/robogistics_brause/object_detection/images/Orange.PNG")# Replace "red_image.png" with the path to your image
        orange_image = orange_image.resize((button_width, button_height)) 
        self.orange_image = ImageTk.PhotoImage(orange_image)

        green_image = Image.open("src/robogistics_brause/robogistics_brause/object_detection/images/Waldmeister.PNG")# Replace "red_image.png" with the path to your image
        green_image = green_image.resize((button_width, button_height)) 
        self.green_image = ImageTk.PhotoImage(green_image)

        yellow_image = Image.open("src/robogistics_brause/robogistics_brause/object_detection/images/Zitrone.PNG")# Replace "red_image.png" with the path to your image
        yellow_image = yellow_image.resize((button_width, button_height)) 
        self.yellow_image = ImageTk.PhotoImage(yellow_image)

        # Create four buttons with different background colors and tyles
        self.red_button = tk.Button(self.root, image=self.red_image,command=self.select_red, width=button_width, height=button_height)
        self.orange_button = tk.Button(self.root, image=self.orange_image,command=self.select_orange, width=button_width, height=button_height)
        self.green_button = tk.Button(self.root, image=self.green_image,command=self.select_green, width=button_width, height=button_height)
        self.yellow_button = tk.Button(self.root, image=self.yellow_image,command=self.select_yellow, width=button_width, height=button_height)

        # Create the checkboxes
        self.checkboxClassic_var = tk.BooleanVar(value=True)
        self.checkboxYOLO_var = tk.BooleanVar(value=False)
        self.checkboxClassic = tk.Checkbutton(self.root, text="Klassisch", variable=self.checkboxClassic_var, command=self.check_classic, bg="white",font=("Arial", 20),highlightthickness=0)
        self.checkboxYOLO = tk.Checkbutton(self.root, text="YOLOV8", variable=self.checkboxYOLO_var, command=self.check_yolo, bg="white",font=("Arial", 20),highlightthickness=0)

        # Pack the buttons into the window using the grid layout manager
        self.red_button.grid(row=0, column=0, padx=10, pady=10)
        self.orange_button.grid(row=0, column=1, padx=10, pady=10)
        self.green_button.grid(row=1, column=0, padx=10, pady=10)
        self.yellow_button.grid(row=1, column=1, padx=10, pady=10)

        # Pack the Checkboxes into the window using the grid layout manager
        self.checkboxClassic.grid(row=2, column=0, padx=10, pady=20)
        self.checkboxYOLO.grid(row=2, column=1, padx=10, pady=20)

       # Check if the text should be displayed initially
        if not self.color_available:
            self.show_text()

        self.root.configure(bg="white")
    # Method to select the "red" color
    def select_red(self):
        self.root.destroy()
        self.color = "red"

    # Method to select the "orange" color
    def select_orange(self):
        self.root.destroy()
        self.color = "orange"

    # Method to select the "green" color
    def select_green(self):
        self.root.destroy()
        self.color = "green"

    # Method to select the "yellow" color
    def select_yellow(self):
        self.root.destroy()
        self.color = "yellow"

    def check_classic(self):
        if self.checkboxClassic_var.get():
            self.checkboxYOLO_var.set(False)
            self.method = "Classic"

    # Method to handle Checkbox 2 selection
    def check_yolo(self):
        if self.checkboxYOLO_var.get():
            self.checkboxClassic_var.set(False)
            self.method = "YOLO"

    # Method to show the color selector window self.root.state('zoomed')and return the selected color
    def get_color_and_method(self):
        self.color = None
        self.root.mainloop()
        return self.color, self.method
    
    # Method to show the text
    def show_text(self):
        self.text_label = tk.Label(self.root, text="Farbe nicht vorhanden. Bitte wählen Sie erneut aus.", font=font.Font(weight="bold"), bg="white", fg="red")
        self.text_label.grid(row=3, column=0, columnspan=2, padx=10, pady=10)

class ColorImage:

    def __init__(self, color, method):
        self.brightness_value_Classic = 60
        self.saturation_value_Classic = 70
        self.brightness_value_YOLO = 50
        self.saturation_value_YOLO = 50
        self.color = color
        self.folder_path = "images_realsense/" 
        self.YOLO_IMG_PATH = "src/robogistics_brause/robogistics_brause/YOLO_image.jpg" # safe the image for YOLO here
        self.image_path = "src/robogistics_brause/robogistics_brause/images/"
        self.new_folder_path = ""
        self.method = method # method "YOLO"
        self.model = YOLO(model='src/robogistics_brause/robogistics_brause/object_detection/Yolov8_model/weights/best.pt')
        self.mask_threshold = 6000  #if mask_threshold > 6000 pixel, pick was successful

    def getTreshold(self):
        # define the lower and upper boundaries of the colors in the HSV color space
        if self.color=="red":
            lower_value = np.array([149, 84, 57], dtype = "uint8") 
            upper_value= np.array([179, 255, 255], dtype = "uint8")

        if self.color=="yellow":
            lower_value = np.array([15, 84, 151], dtype = "uint8") 
            upper_value= np.array([27, 255, 255], dtype = "uint8")

        if self.color=="orange":
            lower_value = np.array([0, 135, 99], dtype = "uint8") 
            upper_value= np.array([13, 255, 255], dtype = "uint8")

        if self.color=="green":
            lower_value = np.array([30, 62, 0], dtype = "uint8") 
            upper_value= np.array([86, 255, 255], dtype = "uint8")

        return lower_value, upper_value
    
    def startStream(self):
        # Configure color streams
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

        align = rs.align(rs.stream.color)

        # Start streaming
        pipeline.start(config)

        # Set the brightness and saturation of the camera
        color_sensor = pipeline.get_active_profile().get_device().first_color_sensor()

        if self.method == "Classic":
            color_sensor.set_option(rs.option.brightness, self.brightness_value_Classic)
            color_sensor.set_option(rs.option.saturation, self.saturation_value_Classic)
        else:
            color_sensor.set_option(rs.option.brightness, self.brightness_value_YOLO)
            color_sensor.set_option(rs.option.saturation, self.saturation_value_YOLO)

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()

        # Stop streaming
        pipeline.stop()

        # Convert images to numpy arrays
        color_array = np.asanyarray(color_frame.get_data())
        bgr_array = cv2.cvtColor(color_array, cv2.COLOR_RGB2BGR)
        color_img = Image.fromarray(bgr_array)
        color_img.save(self.YOLO_IMG_PATH)


        # Check for existing folders with a similar name
        existing_folders = [name for name in os.listdir(self.image_path) if os.path.isdir(os.path.join(self.image_path, name))]
        existing_numbers = [int(name.split("_")[1]) for name in existing_folders if name.startswith("images_")]

        # Find the highest number used so far
        if existing_numbers:
            highest_number = max(existing_numbers)
        else:
            highest_number = -1

        # Create a new folder with an incremented number
        new_number = highest_number + 1
        new_folder_name = f"images_{new_number}"
        new_folder_path = os.path.join(self.image_path, new_folder_name)

        # Create the new folder
        os.makedirs(new_folder_path)
        self.new_folder_path = new_folder_path
        new_image_path = os.path.join(new_folder_path, "color_image.jpg")
        color_img.save(new_image_path)

        return color_array

    def getClassicalMask(self, color_array):

        lower_value, upper_value = self.getTreshold()

        hsv = cv2.cvtColor(color_array, cv2.COLOR_BGR2HSV)    # convert to HSV
        mask = cv2.inRange(hsv, lower_value, upper_value)  # mask for color

        kernel = np.ones((15,15),np.uint8)  # 15x15 kernel for morphological transformation
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        kernel = np.ones((5,5),np.uint8)  # 20x20 kernel for morphological transformation
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
        classical_mask = closing

        image_path = os.path.join(self.new_folder_path, "classical_mask.jpg")
        classical_mask_image = Image.fromarray(classical_mask)
        classical_mask_image.save(image_path)

        return classical_mask
    
    def getYOLOMask(self):
        results = self.model.predict(source=self.YOLO_IMG_PATH, save=False, save_txt=False, stream=True)
        print("RESULT", results)
        color_index = 0
        if self.color == "red":
            color_index = 2

        if self.color == "green":
            color_index = 0

        if self.color == "yellow":
            color_index = 3

        if self.color == "orange":
            color_index = 1

        try:
            for result in results: # only on result in results
                
                # get array results
                all_masks = result.masks.data
                boxes = result.boxes.data
                color = boxes[:, 5] #
                idx_masks_color = torch.where(color == color_index) # index of the chosen coulour
                # use these indices to extract the relevant masks
                for index in idx_masks_color:
                    color_masks = all_masks[index]
                    result_masks = [[],[]]
                    for i in range(len(color_masks)):

                        result_mask = color_masks[i].cpu().numpy()
                        result_masks[0].append(np.count_nonzero(result_mask))
                        result_masks[1].append(result_mask)

                _, final_mask_list = (list(t) for t in zip(*sorted(zip(result_masks[0], result_masks[1]), reverse=True)))


                accumulated_array = np.zeros_like(final_mask_list[0])
                for mask in final_mask_list:
                    accumulated_array = np.logical_or(accumulated_array, mask)

                accumulated_array = accumulated_array.astype('uint8')
                resized_accumulated_array = cv2.resize(accumulated_array, (1280, 720))
                resized_accumulated_array = resized_accumulated_array.astype('uint8')
                image_path = os.path.join(self.new_folder_path, "yolo_mask.jpg")
                grayscale_image = Image.fromarray((resized_accumulated_array * 255).astype(np.uint8), mode='L')
                grayscale_image.save(image_path)

                mask_size = np.count_nonzero(final_mask_list[0])

                binary_array = np.where(final_mask_list[0] != 0, 1, 0)

                binary_array = binary_array.astype('uint8')
                resized_binary_array = cv2.resize(binary_array,(1280, 720))
                resized_binary_array.astype('uint8') # nicht sicher ob notwendig

                # image_path = os.path.join(self.new_folder_path, "yolo_mask.jpg")
                # grayscale_image = Image.fromarray((resized_binary_array * 255).astype(np.uint8), mode='L')
                # grayscale_image.save(image_path)

        except:
            mask_size = 0
            resized_binary_array = 0

        return resized_binary_array, mask_size
    
    def getPixelCoordinates(self, color_image):

        if self.method == "YOLO":
            mask, _ = self.getYOLOMask()
            filename = "yolo_pixel_coordinates.jpg"
        else:
            mask = self.getClassicalMask(color_image)
            filename = "classical_pixel_coordinates.jpg"

        mask_size = np.count_nonzero(mask)

        if mask_size == 0:
            x_pixelkoordinate = 0
            y_pixelkoordinate = 0
            return x_pixelkoordinate, y_pixelkoordinate

        contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # find contours
        biggest_contour = max(contours, key = cv2.contourArea) # find the biggest contour (c) by the area

        # find the center of the contour
        M = cv2.moments(biggest_contour)
        x_pixelkoordinate = int(M['m10']/M['m00'])
        y_pixelkoordinate = int(M['m01']/M['m00'])

        # find the spline
        tck, u = splprep(biggest_contour[:,0,:].T, s=0.0)    
        u_new = np.linspace(u.min(), u.max(), 1000) 
        x_new, y_new = splev(u_new, tck, der=0)

        # Show the original image with the contour
        image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        plt.figure(figsize=(8, 8))
        plt.imshow(image)
        plt.plot(x_new, y_new, 'r')
        plt.plot(x_pixelkoordinate, y_pixelkoordinate, 'bo') 

        # Save the plot image in the "images_realsense/" folder with the filename
        plt.savefig(os.path.join(self.new_folder_path, filename))

        return x_pixelkoordinate, y_pixelkoordinate
    
    def transfer_image_to_local(self):
        #client = docker.from_env()
        client = docker.DockerClient(base_url='unix://var/run/docker.sock')
        container_name_or_id = "brause"
        container_path = self.folder_path
        destination_dir = "/images"
        temp_dir = shutil.mkdtemp()
        container = client.containers.get(container_name_or_id)
        container.get_archive(container_path, temp_dir)
        shutil.move(f"{temp_dir}/{container_path}", destination_dir)
        shutil.rmtree(temp_dir)

    def picSuccessful(self):

        color_image = self.startStream()

        bgr_array = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
        image_path = os.path.join(self.new_folder_path, "check_successful.jpg")
        color_img = Image.fromarray(bgr_array)
        color_img.save(image_path)

        _, mask_size = self.getYOLOMask()

        if mask_size > self.mask_threshold:
            return True
        
        else:
            return False
        
class DepthImage:

    def __init__(self, x_pixelkoordinate, y_pixelkoordinate):
        self.x_pixelkoordinate = x_pixelkoordinate
        self.y_pixelkoordinate = y_pixelkoordinate
        
    def startDepthStream(self):
        # Initialize RealSense pipeline and get depth sensor intrinsics
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        align = rs.align(rs.stream.color)
        profile = pipeline.start(config)

        return pipeline, profile, align
        
    def get3DCoordinates(self, pipeline, profile, align):
        depth_sensor = profile.get_device().first_depth_sensor()

        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()

        intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

        # Define the range of pixels to consider around the desired pixel
        pixel_range = 3

        depth_image = np.asanyarray(depth_frame.get_data())

        pixel = (self.x_pixelkoordinate, self.y_pixelkoordinate)  # example pixel coordinate

        # Get the depth values for the pixels in the specified range
        depth_values = []
        for i in range(pixel[1] - pixel_range, pixel[1] + pixel_range + 1):
            for j in range(pixel[0] - pixel_range, pixel[0] + pixel_range + 1):
                if i >= 0 and i < depth_image.shape[0] and j >= 0 and j < depth_image.shape[1]:
                    depth_values.append(depth_image[i, j])

        # Compute the median depth value
        median_depth_value = np.median(depth_values)

        # Deproject pixel to 3D point in camera coordinates
        point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, pixel, median_depth_value)

        x_cameraFrame = point_3d[0]/1000
        y_cameraFrame = point_3d[1]/1000
        z_cameraFrame = point_3d[2]/1000

        # Stop streaming
        pipeline.stop()

        return x_cameraFrame, y_cameraFrame, z_cameraFrame

def getPose(color_available):
    color_selector = ColorSelector(color_available)
    selected_color, method = color_selector.get_color_and_method()

    image = ColorImage(selected_color, method)
    color_image = image.startStream()

    x_pixelkoordinate, y_pixelkoordinate = image.getPixelCoordinates(color_image)

    if x_pixelkoordinate == 0 and y_pixelkoordinate == 0:
        getPose(False)

    depthImage = DepthImage(x_pixelkoordinate, y_pixelkoordinate)
    pipeline, profile, align = depthImage.startDepthStream()
    x_cameraFrame, y_cameraFrame, z_cameraFrame = depthImage.get3DCoordinates(pipeline, profile, align)
    print("POSE", x_cameraFrame, y_cameraFrame, z_cameraFrame)
    return -x_cameraFrame, y_cameraFrame, z_cameraFrame, selected_color, method