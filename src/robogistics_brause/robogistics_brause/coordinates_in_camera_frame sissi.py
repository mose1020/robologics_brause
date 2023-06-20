import cv2
import matplotlib.pyplot as plt
import numpy as np 
import os
import pyrealsense2 as rs
import tkinter as tk
import torch
from PIL import ImageTk

from PIL import Image
from scipy.interpolate import splprep, splev
from ultralytics import YOLO

class ColorSelector:
    
    def __init__(self):
        # Create the main window
        self.root = tk.Tk()
        self.use_yolo = False


        # Set the window title and size
        self.root.title("Color Selector")
        #self.root.geometry("340x170")
        # Set the window size to the maximum
        self.root.state('zoomed')

     


        # Center the window on the screen
        self.root.eval('tk::PlaceWindow %s center' % self.root.winfo_toplevel())

        red_image = Image.open(r"src\robogistics_brause\robogistics_brause\object_detection\images\Himbeere.PNG")
        orange_image = Image.open(r"src\robogistics_brause\robogistics_brause\object_detection\images\Orange.PNG")
        green_image = Image.open(r"src\robogistics_brause\robogistics_brause\object_detection\images\Waldmeister.PNG")
        yellow_image = Image.open(r"src\robogistics_brause\robogistics_brause\object_detection\images\Zitrone.PNG")

        # Resize the images to fit the buttons
        button_size = (200, 300)
        red_image = red_image.resize(button_size)
        orange_image = orange_image.resize(button_size)
        green_image = green_image.resize(button_size)
        yellow_image = yellow_image.resize(button_size)

        # Convert the images to Tkinter-compatible format
        red_photo = ImageTk.PhotoImage(red_image)
        orange_photo = ImageTk.PhotoImage(orange_image)
        green_photo = ImageTk.PhotoImage(green_image)
        yellow_photo = ImageTk.PhotoImage(yellow_image)

        # Create four labels with images
        self.red_button = tk.Label(self.root, image=red_photo, bd=0, highlightthickness=0)
        self.orange_button = tk.Label(self.root, image=orange_photo, bd=0, highlightthickness=0)
        self.green_button = tk.Label(self.root, image=green_photo, bd=0, highlightthickness=0)
        self.yellow_button = tk.Label(self.root, image=yellow_photo, bd=0, highlightthickness=0)

        # Keep a reference to the images to prevent garbage collection
        self.red_button.image = red_photo
        self.orange_button.image = orange_photo
        self.green_button.image = green_photo
        self.yellow_button.image = yellow_photo

        # Bind click events to the labels
        self.red_button.bind("<Button-1>", lambda event: self.select_red())
        self.orange_button.bind("<Button-1>", lambda event: self.select_orange())
        self.green_button.bind("<Button-1>", lambda event: self.select_green())
        self.yellow_button.bind("<Button-1>", lambda event: self.select_yellow())

        # Pack the labels into the window using the grid layout manager
        self.red_button.grid(row=0, column=0, padx=10, pady=10)
        self.orange_button.grid(row=0, column=1, padx=10, pady=10)
        self.green_button.grid(row=1, column=0, padx=10, pady=10)
        self.yellow_button.grid(row=1, column=1, padx=10, pady=10)

        # Create two checkboxes
        self.checkbox_var = tk.StringVar()
        self.checkbox_var.set("checkbox_classic")  # Set initial selection to checkbox_classic
        self.checkbox_classic = tk.Checkbutton(self.root, text="Klassisch", variable=self.checkbox_var, onvalue="checkbox_classic", offvalue="", command=self.checkbox_select_classic)
        self.checkbox_yolo = tk.Checkbutton(self.root, text="Yolov8", variable=self.checkbox_var, onvalue="checkbox_yolo", offvalue="", command=self.checkbox_select_yolo)

        # Pack the buttons into the window using the grid layout manager
        # self.red_button.grid(row=0, column=0, padx=10, pady=10)
        # self.orange_button.grid(row=0, column=1, padx=10, pady=10)
        # self.green_button.grid(row=1, column=0, padx=10, pady=10)
        # self.yellow_button.grid(row=1, column=1, padx=10, pady=10)
        self.checkbox_classic.grid(row=2, column=0, padx=10, pady=10)
        self.checkbox_yolo.grid(row=2, column=1, padx=10, pady=10)

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

    # Method to show the color selector window and return the selected color
    def get_color(self):
        self.color = None
        self.root.mainloop()
        return self.color
    
    

    
    def checkbox_select_yolo(self):
        # Function to handle checkbox selection
        #checkbox_yolo_state = self.checkbox_yolo.instate(['selected'])

        self.checkbox_yolo.select()
        self.checkbox_classic.deselect()
        self.use_yolo = True
        print("self.use_yolo ",self.use_yolo )
        self.checkbox_var.set("checkbox_yolo")
        self.checkbox_yolo.update()  # Aktualisiere den visuellen Zustand der Checkbox
        self.checkbox_classic.update() 
        self.root.update_idletasks()

    def checkbox_select_classic(self):
        # Function to handle checkbox selection
        #checkbox_classic_state = self.checkbox_classic.instate(['selected'])

        self.checkbox_yolo.deselect()
        self.checkbox_classic.select()
        self.use_yolo = False
        print("self.use_yolo ",self.use_yolo )
        self.checkbox_yolo.update()  # Aktualisiere den visuellen Zustand der Checkbox
        self.checkbox_classic.update() 
        self.root.update_idletasks()
        


class Popup: 
    def close_popup(popup):
    # Function to close the popup window
        popup.destroy()
    def open_popup_window(message):
        # Create the popup window
        popup = tk.Tk()

        # Set the window title
        popup.title("Frabe nicht vorhanden")

        # Create a label widget with the provided message
        label = tk.Label(popup, text=message, padx=10, pady=10)

        # Pack the label widget into the window
        label.pack()

        # Create a button widget labeled "OK"
        ok_button = tk.Button(popup, text="Verstanden", command=close_popup(popup))

        # Pack the button widget into the window
        ok_button.pack()

        # Start the window's event loop
        popup.mainloop()

   
def close_popup(popup):
    # Function to close the popup window
        popup.destroy()
     


class ColorImage:

    def __init__(self, color, use_yolo):
        self.brightness_value = 60
        self.saturation_value = 70
        self.color = color
        self.folder_path = "images/" 
        self.use_yolo = use_yolo

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

        # Start streaming
        pipeline.start(config)

        # Set the brightness and saturation of the camera
        color_sensor = pipeline.get_active_profile().get_device().first_color_sensor()
        color_sensor.set_option(rs.option.brightness, self.brightness_value)
        color_sensor.set_option(rs.option.saturation, self.saturation_value)

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())

        # Stop streaming
        pipeline.stop()

        return color_image

    def getClassicalMask(self, color_image, lower_value, upper_value):

        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)    # convert to HSV
        mask = cv2.inRange(hsv, lower_value, upper_value)  # mask for color

        kernel = np.ones((15,15),np.uint8)  # 15x15 kernel for morphological transformation
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        kernel = np.ones((5,5),np.uint8)  # 20x20 kernel for morphological transformation
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
        classical_mask = closing

        return classical_mask
    
    def getYoloMask(self, color_image):

        model = YOLO(r'src/robogistics_brause/robogistics_brause/object_detection/Yolov8_model/weights/best.pt')
        results = model.predict(source=color_image, save=True, save_txt=False, stream=True)

        color_index = 0 #color index

        if self.color == "red":
            color_index = 2

        if self.color == "green":
            color_index = 0

        if self.color == "yellow":
            color_index = 3

        if self.color == "orange":
            color_index = 1

        all_masks = []
        for result in results:
            
            # get array results
            masks = result.masks.masks
            boxes = result.boxes.boxes
            # extract classes
            clss = boxes[:, 5]
            # get indices of results where class is 0 (people in COCO)
            people_indices = torch.where(clss == color_index)
            # use these indices to extract the relevant masks
            for index in people_indices:
                people_masks = masks[index]
                for i in range(len(people_masks)):

                    result_mask = people_masks[i].cpu().numpy()
                    all_masks.append(result_mask)

        max_ones = 0
        max_mask = None
        for mask in all_masks:
            ones = np.count_nonzero(mask == 1) 
            if ones > max_ones:
                max_ones = ones
                max_mask = mask

        return max_mask

    
    def getPixelCoordinates(self, mask, color_image):
        print("mask", mask)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # find contours
        print("got contours")
        biggest_contour = max(contours, key = cv2.contourArea) # find the biggest contour (c) by the area

        # find the center of the contour
        M = cv2.moments(biggest_contour)
        x_pixelkoordinate = int(M['m10']/M['m00'])
        y_pixelkoordinate = int(M['m01']/M['m00'])

        # find the spline
        tck, u = splprep(biggest_contour[:,0,:].T, s=0.0)    
        u_new = np.linspace(u.min(), u.max(), 1000) 
        x_new, y_new = splev(u_new, tck, der=0)
        print("1")
        # Create the "images" folder if it doesn't exist
        os.makedirs(self.folder_path, exist_ok=True)

        # Determine the next image number
        existing_images = os.listdir(self.folder_path)
        current_image_number = len(existing_images) + 1
        print("2")
        # Define the filename with the current image number
        filename = f"image_{current_image_number}.jpg"

        # Show the original image with the contour
        image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        # plt.figure(figsize=(8, 8))
        # plt.imshow(image)
        # plt.plot(x_new, y_new, 'r')
        # plt.plot(x_pixelkoordinate, y_pixelkoordinate, 'bo') 

        # Save the plot image in the "images" folder with the filename
        plt.savefig(os.path.join(self.folder_path, filename))
        print("3")
        return x_pixelkoordinate, y_pixelkoordinate
    

class DepthImage:

    def __init__(self, x_pixelkoordinate, y_pixelkoordinate):
        self.x_pixelkoordinate = x_pixelkoordinate
        self.y_pixelkoordinate = y_pixelkoordinate
        
    def startDepthStream(self):
        # Initialize RealSense pipeline and get depth sensor intrinsics
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        profile = pipeline.start(config)

        return pipeline, profile
        
    def get3DCoordinates(self, pipeline, profile):
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

        # Define the range of pixels to consider around the desired pixel
        pixel_range = 3

        # Read a depth frame and get a pixel coordinate
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
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

        print("x: ", x_cameraFrame)
        print("y: ", y_cameraFrame)
        print("z: ", z_cameraFrame)

        # Stop streaming
        pipeline.stop()

        return x_cameraFrame, y_cameraFrame, z_cameraFrame


def getPose():
    print("getPose() startet")
    color_selector = ColorSelector()
    print("Fenster offen")
    selected_color = color_selector.get_color()
    print("color geholt")
    image = ColorImage(selected_color,True)
    lower_value, upper_value = image.getTreshold()
    #color_image = image.startStream()
    color_image = cv2.imread(r'F:\Robo3\robologics_brause\src\robogistics_brause\robogistics_brause\object_detection\images\test_image.png', cv2.IMREAD_COLOR)
    print("start stream")
        

    mask =[]
    print("color_selector.use_yolo ", color_selector.use_yolo)
    if color_selector.use_yolo:
        mask = image.getYoloMask(color_image)
        mask = mask.astype(np.uint8)

    else:
        mask = image.getClassicalMask(color_image, lower_value, upper_value)


    if cv2.countNonZero(mask) > 0:
        print("Farbe vorhanden")
        x_pixelkoordinate, y_pixelkoordinate = image.getPixelCoordinates(mask, color_image)
        print("have coordinate")

        depthImage = DepthImage(x_pixelkoordinate, y_pixelkoordinate)
        pipeline, profile = depthImage.startDepthStream()
        x_cameraFrame, y_cameraFrame, z_cameraFrame = depthImage.get3DCoordinates(pipeline, profile)
        print("KOORDI ", y_cameraFrame)
        return x_cameraFrame, y_cameraFrame, z_cameraFrame
    else:
        popup_window = Popup()
        popup_window.open_popup_window("Die Farbe ist nicht vorhanden. Bitte wählen Sie eine andere aus.")
        return



# def main():
#      getPose()

# if __name__ == "__main__":
#      main()

getPose()