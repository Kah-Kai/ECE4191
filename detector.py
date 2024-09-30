import cv2
import numpy as np

class Detector:
    def __init__(self):
        # Constants
        self.FRAME_WIDTH = 640
        self.FRAME_HEIGHT = 480
        self.MAX_NUM_OBJECTS = 5
        self.MIN_OBJECT_AREA = 100
        self.MAX_OBJECT_AREA = self.FRAME_HEIGHT * self.FRAME_WIDTH / 1.5

        self.true_radius = 0.0335                         # Actual radius of tennis ball in m
        self.f = np.loadtxt("params/focal_length.txt")    # Focal length of camera

        # Trackbar names and initial values
        # self.trackbar_values = {
        #     'H_MIN': 20,
        #     'H_MAX': 110,
        #     'S_MIN': 115,
        #     'S_MAX': 255,
        #     'V_MIN': 65,
        #     'V_MAX': 255
        # }

        # (H, S, V) 
        self.hsv_min = (20, 115, 65)
        self.hsv_max = (110, 255, 255)
  
        self.trackbar_window_name = "Trackbars"

        # Start camera
        print("Initialising Camera")
        self.capture = cv2.VideoCapture(0)

        if self.capture.isOpened:
            print("Camera Capture Acquired")
        else:
            print("Error: Could not open video capture")
            return
        
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.FRAME_WIDTH)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FRAME_HEIGHT)


    def nothing(self): pass

    def create_trackbars(self):
        cv2.namedWindow(self.trackbar_window_name)
        for trackbar, initial_value in self.trackbar_values.items():
            cv2.createTrackbar(trackbar, self.trackbar_window_name, initial_value, 256, self.nothing)


    # distance = actual radius / measured radius * focal length
    def get_dist(self, measured_radius, ball_center):
        distance =  (self.true_radius/measured_radius)*self.f
        
        x_shift = self.FRAME_WIDTH/2 - ball_center  # x distance between bounding box centre and centreline in camera view
        theta = np.arctan(x_shift/self.f)           # angle of ball relative to the robot
        
        # relative object location
        #distance_obj = distance/np.cos(theta) # relative distance between robot and object
        x_relative = distance                   # relative x pose
        y_relative = distance*np.tan(theta)     # relative y pose

        return x_relative, y_relative

    
    def morph_ops(self, thresh):
        erode_element = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        dilate_element = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8))
        thresh = cv2.erode(thresh, erode_element, iterations=2)
        thresh = cv2.dilate(thresh, dilate_element, iterations=2)
        return thresh
    
    # Capture frame
    def cam_cap(self):
        ret, frame = self.capture.read()
        if not ret:
            print("Error: Empty frame captured")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        threshold = cv2.inRange(hsv, self.hsv_min,self.hsv_max)

        threshold = self.morph_ops(threshold)

        return self.track_filtered_objects(threshold, frame)
    
    def track_filtered_objects(self, threshold, frame):
        temp = threshold.copy()
        contours, _ = cv2.findContours(temp, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            num_objects = len(contours)
            if num_objects <self.MAX_NUM_OBJECTS:
                lbcx, lbcy, lbc, lbd = self.draw_objects(contours, frame)
                return lbcx, lbcy, lbc, lbd, num_objects
            else:
                cv2.putText(frame, "TOO MUCH NOISE! ADJUST FILTER", (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    def draw_objects(self, contours, frame):
        # Store Radii of Moments
        radii = []
        for contour in contours:
            # Calculate moments for each contour
            moment = cv2.moments(contour)
            area = moment['m00']

            if self.MIN_OBJECT_AREA < area < self.MAX_OBJECT_AREA:
                # Calculate the center of the object
                x = int(moment['m10'] / area)
                y = int(moment['m01'] / area)

                # Fit an ellipse to the contour if possible
                if len(contour) >= 5:  # Need at least 5 points to fit an ellipse
                    ellipse = cv2.fitEllipse(contour)
                    center, axes, angle = ellipse
                    radius = min(axes) / 2 # Use the major axis length as the radius
                    radii.append(radius) # Append to list to find max
                    
                    # Display Derived Distance to ball
                    dist = self.get_dist(radius)

                    # Flag if Largest Ball is Centered
                    if radius == max(radii):
                        
                        if int(center[0]) >= 154 and int(center[0]) <= 164:
                            ball_centered = 1
                        else:
                            ball_centered = 0
                        
                        return center[0], center[1], ball_centered, dist

    def detect_line(self):
        pass

    def run(self):
        # Detect ball and line
        pass