#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point
from cv_bridge import CvBridge, CvBridgeError
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Float64

class PrecisionLandingDetector:
    def __init__(self):
        rospy.init_node('precision_landing_detector', anonymous=True)
        
        # Parameters
        self.landing_target_size = rospy.get_param('~landing_target_size', 0.5)  # meters
        self.camera_topic = rospy.get_param('~camera_topic', '/iris/usb_cam/image_raw')
        
        # Variables
        self.bridge = CvBridge()
        self.current_state = State()
        self.target_detected = False
        self.target_position = Point()
        
        # Publishers
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.landing_target_pub = rospy.Publisher('/landing_target/position', PoseStamped, queue_size=10)
        
        # Subscribers
        self.current_altitude = 0.0
        self.current_position_x = 0.0
        self.current_position_y = 0.0
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.position_callback)
        rospy.Subscriber(self.camera_topic, Image, self.image_callback)
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        
        # Service clients
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        
        rospy.loginfo("Precision Landing Detector initialized")
        
    def state_callback(self, msg):
        self.current_state = msg
        
    def position_callback(self, msg):
	    self.current_altitude = msg.pose.position.z
	    self.current_position_x = msg.pose.position.x
	    self.current_position_y = msg.pose.position.y
       
        
    def image_callback(self, data):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # Process image to detect landing target
            detected, x_offset, y_offset = self.detect_landing_pad(cv_image)
            
            if detected:
                # Target detected, publish position
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "camera_frame"
                
                # Convert pixel coordinates to real-world coordinates
                # This is a simplified conversion for demonstration
                # In a real application, you would use proper camera calibration
                camera_height = self.current_altitude 
                fov_x = 1.3962634  # ~80 degrees in radians
                fov_y = 1.0471975  # ~60 degrees in radians
                
                image_width = cv_image.shape[1]
                image_height = cv_image.shape[0]
                
                # Normalize to [-1, 1] range
                norm_x = (x_offset / (image_width / 2.0)) - 1.0
                norm_y = (y_offset / (image_height / 2.0)) - 1.0
                
                # Convert to actual displacement based on height and FOV
                pose.pose.position.x = camera_height * np.tan(norm_x * fov_x / 2.0)
                pose.pose.position.y = camera_height * np.tan(norm_y * fov_y / 2.0)
                pose.pose.position.z = camera_height
                
                self.landing_target_pub.publish(pose)
                self.target_detected = True
                self.target_position = pose.pose.position
                
                # Display detection results on image
                cv2.circle(cv_image, (int(x_offset + image_width / 2), int(y_offset + image_height / 2)), 10, (0, 255, 0), 2)
                cv2.putText(cv_image, f"Target: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                self.target_detected = False
                
            # Display the processed image
            cv2.imshow("Landing Target Detection", cv_image)
            cv2.waitKey(1)
            
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
    
    def detect_landing_pad(self, image):
        """
        Detect a circular landing pad in the image
        Returns: (detected, x_offset, y_offset)
        """
        # Convert to HSV for better color filtering
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define range for red color detection
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        # Create masks for red detection
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)
        
        # Apply morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.erode(red_mask, kernel, iterations=1)
        red_mask = cv2.dilate(red_mask, kernel, iterations=2)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Initialize variables
        detected = False
        x_offset = 0
        y_offset = 0
        
        # Process contours
        if contours:
            # Find the largest contour by area
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            # Only process if contour is large enough
            if area > 500:  # Adjust this threshold based on your camera view
                # Get center of the contour
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])
                    
                    # Calculate offset from center of image
                    x_offset = center_x - (image.shape[1] / 2)
                    y_offset = center_y - (image.shape[0] / 2)
                    
                    detected = True
        
        return detected, x_offset, y_offset
    
    def execute_precision_landing(self):
        """
        Main function to execute the precision landing
        """
        rate = rospy.Rate(20)  # 20 Hz
        altitude_reached = False;
        
        # Wait for FCU connection
        while not rospy.is_shutdown() and not self.current_state.connected:
            rate.sleep()
        
        rospy.loginfo("FCU connected")
        
        setpoint = PoseStamped()
        setpoint.header.frame_id = "map"
        setpoint.pose.position.x = 0.0
        setpoint.pose.position.y = 0.0
        setpoint.pose.position.z = 3.0  # Start at 3m altitude
        setpoint.pose.orientation.w = 1.0  # Unit quaternion
        
        # Send setpoints for a few seconds before attempting to arm and switch mode
        rospy.loginfo("Publishing initial setpoints...")
        for i in range(100):  # About 5 seconds at 20Hz
            setpoint.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(setpoint)
            rate.sleep()
        
        rospy.loginfo("Trying to arm...")
        # Attempt to arm
        if not self.current_state.armed:
            arm_result = self.arming_client(True)
            rospy.loginfo(f"Arming result: {arm_result}")
            if not arm_result.success:
                rospy.logwarn("Arming failed!")
            
        # Continue publishing setpoints
        for i in range(20):  # 1 more second of setpoints
            setpoint.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(setpoint)
            rate.sleep()
            
        rospy.loginfo("Trying to set OFFBOARD mode...")
        # Try to set OFFBOARD mode
        last_request = rospy.Time.now()
        offboard_sent = False   
        
        landing_stage = "SEARCH"  # Stages: SEARCH, APPROACH, ALIGN, DESCEND, LAND
        
        # Main control loop
        while not rospy.is_shutdown():
            # Update timestamp
            setpoint.header.stamp = rospy.Time.now()
            
            # Always publish setpoint to keep OFFBOARD mode active
            self.setpoint_pub.publish(setpoint)
            
            # Try to switch to OFFBOARD mode every 5 seconds until successful
            if not offboard_sent or (self.current_state.mode != "OFFBOARD" and 
                                    (rospy.Time.now() - last_request) > rospy.Duration(5.0)):
                offboard_result = self.set_mode_client(custom_mode="OFFBOARD")
                rospy.loginfo(f"OFFBOARD request result: {offboard_result.mode_sent}")
                last_request = rospy.Time.now()
                offboard_sent = True
                
            if self.current_altitude >= 3.0:
                altitude_reached = True
                
            # Once in OFFBOARD mode, proceed with actual landing logic
            if self.current_state.mode == "OFFBOARD":
                if landing_stage == "SEARCH":
                # Search for landing pad at 3m altitude
                    if self.target_detected and altitude_reached:
                        landing_stage = "APPROACH"
                        rospy.loginfo("Landing pad detected, approaching...")
                    else:
                        # Hover and search
                        rospy.loginfo("Searching..")
                        setpoint = PoseStamped()
                        setpoint.header.stamp = rospy.Time.now()
                        setpoint.pose.position.x = 0  # Or implement search pattern
                        setpoint.pose.position.y = 0
                        setpoint.pose.position.z = 3.0
                    
                elif landing_stage == "APPROACH":
                    # Move horizontally toward pad at safe altitude
                    setpoint = PoseStamped()
                    setpoint.header.stamp = rospy.Time.now()
                    setpoint.pose.position.x = -self.target_position.y  
                    setpoint.pose.position.y = -self.target_position.x
                    setpoint.pose.position.z = 3.0  # Maintain altitude
                    
                    # Check if we're roughly above the pad
                    if abs(self.target_position.y) < 0.5 and abs(self.target_position.x) < 0.5:
                        landing_stage = "ALIGN"
                        rospy.loginfo("Near landing pad, aligning precisely...")
                        
                elif landing_stage == "ALIGN":
                    # Fine alignment directly above the pad
                    setpoint = PoseStamped()
                    setpoint.header.stamp = rospy.Time.now()
                    setpoint.pose.position.x = -self.target_position.y  
                    setpoint.pose.position.y = -self.target_position.x
                    setpoint.pose.position.z = 2.0  # Lower to 2m
                    
                    # Check if we're precisely aligned
                    if abs(self.target_position.y) < 2 and abs(self.target_position.x) < 2:
                        landing_stage = "DESCEND"
                        rospy.loginfo("Aligned with landing pad, starting gentle descent...")
                        
                elif landing_stage == "DESCEND":
                    # Gentle controlled descent
                    setpoint = PoseStamped()
                    setpoint.header.stamp = rospy.Time.now()
                    
                    # Stay aligned
                    setpoint.pose.position.x = -0.8*self.target_position.y 
                    setpoint.pose.position.y = -0.8*self.target_position.x
                    
                    # Calculate descent rate based on altitude
                    if self.current_altitude > 1.0:
                        descent_to = self.current_altitude - 0.1
                    elif self.current_altitude > 0.5:
                        descent_to = self.current_altitude - 0.05
                    elif self.current_altitude > 0.3:
                        descent_to = self.current_altitude - 0.02
                    else:
                        landing_stage = "LAND"
                        rospy.loginfo("Reached minimum altitude, switching to LAND mode...")
                        continue
                        
                    setpoint.pose.position.z = descent_to
                    
                elif landing_stage == "LAND":
                    # Final landing using PX4's land mode
                    self.set_mode_client(custom_mode="AUTO.LAND")
                    rospy.loginfo("Landing complete.")
                    break
            
                # Publish the appropriate setpoint
                self.setpoint_pub.publish(setpoint)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        detector = PrecisionLandingDetector()
        detector.execute_precision_landing()
    except rospy.ROSInterruptException:
        pass
