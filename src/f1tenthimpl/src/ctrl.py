# updated

import rospy
import math
from std_msgs.msg import String, Bool, Float32, Float64, Float64MultiArray
from ackermann_msgs.msg import AckermannDriveStamped

class PIDController:
    def __init__(self):
        self.rate = rospy.Rate(50)  

        self.kp = 2.2
        self.ki = 0
        self.kd = 0.1
        # PID variables
        self.prev_error = 0.0
        self.integral = 0.0
        
        self.max_steering_angle = 0.2  # max steering angle (in radians)        
        self.prev_time = None

        # Subscribe to the lane detection error from studentVision
        rospy.Subscriber('lane_detection/error', Float32, self.error_callback)
        
        self.drive_pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        
        self.current_error = 0.0
        
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.frame_id = "f1tenth_control"
        self.drive_msg.drive.speed = 0.5 # constant

        self.last_steering_angle = 0.0
        self.lane_detected = False

        
    

    #fetch error from studentVision
    def error_callback(self, msg): 
        self.current_error = msg.data
        if msg.data > 1000:
            self.lane_detected = False
        else:
            self.lane_detected = True
            self.correction_counter = 0
    

    def run(self):
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            if self.prev_time is None:
                delta_time = 0.0
            else:
                delta_time = (current_time - self.prev_time).to_sec()

            if delta_time == 0:
                delta_time = 0.0001
            
            error = self.current_error
            self.integral += error * delta_time
            derivative = (error - self.prev_error) / delta_time
            
            steering_angle = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
            
            steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))
            self.prev_error = error

            
            # publish the drive message
            self.last_steering_angle = steering_angle
            print(steering_angle)
            self.drive_msg.header.stamp = current_time
            self.drive_msg.drive.steering_angle = -steering_angle
            self.drive_pub.publish(self.drive_msg)
            self.prev_time = current_time
            
            self.rate.sleep()
            
if __name__ == '__main__':

    rospy.init_node('pid_controller', anonymous=True)
    controller = PIDController()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass




# with classifier
# uncomment below and comment out above to run with stop sign detection



# import rospy
# import math
# from std_msgs.msg import String, Bool, Float32, Float64, Float64MultiArray
# from ackermann_msgs.msg import AckermannDriveStamped

# class PIDController:
#     def __init__(self):
#         self.rate = rospy.Rate(50)  

#         self.kp = 0.5  # tune
#         self.ki = 0.01  # tune 
#         self.kd = 0.1  # tune
#         # PID variables
#         self.prev_error = 0.0
#         self.integral = 0.0
        
#         self.max_steering_angle = 0.35  # max steering angle (in radians) add min

#         # self.desired_speed = 1.0        #  speed (m/s)
        
#         self.prev_time = None  

#         # Subscribe to the lane detection error from studentVision
#         rospy.Subscriber('lane_detection/error', Float32, self.error_callback)
        

#         self.drive_pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        
#         self.current_error = 0.0
        
#         self.drive_msg = AckermannDriveStamped()
#         self.drive_msg.header.frame_id = "f1tenth_control"
#         self.drive_msg.drive.speed = 0.5

#         self.last_steering_angle = 0.0
#         self.lane_detected = False

#         rospy.Subscriber('/stop_sign/area', Float32, self.stop_sign_callback)

#         self.stop_sign_area = None
#         self.stop_duration = rospy.Duration(3) # 3 sec stop
#         self.stop_start_time = None
#         self.stopped = False
        
    

#     #fetch error from studentVision
#     def error_callback(self, msg): 
#         self.current_error = msg.data
#         if msg.data > 1000:
#             self.lane_detected = False
#         else:
#             self.lane_detected = True
    
#     def stop_sign_callback(self, msg):
#         self.stop_sign_area = msg.data
#         if self.stop_sign_area >= 9000 and not self.stopped:
#             self.stopped = True
#             self.stop_start_time = rospy.Time.now()



#     def run(self):
#         while not rospy.is_shutdown():
#             current_time = rospy.Time.now()

#             if self.stopped:
#                 if self.stop_start_time and (current_time - self.stop_start_time) < self.stop_duration:
#                     # If within stop duration, set speed to 0
#                     self.drive_msg.drive.speed = 0.0
#                 else:
#                     # keep moving after 3 sec
#                     self.stopped = False
#                     self.drive_msg.drive.speed = 0.5  # Reset speed to default
#                 self.drive_pub.publish(self.drive_msg)
#                 self.rate.sleep()
#                 continue

#             if self.prev_time is None:
#                 delta_time = 0.0
#             else:
#                 delta_time = (current_time - self.prev_time).to_sec()

#             if delta_time == 0:
#                 delta_time = 0.0001
            
#             error = self.current_error
#             self.integral += error * delta_time
#             derivative = (error - self.prev_error) / delta_time
            
#             steering_angle = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
            
#             steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))
#             self.prev_error = error

            
#             # publish the drive message
#             self.last_steering_angle = steering_angle
#             print(steering_angle)
#             self.drive_msg.header.stamp = current_time
#             self.drive_msg.drive.steering_angle = -steering_angle
#             self.drive_pub.publish(self.drive_msg)
#             self.prev_time = current_time
            
#             self.rate.sleep()
            
# if __name__ == '__main__':

#     rospy.init_node('pid_controller', anonymous=True)
#     controller = PIDController()
#     try:
#         controller.run()
#     except rospy.ROSInterruptException:
#         pass
