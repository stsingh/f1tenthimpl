
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32

class PIDController:
    def __init__(self):
        rospy.init_node('pid_controller', anonymous=True)
        
        self.kp = 0.5  # tune
        self.ki = 0.0  # tune 
        self.kd = 0.1  # tune
        
        # PID variables
        self.prev_error = 0.0
        self.integral = 0.0
        
        self.max_steering_angle = 0.34  # max steering angle (in radians)
        self.desired_speed = 1.0        #  speed (m/s)
        
        self.prev_time = None

        #speeds (will need to tune hella) (in m/s)
        self.max_velocity = 1.5 
        self.mid_velocity = 1.0 
        self.min_velocity = 0.5        

        # Subscribe to the lane detection error from studentVision
        rospy.Subscriber('lane_detection/error', Float32, self.error_callback) # sahej error publish topic
        
        self.drive_pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=10) # need to figure out the structure of this topic, does it just take steering angles?
        
        self.current_error = 0.0
        
        self.rate = rospy.Rate(10)  
    
    def error_callback(self, msg):
        self.current_error = msg.data
    
    def get_velocity(self, steering_angle):
        if abs(steering_angle) <= 0.1:
            return self.max_velocity
        elif abs(steering_angle) <= 0.2:
            return self.mid_velocity
        else:
            return self.min_velocity

    def run(self):
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            if self.prev_time is None:
                delta_time = 0.0
            else:
                delta_time = (current_time - self.prev_time).to_sec()

            if delta_time == 0:
                delta_time = 0.0001
            
            # PID calculations
            error = self.current_error
            self.integral += error * delta_time
            derivative = (error - self.prev_error) / delta_time
            
            steering_angle = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
            
            steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))
            
            # publish the drive message
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = current_time
            drive_msg.header.frame_id = "base_link"
            drive_msg.drive.steering_angle = steering_angle
            drive_msg.drive.speed = self.desired_speed
            
            self.drive_pub.publish(drive_msg)
            
            self.prev_error = error
            self.prev_time = current_time
            
            self.rate.sleep()
            
if __name__ == '__main__':
    try:
        controller = PIDController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
