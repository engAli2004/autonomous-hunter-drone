import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from pymavlink import mavutil

class VisualServoNode(Node):
    def __init__(self):
        super().__init__('visual_servo_node')
        
        # 1. Initialize ROS 2 Vision Bridge
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt') 
        
        # 2. Connect directly to the PX4 Flight Controller via MAVLink UDP
        self.get_logger().info('Connecting to PX4 Flight Controller...')
        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
        self.master.wait_heartbeat()
        self.get_logger().info('MAVLink Flight Control Link Established!')
        
        # 3. Flight Tuning: Proportional Gain (Kp)
        # This determines how aggressive the drone flies. Higher = Faster.
        self.Kp = 0.005 

    def send_velocity_cmd(self, vx, vy, vz=0.0):
        # MAV_FRAME_BODY_NED: X is forward, Y is right, Z is down
        self.master.mav.set_position_target_local_ned_send(
            0, # time_boot_ms
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, # Bitmask: Ignore positions & accels, only use Velocity
            0, 0, 0, # Position (Ignored)
            vx, vy, vz, # Velocity in m/s
            0, 0, 0, # Acceleration (Ignored)
            0, 0) # Yaw (Ignored)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame_h, frame_w = cv_image.shape[:2]
            center_x, center_y = int(frame_w / 2), int(frame_h / 2)
            
            # Draw Static Center HUD
            cv2.line(cv_image, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 255), 2)
            cv2.line(cv_image, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 255), 2)

            results = self.model(cv_image, conf=0.3, verbose=False)
            
            # Default flight velocities (Hover in place)
            vx, vy = 0.0, 0.0 
            
            if len(results[0].boxes) > 0:
                box = results[0].boxes[0].xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
                
                target_x = int((x1 + x2) / 2)
                target_y = int((y1 + y2) / 2)
                
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(cv_image, (target_x, target_y), 5, (0, 0, 255), -1)
                cv2.line(cv_image, (center_x, center_y), (target_x, target_y), (255, 0, 0), 2)
                
                # --- THE AUTONOMOUS BRAIN ---
                error_x = target_x - center_x
                error_y = target_y - center_y
                
                # Convert Pixels to physical m/s using the Kp Gain
                # Negative error_y (top of screen) means fly FORWARD (Positive Vx)
                vx = -error_y * self.Kp
                # Positive error_x (right of screen) means fly RIGHT (Positive Vy)
                vy = error_x * self.Kp
                
                # Speed Governor: Cap the maximum speed to 2.5 m/s for safety
                vx = max(min(vx, 2.5), -2.5)
                vy = max(min(vy, 2.5), -2.5)
                
                self.get_logger().info(f'Hunting Target | Pitch(Vx): {vx:.2f} m/s, Roll(Vy): {vy:.2f} m/s')
            else:
                self.get_logger().info('Target Lost. Hovering and scanning...')

            # 4. Stream commands to PX4 at 30Hz to keep the Offboard link alive
            self.send_velocity_cmd(vx, vy)

            cv2.imshow("Drone HUD - Autonomous Hunter", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Pipeline error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
