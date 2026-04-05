import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO


class AutonomousTracker(Node):
    def __init__(self):
        super().__init__("autonomous_tracker")

        # 1. The Eyes: Subscribe to the camera feed
        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )

        # 2. The Muscles: Publish velocity commands to MAVROS
        self.vel_publisher = self.create_publisher(
            Twist, "/mavros/setpoint_velocity/cmd_vel_unstamped", 10
        )

        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")

        # 3. The Brain: Proportional Gain (Kp)
        # This controls how aggressively the drone chases the target.
        # Start small so it doesn't violently jerk out of the sky!
        self.Kp = 0.005

        self.get_logger().info("🎯 AEROTHON Target Lock Online. Waiting for visuals...")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width, _ = cv_image.shape

            # Camera Center Crosshairs
            center_x, center_y = int(width / 2), int(height / 2)

            results = self.model(cv_image, verbose=False)

            # Default state: Hover perfectly still
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            if len(results[0].boxes) > 0:
                # Lock onto the first object YOLO sees
                box = results[0].boxes[0].xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = map(int, box)

                # Calculate the center of the object
                obj_cx = int((x1 + x2) / 2)
                obj_cy = int((y1 + y2) / 2)

                # HUD: Draw targeting UI
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(cv_image, (obj_cx, obj_cy), 5, (0, 0, 255), -1)
                cv2.line(
                    cv_image, (center_x, center_y), (obj_cx, obj_cy), (255, 0, 0), 2
                )  # Targeting laser

                # Calculate the Error (Distance between crosshairs and object)
                err_x = obj_cx - center_x
                err_y = obj_cy - center_y

                # Convert Pixel Error to Drone Velocity
                # (Note: In ROS, +X is forward, +Y is left)
                twist.linear.x = -float(err_y) * self.Kp  # Pitch Forward/Back
                twist.linear.y = -float(err_x) * self.Kp  # Roll Left/Right

                self.get_logger().info(
                    f"Tracking locked: Pitch {twist.linear.x:.2f} | Roll {twist.linear.y:.2f}"
                )

            # Transmit velocity to the flight controller
            self.vel_publisher.publish(twist)

            # HUD: Draw Camera Center
            cv2.circle(cv_image, (center_x, center_y), 5, (0, 255, 255), -1)
            cv2.imshow("AEROTHON Tracking HUD", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Matrix glitch: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousTracker()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
