import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
from picamera2 import Picamera2

class CaptureNode(Node):
    def __init__(self):
        super().__init__('capture_node')

        # Declare camera parameters 
        # (WARNING: These parameters should be set in the launch file or config file, they will be overridden)
        self.declare_parameter('camera_index', 0)  
        self.declare_parameter('topic_name', 'image')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('quality', 95)               # JPEG quality (0-100), higher is better quality
        self.declare_parameter('use_compression', True)     # Use compressed image transport
        self.declare_parameter('system_position', 'front')  # front, back, other
        self.declare_parameter('camera_side', 'left')       # left, right, other
        
        # Get parameters
        self.camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = float(self.get_parameter('fps').get_parameter_value().integer_value)
        self.quality = self.get_parameter('quality').get_parameter_value().integer_value
        self.use_compression = self.get_parameter('use_compression').get_parameter_value().bool_value
        self.system_position = self.get_parameter('system_position').get_parameter_value().string_value
        self.camera_side = self.get_parameter('camera_side').get_parameter_value().string_value

        # Initialize camera 
        self.camera = Picamera2(self.camera_index)
        config = self.camera.create_preview_configuration(
            main={"format": 'XRGB8888', "size": (self.width, self.height)}
        )
        self.camera.configure(config)
        
        # Set camera controls for quality
        controls = {
            "FrameRate": self.fps,
        }
        
        # Apply quality-related controls if supported
        if self.quality < 50:
            # Lower quality - prioritize speed
            controls.update({
                "Brightness": 0.0,
                "Contrast": 1.0,
            })
        elif self.quality >= 50 and self.quality < 80:
            # Medium quality
            controls.update({
                "Brightness": 0.0,
                "Contrast": 1.2,
            })
        else:
            # High quality - prioritize image quality
            controls.update({
                "Brightness": 0.1,
                "Contrast": 1.3,
            })
        
        self.camera.set_controls(controls)
        self.camera.start()

        # Declare publisher for camera images
        if self.use_compression:
            self.publisher = self.create_publisher(CompressedImage, topic_name, 10)
        else:
            self.publisher = self.create_publisher(Image, topic_name, 10)

        # Define timer period for publishing images
        timer_period = 1.0 / self.fps # convert fps to period (10 fps = 1/10 seconds = every 0.1 seconds) 
        self.timer = self.create_timer(timer_period, self.publish_image)
        
        # Initialize CvBridge for converting OpenCV images to ROS messages
        self.bridge = CvBridge()


    def publish_image(self):
        try:
            # Capture frame using Picamera2
            cv_image = self.camera.capture_array()
            
            if cv_image is not None:
                # Convert from RGB to BGR for OpenCV/ROS compatibility
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
                
                if self.use_compression:
                    # Create compressed image message
                    compress_msg = CompressedImage()
                    compress_msg.header.stamp = self.get_clock().now().to_msg()
                    compress_msg.header.frame_id = f"camera_{self.system_position}_{self.camera_side}_frame"
                    compress_msg.format = "jpeg"
                    
                    # Apply JPEG compression
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
                    _, encoded_img = cv2.imencode('.jpg', cv_image, encode_param)
                    compress_msg.data = encoded_img.tobytes()
                    
                    self.publisher.publish(compress_msg)
                else:
                    # Publish uncompressed image
                    ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                    ros_image.header.stamp = self.get_clock().now().to_msg()
                    ros_image.header.frame_id = f"camera_{self.system_position}_{self.camera_side}_frame"
                    
                    self.publisher.publish(ros_image)
                
        except Exception as e:
            self.get_logger().error(f'Failed to capture image from camera: {e}')

    def destroy_node(self):
        if hasattr(self, 'camera'):
            self.camera.stop()
            self.camera.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    capture_node = CaptureNode()
    try:
        rclpy.spin(capture_node)
    except KeyboardInterrupt:
        pass
    finally:
        capture_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


            

      #  ret, cv_image = self.camera.read()
      #  if not ret or cv_image is None:
      #      self.get_logger().error('Failed to capture image from camera.')
      #      return
      #  
      #  ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
      #  ros_image.header.stamp = self.get_clock().now().to_msg()
      #  ros_image.header.frame_id = "camera_frame"
      #  
      #  self.publisher.publish(ros_image)

       
       # self.camera = cv2.VideoCapture(0)
       # if self.camera.isOpened():
       #     self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
       #     self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
       #     self.camera.set(cv2.CAP_PROP_FPS, self.fps)
            
       # cv_image = cv2.imread(self.image_path)
        
               
        # self.get_logger().info('Publishing: %s', ros_image.header.stamp)


