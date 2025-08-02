import rclpy
import time
import threading
import re
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import cv2
from cv_bridge import CvBridge
from flask import Flask, Response

# Global frame buffers organized by module and camera
camera_modules = {}  # {module_name: {camera_position: frame}}

class DebugServer(Node):
    def __init__(self):
        super().__init__('debug_server_node')
        
        # Initialize global camera_modules
        global camera_modules
        camera_modules = {}
        
        self.declare_parameter('web_port', 8081)
        self.web_port = self.get_parameter('web_port').get_parameter_value().integer_value
        
        self.qos_profile = qos_profile_sensor_data
        self.bridge = CvBridge()
        
        # Discover camera topics and organize them by modules
        self.discover_and_setup_cameras()

    def discover_and_setup_cameras(self):
        """Discover camera topics and organize them by modules (front, back, etc.)"""
        global camera_modules
        
        topic_list = self.get_topic_names_and_types()
        
        self.get_logger().info(f'Discovered topics: {topic_list}')
        
        # Pattern to extract module and camera position from topic names
        # Examples: /front/camera_left/image -> module: front, position: left
        #           /front/camera_right/image -> module: front, position: right
        #           /front/camera_left/image/compressed -> module: front, position: left (compressed)
        #           /back/camera_left/image -> module: back, position: left
        #           /back/camera_right/image -> module: back, position: right
        camera_pattern = re.compile(r'/(back|front|left|right|top|bottom)/camera_(left|right)/image(?:/compressed)?')
        
        for topic_name, topic_types in topic_list:
            is_compressed = 'sensor_msgs/msg/CompressedImage' in topic_types
            is_uncompressed = 'sensor_msgs/msg/Image' in topic_types
            
            if is_compressed or is_uncompressed:
                match = camera_pattern.match(topic_name)
                self.get_logger().info(f"Checking topic: {topic_name} (compressed: {is_compressed})")
                if match:
                    module, position = match.groups()
                    # Initialize module if not exists
                    if module not in camera_modules:
                        camera_modules[module] = {}
                    
                    # Initialize position frame buffer
                    camera_modules[module][position] = None
                    
                    # Create subscription based on image type
                    if is_compressed:
                        callback = self.create_compressed_camera_callback(module, position)
                        subscription = self.create_subscription(
                            CompressedImage, 
                            topic_name, 
                            callback, 
                            self.qos_profile
                        )
                        self.get_logger().info(f'Subscribed to COMPRESSED {topic_name} -> module: {module}, position: {position}')
                    else:
                        callback = self.create_camera_callback(module, position)
                        subscription = self.create_subscription(
                            Image, 
                            topic_name, 
                            callback, 
                            self.qos_profile
                        )
                        self.get_logger().info(f'Subscribed to UNCOMPRESSED {topic_name} -> module: {module}, position: {position}')
        
        if not camera_modules:
            self.get_logger().warn('No camera topics found with recognized naming pattern')
        else:
            self.get_logger().info(f'Discovered modules: {list(camera_modules.keys())}')

    def create_camera_callback(self, module, position):
        """Create a callback function for a specific camera (uncompressed)"""
        def callback(msg):
            global camera_modules
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            frame_id = msg.header.frame_id
            time_0 = time.time()
            
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                if frame is None:
                    self.get_logger().error(f"Can't decode {module}_{position} frame")
                    return
                
                camera_modules[module][position] = frame
                
            except Exception as e:
                self.get_logger().error(f"{module}_{position} callback error: {e}")
            
            delta_receive = round((time.time() - timestamp) * 1000, 2)
            delta_process = round((time.time() - time_0) * 1000, 2)
            self.get_logger().debug(f"{module.upper()}_{position.upper()} {frame_id}, R: {delta_receive}ms, P: {delta_process}ms")
        
        return callback

    def create_compressed_camera_callback(self, module, position):
        """Create a callback function for a specific camera (compressed)"""
        def callback(msg):
            global camera_modules
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            frame_id = msg.header.frame_id
            time_0 = time.time()
            
            try:
                # Decode compressed image
                np_arr = np.frombuffer(msg.data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if frame is None:
                    self.get_logger().error(f"Can't decode compressed {module}_{position} frame")
                    return
                
                camera_modules[module][position] = frame
                
            except Exception as e:
                self.get_logger().error(f"{module}_{position} compressed callback error: {e}")
            
            delta_receive = round((time.time() - timestamp) * 1000, 2)
            delta_process = round((time.time() - time_0) * 1000, 2)
            self.get_logger().debug(f"{module.upper()}_{position.upper()} COMPRESSED {frame_id}, R: {delta_receive}ms, P: {delta_process}ms")
        
        return callback

# MJPEG Flask server
app = Flask(__name__)

def generate_camera_mjpeg(module, position):
    """Generate MJPEG stream for a specific camera"""
    global camera_modules
    while True:
        if (module in camera_modules and 
            position in camera_modules[module] and 
            camera_modules[module][position] is not None):
            
            ret, jpeg = cv2.imencode('.jpg', camera_modules[module][position])
            if not ret:
                continue
            frame = jpeg.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.03)  # ~30 FPS

def generate_module_stereo_mjpeg(module):
    """Generate combined MJPEG stream for all cameras in a module"""
    global camera_modules
    while True:
        if module in camera_modules:
            frames = []
            # Collect all available frames for this module
            for position in sorted(camera_modules[module].keys()):
                if camera_modules[module][position] is not None:
                    frames.append(camera_modules[module][position])
            
            if frames:
                # Combine frames horizontally
                if len(frames) == 1:
                    combined_frame = frames[0]
                else:
                    combined_frame = np.hstack(frames)
                
                ret, jpeg = cv2.imencode('.jpg', combined_frame)
                if not ret:
                    continue
                frame = jpeg.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.03)  # ~30 FPS

@app.route('/')
def index():
    """Homepage showing all available camera modules"""
    global camera_modules
    
    if not camera_modules:
        return '<h1>Camera Debug Server</h1><p>No camera modules detected</p>'
    
    html = '<h1>Camera Debug Server</h1><h2>Available Camera Modules:</h2>'
    
    for module in sorted(camera_modules.keys()):
        positions = sorted(camera_modules[module].keys())
        html += f'<div style="margin-left: 20px; margin-right: 20px; padding: 20px; border: 1px solid #ccc;">'
        html += f'<h3>{module.upper()} Module</h3>'
        html += f'<p>Detected cameras: {", ".join(positions)}</p>'
        
        # Individual camera links
        for position in positions:
            html += f'<a href="/{module}/{position}" style="margin-right: 10px;">{position.capitalize()}</a>'
        
        # Combined view link
        if len(positions) > 1:
            html += f'<a href="/{module}/stereo" style="margin-left: 20px;"><strong>Combined View</strong></a>'
        
        html += '</div>'
    
    return html

@app.route('/<module>/<position>')
def camera_view(module, position):
    """Show individual camera view"""
    global camera_modules
    
    if module not in camera_modules or position not in camera_modules[module]:
        return f'<h1>Camera not found</h1><p>Module: {module}, Position: {position}</p><a href="/">Back to home</a>'
    
    return f'''
    <h1>{module.upper()} - {position.capitalize()} Camera</h1>
    <img src="/{module}/{position}/feed" width="640px">
    <p><a href="/">Back to home</a> | <a href="/{module}/stereo">Combined view</a></p>
    '''

@app.route('/<module>/stereo')
def module_stereo_view(module):
    """Show combined view of all cameras in a module"""
    global camera_modules
    
    if module not in camera_modules:
        return f'<h1>Module not found: {module}</h1><a href="/">Back to home</a>'
    
    positions = sorted(camera_modules[module].keys())
    return f'''
    <h1>{module.upper()} - Combined View</h1>
    <p>Cameras: {", ".join(positions)}</p>
    <img src="/{module}/stereo/feed" width="1280px">
    <p><a href="/">Back to home</a></p>
    '''

@app.route('/<module>/<position>/feed')
def camera_feed(module, position):
    """MJPEG feed for individual camera"""
    global camera_modules
    
    if module not in camera_modules or position not in camera_modules[module]:
        return "Camera not found", 404
    
    return Response(generate_camera_mjpeg(module, position),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/<module>/stereo/feed')
def module_stereo_feed(module):
    """MJPEG feed for combined module view"""
    global camera_modules
    
    if module not in camera_modules:
        return "Module not found", 404
    
    return Response(generate_module_stereo_mjpeg(module),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def flask_thread(port):
    app.run(host='0.0.0.0', port=port, debug=False, threaded=True)

def main(args=None):
    rclpy.init(args=args)

    ros_node = DebugServer()

    # Start Flask server in separate thread
    flask_threading = threading.Thread(target=flask_thread, args=(ros_node.web_port,), daemon=True)
    flask_threading.start()

    ros_node.get_logger().info(f'Web server started on http://0.0.0.0:{ros_node.web_port}')
    
    global camera_modules
    if camera_modules:
        ros_node.get_logger().info(f'Available modules: {list(camera_modules.keys())}')

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
