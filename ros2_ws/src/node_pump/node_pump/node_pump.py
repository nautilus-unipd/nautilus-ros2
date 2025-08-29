import rclpy
from rclpy.node import Node
from custom_pump_msgs.msg import Pump
from rclpy.qos import QoSProfile, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

import requests
import time
import math
from gpiozero import PWMOutputDevice 

"""
Node that controls the pumps for the BlueBoat.
"""
class NodePumpCtrl(Node):
    """
    Define class' constants
    """
    # Output limits
    MIN_OUTPUT = -1.0
    MAX_OUTPUT = 1.0
    # Target setpoint (change as needed)
    SETPOINT = 0.0
    # Name of the output topic
    TN_PUMP = "/output/pump"
    # Where to retrieve data
    TARGET_URL = "http://192.168.2.2:6040/v1/mavlink/vehicles/1/components/1/messages/ATTITUDE"
    # How many messages are stored in the topic queue
    QOS_DEPTH_PUMP = 5
    # PARAMETERS NAME
    PN_DEBUG = "debug"
    PN_FREQUENCY = "frequency"
    PN_KP = "kp"
    PN_KI = "ki"
    PN_KD = "kd"

    """
    Default class' constructor
    """
    def __init__(self, node_name, **kwargs):
        # Call parent's constructor
        super().__init__(node_name, **kwargs)
        # Initialize node
        self.init_parameters()
        self.init_variables()
        # Print messages
        if self.get_parameter(self.PN_DEBUG).get_parameter_value().bool_value:
            self.print_parameters()
        self.get_logger().info("[+] Pump node initialised!")
        # Start main loop
        self.main_loop()

    """
    Define node's parameters
    """
    def init_parameters(self):
        # Define parameters's description
        param_descr_debug = ParameterDescriptor(description = "Prints additional debugging informations.")
        param_descr_freq = ParameterDescriptor(description = "Frequency at which the data is retrived.")
        param_descr_kp = ParameterDescriptor(description = "Weight for the proportional gain.")
        param_descr_ki = ParameterDescriptor(description = "Weight for the integral gain.")
        param_descr_kd = ParameterDescriptor(description = "Weight for the derivative gain.")
        # Get the parameters
        self.declare_parameter(self.PN_DEBUG, False, param_descr_debug)
        self.declare_parameter(self.PN_FREQUENCY, 0.01, param_descr_freq)
        self.declare_parameter(self.PN_KP, 1.0, param_descr_kp)
        self.declare_parameter(self.PN_KI, 0.0, param_descr_ki)
        self.declare_parameter(self.PN_KD, 0.0, param_descr_kd)

    """
    Define node's variables
    """
    def init_variables(self):
        #GPIO pins for pump control circuits
        self.pump_r = PWMOutputDevice(12) #pin for transistor connected to pump pushing water to the right side tank
        self.pump_l = PWMOutputDevice(13) #pin for transistor connected to pump pushing water to the left side tank
        # Internal PID state
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = 0.0
        # Initialize QoS profile for pump publisher
        qos_profile_pump_pub = QoSProfile(depth = self.QOS_DEPTH_PUMP)
        # Create pump publisher
        self.pump_pub = self.create_publisher(Pump, self.TN_PUMP, qos_profile_pump_pub)

    """
    Print the nodes' parameters
    """
    def print_parameters(self):
        self.get_logger().info(f"[!] Debug:\t{self.get_parameter(self.PN_DEBUG).get_parameter_value().bool_value}")
        self.get_logger().info(f"[!] Frequency:\t{self.get_parameter(self.PN_FREQUENCY).get_parameter_value().double_value}")
        self.get_logger().info(f"[!] KP:\t{self.get_parameter(self.PN_KP).get_parameter_value().double_value}")
        self.get_logger().info(f"[!] KI:\t{self.get_parameter(self.PN_KI).get_parameter_value().double_value}")
        self.get_logger().info(f"[!] KD:\t{self.get_parameter(self.PN_KD).get_parameter_value().double_value}")

    """
    Function that creates a new pump output message and publishes it
    """
    def callback_pub(self, data):
        msg = Pump()
        msg.output = data
        self.pump_pub.publish(msg)
        if self.get_parameter(self.PN_DEBUG).get_parameter_value().bool_value:
            self.get_logger().info(f"[+] Published pump value: {data}")

    """
    Function to update PID values
    """
    def pid_control(self, feedback, current_time):
        # Calculate PID output 
        error = self.SETPOINT - feedback
        dt = current_time - self.last_time if self.last_time else 0.01
        self.integral += error * dt
        if dt > 0: derivative = (error - self.last_error) / dt  
        else: derivative = 0.0
        Kp = self.get_parameter(self.PN_KP).get_parameter_value().double_value
        Ki = self.get_parameter(self.PN_KI).get_parameter_value().double_value
        Kd = self.get_parameter(self.PN_KD).get_parameter_value().double_value
        output = Kp * error + Ki * self.integral + Kd * derivative
        # Clamp output
        output = max(self.MIN_OUTPUT, min(self.MAX_OUTPUT, output))  
        # Update previous values
        self.last_error = error
        self.last_time = current_time
        # Fix output to saturation and activation mosfet voltages
        if output == 0 or output == 1 or output == -1: return output
        elif output > 0 : output = (output * 0.34) + 0.31
        elif output < 0: output = (output * 0.34) - 0.31
        return output

    """
    Node's main loop
    """
    def main_loop(self):
        self.last_time = time.time()
        # WebSocket handler
        try:
            while True:
                try:
                    # Send a request to the boat
                    response = requests.get(self.TARGET_URL)
                    self.get_logger().info(f"\nTime: {time.strftime('%Y-%m-%d %H:%M:%S')} - Request to {self.TARGET_URL}")
                    self.get_logger().info(f"Status: {response.status_code}")
                    # Succesful response
                    if response.status_code == 200:
                        try:
                            data = response.json()
                            # Parsing of "message" section
                            # (i kept the pitch data collection in case in the future there will be 
                            # a need to implement this algorithm for a pitch reduction control system)
                            message = data.get("message", {})
                            pitch_rad = message.get("pitch")
                            roll_rad = message.get("roll")
                            pitchspeed = message.get("pitchspeed")
                            rollspeed = message.get("rollspeed")
                            time_boot_ms = message.get("time_boot_ms")
                            # Parsing of "status" section
                            status = data.get("status", {})
                            time_info = status.get("time", {})
                            counter = time_info.get("counter")
                            last_update = time_info.get("last_update")
                            # Printing out the data 
                            self.get_logger().info("ATTITUDE data:")
                            self.get_logger().info(f"Pitch: {pitch_rad:.3f} rad")
                            self.get_logger().info(f"Roll: {roll_rad:.3f} rad")
                            self.get_logger().info(f"Pitch speed: {pitchspeed}")
                            self.get_logger().info(f"Roll speed: {rollspeed}")
                            self.get_logger().info(f"Time boot (ms): {time_boot_ms}")
                            self.get_logger().info("Status:")
                            self.get_logger().info(f"Counter: {counter}")
                            self.get_logger().info(f"Ultimo update: {last_update}")
                        except ValueError:
                            self.get_logger().info("The response is not in a valid JSON format")
                    else:
                        self.get_logger().info(f"HTTP error: {response.status_code}")
                except requests.exceptions.RequestException as e:
                    self.get_logger().info(f"Error during request: {e}")
                # Return the pump output after retrieving the sensor data
                feedback = float(rollspeed)
                now = time.time()
                output = self.pid_control(feedback, now)
                # Debug
                self.get_logger().info(f"Feedback: {feedback:.3f}, Output: {output:.3f}")
                # Send value to pumps 
                if output>0: 
                    pump_r.value = output
                    pump_l.value = 0
                elif output<0: 
                    pump_r.value = 0
                    pump_l.value = output*-1
                # 1/freq Hz loop
                time.sleep(freq)  
        except Exception as e:
            #If the system gets interrupted, stop the pump output and print the reason of the interruption
            self.get_logger().info("\nScript interrupted")
        finally:
            pump_r.off()
            self.get_logger().info("\nSet right pump value to zero")
            pump_l.off()
            self.get_logger().info("\nSet left pump value to zero")
            self.get_logger().info("\nCause of interruption:", type(e).__name__, " - ", e)

"""
Create instance of the node and execute it
"""
def main(args = None):
    rclpy.init(args = None)
    pump_node = NodePumpCtrl("pump_node")
    rclpy.spin(pump_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
