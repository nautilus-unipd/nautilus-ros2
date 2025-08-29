import requests
import time
import math
from gpiozero import PWMOutputDevice 

#GPIO pins for pump control circuits
pump_r = PWMOutputDevice(12) #pin for transistor connected to pump pushing water to the right side tank
pump_l = PWMOutputDevice(13) #pin for transistor connected to pump pushing water to the left side tank

# PID Parameters
Kp = 1.00
Ki = 0.00
Kd = 0.00

# Output limits
MIN_OUTPUT = -1.0
MAX_OUTPUT = 1.0

# Target setpoint (change as needed)
SETPOINT = 0.0

# Internal PID state
integral = 0.0
last_error = 0.0

# Frequency parameter
freq = 0.01

# PID update function
def pid_control(feedback, current_time):
    
    global integral, last_error, last_time

    # Calculate PID output 
    error = SETPOINT - feedback
    dt = current_time - last_time if last_time else 0.01

    integral += error * dt

    if dt > 0: derivative = (error - last_error) / dt  
    else: derivative = 0.0

    output = Kp * error + Ki * integral + Kd * derivative

    # Clamp output
    output = max(MIN_OUTPUT, min(MAX_OUTPUT, output))  
    
    # Update previous values
    last_error = error
    last_time = current_time
    
    # Fix output to saturation and activation mosfet voltages
    if output == 0 or output == 1 or output == -1: return output
    elif output > 0 : output = (output * 0.34) + 0.31
    elif output < 0: output = (output * 0.34) - 0.31
    return output


def main():

    global last_time
    last_time = time.time()
    target_url = "http://192.168.2.2:6040/v1/mavlink/vehicles/1/components/1/messages/ATTITUDE"

    # WebSocket handler
    try:
        while True:
            try:

                # Send a request to the boat
                response = requests.get(target_url)
                print(f"\nTime: {time.strftime('%Y-%m-%d %H:%M:%S')} - Request to {target_url}")
                print(f"Status: {response.status_code}")

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
                        print("ATTITUDE data:")
                        print(f"Pitch: {pitch_rad:.3f} rad")
                        print(f"Roll: {roll_rad:.3f} rad")
                        print(f"Pitch speed: {pitchspeed}")
                        print(f"Roll speed: {rollspeed}")
                        print(f"Time boot (ms): {time_boot_ms}")

                        print("Status:")
                        print(f"Counter: {counter}")
                        print(f"Ultimo update: {last_update}")

                    except ValueError:
                        print("The response is not in a valid JSON format")
                else:
                    print(f"HTTP error: {response.status_code}")

            except requests.exceptions.RequestException as e:
                print(f"Error during request: {e}")
            
            # Return the pump output after retrieving the sensor data
            feedback = float(rollspeed)
            now = time.time()
            output = pid_control(feedback, now)

            # Debug
            print(f"Feedback: {feedback:.3f}, Output: {output:.3f}")
            
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
        print("\nScript interrupted")
    finally:
        pump_r.off()
        print("\nSet right pump value to zero")
        pump_l.off()
        print("\nSet left pump value to zero")
        print("\nCause of interruption:", type(e).__name__, " - ", e)



if __name__ == '__main__':

    main()
