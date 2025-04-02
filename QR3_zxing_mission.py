# Import necessary libraries
import cv2
import numpy as np
from pymavlink import mavutil
import time

# Configuration Variables
COM_PORT = 'COM8'
BAUD_RATE = 57600
SERVO_CHANNEL = 9
SERVO_PWM_FORWARD = 2000
SERVO_PWM_CLOSE = 1100
SERVO_PWM_STOP = 1500
LATITUDE = 10.0524451
LONGITUDE = 76.620354
VIDEO_SOURCE = 'rtsp://192.168.144.25:8554/main.264'
 
# Connect to Drone
def connect_to_drone():
    print(f'Connecting to drone on {COM_PORT}...')
    try:
        master = mavutil.mavlink_connection(COM_PORT, baud=BAUD_RATE)
        master.wait_heartbeat()
        print(f'✅ Drone connected. Heartbeat from system {master.target_system}, component {master.target_component}')
        return master
    except Exception as e:
        print(f'❌ Connection failed: {e}')
        return None

def clear_mission(master):
    print('🗑️ Clearing previous mission...')
    master.waypoint_clear_all_send()
    time.sleep(1)

# Create mission waypoints
def create_mission():
    return [
        (0, 2, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 5),  # Takeoff to 10m
        (1, 2, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 10.052, 76.621, 5),  # Waypoint 1
        (2, 2, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 10.053, 76.622, 0),  # Waypoint 2
        # (3, 3, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0),  # RTL
    ]
# Upload mission to FC
def upload_mission(master, mission):
    print('📤 Uploading new mission...')
    master.waypoint_count_send(len(mission))
    for i, wp in enumerate(mission):
        seq, frame, command, p1, p2, p3, lat, lon, alt = wp
        master.mav.mission_item_int_send(
            master.target_system, master.target_component, seq, frame, command, 0, 1, 
            p1, p2, p3, int(lat * 1e7), int(lon * 1e7), alt
        )
        msg = master.recv_match(type='MISSION_REQUEST', blocking=True)
        print(f'✅ Waypoint {seq} sent!')

    print('✅ Mission upload complete!')
    
# Set mode to AUTO and start mission
def start_mission(master):
    print('🚀 Switching to AUTO mode and starting mission...')
    master.set_mode('AUTO')
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print('✅ Mission started!')
# Control Servo
def set_servo(master, channel, pwm_value):
    print(f'🔧 Moving servo on channel {channel} to {pwm_value} PWM')
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, channel, pwm_value, 0, 0, 0, 0, 0
    )
    print('✅ Servo moved.')

# Drone Mode Control
def set_guided_mode(master):
    print('🛫 Switching to GUIDED mode...')
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4  # 4 represents GUIDED mode
    )
    print('✅ Mode set to GUIDED.')

# Arm and Takeoff
# def arm_and_takeoff(master, altitude=5):
#     print('🕹️ Arming drone...')
#     master.mav.command_long_send(
#         master.target_system, master.target_component,
#         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#         0, 1, 0, 0, 0, 0, 0, 0
#     )
#     print('✅ Drone armed.')
#     time.sleep(4)
#     print(f'🚀 Taking off to {altitude} meters...')
#     master.mav.command_long_send(
#         master.target_system, master.target_component,
#         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
#         0, 0, 0, 0, 0, 0, altitude, 0
#     )
#     time.sleep(10)

# Return to Launch
def return_to_launch(master):
    print('🚀 Sending Return to Launch (RTL) command...')
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print('✅ RTL command sent.')

# Perform Landing
def land_at_coordinates(drone):
    print('🛬 Sending LAND command to given coordinates...')
    drone.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        0, drone.target_system, drone.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        int(0b110111111000), int(LATITUDE * 1e7), int(LONGITUDE * 1e7), 0, 0, 0, 0, 0, 0, 0, 0, 0
    ))

    drone.mav.command_long_send(
        drone.target_system, drone.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print('✅ Land command sent.')
    time.sleep(10)
def set_waypoint(master, latitude, longitude, altitude):
    print(f'📍 Sending waypoint to Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}m')

    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        0,  # Time_boot_ms (set to 0 for now)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        int(0b110111111000),  # Control position, no yaw or velocity
        int(latitude * 1e7),
        int(longitude * 1e7),
        altitude,
        0, 0, 0,  # Velocity X, Y, Z
        0, 0, 0,  # Acceleration X, Y, Z
        0.0,      # Yaw
        0.0       # Yaw Rate (This was missing)
    ))

    print('✅ Waypoint command sent.')

def get_home_location(master):
    """Retrieve the home location (initial GPS coordinates)."""
    print('📍 Fetching home location...')
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    
    while True:
        msg = master.recv_match(type='HOME_POSITION', blocking=True)
        if msg:
            lat = msg.latitude / 1e7
            lon = msg.longitude / 1e7
            print(f'🏠 Home Location: Latitude={lat}, Longitude={lon}')
            return lat, lon
def arm_drone(master):
    print('🕹️ Arming drone...')
    
    # Send arm command
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print('✅ Arm command sent. Waiting for confirmation...')
    #time.sleep(2)

    # Confirm drone is armed
    while True:
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 1, 1
        )
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print('✅ Drone confirmed as armed.')
            break
        print('⏳ Waiting for drone to arm...')
        #time.sleep(1)        
def navigate_to_location(master, lat, lon, altitude=10):
    """Navigate the drone to a specified GPS location."""
    print(f'✈️ Navigating to GPS coordinates: {lat}, {lon}')
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        int(0b110111111000), int(lat * 1e7), int(lon * 1e7), altitude, 0, 0, 0, 0, 0, 0, 0, 0
    ))

def return_to_home(master, home_lat, home_lon):
    """Return to the home location at a 5m altitude."""
    print('🚀 Returning to home location at 5m altitude...')
    navigate_to_location(master, home_lat, home_lon, altitude=5)
    print('✅ Waypoint navigation to home initiated at 5m altitude.')

# Monitor Altitude and Close Servo
def monitor_altitude_and_close_servo(master):
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            altitude = msg.relative_alt / 1000.0  # Convert mm to meters
            print(f'📏 Current Altitude: {altitude}m')
            if altitude <= 1.0:
                print('🔧 Closing servo as altitude is 1m')
                set_servo(master, SERVO_CHANNEL, SERVO_PWM_CLOSE)
                break
        time.sleep(0.5)

# Main Function
def main():
    cap = cv2.VideoCapture(VIDEO_SOURCE)
    if not cap.isOpened():
        print('❌ Error: Unable to open video stream.')
        return

    qr_detector = cv2.QRCodeDetector()
    drone = connect_to_drone()
    if not drone:
        return
    # set_guided_mode(drone)
    # arm_and_takeoff(drone, altitude=4)
    # print('⏳ Waiting for 5 seconds after landing...')
    # time.sleep(5)


    while True:
        ret, frame = cap.read()
        if not ret:
            print('❌ Failed to read frame')
            continue

        data, bbox, _ = qr_detector.detectAndDecode(frame)

        if bbox is not None and len(bbox) > 0:
            bbox = bbox.astype(int)
            for i in range(len(bbox)):
                pt1 = tuple(bbox[i][0])
                pt2 = tuple(bbox[(i + 1) % len(bbox)][0])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

            if data:
                print(f'✅ QR Code detected: {data}')
                cv2.putText(frame, f"QR: {data}", (bbox[0][0][0], bbox[0][0][1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                home_lat, home_lon = get_home_location(drone)
                print("home location and lattitude")
                print(home_lat,home_lon)
                # arm_and_takeoff(drone, altitude=4)
                # print('⏳ Waiting for 5 seconds after landing...')
                # time.sleep(5)

                land_at_coordinates(drone)
                print('⏳ Waiting for 5 seconds after landing...')
                time.sleep(5)

                print('🔄 Rotating servo forward and back...')
                set_servo(drone, SERVO_CHANNEL, SERVO_PWM_FORWARD)
                time.sleep(0.8)
                set_servo(drone, SERVO_CHANNEL, SERVO_PWM_STOP)
                #time.sleep(5)
                set_servo(drone, SERVO_CHANNEL, SERVO_PWM_CLOSE)
                time.sleep(0.5)
                set_servo(drone, SERVO_CHANNEL, SERVO_PWM_STOP)
                time.sleep(5)
                clear_mission(drone)
                mission = create_mission()
                upload_mission(drone, mission)
                set_guided_mode(drone)
                arm_drone(drone)
                time.sleep(2)
                start_mission(drone)
                monitor_altitude_and_close_servo(drone)
                
                # time.sleep(3)
                # arm_and_takeoff(drone, altitude=4)
                # #set_waypoint(drone, 10.04902393, 7.3432342, 3)
                # return_to_home(drone, home_lat, home_lon)
                break

        cv2.imshow('QR Code Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    if drone:
        drone.close()
        print('🔌 Drone connection closed.')

if __name__ == '__main__':
    main()
