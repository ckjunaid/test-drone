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
def create_mission(home_lat, home_lon):
    return [
        (0, 2, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 5),  # Takeoff to 5m
        (1, 2, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, home_lat, home_lon, 5),  # Navigate to home location
        (2, 2, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, home_lat, home_lon, 0),  # Land at home location
    ]

# Upload mission to FC
def upload_mission(master, mission):
    print('📤 Uploading new mission...')
    master.waypoint_count_send(len(mission))
    for i, wp in enumerate(mission):
        seq, frame, command, p1, p2, p3, p4, lat, lon, alt = wp
        master.mav.mission_item_int_send(
            master.target_system, master.target_component, seq, frame, command, 0, 1, 
            p1, p2, p3, p4, int(lat * 1e7), int(lon * 1e7), alt
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

# Get Home Location
def get_home_location(master):
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

# Set Servo Position
def set_servo(master, channel, pwm_value):
    print(f'🔧 Moving servo on channel {channel} to {pwm_value} PWM')
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, channel, pwm_value, 0, 0, 0, 0, 0
    )
    print('✅ Servo moved.')

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
                home_lat, home_lon = get_home_location(drone)
                print("Home location latitude and longitude:", home_lat, home_lon)

                clear_mission(drone)
                mission = create_mission(home_lat, home_lon)
                upload_mission(drone, mission)
                start_mission(drone)
                monitor_altitude_and_close_servo(drone)
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
