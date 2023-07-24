import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import math

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lonreturn 
    math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def distance_to_current_waypoint(awp):
    distancetopoint = get_distance_metres(vehicle.location.global_frame, awp)
    return distancetopoint

# Connect to the PX4 vehicle
#connection_string = 'udp:127.0.0.1:14550'
#vehicle = connect(connection_string, wait_ready=True)
vehicle = connect('udp:127.0.0.1:14550')
print("Connected!")

# Set the vehicle mode to GUIDED
vehicle.mode = VehicleMode("GUIDED")

# Arm the vehicle
vehicle.armed = True
while not vehicle.armed:
    print("Waiting for vehicle to arm...")
    time.sleep(1)
vehicle.simple_takeoff(10)

# Wait for the drone to reach a certain altitude
while True:
    altitude = vehicle.location.global_relative_frame.alt
    if altitude >= 9.5:
        break
    time.sleep(1)

# Define the mission waypoints
waypoints = [
    LocationGlobalRelative(-35.363261, 149.165230, 10),
    LocationGlobalRelative(-35.362933, 149.164652, 10),
    LocationGlobalRelative(-35.363275, 149.164340, 10),
    LocationGlobalRelative(-35.363700, 149.164889, 10)
]

# Set the target waypoint with a varying altitude
target_altitude = waypoints.index(waypoint) * 5 + 10
target_location = LocationGlobalRelative(waypoint.lat, waypoint.lon, target_altitude)
vehicle.simple_goto(target_location)

# Wait for the vehicle to reach the waypoint
while True:
    current_pos = vehicle.location.global_relative_frame
    dist = current_pos.distance_to(target_location)
    if dist < 1:
        break
    time.sleep(1)

# Set the vehicle mode to LAND
vehicle.mode = VehicleMode("LAND")

# Wait for the vehicle to land
while vehicle.armed:
    print("Waiting for vehicle to land...")
    time.sleep(1)

# Disconnect from the vehicle
vehicle.close()
