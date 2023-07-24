from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

# Connect to the vehicle
vehicle = connect('udp:127.0.0.1:14550')

# Arm and take off
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
vehicle.simple_takeoff(10)

# Wait for the drone to reach a certain altitude
while True:
    altitude = vehicle.location.global_relative_frame.alt
    if altitude >= 9.5:  # target altitude - 0.5 meters
        break
    time.sleep(1)

# Define the mission waypoints
waypoints = [
    LocationGlobalRelative(37.793105, -122.398768, 20),
    LocationGlobalRelative(37.793109, -122.398824, 20),
    LocationGlobalRelative(37.793095, -122.398857, 20),
    LocationGlobalRelative(37.793057, -122.398843, 20),
    LocationGlobalRelative(37.793042, -122.398797, 20),
    LocationGlobalRelative(37.793050, -122.398751, 20),
    LocationGlobalRelative(37.793084, -122.398722, 20),
    LocationGlobalRelative(37.793119, -122.398724, 20)
]

# Fly the mission
for wp in waypoints:
    vehicle.simple_goto(wp)
    while True:
        distance = vehicle.location.global_relative_frame.distance_to(wp)
        if distance <= 1:  # target radius in meters
            break
        time.sleep(1)

# Land the drone
vehicle.mode = VehicleMode("LAND")

# Close the connection
vehicle.close()
