import json
import requests

server = "http://127.0.0.1:5000"

# Get Rides
r = requests.get(server + "/rides")
print(r.text)

# Get a Ride
r = requests.get(server + "/rides" + "/testRideID")
print(r.text)

# Cancel a Ride
r = requests.delete(server + "/rides" + "/testRideID")
print(r.text)

# Confirm Arrival
r = requests.post(server + "/confirmArrival")
print(r.text)

# Confirm Pickup
r = requests.post(server + "/confirmPickup")
print(r.text)


""" 
# Alex's depricated test for addRide
ride_details = {
    'riderName': "test",
    'riderLat': 10.0,
    'riderLon': 11.0,
    'destLat': 12.0,
    'destLon': 13.0,
    'time': 1283773
}
r = requests.post("http://127.0.0.1:5000/addRide", json=ride_details)
 """

