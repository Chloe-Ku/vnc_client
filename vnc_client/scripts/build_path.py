import googlemaps_util as api
import split_path as sp
import json

# Hacky work-around to be able to import from a folder above this one.
import sys
import os

sys.path.insert(1, os.path.join(sys.path[0], '../path_generation'))
import position as p
sys.path.insert(1, os.path.join(sys.path[0], '../gps_to_odom/src'))
import coord_to_odom as c2o

import logging
logger = logging.getLogger(__name__)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(module)s - %(levelname)s - %(message)s')
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
ch.setFormatter(formatter)
logging_path = os.path.dirname(os.path.realpath(__file__)).split("src/")[0] + "log"
fh = logging.FileHandler(logging_path)
fh.setLevel(logging.DEBUG)
fh.setFormatter(formatter)
logger.addHandler(ch)
logger.addHandler(fh)
logger.setLevel(logging.DEBUG)

# 
# Generates a list of Vehicle Odometry states representing the Path a vehicle should follow
# from the specified start latitude/longitude to the specified end latitude/longitude. 
# Each index is essentially a "waypoint" along the path, with the first index in the list
# being the first waypoint and so on. Relevant Odom info at each waypoint is pose.pose.position.x, 
# pose.pose.position.y, pose.heading_angle. x and y positions are measured in meters from the 
# coordinate system origin.\
# 
# @param startLat a float value for the starting latitude
# @param startLon a float value for the starting longitude
# @param endLat a float value for the ending latitude
# @param endLon a float value for the ending longitude
# @return a List of Odom waypoints for a vehicle to follow from the start lat/lon to the end lat/lon
def buildPath(startLat, startLon, endLat, endLon):
    logger.debug("Method started")

    # Package our start and end coordinates into dictionaries
    startCoords = {
        "latitude": startLat,
        "longitude": startLon
    }

    endCoords = {
        "latitude": endLat, 
        "longitude": endLon
    }

    logger.debug("Google API request")

    # Make a call to the Google Maps API to get directions from the start to the end coordinates. 
    # This will return a list of latitude/longitude pairs that represent nodes in an edge list. 
    response = api.sendGoogleDirectionsRequest(startCoords, endCoords)

    # Extract the content field from the API response and convert the content byte string to a JSON
    contentString = response.content.decode('utf-8')
    contentJSON = json.loads(contentString)

    logger.debug("Decoded response")
    logger.debug(f"Response: {contentJSON}")

    # Select the encoded polyline from the JSON
    encodedPolyline = contentJSON['routes'][0]['overview_polyline']['points']

    logger.debug("Decoding polyline")

    # Decode the polyline to get a list of latitude/longitude pairs to follow on the path```
    decodedPolyline = api.decode_polyline(encodedPolyline)

    logger.debug("Building edges")

    # Create a list of edges in the required format for the split_path.py
    edges = []
    for i in range(len(decodedPolyline) - 1):

        newEdge = {
            "distance": 0.0,
            "vertex2": {
                "name": "",
                "latitude": decodedPolyline[i + 1][0],
                "longitude": decodedPolyline[i + 1][1] 
            }, 
            "vertex1": {
                "name": "",
                "latitude": decodedPolyline[i][0],
                "longitude": decodedPolyline[i][1]
            }
        }

        edges.append(newEdge)

    logger.debug("Splitting edges")

    #Split up any edge longer than 10 meters long. 10 meters was arbitrarily selected
    splitEdges = sp.splitEdges(edges, 10)

    logger.debug("Creating positions")

    # Use the haversine formula to convert the latitude/longitude edges into a path of discrete (x, y, theta) points
    # that represent waypoints for the Vehicle to follow
    positions = p.createPositions(splitEdges)

    logger.debug("Converting to odom")
    return c2o.convert_to_odom(positions)
