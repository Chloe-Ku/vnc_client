import math
import json
import requests
import utm
import geopy.distance

# Hacky work-around to be able to import from a folder above this one.
import sys
import os
# sys.path.insert(1, os.path.join(sys.path[0], '../databases'))
# from sqldb import SQLDB
# from neodb import NeoDB


def sliceEdge(edge, separation):
    """
    Method for splitting one edge into pieces separation apart
    With format:

    "edges":[
             {
                "distance":99.41809404247414,
                "vertex2":{
                   ...
                   "latitude":35.769412931521394,
                   "longitude":-78.6758533587986
                },
                "vertex1":{
                   ...
                   "latitude":35.7701072204963,
                   "longitude":-78.67515772981655
                }
             }

    Parameters
    ----------
    edge: JSON edges object with distance & two vertices
    
    separation: int distance between chunks of edge

    Returns
    -------
    vertexList: Array of JSON vertex points
    """

    # Assuming vertex 1 is starting and 2 is ending
    # print(edge)
    start = edge['vertex1']
    end = edge['vertex2']
    distance = edge['distance']

    vertexList = []
    vertexList.append(start)

    # Getting slope of the edge, and the angle between different chunk points
    endSameLat = (start["latitude"], end["longitude"])
    endSameLong = (end["latitude"], start["longitude"])
    startCoods = (start["latitude"], start["longitude"])
    endCoods = (end["latitude"], end["longitude"])

    distance = geopy.distance.distance(startCoods, endCoods).m
    latitudeMeters = geopy.distance.distance(startCoods, endSameLat).m
    longitudeMeters = geopy.distance.distance(startCoods, endSameLong).m
    changeLatitude = (end["latitude"] - start["latitude"])
    changeLongitude = (end["longitude"] - start["longitude"])

    if changeLatitude == 0 or latitudeMeters == 0:
        changeLatitudePerMeter = 0
    else:
        changeLatitudePerMeter = changeLatitude / latitudeMeters
    
    if changeLongitude == 0 or longitudeMeters == 0:
        changeLongitudePerMeter = 0
    else:
        changeLongitudePerMeter = changeLongitude / longitudeMeters
        

    # x1, y1 = utm.from_latlon(start["latitude"], start["longitude"])
    # x2, y2 = utm.from_latlon(end["latitude"], end["longitude"])
    slopeAngle = math.atan2(latitudeMeters, longitudeMeters)
    numSteps =  int(math.ceil(distance/separation))
    #print(numSteps)

    # Getting steps in latitude and longitude per separation point
    stepLatitude = separation * math.sin(slopeAngle) * changeLatitudePerMeter
    stepLongitude = separation * math.cos(slopeAngle) * changeLongitudePerMeter

    vertexObj = ({
                "weight": 0.0,
                "name": "",
                "hub": False,
                "latitude": 0,
                "longitude": 0
            })

    for step in range(1, numSteps):
        newVertex = vertexObj.copy()
        newVertex["latitude"] = start["latitude"] + (step * stepLatitude)
        newVertex["longitude"] = start["longitude"] + (step * stepLongitude)
        newVertex["name"] = "chunk %d of vertex %s and %s" % \
            (step, start["name"], end["name"])
        # print(newVertex)
        vertexList.append(newVertex)

    vertexList.append(end)
    return (vertexList)
