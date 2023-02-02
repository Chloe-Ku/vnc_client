
import requests
import sys
import os
# sys.path.insert(1, os.path.join(sys.path[0], '../databases'))
# print(sys.path)
# from sqldb import SQLDB
# from neodb import NeoDB


def decode_polyline(polyline_str):
    """
    Decodes google polyline
    Created by Mike Davlantes
    https://stackoverflow.com/questions/
    15380712/how-to-decode-polylines-from-google-maps-direction-api-in-php
    """
    index, lat, lng = 0, 0, 0
    coordinates = []
    changes = {'latitude': 0, 'longitude': 0}

    # Coordinates have variable length when encoded, so just keep
    # track of whether we've hit the end of the string. In each
    # while loop iteration, a single coordinate is decoded.
    while index < len(polyline_str):
        # Gather lat/lon changes, store them in a dictionary to apply them later
        for unit in ['latitude', 'longitude']:
            shift, result = 0, 0

            while True:
                byte = ord(polyline_str[index]) - 63
                index+=1
                result |= (byte & 0x1f) << shift
                shift += 5
                if not byte >= 0x20:
                    break

            if (result & 1):
                changes[unit] = ~(result >> 1)
            else:
                changes[unit] = (result >> 1)

        lat += changes['latitude']
        lng += changes['longitude']

        coordinates.append((lat / 100000.0, lng / 100000.0))

    return coordinates


def sendGoogleDirectionsRequest(origin, dest):
    """ Sends request to api for getting polyline
    """
    oLat = ("{0:.5f}".format(origin['latitude']))
    oLong = ("{0:.5f}".format(origin['longitude']))
    dLat = ("{0:.5f}".format(dest['latitude']))
    dLong = ("{0:.5f}".format(dest['longitude']))

    key = "AIzaSyDTnFEvZN10E6wmaCyjkkIIX7l5TNYiLGo"
    url = "https://maps.googleapis.com/maps/api/directions/json"
    params = {
        'origin': oLat + "," + oLong,
        'destination': dLat + "," + dLong,
        #Changed mode from "walking" to "biking" for testing
        'mode': "driving",
        'units': "metric",
        'key': key
    }
    #print(url)
    HEADERS = {
        'User-Agent': 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 '
                      '(KHTML, like Gecko) Chrome/67.0.3396.87 Safari/537.36',
    }
    response = requests.get(url, timeout=5, headers=HEADERS, params=params)
    #print((response.text))

    return response


# def getGoogleMapPath(nameOrigin, nameDest):
#     """
#     # Example of correct string
#     # MATCH p=(n1:Waypoint{name: "Node 1"})-[r:CONNECTED]->
#     # (n2:Waypoint{name: "Node 3"}) RETURN n1, n2, r
#     # return n1,n2
#     """
#     # Get a cypher for returning the waypoints
#     listNames = []
#     listNames.append(nameOrigin)
#     listNames.append(nameDest)
#     listNames.sort()
#     cypherStr = "MATCH p=(n1:Waypoint{{name: '{name1}'}})" \
#                 "-[r:CONNECTED]->" \
#                 "(n2:Waypoint{{name: '{name2}'}})" \
#                 "RETURN n1, n2, r".format(name1=listNames[0], name2=listNames[1])
#     print(cypherStr)
#     args = {}
#     # Done based off neodb.py:getGraph
#     response = NeoDB().executeOne(cypherStr, args)

#     if not response:
#         print(response.message)
#         exit()

#     for r in response:
#         origin = {k: v for (k, v) in r[0].items()}
#         dest = {k: v for (k, v) in r[1].items()}
#         edge = {k: v for (k, v) in r[2].items()}

#     polyline_str = edge['polyline']
#     return polyline_str
