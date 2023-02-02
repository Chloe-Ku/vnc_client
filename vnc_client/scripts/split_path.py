from math import radians, sin, cos, acos
# from network_util import sliceEdge
from network_util import sliceEdge

"""
Calculates the distance of a JSON edge object
With format: 
    {
        "distance": 0.0,
        "vertex2": {
            ...
            "latitude": 35.771530000000006,
            "longitude": -78.67469000000001
        },
        "vertex1": {
            ...
            "latitude": 35.77138,
            "longitude": -78.67438000000001
        }
    }
    
Returns edge object with updated distance in meters
"""
def calcDistanceEdge(edge):
    radius = 6371.01
    latStart = radians(edge['vertex1']['latitude'])
    longStart = radians(edge['vertex1']['longitude'])
    latEnd = radians(edge['vertex2']['latitude'])
    longEnd = radians(edge['vertex2']['longitude'])

    edge['distance'] = (radius * acos(sin(latStart) * sin(latEnd) + cos(latStart) * cos(latEnd) * cos(longStart - longEnd))) * 1000
    return edge


"""
Calculates the distance between two vertices
With format:
vertex = [latitude, longitude]

Returns distance in meters
"""
def calcDistanceVertices(startVertex, endVertex):
    radius = 6371.01
    latStart = radians(startVertex[0])
    longStart = radians(startVertex[1])
    latEnd = radians(endVertex[0])
    longEnd = radians(endVertex[1])

    return (radius * acos(sin(latStart) * sin(latEnd) + cos(latStart) * cos(latEnd) * cos(longStart - longEnd))) * 1000


"""
Determines if the distance of an edge is less than the max distance
"""
def distanceLessThan(edge, maxDist):
    edgeDist = edge['distance']
    if edgeDist < maxDist:
        return True
    else:
        return False


"""
WIP: 
Splits all edges that are greater than the given distance

Returns array list of new latitude and longitude points
"""
def splitEdges(edges, distance):
    slicedEdges = []
    for i in range(len(edges)):
        edges[i] = calcDistanceEdge(edges[i])
        if distanceLessThan(edges[i], distance):
            slicedEdges.append(edges[i])
        # if the edge is greater than the desired distance, slice edge
        else:
            # get list of all new edges
            vertexList = sliceEdge(edges[i], distance)
            # create new edges for each set of vertices and add to list
            for idx in range(0, len(vertexList) - 1):
                newEdge = {
                    "distance": 0.0,
                    "vertex2": {
                        "weight": 0.0,
                        "name": "",
                        "hub": False,
                        "latitude": vertexList[idx + 1]['latitude'],
                        "longitude": vertexList[idx + 1]['longitude']
                    },
                    "vertex1": {
                        "weight": 0.0,
                        "name": "",
                        "hub": False,
                        "latitude": vertexList[idx]['latitude'],
                        "longitude": vertexList[idx]['longitude']
                    }
                }
                newEdge = calcDistanceEdge(newEdge)
                slicedEdges.append(newEdge)
    return slicedEdges
