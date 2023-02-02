#
# Contains classes with useful functions for handling vehicle/server communication.
#

import math
from enum import Enum

# from dijkstar import Graph, find_path

from vehicle import Vehicle

from typing import Optional

class State(Enum): 
    IDLE     = 1
    TO_RIDER = 2
    TO_DEST  = 3
    ENROUTE_TO_RIDER = 4
    ENROUTE_TO_DEST = 5
    ENROUTE_TO_CHARGER = 6
    CHARGING = 7


class FSMUtil(object):
    @staticmethod
    def expectClass(obj, theClass):
        if (obj.__class__ != theClass):
            errStr = 'Expected %s; Got: %s. ' % (theClass.__name__, obj.__class__.__name)
            raise Exception(errStr)


class SpatialUtil(object):
    @staticmethod
    def getDist(lat1, lon1, lat2, lon2):
        dLat = lat2 - lat1
        dLon = lon2 - lon1
        avgLat = (lat1 + lat2) / 2.0
        dY = (10000000.0 / 90.0) * dLat
        dX = (10000000.0 / 90.0) * dLon * math.cos(math.radians(avgLat))

        return math.sqrt((dX*dX) + (dY*dY))


    @staticmethod
    def getSpeed(maxSpeed, curLat, curLon, toLat, toLon):
        dist = SpatialUtil.getDist(curLat, curLon, toLat, toLon)

        if (dist > Vehicle.startSlowingDist):
            return maxSpeed

        return maxSpeed * (dist / Vehicle.startSlowingDist)

    @staticmethod
    def atCoords(curLat, curLon, toLat, toLon):
        dist = SpatialUtil.getDist(curLat, curLon, toLat, toLon)

        return dist <= Vehicle.proximityAllowed


class PathUtil(object):
    def shortestPath(graph, startNode, endNode):
        g = Graph()

        edgeMap = graph['edges']
        for edgeName in edgeMap:
            edge = edgeMap[edgeName]

            # Make it 'undirected' from a directed representation
            g.add_edge(edge['vertex1'], edge['vertex2'], {'cost': edge['distance']})
            g.add_edge(edge['vertex2'], edge['vertex1'], {'cost': edge['distance']})

        cost_func = lambda u, v, e, prev_e: e['cost']

        nodeList = find_path(g, startNode, endNode, cost_func=cost_func).nodes

        vertexMap = graph['vertices']
        return [vertexMap[n] for n in nodeList]


class VehicleDBUtil(object):
    @staticmethod
    def registerVehicle(db, initRequest):
        print('registerVehicle(%s)' % initRequest)

        # Get the vehicle (if it exists)
        whereObj = {
            'name': initRequest.name
        }
        response = db.read('Vehicles', ['name'], ['name'], whereObj)
        if (response.success != True):
            print('Failed to read for vehicle existence: %s' % response.message)
            return False

        if (len(response.results) > 0):
            # Just update, because it already exists
            setAttrs = {
                'lat':         initRequest.lat,
                'lon':         initRequest.lon,
                'batteryLife': initRequest.batteryLife,
                'enabled': 1
            }
            whereAttrs = {
                'name': initRequest.name
            }
            response = db.update('Vehicles', setAttrs, whereAttrs)
            if (response.success != True):
                print('Failed to update vehicle: %s' % response.message)
                return False
        else:
            # Just insert, because it doesn't exist yet
            obj = {
                'name':        initRequest.name,
                'lat':         initRequest.lat,
                'lon':         initRequest.lon,
                'batteryLife': initRequest.batteryLife,
                'enabled':     1
            }
            cols = list(obj.keys())
            response = db.create('Vehicles', cols, obj)
            if (response.success != True):
                print('Failed to create vehicle: %s' % response.message)
                return False

        return True


    @staticmethod
    def updateLocation(db, vName, t, lat, lon):
        wasSuccessful = True

        #
        # Update the coordinate history
        #
        obj = {
            'vehicleName': vName,
            't':           t,
            'lat':         lat,
            'lon':         lon
        }
        cols = list(obj.keys())
        response = db.create('CoordHistory', cols, obj)
        if (response.success != True):
            print('Failed to insert into CoordinateHistory: %s' % response.message)
            wasSuccessful = False

        #
        # Update vehicle's coordinates
        #
        setAttrs = {
            'lat': lat,
            'lon': lon
        }
        whereAttrs = {
            'name': vName
        }
        response = db.update('Vehicles', setAttrs, whereAttrs)
        if (response.success != True):
            print('Failed to update Vehicle coordinates: %s' % response.message)
            wasSuccessful = False

        return wasSuccessful


    @staticmethod
    def disableVehicle(db, name):
        setAttrs = {
            'enabled': 0
        }
        whereAttrs = {
            'name': name
        }
        response = db.update('Vehicles', setAttrs, whereAttrs)
        if (response.success != True):
            print('Failed to update vehicle: %s' % response.message)
            return False
        else:
            return True

    @staticmethod
    def nextDisabledVehicle(db):
        """This method will return the first available disabled vehicle in the database.

        :return: a currently disabled Vehicle
        :rtype: Vehicle
        """
        # Search the database for the next disabled Vehicle
        whereObj = {
            'enabled': 0
        }
        response = db.read('Vehicles', ['name'], ['enabled'], whereObj)
        if (response.success != True):
            print('Failed to read for vehicle existence: %s' %
                  response.message)
            return False

        if (len(response.results) <= 0):
            return None
            
            #response = db.update('Vehicles', setAttrs, whereAttrs)
            #if (response.success != True):
            #    print('Failed to update vehicle: %s' % response.message)
            #    return False
        
        return response.results[0]


class CoolFunctions(object):
    """
    Some cool functions (used by VehicleServer for a demo/test).

    Each function maps from time to change in (lat, lon)
    """

    @staticmethod
    def line(t):
        dLat = 5
        dLon = 7

        # print('line(%s) = %s' % (t, (dLat, dLon)))

        return (dLat, dLon)


    @staticmethod
    def circle(t):
        period = 10.0
        radius = 5

        multiplier = (2 * math.pi) / period
        dLat = radius * math.cos(t * multiplier)
        dLon = radius * math.sin(t * multiplier)

        # print('circle(%s) = %s' % (t, (dLat, dLon)))

        return (dLat, dLon)


    @staticmethod
    def wave(t):
        period = 10.0
        amplitude = 1

        multiplier = (2 * math.pi) / period
        dLat = amplitude * math.cos(t * multiplier)
        dLon = 1

        # print('wave(%s) = %s' % (t, (dLat, dLon)))

        return (dLat, dLon)
