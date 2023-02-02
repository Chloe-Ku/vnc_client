import sys
import json
import math

# Takes in a set of latitude/longitude coordinates and
# breaks them up into x/y coordinates. Calculates new position
# after travelling at a fixed acceleration (tgtAcc) for the given timeInterval.
# Once tgtVelocity is reached, acceleration is set to zero
# Assumes vehicle starts at rest

tgtVelocity = 5 #target velocity, in meters per sec
tgtAcc = .5 #target acceleration, .5 meters per sec squared
timeInterval = .5 #update position after every time this amount of time, in seconds, passes

#coordinates of Base station (latitude and longitude)
BaseY = 35.771180 #longitude
BaseX = -78.672857 #latitude



#update velocity 
def deltaV(v):
    global tgtAcc
    global timeInterval
    global tgtVelocity
    newV = v + tgtAcc*timeInterval
    if (newV >= tgtVelocity):
        tgtAcc = 0
    return newV

#update position given bearing and distance (Source - https://www.movable-type.co.uk/scripts/latlong.html - Destination point given distance and bearing from start point)
def deltaP(lon, lat, bearing, distance):
    R = 6371e3
    lat = math.radians(lat)
    lon = math.radians(lon)
    newLat = math.asin( math.sin(lat)*math.cos(distance/R) +
                    math.cos(lat)*math.sin(distance/R)*math.cos(bearing) );
    newLon = lon + math.atan2(math.sin(bearing)*math.sin(distance/R)*math.cos(lat),
                         math.cos(distance/R)-math.sin(lat)*math.sin(newLat));
    return [math.degrees(newLat), math.degrees(newLon)]

#haversine distance formual, used to convert from global lat/lon to metric x y coordinates. Base station is origin (Source - https://www.movable-type.co.uk/scripts/latlong.html - Distance)
def toMetric(lat1, lon1, lat2, lon2):
    R = 6371e3
    flip = False
    if (lat1 < lat2 or lon1 < lon2):
        flip = True
    deltaLat = math.radians(lat2-lat1)
    deltaLon = math.radians(lon2-lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)

    a = math.sin(deltaLat/2) * math.sin(deltaLat/2) + math.cos(lat1) * math.cos(lat2) * math.sin(deltaLon/2) * math.sin(deltaLon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    if (flip):
        return -R * c #assumes negative lat/lon values used (as opposed to N/W) 
    return R * c #in meters

#find bearing (Source - https://www.movable-type.co.uk/scripts/latlong.html - Bearing)
def bearing(lat1, lat2, lon1, lon2):
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    lon1 = math.radians(lon1)
    lon2 = math.radians(lon2)
    
    y = math.sin(lon2-lon1) * math.cos(lat2)
    x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(lon2-lon1)
    brng = math.atan2(y, x)
    return brng
  


### "MAIN" ###
# with open('output.json','r') as file:
#     input = json.load(file)

def createPositions(inputEdges):

    #make a list containing dictionaries of x, y and z positions
    output = []
    tmpVel = 0 #intermediate value for velocity

    for line_segment in inputEdges:
        #if (float(line_segment['distance']) < tgtVelocity/2):
        #   continue
        init_bearing = bearing(line_segment['vertex1']['latitude'],
                            line_segment['vertex2']['latitude'],
                            line_segment['vertex1']['longitude'],
                            line_segment['vertex2']['longitude'])
        #print(init_bearing)


        #set up initial components
        xPosition = line_segment['vertex1']['longitude']     #starting x
        yPosition = line_segment['vertex1']['latitude']    #starting y
        distanceTravelled = 0                               #tracks how much progress has been made on current line segment
    
        while (line_segment['distance'] > distanceTravelled and distanceTravelled < 10): #keep looping until we pass over the 2nd point of the line segment
            
            #update values with calculation methods
            tmpVel = deltaV(tmpVel) #This is actually the speed, the magnitude of the velocity (scalar)
            increasedDistance = tmpVel * timeInterval
            newP = deltaP(xPosition, yPosition, init_bearing, increasedDistance)
            yPosition = newP[0]
            xPosition = newP[1]

            #useful for hand testing by adding custom waypoints to google maps
            #print(newP[0])
            #print(newP[1])

            #update distance travelled for loop condition
            distanceTravelled += increasedDistance

            # If we've overshot the end of the edge, change the position (latitude/longitude) to the end of the edge
            if distanceTravelled > line_segment['distance']:
                xPosition = line_segment['vertex2']['longitude']
                yPosition = line_segment['vertex2']['latitude']


            #update dictionary with values
            output.append({"position": {
                "x": toMetric(BaseY, xPosition, BaseY, BaseX ),
                "y": toMetric(yPosition, BaseX, BaseY, BaseX),
                "z": 0
            },"velocity":tmpVel})           

    #print output to console instead of writing to file / DEMO VERSION
    # print(json.dumps(output, sort_keys=True, indent=4))

    return output