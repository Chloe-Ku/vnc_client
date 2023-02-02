# #!/usr/bin/python3
# Logic for how to actually handle a client connection is here
#

import random
import time
import json
from pprint import pprint
import threading

from command import *
from vehicle_util import *
from vehicle import Vehicle

import logging
import os

logger = logging.getLogger(__name__)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(module)s - %(levelname)s - %(message)s')
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
ch.setFormatter(formatter)
logging_path = "C:\\Users\\alajd\\git\\2022SpringTeam01-EcoPRT\\src\\vnc_server\\client_log"
fh = logging.FileHandler(logging_path)
fh.setLevel(logging.DEBUG)
fh.setFormatter(formatter)
logger.addHandler(ch)
logger.addHandler(fh)
logger.setLevel(logging.DEBUG)

# Hacky work-around to be able to import from a folder above this one.
#import sys
#import os
#sys.path.insert(1, os.path.join(sys.path[0], '../gps_to_odom/src'))

import build_path

import rospy
from custom_msgs.msg import VehicleOdomInfo
from sensor_msgs.msg import NavSatFix, Joy

#
# abstractmethod decorator reuqires the function to be implemented
# in its base classes.
#
from abc import ABCMeta, abstractmethod


class CommStrategy():

    __metaclass__ = ABCMeta

    """
    Servers should call startServer().
    Client should call startClient().
    """

    def __init__(self, sock):
        self.sock = sock

    @abstractmethod
    def startClient(self, theClient):
        pass

# A callback function for handling receiving an Odom message. Will need to read the data from the
# VehicleOdomInfo message and use it to update the position (x,y,x) and Header angle of the Vehicle
def OdomCallBack(data, strategy):
    # Extract the (x, y, z) position from the Odom info
    position = data.odom.pose.pose.position
    # Extract the x and y position. z will always be zero
    x = position.x
    y = position.y
    # Extract the current steering angle of the Vehicle
    steering = data.steering
    # Extract the current heading angle of the Vehicle
    heading = data.heading_angle

    # Persist the relevant odometry information to our Vehicle object
    # self.vehicle.setLocation(lat, lon)
    strategy.vehicle.setSteering(steering)
    strategy.vehicle.setHeading(heading)

    # print("Steering: %f" % (strategy.vehicle.getSteering()))
    # print("Heading: %f" % (strategy.vehicle.getHeading()))


class GoodStrategy(CommStrategy):
    """
    Actually handle the client in a nice way lol
    """

    # Use the run function to set up our ROS node and subscribers to the odom topic.
    # The call back function for our subscriber to the odom topic will update the state of the
    # GoodStrategy.vehicle object with the current state of the vehicle
    def start(self, theClient):
        #TODO: 1) Create ROS node and subscribers to the Odom topic. DONE
        #      2) Create a call back function for the Odom topic to update the state of GoodStrategy.vehicle. DONE
        #      3) Migrate instantiation of GoodStrategy.vehicle from startClient to run. DONE
        #      4) Create a thread safe way to update and read the state of GoodStrategy.vehicle. DONE
        #      5) Update functionality in Vehicle.py to remove the drive engine. All Vehicle needs to do is persist the
        #         current vehicle state and provide a thread safe means for reading and writing it. The simulation engine
        #         will handle the actual driving and updating of vehicle position. DONE
        #      6) Start the subscriber/callback loop for the node. DONE
        #      7) Call the GoodStrategy.startClient() method. DONE
        #      8) Create a subscriber to the /ecoprt/joy_controller topic to see which mode the vehicle is in (Auto of Manual)
        #      9) Create a publisher to the /ecoprt/joy topic to switch the vehicle into Auto mode

        self.riderLat = 0.0
        self.riderLon = 0.0
        self.destinationLat = 0.0
        self.destinationLon = 0.0
        self.vState = State.IDLE

        #1
        # Instantiate our ROS node
        node = rospy.init_node("VNC_Client", anonymous=False)

        #3
        # Instantiate the Vehicle member object of GoodStrategy. This will be used to persist the state of the Vehicle as read from
        # the VehicleOdomInfo topic
        self.vehicle = Vehicle.genVehicle()

        #7
        # Start a thread running to communicate with the server
        # This will successfully start running the socket communication thread with the server
        commThread = threading.Thread(target=self.startClient, args=(theClient,))
        commThread.start()

        #2
        # Create the subscriber to the topic publishing Vehicle odometry information. This will update the state of GoodStrategy.Vehicle
        # With the current position and header angle of the Vehicle. Pass the GoodStrategy object to the callback function too so it can
        # update GoodStrategy.Vehicle

        rospy.Subscriber("/ecoprt/odom", VehicleOdomInfo, OdomCallBack, (self))

        # Create our Joy publisher to change the Vehicle state
        self.joyPub = rospy.Publisher("/ecoprt/joy", Joy, queue_size=10)

        rospy.spin()


    # Starts communication with the VNC. Sends the current internal state representation of this vehicle to the server
    def startClient(self, theClient):
        logger.debug('GoodStrategy.startClient()')

        try:
            self.theClient = theClient

            # Make the initial handshake with the server. This will initialize the vehicle in
            # the server
            logger.info("Sending InitRequest to Server")
            initRequest = InitRequest(self.vehicle.getDict())
            initRequest.send(self.sock)

            initAck = Command.recv(self.sock)
            logger.info("Received InitAcknowledge from Server")

            # Assume that it's idle to being with
            self.vState = State.IDLE

            # Now do the regular communication
            while True:
                self.pingPongClient()
                time.sleep(0.20)
        except Exception as e:
            logger.exception('EXCEPTION: %s' % e)
        finally:
            self.sock.close()

    # The logic of a Finite state machine to handle the transition of the Vehicle from Idle, Enroute to the Rider, at the Rider, Enroute
    # to the destination, and finally at the destination
    def pingPongClient(self):
        if (self.vState == State.IDLE):
            #
            # From the IDLE state
            #
            logger.info('Vehicle is idle')

            # Receive a Command from the Server
            cmd = Command.recv(self.sock)

            # If the Command is an IdleRequest
            if (cmd.__class__ == IdleRequest):
                # Stay idle
                # Create an IdleAck with the current internal state of our vehicle
                ack = IdleAck(self.vehicle.getDict())
                ack.send(self.sock)
            elif (cmd.__class__ == IdleToEnrouteToRiderRequest):
                # Otherwise, if the Command is a ToToRiderRequest transition to the To_Rider state
                #TODO: Rework the transition from Idle to ToRider. Our message simply needs to contain the coordinates that the rider
                #      needs to get picked up at. From there, we can utilize the Google Maps API and path generation files to create
                #      a bag file of points that the auto_control_pdd package will follow. Write the bag file to the correct location and then
                #      set the Vehicle into auto mode to follow the path

                # Extract the latitude and longitude that the rider wishes to be picked up at
                self.riderLat = float(cmd.lat)
                self.riderLon = float(cmd.lon)

                # Get the current location of the Vehicle
                (vehicleLat, vehicleLon) = self.vehicle.getLocation()

                # Generate bag file from the riderLat, riderLon, self.vehicle.lat, self.vehicle.lon
                # Then, write the bag file to the necessary location and change the vehicle to auto mode
                # This path contains all the odom info that the vehicle will need to follow the path.
                # Of particular interest are
                pathToRider = build_path.buildPath(vehicleLat, vehicleLon, self.riderLat, self.riderLon)

                header = rospy.Header()
                header.stamp = rospy.Time.now()

                msg1 = Joy()
                msg1.header = header
                msg1.axes = [-0.0, -0.0, 1.0, -0.0, -0.0, -1.0, -0.0, 0.0]
                msg1.buttons = [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]

                msg2 = Joy()
                msg2.header = header
                msg1.axes = [-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, 1.0, 0.0]
                msg1.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

                self.joyPub.publish(msg1)

                self.joyPub.publish(msg2)


                # Create a IdleToRiderAck message with the current internal state of our vehicle
                ack = IdleToEnrouteToRiderAck(self.vehicle.getDict())
                ack.send(self.sock)

                #logger.info('\tChanging to ENROUTE_TO_RIDER')
                self.vState = State.ENROUTE_TO_RIDER

            elif (cmd.__class__ == IdleToEnrouteToCharger):

                # Get the current location of the Vehicle
                (vehicleLat, vehicleLon) = self.vehicle.getLocation()

                # Set destination location to chosen charger
                self.destinationLat = float(cmd.lat)
                self.destinationLon = float(cmd.lon)

                # Generate bag file from the chargerLat, chargerLon, self.vehicle.lat, self.vehicle.lon
                # Then, write the bag file to the necessary location and change the vehicle to auto mode
                # This path contains all the odom info that the vehicle will need to follow the path.
                # Of particular interest are 
                pathToRider = build_path.buildPath(vehicleLat, vehicleLon, self.destinationLat, self.destinationLon)

                header = rospy.Header()
                header.stamp = rospy.Time.now()
                
                msg1 = Joy()
                msg1.header = header
                msg1.axes = [-0.0, -0.0, 1.0, -0.0, -0.0, -1.0, -0.0, 0.0]
                msg1.buttons = [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]

                msg2 = Joy()
                msg2.header = header
                msg1.axes = [-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, 1.0, 0.0]
                msg1.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

                self.joyPub.publish(msg1)

                self.joyPub.publish(msg2)
                

                # Create a IdleToEnrouteToChargerAck message with the current internal state of our vehicle
                ack = IdleToEnrouteToChargerAck(self.vehicle.getDict())
                ack.send(self.sock)

                logger.info('\tChanging to ENROUTE_TO_CHARGER')
                self.vState = State.ENROUTE_TO_CHARGER
            else:
                pass

        elif (self.vState == State.ENROUTE_TO_RIDER):
            #
            # Vehicle state is ENROUTE_TO_RIDER
            #

            logger.debug("Vehicle is enroute to rider")
            vehicleLat, vehicleLon = self.vehicle.getLocation()
            logger.debug("Vehicle location: %s", {json.dumps((vehicleLat, vehicleLon))})

            # If we aren't yet to the rider latitude and longitude
            if not (SpatialUtil.atCoords(vehicleLat, vehicleLon, self.riderLat, self.riderLon)):
                # Send the current vehicle internal state to the server and don't change state
                ack = EnrouteToRiderAck(self.vehicle.getDict())
                ack.send(self.sock)
            else:
                # Otherwise, if we're at the rider latitude/longitude transition to TO_RIDER and sent the message to the server
                self.vState = State.TO_RIDER
                ack = EnrouteToRiderToToRiderAck(self.vehicle.getDict())
                ack.send(self.sock)
                logger.info("Changing to TO_RIDER")

        elif (self.vState == State.TO_RIDER):
            #
            # Vehicle state is TO_RIDER
            #
            logger.debug('Vehicle has reached the rider')

            # If we're TO_RIDER we need to receive a message from the server telling us the destination
            cmd = Command.recv(self.sock)

            if(cmd.__class__ == ToRiderCancel):
                # Extract the rider latitude and longitude
                self.destinationLat = cmd.lat
                self.destinationLon = cmd.lon

                # Update the state of our vehicle to IDLE
                self.vState = State.IDLE

                # And send a command acknowledgement to the Server
                ack = ToRiderCancelAck(self.vehicle.getDict())
                ack.send(self.sock)

            # If the command we received is an instance of ToRiderToEnrouteToDestRequest
            elif(cmd.__class__ == ToRiderToEnrouteToDestRequest):

                # Extract the destination latitude and longitude
                self.destinationLat = cmd.lat
                self.destinationLon = cmd.lon

                # Get the current Vehicle latitude and longitude
                (vehicleLat, vehicleLon) = self.vehicle.getLocation()

                # Create bag file from (vehicleLat, vehicleLon) to (destinationLat, destinationLon),
                # write the file to the correct location, and chang the mode to auto

                pathToDest = build_path.buildPath(vehicleLat, vehicleLon, self.destinationLat, self.destinationLon)

                # Update the state of our vehicle to ENROUTE_TO_DEST
                self.vState = State.ENROUTE_TO_DEST

                # And send a command acknowledgement to the Server
                ack = ToRiderToEnrouteToDestAck(self.vehicle.getDict())
                ack.send(self.sock)

                logger.debug("Changing to ENROUTE_TO_DEST")

            else:
                pass

        elif (self.vState == State.ENROUTE_TO_DEST):
            #
            # Vehicle state is ENROUTE_TO_DEST
            #

            logger.debug("Vehicle is enroute to destination")
            # Get the current coordinates of the vehicle
            (vehicleLat, vehicleLon) = self.vehicle.getLocation()

            # If we're not at the destination latitude and longitude
            if not (SpatialUtil.atCoords(self.destinationLat, self.destinationLon, vehicleLat, vehicleLon)):
                # Just send an updated message for the current state of the vehicle
                ack = EnrouteToDestAck(self.vehicle.getDict())
                ack.send(self.sock)
            else:
                # Otherwise, let the server know that we're at the destination and update our state to TO_DEST
                self.vState = State.TO_DEST
                ack = EnrouteToDestToToDestAck(self.vehicle.getDict())
                ack.send(self.sock)
                logger.debug("Changing to TO_DEST")

        elif (self.vState == State.TO_DEST):
            #
            # Vehicle state is TO_DEST
            #
            logger.debug('Vehicle has reached the destination')

            # If we're TO_DEST we need to receive a message from the server telling us the rider has confirmed arrival
            cmd = Command.recv(self.sock)

            if(cmd.__class__ == ToDestCancel):
                # Extract the rider latitude and longitude
                self.destinationLat = cmd.lat
                self.destinationLon = cmd.lon

                # Update the state of our vehicle to IDLE
                self.vState = State.IDLE

                # And send a command acknowledgement to the Server
                ack = ToDestCancelAck(self.vehicle.getDict())
                ack.send(self.sock)

            # If the command we received is an instance of ToDestToIdleRequest
            elif(cmd.__class__ == ToDestToIdle):

                # Get the current Vehicle latitude and longitude
                (vehicleLat, vehicleLon) = self.vehicle.getLocation()

                # Update the state of our vehicle to IDLE
                self.vState = State.IDLE

                # And send a command acknowledgement to the Server
                ack = ToDestToIdleAck(self.vehicle.getDict())
                ack.send(self.sock)

                logger.debug("Changing to IDLE")

            else:
                pass

        
        elif (self.vState == State.ENROUTE_TO_CHARGER):
            # 
            # Vehicle state is ENROUTE_TO_CHARGER
            # 

            logger.debug("Vehicle is enroute to charger")

            # Get the current coordinates of the vehicle
            (vehicleLat, vehicleLon) = self.vehicle.getLocation()

            # If we're not at the destination latitude and longitude
            if not (SpatialUtil.atCoords(self.destinationLat, self.destinationLon, vehicleLat, vehicleLon)):
                logger.debug("Not at Charging location yet")
                # Just send an updated message for the current state of the vehicle
                ack = EnrouteToChargerAck(self.vehicle.getDict())
                ack.send(self.sock)
            else:
                # Otherwise, let the server know that we're at the destination and update our state to TO_DEST
                self.vState = State.CHARGING
                ack = EnrouteToChargerToChargingAck(self.vehicle.getDict())
                ack.send(self.sock)
                logger.debug("Changing to CHARGING")

        elif (self.vState == State.CHARGING):
            # 
            # Vehicle state is CHARGING
            # 

            logger.debug("Vehicle is charging")

            # If we're CHARGING we need to receive a message from the server telling us the vehicle is finished charging 
            cmd = Command.recv(self.sock)

            if(cmd.__class__ == ChargingToIdle):

                # Update the state of our vehicle to IDLE
                self.vState = State.IDLE

                # And send a command acknowledgement to the Server
                ack = ChargingToIdleAck(self.vehicle.getDict())
                ack.send(self.sock)
            else:
                #Uncomment to test leaving charging state:
                #self.vehicle.batteryLife +=5
                logger.debug("Incrementing battery life")
                # And send a command acknowledgement to the Server
                ack = ChargingAck(self.vehicle.getDict())
                ack.send(self.sock)
                pass


