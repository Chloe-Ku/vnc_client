#!/usr/bin/env python3

import socket
import threading
import random
from time import sleep

# from communication import Communication
from comm_strategy import GoodStrategy


class VehicleClient(object,):
    # Define the host and port to connect to
    host = 'localhost'
    port = 12345

    def __init__(self):
        # Create the socket
        try:
            # AF_INET means IPv4, SOCK_STREAM means TCP
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except Exception as e:
            print('Failed to create socket: %s' % e)
            exit()

        # Connect the socket
        try:
            self.sock.connect((VehicleClient.host, VehicleClient.port))
        except Exception as e:
            print('Failed to connect: %s' % e)
            exit()

    def run(self):
        # myHandler = RandomStrategy(self.sock)
        self.myHandler = GoodStrategy(self.sock)

        self.myHandler.start(self)


def main():

    vc = VehicleClient()
    vc.run()


if __name__ == '__main__':
    main()
