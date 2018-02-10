#!/usr/bin/env python

import zmq
import time
import rospy


class PupilCapture():

    def setup(self, log):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.logger = log

        # setting IP
        host = rospy.get_param('/gazetracking/host', '127.0.0.1')
        port = rospy.get_param('/gazetracking/port', '50020')
        addr = 'tcp://{}:{}'.format(host, port)
        
        self.socket.connect(addr)

        # Not sure this is necessary
        #socket.send('T 0.0') #set timebase to 0.0

        # Test connection speed
        t = time.time()
        self.socket.send('t') # ask for timestamp; ignoring for now
        self.socket.recv()
        delay = time.time()-t

        self.logger.info("Pupil remote controller connected to " + addr + ", command time delay: " + str(delay))

    def start(self):
        self.socket.send('R') #start recording with specified name
        self.socket.recv()

    def stop(self):
        self.socket.send('r') # stop recording
        self.socket.recv()