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

        self.logger.info('Connecting to ' + addr)
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


class PupilCaptureCreator():
    def __init__(self):
        self.connection = None
        
    def update(self, endpoint):
        try:
            self.connection = ConfigurablePupilCapture(endpoint)
            return True, 'Connected'
        except Exception as e:
            return False, str(e)
    
    def get_connection(self):
        if self.connection is None:
            raise ValueError('Failed to configure pupil connection!')
        return self.connection
        
class ConfigurablePupilCapture():
    def __init__(self, endpoint, timeout=1000.): # ms
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(endpoint)
        
        # Test connection
        t = time.time()
        self.socket.send('t') # ask for timestamp; ignoring for now
        
        try:
            self._recv(timeout)
        except RuntimeError:
            self.context.destroy(linger=0)
            raise ValueError("Timed out")
        
    def _recv(self, timeout=None):
        if timeout is None:
            self.socket.recv()
        else:
            evt = self.socket.poll(timeout, zmq.POLLIN)
            if evt == 0:
                raise RuntimeError("Timed out getting response from socket")
            else:
                return self.socket.recv(flags=zmq.NOBLOCK)
    
    def start(self, timeout=None):
        self.socket.send('R') #start recording with specified name
        self.socket.recv()
        return True # TODO: check of some sort?

    def stop(self, timeout=None):
        self.socket.send('r') # stop recording
        self.socket.recv()
        return True # TODO: check of some sort?
        
    def send_event(self, event_name, event_contents, timeout=None):
        return False
        # TODO: add annotationshttps://docs.pupil-labs.com/developer/core/network-api/#remote-annotations
