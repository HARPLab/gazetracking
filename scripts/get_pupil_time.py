#!/usr/bin/env python

import zmq
import rospy


class PupilConnection:
    SOCKET_ENDPOINT = 'tcp://127.0.0.1:50020'
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)

        # setting IP
        self.socket.connect(PupilConnection.SOCKET_ENDPOINT)

#         # Test connection speed
#         t = time.time()
#         self.socket.send('t') # ask for timestamp; ignoring for now
#         self.socket.recv()
#         delay = time.time()-t

# 
#     def start(self):
#         self.socket.send('R') #start recording with specified name
#         self.socket.recv()
# 
#     def stop(self):
#         self.socket.send('r') # stop recording
#         self.socket.recv()

    def get_pupil_time(self):
        self.socket.send_string('t')
        tm = self.socket.recv_string()
        return float(tm)
    
if __name__ == '__main__':
    rospy.init_node('pupil_time', anonymous=True)
    conn = PupilConnection()
    tm1 = rospy.get_time()
    pupil_tm = conn.get_pupil_time()
    tm2 = rospy.get_time()
    
    print 'ROS time:   {}'.format( (tm1 + tm2)/2)
    print 'Pupil time: {}'.format(pupil_tm)
