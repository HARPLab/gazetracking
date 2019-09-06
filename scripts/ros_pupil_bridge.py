#!/usr/bin/env python

import rospy
import zmq
import msgpack
import cv_bridge
import ibmmpy.msg
import sensor_msgs.msg
import numpy as np

class GazePublisher:
    TOPIC_MAPPING = {
        'gaze.2d.0.': 'world_data',
        'gaze.2d.1.': 'world_data',
        'gaze.3d.0.': 'world_data',
        'gaze.3d.1.': 'world_data',
        'gaze.3d.01.': 'world_data',
        'pupil.0': 'eye0_data',
        'pupil.1': 'eye1_data'}
    ZMQ_TOPICS = ['gaze', 'pupil']
    def __init__(self, topic='gaze', throttle_period=None):
        self.pub = rospy.Publisher(topic, ibmmpy.msg.GazeData, queue_size=1)
        self.data_queue = {v: [] for v in GazePublisher.TOPIC_MAPPING.values()}
        if throttle_period:
            self.timer = rospy.Timer(throttle_period, lambda _: self.publish()) # discard timerevent
        else:
            self.timer = None
    
    def __call__(self, msgs):
        data = msgpack.loads(msgs[1])
        msg = ibmmpy.msg.GazeDataPoint()
        msg.position.x = data['norm_pos'][0]
        msg.position.y = data['norm_pos'][1]
        msg.confidence = data['confidence']
        msg.header.stamp = rospy.Time.from_sec(data['timestamp'])
        self.data_queue[GazePublisher.TOPIC_MAPPING[msgs[0]]].append(msg)
        
        if self.timer is None:
            self.publish()
            
    def publish(self):
        if any(len(v) > 0 for v in self.data_queue.values()):
            msg = ibmmpy.msg.GazeData(**self.data_queue)
            self.pub.publish(msg)
            self.data_queue = { v: [] for v in GazePublisher.TOPIC_MAPPING.values() }
        
class FramePublisher:
    ZMQ_TOPICS = ['frame.world']     
    def __init__(self, topic='pupil_world/image_raw'):
        self.pub = rospy.Publisher(topic, sensor_msgs.msg.Image, queue_size=1)
        self.bridge = cv_bridge.CvBridge()
        
    def __call__(self, msgs):
        info = msgpack.loads(msgs[1])
        if info['format'] == 'bgr':
            data = np.frombuffer(msgs[2], dtype=np.uint8).reshape((info['height'], info['width'], 3))
            msg = self.bridge.cv2_to_imgmsg(data, encoding='bgr8')
            self.pub.publish(msg)
        else:
            rospy.logwarn('Received frame with unsupported message format {}'.format(info['format']))



class PupilBridge:
    def __init__(self, pupil_host, pupil_port, components):
        self.context = zmq.Context()
        req = self.context.socket(zmq.REQ)
        req.connect('tcp://{}:{}'.format(pupil_host, pupil_port))
        req.send('SUB_PORT')
        sub_port = req.recv()
        
        self.sub = self.context.socket(zmq.SUB)
        self.sub.connect('tcp://{}:{}'.format(pupil_host, sub_port))
        
        for c in components:
            for t in c.ZMQ_TOPICS:
                self.sub.set(zmq.SUBSCRIBE, t)
                
        self.components = components
        
    def run(self):
        while not rospy.is_shutdown():
            msg = self.sub.recv_multipart()
            for component in self.components:
                if any(msg[0].startswith(topic) for topic in component.ZMQ_TOPICS):
                    component(msg)

def main():
    rospy.init_node('pupil_bridge')
    pupil_host = rospy.get_param('~pupil_host', 'localhost')
    pupil_port = rospy.get_param('~pupil_port', 50020)
    
    components = []
    
    if rospy.get_param('~enable_gaze', False):
        throttle_period = rospy.get_param('~gaze_throttle_period', None)
        
        components.append(GazePublisher(
                rospy.get_param('~gaze_topic', 'gaze'),
                rospy.Duration(throttle_period) if throttle_period is not None else None
            ))
        
    if rospy.get_param('~enable_frame', False):
        components.append(FramePublisher(
                rospy.get_param('~frame_topic', 'pupil_world/image_raw')
            ))
        
    if len(components) == 0:
        rospy.logerr('No components requested, aborting')
        return
    
    bridge = PupilBridge(pupil_host, pupil_port, components)
    bridge.run()
    
if __name__ == '__main__':
    main()
                          
        
    
           

        
        
        
        
