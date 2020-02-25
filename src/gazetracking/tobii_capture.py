#!/usr/bin/env python

'''
Modified from Tobii API example file
'''

import urllib2
import json
import time
import threading
import socket
from contextlib import contextmanager

# GLASSES_IP = "10.46.16.86"  # IPv4 address
# PORT = 49152
# base_url = 'http://' + GLASSES_IP
# timeout = 1

# Keep-alive message content used to request live data and live video streams
KA_DATA_MSG = "{\"type\": \"live.data.unicast\", \"key\": \"some_GUID\", \"op\": \"start\"}"
KA_VIDEO_MSG = "{\"type\": \"live.video.unicast\", \"key\": \"some_other_GUID\", \"op\": \"start\"}"


class TobiiInterface:
    def __init__(self, ip, port=49152, timeout=1):
        self.ip = ip
        self.port = port
        self.base_url = 'http://' + self.ip
        self.timeout = 1
        # todol check connection? init stuff maybe?
        
    def _post_request(self, api_action, data=None):
        url = self.base_url + api_action
        req = urllib2.Request(url)
        req.add_header('Content-Type', 'application/json')
        data = json.dumps(data)
        response = urllib2.urlopen(req, data)
        data = response.read()
        json_data = json.loads(data)
        return json_data
    
    def _wait_for_status(self, api_action, key, values, rate=0.1):
        # TODO: adjust rate, add timeout
        url = self.base_url + api_action
        running = True
        while running:
            req = urllib2.Request(url)
            req.add_header('Content-Type', 'application/json')
            response = urllib2.urlopen(req, None)
            data = response.read()
            json_data = json.loads(data)
            if json_data[key] in values:
                running = False
            time.sleep(rate)
            
        return json_data[key]
    
    #### Setup
    def create_project(self):
        json_data = self._post_request('/api/projects')
        return json_data['pr_id']
    
    def create_participant(self, project_id):
        data = {'pa_project': project_id}
        json_data = self._post_request('/api/participants', data)
        return json_data['pa_id']
    
    def create_calibration(self, project_id, participant_id):
        data = {'ca_project': project_id, 'ca_type': 'default', 'ca_participant': participant_id}
        json_data = self._post_request('/api/calibrations', data)
        return json_data['ca_id']
    
    ### Calibration
    def run_calibration(self, calibration_id):
        self._post_request('/api/calibrations/' + calibration_id + '/start')
        status = self._wait_for_status('/api/calibrations/' + calibration_id + '/status', 'ca_state', ['failed', 'calibrated'])
        return (status != 'failed')
    
    ### Recording
    def create_recording(self, participant_id):
        data = {'rec_participant': participant_id}
        json_data = self._post_request('/api/recordings', data)
        return json_data['rec_id']
    def start_recording(self, recording_id):
        self._post_request('/api/recordings/' + recording_id + '/start')

    def stop_recording(self, recording_id):
        self._post_request('/api/recordings/' + recording_id + '/stop')
        status = self._wait_for_status('/api/recordings/' + recording_id + '/status', 'rec_state', ['failed', 'done'])
        return (status != 'failed')
    
    @contextmanager
    def recorder(self, recording_id):
        try:
            self.start_recording(recording_id)
            yield recording_id
        finally:
            self.stop_recording(recording_id)
    
    ### Communication
    def send_event(self, _type, tag='', timestamp=0):
        data = { 'ets': timestamp, 'type': _type, 'tag': tag }
        self._post_request('/api/events', data)
        
    
    ### Visualization (for testing only!)
    def start_visualization(self):
        # TODO: overlay the data somehow, maybe with some cv+gstreamer thing?
        
        # set up connections
        peer = (self.ip, self.port)
        iptype = socket.AF_INET6 if ':' in peer[0] else socket.AF_INET
        vid_socket = socket.socket(iptype, socket.SOCK_DGRAM)
        data_socket = socket.socket(iptype, socket.SOCK_DGRAM)
        
        # set up keepalive
        self._viz_running = True
        self.vid_thread = threading.Thread(self.send_keepalive_msg, [vid_socket, KA_VIDEO_MSG, peer])
        self.vid_thread.start()
        self.data_thread = threading.Thread(self.send_keepalive_msg, [data_socket, KA_DATA_MSG, peer])
        self.data_thread.start()
        
        # set up gstreamer (blocked here so the rest of the class can run without it installed)
        global pygst
        global gst
        if pygst is None:
            import pygst
            pygst.require('0.10')
            import gst
        
        PIPELINE_DEF = "udpsrc do-timestamp=true name=src blocksize=1316 closefd=false buffer-size=5600 !" \
               "mpegtsdemux !" \
               "queue !" \
               "ffdec_h264 max-threads=0 !" \
               "ffmpegcolorspace !" \
               "xvimagesink name=video"

        self._pipeline = None
        try:
            self._pipeline = gst.parse_launch(PIPELINE_DEF)
        except Exception, e:
            print e
            self.stop_visualization()
            return
    
        src = self._pipeline.get_by_name("src")
        src.set_property("sockfd", vid_socket.fileno())
    
        self._pipeline.set_state(gst.STATE_PLAYING)
        
    def send_keepalive_msg(self, socket, msg, peer):
        while self._viz_running:
            socket.sendto(msg, peer)
            time.sleep(self.timeout)
            
    def stop_visualization(self):
        self._viz_running = False
        self.vid_thread.join()
        self.data_thread.join()
        if self._pipeline:
            self._pipeline.set_state(gst.STATE_NULL)
    
    @staticmethod
    def scan_for_devices():
        broadcast_peer = ("<broadcast>", 13006 )
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(1.)
        
        data =  "{ \"type\": \"discover\" }"
        sock.sendto( broadcast_peer, data)
        try:
            data = sock.recvfrom(broadcast_peer, 1024)
        except socket.timeout:
            data = ""
        info = json.loads(data)
        
        if "id" in info:
            return info["ipv4"], info
        else:
            return None, None
        

class TobiiCLI:
    def __init__(self):
        self.cur_proj = None
        self.cur_part = None
        self.cur_calib = None
        self.cur_rec = None
        self.interface = None
        
    def print_status(self):
        print("Connected: {}".format(self.interface is not None))
        if self.cur_proj:
            print("Project ID: {}".format(self.cur_proj) )
        if self.cur_part:
            print("Participant ID: {}".format(self.cur_part) )
        if self.cur_calib:
            print("Calibration ID: {}".format(self.cur_calib) )
        if self.cur_rec:
            print("Recording ID: {}".format(self.cur_rec) )
            
            
    def run(self):
        while True:
            command = raw_input("Enter a command:")
            
            if command.startswith("h"):
                self.print_help()
            
            elif command.startswith("q"):
                break
                
            elif command.startswith("co"):
                conn_info = command.split(" ")
                if len(conn_info) != 2:
                    print("Usage: co[nnect] ip")
                else:
                    self.interface = TobiiInterface(conn_info[1])
                    print("Connected to Tobii at {}".format(self.interface.base_url))
            
            elif command.startswith("s"):
                ip, info = TobiiInterface.scan_for_devices()
                if ip is not None: 
                    print("Found Tobii device {} (SN: {}) at address {}".format(info["name"], info["id"], info["ipv4"]))
                    self.interface = TobiiInterface(ip)
                else:
                    print("No devices found")
            
            ### after this point, interface required
            elif not self.interface:
                print("Must connect first")
                    
            elif command.startswith("pr"):
                if ' ' not in command:
                    self.cur_proj = self.interface.create_project()
                    print("Created project with id {}".format(self.cur_proj))
                else:
                    proj_info = command.split(' ')
                    self.cur_proj = proj_info[1]
                    print("Set project id to {}".format(self.cur_proj))
                    
            elif command.startswith("v"):
                subcommands = command.split(" ")
                if len(subcommands) < 2 or subcommands[1] == "start":
                    self.interface.start_visualization()
                elif subcommands[1] == "stop":
                    self.interface.stop_visualization()
                else:
                    print("Unknown subcommand: {}. Expected start or stop".format(subcommands[1]))
            
            ### after this point, project is required
            elif not self.cur_proj:
                print("Must set or create project first")
            
            elif command.startswith("pa"):
                if ' ' not in command:
                    self.cur_part = self.interface.create_participant(self.cur_proj)
                    print("Created participant in project {} with id {}".format(self.cur_proj, self.cur_part))
                else:
                    part_info = command.split(' ')
                    self.cur_part = part_info[1]
                    print("Set project id to {}".format(self.cur_part))
                    
            ### after this point, participant is required
            elif not self.cur_proj:
                print("Must set or create project first")      
                
            elif command.startswith("ca"):
                print("Calibration started. Present target to wearer.")
                cal_id = self.interface.create_calibration(self.cur_proj, self.cur_part)
                res = self.interface.run_calibration(cal_id)
                print("Calibration success: {}".format(res))
                
            elif command.startswith("r"):
                subcommands = command.split(" ")
                if len(subcommands) < 2 or subcommands[1] == "start":
                    if not self.cur_rec:
                        self.cur_rec = self.interface.create_recording(self.cur_part)
                    self.interface.start_recording(self.cur_rec)
                    print("Started recording with id {}".format(self.cur_rec))
                elif subcommands[1] == "stop":
                    self.interface.stop_recording(self.cur_rec)
                    print("Stopped recording with id {}".format(self.cur_rec))
                else:
                    print("Unknown subcommand: {}. Expected start or stop".format(subcommands[1]))

    def print_help(self):
        print("co[nnect] ip: connect to the tobii client at the specified ip")
        print("s[can]: scan the network for devices and connect to the first one found")
        print("pr[oject]: create new project")
        print("pr[oject] id: set project id to known value id")  
        print("pa[rticipant]: create new participant")  
        print("pa[rticipant] id: set project id to known value id")
        print("ca[librate]: run calibration")
        print("r[ecording] start|stop: start or stop a new recording")
        print("v[isualization] start|stop: start or stop the streaming video")
        print("h[elp]: print this help")
        print("q[uit]: quit")
        
    
if __name__ == '__main__':
    cli = TobiiCLI()
    cli.run()
            
 

