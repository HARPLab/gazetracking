#!/usr/bin/env python

import Tkinter
import ttk
import tkFont
import gazetracking.pupil_capture as pupil

class PupilConnection:
    def __init__(self):
        self.connection = None
        
    def update(self, endpoint):
        try:
            self.connection = pupil.ConfigurablePupilCapture(endpoint)
            return True, 'Connected'
        except Exception as e:
            return False, str(e)
    
    def get_connection(self):
        if self.connection is None:
            raise ValueError('Failed to configure pupil connection!')
        return self.connection
        

class TobiiConnection:
    pass


class GazeInterfaceSelector:
    def __init__(self, frame, default_font):  
        self.status_font = default_font.copy()
        self.status_font.configure(slant=tkFont.ITALIC)
        print(self.status_font.actual())
    
        # Add info for connection
        self.gaze_conn_info_elems = {'none': []}
        
        # Pupil info
        self.pupil_frame = Tkinter.Frame(frame)
        self.pupil_frame.grid(row=2, column=1)
        self.pupil_connection = PupilConnection()
        
        # Pupil endpoint
        self.label_pupil_endpoint = Tkinter.Label(self.pupil_frame, text="Pupil endpoint")
        self.label_pupil_endpoint.grid(sticky=Tkinter.W)
        self.value_pupil_endpoint = Tkinter.StringVar()
        self.entry_pupil_endpoint = Tkinter.Entry(self.pupil_frame, textvariable=self.value_pupil_endpoint)
        self.entry_pupil_endpoint.grid(sticky=Tkinter.W)
        self.gaze_conn_info_elems['pupil'] = [self.entry_pupil_endpoint]
        self.entry_pupil_endpoint['state'] = 'disabled'
        self.value_pupil_endpoint.set("tcp://127.0.0.1:50020")
        self.entry_pupil_endpoint.bind("<Return>", self.update_pupil_endpoint)
        
        # Pupil status
        self.label_pupil_status = Tkinter.Label(self.pupil_frame, text="Not connected", font=self.status_font)
        self.label_pupil_status.grid(sticky=Tkinter.W)
        
        
        # Tobii frame
        self.tobii_frame = Tkinter.Frame(frame)
        self.tobii_frame.grid(row=3, column=1)
        
        # Tobii endpoint
        self.label_tobii_endpoint = Tkinter.Label(self.tobii_frame, text="Tobii endpoint")
        self.label_tobii_endpoint.grid(sticky=Tkinter.W)
        self.value_tobii_endpoint = Tkinter.StringVar()
        self.entry_tobii_endpoint = Tkinter.Entry(self.tobii_frame, textvariable=self.value_tobii_endpoint)
        self.entry_tobii_endpoint.grid(sticky=Tkinter.W)
        self.entry_tobii_endpoint['state'] = 'disabled'
        self.value_tobii_endpoint.set("192.168.1.100")
        
        self.entry_tobii_endpoint.bind("<Return>", self.update_tobii_endpoint)
        
        # Tobii project
        self.label_tobii_project = Tkinter.Label(self.tobii_frame, text='Project ID')
        self.label_tobii_project.grid(sticky=Tkinter.W)
        self.value_tobii_project = Tkinter.StringVar()
        self.combobox_tobii_project = ttk.Combobox(self.tobii_frame, textvariable=self.value_tobii_project)
        self.combobox_tobii_project.grid(sticky=Tkinter.W)
        self.combobox_tobii_project['state'] = 'disabled'
        
        self.combobox_tobii_project.bind("<Return>", self.update_tobii_project)
        
        # Tobii participant
        self.label_tobii_participant = Tkinter.Label(self.tobii_frame, text='Participant ID')
        self.label_tobii_participant.grid(sticky=Tkinter.W)
        self.value_tobii_participant = Tkinter.StringVar()
        self.combobox_tobii_participant = ttk.Combobox(self.tobii_frame, textvariable=self.value_tobii_participant)
        self.combobox_tobii_participant.grid(sticky=Tkinter.W)
        self.combobox_tobii_participant['state'] = 'disabled'
        
        # Tobii calibration
        self.label_tobii_calibration = Tkinter.Label(self.tobii_frame, text='Calibration ID')
        self.label_tobii_calibration.grid(sticky=Tkinter.W)
        self.value_tobii_calibration = Tkinter.StringVar()
        self.combobox_tobii_calibration = ttk.Combobox(self.tobii_frame, textvariable=self.value_tobii_calibration)
        self.combobox_tobii_calibration.grid(sticky=Tkinter.W)
        self.combobox_tobii_calibration['state'] = 'disabled'
        self.combobox_tobii_project.bind("<Return>", self.update_tobii_calibration)
        self.button_tobii_calibrate = Tkinter.Button(self.tobii_frame, text="Calibrate", state="disabled", command=self.run_tobii_calibration)
        
        # Tobii recording
        self.label_tobii_recording = Tkinter.Label(self.tobii_frame, text='Recording ID')
        self.label_tobii_recording.grid(sticky=Tkinter.W)
        self.value_tobii_recording = Tkinter.StringVar()
        self.combobox_tobii_recording = ttk.Combobox(self.tobii_frame, textvariable=self.value_tobii_recording)
        self.combobox_tobii_recording.grid(sticky=Tkinter.W)
        self.combobox_tobii_recording['state'] = 'disabled'
        self.combobox_tobii_recording.bind("<Return>", self.update_tobii_calibration)

        # Tobii status
        self.label_tobii_status = Tkinter.Label(self.tobii_frame, text="Not connected", font=self.status_font)
        self.label_tobii_status.grid(sticky=Tkinter.W)

        self.gaze_conn_info_elems['tobii'] = [self.entry_tobii_endpoint, self.combobox_tobii_project]
    
    def update_pupil_endpoint(self, evt):
        self.label_pupil_status['text'] = 'Trying to connect...'
        self.label_pupil_status.update_idletasks()
        _, msg = self.pupil_connection.update(self.value_pupil_endpoint.get())
        self.label_pupil_status['text'] = msg
    
    def update_tobii_endpoint(self, evt):
        if self.entry_tobii_endpoint['state'] == 'normal':
            # try to connect with Tobii to list projects.....
            self.combobox_tobii_project['state'] = 'normal'
            self.combobox_tobii_project['values'] = ['a', 'b']
            
    def update_tobii_project(self, evt):
        pass
    
    def update_tobii_participant(self, evt):
        pass
    
    def update_tobii_calibration(self, evt):
        pass
    
    def run_tobii_calibration(self, evt):
        pass
    
    def update_tobii_recording(self, evt):
        pass


    def select_gaze(self, gaze_option):
        for k in self.gaze_conn_info_elems.keys():
            state = "normal" if k == gaze_option else "disabled"
            for elem in self.gaze_conn_info_elems[k]:
                elem['state'] = state
                
