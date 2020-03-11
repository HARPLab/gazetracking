#!/usr/bin/env python

import Tkinter
import ttk
import tkFont

class PupilConnection:
    pass

class TobiiConnection:
    pass


class GazeInterfaceSelector:
    def __init__(self, frame, default_font):  
        self.status_font = default_font.copy()
        self.status_font.configure(slant=tkFont.ITALIC)
        print(self.status_font.actual())
    
        # Add info for connection
        self.gaze_conn_info_elems = {'none': []}
        
        self.pupil_frame = Tkinter.Frame(frame)
        self.pupil_frame.grid(row=2, column=1)
        
        self.label_pupil_endpoint = Tkinter.Label(self.pupil_frame, text="Pupil endpoint")
        self.label_pupil_endpoint.grid(sticky=Tkinter.W)
        self.value_pupil_endpoint = Tkinter.StringVar()
        self.entry_pupil_endpoint = Tkinter.Entry(self.pupil_frame, textvariable=self.value_pupil_endpoint)
        self.entry_pupil_endpoint.grid(sticky=Tkinter.W)
        self.gaze_conn_info_elems['pupil'] = [self.entry_pupil_endpoint]
        self.entry_pupil_endpoint['state'] = 'disabled'
        self.value_pupil_endpoint.set("tcp://127.0.0.1:50020")
        
        self.label_pupil_status = Tkinter.Label(self.pupil_frame, text="Not connected", font=self.status_font)
        self.label_pupil_status.grid(sticky=Tkinter.W)
        
        
        self.tobii_frame = Tkinter.Frame(frame)
        self.tobii_frame.grid(row=3, column=1)
        
        self.label_tobii_endpoint = Tkinter.Label(self.tobii_frame, text="Tobii endpoint")
        self.label_tobii_endpoint.grid(sticky=Tkinter.W)
        self.value_tobii_endpoint = Tkinter.StringVar()
        self.entry_tobii_endpoint = Tkinter.Entry(self.tobii_frame, textvariable=self.value_tobii_endpoint)
        self.entry_tobii_endpoint.grid(sticky=Tkinter.W)
        self.entry_tobii_endpoint['state'] = 'disabled'
        self.value_tobii_endpoint.set("192.168.1.100")
        
        self.entry_tobii_endpoint.bind("<FocusOut>", self.update_tobii_endpoint)
        
        self.label_tobii_project = Tkinter.Label(self.tobii_frame, text='Project ID')
        self.label_tobii_project.grid(sticky=Tkinter.W)
        self.value_tobii_project = Tkinter.StringVar()
        self.combobox_tobii_project = ttk.Combobox(self.tobii_frame, textvariable=self.value_tobii_project)
        self.combobox_tobii_project.grid(sticky=Tkinter.W)
        self.combobox_tobii_project['state'] = 'disabled'
        
        
        self.gaze_conn_info_elems['tobii'] = [self.entry_tobii_endpoint, self.combobox_tobii_project]
    
    def update_tobii_endpoint(self, evt):
        print('Called update')
        if self.entry_tobii_endpoint['state'] == 'normal':
            # try to connect with Tobii to list projects.....
            self.combobox_tobii_project['state'] = 'normal'
            self.combobox_tobii_project['values'] = ['a', 'b'] 
            
    
    def select_gaze(self, gaze_option):
        for k in self.gaze_conn_info_elems.keys():
            state = "normal" if k == gaze_option else "disabled"
            for elem in self.gaze_conn_info_elems[k]:
                elem['state'] = state
                
