#!/usr/bin/env python

import Tkinter
import ttk
import tkFont
import gazetracking.pupil_capture as pupil
import gazetracking.tobii_interface as tobii
import logging

class GazeInterfaceSelector:
    def __init__(self, frame, default_font=None, default_bg=None):
        self._bg_color = default_bg or frame.master.cget('bg')
        default_font = default_font or tkFont.nametofont("TkDefaultFont")
        self.label_font = default_font.copy()
        self.label_font.configure(weight='bold')
        
        self.gaze_label = Tkinter.Label(frame, text="Gaze capture\n", font=self.label_font)
        self.gaze_label.grid(columnspan=2, sticky=Tkinter.W+Tkinter.E)
        
        GAZE_TYPES = ('none', 'pupil', 'tobii')
        LABELS = {'none': 'None', 'pupil': 'Pupil', 'tobii': 'Tobii'}
        self.selector_buttons = { gaze_type: Tkinter.Button(frame, text=LABELS[gaze_type], command=self.make_gaze_selector(gaze_type)) 
                                 for gaze_type in GAZE_TYPES }
        for i, k in enumerate(GAZE_TYPES):
            self.selector_buttons[k].grid(row=1+i, sticky=Tkinter.N+Tkinter.W+Tkinter.E)
        
        self.gaze_option = 'none'
        
                
        
        self.status_font = default_font.copy()
        self.status_font.configure(slant=tkFont.ITALIC)
                
        # Add info for connection
        self.gaze_conn_info_elems = {'none': []}
        self.connections = {'none': None}
        
        # Pupil info
        self.pupil_frame = Tkinter.Frame(frame)
        self.pupil_frame.grid(row=2, column=2)
        self.pupil_connection = pupil.PupilCaptureCreator()
        self.connections['pupil'] = self.pupil_connection
        
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
        self.tobii_frame.grid(row=3, column=2)
        self.tobii_connection = tobii.TobiiConnection()
        self.connections['tobii'] = self.tobii_connection
        
        
        # Tobii endpoint
        self.label_tobii_endpoint = Tkinter.Label(self.tobii_frame, text="Tobii endpoint")
        self.label_tobii_endpoint.grid(sticky=Tkinter.W)
        self.value_tobii_endpoint = Tkinter.StringVar()
        self.entry_tobii_endpoint = Tkinter.Entry(self.tobii_frame, textvariable=self.value_tobii_endpoint)
        self.entry_tobii_endpoint.grid(sticky=Tkinter.W)
        self.entry_tobii_endpoint['state'] = 'disabled'
        self.value_tobii_endpoint.set("192.168.1.100")
               
        # Tobii project
        self.label_tobii_project = Tkinter.Label(self.tobii_frame, text='Project ID')
        self.label_tobii_project.grid(sticky=Tkinter.W)
        self.value_tobii_project = Tkinter.StringVar()
        self.combobox_tobii_project = ttk.Combobox(self.tobii_frame, values=[], textvariable=self.value_tobii_project)
        self.combobox_tobii_project.grid(sticky=Tkinter.W)
        self.combobox_tobii_project['state'] = 'disabled'

        
        # Tobii participant
        self.label_tobii_participant = Tkinter.Label(self.tobii_frame, text='Participant ID')
        self.label_tobii_participant.grid(sticky=Tkinter.W)
        self.value_tobii_participant = Tkinter.StringVar()
        self.combobox_tobii_participant = ttk.Combobox(self.tobii_frame, values=[], textvariable=self.value_tobii_participant)
        self.combobox_tobii_participant.grid(sticky=Tkinter.W)
        self.combobox_tobii_participant['state'] = 'disabled'
        
        # Tobii calibration
        self.cal_frame = Tkinter.Frame(self.tobii_frame)
        self.cal_frame.grid(sticky=Tkinter.W)
        self.label_tobii_calibration = Tkinter.Label(self.cal_frame, text='Calibrate')
        self.label_tobii_calibration.grid(sticky=Tkinter.W)
#         self.value_tobii_calibration = Tkinter.StringVar()
#         self.combobox_tobii_calibration = ttk.Combobox(self.cal_frame, values=[], textvariable=self.value_tobii_calibration)
#         self.combobox_tobii_calibration.grid(sticky=Tkinter.W)
#         self.combobox_tobii_calibration['state'] = 'disabled'
        self.button_tobii_calibration = Tkinter.Button(self.cal_frame, text="Calibrate", state="disabled")
        self.button_tobii_calibration.grid(row=1, column=1, sticky=Tkinter.W)
        
        # Tobii recording
        self.label_tobii_recording = Tkinter.Label(self.tobii_frame, text='Recording ID')
        self.label_tobii_recording.grid(sticky=Tkinter.W)
        self.value_tobii_recording = Tkinter.StringVar()
        self.combobox_tobii_recording = ttk.Combobox(self.tobii_frame, values=[], textvariable=self.value_tobii_recording)
        self.combobox_tobii_recording.grid(sticky=Tkinter.W)
        self.combobox_tobii_recording['state'] = 'disabled'

        # Tobii status
        self.label_tobii_status = Tkinter.Label(self.tobii_frame, text="Not connected", font=self.status_font, wraplength=250, justify='left')
        self.label_tobii_status.grid(sticky=Tkinter.W)
        
        # Set up callbacks
        self.entry_tobii_endpoint.bind("<Return>", 
               self.make_update_tobii_values(
                   pre_status = "Connecting...",
                   success_status = "Connected.",
                   update_fcn = self.tobii_connection.update_endpoint,
                   var = self.value_tobii_endpoint,
                   next_field = self.combobox_tobii_project
                   ))
        self.combobox_tobii_project.bind("<Return>", 
               self.make_update_tobii_values(
                   pre_status = "Setting project...",
                   success_status = "Project set.",
                   update_fcn = self.tobii_connection.update_project,
                   var = self.value_tobii_project,
                   next_field = self.combobox_tobii_participant
                   ))
        self.combobox_tobii_participant.bind("<Return>", 
               self.make_update_tobii_values(
                   pre_status = "Setting participant...",
                   success_status = "Participant set.",
                   update_fcn = self.tobii_connection.update_participant,
                   var = self.value_tobii_participant,
                   next_field = self.button_tobii_calibration
                   ))
#         self.combobox_tobii_calibration.bind("<Return>", 
#                self.make_update_tobii_values(
#                    pre_status = "Setting calibration...",
#                    success_status = "Calibration set.",
#                    update_fcn = self.tobii_connection.update_calibration,
#                    var = self.value_tobii_calibration,
#                    next_field = self.button_tobii_calibrate
#                    ))
        self.button_tobii_calibration.configure(command= 
               self.make_update_tobii_values(
                   pre_status = "Running calibration. Present target to participant.",
                   success_status = "Calibration successful.",
                   update_fcn = self.tobii_connection.update_calibration,
                   var = None,
                   next_field = self.combobox_tobii_recording
                   ))
        self.combobox_tobii_recording.bind("<Return>", 
               self.make_update_tobii_values(
                   pre_status = "Setting recording...",
                   success_status = "Ready.",
                   update_fcn = self.tobii_connection.update_recording,
                   var = self.value_tobii_recording,
                   next_field = None
                   ))


        self.gaze_conn_info_elems['tobii'] = [self.entry_tobii_endpoint, 
                                              self.combobox_tobii_project,
                                              self.combobox_tobii_participant,
#                                               self.combobox_tobii_calibration,
                                              self.button_tobii_calibration,
                                              self.combobox_tobii_recording]
        
        self._enabled_cache = {k1: {k2: False for k2 in self.gaze_conn_info_elems[k1]} 
                               for k1 in self.gaze_conn_info_elems.keys()}
        self._enabled_cache['pupil'][self.entry_pupil_endpoint] = True
        self._enabled_cache['tobii'][self.entry_tobii_endpoint] = True
    
    def update_pupil_endpoint(self, evt):
        self.label_pupil_status['text'] = 'Trying to connect...'
        self.label_pupil_status.update_idletasks()
        _, msg = self.pupil_connection.update(self.value_pupil_endpoint.get())
        self.label_pupil_status['text'] = msg
        
    def make_update_tobii_values(self, pre_status, success_status, update_fcn, var, next_field):
        def update_tobii_values(evt=None):
            self.label_tobii_status['text'] = pre_status
            self.label_tobii_status.update_idletasks()
            
            if var is None:
                value = None
            elif hasattr(evt.widget, 'selection_vals') and evt.widget.current() >= 0:
                print('selection vals: {}, cur: {}'.format(evt.widget.selection_vals, evt.widget.current()))
                value = evt.widget.selection_vals[evt.widget.current()] 
            else:
                value = var.get() 
            res, msg, val, vals, next_vals = update_fcn(value)
            if res:
                if vals is not None and evt is not None:
                    evt.widget['values'] = [repr(n) for n in vals]
                    evt.widget.selection_vals = vals
                if val is not None and val != '':
                    var.set(val)
                if next_field is not None:
                    if next_vals is not None:
                        next_field.configure(values=[repr(n) for n in next_vals])
                        next_field.set("")
                        next_field.selection_vals = next_vals
                    next_field.configure(state='normal')
                self.label_tobii_status['text'] = success_status
            else:
                self.label_tobii_status['text'] = msg
            # update enabled status
            enabled_status = self.tobii_connection.get_enabled_status()
            if not enabled_status["endpoint"]:
                self.entry_tobii_endpoint.configure(state='disabled')
            if not enabled_status["project"]:
                self.combobox_tobii_project.configure(state='disabled', values=[])
            if not enabled_status["participant"]:
                self.combobox_tobii_participant.configure(state='disabled', values=[])
            if not enabled_status["calibration"]:
                self.button_tobii_calibration.configure(state='disabled')
            if not enabled_status["recording"]:
                self.combobox_tobii_recording.configure(state='disabled', values=[])
                
        return update_tobii_values
            
    def make_gaze_selector(self, val):
        def select_gaze():
            for k in self.gaze_conn_info_elems.keys():
                do_enable = (k == val)
                for elem in self.gaze_conn_info_elems[k]:
                    if do_enable:
                        elem['state'] = "normal" if do_enable and self._enabled_cache[k][elem] else "disabled"
                    else:
                        if k == self.gaze_option:
                            self._enabled_cache[k][elem] = (elem['state'] == "normal")
                        elem["state"] = "disabled"
                btn_args = {'bg': "#cc0000" if do_enable else self._bg_color,
                            'activebackground': "#ff0000" if do_enable else "white" }    
                print('{}: {}'.format(k, btn_args))
                self.selector_buttons[k].configure(**btn_args)
            self.gaze_option = val
            
        return select_gaze
    
    def get_recorder(self):
        conn = self.connections[self.gaze_option]
        try:
            return conn.get_connection() if conn else None
        except BaseException as e:
            print(e)
            return None

class GazeInterfaceTester():
    def __init__(self):
        self.master = Tkinter.Tk()
        self.gaze_frame = Tkinter.Frame(self.master)
        self.selector = GazeInterfaceSelector(self.gaze_frame)
        self.gaze_frame.grid()
        
        self._start_gaze = False
        self.confirm_button = Tkinter.Button(self.master, text='Start recorder', command=self.start_gaze)
        self.confirm_button.grid(sticky=Tkinter.W)
    
    def start_gaze(self):
        self._start_gaze = True
        
    def stop(self):
        self._running = False
        
    def run(self):
        import time
        self._running = True
        while self._running:
            self.master.update_idletasks()
            self.master.update()
            
            if self._start_gaze:
                return self.selector.get_recorder()
            time.sleep(0.01)
            
        
if __name__ == "__main__":
    tester = GazeInterfaceTester()
    recorder = tester.run()
    if recorder:
        while True:
            input = raw_input('enter start, stop, event <name> <details>, quit\n> ')
            if input == 'start':
                res = recorder.start()
                print('started, res={}'.format(res))
            elif input == 'stop':
                res = recorder.stop()
                print('stopped, res={}'.format(res))
            elif input.startswith('event'):
                vals = ' '.split(input)
                v1 = vals[1] if len(vals) > 1 else ''
                v2 = vals[2] if len(vals) > 2 else ''
                res = recorder.send_event(v1, v2)
                print('sent event {} {}, res={}'.format(v1, v2, res))
            elif input == 'quit':
                break
    
    
       
    
