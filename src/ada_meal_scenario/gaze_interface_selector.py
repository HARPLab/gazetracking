#!/usr/bin/env python

import traceback
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

class _DummyTobii:
    def __init__(self):
        self.data = {}
        
    def __gen_index(self, vals):
        if len(vals) == 0:
            return 0
        else:
            def filt(x):
                try:
                    return int(x)
                except:
                    return -1
            return max([ filt(x) for x in vals])+1
    
    def get_projects(self):
        return self.data.keys()
    
    def set_project(self, proj):
        print('setting proj: {}'.format(proj))
        if proj not in self.data:
            print('init proj')
            self.data[proj] = {}
        return proj, self.data.keys(), self.data[proj].keys()
    
    def set_participant(self, proj, part):
        if part not in self.data[proj]:
            self.data[proj][part] = {'cal': [], 'rec': [] }
        return part, self.data[proj].keys(), self.data[proj][part]['cal']
    
    def set_calibration(self, proj, part, cal=None):
        if cal is None or cal == '':
            cal = self.__gen_index(self.data[proj][part]['cal'])
        if cal not in self.data[proj][part]['cal']:
            self.data[proj][part]['cal'] += [cal]
        return cal, self.data[proj][part]['cal'], self.data[proj][part]['rec']
    
    def set_recording(self, proj, part, cal, rec=None):
        if rec is None:
            rec = self.__gen_index(self.data[proj][part]['rec'])
        if rec not in self.data[proj][part]['rec']:
            self.data[proj][part]['rec'] += [rec]
        return rec, self.data[proj][part]['rec'], None
            
    
class TobiiConnection:
    
    class _States:
        DISCONNECTED = 0
        REQ_PROJECT = 10
        REQ_PARTICIPANT = 20
        REQ_CALIBRATION = 30
        REQ_RECORDING = 40
        READY = 50
    

    class _Decorators:
        @staticmethod
        def make_state_transition(*args, **kwargs):
            def decorator(fcn):
                def wrapper(inst, *args_inner, **kwargs_inner):
                    kwargs_all = dict(**kwargs)
                    kwargs_all.update(kwargs_inner)
                    print(inst)
                    print(args+args_inner)
                    print(kwargs_all)
                    return inst._do_state_transition(fcn, *(args+args_inner), **kwargs_all)
                return wrapper
            return decorator
    
    def __init__(self):
        self._state = TobiiConnection._States.DISCONNECTED
        self._endpoint = None
        self._project = None
        self._participant = None
        self._calibration = None
        self._recording = None
        self._connection = {}
        
    def __reset_state_to(self, state):
        if state <= TobiiConnection._States.DISCONNECTED:
            self._endpoint = None
            print("reset endpt")
#             self._connection = {} # add back in when we have a transient connection, or not I guess
        if state <= TobiiConnection._States.REQ_PROJECT:
            self._project = None
        if state <= TobiiConnection._States.REQ_PARTICIPANT:
            self._participant = None
        if state <= TobiiConnection._States.REQ_CALIBRATION:
            self._calibration = None
        if state <= TobiiConnection._States.REQ_RECORDING:
            self._recording = None
        
        self._state = state
        
    def _do_state_transition(self, validator, expected_state, next_state, value_name, value):
        print("Setting {} to {} (state={})".format(value_name, value, self._state))
        if self._state < expected_state:
            raise RuntimeError("Cannot update {}: in invalid state {}".format(value_name, self._state))
        # short circuit
        if getattr(self, value_name) == value:
            return True, None, value, None, None
        
        try:
            val, cur_opts, next_opts = validator(self, value)
            setattr(self, value_name, val)
            self.__reset_state_to(next_state)
            return True, None, val, cur_opts, next_opts
        except BaseException as e:
            traceback.print_exc()
            # reset state
            # or before if no connection?
            self.__reset_state_to(expected_state)
            return False, str(e), val, None, []
        
        
        
    def get_enabled_status(self):
        return { 
            'endpoint': True,
            'project': self._state >= TobiiConnection._States.REQ_PROJECT,
            'participant': self._state >= TobiiConnection._States.REQ_PARTICIPANT,
            'calibration': self._state >= TobiiConnection._States.REQ_CALIBRATION,
            'recording': self._state >= TobiiConnection._States.REQ_RECORDING
             }
        
    
    @_Decorators.make_state_transition(_States.DISCONNECTED, _States.REQ_PROJECT, '_endpoint')
    def update_endpoint(self, endpoint):
        if endpoint not in self._connection:
            self._connection[endpoint] = _DummyTobii()
        return endpoint, None, self._connection[endpoint].get_projects()
     
    @_Decorators.make_state_transition(_States.REQ_PROJECT, _States.REQ_PARTICIPANT, '_project')   
    def update_project(self, proj):
        return self._connection[self._endpoint].set_project(proj)
     
    @_Decorators.make_state_transition(_States.REQ_PARTICIPANT, _States.REQ_CALIBRATION, '_participant')      
    def update_participant(self, part):
        return self._connection[self._endpoint].set_participant(self._project, part)
    
    @_Decorators.make_state_transition(_States.REQ_CALIBRATION, _States.REQ_RECORDING, '_calibration')  
    def update_calibration(self, cal=None):
        return self._connection[self._endpoint].set_calibration(self._project, self._participant, cal)
    
    @_Decorators.make_state_transition(_States.REQ_RECORDING, _States.READY, '_recording')  
    def update_recording(self, recording=None):
        return self._connection[self._endpoint].set_recording(self._project, self._participant, self._calibration, recording)
        
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
        self.tobii_connection = TobiiConnection()
        
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
        self.label_tobii_calibration = Tkinter.Label(self.cal_frame, text='Calibration ID')
        self.label_tobii_calibration.grid(sticky=Tkinter.W)
        self.value_tobii_calibration = Tkinter.StringVar()
        self.combobox_tobii_calibration = ttk.Combobox(self.cal_frame, values=[], textvariable=self.value_tobii_calibration)
        self.combobox_tobii_calibration.grid(sticky=Tkinter.W)
        self.combobox_tobii_calibration['state'] = 'disabled'
        self.button_tobii_calibrate = Tkinter.Button(self.cal_frame, text="Calibrate", state="disabled")
#         self.button_tobii_calibrate.grid(row=1, column=1, sticky=Tkinter.W)
        
        # Tobii recording
        self.label_tobii_recording = Tkinter.Label(self.tobii_frame, text='Recording ID')
        self.label_tobii_recording.grid(sticky=Tkinter.W)
        self.value_tobii_recording = Tkinter.StringVar()
        self.combobox_tobii_recording = ttk.Combobox(self.tobii_frame, values=[], textvariable=self.value_tobii_recording)
        self.combobox_tobii_recording.grid(sticky=Tkinter.W)
        self.combobox_tobii_recording['state'] = 'disabled'

        # Tobii status
        self.label_tobii_status = Tkinter.Label(self.tobii_frame, text="Not connected", font=self.status_font)
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
                   next_field = self.combobox_tobii_calibration
                   ))
        self.combobox_tobii_calibration.bind("<Return>", 
               self.make_update_tobii_values(
                   pre_status = "Setting calibration...",
                   success_status = "Calibration set.",
                   update_fcn = self.tobii_connection.update_calibration,
                   var = self.value_tobii_calibration,
                   next_field = self.combobox_tobii_recording
                   ))
#         self.button_tobii_calibrate.configure(command= 
#                self.make_update_tobii_values(
#                    pre_status = "Running calibration. Present target to participant.",
#                    success_status = "Calibration successful.",
#                    update_fcn = self.tobii_connection.update_calibration,
#                    var = None,
#                    next_field = [self.combobox_tobii_recording]
#                    ))
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
                                              self.combobox_tobii_calibration,
                                              self.button_tobii_calibrate,
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
            res, msg, val, vals, next_vals = update_fcn(var.get() if var is not None else None)
            if res:
                if vals is not None and evt is not None:
                    evt.widget['values'] = vals
                if val is not None and val != '':
                    var.set(val)
                if next_field is not None:
                    if next_vals is not None:
                        next_field.configure(values=next_vals)
                    next_field.configure(state='normal')
                    next_field.set("")
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
                self.combobox_tobii_calibration.configure(state='disabled', values=[])
            if not enabled_status["recording"]:
                self.combobox_tobii_recording.configure(state='disabled', values=[])
                
        return update_tobii_values
            


    def select_gaze(self, gaze_option, prev_option):
        for k in self.gaze_conn_info_elems.keys():
            do_enable = k == gaze_option
            for elem in self.gaze_conn_info_elems[k]:
                if do_enable:
                    elem['state'] = "normal" if do_enable and self._enabled_cache[k][elem] else "disabled"
                else:
                    if k == prev_option:
                        self._enabled_cache[k][elem] = (elem['state'] == "normal")
                    elem["state"] = "disabled"
                
