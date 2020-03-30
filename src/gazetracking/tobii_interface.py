


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
        return part, self.data[proj].keys(), None
    
    def set_calibration(self, proj, part, cal=None):
        if cal is None or cal == '':
            cal = self.__gen_index(self.data[proj][part]['cal'])
        if cal not in self.data[proj][part]['cal']:
            self.data[proj][part]['cal'] += [cal]
        return None, [], self.data[proj][part]['rec']
    
    def set_recording(self, proj, part, cal, rec=None):
        if rec is None:
            rec = self.__gen_index(self.data[proj][part]['rec'])
        if rec not in self.data[proj][part]['rec']:
            self.data[proj][part]['rec'] += [rec]
        return rec, self.data[proj][part]['rec'], None

try:
    import tobiiglassesctrl
except ImportError:
    tobiiglassesctrl = None

class TobiiSelection:
    def __init__(self, name, _id):
        self.name = name
        self.id = _id
        
    def __repr__(self):
        return "{} ({})".format(self.name, self.id)
    
    def __str__(self):
        return self.__repr__()  

class _RemoteTobii:
    
    def __init__(self, endpoint):
        if tobiiglassesctrl is None:
            raise ImportError("Required model tobiiglassesctrl not found. Install this module with pip install tobiiglassesctrl to continue.")
        try:
            self._connection = tobiiglassesctrl.TobiiGlassesController(endpoint, timeout=2)
        except NameError: # fix python2 missing ConnectionError
            raise RuntimeError('Timed out connecting to {}'.format(endpoint))
        
        
    def get_projects(self):
        return [ TobiiSelection(pr['pr_info']['Name'], pr['pr_id']) for pr in self._connection.get_projects() ]
    
    def _get_participants_for_project(self, proj_id):
        return [ TobiiSelection(part['pa_info']['Name'], part['pa_id'])  for part in self._connection.get_participants() if part['pa_project'] == proj_id ]
        
    def set_project(self, proj_name):
        if isinstance(proj_name, TobiiSelection):
            proj_name, proj_id = proj_name.name, proj_name.id
        else:
            proj_id = self._connection.get_project_id(proj_name)
        if proj_id is None:
            proj_id = self._connection.create_project(proj_name)
        return TobiiSelection(proj_name, proj_id), self.get_projects(), self._get_participants_for_project(proj_id)
    
    def set_participant(self, proj, part_name):
        if isinstance(part_name, TobiiSelection):
            part_name, part_id = part_name.name, part_name.id
        else:
            part_id = self._connection.get_project_id(part_name)
        if part_id is None:
            part_id = self._connection.create_participant(proj.id, part_name)
        return TobiiSelection(part_name, part_id), self._get_participants_for_project(proj.id), None
    
    def set_calibration(self, proj, part, cal=None):
        cal_id = self._connection.create_calibration(proj.id, part.id)
        self._connection.start_calibration(cal_id)
        res = self._connection.wait_until_calibration_is_done(cal_id)
        if not res:
            raise RuntimeError("Calibration failed, please retry")
        return None, [], None
    
    def set_recording(self, proj, part, cal=None, rec=''):
        # workaround for TobiiGlassesConnection requiring a project name
        self._connection.participant_name = part.name
        rec_id = self._connection.create_recording(part.id, rec)
        return TobiiSelection(rec, rec_id), [], None
    
    def make_recorder(self, rec):
        return TobiiRecorder(self._connection, rec.id)
    
class TobiiRecorder:
    def __init__(self, conn, rec_id):
        self._connection = conn
        self._rec_id = rec_id
        
    def start(self):
        return self._connection.start_recording(self._rec_id)
    
    def stop(self):
        return self._connection.stop_recording(self._rec_id)
    
    def send_event(self, event_name, event_contents):
        return self._connection.send_custom_event(event_name, event_contents)          
    
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
        if self._state < expected_state:
            raise RuntimeError("Cannot update {}: in invalid state {}".format(value_name, self._state))
        # short circuit -- removed bc TobiiSelection != str
#         if getattr(self, value_name) == value:
#             return True, None, value, None, None
        
        try:
            val, cur_opts, next_opts = validator(self, value)
            setattr(self, value_name, val)
            self.__reset_state_to(next_state)
            return True, None, val, cur_opts, next_opts
        except urllib2.HTTPError as e:
            self.__reset_state_to(expected_state)
            return False, str(e) + ': ' + e.read(), value, None, []            
        except BaseException as e:
            traceback.print_exc()
            # reset state
            # or before if no connection?
            self.__reset_state_to(expected_state)
            return False, str(e), value, None, []
        
        
        
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
            self._connection[endpoint] = _RemoteTobii(endpoint)
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
    
    def get_connection(self):
        if self.state < TobiiConnection._States.READY:
            raise RuntimeError("Can't get connection, tobii not ready")
        return self._connection[self._endpoint].make_recorder(self._recording)
        