#!/usr/bin/env python

try:
    import Tkinter as tk
    import ttk
    from urllib2 import HTTPError
except ImportError:
    import tkinter as tk
    import tkinter.ttk as ttk
    from urllib.error import HTTPError
    
import tobiiglassesctrl
import cv2
import threading
import PIL.Image, PIL.ImageTk
import traceback


class StatusDisplayGenerator():
    class _StatusDisplay():
        def __init__(self, parent, label_text, display_fcn, collapsing=False, sub=None):
            self.title_label = tk.Label(parent, text=label_text)
            self.title_label.grid(column=0, sticky=tk.W)
            self.data_label = tk.Label(parent, text='')
            self.data_label.grid(row=self.title_label.grid_info()['row'], column=1, sticky=tk.W)
            self.display_fcn = display_fcn
            self._grid_info = [self.title_label.grid_info(), self.data_label.grid_info()]
            self.collapsing = collapsing
            self.sub = sub
        def update(self, status):
            if self.sub is not None:
                status = status[self.sub]
            text = self.display_fcn(status)
            if self.collapsing and text is None:
                self.hide()
            else:
                self.show()
                self.data_label.configure(text=text)
        def clear(self):
            self.data_label.configure(text = '')
            if self.collapsing:
                self.hide()
        def hide(self):
            self.title_label.grid_forget()
            self.data_label.grid_forget()
        def show(self):
            self.title_label.grid(**self._grid_info[0])
            self.data_label.grid(**self._grid_info[1])
            
    def __init__(self, label_text, display_fcn, *args, **kwargs):
        self.label_text = label_text
        self.display_fcn = (lambda s: display_fcn.format(**s)) if isinstance(display_fcn, str) else display_fcn
        self.args, self.kwargs = args, kwargs
        
    def __call__(self, parent):
        return StatusDisplayGenerator._StatusDisplay(parent, self.label_text, self.display_fcn, *self.args, **self.kwargs)

def make_safe_display_func(fmt_or_cb, default=None):
    if isinstance(fmt_or_cb, str):
        cb = lambda s: fmt_or_cb.format(**s)
    else:
        cb = fmt_or_cb
    def fn(st):
        try:
            return cb(st)
        except KeyError:
            return default
    return fn
            

INFO_VARS = [
        StatusDisplayGenerator('Name', '{sys_name}'),
        StatusDisplayGenerator('Description', '{sys_descr}'),
        StatusDisplayGenerator('Serial number', '{sys_serial}'),
        StatusDisplayGenerator('MAC Address', '{sys_macaddr}'),
        StatusDisplayGenerator('Hostname', '{sys_hostname}'),
        StatusDisplayGenerator('Version', '{sys_version}'),
        StatusDisplayGenerator('API Version', '{sys_api_version}'),
    ]

STATUS_VARS = [
        StatusDisplayGenerator('Status', '{sys_status}'),
        StatusDisplayGenerator('System time', '{sys_time}'),
        StatusDisplayGenerator('Uptime', '{sys_uptime} s'),
        StatusDisplayGenerator('Battery', '{level}% ({remaining_time} s remaining)', sub='sys_battery'),
        StatusDisplayGenerator('Head unit', '{state}', sub='sys_headunit'),
        StatusDisplayGenerator('Storage', make_safe_display_func(lambda s: '{remaining:.2f} GB/{capacity:.2f} GB ({remaining_time} s remaining)'.format(
            capacity=s['capacity']/1e9, 
            remaining=s['remaining']/1e9, 
            remaining_time=s['remaining_time']), 'None'), sub='sys_storage')
    ]

RECORDING_VARS = {
        'cal_id': StatusDisplayGenerator('Calibration ID', make_safe_display_func('{ca_id}'), sub='sys_calibration', collapsing=True),
        'rec_id': StatusDisplayGenerator('Recording ID', make_safe_display_func('{rec_id}'), sub='sys_recording', collapsing=True),
        # more?
    }

class LiveVideoDisplay():
    def __init__(self, parent):
        self.canvas = tk.Canvas(parent, background='black')
        self.canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        self._img_id = self.canvas.create_image((0,0), anchor='nw')
        self.canvas.bind('<Configure>', self._update_image)
        
        self._pil_img = None
        self._tk_img = None
        self._up_to_date = True
        self._thread = None
        self._connected = False
        self._lock = threading.Lock()
        self._aspect_ratio = None
        
    def connect(self, endpoint):
        if not self._connected:
            self._video_capture = cv2.VideoCapture(endpoint)
            self._connected = True
            self._thread = threading.Thread(target=self._load_images)
            self._thread.start()            
            self.canvas.after(30, self._update_image_runner)
        
    def disconnect(self):
        if self._connected:
            self._connected = False
            self.canvas.itemconfigure(self._img_id, image=None)
            self._thread.join()
        
    def _load_images(self):
        while self._connected:
            ret, frame = self._video_capture.read()
            if not ret:
                self._connected = False
            with self._lock:
                self._up_to_date = False
                self._pil_img = PIL.Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                if self._aspect_ratio is None:
                    self._aspect_ratio = float(self._pil_img.height) / self._pil_img.width
        # clean up own data
        self._aspect_ratio = None
        self._pil_img = None
                    
    def _update_image_runner(self):
        if self._connected:
            self._update_image()
            self.canvas.after(30, self._update_image_runner)
                    
    def _update_image(self, evt=None):
        if self._connected:
            canvas_width, canvas_height = self.canvas.winfo_width(), self.canvas.winfo_height()
            with self._lock:
                if not self._up_to_date or (evt is not None and self._pil_img is not None): # either new image or scale request        
                    target_width = int(min(canvas_width, canvas_height / self._aspect_ratio))
                    target_height = int(min(canvas_height, canvas_width * self._aspect_ratio))
                    self._tk_img = PIL.ImageTk.PhotoImage(self._pil_img.resize((target_width, target_height)))
                    self._up_to_date = True
            self.canvas.itemconfigure(self._img_id, image=self._tk_img)
            
    @property # make this read-only
    def connected(self):
        return self._connected
        


class TobiiMonitor():
    def __init__(self):
        self.master = tk.Tk()
        
        self._connection = None
        
        # Connecting to the device
        self.connect_frame = tk.Frame(self.master)
        self.connect_frame.grid(row=0, column=0)
        
        self.endpoint_var = tk.StringVar()
        self.endpoint_entry = tk.Entry(self.connect_frame, text='192.168.0.102', textvariable=self.endpoint_var)
        self.endpoint_entry.grid(row=0, column=0)
        self.endpoint_entry.bind('<Return>', self.connect_to_endpt)
        self.connect_button = tk.Button(self.connect_frame, text='Connect', command=self.connect_to_endpt)
        self.connect_button.grid(row=0, column=1)
        self.scan_button = tk.Button(self.connect_frame, text='Scan', command=self.connect)
        self.scan_button.grid(row=0, column=2)
        self.connect_status = tk.Label(self.connect_frame, text='Not connected', wraplength=200)
        self.connect_status.grid(row=1, column=0, columnspan=3, sticky=tk.W)
        
        self.status_displays = []
        
        # Basic display
        self.info_frame = tk.LabelFrame(self.master, text='Info')
        self.info_frame.grid(row=1, column=0, sticky=tk.W, padx=5)
        self.info_frame.grid_columnconfigure(0, minsize=100)
        self.info_frame.grid_columnconfigure(1, minsize=100)
        self.status_displays.extend((gen(self.info_frame) for gen in INFO_VARS))
        
        # Status display
        self.status_frame = tk.LabelFrame(self.master, text='Status')
        self.status_frame.grid(row=2, column=0, sticky=tk.W, padx=5)
        self.status_frame.grid_columnconfigure(0, minsize=100)
        self.status_frame.grid_columnconfigure(1, minsize=100)
        self.status_displays.extend((gen(self.status_frame) for gen in STATUS_VARS))
        
        # Recording display
        self.recording_frame = tk.LabelFrame(self.master, text='Recording')
        self.recording_frame.grid(row=3, column=0, sticky=tk.W, padx=5)
        self.calib_status = StatusDisplayGenerator('Calibration status:', self._update_calibration_status)(self.recording_frame)
        self.calib_id = RECORDING_VARS['cal_id'](self.recording_frame)
        self.calibrate_button = tk.Button(self.recording_frame, text='Calibrate (test only)', state='disabled', command=self._calibrate)
        self.calibrate_button.grid(column=0, columnspan=2, sticky=tk.W)
        self.calib_id.hide()
        self._current_proj = None
        self._current_part = None
        
        self.recording_status = StatusDisplayGenerator('Recording status:', self._update_recording_status)(self.recording_frame)
        self.status_displays.append(self.recording_status)
        self.rec_id = RECORDING_VARS['rec_id'](self.recording_frame)
        
        self.recording_control_frame = tk.Frame(self.recording_frame)
        self.recording_control_frame.grid(column=0, columnspan=2, sticky=tk.N+tk.E+tk.S+tk.W)
        self.recording_new = tk.Button(self.recording_control_frame, text='Start test recording', state='disabled', command=self._start_new_recording)
        self.recording_new.pack(side=tk.LEFT)
        self.recording_pause = tk.Button(self.recording_control_frame, text='Pause', state='disabled', command=self._pause_resume_recording)
        self.recording_pause.pack(side=tk.LEFT)
        self.recording_stop = tk.Button(self.recording_control_frame, text='Stop', state='disabled', command=self._stop_recording)
        self.recording_stop.pack(side=tk.LEFT)
        
        self.status_displays.extend((self.calib_status, self.calib_id, self.recording_status, self.rec_id))
        self._current_recording = None
        self._is_recording = False
        
        # Streaming display
        self.streaming_frame = tk.Frame(self.master)
        self.streaming_frame.grid(row=0, column=1, rowspan=5, sticky=tk.N+tk.E+tk.S+tk.W)
        self.master.grid_columnconfigure(1, weight=1)
        self.master.grid_rowconfigure(5, weight=1)
        self.video_display = LiveVideoDisplay(self.streaming_frame)
        self.video_button = tk.Button(self.streaming_frame, text='Start video', command=self._toggle_video, state='disabled')
        self.video_button.pack(side=tk.TOP, anchor='w')
        
        # Set up everything
        self.clear_connection()
        
    def run(self):
        self.master.mainloop()
        
        
    # Connection stuff
    def connect(self, endpt=None):
        try:
            self._connection = tobiiglassesctrl.TobiiGlassesController(address=endpt, video_scene=True, timeout=3.)
            self.connect_status.configure(text = 'Connected to {}'.format(self._connection.get_address()))
            self._update_status()
            self._toggle_video(on=False)
        except BaseException as e:
            traceback.print_exc()
            self.connect_status.configure(text = 'Failed to connect: {}'.format(e))
            self.clear_connection()
            
            
    def connect_to_endpt(self, evt=None):
        self.connect(self.endpoint_var.get())
        
    def talk_to_device(self, func, *args, **kwargs):
        try:
            return func(*args, **kwargs)
        except HTTPError as e:
            traceback.print_exc()
            self.connect_status.configure(text='Received error from device: {}'.format(e.read()))
            return None
        except BaseException as e:
            traceback.print_exc()
            self.connect_status.configure(text = 'Error communicating with device: {}'.format(e))
            self.clear_connection()
            return None
        
    def clear_connection(self):
        self._connection = None
        self._toggle_video(on=False)
        self.video_button.configure(state='disabled', text='Start stream')
        for st in self.status_displays:
            st.clear()
        self.calibrate_button.configure(state='disabled')
        self.recording_new.configure(state='disabled')
        self.recording_pause.configure(text='Pause', state='disabled')
        self.recording_stop.configure(state='disabled')
        self._current_proj = self._current_part = self._current_recording = None
        self._is_recording = False
        
    # Status update
    def _update_status(self):
        if self._connection is not None:
            status = self.talk_to_device(self._connection.get_status)
            if status is not None:
                for disp in self.status_displays:
                    disp.update(status)
                self.master.after(500, self._update_status)
                
    # Recording monitoring
    def _update_calibration_status(self, status):
        self.calibrate_button.configure(state='normal')
        if len(status['sys_calibration']) == 0:
            if 'sys_sim_clb' in status:
                self.calib_status.data_label.configure(text=status['sys_sim_clb'])
            else:
                self.calib_status.data_label.configure(text='Available')
            self.calibrate_button.configure(state='normal')
        else:
            self.calib_status.data_label.configure(text = status['sys_calibration']['ca_state'])
            self.calibrate_button.configure(state='normal' if status['sys_calibration']['ca_state'] == 'calibrated' else 'disabled')
    
    def _init_tobii_data(self):
        if self._current_proj is None:
            self._current_proj = self.talk_to_device(self._connection.create_project, 'testproject')
            if self._current_proj is None:
                return False
        if self._current_part is None:
            self._current_part = self.talk_to_device(self._connection.create_participant, self._current_proj, 'testparticipant')
            if self._current_part is None:
                return False
        return True
    
    def _calibrate(self):
        if not self._init_tobii_data():
            return
        
        cal_id = self.talk_to_device(self._connection.create_calibration, self._current_proj, self._current_part)
        if cal_id is not None:
            self.talk_to_device(self._connection.start_calibration, cal_id)
            
    def _update_recording_status(self, status):
        if len(status['sys_recording']) == 0:
            self._is_recording = False
            rec_st = 'None'
        else:
            rec_st = status['sys_recording']['rec_state']
            if rec_st == 'recording':
                self._is_recording = True
                self._current_recording = status['sys_recording']['rec_id']
            else:
                self._is_recording = False
        self.recording_status.data_label.configure(text=rec_st)
        if self._is_recording:
            self.recording_pause.configure(text='Pause', state='normal')
            self.recording_new.configure(state='disabled')
        elif self._current_recording is not None:
            self.recording_pause.configure(text='Resume', state='normal')
            self.recording_new.configure(state='disabled')
        else:
            self.recording_pause.configure(text='Pause', state='disabled')
            self.recording_new.configure(state='normal')
        if self._current_recording is not None:
            self.recording_stop.configure(state='normal')
        else:
            self.recording_stop.configure(state='disabled')
            
    def _start_new_recording(self):
        if not self._init_tobii_data():
            return
        rec = self.talk_to_device(self._connection.create_recording, self._current_part)
        if rec is None:
            return
        self.talk_to_device(self._connection.start_recording, rec)
        
    def _pause_resume_recording(self):
        if self._current_recording is not None:
            if self._is_recording:
                self.talk_to_device(self._connection.pause_recording, self._current_recording)
            else:
                self.talk_to_device(self._connection.start_recording, self._current_recording)
                
    def _stop_recording(self):
        if self._current_recording is not None:
            self.talk_to_device(self._connection.stop_recording, self._current_recording)
        self._current_recording = None        
    
            
    # Live streaming
    def _toggle_video(self, on=None):
        if self._connection is None:
            return
        if self.video_display.connected or on is False:
            self.video_display.disconnect()
            self.video_button.configure(text = 'Start stream', state='normal')
            # if this fails, the call to clear_connection() overwrites the above, so needs to be called last
            self.talk_to_device(self._connection.stop_streaming)
        elif not self.video_display.connected or on is True:
            self.talk_to_device(self._connection.start_streaming)
            self.talk_to_device(self.video_display.connect, "rtsp://{}:8554/live/scene".format(self._connection.get_address()))
            if self._connection is not None: # the connection didn't fail
                self.video_button.configure(text = 'Stop stream')


if __name__ == "__main__":
    monitor = TobiiMonitor()
    monitor.run()
        
    
        
        

