#!/usr/bin/env python

try:
    import Tkinter as tk
    import Tkinter.filedialog as fd
    import ttk
except ImportError:
    import tkinter as tk
    import tkinter.filedialog as fd
    import tkinter.ttk as ttk

import tobiiglasses.entities
import tobiiglasses.utils
import tobiiglasses.recordings
import tobiiglasses.video
import datetime
import logging
import os
import cv2
import numpy as np
import re

import PIL.Image, PIL.ImageTk

logging.basicConfig(level=logging.DEBUG)
        
        
class TobiiParticipantFull:
    """
    Workaround implementation of tobiiglasses.TobiiParticipant that includes more info
    """
    def __init__(self, ppt_path):
        res = tobiiglasses.utils.load_json_from_file(ppt_path, tobiiglasses.entities.TobiiParticipant.ppt_filename)
        self.__parse_json_ppt__(res)

    def __parse_json_ppt__(self, item):
        self.__ppt_name__ = item["pa_info"]["Name"]
        self.__ppt_notes__ = item["pa_info"]["Notes"]
        self.__ppt_id__ = item["pa_id"]
        self.__ppt_created__ = datetime.datetime.strptime(item["pa_created"], tobiiglasses.entities.TOBII_DATETIME_FORMAT)
        self.__ppt_project__ = item["pa_project"]

    def getName(self):
        return self.__ppt_name__
    
    def getId(self):
        return self.__ppt_id__
    
    def getNotes(self):
        return self.__ppt_notes__
    
    def getCreationDate(self):
        return self.__ppt_created__
    
    def to_vals(self):
        return (self.__ppt_id__, self.__ppt_created__, self.__ppt_notes__)
    
    def __eq__(self, other):
        return self.__ppt_id__ == other.__ppt_id__ and self.__ppt_project__ == other.__ppt_project__
    
    def __ne__(self, other):
        return not self.__eq__(other)
    
    def getIid(self):
        return 'PR[{}].PPT[{}]'.format(self.__ppt_project__, self.__ppt_id__)
    
"""
Add some extra helper functions to entities
"""
tobiiglasses.entities.TobiiProject.getIid = lambda self: 'PR<{}>'.format(self.__project_id__)
tobiiglasses.entities.TobiiProject.to_vals = lambda self: (self.getId(), self.getCreationDate(), '')
tobiiglasses.entities.TobiiRecording.getIid = lambda self: 'PR<{}>.R<{}>'.format(self.__project__.getId(), self.__rec_id__)
tobiiglasses.entities.TobiiRecording.to_vals = lambda self: (self.getId(), self.getCreationDate(), "")
tobiiglasses.entities.TobiiSegment.getIid = lambda self: 'PR<{}>.R<{}>.S<{}>'.format(self.__seg_project_id__, self.__seg_rec_id__, self.__segment_id__)
tobiiglasses.entities.TobiiSegment.to_vals = lambda self: (self.getId(), self.getStartDateTime(), '', '{:.2f}'.format(self.getLengthUs() * 1e-6))


__segment_regex__ = re.compile('PR<([^>]+)>\.R<([^>]+)>\.S<(\d+)>')
def get_recording_info_from_iid(iid):
    m = __segment_regex__.match(iid)
    if m is None:
        return None, None, None
    else:
        return m[1], m[2], int(m[3])
    
"""
Bugfix on video advancing
"""
# def __video_mapped_gaze_next__(self):
#     if (self.__cap__.isOpened()):
#         ret, frame = self.__cap__.read()
#         if ret == True:
#             if self.__current_time__ > self.__vts_list__[self.__i__]:
#                 if self.__i__ < len(self.__vts_list__) - 1:
#                     self.__i__+=1
#             delay = int(self.__vts__[self.__vts_list__[self.__i__]])
#             print(delay)
#             T = self.__df__.index[self.__df__.index.get_loc(self.__current_time__+delay, method='nearest')]
#             x = self.__df__.at[T, tobiiglasses.gazedata.GazeData.GazePixelX]
#             y = self.__df__.at[T, tobiiglasses.gazedata.GazeData.GazePixelY]
#         else:
#             raise StopIteration
#         self.__current_time__ += self.__frame_duration__
#         return (frame, x, y, self.__current_time__)
#     else:
#         raise StopIteration
# tobiiglasses.video.VideoFramesAndMappedGaze.__next__ = __video_mapped_gaze_next__
"""
Better error handling
"""
def get_all_projects(root_dir):
    projects = []
    projects_path = os.path.join(root_dir, tobiiglasses.entities.PROJECTS_DIRNAME)
    for item in os.listdir (projects_path):
        if os.path.isdir(os.path.join(projects_path, item)):
            try:
                projects.append(tobiiglasses.entities.TobiiProject(projects_path, item))
            except FileNotFoundError:
                pass
    return projects

def get_all_recordings(root_dir, project_id):
    recordings_path = tobiiglasses.entities.get_recordings_path(root_dir, project_id)
    recordings = []
    for item in os.listdir (recordings_path):
        if os.path.isdir(os.path.join(recordings_path, item)):
            try:
                if not os.path.exists(os.path.join(recordings_path, item, tobiiglasses.entities.TobiiParticipant.ppt_filename)):
                    # symlink the participant description in
                    res = tobiiglasses.utils.load_json_from_file(os.path.join(recordings_path, item), tobiiglasses.entities.TobiiRecording.rec_filename)
                    os.symlink(os.path.join(recordings_path, '..', 'participants', res['rec_participant'], tobiiglasses.entities.TobiiParticipant.ppt_filename),
                        os.path.join(recordings_path, item, tobiiglasses.entities.TobiiParticipant.ppt_filename))
                if not os.path.isdir(os.path.join(recordings_path, item, tobiiglasses.entities.SEGMENTS_DIRNAME)):
                    # create a blank segments directory
                    os.mkdir(os.path.join(recordings_path, item, tobiiglasses.entities.SEGMENTS_DIRNAME))
                recordings.append(tobiiglasses.entities.TobiiRecording(recordings_path, item))
            except (FileNotFoundError, OSError):
                pass
    return recordings

class TobiiRecordingExplorer():
    def __init__(self):
        self.master = tk.Tk()
        self.root_dir = None
        
        # Directory explorer
        self.explorer_frame = tk.Frame(self.master)
        self.explorer_frame.pack(side=tk.LEFT)
        
        # Root dir selector
        self.select_root_dir_btn = tk.Button(self.explorer_frame, text='Select root directory', command=self.select_root_dir)
        self.select_root_dir_btn.grid()
        
        self.select_root_dir_var = tk.StringVar()
        self.select_root_dir_label = tk.Label(self.explorer_frame, text='', textvariable=self.select_root_dir_var)
        self.select_root_dir_label.grid(row=0, column=1, sticky=tk.W)
        
        self.select_data_tree = ttk.Treeview(self.explorer_frame, columns=('id', 'date_created', 'notes', 'duration'), selectmode='browse')
        self.select_data_tree.column("#0", width=170)
        self.select_data_tree.column("id", width=80)
        self.select_data_tree.column("date_created", width=160)
        self.select_data_tree.column("notes", width=100)
        self.select_data_tree.column("duration", width=60)
        self.select_data_tree.heading("#0", text='Name', anchor=tk.W)
        self.select_data_tree.heading("id", text='ID', anchor=tk.W)
        self.select_data_tree.heading("date_created", text='Date created', anchor=tk.W)
        self.select_data_tree.heading("notes", text='Notes', anchor=tk.W)
        self.select_data_tree.heading("duration", text='Duration', anchor=tk.W)
        
        self.select_data_tree.grid(row=1, column=0, columnspan=2, sticky=tk.N+tk.S+tk.E+tk.W)
        self.select_data_tree.bind('<Double-Button-1>', self.select_segment)
        
        # Video display
        self.display_frame = tk.Frame(self.master)
        self.display_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=1)
        
        
        self.main_canvas = tk.Canvas(self.display_frame, background='#ccc')
        self.main_canvas.pack(fill=tk.BOTH, expand=1)
        self._img_id = self.main_canvas.create_image((0,0), anchor='nw')
        
        self.main_canvas.rowconfigure(0, weight=1)
        self.main_canvas.columnconfigure(0, weight=1)
        self.main_canvas.bind('<Configure>', self._rescale_canvas)
        
        self.control_frame = tk.Frame(self.display_frame)
        self.control_frame.pack(side=tk.BOTTOM, fill=tk.X, expand=0)
        
        self.play_button = tk.Button(self.control_frame, text='\u25B6', command=self.play, state='disabled')
        self.play_button.pack(side=tk.LEFT)
        self.stop_button = tk.Button(self.control_frame, text='\u25A0', command=self.stop, state='disabled')
        self.stop_button.pack(side=tk.LEFT)
        self.time_slider = tk.Scale(self.control_frame, orient=tk.HORIZONTAL, command=self.select_time, state='disabled', resolution=0.01)
        self.time_slider.pack(side=tk.LEFT, fill=tk.X, expand=1)
        self._playing = False
        
        self.clear_segment()
        
    
    def select_root_dir(self):
        # Prompt for new root directory
        root_dir = fd.askdirectory(mustexist=True)
        if root_dir is not None and root_dir != self.root_dir:
            self.set_root_dir(root_dir)
    
    def set_root_dir(self, root_dir):
        # Reset the segment view
        self.clear_segment()
        # Reset the label
        self.select_root_dir_var.set(root_dir)
        self.root_dir = root_dir
        
        # Load in all data
        self.select_data_tree.delete(*self.select_data_tree.get_children(""))
        self._recordings = {}
        
        for proj in tobiiglasses.entities.get_all_projects(root_dir):
            
            self.select_data_tree.insert(parent='', index='end', iid=proj.getIid(), text=proj.getName(), values=proj.to_vals())
            
            # load in all recordings
            try:
                self._recordings[proj.getId()] = get_all_recordings(root_dir, proj.getId())
            except FileNotFoundError:
                self._recordings[proj.getId()] = []
            for rec in self._recordings[proj.getId()]:
                print(rec)
                # update the recording participant record 
                rec.__participant__ = TobiiParticipantFull(rec.__rec_path__)
                # add the participant if not already existing
                if not self.select_data_tree.exists(rec.__participant__.getIid()):
                    self.select_data_tree.insert(parent=proj.getIid(), index='end', text=rec.__participant__.getName(), iid=rec.__participant__.getIid(), values=rec.__participant__.to_vals())
                
                self.select_data_tree.insert(parent=rec.__participant__.getIid(), index='end', text=rec.getName(), iid=rec.getIid(), values=rec.to_vals())
                
                for seg in rec.__segments__:
                    seg.__seg_project_id__ = rec.__project__.getId()
                    seg.__seg_rec_id__ = rec.getId() 
                    self.select_data_tree.insert(parent=rec.getIid(), index='end', text=seg.getId(), iid=seg.getIid(), values=seg.to_vals())
                    
    
    def select_segment(self, evt):
        selected_seg = self.select_data_tree.focus()
        pr_id, rec_id, seg_id = get_recording_info_from_iid(selected_seg)
        if pr_id is not None: # it's a valid segment
            self.set_segment(pr_id, rec_id, seg_id)
    
    def set_segment(self, proj_id, rec_id, seg_id):
        # Load in all data
        # TODO: This may take a long time! Maybe do it in a separate thread? Lazily? Show progressbar?
        self._recording = tobiiglasses.recordings.Recording(self.root_dir, proj_id, rec_id)
        self._segment_id = seg_id
        self._segment = self._recording.__recording__.getSegment(self._segment_id)
        self._gaze_data = self._recording.getGazeData(self._segment_id)
        
        self._frames = []
#         self._t = []
        self._gaze = []
        self._aspect_ratio = float(self._segment.getFrameHeight()) / self._segment.getFrameWidth()
        
        for frame, x, y, T in tobiiglasses.video.VideoFramesAndMappedGaze(self._gaze_data, cv2.VideoCapture(self._segment.getVideoFilename()), self._gaze_data.getFrameFPS()):
            self._frames.append(frame)
#             self._t.append(T)
            self._gaze.append([x,y])
            
#         self._t = np.array(self._t)
        self._gaze = np.array(self._gaze)
        # update the canvas with the right video size
        self.main_canvas.width = self._segment.getFrameWidth()
        self.main_canvas.height = self._segment.getFrameHeight()
        
        self.set_time(0)
        
        # enable the buttons
        self.play_button.configure(state = 'normal')
        self.stop_button.configure(state = 'normal')
        self.time_slider.configure(state = 'normal')
        self.time_slider.configure(from_=0., to=len(self._gaze)/self._gaze_data.getFrameFPS())
        
        self._frame_duration = int(1000. / self._gaze_data.getFrameFPS())
        
    def clear_segment(self):
        self._recording = None
        self._segment_id = -1
        self._segment = None
        self._gaze_data = None
        self._frames = np.empty((0,1,1,3))
        self._gaze = np.empty((0,2))
        self.main_canvas.image = None
        self._frame_pil = None
        self._frame_tk = None
        self.play_button.configure(state = 'disabled')
        self.stop_button.configure(state = 'disabled')
        self.time_slider.configure(state = 'disabled')
        self.time_slider.configure(from_=0, to=0)
        self._frame_duration = 0
        self._playing = False
        
    def set_time(self, t_idx):
        # grab the appropriate frame
        t_idx = min(t_idx, len(self._gaze)-1)
        frame = self._frames[t_idx]
        # draw the gaze in
        # TODO: make these parameters into constants, maybe draw as an option, maybe draw history, etc etc
#         pos = tuple(self._gaze[t_idx,:].astype(np.uint8))
        try:
            pos = tuple(self._gaze[t_idx,:].astype(int))
            cv2.circle(frame, pos, 30, (0,0,255) , 2)
        except (ValueError, OverflowError):
            pass
        
        # set the canvas display
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self._frame_pil = PIL.Image.fromarray(frame)
        self._rescale_canvas(None)
        self._cur_idx = t_idx
        
    def _rescale_canvas(self, evt):
        if self._frame_pil is not None:
            canvas_width, canvas_height = self.main_canvas.winfo_width(), self.main_canvas.winfo_height()
            target_width = int(min(canvas_width, canvas_height / self._aspect_ratio))
            target_height = int(min(canvas_height, canvas_width * self._aspect_ratio))
            
            self._frame_tk = PIL.ImageTk.PhotoImage(self._frame_pil.resize((target_width, target_height)))
            self.main_canvas.itemconfig(self._img_id, image=self._frame_tk)
        
    def play(self):
        if not self._playing:
            self._playing = True
            self._player = self._advance_frame()
    
    def _advance_frame(self):
        if self._cur_idx >= len(self._gaze)-1:
            self._playing = False
        if self._playing:
            self.set_time(t_idx=self._cur_idx+1)
            self.time_slider.set(self._cur_idx * 1e-3 * self._frame_duration)
            self._player = self.master.after(self._frame_duration, self._advance_frame)
    
    def stop(self):
        if self._playing:
            self._playing = False
            self.master.after_cancel(self._player)
    
    def select_time(self, t):
        if self._recording is not None:
#             self._playing = False
            self.set_time(int(float(t) * self._gaze_data.getFrameFPS()))
        
    def run(self):
        self.master.mainloop()

if __name__ == "__main__":
    explorer = TobiiRecordingExplorer()
    explorer.run()
