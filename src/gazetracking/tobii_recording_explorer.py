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
import datetime
import logging
import traceback
import os

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
tobiiglasses.entities.TobiiProject.getIid = lambda self: 'PR[{}]'.format(self.__project_id__)
tobiiglasses.entities.TobiiProject.to_vals = lambda self: (self.getId(), self.getCreationDate(), '')
tobiiglasses.entities.TobiiRecording.getIid = lambda self: 'PR[{}].R[{}]'.format(self.__project__.getId(), self.__rec_id__)
tobiiglasses.entities.TobiiRecording.to_vals = lambda self: (self.getId(), self.getCreationDate(), "")
tobiiglasses.entities.TobiiSegment.getIid = lambda self: 'PR[{}].R[{}].S[{}]'.format(self.__seg_project_id__, self.__seg_rec_id__, self.__segment_id__)
tobiiglasses.entities.TobiiSegment.to_vals = lambda self: (self.getId(), self.getStartDateTime(), '', '{:.2f}'.format(self.getLengthUs() * 1e-6))


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
        self.explorer_frame.grid()
        
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
        
        self.select_data_tree.grid(row=1, column=0, columnspan=2)
    
    
    def select_root_dir(self):
        # Prompt for new root directory
        root_dir = fd.askdirectory(mustexist=True)
        if root_dir is not None and root_dir != self.root_dir:
            self.set_root_dir(root_dir)
    
    def set_root_dir(self, root_dir):
        # Reset the segment view
        self.set_segment(None)
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
                    
    
    def set_segment(self, segment):
        pass
    
    def run(self):
        self.master.mainloop()

if __name__ == "__main__":
    explorer = TobiiRecordingExplorer()
    explorer.run()