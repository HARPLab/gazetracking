# gazetracking
Eye gaze tracking using Pupil Labs head-mounted tracker.

## Publishing gaze positions with ROS
Start Pupil Capture. Ensure that the Pupil Remote plugin is running (available in Pupil Capture under "General" -> "Open plugin").

Ensure that the Pupil Remote `IP` and `port` are set to match the values in `pupil_listener.py` (usually `tcp://127.0.0.1:50020`).

Run the pupil listener:
```bash
rosrun gazetracking pupil_listener.py
```

This should publish information on two ROS topics: `pupil_info` and `gaze_info`. The `gaze_info` topic contains a subset of the information from `pupil_info` that is most commonly used, specifically the position of the gaze (`norm_pos`) and the confidence value (`confidence`).

For more information about the data contained in `pupil_info`, see the Pupil Capture website here: [https://github.com/pupil-labs/pupil/wiki/Data-Format](https://github.com/pupil-labs/pupil/wiki/Data-Format).

## Synchronizing time with ROS
First you'll need to install the appropriate python dependencies (see [https://docs.pupil-labs.com/#linux-dependencies] but note that we need only a subset of these):

`libuvc` from pupil-labs:
```
git clone https://github.com/pupil-labs/libuvc
cd libuvc
mkdir build
cd build
cmake ..
make && sudo make install
```

```
pip install git+https://github.com/zeromq/pyre
pip install git+https://github.com/pupil-labs/pyuvc
```
