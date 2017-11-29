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

### Installation
In addition to standard package installable via `pip`, you'll need to install `pyre`:

```
pip install git+https://github.com/zeromq/pyre
```

### Running the service
In a terminal, after sourcing ros, run

```
rosrun gazetracking ros_pupil_time_sync.py
```

Then, in Pupil Capture, enable the plugin `Time Sync`. It should show some debug messages, and the `ros` window should show a message like

```
"<hostname>" joined "default-time_sync-v1" group. Announcing service.
```

To test the time synchronization, you can use the utility `get_pupil_time.py` in this package. It should print the current ROS time and the current Pupil Capture time. If they are not identical, something's not set up correctly.

Note that due to a fundamental Python issue, `ros_pupil_time_sync.py` is *not* interruptible with Ctrl-C. If you want to quit it, kill it from another thread with `kill -9 PID`. The following absurd bash command will scrape `ps` for the right process and kill it for you:

```
kill -9 $(ps a | grep ros_pupil_time_sync | head -1 | tr -s " " | cut -d " " -f 1)
```
