# SPDX-License-Identifier: MIT
"""grocery_video.py without ffmpeg or ROS: the frame timeline that joins the video to Gazebo time."""
import importlib.util
import os

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("grocery_video", os.path.join(HERE, "..", "scripts", "grocery_video.py"))
grocery_video = importlib.util.module_from_spec(spec)
spec.loader.exec_module(grocery_video)


def test_timeline_without_missing_frames():
    rows, summary = grocery_video.frame_timeline([10.0, 10.5, 11.0, 11.5], fps=10)
    assert rows[3] == (3, 0.3, 11.5)
    assert summary == {"frames": 4, "largest_gap_s": 0.5, "missing_frames": 0, "gazebo_span_s": 1.5}


def test_missing_frames_are_counted_and_the_rows_keep_their_gazebo_time():
    """Two frames lost after 11.0 s: from then on the video's playback time is 0.2 s of video (two frames at 10 fps)
    behind where Gazebo time would put it; the CSV row still carries each frame's own Gazebo stamp."""
    rows, summary = grocery_video.frame_timeline([10.0, 10.5, 11.0, 12.5, 13.0], fps=10)
    assert summary["missing_frames"] == 2 and summary["largest_gap_s"] == 1.5
    assert rows[3] == (3, 0.3, 12.5)
