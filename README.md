# line_cutting
This repository contains scripts used for the DVRK cutting experiments.

Files, Scripts, and Directories:

calibration_data/: contains all data collected for stereo calibration and 3d line recognition.
image_saver.py: saves images from both cameras to a directory specified in the script.
shape_tracer.py: saves 3d point in robot frame for both psm1 and psm2 (to separate files).
line_cut.py: naive implementation of the line cutting experiment with no perception
line_cut_trajectory.py: implementation of the line cutting experiment using the points in the robot frame, as well as smoothing using a Savitzky-Golay filter, and simple error correction.
demo_recorder.py: records robot demostrations to file. Note: the GUI is currently broken, so ctrl-z to exit/stop recording.
image_utils/: contains scripts and utils for using camera input to detect lines, fit surfaces, and query points in pixel space
EdgeDetection/: some edge detection code
