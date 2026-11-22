# MATLAB experiment analysis

[`cyclic_pursuit_result_analysis.m`](cyclic_pursuit_result_analysis.m) is the historical MATLAB script from the `analysis` branch. It generates trajectory, yaw, and relative-distance figures from ROS 2 bag recordings.

The script contains hard-coded **historical rosbag filenames** and assumes those data files are available locally. The large `.db3` recordings were not transferred into the consolidated project; supply appropriate recordings and update paths before running the analysis.

The code is preserved here for review, not presented as a self-contained reproducible benchmark.
