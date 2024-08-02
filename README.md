
# ROS2 Bag to PCD Converter

This script extracts point clouds from a ROS 2 bag file and saves each point cloud as a `.pcd` file along with its timestamp.

## Dependencies

To run this script, you need to have the following dependencies installed:

- Python 3.8 or higher
- `rosbags`
- `open3d`
- `sensor_msgs`

You can install these dependencies using `pip`:

```sh
pip3 install rosbags open3d
```

## Usage

### Command Line Arguments

- `--bag`: Path to the ROS 2 bag file folder which contains `.db3` and metadata files.

- `--topic`: Point cloud topic name which message type is `sensor_msgs/PointCloud2`.

### Example

```sh
python3 extract_pointclouds_with_timestamps.py --bag <folder_path_to_rosbag_file> --topic <pointcloud_topic_name>
```

### Script Description

1. The script reads a ROS 2 bag file specified by the `--bag` argument.
2. It extracts point cloud messages from the topic specified by the `--topic` argument.
3. Each point cloud is saved as a `.pcd` file in the `pointclouds` directory.
4. Corresponding timestamps are saved in text files in the same directory.

### Output Files

- Point cloud files: `pointclouds/0.pcd`, `pointclouds/1.pcd`, ...
- Timestamp files: `pointclouds/timestamp_0.txt`, `pointclouds/timestamp_1.txt`, ...

## Example

```sh
python3 extract_pointclouds_with_timestamps.py --bag /path/to/ros2bag/file/folder/ --topic /pointcloud_topic_name
```

This command will create the `pointclouds` directory and save the extracted point clouds and timestamps in it.
