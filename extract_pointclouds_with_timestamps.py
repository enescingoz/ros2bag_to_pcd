import os
import argparse
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores
from sensor_msgs.msg import PointCloud2, PointField
import open3d as o3d
import numpy as np
import struct

def save_point_cloud_as_pcd(points, filename):
    # Create an empty point cloud
    cloud = o3d.geometry.PointCloud()

    # Set the points to the point cloud
    cloud.points = o3d.utility.Vector3dVector(points)

    # Save the point cloud to a PCD file
    o3d.io.write_point_cloud(filename, cloud)

def save_timestamp_as_txt(timestamp, filename):
    with open(filename, 'w') as f:
        f.write(str(timestamp))

def read_points(cloud_msg):
    """
    Extracts points from a PointCloud2 message.
    """
    points = []
    fmt = _get_struct_fmt(cloud_msg)
    width, height, point_step, row_step, data = cloud_msg.width, cloud_msg.height, cloud_msg.point_step, cloud_msg.row_step, cloud_msg.data

    unpack_from = struct.Struct(fmt).unpack_from
    for v in range(height):
        offset = row_step * v
        for u in range(width):
            p = unpack_from(data, offset)
            points.append([p[0], p[1], p[2]])
            offset += point_step

    return np.array(points)

def _get_struct_fmt(cloud_msg):
    """
    Get the struct format to unpack each point.
    """
    field_structs = {
        PointField.INT8: 'b',
        PointField.UINT8: 'B',
        PointField.INT16: 'h',
        PointField.UINT16: 'H',
        PointField.INT32: 'i',
        PointField.UINT32: 'I',
        PointField.FLOAT32: 'f',
        PointField.FLOAT64: 'd'
    }

    fmt = '<'  # Little-endian
    for field in cloud_msg.fields:
        fmt += field_structs[field.datatype] * (field.count if field.count > 0 else 1)

    return fmt

def main():
    parser = argparse.ArgumentParser(description='Extract and save point clouds from a ROS 2 bag file.')
    parser.add_argument('--bag', type=str, required=True, help='Path to the ROS 2 bag file.')
    parser.add_argument('--topic', type=str, required=True, help='Point cloud topic to extract.')
    args = parser.parse_args()

    # Create output directory if it doesn't exist
    output_dir = 'pointclouds'
    os.makedirs(output_dir, exist_ok=True)

    typestore = get_typestore(Stores.ROS2_HUMBLE)  # Update this to your specific ROS 2 distribution

    # Open the ROS 2 bag file
    with Reader(args.bag) as reader:
        index = 0
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == args.topic:
                # Deserialize the PointCloud2 message
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

                # Extract points
                points = read_points(msg)

                # Generate the output filenames
                pcd_filename = os.path.join(output_dir, f'{index}.pcd')
                txt_filename = os.path.join(output_dir, f'timestamp_{index}.txt')

                # Save the point cloud as a PCD file
                save_point_cloud_as_pcd(points, pcd_filename)
                print(f'Saved: {pcd_filename}')

                # Save the timestamp as a TXT file
                save_timestamp_as_txt(timestamp, txt_filename)
                print(f'Saved: {txt_filename}')
                
                index += 1

if __name__ == '__main__':
    main()
