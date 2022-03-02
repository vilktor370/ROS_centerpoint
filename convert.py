import open3d as o3d
import struct
import numpy as np
def bin_to_pcd(binFileName):
    size_float = 4
    list_pcd = []
    with open(binFileName, "rb") as f:
        byte = f.read(size_float * 4)
        while byte:
            x, y, z, intensity = struct.unpack("ffff", byte)
            list_pcd.append([x, y, z])
            byte = f.read(size_float * 4)
    np_pcd = np.asarray(list_pcd)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_pcd)
    return pcd

def main():
    # binFileName = 'samples/LIDAR_TOP/n008-2018-08-01-15-16-36-0400__LIDAR_TOP__1533151603547590.pcd.bin'
    # pcd = bin_to_pcd(binFileName)
    # o3d.io.write_point_cloud('/home/tony/Desktop/ROS_centerpoint/lidar.pcd', pcd)
    pcd = o3d.io.read_point_cloud('lidar.pcd')
    print(np.asarray(pcd.points).shape)
    # o3d.visualization.draw_geometries([pcd])
main()