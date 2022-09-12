#!/usr/bin/env python3
# Developed by Xieyuanli Chen and Thomas Läbe
# This file is covered by the LICENSE file in the root of this project.
# Brief: a script to generate depth data
import matplotlib.pyplot as plt
import numpy as np

try:
    from utils import *
except:
    from utils import *


velodatatype = np.dtype({
    'x': ('<u2', 0),
    'y': ('<u2', 2),
    'z': ('<u2', 4),
    'i': ('u1', 6),
    'l': ('u1', 7)})
velodatasize = 8



def data2xyzi(data, flip=True):
    xyzil = data.view(velodatatype)
    xyz = np.hstack(
        [xyzil[axis].reshape([-1, 1]) for axis in ['x', 'y', 'z']])
    xyz = xyz * 0.005 - 100.0

    if flip:
        R = np.eye(3)
        R[2, 2] = -1
        # xyz = xyz @ R
        xyz = np.matmul(xyz, R)

    return xyz, xyzil['i']

def get_velo(velofile):
    return data2xyzi(np.fromfile(velofile))


def gen_depth_data(scan_folder, dst_folder, normalize=False):
    """ Generate projected range data in the shape of (64, 900, 1).
        The input raw data are in the shape of (Num_points, 3).
    """
    # specify the goal folder
    dst_folder = os.path.join(dst_folder, 'depth')
    try:
        os.stat(dst_folder)
        print('generating depth data in: ', dst_folder)
    except:
        print('creating new depth folder: ', dst_folder)
        os.mkdir(dst_folder)

    # load LiDAR scan files
    scan_paths = load_files(scan_folder)
    # scan_paths_new = sorted(scan_paths, key=lambda x: x[:-4])
    # print(scan_paths_new == scan_paths)
    scan_files = os.listdir(scan_folder)

    depths = []

    # iterate over all scan files
    for idx in range(0, len(scan_paths)):
        # load a point cloud
        print(scan_paths[idx])
        current_vertex = get_velo(scan_paths[idx])[0]
        # plt.scatter(current_vertex[:, 0], current_vertex[:, 1], s=0.1)
        # plt.show()
        # plt.scatter(current_vertex[:, 0], current_vertex[:, 2], s=0.1)
        # plt.show()
        # current_vertex_refine = np.zeros_like(current_vertex)
        # current_vertex_refine[:,0] = current_vertex[:,0]
        # current_vertex_refine[:,1] = -current_vertex[:,1]
        # current_vertex_refine[:,2] = -current_vertex[:,-2]

        fov_up = 30.67
        fov_down = -10.67
        proj_H = 32
        proj_W = 900
        lowest = 0.1
        highest = 6
        proj_range, proj_vertex, _ = range_projection(current_vertex,
                                                      fov_up=fov_up,
                                                      fov_down=fov_down,
                                                      proj_H=proj_H,
                                                      proj_W=proj_W,
                                                      max_range=80,
                                                      cut_z=False,
                                                      low=lowest,
                                                      high=highest)

        # normalize the image
        if normalize:
            proj_range = proj_range / np.max(proj_range)

        # generate the destination path
        # dst_path = os.path.join(dst_folder, str(idx).zfill(6)+".png")
        dst_path = os.path.join(dst_folder, str(idx).zfill(6))

        # plt.imshow(proj_range)
        # plt.show()

        np.save(dst_path, proj_range)
        depths.append(proj_range)
        print('finished generating depth data at: ', dst_path)

    return depths


if __name__ == '__main__':
    # scan_folder = '/media/mjy/Samsung_T5/NCLT_dataset/velodyne_data/2012-09-28_vel/velodyne_sync'
    # dst_folder = '/media/mjy/Samsung_T5/NCLT_dataset/velodyne_data/2012-09-28_vel/range_image'
    # scan_folder = '/media/mjy/Samsung_T5/NCLT_dataset/velodyne_data/2013-04-05_vel/velodyne_sync'
    # dst_folder = '/media/mjy/Samsung_T5/NCLT_dataset/velodyne_data/2013-04-05_vel/range_image'
    # scan_folder = '/media/mjy/Samsung_T5/NCLT_dataset/velodyne_data/2012-02-05_vel/velodyne_sync'
    # dst_folder = '/media/mjy/Samsung_T5/NCLT_dataset/velodyne_data/2012-02-05_vel/range_image'
    # scan_folder = '/media/mjy/Samsung_T5/NCLT_dataset/velodyne_data/2012-06-15_vel/velodyne_sync'
    # dst_folder = '/media/mjy/Samsung_T5/NCLT_dataset/velodyne_data/2012-06-15_vel/range_image'
    # scan_folder = '/media/mjy/Samsung_T5/NCLT_dataset/velodyne_data/2012-10-28_vel/velodyne_sync'
    # dst_folder = '/media/mjy/Samsung_T5/NCLT_dataset/velodyne_data/2012-10-28_vel/range_image'
    scan_folder = '/media/mjy/Samsung_T5/NCLT_dataset/velodyne_data/2013-02-23_vel/velodyne_sync'
    dst_folder = '/media/mjy/Samsung_T5/NCLT_dataset/velodyne_data/2013-02-23_vel/range_image'
    depth_data = gen_depth_data(scan_folder, dst_folder)