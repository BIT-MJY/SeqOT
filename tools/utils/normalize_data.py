#!/usr/bin/env python3
# Developed by Xieyuanli Chen and Thomas Läbe
# This file is covered by the LICENSE file in the root of this project.
# Brief: a simple example to normalize the overlap data, one could do the same to yaw

import numpy as np


def normalize_data(ground_truth_mapping):
  """Normalize the training data according to the overlap value.
     Args:
       ground_truth_mapping: the raw ground truth mapping array
     Returns:
       dist_norm_data: normalized ground truth mapping array
  """
  gt_map = ground_truth_mapping
  bin_0_9 = gt_map[np.where(gt_map[:, 2] < 0.1)]
  bin_10_19 = gt_map[(gt_map[:, 2] < 0.2) & (gt_map[:, 2] >= 0.1)]
  bin_20_29 = gt_map[(gt_map[:, 2] < 0.3) & (gt_map[:, 2] >= 0.2)]
  bin_30_39 = gt_map[(gt_map[:, 2] < 0.4) & (gt_map[:, 2] >= 0.3)]
  bin_40_49 = gt_map[(gt_map[:, 2] < 0.5) & (gt_map[:, 2] >= 0.4)]
  bin_50_59 = gt_map[(gt_map[:, 2] < 0.6) & (gt_map[:, 2] >= 0.5)]
  bin_60_69 = gt_map[(gt_map[:, 2] < 0.7) & (gt_map[:, 2] >= 0.6)]
  bin_70_79 = gt_map[(gt_map[:, 2] < 0.8) & (gt_map[:, 2] >= 0.7)]
  bin_80_89 = gt_map[(gt_map[:, 2] < 0.9) & (gt_map[:, 2] >= 0.8)]
  bin_90_100 = gt_map[(gt_map[:, 2] <= 1) & (gt_map[:, 2] >= 0.9)]



  dist_norm_data = np.concatenate((bin_0_9, bin_10_19, bin_20_29, bin_30_39, bin_40_49,
                                  bin_50_59, bin_60_69, bin_70_79, bin_80_89, bin_90_100))

  # print("Distribution normalized data: ", dist_norm_data)
  print("size of normalized data: ", len(dist_norm_data))

  return dist_norm_data
  

if __name__ == '__main__':
  # load the ground truth data
  ground_truth_file = 'path/to/the/ground-truth/file'
  ground_truth_mapping = np.load(ground_truth_file)
  ground_truth_mapping = ground_truth_mapping['arr_0'].astype('float32')
  print(ground_truth_mapping)

  normalize_data(ground_truth_mapping)

