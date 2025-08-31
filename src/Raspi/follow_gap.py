import numpy as np

def follow_gap(lidar_ranges, min_distance=0.5):
    ranges = np.array(lidar_ranges)
    ranges[ranges < min_distance] = 0
    max_gap_index = np.argmax(ranges)
    return max_gap_index
