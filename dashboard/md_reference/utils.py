import numpy as np


def remove_outlier_points(points_tuples, k_nearest=2, threshold=2.0):
    """
    Robust outlier detection for list of (x,y) tuples.
    Only requires numpy.
    """
    points = np.array(points_tuples)
    n_points = len(points)

    dist_matrix = np.zeros((n_points, n_points))
    for i in range(n_points):
        for j in range(i + 1, n_points):
            dist = np.sqrt(np.sum((points[i] - points[j]) ** 2))
            dist_matrix[i, j] = dist
            dist_matrix[j, i] = dist

    k = min(k_nearest, n_points - 1)
    neighbor_distances = np.partition(dist_matrix, k, axis=1)[:, :k]
    avg_neighbor_dist = np.mean(neighbor_distances, axis=1)

    median_dist = np.median(avg_neighbor_dist)
    mask = avg_neighbor_dist <= threshold * median_dist

    filtered_tuples = [t for t, m in zip(points_tuples, mask) if m]
    return filtered_tuples
