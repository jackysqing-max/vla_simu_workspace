"""Geometry helpers used by the perception fusion node."""

import numpy as np


def intrinsics_from_camera_info(msg):
    """Extract `(fx, fy, cx, cy)` from `sensor_msgs/CameraInfo`."""
    return msg.k[0], msg.k[4], msg.k[2], msg.k[5]


def mask_centroid(mask_u8: np.ndarray):
    """Return the centroid `(u, v)` of a binary mask or `None` if empty."""
    ys, xs = np.nonzero(mask_u8 > 0)
    if len(xs) == 0:
        return None
    return float(xs.mean()), float(ys.mean())


def masked_valid_depth_values(mask_u8: np.ndarray, depth_m: np.ndarray):
    """Return valid depth samples inside the mask and the corresponding mask."""
    mask = mask_u8 > 0
    valid = mask & np.isfinite(depth_m) & (depth_m > 0.0)
    return depth_m[valid], valid


def pixel_to_xyz(u: float, v: float, z: float, fx: float, fy: float, cx: float, cy: float):
    """Project a single pixel and depth value into camera coordinates."""
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return float(x), float(y), float(z)


def xyz_to_pixel(x: float, y: float, z: float, fx: float, fy: float, cx: float, cy: float):
    """Project a camera-frame 3D point back into image coordinates."""
    if z <= 1e-9:
        return None
    u = fx * x / z + cx
    v = fy * y / z + cy
    return float(u), float(v)


def masked_depth_to_xyz(
    depth_m: np.ndarray,
    mask_u8: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
):
    """Project all valid mask pixels into a dense XYZ point set."""
    height, width = depth_m.shape
    us, vs = np.meshgrid(np.arange(width), np.arange(height))

    valid = (mask_u8 > 0) & np.isfinite(depth_m) & (depth_m > 0.0)
    if valid.sum() == 0:
        return np.zeros((0, 3), dtype=np.float32)

    z = depth_m[valid]
    u = us[valid].astype(np.float32)
    v = vs[valid].astype(np.float32)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.stack([x, y, z], axis=-1).astype(np.float32)


def masked_rgbd_to_xyzrgbuv(
    depth_m: np.ndarray,
    mask_u8: np.ndarray,
    rgb_u8: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
):
    """Return masked XYZ, RGB, and UV samples from aligned RGB-D observations."""
    height, width = depth_m.shape
    us, vs = np.meshgrid(np.arange(width), np.arange(height))

    valid = (mask_u8 > 0) & np.isfinite(depth_m) & (depth_m > 0.0)
    if valid.sum() == 0:
        return (
            np.zeros((0, 3), dtype=np.float32),
            np.zeros((0, 3), dtype=np.float32),
            np.zeros((0, 2), dtype=np.float32),
        )

    z = depth_m[valid].astype(np.float32)
    u = us[valid].astype(np.float32)
    v = vs[valid].astype(np.float32)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    xyz = np.stack([x, y, z], axis=-1).astype(np.float32)
    rgb = rgb_u8[valid].astype(np.float32) / 255.0
    uv = np.stack([u, v], axis=-1).astype(np.float32)
    return xyz, rgb, uv


def evenly_spaced_subsample_indices(count: int, max_count: int):
    """Keep sample count bounded while staying deterministic frame-to-frame."""
    if max_count <= 0 or count <= max_count:
        return np.arange(count, dtype=np.int32)
    return np.linspace(0, count - 1, num=max_count, dtype=np.int32)


def pca_project(features: np.ndarray, out_dim: int):
    """Project features onto the leading principal components."""
    if features.shape[0] == 0:
        return np.zeros((0, out_dim), dtype=np.float32)
    if features.shape[1] <= out_dim:
        return features.astype(np.float32)

    centered = features - features.mean(axis=0, keepdims=True)
    _u, _s, vh = np.linalg.svd(centered, full_matrices=False)
    basis = vh[:out_dim].T
    return (centered @ basis).astype(np.float32)


def _pairwise_sq_dist(points: np.ndarray, centers: np.ndarray):
    diff = points[:, None, :] - centers[None, :, :]
    return np.sum(diff * diff, axis=-1)


def kmeans(points: np.ndarray, k: int, max_iters: int = 30, seed: int = 0):
    """Run a small deterministic k-means suitable for masked object features."""
    num_points = points.shape[0]
    if num_points == 0:
        return np.zeros((0,), dtype=np.int32), np.zeros((0, points.shape[1]), dtype=np.float32)

    k = max(1, min(int(k), num_points))
    rng = np.random.default_rng(seed)

    first_index = int(rng.integers(num_points))
    centers = [points[first_index]]
    min_sq_dist = np.sum((points - centers[0]) ** 2, axis=1)

    # k-means++ style seeding keeps cluster centers separated without extra deps.
    while len(centers) < k:
        denom = float(min_sq_dist.sum())
        if denom <= 1e-12:
            break
        probs = min_sq_dist / denom
        next_index = int(rng.choice(num_points, p=probs))
        centers.append(points[next_index])
        min_sq_dist = np.minimum(
            min_sq_dist,
            np.sum((points - centers[-1]) ** 2, axis=1),
        )

    centers = np.asarray(centers, dtype=np.float32)
    labels = np.zeros((num_points,), dtype=np.int32)

    for _ in range(max_iters):
        sq_dist = _pairwise_sq_dist(points, centers)
        new_labels = np.argmin(sq_dist, axis=1).astype(np.int32)

        if np.array_equal(new_labels, labels):
            break
        labels = new_labels

        for cluster_index in range(centers.shape[0]):
            mask = labels == cluster_index
            if np.any(mask):
                centers[cluster_index] = points[mask].mean(axis=0)

    return labels, centers


def mean_shift_merge(points_xyz: np.ndarray, bandwidth: float, max_iters: int = 20):
    """Merge nearby 3D proposals into stable modes with a flat-kernel mean shift."""
    if points_xyz.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float32)
    if bandwidth <= 1e-6:
        return points_xyz.astype(np.float32)

    shifted = points_xyz.astype(np.float32).copy()
    for _ in range(max_iters):
        updated = shifted.copy()
        max_move = 0.0
        for index, center in enumerate(shifted):
            distances = np.linalg.norm(points_xyz - center, axis=1)
            neighbors = points_xyz[distances <= bandwidth]
            if neighbors.shape[0] == 0:
                continue
            updated[index] = neighbors.mean(axis=0)
            max_move = max(max_move, float(np.linalg.norm(updated[index] - center)))
        shifted = updated
        if max_move < 1e-4:
            break

    modes = []
    for candidate in shifted:
        if not modes:
            modes.append(candidate)
            continue
        if min(np.linalg.norm(candidate - mode) for mode in modes) > 0.5 * bandwidth:
            modes.append(candidate)
    return np.asarray(modes, dtype=np.float32)


def select_primary_candidate(candidates_xyz: np.ndarray, reference_xyz: np.ndarray):
    """Pick the candidate nearest the masked cloud centroid for single-target tracking."""
    if candidates_xyz.shape[0] == 0:
        return None
    if reference_xyz.shape[0] == 0:
        return int(0)
    center = reference_xyz.mean(axis=0)
    distances = np.linalg.norm(candidates_xyz - center[None, :], axis=1)
    return int(np.argmin(distances))


def rekep_rgbd_candidates(
    xyz: np.ndarray,
    rgb: np.ndarray,
    uv: np.ndarray,
    *,
    num_clusters: int,
    max_samples: int,
    pca_dim: int,
    meanshift_bandwidth: float,
    xyz_weight: float,
    rgb_weight: float,
    seed: int,
):
    """
    Generate ReKep-style 3D proposal points from masked RGB-D samples.

    ReKep clusters dense feature samples inside a segmentation mask, then merges
    the resulting 3D proposals with mean shift. We approximate the same
    structure with masked RGB-D features available in this workspace:
    - subsample masked pixels
    - build joint XYZ + RGB features
    - PCA to a compact embedding
    - k-means to get proposal groups
    - mean shift in 3D to merge nearby proposals
    """
    if xyz.shape[0] == 0:
        return (
            np.zeros((0, 3), dtype=np.float32),
            np.zeros((0, 2), dtype=np.float32),
        )

    keep_indices = evenly_spaced_subsample_indices(xyz.shape[0], max_samples)
    xyz_kept = xyz[keep_indices]
    rgb_kept = rgb[keep_indices]

    xyz_scale = max(float(np.std(xyz_kept)), 1e-6)
    features = np.concatenate(
        [
            float(xyz_weight) * (xyz_kept / xyz_scale),
            float(rgb_weight) * rgb_kept,
        ],
        axis=1,
    ).astype(np.float32)
    embedding = pca_project(features, max(1, int(pca_dim)))
    labels, _centers = kmeans(embedding, k=num_clusters, seed=seed)

    proposal_xyz = []
    for cluster_index in np.unique(labels):
        cluster_mask = labels == cluster_index
        cluster_xyz = xyz_kept[cluster_mask]
        if cluster_xyz.shape[0] == 0:
            continue
        proposal_xyz.append(np.median(cluster_xyz, axis=0))

    proposal_xyz = np.asarray(proposal_xyz, dtype=np.float32)
    merged_xyz = mean_shift_merge(proposal_xyz, bandwidth=float(meanshift_bandwidth))
    proposal_uv = np.zeros((merged_xyz.shape[0], 2), dtype=np.float32)
    return merged_xyz, proposal_uv
