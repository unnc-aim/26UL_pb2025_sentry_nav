"""Search a saved occupancy map using measured planar scan views."""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt


def move_pose(pose, previous_odom, current_odom):
    """Apply measured base motion to a map pose at the previous scan time."""
    heading = pose[2] - previous_odom[2]
    c, s = math.cos(heading), math.sin(heading)
    dx, dy = np.asarray(current_odom[:2]) - previous_odom[:2]
    angle = heading + current_odom[2]
    return [pose[0] + c * dx - s * dy, pose[1] + s * dx + c * dy,
            math.atan2(math.sin(angle), math.cos(angle))]


class ScanMapMatcher:
    """Cache map geometry for coarse search and current-scan verification."""

    def __init__(self, grid, resolution, origin, xy_step=0.2,
                 angle_step=math.radians(3), min_score=0.85, min_margin=0.08):
        self.grid = np.asarray(grid)
        self.resolution = float(resolution)
        self.origin = np.asarray(origin, dtype=float)
        self.xy_step, self.angle_step = xy_step, angle_step
        self.min_score, self.min_margin = min_score, min_margin
        if (self.grid.ndim != 2 or min(self.grid.shape) < 2
                or not np.any(self.grid >= 65) or not np.any(self.grid == 0)
                or self.origin.shape != (3,) or not np.isfinite(self.origin).all()
                or not all(math.isfinite(v) and v > 0 for v in
                           (resolution, xy_step, angle_step))
                or not 0 < min_score <= 1 or not 0 < min_margin <= 1):
            raise ValueError('Search needs a finite map with free cells and obstacles')
        self.height, self.width = self.grid.shape
        self.distance = distance_transform_edt(self.grid < 65) * resolution
        self.endpoint_likelihood = np.exp(-0.5 * (self.distance / 0.2) ** 2)
        self.outside_likelihood = math.exp(-0.5 * (2.0 / 0.2) ** 2)
        self.c, self.s = math.cos(origin[2]), math.sin(origin[2])

    @staticmethod
    def scan_points(scan, max_points=240):
        values = (scan.range_min, scan.range_max, scan.angle_min, scan.angle_increment)
        if (not all(math.isfinite(v) for v in values)
                or not 0 <= scan.range_min < scan.range_max or scan.angle_increment <= 0):
            raise ValueError('Scan bounds and angles must be finite and ordered')
        ranges = np.asarray(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = (np.isfinite(ranges) & (ranges > 0)
                 & (ranges >= scan.range_min) & (ranges < scan.range_max))
        ranges, angles = ranges[valid], angles[valid]
        if len(ranges) < 80:
            raise ValueError('Waiting for at least 80 finite scan returns')
        if max_points is not None and len(ranges) > max_points:
            indices = np.linspace(0, len(ranges) - 1, max_points, dtype=int)
            ranges, angles = ranges[indices], angles[indices]
        return np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))

    def combine_views(self, views, reference_odom):
        """Align measured views, take the median per half-degree, and cap search work."""
        points = np.concatenate([
            self.endpoints(points, move_pose([0, 0, 0], reference_odom, odom))[0]
            for points, odom in views])
        bins = np.rint((np.arctan2(points[:, 1], points[:, 0]) + math.pi)
                       * 720 / (2 * math.pi)).astype(int) % 720
        unique = np.unique(bins)
        points = np.asarray([np.median(points[bins == index], axis=0) for index in unique])
        if len(points) > 240:
            points = points[np.linspace(0, len(points) - 1, 240, dtype=int)]
        return points, len(unique) * 0.5

    def cells(self, points):
        delta = np.asarray(points) - self.origin[:2]
        local = np.stack((self.c * delta[..., 0] + self.s * delta[..., 1],
                          -self.s * delta[..., 0] + self.c * delta[..., 1]), axis=-1)
        indices = np.floor(local / self.resolution).astype(int)
        inside = ((indices[..., 0] >= 0) & (indices[..., 0] < self.width)
                  & (indices[..., 1] >= 0) & (indices[..., 1] < self.height))
        return indices, inside

    def free(self, points):
        indices, inside = self.cells(points)
        result = np.zeros(inside.shape, dtype=bool)
        result[inside] = self.grid[indices[..., 1][inside], indices[..., 0][inside]] == 0
        return result

    def wall_distances(self, points):
        indices, inside = self.cells(points)
        result = np.full(inside.shape, 2.0)
        result[inside] = self.distance[indices[..., 1][inside], indices[..., 0][inside]]
        return result

    @staticmethod
    def endpoints(points, poses):
        poses = np.atleast_2d(poses)
        c, s = np.cos(poses[:, 2, None]), np.sin(poses[:, 2, None])
        return np.stack((poses[:, 0, None] + c * points[:, 0] - s * points[:, 1],
                         poses[:, 1, None] + s * points[:, 0] + c * points[:, 1]), axis=-1)

    def endpoint_scores(self, points, poses, cancel=None):
        scores = np.empty(len(poses))
        same_heading = len(poses) > 0 and np.all(poses[:, 2] == poses[0, 2])
        if same_heading:
            # Each coarse-search batch shares one heading: rotate each ray once.
            angle = poses[0, 2] - self.origin[2]
            c, s = math.cos(angle), math.sin(angle)
            ray_x = (c * points[:, 0] - s * points[:, 1]) / self.resolution
            ray_y = (s * points[:, 0] + c * points[:, 1]) / self.resolution
            delta = poses[:, :2] - self.origin[:2]
            center_x = (self.c * delta[:, 0] + self.s * delta[:, 1]) / self.resolution
            center_y = (-self.s * delta[:, 0] + self.c * delta[:, 1]) / self.resolution
        for begin in range(0, len(poses), 2048):
            if cancel is not None and cancel.is_set():
                raise RuntimeError('Map search interrupted')
            batch = poses[begin:begin + 2048]
            # Score directly in map cells, reusing the map's precomputed likelihoods.
            # Separate x/y arrays avoid constructing and copying an N x rays x 2 tensor.
            if same_heading:
                ix = np.floor(center_x[begin:begin + 2048, None] + ray_x).astype(np.intp)
                iy = np.floor(center_y[begin:begin + 2048, None] + ray_y).astype(np.intp)
            else:
                c, s = np.cos(batch[:, 2, None]), np.sin(batch[:, 2, None])
                dx = batch[:, 0, None] + c * points[:, 0] - s * points[:, 1] - self.origin[0]
                dy = batch[:, 1, None] + s * points[:, 0] + c * points[:, 1] - self.origin[1]
                ix = np.floor((self.c * dx + self.s * dy) / self.resolution).astype(np.intp)
                iy = np.floor((-self.s * dx + self.c * dy) / self.resolution).astype(np.intp)
            inside = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
            np.clip(ix, 0, self.width - 1, out=ix)
            np.clip(iy, 0, self.height - 1, out=iy)
            likelihoods = self.endpoint_likelihood[iy, ix]
            likelihoods[~inside] = self.outside_likelihood
            values = likelihoods.mean(axis=1)
            values[~self.free(batch[:, :2])] = -1.0
            scores[begin:begin + len(batch)] = values
        return scores

    @staticmethod
    def distinct(pose, others):
        return all(np.linalg.norm(np.asarray(pose[:2]) - other[:2]) > 0.8
                   or abs(math.atan2(math.sin(pose[2] - other[2]),
                                     math.cos(pose[2] - other[2]))) > math.radians(20)
                   for other in others)

    def rank(self, points, poses, with_indices=False, cancel=None):
        rows = []
        ranges = np.linalg.norm(points, axis=1)
        steps = np.arange(0.25, ranges.max(), 0.1)
        usable = steps[None, :] < ranges[:, None] - 0.2
        for index, pose in enumerate(poses):
            if cancel is not None and cancel.is_set():
                raise RuntimeError('Map search interrupted')
            if not self.free(np.asarray(pose[:2])):
                continue
            endpoints = self.endpoints(points, pose)[0]
            distances = self.wall_distances(endpoints)
            rays = (np.asarray(pose[:2]) + (endpoints - pose[:2])[:, None, :]
                    / ranges[:, None, None] * steps[None, :, None])
            indices, inside = self.cells(rays)
            occupied = np.zeros(inside.shape, dtype=bool)
            occupied[inside] = self.grid[indices[..., 1][inside], indices[..., 0][inside]] >= 65
            occupied &= usable
            # Two consecutive samples reduce sensitivity to isolated map pixels.
            conflict = float((occupied[:, :-1] & occupied[:, 1:]).any(axis=1).mean())
            likelihoods = np.exp(-0.5 * (distances / 0.2) ** 2)
            # Limit the effect of a few changed objects or stray returns on the match score.
            # The hit ratio and occupied-ray checks still include every point.
            hit = float(np.sort(likelihoods)[int(len(likelihoods) * 0.1):].mean())
            rows.append(dict(pose=list(pose), score=hit - 0.5 * conflict,
                             hit_ratio=float((distances <= 0.2).mean()),
                             ray_conflict=conflict))
            if with_indices:
                rows[-1]['candidate_index'] = index
        return sorted(rows, key=lambda row: row['score'], reverse=True)

    def supported(self, row):
        return (row['score'] >= self.min_score and row['hit_ratio'] >= 0.8
                and row['ray_conflict'] <= 0.2)

    def accepted(self, rows):
        if not rows:
            return False
        best = rows[0]
        margin = best['score'] - rows[1]['score'] if len(rows) > 1 else 0.0
        return self.supported(best) and margin >= self.min_margin

    def observe(self, points, candidates, previous_odom, current_odom, views=None):
        """Combine aligned recent views while checking the newest measurement."""
        moved = [move_pose(row['pose'], previous_odom, current_odom) for row in candidates]
        current = self.rank(points, moved, with_indices=True)
        rows = current
        if views:
            combined, _ = self.combine_views(views, current_odom)
            rows = self.rank(combined, moved, with_indices=True)
        current_by_index = {row['candidate_index']: row for row in current}
        for row in rows:
            original = candidates[row['candidate_index']]
            latest = current_by_index[row['candidate_index']]
            row['current_score'] = latest['score']
            row['score'] = min(original['score'], row['score'], latest['score'])
            row['hit_ratio'] = min(original['hit_ratio'], row['hit_ratio'], latest['hit_ratio'])
            row['ray_conflict'] = max(original['ray_conflict'], row['ray_conflict'], latest['ray_conflict'])
        return sorted(rows, key=lambda row: row['score'], reverse=True)

    def search(self, points, cancel):
        """Search all sampled free positions and headings, then refine candidates."""
        stride = max(1, round(self.xy_step / self.resolution))
        ys, xs = np.where(self.grid[::stride, ::stride] == 0)
        x, y = (xs * stride + 0.5) * self.resolution, (ys * stride + 0.5) * self.resolution
        centers = np.column_stack((self.origin[0] + self.c * x - self.s * y,
                                   self.origin[1] + self.s * x + self.c * y))
        if not len(centers):
            raise ValueError('Search grid has no free positions')
        candidates = []
        # ponytail: grid search suits current field maps; larger maps need a hierarchical search.
        for angle in np.arange(-math.pi, math.pi, self.angle_step):
            if cancel.is_set():
                raise RuntimeError('Map search interrupted')
            poses = np.column_stack((centers, np.full(len(centers), angle)))
            scores = self.endpoint_scores(points, poses, cancel=cancel)
            for _ in range(12):
                index = int(np.argmax(scores))
                if scores[index] < 0:
                    break
                candidates.append((float(scores[index]), poses[index]))
                scores[np.linalg.norm(centers - centers[index], axis=1) <= 0.8] = -1
        selected = []
        for _, pose in sorted(candidates, key=lambda item: item[0], reverse=True):
            if self.distinct(pose, selected):
                selected.append(pose)
            if len(selected) == 50:
                break
        dx, dy, da = np.meshgrid(
            np.arange(-self.xy_step, self.xy_step + 0.001, 0.05),
            np.arange(-self.xy_step, self.xy_step + 0.001, 0.05),
            np.arange(-self.angle_step, self.angle_step + 0.0001, math.radians(0.5)),
            indexing='ij')
        offsets = np.column_stack((dx.ravel(), dy.ravel(), da.ravel()))
        refined = []
        for pose in selected:
            if cancel.is_set():
                raise RuntimeError('Map search interrupted')
            poses = pose + offsets
            best = poses[int(np.argmax(self.endpoint_scores(points, poses, cancel=cancel)))]
            best[2] = math.atan2(math.sin(best[2]), math.cos(best[2]))
            if self.distinct(best, refined):
                refined.append(best)
        return self.rank(points, refined, cancel=cancel)
