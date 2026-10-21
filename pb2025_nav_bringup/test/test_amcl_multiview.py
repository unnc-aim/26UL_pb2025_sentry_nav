"""Check complementary gimbal views and current-measurement validation."""
import math
from pathlib import Path
import sys
from threading import Event

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from scan_map_matcher import ScanMapMatcher


def main():
    grid = np.zeros((200, 200), dtype=np.int8)
    # Each false position shares one wall with the true position.
    grid[19:42, 50] = 100
    grid[50, 19:42] = 100
    grid[19:42, 140] = 100
    grid[140, 19:42] = 100
    matcher = ScanMapMatcher(grid, .1, [0, 0, 0])
    offsets = np.linspace(-1, 1, 160)
    front = np.column_stack((np.full(160, 2.05), offsets))
    side = np.column_stack((offsets, np.full(160, 2.05)))
    candidates = [dict(pose=pose, score=1., hit_ratio=1., ray_conflict=0.)
                  for pose in ([3., 3., 0.], [12., 3., 0.], [3., 12., 0.])]
    origin = [0., 0., 0.]
    for view in (front, side):
        assert not matcher.accepted(matcher.observe(view, candidates, origin, origin))

    # Measurements from different times are aligned by measured base motion.
    moved = [.04, -.03, .35]
    c, s = math.cos(moved[2]), math.sin(moved[2])
    shifted = (side - moved[:2]) @ np.array([[c, -s], [s, c]])
    views = [(front, origin), (shifted, moved)]
    checked = matcher.observe(side, candidates, origin, origin, views=views)
    assert matcher.accepted(checked), checked
    assert checked[0]['candidate_index'] == 0, checked
    assert checked[0]['current_score'] >= matcher.min_score

    # Historical agreement alone keeps publication waiting on a bad current view.
    bad = side + [0., 4.]
    checked = matcher.observe(bad, candidates, origin, origin, views=views)
    assert not matcher.accepted(checked), checked
    assert checked[0]['current_score'] < matcher.min_score, checked

    for origin in ([0., 0., 0.], [-2., 1., .63]):
        matcher = ScanMapMatcher(grid, .1, origin)
        poses = np.array([[3., 3., .35], [12., 3., .35], [3., 12., .35], [-20., 20., .35]])
        distances = matcher.wall_distances(matcher.endpoints(front, poses))
        expected = np.exp(-.5 * (distances / .2) ** 2).mean(axis=1)
        expected[~matcher.free(poses[:, :2])] = -1.
        assert np.allclose(matcher.endpoint_scores(front, poses), expected, rtol=0, atol=1e-14)
    cancel = Event()
    cancel.set()
    try:
        matcher.endpoint_scores(front, poses, cancel=cancel)
    except RuntimeError:
        pass
    else:
        raise AssertionError('Scoring must honor search cancellation')
    print('PASS: complementary views, alignment, current view, scoring equivalence and cancellation')


if __name__ == '__main__':
    main()
