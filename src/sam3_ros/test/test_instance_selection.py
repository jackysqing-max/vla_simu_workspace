import numpy as np

from sam3_ros.instance_selection import select_instance_indices


def test_instance_selection_applies_relative_threshold_and_limit():
    scores = np.asarray([0.90, 0.60, 0.19, 0.18, 0.05])

    selected = select_instance_indices(
        scores,
        absolute_threshold=0.05,
        relative_threshold=0.20,
        max_instances=3,
    )

    assert selected.tolist() == [0, 1, 2]


def test_instance_selection_does_not_promote_below_absolute_threshold():
    selected = select_instance_indices(
        np.asarray([0.04, 0.03]),
        absolute_threshold=0.05,
        relative_threshold=0.20,
        max_instances=16,
    )

    assert selected.size == 0
