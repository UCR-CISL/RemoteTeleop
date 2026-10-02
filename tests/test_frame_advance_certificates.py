import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from scripts.investigation.advance_cooperscene_frames import CooperFrameAdvanceSearch, strict_rejection_certificate
from scripts.investigation.first_frame_xyz import registration_config


def test_certificate_rejects_impossible_cloud_and_does_not_reject_exact_overlap():
    points = np.tile([20., 0, 0], (100, 1))
    config = registration_config("cooperscene")
    impossible = strict_rejection_certificate(points, np.full(100, 20.), config)
    assert impossible["certified_impossible"]
    assert impossible["inlier_ratio_upper_bound"] == 0
    assert not strict_rejection_certificate(points, np.zeros(100), config)["certified_impossible"]


def test_residual_lower_bound_holds_under_allowed_map_frame_corrections():
    rng = np.random.default_rng(82)
    points = rng.normal(size=(200, 3)) * 25
    target = rng.normal(size=(350, 3)) * 20
    tree = cKDTree(target)
    original = tree.query(points)[0]
    config = registration_config("cooperscene")
    lower = np.maximum(0, original - config.max_correction_translation
        - 2*np.sin(np.radians(config.max_correction_rotation_degrees)/2)*np.linalg.norm(points, axis=1) - 1e-9)
    for _ in range(20):
        axis = rng.normal(size=3)
        rotation = Rotation.from_rotvec(axis/np.linalg.norm(axis)*np.radians(rng.uniform(0, 3))).as_matrix()
        translation = rng.normal(size=3)
        translation *= rng.uniform(0, 1)/np.linalg.norm(translation)
        actual = tree.query(points @ rotation.T + translation)[0]
        assert np.all(actual >= lower)


def test_frame_selection_respects_strict_preference_and_inclusive_hard_cutoff():
    record = dict(accepted=True, fresh_error={"xyz_m": .09}, scan_registration={"accepted": True})
    assert CooperFrameAdvanceSearch.hits(dict(record, sequence_fraction=.1499))["target_preferred"]
    assert not CooperFrameAdvanceSearch.hits(dict(record, sequence_fraction=.15))["target_preferred"]
    assert CooperFrameAdvanceSearch.hits(dict(record, sequence_fraction=.25))["target_eligible"]
    assert not CooperFrameAdvanceSearch.hits(dict(record, sequence_fraction=.2501))["target_eligible"]
