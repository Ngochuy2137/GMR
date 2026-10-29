"""Check both K1 models with synthetic human motion and different starts."""

import json
import os
from importlib.metadata import version
from pathlib import Path

import numpy as np
import pytest
from synthetic_motion import TPOSE_QUAT, build_frames

from general_motion_retargeting import GeneralMotionRetargeting


@pytest.fixture(scope="module")
def results():
    """Optionally save measured errors after the tests finish."""
    records = []
    yield records
    output = os.environ.get("GMR_K1_REPORT")
    if output:
        path = Path(output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(
                {
                    "upstream_commit": "25f56db79ac411258b377b6d4ccba0c8ff3372ac",
                    "source_commit": "12a97516f57469078707afb587a5f84c1cb110d1",
                    "packages": {
                        name: version(name)
                        for name in ["mink", "mujoco", "numpy", "scipy", "daqp"]
                    },
                    "scope": "Synthetic human motion; no video, SMPL-X body model, physics, or real robot",
                    "cases": records,
                },
                indent=2,
            )
            + "\n"
        )


@pytest.mark.parametrize("robot", ["booster_k1", "booster_k1_legacy"])
@pytest.mark.parametrize(
    "start", ["default", "alternate_root_height", "alternate_timestep"]
)
def test_k1_synthetic_motion(robot, start, results):
    """Track human targets while respecting the selected model's joint limits."""
    retargeter = GeneralMotionRetargeting("smplx", robot, verbose=False)
    if start == "alternate_root_height":
        qpos = retargeter.configuration.data.qpos.copy()
        qpos[2] = 0.0 if qpos[2] > 0.5 else 1.0
        retargeter.configuration.update(qpos)
    elif start == "alternate_timestep":
        retargeter.model.opt.timestep = (
            0.001 if retargeter.model.opt.timestep > 0.0015 else 0.002
        )

    errors = {name: 0.0 for name in ["pelvis", "left_foot", "right_foot"]}
    poses = []
    for source in build_frames():
        frame = dict(source)
        # K1 tracks the head; upstream's G1/T1 fixture does not include it.
        frame["head"] = (
            frame["pelvis"][0] + np.array([0.0, 0.0, 0.75]),
            np.array(TPOSE_QUAT),
        )
        pose = retargeter.retarget(frame)
        poses.append(pose)
        for name in errors:
            task = retargeter.human_body_to_task2[name]
            errors[name] = max(
                errors[name],
                float(np.linalg.norm(task.compute_error(retargeter.configuration)[:3])),
            )

    poses = np.stack(poses)
    assert poses.shape == (90, 29)
    assert np.isfinite(poses).all()
    np.testing.assert_allclose(np.linalg.norm(poses[:, 3:7], axis=1), 1.0, atol=1e-6)
    assert (poses[:, 2] > 0.3).all() and (poses[:, 2] < 0.9).all()
    model = retargeter.model
    max_excess = 0.0
    for index in range(1, model.njnt):
        values = poses[:, model.jnt_qposadr[index]]
        lower, upper = model.jnt_range[index]
        max_excess = max(
            max_excess, float(np.maximum(lower - values, values - upper).max())
        )
    assert max_excess < 1e-6
    # The 5 cm target tolerance also includes the first frame after initialization.
    assert max(errors.values()) < 0.05, errors
    max_joint_step = float(np.abs(np.diff(poses[:, 7:], axis=0)).max())
    assert max_joint_step < 0.25
    results.append(
        {
            "robot": robot,
            "start": start,
            "timestep_s": float(model.opt.timestep),
            "frames": len(poses),
            "passed": True,
            "max_tracking_error_m": errors,
            "root_height_range_m": [float(poses[:, 2].min()), float(poses[:, 2].max())],
            "max_joint_limit_excess_rad": max_excess,
            "max_joint_step_rad": max_joint_step,
        }
    )
