"""Check pinned K1 assets and compare MuJoCo poses with URDF kinematics.

Optional Booster CSV files are replayed as poses, without physics or control.
Source joint-limit violations are reported without changing the motion.
"""

import argparse
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import runpy
import time
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'assets/booster_k1'


def transform(position, rotation):
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = position
    return matrix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motion', type=Path, action='append', default=[],
                        help='Booster K1 CSV. Repeat for multiple clips.')
    parser.add_argument('--fps', type=float, default=50.0,
                        help='CSV frame rate; Booster documents these clips at 50 FPS.')
    parser.add_argument('--view', action='store_true', help='Show the CSV pose replay.')
    parser.add_argument('--out', type=Path, default=Path('outputs/k1_model_check.json'))
    args = parser.parse_args()
    if args.fps <= 0:
        parser.error('--fps must be positive')

    manifest = json.loads((ASSETS / 'manifest.json').read_text())
    for group in [manifest['files'], manifest['legacy']['files']]:
        for relative, expected in group.items():
            actual = hashlib.sha256((ASSETS / relative).read_bytes()).hexdigest()
            if actual != expected:
                raise ValueError(f'Asset checksum mismatch: {relative}')

    # Read the path table without importing optional retargeting dependencies.
    params = runpy.run_path(str(ROOT / 'general_motion_retargeting/params.py'))
    model = mj.MjModel.from_xml_path(str(params['ROBOT_XML_DICT']['booster_k1']))
    legacy = mj.MjModel.from_xml_path(str(params['ROBOT_XML_DICT']['booster_k1_legacy']))
    if (model.nq, model.nv, legacy.nq, legacy.nv) != (29, 28, 29, 28):
        raise ValueError('Expected a free root and 22 hinge joints in both models')
    for key in ['booster_k1', 'booster_k1_legacy']:
        target = model if key == 'booster_k1' else legacy
        config = json.loads(params['IK_CONFIG_DICT']['smplx'][key].read_text())
        names = {config['robot_root_name'], params['ROBOT_BASE_DICT'][key]}
        for table in ['ik_match_table1', 'ik_match_table2']:
            names.update(config[table])
        for name in names:
            if mj.mj_name2id(target, mj.mjtObj.mjOBJ_BODY, name) < 0:
                raise ValueError(f'Missing body in {key}: {name}')

    urdf = ET.parse(ASSETS / 'booster_22dof/K1_22dof.urdf').getroot()
    links = {link.get('name') for link in urdf.findall('link')}
    joints = []
    for joint in urdf.findall('joint'):
        origin = joint.find('origin')
        xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
        rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
        entry = dict(name=joint.get('name'), kind=joint.get('type'),
                     parent=joint.find('parent').get('link'),
                     child=joint.find('child').get('link'),
                     origin=transform(xyz, Rotation.from_euler('xyz', rpy).as_matrix()))
        if entry['kind'] != 'fixed':
            index = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, entry['name'])
            if index < 0:
                raise ValueError(f'Missing joint: {entry["name"]}')
            entry['index'] = index
            entry['address'] = int(model.jnt_qposadr[index])
            entry['axis'] = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
            limit = joint.find('limit')
            np.testing.assert_allclose(model.jnt_axis[index], entry['axis'], atol=1e-12)
            np.testing.assert_allclose(model.jnt_range[index],
                                       [float(limit.get('lower')), float(limit.get('upper'))],
                                       atol=1e-12)
        joints.append(entry)
    active = [joint for joint in joints if joint['kind'] != 'fixed']
    if len(active) != 22:
        raise ValueError('Expected 22 URDF joints')
    mass = sum(float(x.get('value')) for x in urdf.findall('link/inertial/mass'))
    np.testing.assert_allclose(model.body_mass.sum(), mass, atol=1e-9)
    data = mj.MjData(model)
    max_position_error = 0.0
    max_rotation_error = 0.0
    poses_checked = 0

    def check_pose(qpos):
        nonlocal max_position_error, max_rotation_error, poses_checked
        if not np.isfinite(qpos).all():
            raise ValueError('Motion contains non-finite values')
        data.qpos[:] = qpos
        mj.mj_forward(model, data)
        poses = {'trunk': transform(qpos[:3], Rotation.from_quat(qpos[[4, 5, 6, 3]]).as_matrix())}
        pending = list(joints)
        while pending:
            remaining = []
            for joint in pending:
                if joint['parent'] not in poses:
                    remaining.append(joint)
                    continue
                local = joint['origin']
                if joint['kind'] != 'fixed':
                    rotation = Rotation.from_rotvec(joint['axis'] * qpos[joint['address']])
                    local = local @ transform(np.zeros(3), rotation.as_matrix())
                poses[joint['child']] = poses[joint['parent']] @ local
            if len(remaining) == len(pending):
                raise ValueError('URDF tree is disconnected')
            pending = remaining
        for name in links:
            index = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, name)
            if index < 0:
                raise ValueError(f'Missing URDF body in XML: {name}')
            expected = poses[name]
            max_position_error = max(max_position_error,
                                     float(np.abs(data.xpos[index] - expected[:3, 3]).max()))
            max_rotation_error = max(max_rotation_error,
                                     float(np.abs(data.xmat[index].reshape(3, 3) - expected[:3, :3]).max()))
        poses_checked += 1

    rng = np.random.default_rng(42)
    check_pose(model.qpos0.copy())
    for _ in range(5):
        qpos = model.qpos0.copy()
        qpos[:3] = rng.uniform(-1, 1, 3)
        qpos[3:7] = Rotation.from_rotvec(rng.uniform(-1, 1, 3)).as_quat()[[3, 0, 1, 2]]
        for joint in active:
            qpos[joint['address']] = rng.uniform(*model.jnt_range[joint['index']])
        check_pose(qpos)

    motions = []
    for path in args.motion:
        frames = np.atleast_1d(np.genfromtxt(path, delimiter=',', names=True))
        if not len(frames):
            raise ValueError(f'Empty motion: {path}')
        positions = np.column_stack([frames[f'root_pos_{axis}'] for axis in 'xyz'])
        quaternions = np.column_stack([frames[f'root_quat_{axis}'] for axis in 'wxyz'])
        norm_error = float(np.abs(np.linalg.norm(quaternions, axis=1) - 1).max())
        if norm_error > 1e-3:
            raise ValueError(f'Invalid root quaternion in {path}')
        qposes = np.tile(model.qpos0, (len(frames), 1))
        qposes[:, :3] = positions
        qposes[:, 3:7] = quaternions / np.linalg.norm(quaternions, axis=1, keepdims=True)
        violations = {}
        aliases = {}
        for joint in active:
            name = joint['name']
            column = name if name in frames.dtype.names else name.removeprefix('aa')
            if column not in frames.dtype.names:
                raise ValueError(f'Missing CSV joint: {name}')
            if column != name:
                aliases[column] = name
            values = frames[column]
            qposes[:, joint['address']] = values
            lower, upper = model.jnt_range[joint['index']]
            excess = np.maximum(lower - values, values - upper)
            count = int((excess > 1e-6).sum())
            if count:
                violations[name] = dict(frames=count, max_excess_rad=float(excess.max()))
        for qpos in qposes:
            check_pose(qpos)
        motions.append(dict(file=path.name,
                            sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                            frames=len(frames), fps=args.fps,
                            column_aliases=aliases, max_quaternion_norm_error=norm_error,
                            joint_limit_violations=violations,
                            within_joint_limits=not violations))
        if args.view:
            import mujoco.viewer
            with mujoco.viewer.launch_passive(model, data) as viewer:
                for qpos in qposes:
                    if not viewer.is_running():
                        break
                    start = time.monotonic()
                    data.qpos[:] = qpos
                    mj.mj_forward(model, data)
                    viewer.cam.lookat[:] = qpos[:3]
                    viewer.cam.distance = 2.0
                    viewer.sync()
                    time.sleep(max(0, 1 / args.fps - (time.monotonic() - start)))

    passed = max_position_error < 1e-6 and max_rotation_error < 2e-6
    report = dict(model_check_passed=passed, source_commit=manifest['commit'],
                  packages={name: version(name) for name in ['mujoco', 'numpy', 'scipy']},
                  poses_checked=poses_checked, bodies_per_pose=len(links),
                  max_position_error_m=max_position_error,
                  max_rotation_matrix_error=max_rotation_error,
                  mass_kg=mass, motions=motions,
                  scope='MuJoCo pose replay and independent URDF kinematics; no physics or human retargeting')
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    if not passed:
        raise SystemExit('K1 model comparison failed')


if __name__ == '__main__':
    main()
