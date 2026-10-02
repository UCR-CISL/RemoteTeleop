#!/usr/bin/env python3
"""Stage replay inputs and write a ready-to-run batch manifest.

Mounted CooperScene data is only read. Transfers are resumable and never delete
files. Dry-run prints the transfer plan without SSH, rsync, or output writes.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path, PurePosixPath
import shlex
import subprocess
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.common.config import RemoteTeleopConfig
from src.localization.cooperscene_calibration import camera_intrinsic
from src.localization.transforms import _rigid_transform

LAMBDA_REPO = "/home/justin/Documents/CISL-Projects/RemoteTeleop"
ALIEN_REPO = "/home/coop3r-slam/Documents/RemoteTeleop"

# Self-contained so inspection does not depend on remote source-code staging.
INSPECT_SCRIPT = '''import json, sys, struct
from mcap.reader import make_reader
from foxglove_schemas_protobuf.CompressedImage_pb2 import CompressedImage
path, agent = sys.argv[1:]
with open(path, 'rb') as stream:
    reader = make_reader(stream)
    summary = reader.get_summary()
    topic = '/' + agent + '_Camera'
    channels = [c for c in summary.channels.values() if c.topic == topic]
    if len(channels) != 1:
        raise ValueError('Missing or ambiguous camera topic: ' + topic)
    count = summary.statistics.channel_message_counts[channels[0].id]
    metadata = next((m.metadata for m in reader.iter_metadata() if m.name == 'teleop'), {})
    _, _, record = next(reader.iter_messages(topics=[topic]))
    image = CompressedImage.FromString(record.data)
    if image.format.lower() != 'png' or image.data[:8] != b'\\x89PNG\\r\\n\\x1a\\n':
        raise ValueError('Expected source PNG camera frame')
    width, height = struct.unpack('>II', image.data[16:24])
    if count <= 0:
        raise ValueError('Empty camera stream')
    print(json.dumps(dict(expected_samples=count, camera_source_size=[width, height], metadata=metadata)))
'''


class BatchPreparer:
    """Inspect and stage inputs before atomically publishing a queue manifest."""

    def __init__(self, args, run=subprocess.run):
        self.args = args
        self.config = RemoteTeleopConfig.from_yaml(args.config)
        self.run_command = run

    def command(self, command):
        print(shlex.join(str(part) for part in command), flush=True)
        if self.args.dry_run:
            return ""
        result = self.run_command(list(map(str, command)), check=True, text=True,
                                  stdout=subprocess.PIPE)
        return result.stdout

    @staticmethod
    def host(endpoint):
        return f"{endpoint.user}@{endpoint.address}"

    def ssh(self, endpoint, command):
        return self.command(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=15",
                             self.host(endpoint), shlex.join(map(str, command))])

    def copy(self, source, destination):
        self.command(["rsync", "-a", "--partial", "--protect-args", "-e",
                      "ssh -o BatchMode=yes -o ConnectTimeout=15", source, destination])

    def inspect(self, path, agent, endpoint=None):
        command = [self.args.vehicle_python if endpoint else sys.executable,
                   "-c", INSPECT_SCRIPT, str(path), str(agent)]
        output = self.ssh(endpoint, command) if endpoint else self.command(command)
        return json.loads(output) if output else None

    def nuscenes_localization(self, scene):
        paths = {name: self.args.nuscenes_localization_root / scene / filename
                 for name, filename in (("nuscenes_localization_transform", "localization.json"),
                                        ("localization_reference_transform", "reference.json"))}
        for name, path in paths.items():
            output = self.ssh(self.config.vehicle, ["cat", path])
            if self.args.dry_run:
                continue
            document = json.loads(output)
            if str(document.get("scene_id")) != scene:
                raise ValueError(f"Localization scene mismatch: {path}")
            if name == "nuscenes_localization_transform" and document.get("registration", {}).get("accepted") is not True:
                raise ValueError(f"Localization registration not accepted: {path}")
            transform = document.get("reference_T_dataset", document.get("gaussian_T_nuscenes_global")) if name == "localization_reference_transform" else document.get("gaussian_T_nuscenes_global")
            _rigid_transform(transform, str(path))
        return {**{key: str(value) for key, value in paths.items()},
                "localization_reference_kind": "recorded_pose_alignment_baseline"}

    def prepare(self):
        args, vehicle, remote = self.args, self.config.vehicle, self.config.remote
        for path in (args.cooperscene_localization_transform,
                     args.cooperscene_stored_localization_transform,
                     args.cooperscene_reference_transform):
            if not args.dry_run and not path.is_file():
                raise FileNotFoundError(path)
            if not args.dry_run:
                _rigid_transform(json.loads(path.read_text())["gaussian_T_cooperscene"], str(path))
        if not args.dry_run:
            config = json.loads(args.cooperscene_localization_transform.read_text()).get("config", {})
            if config.get("enu_prior_position_mode") != "xyz":
                raise ValueError("CooperScene localization must use enu_prior_position_mode=xyz")
            if PurePosixPath(config.get("splat_path", "")).name != args.cooperscene_splat.name:
                raise ValueError("CooperScene localization splat_path must match selected map")
        vehicle_inputs = vehicle.repo_path / "artifacts/batch-inputs"
        remote_maps = remote.repo_path / "data/batch-nuscenes"
        self.ssh(vehicle, ["mkdir", "-p", vehicle_inputs])
        self.ssh(remote, ["mkdir", "-p", remote_maps])
        self.ssh(remote, ["test", "-s", args.cooperscene_splat])
        if not args.dry_run:
            args.cache.mkdir(parents=True, exist_ok=True)
        transforms = {}
        for name, path in (("cooperscene_localization_transform", args.cooperscene_localization_transform),
                           ("cooperscene_stored_localization_transform", args.cooperscene_stored_localization_transform),
                           ("localization_reference_transform", args.cooperscene_reference_transform)):
            destination = vehicle_inputs / (name + ".json")
            self.copy(str(path), f"{self.host(vehicle)}:{destination}")
            transforms[name] = str(destination)
        transforms["localization_reference_kind"] = "shared_enu_prior"
        runs = []
        for scene in args.scenes:
            localization = self.nuscenes_localization(scene)
            mcap = args.nuscenes_mcap_root / f"{scene}.mcap"
            info = self.inspect(mcap, 1, vehicle)
            camera = None
            if info:
                metadata = info.pop("metadata")
                if metadata.get("dataset") != "nuscenes":
                    raise ValueError(f"Unexpected dataset for {mcap}")
                camera = self.camera(json.loads(metadata["camera_intrinsic"]))
            source_map = args.nuscenes_map_root / scene / "exports/background_40000.ply"
            cache = args.cache / f"nuscenes-{scene}.ply"
            target = remote_maps / f"{scene}.ply"
            self.copy(f"{self.host(vehicle)}:{source_map}", str(cache))
            self.copy(str(cache), f"{self.host(remote)}:{target}")
            self.ssh(remote, ["test", "-s", target])
            if info:
                runs.append(dict(id=f"nuscenes-{scene}", dataset="nuscenes", scene=scene,
                                 mcap=str(mcap), splat=str(target), agent=1,
                                 **camera, **info, **localization))
        for take in args.takes:
            source = args.cooperscene_root / f"{take}.mcap"
            target = vehicle_inputs / f"cooperscene-{take}.mcap"
            inspections = {agent: self.inspect(source, agent) for agent in args.agents}
            self.copy(str(source), f"{self.host(vehicle)}:{target}")
            for agent in args.agents:
                info = inspections[agent]
                if info:
                    info.pop("metadata")
                    runs.append(dict(id=f"cooperscene-{take}-{agent}", dataset="cooperscene",
                                     scene=str(take), mcap=str(target), splat=str(args.cooperscene_splat),
                                     agent=agent, **self.camera(camera_intrinsic(agent).reshape(-1)),
                                     **info, **transforms))
        if args.dry_run:
            print("Dry-run only: manifest requires successful data inspection and staging.")
            return None
        manifest = dict(version=1, defaults=dict(config_path=str(args.config.resolve()),
                        remote_python=args.remote_python, vehicle_python=args.vehicle_python,
                        timeout_seconds=900), runs=runs)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.output.with_suffix(args.output.suffix + ".partial")
        temporary.write_text(json.dumps(manifest, indent=2) + "\n")
        temporary.replace(args.output)
        return manifest

    @staticmethod
    def camera(values):
        if len(values) != 9 or not all(math.isfinite(v) for v in values) or values[0] <= 0 or values[4] <= 0:
            raise ValueError("Expected a valid flattened 3x3 camera intrinsic")
        return dict(camera_fx=float(values[0]), camera_fy=float(values[4]),
                    camera_cx=float(values[2]), camera_cy=float(values[5]))


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--config", type=Path, default=Path("cfg/alien3_lambda_proxy_batch.yaml"))
    result.add_argument("--output", type=Path, default=Path("artifacts/proxy-batch/manifest.json"))
    result.add_argument("--cache", type=Path, default=Path("artifacts/proxy-batch/map-cache"))
    result.add_argument("--scenes", nargs="+", default=["152", "164", "171", "200", "209", "359", "529", "916"])
    result.add_argument("--takes", nargs="+", type=int, choices=range(1, 15), default=list(range(1, 6)))
    result.add_argument("--agents", nargs="+", type=int, choices=(1, 2, 3), default=[1, 2, 3])
    result.add_argument("--cooperscene-root", type=Path, default=Path("/mnt/bcc-data/websites/data.ucr.edu-datasets-cooperscene/mcap"))
    result.add_argument("--cooperscene-localization-transform", type=Path, required=True)
    result.add_argument("--cooperscene-stored-localization-transform", type=Path, default=Path("config/cooperscene_native_identity.json"))
    result.add_argument("--cooperscene-reference-transform", type=Path, default=Path("config/cooperscene_riverside_iowa_enu_prior.json"))
    result.add_argument("--cooperscene-splat", type=PurePosixPath, default=PurePosixPath(ALIEN_REPO + "/data/riverside_iowa.ply"))
    result.add_argument("--nuscenes-mcap-root", type=PurePosixPath, default=PurePosixPath(LAMBDA_REPO + "/artifacts/nuscenes_10hz_foxglove_mcap"))
    result.add_argument("--nuscenes-localization-root", type=PurePosixPath, default=PurePosixPath(LAMBDA_REPO + "-proxy-batch/artifacts/batch-localization/nuscenes"))
    result.add_argument("--nuscenes-map-root", type=PurePosixPath, default=PurePosixPath("/home/justin/bilateral-driving-repro/ckpts/nuscenes_pretrained_checkpoints_6cams"))
    result.add_argument("--vehicle-python", default=LAMBDA_REPO + "-proxy-batch/batch-python")
    result.add_argument("--remote-python", default=ALIEN_REPO + "-proxy-batch/batch-python")
    result.add_argument("--dry-run", action="store_true")
    return result


if __name__ == "__main__":
    BatchPreparer(parser().parse_args()).prepare()
