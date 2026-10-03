"""Freeze one CooperScene map/reference correction and test held-out map residuals.

Offline analysis only. Registration agreement is not independent ground truth.
No per-frame correction or ICP is performed by this script. Existing files are read
only. Output holds a manifest so the mutable upstream search is snapshotted.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import csv
import hashlib
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.investigation.first_frame_xyz import map_proxy, source_points


def rigid_median(transforms):
    """Component medians in the local (<5 degree) rotation-vector chart."""
    transforms = np.asarray(transforms)
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec(np.median(
        Rotation.from_matrix(transforms[:, :3, :3]).as_rotvec(), axis=0)).as_matrix()
    result[:3, 3] = np.median(transforms[:, :3, 3], axis=0)
    return result


def summary(values):
    values = np.asarray(values)
    return dict(median=np.median(values, axis=0).tolist(),
                p05=np.percentile(values, 5, axis=0).tolist(),
                p95=np.percentile(values, 95, axis=0).tolist())


def write_csv(path, rows):
    if rows:
        with path.open("w") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


class ReferenceTiltValidation:
    def __init__(self, attempts, output, stride):
        self.attempts, self.output, self.stride = attempts, output, stride
        output.mkdir(parents=True, exist_ok=True)
        self.records = defaultdict(list)
        self.distribution = []
        self.references = {}
        self.manifest = []

    def collect(self):
        for directory in sorted(self.attempts.iterdir()):
            if not directory.is_dir():
                continue
            for path in sorted(directory.glob("f*.json")):
                content = path.read_bytes()
                record = json.loads(content)
                record["attempt_path"] = str(path)
                self.records[directory.name].append(record)
                self.manifest.append(dict(path=str(path), sha256=hashlib.sha256(content).hexdigest()))
                # Each frame's saved reference is checked, even though most are constant.
                with np.load(record["metadata"]["npz_path"]) as frame:
                    reference = frame["reference_map_T_dataset"]
                self.references[str(path)] = reference
                methods = dict(record["methods"])
                raw = record.get("evaluation", {}).get("shared_scan")
                if raw:
                    methods["raw_scan"] = dict(raw, map_T_dataset=raw["map_T_source"])
                for name, method in methods.items():
                    if not method.get("accepted") or "map_T_dataset" not in method:
                        continue
                    transform = np.asarray(method["map_T_dataset"])
                    correction = transform @ np.linalg.inv(reference)
                    rpy = Rotation.from_matrix(correction[:3, :3]).as_euler("xyz", degrees=True)
                    self.distribution.append(dict(run_id=directory.name, frame_offset=record["frame_offset"],
                        method=name, roll_deg=rpy[0], pitch_deg=rpy[1], yaw_deg=rpy[2],
                        correction_x_m=correction[0, 3], correction_y_m=correction[1, 3],
                        correction_z_m=correction[2, 3], correction=correction))
        print("Snapshot",len(self.manifest),"attempts",len(self.distribution),"accepted method results",flush=True)
        (self.output / "input-manifest.json").write_text(json.dumps(self.manifest, indent=2))
        write_csv(self.output / "accepted-transform-distribution.csv",[
            {k:v for k,v in r.items() if k != "correction"} for r in self.distribution])

    def calibrate(self):
        calibration = [r for r in self.distribution if r["run_id"] == "cooperscene-1-1"
                       and r["method"] == "raw_scan" and r["frame_offset"] <= 100]
        if not calibration:
            raise ValueError("No accepted raw scans in the declared calibration partition")
        self.correction = rigid_median([r["correction"] for r in calibration])
        record = dict(calibration_run="cooperscene-1-1", calibration_offsets=[r["frame_offset"] for r in calibration],
            estimator="component median translation and local rotation vectors of accepted raw scans",
            correction_convention="corrected_map_T_dataset = fixed_C @ original_reference_map_T_dataset",
            fixed_C=self.correction.tolist(), rpy_deg=Rotation.from_matrix(self.correction[:3,:3]).as_euler("xyz",degrees=True).tolist(),
            independent_ground_truth=False, fixed_global_pivot=True,
            limitations="Raw scan fits are initialized from inherited registration; calibration and held-out scans share map geometry. This evaluates frame consistency, not pose accuracy.")
        (self.output / "frozen-calibration.json").write_text(json.dumps(record, indent=2))
        print("Frozen correction",record["rpy_deg"],self.correction[:3,3],flush=True)
        aggregate = []
        for run_id in self.records:
            for method in ("raw_scan", "xyz", "bev", "bev_lidar_height"):
                rows = [r for r in self.distribution if r["run_id"] == run_id and r["method"] == method]
                if not rows:
                    aggregate.append(dict(run_id=run_id, method=method, accepted_count=0))
                    continue
                group = dict(run_id=run_id, method=method, accepted_count=len(rows),
                             rpy_deg=summary([[r["roll_deg"],r["pitch_deg"],r["yaw_deg"]] for r in rows]),
                             translation_m=summary([r["correction"][:3,3] for r in rows]))
                if run_id.startswith("cooperscene"):
                    group["remaining_rotation_deg"] = summary([np.degrees(Rotation.from_matrix(
                        r["correction"][:3,:3] @ self.correction[:3,:3].T).magnitude()) for r in rows])
                    group["remaining_translation_m"] = summary([np.linalg.norm(r["correction"][:3,3]-self.correction[:3,3]) for r in rows])
                aggregate.append(group)
        (self.output / "distribution-summary.json").write_text(json.dumps(aggregate, indent=2))
        self.plot_distributions()

    def plot_distributions(self):
        fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
        names = [r for r in self.records if r.startswith("cooperscene")]
        for axis, key in zip(axes, ("roll_deg", "pitch_deg", "yaw_deg")):
            values = [[r[key] for r in self.distribution if r["run_id"] == n and r["method"] == "raw_scan"] for n in names]
            axis.boxplot([v or [np.nan] for v in values], showfliers=False)
            axis.axhline(Rotation.from_matrix(self.correction[:3,:3]).as_euler("xyz",degrees=True)[("roll_deg","pitch_deg","yaw_deg").index(key)],color="red",label="Frozen take1/agent1 first 101 offsets")
            axis.axhline(0,color="gray",lw=0.7);axis.set_ylabel(key);axis.grid(alpha=0.2)
        axes[0].legend();axes[-1].set_xticks(range(1,len(names)+1),[n.replace("cooperscene-","") for n in names],rotation=45)
        axes[-1].set_xlabel("Take–agent (empty: no accepted raw scan)")
        fig.suptitle("Existing accepted raw-scan correction to original ENU reference; no per-frame refit")
        fig.tight_layout();fig.savefig(self.output/"correction-distribution.png",dpi=140);plt.close(fig)

    def score(self):
        rows=[]
        current_map=None
        tree=None
        target=None
        selected=[]
        for run_id,records in self.records.items():
            # Uniform deterministic coverage over available attempt offsets, not acceptance or error.
            offsets=np.asarray([r["frame_offset"] for r in records])
            chosen={int(np.argmin(abs(offsets-fraction*max(offsets)))) for fraction in (0,.25,.5,.75,1)}
            # Include known problematic support cases without using them for calibration.
            for offset in (871,):
                if run_id=="cooperscene-2-3" and offset in offsets:chosen.add(int(np.where(offsets==offset)[0][0]))
            selected.extend(records[i] for i in sorted(chosen))
        (self.output/"evaluation-selection.json").write_text(json.dumps([dict(run_id=r["run_id"],frame_offset=r["frame_offset"],attempt_path=r["attempt_path"]) for r in selected],indent=2))
        for record in selected:
            run_id=record["run_id"];md=record["metadata"];offset=record["frame_offset"]
            if md["map_path"]!=current_map:
                tree=None;target=None
                target=map_proxy(md["map_path"])
                tree=cKDTree(target)
                current_map=md["map_path"]
            with np.load(md["npz_path"]) as frame:
                source=source_points(frame,mask_dynamic=(md["dataset"]=="cooperscene"))[::self.stride]
            reference=self.references[record["attempt_path"]]
            transforms={"original_reference":reference}
            if run_id.startswith("cooperscene"):
                transforms["fixed_rigid_reference"]=self.correction @ reference
                rotation=self.correction.copy();rotation[:3,3]=0
                transforms["fixed_rotation_global_origin"]=rotation @ reference
            for method in ("xyz","bev","bev_lidar_height"):
                result=record["methods"].get(method,{})
                if "map_T_dataset" in result:
                    transforms["existing_"+method]=np.asarray(result["map_T_dataset"])
            scan=record.get("evaluation",{}).get("shared_scan",{})
            if "map_T_source" in scan:transforms["existing_raw_scan"]=np.asarray(scan["map_T_source"])
            projected={}
            for name,T in transforms.items():
                xyz=source@T[:3,:3].T+T[:3,3]
                distances=tree.query(xyz,workers=2)[0]
                row=dict(run_id=run_id,frame_offset=offset,variant=name,
                    split="calibration" if run_id=="cooperscene-1-1" and offset<=100 else "holdout",
                    source_points=len(source),target_points=len(target),source_stride=self.stride,
                    median_m=float(np.median(distances)),p95_m=float(np.percentile(distances,95)),
                    inlier_fraction_02m=float(np.mean(distances<=.2)),inlier_fraction_05m=float(np.mean(distances<=.5)),
                    existing_candidate_accepted=(scan.get("accepted") if name=="existing_raw_scan" else record["methods"].get(name.removeprefix("existing_"),{}).get("accepted") if name.startswith("existing_") else ""))
                rows.append(row);projected[name]=xyz
            print(run_id,offset,"reference",rows[-len(transforms)]["median_m"],"scored",len(transforms),flush=True)
            if run_id in ("cooperscene-1-1","cooperscene-2-2","cooperscene-3-3","cooperscene-5-2") and offset==0:
                self.plot_overlay(run_id,offset,target,projected)
            write_csv(self.output/"heldout-residuals.csv",rows)
        self.plot_residuals(rows)

    def score_independent(self, audit_path, extrinsics_path=None):
        """Compare GNSS/YAML coordinate recovery without using scan fits to set it."""
        audit_bytes = audit_path.read_bytes()
        audit = json.loads(audit_bytes)
        (self.output / "independent-transform-source.json").write_text(json.dumps(dict(
            path=str(audit_path.resolve()), sha256=hashlib.sha256(audit_bytes).hexdigest(),
            convention="map_T_native = original_origin_translation @ raw_gnss_enu_T_native",
            note="Raw GNSS/native pose pairs calibrate coordinate conventions. No map points or localization results set these transforms."), indent=2))
        selection = json.loads((self.output / "evaluation-selection.json").read_text())
        rows = []
        attitude_rows = []
        physical_rows = []
        physical_transforms = []
        extrinsics = json.loads(extrinsics_path.read_text()) if extrinsics_path else None
        target = None
        tree = None
        current_map = None
        for item in selection:
            run_id = item["run_id"]
            if not run_id.startswith("cooperscene"):
                continue
            record = json.loads(Path(item["attempt_path"]).read_text())
            md = record["metadata"]
            if md["map_path"] != current_map:
                target = map_proxy(md["map_path"])
                tree = cKDTree(target)
                current_map = md["map_path"]
            with np.load(md["npz_path"]) as frame:
                source = source_points(frame, mask_dynamic=True)[::self.stride]
                reference = frame["reference_map_T_dataset"]
                dataset_T_lidar = frame["dataset_T_lidar"]
                native_lidar_origin = dataset_T_lidar[:3,3]
            T = (np.asarray(audit["references"][run_id]["map_T_dataset"]) if "references" in audit
                 else reference @ np.asarray(audit["position_fits"][run_id]["raw_gnss_enu_T_native"]["transform"]))
            xyz = source @ T[:3, :3].T + T[:3, 3]
            distances = tree.query(xyz, workers=2)[0]
            rows.append(dict(run_id=run_id, frame_offset=item["frame_offset"],
                variant="independent_gnss_reference", source_points=len(source), source_stride=self.stride,
                median_m=float(np.median(distances)), p95_m=float(np.percentile(distances,95)),
                inlier_fraction_02m=float(np.mean(distances<=.2)), inlier_fraction_05m=float(np.mean(distances<=.5))))
            if "references" in audit:
                # Source-derived attitude hypothesis: preserve recovered sensor origin,
                # remove constant raw-INS/native attitude basis delta. This is not a
                # global map gauge correction and is never promoted to ground truth.
                C = np.asarray(audit["references"][run_id]["corrected_lidar_R_raw_ins_mean"])
                diagnostic = T.copy()
                diagnostic[:3,:3] = C.T @ T[:3,:3]
                diagnostic[:3,3] = T[:3,:3] @ native_lidar_origin + T[:3,3] - diagnostic[:3,:3] @ native_lidar_origin
                delta_origin = np.linalg.norm((diagnostic[:3,:3] @ native_lidar_origin + diagnostic[:3,3]) - (T[:3,:3] @ native_lidar_origin + T[:3,3]))
                attitude_xyz = source @ diagnostic[:3,:3].T + diagnostic[:3,3]
                attitude_distances = tree.query(attitude_xyz,workers=2)[0]
                attitude_rows.append(dict(run_id=run_id,frame_offset=item["frame_offset"],
                    variant="raw_ins_attitude_diagnostic",source_points=len(source),source_stride=self.stride,
                    median_m=float(np.median(attitude_distances)),p95_m=float(np.percentile(attitude_distances,95)),
                    inlier_fraction_02m=float(np.mean(attitude_distances<=.2)),
                    inlier_fraction_05m=float(np.mean(attitude_distances<=.5)),
                    lidar_origin_change_m=float(delta_origin)))
            if extrinsics is not None:
                agent = run_id.rsplit("-", 1)[1]
                E = np.asarray(extrinsics[agent][f"tower{agent}-xsens,tower{agent}-lidar"])
                map_T_xsens = T @ dataset_T_lidar
                map_T_xsens[:3,:3] = C.T @ map_T_xsens[:3,:3]
                physical = map_T_xsens @ np.linalg.inv(E) @ np.linalg.inv(dataset_T_lidar)
                physical_xyz = source @ physical[:3,:3].T + physical[:3,3]
                physical_distances = tree.query(physical_xyz, workers=2)[0]
                physical_rows.append(dict(run_id=run_id, frame_offset=item["frame_offset"],
                    variant="physical_lidar_reference",source_points=len(source),source_stride=self.stride,
                    median_m=float(np.median(physical_distances)),p95_m=float(np.percentile(physical_distances,95)),
                    inlier_fraction_02m=float(np.mean(physical_distances<=.2)),
                    inlier_fraction_05m=float(np.mean(physical_distances<=.5))))
                physical_transforms.append(dict(run_id=run_id,frame_offset=item["frame_offset"],
                    map_T_dataset=physical.tolist(),map_T_lidar=(physical@dataset_T_lidar).tolist()))
            print(run_id, item["frame_offset"], "independent median", rows[-1]["median_m"], flush=True)
            if run_id in ("cooperscene-1-1", "cooperscene-2-2", "cooperscene-3-3", "cooperscene-5-2") and item["frame_offset"] == 0:
                transforms = {"original_reference": reference, "independent_gnss_reference": T}
                candidate = record["methods"].get("bev", {}).get("map_T_dataset")
                if candidate is not None:
                    transforms["existing_bev"] = np.asarray(candidate)
                projected = {name: source @ pose[:3,:3].T + pose[:3,3] for name,pose in transforms.items()}
                self.plot_overlay(run_id, 0, target, projected, independent=True)
        write_csv(self.output / "independent-gnss-residuals.csv", rows)
        baseline = list(csv.DictReader((self.output / "heldout-residuals.csv").open()))
        write_csv(self.output / "raw-ins-attitude-residuals.csv", attitude_rows)
        write_csv(self.output / "physical-lidar-residuals.csv", physical_rows)
        (self.output/"physical-lidar-transforms.json").write_text(json.dumps(dict(extrinsics_path=str(extrinsics_path), extrinsics_sha256=hashlib.sha256(extrinsics_path.read_bytes()).hexdigest() if extrinsics_path else None, convention="map_T_lidar = map_T_raw_xsens @ inverse(lidar_T_xsens)", frames=physical_transforms),indent=2))
        self.plot_independent(rows, baseline, attitude_rows, physical_rows)

    def plot_independent(self, rows, baseline, attitude_rows, physical_rows):
        names = sorted({r["run_id"] for r in rows})
        fig, axes = plt.subplots(1,2,figsize=(15,6))
        for name,label,color in (("original_reference","Original reference","gray"),("fixed_rigid_reference","One scan-calibrated correction","tab:blue"),("independent_gnss_reference","Per-sequence raw GNSS convention","tab:green"),("physical_lidar_reference","Raw INS with physical LiDAR extrinsic","tab:red")):
            values = physical_rows if name=="physical_lidar_reference" else rows if name=="independent_gnss_reference" else baseline
            if not values:
                continue
            for ax,key in zip(axes,("median_m","p95_m")):
                ys=[np.median([float(r[key]) for r in values if r["run_id"]==run and r["variant"]==name]) for run in names]
                ax.plot(range(len(names)),ys,"o-",color=color,label=label)
                ax.set_ylabel(key);ax.set_xticks(range(len(names)),[n.replace("cooperscene-","") for n in names],rotation=65);ax.grid(alpha=.2)
        axes[0].legend();fig.suptitle("Same 76 CooperScene samples: corrections derived without per-frame cloud fitting")
        fig.tight_layout();fig.savefig(self.output/"independent-gnss-residuals.png",dpi=150);plt.close(fig)

    def plot_overlay(self,run_id,offset,target,projected,independent=False):
        center=np.median(projected.get("existing_bev",projected["original_reference"]),axis=0)
        keep=np.linalg.norm(target[:,:2]-center[:2],axis=1)<35
        local=target[keep][::max(1,int(keep.sum()/35000))]
        fig,axes=plt.subplots(1,3,figsize=(16,5),sharex=True,sharey=True)
        for ax,name in zip(axes,("original_reference", "independent_gnss_reference" if independent else "fixed_rigid_reference", "existing_bev")):
            ax.scatter(local[:,0],local[:,2],s=.25,c="gray",alpha=.3,rasterized=True)
            xyz=projected.get(name,projected["original_reference"]);ax.scatter(xyz[:,0],xyz[:,2],s=.4,c="tab:orange",alpha=.5,rasterized=True)
            ax.set_title(name);ax.set_xlabel("Native map X (m)");ax.set_ylim(-5,4);ax.set_xlim(center[0]-30,center[0]+30);ax.set_aspect("equal")
        axes[0].set_ylabel("Native map Z (m)");fig.suptitle(f"{run_id}, offset {offset}: static LiDAR orange / Gaussian means gray")
        fig.tight_layout();fig.savefig(self.output/f"overlay-{'gnss-' if independent else ''}{run_id}-{offset}.png",dpi=170);plt.close(fig)

    def plot_residuals(self,rows):
        names=[r for r in self.records if r.startswith("cooperscene")]
        fig,axes=plt.subplots(1,2,figsize=(15,6))
        for name,label,color in (("original_reference","Original ENU reference","gray"),("fixed_rigid_reference","One frozen rigid correction","tab:blue"),("existing_raw_scan","Existing raw scan candidates","tab:orange")):
            for ax,key in zip(axes,("median_m","p95_m")):
                vals=[np.median([r[key] for r in rows if r["run_id"]==run and r["variant"]==name and r["split"]=="holdout"]) if any(r["run_id"]==run and r["variant"]==name and r["split"]=="holdout" for r in rows) else np.nan for run in names]
                ax.plot(range(len(names)),vals,"o-",color=color,label=label);ax.set_ylabel(key);ax.set_xticks(range(len(names)),[n.replace("cooperscene-","") for n in names],rotation=65);ax.grid(alpha=.2)
        axes[0].legend();fig.suptitle("Held-out static LiDAR → map nearest-neighbor residuals (uniform available offsets)")
        fig.tight_layout();fig.savefig(self.output/"heldout-residuals.png",dpi=150);plt.close(fig)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempts",type=Path,default=Path("/tmp/RemoteTeleop-hybrid-road-height/artifacts/full-sequence-localization/attempts"))
    parser.add_argument("--output",type=Path,default=Path("artifacts/reference-orientation/tilt-validation"))
    parser.add_argument("--stride",type=int,default=8)
    parser.add_argument("--independent-audit",type=Path,help="Append independent GNSS scores using the existing frozen evaluation selection")
    parser.add_argument("--sensor-extrinsics",type=Path,help="Optional LiDAR_T_Xsens calibration export for physical pose reconstruction")
    args=parser.parse_args()
    validator=ReferenceTiltValidation(args.attempts,args.output,args.stride)
    if args.independent_audit:
        validator.score_independent(args.independent_audit,args.sensor_extrinsics)
    else:
        validator.collect();validator.calibrate();validator.score()
