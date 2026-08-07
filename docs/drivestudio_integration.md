# DriveStudio integration status

DriveStudio is pinned as a submodule so its preprocessing and checkpoint
formats can be inspected without copying upstream code into `src/`. Static
scene training is intentionally not part of the current vertical slice.

The future adapter must:

1. preprocess one nuScenes scene with images, LiDAR, calibration, objects, and
   fine dynamic masks;
2. exclude dynamic vehicles from the background representation;
3. preserve the metric nuScenes camera poses and record any scene
   normalization;
4. expose the trained checkpoint through `StaticSceneRenderer`; and
5. composite external dynamic-car assets without changing their authoritative
   world transforms.

`DriveStudioConfig.map_from_nuscenes_global` remains unset until a known
nuScenes camera pose has been rendered and the map registration has been
validated. No DriveStudio training dependencies are activated in this MVP.
