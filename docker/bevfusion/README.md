# BEVFusion sidecar runtime

Build the small transport layer on top of the existing detector image:

```sh
docker build -t remote-teleop-bevfusion:latest docker/bevfusion
```

Run this directory, and only this directory, into `/workspace` in the derived image:

```sh
docker run --rm --gpus 'device=1' --network host --ipc host \
  --mount type=bind,src="$PWD/docker/bevfusion",dst=/workspace,readonly \
  remote-teleop-bevfusion:latest python /workspace/bevfusion_server.py \
  --config /workspace/nuscenes_front.py --metrics-path /dev/stdout
```

Place the converted checkpoint at `checkpoints/bevfusion_converted.pth`. It is
intentionally ignored by Git. The current runtime intentionally adapts the
six-camera nuScenes checkpoint to one front camera; treat its predictions as
experimental until the MCAP acceptance run is complete.
