FROM mmdet3d-bevfusion:latest

# The upstream detector image has MMDetection3D/CUDA but no ZMQ transport.
RUN python -m pip install --no-cache-dir pyzmq==26.4.0
