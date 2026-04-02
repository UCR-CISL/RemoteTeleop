import os
os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'video_codec;h264_nvenc|preset;fast'

import pyzed.sl as sl
import time

def main():
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.sdk_gpu_id = 0
    init_params.enable_image_enhancement = True
    init_params.sdk_gpu_id = 0
    # init_params.camera_buffer_count = 4
    init_params.grab_compute_capping_fps = 0
    init_params.depth_mode = sl.DEPTH_MODE.NONE
    
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open ZED")
        return
    
    stream = sl.StreamingParameters()
    stream.codec = sl.STREAMING_CODEC.H265
    stream.port = 30000
    stream.bitrate = 4000
    
    status = zed.enable_streaming(stream)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Streaming failed: {status}")
        return
    
    print("Streaming started on port 30000...")
    
    image = sl.Mat()
    try:
        while True:
            if zed.grab() == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image, sl.VIEW.LEFT, sl.MEM.GPU)
    except KeyboardInterrupt:
        zed.disable_streaming()
        zed.close()

if __name__ == "__main__":
    main()