In Remote Teleop venv 
``python v2/streaming/image_pipeline/gstreamer_sender.py --stream-host 127.0.0.1 --stream-port 5000``

In system environment (Make sure to conda deactivate!)
``python3 v2/streaming/image_pipeline/ros_image_bridge.py --topic /lucid/image_raw``

In remote teleop venv (conda deactivate too!)
``python3 v2/streaming/gstream_zed_receiver.py --sw``