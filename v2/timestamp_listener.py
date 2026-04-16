# udp server that listens for timestamp messages and prints them to the console

import socket
import collections
import time

port = 22102


frame_dict = {}





ros_to_gst_deltas = collections.deque(maxlen=100)  # store the last 100 deltas to calculate a moving average
hostgst_to_clientgst_deltas = collections.deque(maxlen=100)  # store the last 100 deltas to calculate a moving average
clientgst_to_display_deltas = collections.deque(maxlen=100)  # store the last 100 deltas to calculate a moving average

def run_server(sock):
    total_bytes_received = 0
    first_message_time = None
    msg_count = 0

    recv_display_perf_counter_start = None
    capture_send_perf_counter_start = None
    while True:
        data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
        msg = data.decode()
        msg_type, rest = msg.split(":")
        msg_count += 1

        if msg_type == "CAPTURE":
            # timestamp_msg = f"CAPTURE:frame={self.frame_count},pts={msg.header.stamp.sec}.{msg.header.stamp.nanosec},localtime={time.time()}"
            frame_id, pts, localtime = [x.split("=")[1] for x in rest.split(",")] # frame=1,pts=123456,localtime=1690000000.123
            frame_dict[frame_id] = (pts, 0, localtime, 0, "CAPTURE") # store the capture timestamp for this frame_id
            if capture_send_perf_counter_start is None:
                capture_send_perf_counter_start = float(localtime)
            
            # print(f"Received CAPTURE message: {rest}")
        elif msg_type == "SEND":
            # remove the name= stuff and split by comma
            frame_id, pts, duration, localtime, length = [x.split("=")[1] for x in rest.split(",")] # frame=1,pts=123456,duration=33333,localtime=1690000000.123,len=123456
            # print("send: ", (frame_id, pts, duration, localtime, length))
            if frame_id in frame_dict:
                # print(f"Received SEND message: {rest} with matching CAPTURE message")
                capture_pts, _, capture_localtime, _, _ = frame_dict[frame_id]
                delta = float(localtime) - float(capture_localtime)
                ros_to_gst_deltas.append(delta)

                if msg_count % 100 == 0:
                    avg_delta = sum(ros_to_gst_deltas) / len(ros_to_gst_deltas)
                    print(f"Average time from ROS capture to GStreamer send: {avg_delta/1000000:.3f} milliseconds over {len(ros_to_gst_deltas)} frames")
                if len(ros_to_gst_deltas) > 100:
                    ros_to_gst_deltas.popleft()  # remove the oldest delta to keep the size at 100 
                # print(f"Frame {frame_id}: Captured at {capture_localtime}, Sent at {localtime}, delta is {float(localtime) - float(capture_localtime)} seconds")
            else:
                print(f"Received SEND message for frame {frame_id} but no matching CAPTURE message found (make sure to run reciever before running sender... sorry!)")
                pass
            frame_dict[frame_id] = (pts, duration, localtime, length, "SEND") # store the send timestamp for this frame_id, along with duration and length for debugging
        elif msg_type == "RECV":
            # many RECV datagrams since we have udp messages
            frame_id, pts, duration, localtime, length = [x.split("=")[1] for x in rest.split(",")] # frame=1,pts=123456,duration=33333,localtime=1690000000.123,len=123456
            if recv_display_perf_counter_start is None:
                recv_display_perf_counter_start = float(localtime)
            # print("recv: ", rest)
            if frame_id in frame_dict:
                
                sent_pts, sent_duration, sent_localtime, sent_length, last_msg_type = frame_dict[frame_id]
                # if last_msg_type != "SEND":
                #     continue
                # print(f"Received RECV message for frame {frame_id} with matching SEND message")
                # print(f"Received RECV message: {rest} with matching SEND message")
                frame_dict[frame_id] = (sent_pts, sent_duration, sent_localtime, sent_length, "RECV") # update the last message type to RECV
                # print(f"Frame {frame_id}: Sent at {sent_localtime}, Received at {localtime}, delta is {float(localtime) - float(sent_localtime)} seconds")
                delta = float(localtime) - float(sent_localtime)

                hostgst_to_clientgst_deltas.append(delta)
                total_bytes_received += int(length)
                if first_message_time is None:
                    first_message_time = float(localtime)
                elapsed_time = float(localtime) - first_message_time
                bandwidth = total_bytes_received *1000000000/ elapsed_time if elapsed_time > 0 else 0

                if msg_count % 100 == 0:
                    avg_delta = sum(hostgst_to_clientgst_deltas) / len(hostgst_to_clientgst_deltas)
                    print(f"Average time from GStreamer send to recv receive: {avg_delta/1000000:.3f} milliseconds over {len(hostgst_to_clientgst_deltas)} frames")
                    if len(hostgst_to_clientgst_deltas) > 100:
                        hostgst_to_clientgst_deltas.popleft()  
                    print(f"Total bytes received: {total_bytes_received}, Elapsed time: {elapsed_time/1000000000:.2f} seconds, Average bandwidth: {bandwidth:.2f} bytes/sec")
            else:
                print(f"Received RECV message for frame {frame_id} but no matching SEND message found (make sure to run reciever before running sender... sorry!)")
                pass
        elif msg_type == "DISPLAY":
            frame_id, pts, duration, localtime, length = [x.split("=")[1] for x in rest.split(",")] # frame=1,pts=123456,duration=33333,localtime=1690000000.123,len=123456
            # print("display: ", rest)
            if frame_id in frame_dict:
                # print(f"Received DISPLAY message {rest} with matching SEND message")
                sent_pts, sent_duration, sent_localtime, sent_length, last_msg_type = frame_dict[frame_id]
                del frame_dict[frame_id] # remove frame from dict since we're done
                # print(f"Frame {frame_id}: Sent at {sent_localtime}, Displayed at {localtime}, delta is {float(localtime) - float(sent_localtime)} seconds")
                delta = float(localtime) - float(sent_localtime)
                clientgst_to_display_deltas.append(delta)
                if msg_count % 100 == 0:
                    avg_delta = sum(clientgst_to_display_deltas) / len(clientgst_to_display_deltas)
                    print(f"Average time from GStreamer send to display receive: {avg_delta/1000000:.3f} milliseconds over {len(clientgst_to_display_deltas)} frames")
                if len(clientgst_to_display_deltas) > 100:
                    clientgst_to_display_deltas.popleft()  
            else:
                print(f"Received DISPLAY message for frame {frame_id} but no matching SEND message found (make sure to run reciever before running sender... sorry!)")
                pass
        else:
            print(f"Received unknown message type: {msg_type}")

        if msg_count % 100 == 0:
            print(f"Processed {msg_count} messages so far...", flush=True)


if __name__ == '__main__':
    import signal
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', port))
    print(f"UDP timestamp listener started on 127.0.0.1:{port}")
    def signal_handler(sig, frame):
        print("Interrupt received, shutting down...")
        sock.close()
        exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    
    print("Timestamp listener is running. Press Ctrl+C to stop.")
    run_server(sock)