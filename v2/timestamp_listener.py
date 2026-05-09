# udp server that listens for timestamp messages and prints them to the console

import socket
import collections
import csv
import time


class TimestampListener:
    def __init__(self, host='127.0.0.1', port=22102, csv_path='localhost_times.csv'):
        self.host = host
        self.port = port
        self.csv_path = csv_path

        # shared state
        self.capture_by_frame_id = {}
        self.stream_state_by_frame_id = {}
        self.recv_localtime_by_frame_id = {}
        self.frame_times_by_frame_id = {}

        # moving average deques
        self.ros_to_gst_deltas = collections.deque(maxlen=100)
        self.hostgst_to_clientgst_deltas = collections.deque(maxlen=100)
        self.clientgst_to_display_deltas = collections.deque(maxlen=100)

        # runtime counters / accumulators
        self.total_bytes_received = 0
        self.first_message_time = None
        self.msg_count = 0

        # perf counters
        self.recv_display_perf_counter_start = None
        self.capture_send_perf_counter_start = None

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        if self.csv_file.tell() == 0:
            self.csv_writer.writerow([
                'frame_id',
                'pts',
                'duration',
                'length',
                'capture_localtime_ns',
                'send_localtime_ns',
                'recv_localtime_ns',
                'display_localtime_ns',
                'capture_arrival_ns',
                'send_arrival_ns',
                'recv_arrival_ns',
                'display_arrival_ns',
                'capture_to_send_ns',
                'send_to_recv_ns',
                'recv_to_display_ns',
                'send_to_display_cross_host_ns',
                'observed_capture_to_send_ns',
                'observed_send_to_recv_ns',
                'observed_recv_to_display_ns',
                'observed_send_to_display_ns',
            ])
            self.csv_file.flush()

    def close(self):
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()

    def run_server(self, sock):
        while True:
            data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
            received_ns = time.perf_counter_ns()
            msg = data.decode()
            try:
                msg_type, rest = msg.split(":", 1)
            except ValueError:
                print("Malformed message received:", msg)
                continue
            self.msg_count += 1

            if msg_type == "CAPTURE":
                self.handle_capture_message(rest, received_ns)
            elif msg_type == "SEND":
                self.handle_send_message(rest, received_ns)
            elif msg_type == "RECV":
                self.handle_recv_message(rest, received_ns)
            elif msg_type == "DISPLAY":
                self.handle_display_message(rest, received_ns)
            else:
                print(f"Received unknown message type: {msg_type}")

            if self.msg_count % 100 == 0:
                print(f"Processed {self.msg_count} messages so far...", flush=True)

    def _parse_fields(self, rest):
        fields = {}
        for item in rest.split(","):
            key, value = item.split("=", 1)
            fields[key] = value
        return fields

    def handle_capture_message(self, rest, received_ns):
        fields = self._parse_fields(rest)
        frame_id = fields.get('frame', '')
        pts = fields.get('pts', '')
        localtime = fields.get('localtime', '')
        self.capture_by_frame_id[frame_id] = {
            'capture': float(localtime),
            'capture_arrival': received_ns,
            'pts': pts,
            'duration': '',
            'length': '',
        }
        if self.capture_send_perf_counter_start is None:
            self.capture_send_perf_counter_start = float(localtime)
        print(f"Received CAPTURE message: {rest}")

    def handle_send_message(self, rest, received_ns):
        fields = self._parse_fields(rest)
        frame_id = fields.get('frame', '')
        pts = fields.get('pts', '')
        duration = fields.get('duration', '')
        localtime = fields.get('localtime', '')
        length = fields.get('len', '')
        send_localtime = float(localtime)
        send_arrival_ns = received_ns
        print("send: ", (frame_id, pts, duration, localtime, length))

        frame_entry = self.frame_times_by_frame_id.setdefault(frame_id, {})
        if 'send' not in frame_entry:
            frame_entry['send'] = send_localtime
        frame_entry['send_arrival'] = send_arrival_ns
        frame_entry['pts'] = pts
        frame_entry['duration'] = duration
        frame_entry['length'] = length
        frame_entry['frame_id'] = frame_id

        if frame_id in self.capture_by_frame_id:
            print(f"Received SEND message: {rest} with matching CAPTURE message")
            capture_localtime = self.capture_by_frame_id[frame_id]['capture']
            capture_arrival_ns = self.capture_by_frame_id[frame_id]['capture_arrival']
            frame_entry['capture'] = capture_localtime
            frame_entry['capture_arrival'] = capture_arrival_ns
            delta = send_localtime - capture_localtime
            self.ros_to_gst_deltas.append(delta)

            if self.msg_count % 100 == 0 and len(self.ros_to_gst_deltas) > 0:
                avg_delta = sum(self.ros_to_gst_deltas) / len(self.ros_to_gst_deltas)
                print(f"Average time from ROS capture to GStreamer send: {avg_delta/1000000:.3f} milliseconds over {len(self.ros_to_gst_deltas)} frames")
            print(f"Frame {frame_id}: Captured at {capture_localtime}, Sent at {localtime}, delta is {(send_localtime - capture_localtime)/1000000:.3f} milliseconds")
        else:
            print(f"Received SEND message for frame {frame_id} but no matching CAPTURE message found (make sure to run reciever before running sender... sorry!)")

        capture_arrival_ns = self.capture_by_frame_id.get(frame_id, {}).get('capture_arrival')
        if capture_arrival_ns is not None:
            frame_entry['observed_capture_to_send'] = send_arrival_ns - capture_arrival_ns

        prev_state = self.stream_state_by_frame_id.get(frame_id)
        if prev_state is None:
            self.stream_state_by_frame_id[frame_id] = (frame_id, pts, duration, localtime, length, "SEND")

    def handle_recv_message(self, rest, received_ns):
        fields = self._parse_fields(rest)
        frame_id = fields.get('frame', '')
        pts = fields.get('pts', '')
        duration = fields.get('duration', '')
        localtime = fields.get('localtime', '')
        length = fields.get('len', '')
        recv_localtime = float(localtime)
        recv_arrival_ns = received_ns
        frame_entry = self.frame_times_by_frame_id.setdefault(frame_id, {})
        if 'recv' not in frame_entry:
            frame_entry['recv'] = recv_localtime
        frame_entry['recv_arrival'] = recv_arrival_ns
        frame_entry['pts'] = pts
        frame_entry['duration'] = duration
        frame_entry['length'] = length
        frame_entry['frame_id'] = frame_id
        self.recv_localtime_by_frame_id.setdefault(frame_id, recv_localtime)

        if self.recv_display_perf_counter_start is None:
            self.recv_display_perf_counter_start = recv_localtime
        print("recv: ", rest)
        if frame_id in self.stream_state_by_frame_id:
            sent_frame_id, sent_pts, sent_duration, sent_localtime, sent_length, last_msg_type = self.stream_state_by_frame_id[frame_id]
            if last_msg_type == "RECV":
                # RECV can be packet-level; keep the first RECV for this frame PTS.
                return
            if last_msg_type != "SEND":
                return
            print(f"Received RECV message for frame {frame_id} with matching SEND message")
            self.stream_state_by_frame_id[frame_id] = (sent_frame_id, sent_pts, sent_duration, sent_localtime, sent_length, "RECV")
            print(f"Frame {frame_id}: Sent at {sent_localtime}, Received at {localtime}, delta is {(float(localtime) - float(sent_localtime))/1000000:.3f} milliseconds")
            delta = float(localtime) - float(sent_localtime)
            self.recv_localtime_by_frame_id[frame_id] = recv_localtime
            frame_entry['observed_send_to_recv'] = recv_arrival_ns - frame_entry.get('send_arrival', recv_arrival_ns)

            self.hostgst_to_clientgst_deltas.append(delta)
            try:
                self.total_bytes_received += int(length)
            except Exception:
                pass
            if self.first_message_time is None:
                self.first_message_time = float(localtime)
            elapsed_time = float(localtime) - self.first_message_time
            bandwidth = self.total_bytes_received * 1000000000 / elapsed_time if elapsed_time > 0 else 0

            if self.msg_count % 100 == 0 and len(self.hostgst_to_clientgst_deltas) > 0:
                avg_delta = sum(self.hostgst_to_clientgst_deltas) / len(self.hostgst_to_clientgst_deltas)
                print(f"Average time from GStreamer send to recv receive: {avg_delta/1000000:.3f} milliseconds over {len(self.hostgst_to_clientgst_deltas)} frames")
                print(f"Total bytes received: {self.total_bytes_received}, Elapsed time: {elapsed_time/1000000000:.2f} seconds, Average bandwidth: {bandwidth:.2f} bytes/sec")
        else:
            print(f"Received RECV message for frame {frame_id} / pts {pts} but no matching SEND message found (make sure to run reciever before running sender... sorry!)")

    def handle_display_message(self, rest, received_ns):
        fields = self._parse_fields(rest)
        frame_id = fields.get('frame', '')
        pts = fields.get('pts', '')
        duration = fields.get('duration', '')
        localtime = fields.get('localtime', '')
        length = fields.get('len', '')
        display_localtime = float(localtime)
        display_arrival_ns = received_ns
        frame_entry = self.frame_times_by_frame_id.setdefault(frame_id, {})
        frame_entry['display'] = display_localtime
        frame_entry['display_arrival'] = display_arrival_ns
        frame_entry['pts'] = pts
        frame_entry['duration'] = duration
        frame_entry['length'] = length
        frame_entry['frame_id'] = frame_id

        print("display: ", rest)
        state = self.stream_state_by_frame_id.pop(frame_id, None)
        if state is None:
            print(f"Received DISPLAY message for frame {frame_id} / pts {pts} without matching SEND; writing partial CSV row")
        else:
            print(f"Received DISPLAY message {rest} with matching SEND message")

        recv_localtime = self.recv_localtime_by_frame_id.pop(frame_id, frame_entry.get('recv'))
        send_localtime = frame_entry.get('send')
        capture_arrival_ns = frame_entry.get('capture_arrival')
        send_arrival_ns = frame_entry.get('send_arrival')
        recv_arrival_ns = frame_entry.get('recv_arrival')
        display_arrival_ns = frame_entry.get('display_arrival')
        delta = None
        if recv_localtime is not None:
            delta = display_localtime - recv_localtime
            print(f"Frame {frame_id}: Received at {recv_localtime}, Displayed at {localtime}, delta is {delta/1000000:.3f} milliseconds")
        elif send_localtime is not None:
            delta = display_localtime - send_localtime
            print(f"Frame {frame_id}: Sent at {send_localtime}, Displayed at {localtime}, delta is {delta/1000000:.3f} milliseconds (cross-host clock)")

        if delta is not None:
            self.clientgst_to_display_deltas.append(delta)
            if self.msg_count % 100 == 0 and len(self.clientgst_to_display_deltas) > 0:
                avg_delta = sum(self.clientgst_to_display_deltas) / len(self.clientgst_to_display_deltas)
                print(f"Average time from GStreamer recv to display: {avg_delta/1000000:.3f} milliseconds over {len(self.clientgst_to_display_deltas)} frames")

        capture_t = frame_entry.get('capture')
        send_t = frame_entry.get('send')
        recv_t = recv_localtime if recv_localtime is not None else frame_entry.get('recv')
        display_t = frame_entry.get('display')

        capture_to_send = (send_t - capture_t) if capture_t is not None and send_t is not None else ''
        send_to_recv = (recv_t - send_t) if recv_t is not None and send_t is not None else ''
        recv_to_display = (display_t - recv_t) if display_t is not None and recv_t is not None else ''
        send_to_display_cross_host = (display_t - send_t) if display_t is not None and send_t is not None else ''

        observed_capture_to_send = (send_arrival_ns - capture_arrival_ns) if capture_arrival_ns is not None and send_arrival_ns is not None else ''
        observed_send_to_recv = (recv_arrival_ns - send_arrival_ns) if recv_arrival_ns is not None and send_arrival_ns is not None else ''
        observed_recv_to_display = (display_arrival_ns - recv_arrival_ns) if display_arrival_ns is not None and recv_arrival_ns is not None else ''
        observed_send_to_display = (display_arrival_ns - send_arrival_ns) if display_arrival_ns is not None and send_arrival_ns is not None else ''

        self.csv_writer.writerow([
            frame_entry.get('frame_id', frame_id),
            frame_entry.get('pts', ''),
            frame_entry.get('duration', ''),
            frame_entry.get('length', ''),
            capture_t if capture_t is not None else '',
            send_t if send_t is not None else '',
            recv_t if recv_t is not None else '',
            display_t if display_t is not None else '',
            capture_arrival_ns if capture_arrival_ns is not None else '',
            send_arrival_ns if send_arrival_ns is not None else '',
            recv_arrival_ns if recv_arrival_ns is not None else '',
            display_arrival_ns if display_arrival_ns is not None else '',
            capture_to_send,
            send_to_recv,
            recv_to_display,
            send_to_display_cross_host,
            observed_capture_to_send,
            observed_send_to_recv,
            observed_recv_to_display,
            observed_send_to_display,
        ])
        self.csv_file.flush()
        self.frame_times_by_frame_id.pop(frame_id, None)


if __name__ == '__main__':
    import signal
    listener = TimestampListener()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((listener.host, listener.port))
    print(f"UDP timestamp listener started on {listener.host}:{listener.port}")

    def signal_handler(sig, frame):
        print("Interrupt received, shutting down...")
        listener.close()
        sock.close()
        exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    print("Timestamp listener is running. Press Ctrl+C to stop.")
    listener.run_server(sock)