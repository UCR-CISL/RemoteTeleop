# udp server that listens for timestamp messages and prints them to the console

import socket
import collections
import csv
import time




class TimestampListener:
    def __init__(self, host='0.0.0.0', port=22102, csv_path='localhost_times.csv'):
        self.host = host
        self.port = port
        self.csv_path = csv_path

        # last data
        self.last_capture = None
        self.last_send = None
        self.last_recv = None
        self.last_display = None

        # runtime counters / accumulators
        self.total_bytes_received = 0
        self.first_message_time = None
        self.msg_count = 0

        # perf counters
        self.recv_display_perf_counter_start = None
        self.capture_send_perf_counter_start = None

        self.entries = {
            'capture': collections.deque(),
            'send': collections.deque(),
            'recv': collections.deque(),
            'display': collections.deque(),
        }

        self.entry_counts = {
            'capture': 0,
            'send': 0,
            'recv': 0,
            'display': 0,
        }

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        if self.csv_file.tell() == 0:
            self.csv_writer.writerow(['count', 'capture_time', 'send_time', 'recv_time', 'display_time'])
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


    def add_entry(self, entry, type) :
        if type not in self.entries:
            print(f"Unknown entry type: {type}")
            return
        self.entries[type].append(entry)
        self.entry_counts[type] += 1

        if self.get_entry_count() % 100 == 0:
            # flush all to csv
            while self.get_entry_count() > 0:
                capture_entry = self.entries['capture'].popleft()
                send_entry = self.entries['send'].popleft()
                recv_entry = self.entries['recv'].popleft()
                display_entry = self.entries['display'].popleft()
                self.csv_writer.writerow([
                    self.msg_count,
                    capture_entry[2],  # localtime
                    send_entry[3],     # localtime
                    recv_entry[3],     # localtime
                    display_entry[3],  # localtime
                ])
            self.csv_file.flush()

    def get_entry_count(self):
        # return min count of all entry types
        return min(self.entry_counts.values())

    def handle_capture_message(self, rest):
        fields = self._parse_fields(rest)
        frame_id = fields.get('frame', '')
        pts = fields.get('pts', '')
        localtime = float(fields.get('localtime', ''))

        if self.capture_send_perf_counter_start is None:
            self.capture_send_perf_counter_start = localtime
        print(f"Received CAPTURE message: {rest}")
        last_entry = self.last_capture
        if last_entry is not None:
            last_frame_id, last_pts, last_localtime = last_entry
            print(f"Last CAPTURE message was for frame {last_frame_id} / pts {last_pts} at localtime {last_localtime}")
            delta = localtime - last_localtime

        self.last_capture = (frame_id, pts, localtime)


    def handle_send_message(self, rest):
        fields = self._parse_fields(rest)
        frame_id = fields.get('frame', '')
        pts = fields.get('pts', '')
        duration = fields.get('duration', '')
        localtime = float(fields.get('localtime', ''))
        length = fields.get('len', '')
        print(f"Received SEND message: {rest}")
        last_entry = self.last_send
        if last_entry is not None:
            last_frame_id, last_pts, last_duration, last_localtime, last_length = last_entry
            print(f"Last SEND message was for frame {last_frame_id} / pts {last_pts} / duration {last_duration} / len {last_length} at localtime {last_localtime}")
            delta = localtime - last_localtime
        self.last_send = (frame_id, pts, duration, localtime, length)

    def handle_recv_message(self, rest):
        fields = self._parse_fields(rest)
        frame_id = fields.get('frame', '')
        pts = fields.get('pts', '')
        duration = fields.get('duration', '')
        localtime = float(fields.get('localtime', ''))
        length = fields.get('len', '')
        print(f"Received RECV message: {rest}")
        last_entry = self.last_recv
        if last_entry is not None:
            last_frame_id, last_pts, last_duration, last_localtime, last_length = last_entry
            print(f"Last RECV message was for frame {last_frame_id} / pts {last_pts} / duration {last_duration} / len {last_length} at localtime {last_localtime}")
            delta = localtime - last_localtime
        self.last_recv = (frame_id, pts, duration, localtime, length)
    
    def handle_display_message(self, rest):
        fields = self._parse_fields(rest)
        frame_id = fields.get('frame', '')
        pts = fields.get('pts', '')
        duration = fields.get('duration', '')
        localtime = float(fields.get('localtime', ''))
        length = fields.get('len', '')
        print(f"Received DISPLAY message: {rest}")
        last_entry = self.last_display
        if last_entry is not None:
            last_frame_id, last_pts, last_duration, last_localtime, last_length = last_entry
            print(f"Last DISPLAY message was for frame {last_frame_id} / pts {last_pts} / duration {last_duration} / len {last_length} at localtime {last_localtime}")
            delta = localtime - last_localtime
        self.last_display = (frame_id, pts, duration, localtime, length)




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