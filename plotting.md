python3 plot_latency.py --csv localhost_times.csv --column observed_send_to_recv_ns --out send_to_recv.png
python3 plot_latency.py --csv localhost_times.csv --column observed_capture_to_send_ns --window 50 --out capture_to_send.png
python3 plot_latency.py --csv localhost_times.csv --column observed_send_to_display_ns --out cross_host.png
python3 plot_latency.py --csv localhost_times.csv --column observed_recv_to_display_ns --out recv_to_display.png