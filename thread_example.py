# -*- coding: utf-8 -*-
"""
Created on Thu Feb  5 15:38:41 2026

@author: frank
"""

import threading
import time
import queue

# Thread-safe queue for communication
data_queue = queue.Queue()

# Event to stop the real-time thread
stop_event = threading.Event()

def realtime_task():
    interval = 0.1  # 100 ms
    next_time = time.perf_counter()

    while not stop_event.is_set():
        # Real-time logic
        try:
            data = data_queue.get_nowait()
            print(f"[RT] Processing:", data)
        except queue.Empty:
            pass

        # Do time-critical work here
        print("[RT] Tick")

        # Precise timing control
        next_time += interval
        sleep_time = next_time - time.perf_counter()
        if sleep_time > 0:
            time.sleep(sleep_time)

    print("[RT] Stopped")

# Start real-time thread
rt_thread = threading.Thread(target=realtime_task, daemon=True)
rt_thread.start()

# Main thread logic
for i in range(5):
    data_queue.put(i)
    time.sleep(0.3)

# Stop everything cleanly
stop_event.set()
rt_thread.join()
print("Main thread exiting")
