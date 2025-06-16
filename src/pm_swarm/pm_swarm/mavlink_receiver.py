# Drone 2
#!/usr/bin/env python3 
import time 
from pymavlink import mavutil 
 
conn = mavutil.mavlink_connection('udp:0.0.0.0:14551') 
 
while True: 
    m = conn.recv_match(blocking=True) 
    if m and m.get_type() == 'STATUSTEXT': 
        sid = m.get_srcSystem() 
        if sid == 10: 
            print("Message received from GCS:", m.text) 
        elif sid == 1: 
            print("Message received from Drone 1:", m.text) 
        elif sid == 2: 
            print("Message received from Drone 2:", m.text) 
        else: 
            print(f"Message received from System {sid}:", m.text) 
    time.sleep(0.1)  # Adjust sleep time as needed
