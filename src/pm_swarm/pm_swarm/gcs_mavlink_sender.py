#!/usr/bin/env python3 
import time 
from pymavlink import mavutil 
 
# Connect locally to mavlink-router, which listens on 127.0.0.1:14550 
conn = mavutil.mavlink_connection( 
    'udpout:127.0.0.1:14550', 
    source_system=10,  # GCS system ID 
    source_component=1 
) 
 
while True: 
    # Send a message labeled for Drone 1 
    conn.mav.statustext_send( 
        severity=mavutil.mavlink.MAV_SEVERITY_NOTICE, 
        text="Message sent to Drone 1".encode('ascii') 
    ) 
    time.sleep(0.5)
 
    # Send a message labeled for Drone 2 
    conn.mav.statustext_send( 
        severity=mavutil.mavlink.MAV_SEVERITY_NOTICE, 
    text="Message sent to Drone 2".encode('ascii') 
    )    
    time.sleep(0.5)
