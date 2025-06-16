#!/usr/bin/env python3 
import time 
from pymavlink import mavutil 

# Drone 1 is system ID = 1 
conn = mavutil.mavlink_connection( 
    'udpout:127.0.0.1:14550', 
    source_system=1, 
    source_component=1 
) 

while True: 
    # Send a message labeled for GCS 
    conn.mav.statustext_send( 
        severity=mavutil.mavlink.MAV_SEVERITY_NOTICE, 
        text="Message sent to GCS".encode('ascii') 
    ) 
    time.sleep(2) 
    # Send a message labeled for Drone 2 
    conn.mav.statustext_send( 
        severity=mavutil.mavlink.MAV_SEVERITY_NOTICE, 
        text="Message sent to Drone 2".encode('ascii') 
    ) 
    time.sleep(0.5) 