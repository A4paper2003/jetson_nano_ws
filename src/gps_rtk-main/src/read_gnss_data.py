
#!/usr/bin/env python3

import rospy
import serial
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from ublox_msgs.msg import NavPVT  # Import the NavPVT message type
from pyubx2 import UBXMessage, UBXReader, SET, RTCM3_PROTOCOL, UBX_PROTOCOL

# Fetch parameters from the ROS parameter server
GNSS_PORT = rospy.get_param('gnss/port', '/dev/tty_gps_module')  # Default to '/dev/tty_gps_module'
BAUDRATE = rospy.get_param('gnss/baudrate', 38400)  # Default to 115200
TOPIC = rospy.get_param('gnss/topic', '/gps/fix')  # Default to '/gps/fix'
HEADING_TOPIC = rospy.get_param('gnss/heading_topic', '/gps/heading')  # Topic for heading
HEADING_TOPIC_F9H = '/gps/heading_f9h'
DEBUG = rospy.get_param('gnss/debug', False)  # Default to False
F9H_PORT = rospy.get_param('gnss/f9h_port', '/dev/tty_f9h')  # F9H rover port
F9H_BAUDRATE = rospy.get_param('gnss/f9h_baudrate', 38400)  # Default to 38400

# Function to configure the F9P GNSS receiver (base)
def configure_gnss(ser):
    # Enable RTCM3 messages for GNSS corrections
    rtcm_messages = [
        (0x05, "RTCM 1005 (Stationary RTK reference station ARP)"),
        (0x4D, "RTCM 1077 (GPS MSM7 - Full GPS constellation data)"),
        (0x57, "RTCM 1087 (GLONASS MSM7 - Full GLONASS constellation data)"),
        (0x61, "RTCM 1097 (Galileo MSM7 - Full Galileo constellation data)"),
        (0x7F, "RTCM 1127 (BeiDou MSM7 - Full BeiDou constellation data)"),
        (0xE6, "RTCM 1230 (GLONASS code-phase biases)"),
        (0xFE, "RTCM 4072.0 (u-blox proprietary Reference station PVT)"),
    ]

    print("Configuring F9P as moving base station (generating RTCM3):")
    for msg_id, msg_desc in rtcm_messages:
        # Configure via legacy CFG-MSG approach
        msg_rtcm = UBXMessage("CFG", "CFG-MSG", SET, 
                            msgClass=0xF5, msgID=msg_id, 
                            rateDDC=0, rateUART1=1, rateUSB=1, 
                            rateUART2=0, rateSPI=0)
        ser.write(msg_rtcm.serialize())
        if DEBUG:
            print(f"Sent {msg_desc}: {msg_rtcm}")
    
    # Configure for UBX+RTCM protocol on UART1
    msg_prt = UBXMessage("CFG", "CFG-PRT", SET, 
                        portID=1, mode=0x08D0, baudRate=BAUDRATE, 
                        inProtoMask=0x07, outProtoMask=0x07)  # 0x07 = UBX+NMEA+RTCM3
    ser.write(msg_prt.serialize())
    print("F9P port configured for UBX + RTCM3 input/output.")
    
    # For Gen 9+ receivers, also use configuration database approach
    try:
        from pyubx2 import SET_LAYER_RAM, TXN_NONE
        
        # Map of RTCM message types to config database keys
        config_data = [
            ("CFG_MSGOUT_RTCM_3X_TYPE1005_UART1", 1),
            ("CFG_MSGOUT_RTCM_3X_TYPE1077_UART1", 1),
            ("CFG_MSGOUT_RTCM_3X_TYPE1087_UART1", 1),
            ("CFG_MSGOUT_RTCM_3X_TYPE1097_UART1", 1),
            ("CFG_MSGOUT_RTCM_3X_TYPE1127_UART1", 1),
            ("CFG_MSGOUT_RTCM_3X_TYPE1230_UART1", 1),
            ("CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART1", 1),
            
            # Also enable NAV-RELPOSNED for heading from F9P (preferred source)
            ("CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1", 1),
            ("CFG_MSGOUT_UBX_NAV_RELPOSNED_USB", 1)
        ]
        
        config_msg = UBXMessage.config_set(SET_LAYER_RAM, TXN_NONE, config_data)
        ser.write(config_msg.serialize())
        print("Configuration also sent via CFG-VALSET for Gen 9+ receivers")
    except Exception as e:
        print(f"Note: CFG-VALSET configuration skipped: {e}")
    
    # Configure NAV-PVT message output
    msg_nav_pvt = UBXMessage("CFG", "CFG-MSG", SET, 
                           msgClass=0x01, msgID=0x07, 
                           rateDDC=0, rateUART1=1, rateUSB=1, 
                           rateUART2=0, rateSPI=0)
    ser.write(msg_nav_pvt.serialize())
    print("NAV-PVT message configured for position and RTK status")
    
    # Configure NAV-RELPOSNED message for heading (legacy method)
    msg_nav_relposned = UBXMessage("CFG", "CFG-MSG", SET, 
                                 msgClass=0x01, msgID=0x3C, 
                                 rateDDC=0, rateUART1=1, rateUSB=1, 
                                 rateUART2=0, rateSPI=0)
    ser.write(msg_nav_relposned.serialize())
    print("NAV-RELPOSNED message configured for heading information")
    # Configure message rate for NAV-RELPOSNED (20Hz)
    # msg_rate = UBXMessage("CFG", "CFG-RATE", SET, 
    #                     measRate=50,   # Measurement rate in ms (50ms = 20Hz)
    #                     navRate=1,     # Navigation rate (cycles per measurement)
    #                     timeRef=1)     # Alignment to UTC time
    # ser.write(msg_rate.serialize())
    # print("Message rate configured to 20Hz")
    
    print("F9P base configuration complete!")

# Function to configure the F9H as a rover
def configure_f9h_rover(ser):
    print("Configuring F9H as rover (receiving RTCM3):")
    
    # Configure for UBX+RTCM protocol on UART1
    msg_prt = UBXMessage("CFG", "CFG-PRT", SET, 
                        portID=1, mode=0x08D0, baudRate=BAUDRATE, 
                        inProtoMask=0x07, outProtoMask=0x03)  # inProtoMask=0x07 for RTCM3 input
    ser.write(msg_prt.serialize())
    print("F9H port configured to accept RTCM3 input.")
    
    # For Gen 9+ receivers, also use configuration database approach
    try:
        from pyubx2 import SET_LAYER_RAM, TXN_NONE
        
        # Configure rover settings
        config_data = [
            # Set up rover mode for RTK
            ("CFG_NAVSPG_DYNMODEL", 3),       # 3=Pedestrian, 4=Automotive
            ("CFG_NAVHPG_DGNSSMODE", 3),      # 3=RTK fixed, 2=RTK float
            ("CFG_NAVSPG_UTCSTANDARD", 0),    # 0=Auto detect
            
            # Enable RTCM input processing
            ("CFG_UART1_INPROT_RTCM3X", 1),   # Enable RTCM3 input on UART1
            ("CFG_USB_INPROT_RTCM3X", 1),     # Enable RTCM3 input on USB
            
            # Enable NAV-PVT output
            ("CFG_MSGOUT_UBX_NAV_PVT_UART1", 1),
            
            # Enable NAV-RELPOSNED output for heading information over both interfaces
            ("CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1", 1),
            ("CFG_MSGOUT_UBX_NAV_RELPOSNED_USB", 1)
        ]
        
        config_msg = UBXMessage.config_set(SET_LAYER_RAM, TXN_NONE, config_data)
        ser.write(config_msg.serialize())
        print("Configuration sent via CFG-VALSET")
    except Exception as e:
        print(f"Note: CFG-VALSET configuration skipped: {e}")
    
    # Explicitly configure NAV-PVT and NAV-RELPOSNED messages with legacy method as fallback
    msg_nav_pvt = UBXMessage("CFG", "CFG-MSG", SET, 
                           msgClass=0x01, msgID=0x07, 
                           rateDDC=0, rateUART1=1, rateUSB=1, 
                           rateUART2=0, rateSPI=0)
    ser.write(msg_nav_pvt.serialize())
    
    msg_nav_relposned = UBXMessage("CFG", "CFG-MSG", SET, 
                                 msgClass=0x01, msgID=0x3C, 
                                 rateDDC=0, rateUART1=1, rateUSB=1, 
                                 rateUART2=0, rateSPI=0)
    ser.write(msg_nav_relposned.serialize())
    
    print("NAV-PVT and NAV-RELPOSNED messages configured for F9H")
    # Configure message rate for NAV-RELPOSNED (20Hz)
    # msg_rate = UBXMessage("CFG", "CFG-RATE", SET, 
    #                     measRate=50,   # Measurement rate in ms (50ms = 20Hz)
    #                     navRate=1,     # Navigation rate (cycles per measurement)
    #                     timeRef=1)     # Alignment to UTC time
    # ser.write(msg_rate.serialize())
    print("Message rate configured to 20Hz")
    print("F9H rover configuration complete!")

# Function to calculate the covariance matrix from horizontal and vertical accuracies
def calculate_covariance(hAcc, vAcc):
    """
    Calculate the covariance matrix from horizontal and vertical accuracies.
    
    Args:
    hAcc (float): Horizontal accuracy in mm.
    vAcc (float): Vertical accuracy in mm.
    
    Returns:
    list: Covariance matrix (3x3) as a flat list.
    """
    # Convert accuracies from mm to meters
    hAcc_m = hAcc / 1000.0  # Convert from mm to meters
    vAcc_m = vAcc / 1000.0  # Convert from mm to meters
    
    # Calculate variances (square of the accuracy)
    varH = hAcc_m ** 2
    varV = vAcc_m ** 2
    
    # Construct the covariance matrix (diagonal, no correlation between latitude, longitude, and altitude)
    covariance_matrix = [varH, 0, 0, 0, varH, 0, 0, 0, varV]
    
    return covariance_matrix

# Function to get cardinal direction from heading
def get_cardinal_direction(heading_deg):
    """
    Convert heading in degrees to cardinal direction.
    
    Args:
    heading_deg (float): Heading in degrees (0-359.99)
    
    Returns:
    str: Cardinal direction (N, NE, E, etc.)
    """
    cardinal_dirs = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", 
                    "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW", "N"]
    idx = round(heading_deg / 22.5)
    return cardinal_dirs[idx]

# Function to read GNSS data from both receivers, forward RTCM3, and publish NavSatFix and heading
def read_gnss_data(ser_f9p, ser_f9h, pub_fix, pub_heading, pub_heading_f9h,pub_heading_f9p_text, pub_heading_f9h_text):
    # Create readers for both receivers
    ubr_f9p = UBXReader(ser_f9p, protfilter=UBX_PROTOCOL | RTCM3_PROTOCOL)  # Accept both UBX and RTCM3
    ubr_f9h = UBXReader(ser_f9h, protfilter=UBX_PROTOCOL)  # Only care about UBX from F9H
    
    # Counter for RTCM messages (for debugging)
    rtcm_counter = 0
    
    # Store F9H's RTK status
    f9h_rtk_status = "Unknown"
    f9h_num_sv = 0
    
    while not rospy.is_shutdown():
        try:
            # Check for data from F9P (base)
            if ser_f9p.in_waiting > 0:
                raw_data, parsed_data = ubr_f9p.read()
                
                # If we received an RTCM3 message, forward it to F9H
                if raw_data and raw_data[0] == 0xD3:  # RTCM3 messages start with 0xD3
                    ser_f9h.write(raw_data)
                    rtcm_counter += 1
                    if DEBUG and rtcm_counter % 100 == 0:  # Only print occasionally to avoid flooding
                        print(f"Forwarded RTCM3 message to F9H (total: {rtcm_counter})")
                
                # # Process NAV-RELPOSNED messages from F9P for heading
                if parsed_data and parsed_data.identity == "NAV-RELPOSNED":
                    # Extract heading information (in degrees)
                    if hasattr(parsed_data, 'relPosHeading'):
                        heading_deg = parsed_data.relPosHeading
                        
                        # Create NavPVT message to store heading
                        msg = NavPVT()
                        msg.heading = int(heading_deg * 1e5)  # Convert to deg * 1e-5 as per NavPVT format
                        
                        # Set the flags to indicate heading is valid
                        msg.flags = msg.FLAGS_HEAD_VEH_VALID
                        
                        msg.iTOW = parsed_data.iTOW
                        

                        msg.headAcc = int(parsed_data.accHeading)
                        
                        if parsed_data.carrSoln == 2:
                            msg.flags |= msg.CARRIER_PHASE_FIXED
                        elif parsed_data.carrSoln == 1:
                            msg.flags |= msg.CARRIER_PHASE_FLOAT
                        # Get cardinal direction for display
                        cardinal = get_cardinal_direction(heading_deg)
                        # Publish the heading message
                        pub_heading.publish(msg)
                        pub_heading_f9p_text.publish(f"Heading: {heading_deg:.1f}° ({cardinal})")
                        
                        # Print simple heading information
                        print(f"{heading_deg:.1f}° ({cardinal})")
                
                # Process NAV-PVT messages for NavSatFix publishing
                if parsed_data and hasattr(parsed_data, 'identity') and parsed_data.identity == "NAV-PVT":
                    # Extract relevant data
                    lat = parsed_data.lat
                    lon = parsed_data.lon
                    alt = parsed_data.height  # Altitude in meters above the WGS84 ellipsoid
                    hacc = parsed_data.hAcc
                    vacc = parsed_data.vAcc
                    num_sv = parsed_data.numSV
                    
                    # Check RTK status based on CARRSOLN (carrier phase solution)
                    carrsoln = parsed_data.carrSoln if hasattr(parsed_data, 'carrSoln') else 0  # 0 = NO RTK, 1 = RTK FLOAT, 2 = RTK FIXED
                    if carrsoln == 0:
                        rtk_status = "No RTK"
                    elif carrsoln == 1:
                        rtk_status = "RTK FLOAT"
                    elif carrsoln == 2:
                        rtk_status = "RTK FIXED"
                    else:
                        rtk_status = "Unknown RTK Status"
                    
                    # Prepare the NavSatFix message
                    msg = NavSatFix()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "gps"
                    
                    # Set the status based on RTK status
                    if carrsoln >= 1:
                        msg.status.status = msg.status.STATUS_FIX  # RTK fixed
                        msg.status.service = msg.status.SERVICE_GPS
                    else:
                        msg.status.status = msg.status.STATUS_NO_FIX  # No valid fix
                        msg.status.service = msg.status.SERVICE_GPS
                    
                    # Set the GNSS data
                    msg.latitude = lat
                    msg.longitude = lon
                    msg.altitude = alt
                    
                    # Calculate and set the covariance matrix
                    msg.position_covariance = calculate_covariance(hacc, vacc)
                    msg.position_covariance_type = msg.COVARIANCE_TYPE_DIAGONAL_KNOWN  # Known diagonal covariance
                    
                    # Publish the NavSatFix message
                    pub_fix.publish(msg)
                    
                    # Print position information
                    if DEBUG:
                        hacc_cm = hacc / 10.0  # Convert horizontal accuracy to cm
                        vacc_cm = vacc / 10.0  # Convert vertical accuracy to cm
                        
                        # Include F9H RTK status in output
                        f9h_info = f", F9H: {f9h_rtk_status} ({f9h_num_sv} SVs)"
                        
                        print(f"Position: Lat={lat:.7f}, Lon={lon:.7f}, Alt={alt:.2f}m, RTK={rtk_status}, SV={num_sv}{f9h_info}")
                        print(f"Accuracy: Horizontal={hacc_cm:.2f}cm, Vertical={vacc_cm:.2f}cm")
            
            # Check for data from F9H (rover) - for heading as fallback
            if ser_f9h.in_waiting > 0:
                raw_data, parsed_data = ubr_f9h.read()
                
                # Process NAV-PVT messages for F9H RTK status
                if parsed_data and hasattr(parsed_data, 'identity') and parsed_data.identity == "NAV-PVT":
                    f9h_num_sv = parsed_data.numSV if hasattr(parsed_data, 'numSV') else 0
                    
                    # Check RTK status based on CARRSOLN (carrier phase solution)
                    if hasattr(parsed_data, 'carrSoln'):
                        carrsoln = parsed_data.carrSoln  # 0 = NO RTK, 1 = RTK FLOAT, 2 = RTK FIXED
                        if carrsoln == 0:
                            f9h_rtk_status = "No RTK"
                        elif carrsoln == 1:
                            f9h_rtk_status = "RTK FLOAT"
                        elif carrsoln == 2:
                            f9h_rtk_status = "RTK FIXED"
                        else:
                            f9h_rtk_status = "Unknown"
                            
                    if DEBUG:
                        print(f"F9H Status: {f9h_rtk_status}, SVs: {f9h_num_sv}")
                
                # Process NAV-RELPOSNED messages for heading information
                if parsed_data and parsed_data.identity == "NAV-RELPOSNED":
                    # Extract heading information (in degrees)
                    if hasattr(parsed_data, 'relPosHeading'):
                        heading_deg = parsed_data.relPosHeading
                        
                        # Create NavPVT message to store heading
                        msg = NavPVT()
                        msg.heading = int(heading_deg * 1e5)  # Convert to deg * 1e-5 as per NavPVT format
                        
                        # Set the flags to indicate heading is valid
                        msg.flags = msg.FLAGS_HEAD_VEH_VALID
                        
                        # Set iTOW if available
                        if hasattr(parsed_data, 'iTOW'):
                            msg.iTOW = parsed_data.iTOW
                        
                        # Set accuracy if available (headAcc in NavPVT is in deg * 1e-5)
                        if hasattr(parsed_data, 'accHeading'):
                            msg.headAcc = int(parsed_data.accHeading)
                        
                        # Set carrier phase status if available
                        if hasattr(parsed_data, 'carrSoln'):
                            if parsed_data.carrSoln == 2:
                                msg.flags |= msg.CARRIER_PHASE_FIXED
                            elif parsed_data.carrSoln == 1:
                                msg.flags |= msg.CARRIER_PHASE_FLOAT
                        
                        # Publish the heading message
                        pub_heading_f9h.publish(msg)
                        
                        # Get cardinal direction for display
                        cardinal = get_cardinal_direction(heading_deg)
                        # Publish the heading message
                        pub_heading_f9h.publish(msg)
                        pub_heading_f9h_text.publish(f"{heading_deg:.1f}° ({cardinal})")
                        # Print simple heading information
                        print(f"Heading: {heading_deg:.1f}° ({cardinal})")
                
        except Exception as e:
            print(f"Error reading/processing GNSS data: {e}")
            pass

# Main function
if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('gnss_publisher', anonymous=True)
    
    # Create publishers for NavSatFix and heading messages
    pub_fix = rospy.Publisher(TOPIC, NavSatFix, queue_size=10)
    pub_heading = rospy.Publisher(HEADING_TOPIC, NavPVT, queue_size=10)  # Changed to NavPVT
    pub_heading_f9h = rospy.Publisher(HEADING_TOPIC_F9H, NavPVT, queue_size=10)  # F9P publisher
    pub_heading_f9p_text = rospy.Publisher('/gps/f9p_text', String, queue_size=10)
    pub_heading_f9h_text = rospy.Publisher('/gps/f9h_text', String, queue_size=10)
    
    try:
        print(f"Opening connection to F9P base at {GNSS_PORT}")
        with serial.Serial(port=GNSS_PORT, baudrate=BAUDRATE, timeout=0.1) as ser_f9p:
            # Configure the F9P base
            configure_gnss(ser_f9p)
            
            print(f"Opening connection to F9H rover at {F9H_PORT}")
            with serial.Serial(port=F9H_PORT, baudrate=F9H_BAUDRATE, timeout=0.1) as ser_f9h:
                # Configure the F9H rover
                configure_f9h_rover(ser_f9h)
                
                # Start reading data from both receivers, forwarding RTCM3, and publishing messages
                print("Reading data from both receivers, forwarding RTCM3, and publishing position and heading...")
                read_gnss_data(ser_f9p, ser_f9h, pub_fix, pub_heading, pub_heading_f9h, pub_heading_f9p_text, pub_heading_f9h_text)
    
    except serial.SerialException as e:
        rospy.logerr(f"Error opening serial port: {e}")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")