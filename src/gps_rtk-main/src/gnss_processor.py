#!/usr/bin/env python3

import rospy
import serial
import threading
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from ublox_msgs.msg import NavPVT
from datetime import datetime
import calendar
from pyubx2 import UBXMessage, UBXReader, SET, RTCM3_PROTOCOL, UBX_PROTOCOL, SET_LAYER_RAM, TXN_NONE
class GNSSDevice:
    def __init__(self, port, baudrate, name):
        self.port = port
        self.baudrate = baudrate
        self.name = name
        self.ser = None
        self.ubr = None
        self.rtcm_counter = 0

    def connect(self):
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=0.1)
            rospy.loginfo(f"Connected to {self.name} at {self.port}")
            return True
        except serial.SerialException as e:
            rospy.logerr(f"Failed to connect to {self.name}: {e}")
            return False

    def configure_base(self):
        """Configure F9P as base station"""
        # RTCM3 message configuration
        rtcm_messages = [
            (0x05, "RTCM 1005 (Stationary RTK reference station ARP)"),
            (0x4D, "RTCM 1077 (GPS MSM7 - Full GPS constellation data)"),
            (0x57, "RTCM 1087 (GLONASS MSM7 - Full GLONASS constellation data)"),
            (0x61, "RTCM 1097 (Galileo MSM7 - Full Galileo constellation data)"),
            (0x7F, "RTCM 1127 (BeiDou MSM7 - Full BeiDou constellation data)"),
            (0xE6, "RTCM 1230 (GLONASS code-phase biases)"),
            (0xFE, "RTCM 4072.0 (u-blox proprietary Reference station PVT)"),
        ]        

        
        for msg_id, msg_desc in rtcm_messages:
            # Configure via legacy CFG-MSG approach
            msg_rtcm = UBXMessage("CFG", "CFG-MSG", SET, 
                                msgClass=0xF5, msgID=msg_id, 
                                rateDDC=0, rateUART1=1, rateUSB=1, 
                                rateUART2=0, rateSPI=0)
            self.ser.write(msg_rtcm.serialize())
            rospy.loginfo(f"[{self.name}]: sent {msg_desc} {msg_rtcm}")

        try:            
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
            self.ser.write(config_msg.serialize())
            rospy.loginfo(f"[{self.name}]: Configuration also sent via CFG-VALSET for Gen 9+ receivers")
        except Exception as e:
            rospy.logwarn(f"[{self.name}]: CFG-VALSET configuration skipped: {e}")

    
        # Configure NAV-PVT message output
        msg_nav_pvt = UBXMessage("CFG", "CFG-MSG", SET, 
                            msgClass=0x01, msgID=0x07, 
                            rateDDC=0, rateUART1=1, rateUSB=1, 
                            rateUART2=0, rateSPI=0)
        self.ser.write(msg_nav_pvt.serialize())
        rospy.loginfo(f"[{self.name}]: NAV-PVT message configured for position and RTK status")

        msg_nav_relposned = UBXMessage("CFG", "CFG-MSG", SET, 
                                 msgClass=0x01, msgID=0x3C, 
                                 rateDDC=0, rateUART1=1, rateUSB=1, 
                                 rateUART2=0, rateSPI=0)
        self.ser.write(msg_nav_relposned.serialize())
        rospy.loginfo(f"[{self.name}]: NAV-RELPOSNED message configured for heading information")
        
        
        self.ubr = UBXReader(self.ser, protfilter=UBX_PROTOCOL | RTCM3_PROTOCOL)
        
        rospy.loginfo(f"[{self.name}]: configure base station finished")




    def read_base(self, rover_ser, publisher):
        if self.ser.in_waiting <= 0:
            # rospy.loginfo(f"[{self.name}]: nothing to read")
            return
        
        raw_data, parsed_data = self.ubr.read()
    
        if raw_data and raw_data[0] == 0xD3:  # RTCM3 messages start with 0xD3
            #rover_ser.write(raw_data)
            self.rtcm_counter += 1
            if self.rtcm_counter % 100 == 0:  # Only print occasionally to avoid flooding
                rospy.loginfo(f"[{self.name}]: Forwarded RTCM3 message to F9H (total: {self.rtcm_counter})")
            
        if parsed_data and parsed_data.identity == "NAV-RELPOSNED":
            pass

        # Process NAV-PVT messages for NavSatFix publishing
        if parsed_data and hasattr(parsed_data, 'identity') and parsed_data.identity == "NAV-PVT":
            # Extract relevant data
            lat = parsed_data.lat
            lon = parsed_data.lon
            alt = parsed_data.height * 1e-3 # ellipsoidal height in MM above the WGS84 ellipsoid
            hacc = parsed_data.hAcc
            vacc = parsed_data.vAcc
            num_sv = parsed_data.numSV

            msg = NavSatFix()
            msg.header.stamp = self.get_message_ros_time(parsed_data)
            msg.header.frame_id = self.name


            # Check RTK status based on CARRSOLN (carrier phase solution)
            # !!! NOTE !!! 
            # we are not using the status as intended!!
            # this is not meant to display rtk status
            # 
            # no rtk    = STATUS_NO_FIX   = -1
            # rtk float = STATUS_FIX      = 0
            # rtk_fixed = STATUS_SBAS_FIX = 1
            # 
            #
            carrsoln = parsed_data.carrSoln if hasattr(parsed_data, 'carrSoln') else 0  # 0 = NO RTK, 1 = RTK FLOAT, 2 = RTK FIXED
            if carrsoln == 0:
                rtk_status = "No RTK"
                msg.status.status = msg.status.STATUS_NO_FIX
            elif carrsoln == 1:
                rtk_status = "RTK FLOAT"
                msg.status.status = msg.status.STATUS_FIX
            elif carrsoln == 2:
                rtk_status = "RTK FIXED"
                msg.status.status = msg.status.STATUS_SBAS_FIX 
            else:
                rtk_status = "Unknown RTK Status"

            msg.status.service = msg.status.SERVICE_GPS

            # Set the GNSS data
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = alt
                        
            # Calculate and set the covariance matrix
            msg.position_covariance = self.calculate_covariance(hacc, vacc)
            msg.position_covariance_type = msg.COVARIANCE_TYPE_DIAGONAL_KNOWN  # Known diagonal covariance
                    
            publisher.publish(msg)

            rospy.loginfo(self.name.center(25,"="))
            rospy.loginfo(f"unixtime: {msg.header.stamp}")
            rospy.loginfo(f"RTK: {rtk_status}")
            rospy.loginfo(f"SVs: {num_sv}")
            rospy.loginfo(f"Lat: {lat:.7f}")
            rospy.loginfo(f"Lon: {lon:.7f}")
            rospy.loginfo(f"alt: {alt:.7f}")
            rospy.loginfo(f"hAcc: {hacc / 10.0} cm")
            rospy.loginfo(f"vAcc: {vacc / 10.0} cm")
            rospy.loginfo("="*(25))


    def read_rover(self,publisher):

        if self.ser.in_waiting <= 0:
            # rospy.loginfo(f"[{self.name}]: nothing to read")
            return
        
        raw_data, parsed_data = self.ubr.read()
        rtk_status = "No RTK"

        # Process NAV-PVT messages for F9H RTK status
        if parsed_data and hasattr(parsed_data, 'identity') and parsed_data.identity == "NAV-PVT":
            f9h_num_sv = parsed_data.numSV if hasattr(parsed_data, 'numSV') else 0
           # Check RTK status based on CARRSOLN (carrier phase solution)
            if hasattr(parsed_data, 'carrSoln'):
                carrsoln = parsed_data.carrSoln  # 0 = NO RTK, 1 = RTK FLOAT, 2 = RTK FIXED
                if carrsoln == 0:
                    rtk_status = "No RTK"
                elif carrsoln == 1:
                    rtk_status = "RTK FLOAT"
                elif carrsoln == 2:
                    rtk_status = "RTK FIXED"
                else:
                    rtk_status = "Unknown"

        
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
                msg.iTOW = parsed_data.iTOW
                msg.headAcc = int(parsed_data.accHeading)
                
                if parsed_data.carrSoln == 2:
                    msg.flags |= msg.CARRIER_PHASE_FIXED
                elif parsed_data.carrSoln == 1:
                    msg.flags |= msg.CARRIER_PHASE_FLOAT

                rospy.loginfo(self.name.center(25,"="))
                # rospy.loginfo(f"unixtime: {msg.header.s/s/s.stamp}")
                rospy.loginfo(f"RTK: {rtk_status}")
                rospy.loginfo(f"heading (deg): {heading_deg}")
                rospy.loginfo(f"heading (rad): {heading_deg * 3.1415 / 180}")                
                rospy.loginfo("="*(25))

                publisher.publish(msg)

    def get_message_ros_time(self,parsed_data):

        year = parsed_data.year
        month = parsed_data.month
        day = parsed_data.day

        hour = parsed_data.hour
        min = parsed_data.min
        second = parsed_data.second

        utc_time = datetime(
            year=year,
            month=month,
            day=day,
            hour=hour,
            minute=min,     # Replace with your `min` variable
            second=second   # Replace with your `second` variable
        )

        # Convert to UNIX timestamp (seconds since epoch)
        unix_secs = calendar.timegm(utc_time.timetuple())
        ros_secs = int(unix_secs)
        ros_nsecs = 0

        # Create ROS Time object
        return rospy.Time(ros_secs, ros_nsecs)

    def calculate_covariance(self, hAcc, vAcc):
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
   

class GNSSProcessor:
    def __init__(self):
        self.f9p = GNSSDevice(
            rospy.get_param('gnss/port', '/dev/tty_gps_module'),
            rospy.get_param('gnss/baudrate', 38400),
            "f9p_base"
        )
        # self.f9h = GNSSDevice(
        #     rospy.get_param('gnss/f9h_port', '/dev/tty_f9h'),
        #     rospy.get_param('gnss/f9h_baudrate', 38400),
        #     "f9h_rover"
        # )

        self.pub_fix = rospy.Publisher(
            rospy.get_param('gnss/topic', '/gps/fix'), 
            NavSatFix, 
            queue_size=10
        )

        # self.pub_heading_f9h = rospy.Publisher(
        #     rospy.get_param('gnss/heading_topic','/gps/heading'),
        #     NavPVT,
        #     queue_size=10
        # )

        self.f9p.connect()
        self.f9p.configure_base()

        # self.f9h.connect()
        # self.f9h.configure_rover()

    def run(self):

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.f9p.read_base(None,self.pub_fix)
            
            #self.f9h.read_rover(self.pub_heading_f9h)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('gnss_processor')
    processor = GNSSProcessor()
    processor.run()