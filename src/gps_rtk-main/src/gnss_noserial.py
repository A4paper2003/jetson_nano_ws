#!/usr/bin/env python3

import rospy
import gpsd
from sensor_msgs.msg import NavSatFix

class GNSSProcessor:
    def __init__(self):
        """
        Initializes the node, connects to gpsd, and sets up the ROS publisher.
        """
        rospy.loginfo("Connecting to gpsd...")
        try:
            # This one line replaces all the serial connection and configuration code
            gpsd.connect()
            rospy.loginfo("Successfully connected to gpsd.")
        except Exception as e:
            rospy.logerr(f"Failed to connect to gpsd: {e}")
            rospy.signal_shutdown(f"Could not connect to gpsd: {e}")
            return

        # Get the topic from the ROS parameter server, with a default value
        topic_name = rospy.get_param('~gnss/topic', '/gps/fix')
        self.frame_id = rospy.get_param('~gnss/frame_id', 'gps')
        
        # Set up the publisher
        self.pub_fix = rospy.Publisher(topic_name, NavSatFix, queue_size=10)
        
        rospy.loginfo(f"Publishing NavSatFix messages on topic: {topic_name}")
        rospy.loginfo(f"Using frame_id: {self.frame_id}")

    def run(self):
        """
        Main loop that polls gpsd for data and publishes it.
        """
        rate = rospy.Rate(5)  # 5 Hz, adjust as needed
        while not rospy.is_shutdown():
            try:
                # Get the current data packet from gpsd
                packet = gpsd.get_current()

                # We need a 3D fix to publish a valid NavSatFix message
                if packet.mode >= 3:
                    
                    msg = NavSatFix()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = self.frame_id

                    # Populate the status field
                    # gpsd mode 3 = 3D fix. This corresponds to STATUS_FIX.
                    msg.status.status = msg.status.STATUS_FIX
                    msg.status.service = msg.status.SERVICE_GPS

                    # Position data
                    msg.latitude = packet.lat
                    msg.longitude = packet.lon
                   #msg.altitude = packet.alt_msl  # Altitude above mean sea level

                    # Position covariance
                    # Use the error estimates from gpsd (epx, epy, epv)
                    # These are 1-sigma errors in meters. Covariance is the square of the error.
                    var_x = packet.epx**2 if hasattr(packet, 'epx') else 0.0
                    var_y = packet.epy**2 if hasattr(packet, 'epy') else 0.0
                    var_v = packet.epv**2 if hasattr(packet, 'epv') else 0.0
                    
                    msg.position_covariance = [var_x, 0, 0,
                                               0, var_y, 0,
                                               0, 0, var_v]
                    msg.position_covariance_type = msg.COVARIANCE_TYPE_DIAGONAL_KNOWN

                    self.pub_fix.publish(msg)
                    
                    if rospy.get_param('~gnss/debug', True):
                         rospy.loginfo_throttle(5, f"Published 3D fix: Lat {packet.lat:.6f}, Lon {packet.lon:.6f}")

                else:
                    # Log that we are waiting for a better fix
                    rospy.loginfo_throttle(10, f"Waiting for a 3D fix. Current mode: {packet.mode}")

            except Exception as e:
                rospy.logwarn(f"Error while processing GPS data: {e}")
            
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('gnss_processor_gpsd')
    try:
        processor = GNSSProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("gpsd_processor node shutting down.")