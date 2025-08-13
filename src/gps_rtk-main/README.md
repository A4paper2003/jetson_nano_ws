# **GPS RTK ROS Bridge**

This package enables seamless integration of RTK GPS (Real-Time Kinematic Global Positioning System) with a ROS environment. It provides dual-module positioning and heading by connecting a u-blox F9P base station with an F9H rover, delivering high-precision positioning and heading data as ROS messages.

---

## **Prerequisites**

### **1. Install RTKLIB**  
RTKLIB is an open-source program library for precise positioning with GNSS. Install it using the following command:  
```bash
sudo apt install rtklib
```

### **2. Map the GPS Modules to Fixed TTY Ports**  
To ensure consistent access to your GPS devices, we map their dynamic assignments to fixed names. This avoids conflicts and simplifies device handling.

1. Create a udev rules file:  
   ```bash
   sudo nano /etc/udev/rules.d/50-ardusimple.rules
   ```

2. Add the following lines to the file:  
   ```bash
   # For F9P module (base)
   KERNEL=="ttyACM[0-9]*", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK="tty_f9p", GROUP="dialout", MODE="0666"

   # For F9H module with HL-340 adapter (rover)
   KERNEL=="ttyUSB[0-9]*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK="tty_f9h", GROUP="dialout", MODE="0666"
   ```
   - `idVendor` and `idProduct`: Specific to the devices. Use `lsusb` to confirm these values for your devices.
   - `SYMLINK`: The custom names you want to assign.
   - `GROUP`: Ensures access rights for users in the `dialout` group.
   - `MODE`: Sets permissions for the device files.

3. Save and exit the file.  

4. Reload the udev rules:  
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

Now, your GPS devices will consistently appear at `/dev/tty_f9p` and `/dev/tty_f9h`.

---

## **Setup Instructions**

### **1. Clone the Repository**  
Navigate to your ROS workspace's `src/` folder and clone the package repository:  
```bash
cd ~/catkin_ws/src
git clone https://github.com/Kodifly/gps_rtk
```

### **2. Build the Package**  
Return to the root of your ROS workspace and build the package:  
```bash
cd ~/catkin_ws
catkin_make
```

### **3. Configure Parameters**  
The GNSS module settings are stored in the `gnss_config.yaml` file. You can find this file in the `config/` folder of the package.  

**Edit the `gnss_config.yaml` file** to configure your GPS modules:  
```yaml
gnss:
  port: "/dev/tty_f9p"           # F9P base module
  baudrate: 115200               # Baudrate for the F9P module
  f9h_port: "/dev/tty_f9h"       # F9H rover module
  f9h_baudrate: 38400            # Baudrate for the F9H module  
  topic: "/gps/fix"              # ROS topic to publish NavSatFix messages
  heading_topic: "/gps/heading"  # ROS topic to publish heading messages
  debug: false                   # Debug flag to control logging verbosity
```

---

## **Requirements**

Ensure that the following Python packages are installed:

```text
rospy
pyubx2
pyserial
```

You can install them using the `requirements.txt` file:

```bash
pip install -r requirements.txt
```

Additionally, you need the ublox_msgs ROS package for the NavPVT message type:

```bash
sudo apt install ros-$ROS_DISTRO-ublox-msgs
```

---

## **Usage**

### Launch with ROS  
Ensure your workspace is sourced:  
```bash
source ~/catkin_ws/devel/setup.bash
```

Launch the RTK GPS setup using the provided ROS launch file:  
```bash
roslaunch gps_rtk rtk.launch
```
---

## **How It Works**

This package implements a dual-receiver RTK setup for both positioning and heading:

1. **F9P Module (Base):**
   - Configured as a moving base station
   - Generates RTCM3 correction messages
   - Provides position data via NAV-PVT messages
   - Provides heading data via NAV-RELPOSNED messages

2. **F9H Module (Rover):**
   - Receives RTCM3 corrections from the F9P module
   - Provides backup heading data if the F9P heading is unavailable

3. **Data Flow:**
   - RTCM3 messages flow from F9P to F9H
   - Position data is published as sensor_msgs/NavSatFix
   - Heading data is published as ublox_msgs/NavPVT

4. **Update Rate:**
   - Both modules are configured for a 20Hz update rate

---

## **Published Topics**

- **`/gps/fix` (sensor_msgs/NavSatFix):**
  - High-precision RTK position data
  - Includes latitude, longitude, altitude
  - Covariance data based on horizontal and vertical accuracy

- **`/gps/heading` (ublox_msgs/NavPVT):**
  - Heading information in degrees
  - RTK quality flags
  - Heading accuracy

---

## **Additional Notes**

- **RTK Hardware Setup:**  
  This package requires two u-blox receivers: an F9P (base) and an F9H (rover), with RTK antennas properly installed with the correct baseline orientation for heading determination.

- **Heading Calculation:**
  The heading is derived from the relative position between two GNSS antennas, providing true north reference that's more reliable than a magnetometer in environments with magnetic interference.

---

## **Troubleshooting**

1. **Device Not Found:**  
   Ensure the `/dev/tty_f9p` and `/dev/tty_f9h` mappings work. Check using:  
   ```bash
   ls -l /dev/tty_f9p /dev/tty_f9h
   ```

2. **RTK Status Issues:**  
   Verify that both receivers are properly connected and have good sky view. The console output will show current RTK status.

3. **Heading Issues:**
   - Ensure both antennas have a clear sky view
   - Check that antennas are properly oriented and at the correct distance
   - Verify RTK solution is FLOAT or FIXED for reliable heading

4. **Message Rate:**
   The system is configured for 20Hz updates. If you need a different rate, modify the `measRate` parameter in the configuration functions:
   ```python
   msg_rate = UBXMessage("CFG", "CFG-RATE", SET, 
                       measRate=50,   # Measurement rate in ms (50ms = 20Hz)
                       navRate=1,     # Navigation rate (cycles per measurement)
                       timeRef=1)     # Alignment to UTC time
   ```

5. **ROS Message Not Publishing:**  
   Confirm topics are available using `rostopic list` and monitor data with `rostopic echo /gps/heading`