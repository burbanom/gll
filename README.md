# Interfacing the Beaglebone Black (BBB) with the Garmin Lidar Lite V3 (GLL)
The sensor is connected to pins 19 and 20 on the P9 of the BBB.
The i2c bus 2 is the one that is activated by default on the BBB.
You will need to install the i2c-tools and python-smbus packages
using the package manager of your linux distro, e.g. sudo apt-get install PACKAGE-NAME
To detect the devices that are connected to it we run the 
following command:
sudo i2cdetect -y -r 2
The following output is obtained:
  
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- UU UU UU UU -- -- -- -- -- -- -- -- 
60: -- -- 62 -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --                         


This means that our device is on address 0x62. 
The 'datasheet' for the GLL indicates the different registers 
on the device: https://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf 

These interactions are carried out in the python script i2c_gll.py
that is found in this repository. 

Sources:

For a tutorial on i2c on BB
[1] http://derekmolloy.ie/beaglebone/beaglebone-an-i2c-tutorial-interfacing-to-a-bma180-accelerometer/
Technical datasheet for the GLL sensor
[2] https://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf
An example of python-smbus usage
[3] https://github.com/ControlEverythingCom/Raspberry-Pi-I2C-Python
For the actual wiring of the BBB and the sensor
[4] https://www.robot-electronics.co.uk/htm/arduino_examples.htm#SRF02,%20SRF08,%20SRF10,%20SRF235
