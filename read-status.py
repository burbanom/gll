# coding: utf-8
import smbus
import time

__GLL_FULL_DELAY_HIGH   = 0x0F
__GLL_FULL_DELAY_LOW    = 0x10
__WRITE_ADDRESS         = 0x62
__READ_ADDRESS          = 0x62

# use ic2 bus 2
bus = smbus.SMBus(2)
#bus.write_byte_data(0x62,0x00,0x00)
# write 0x04 to register 0x00 of a device located at 0x62 on this bus.
bus.write_byte_data(__WRITE_ADDRESS,0x11,0xff)
    
   
for cycle in range(10):
    #time.sleep(1e-05)  # wait 67ms
    status = bus.read_byte_data(__READ_ADDRESS,0x01)
    print('The status is ' + str(status))
    print('The distance high byte is ' + str(bus.read_byte_data(__READ_ADDRESS,__GLL_FULL_DELAY_HIGH)) + ' cm')
    print('The distance low byte is ' + str(bus.read_byte_data(__READ_ADDRESS,__GLL_FULL_DELAY_HIGH)) + ' cm')
