## I2C Imports
import smbus
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import struct

bus = smbus.SMBus(1)
ardAddress = 0x04 #arduinos initialized I2C address


def I2CMessage(offset, floatNum):
  #general I2C message to send
    #struct.unpack("f", struct.pack("f", 0.00582811585976)) might be useful if datatypes messed up
    print(type(floatNum))
    ba = list(bytes(struct.pack('<f', floatNum)))

    print(ba)
    bus.write_block_data(ardAddress, offset, ba)
    pass


def rotate(degrees):
    I2CMessage(2, float(degrees))
    
    I2CMessage(4,0.0)
    pass

def moveForward(cm):
    I2CMessage(3, float(cm))
    I2CMessage(5,0.0)
    pass

I2CMessage(0,0)

while(True):
    instruct = input("Instruction: (r)otate or (m)ove or (s)can")
    if instruct == "r":
        rotate(input("Enter num degrees to rotate, left is negative"))
    elif instruct == "m":
        moveForward(input("Enter cm to move forward"))
    elif instruct == "s":
        #send scan signal
        I2CMessage(1,0)
        input("Enter any message to stop")
        I2CMessage(0,0)

    pass
