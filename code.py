# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
'''Simple Test for the PiCowbell CAN Bus with Raspberry Pi Pico'''

from time import sleep
import board
import busio
from digitalio import DigitalInOut
from adafruit_mcp2515.canio import Message, RemoteTransmissionRequest
from adafruit_mcp2515 import MCP2515 as CAN


#cs = DigitalInOut(board.GP24)
#cs.switch_to_output()
#spi = busio.SPI(board.GP18, board.GP19, board.GP16)

cs = DigitalInOut(board.CAN_CS)
cs.switch_to_output()
spi = board.SPI()

can_bus = CAN(
    spi, cs, baudrate=250000, loopback=False, silent=False
)  # use loopback to test without another device

sleep(2)
while True:
    #prepare the message
    msgToSend = Message(id=0x123, data=b"DEADBEEF", extended=False)
    can_bus.send(msgToSend)
    #send message

    state = can_bus.state
    print("State:", state)



    #receive a message
    msgToRecv = can_bus.read_message()
    if msgToRecv != None:
        print("Message ID ", hex(msgToRecv.id))
        if isinstance(msgToRecv , Message):
            print("message data:", msgToRecv.data)


    sleep(1)
