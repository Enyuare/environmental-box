import board
import digitalio
import time
import struct
import busio
import adafruit_tca9548a
import adafruit_bmp3xx
import adafruit_ahtx0
import adafruit_lsm9ds1
import analogio
import math
from digitalio import DigitalInOut
from adafruit_mcp2515.canio import Message, RemoteTransmissionRequest
from adafruit_mcp2515 import MCP2515 as CAN
from adafruit_bno08x_rvc import BNO08x_RVC

TempSID = 0
PressureSID = 0
HumiditySID = 0
BatterySID = 0
AttitudeSID = 0

analog_pin = analogio.AnalogIn(board.A0)
analog_pin1 = analogio.AnalogIn(board.A1)
analog_pin2 = analogio.AnalogIn(board.A2)

uart = busio.UART(board.TX, board.RX, baudrate=115200, receiver_buffer_size=2048)
rvc = BNO08x_RVC(uart)

# Create I2C bus as normal
i2c = board.I2C()  # uses board.SCL and board.SDA
# Create the TCA9548A object and give it the I2C bus
tca = adafruit_tca9548a.TCA9548A(i2c)

# Function to read voltage at the ADC pin
def read_voltage(pin):
    # Convert raw ADC value to voltage
    return (pin.value / 65535) * 3.3  # Use 65535 for a 16-bit ADC

def send_temperature(can_bus, temperature_celsius, temp_instance, temp_source, Source_Address, Priority=5, PGN=130312):
    global TempSID
    # Convert temperature to 0.01°C units
    Temperature = int(temperature_celsius * 100)
    Temperature_bytes = Temperature.to_bytes(2, 'little', signed=True)

    # Set Temperature (not used)
    Set_Temperature_bytes = b'\xFF\x7F'  # Data Not Available

    # Reserved byte
    Reserved_byte = 0xFF

    # Construct the data payload
    datatemp = bytes([
        TempSID,
        temp_instance,
        temp_source,
        Temperature_bytes[0],
        Temperature_bytes[1],
        Set_Temperature_bytes[0],
        Set_Temperature_bytes[1],
        Reserved_byte
    ])

    # Construct the CAN ID
    CAN_ID = (Priority << 26) | (PGN << 8) | Source_Address

    # Create and send the CAN message
    message = Message(id=CAN_ID, data=datatemp, extended=True)
    #send_success = can_bus.send(message)
    #print(f"Sent Temperature: {temperature_celsius:.2f}°C, Instance: {temp_instance}, Source: {temp_source}, Success: {send_success}")
    #print(f"Sent CAN ID: {hex(CAN_ID)}, Data: {datatemp.hex()}")

    TempSID = TempSID + 1
    if TempSID == 252:
        TempSID = 0

def send_pressure(can_bus, pressure_pascals, pressure_instance, pressure_source, Source_Address, Priority=5, PGN=130314):
    global PressureSID
    # Encode the pressure as a 32-bit float, little-endian
    Actual_Pressure_bytes = struct.pack('<f', pressure_pascals)

    # Construct the data payload
    data_pressure = bytes([
        PressureSID,
        pressure_instance,
        pressure_source,
        Actual_Pressure_bytes[0],
        Actual_Pressure_bytes[1],
        Actual_Pressure_bytes[2],
        Actual_Pressure_bytes[3]
    ])

    # Construct the CAN ID
    CAN_ID = (Priority << 26) | (PGN << 8) | Source_Address

    # Create and send the CAN message
    message_pressure = Message(id=CAN_ID, data=data_pressure, extended=True)
    #send_success = can_bus.send(message_pressure)
    #print(f"Sent Pressure: {pressure_pascals:.2f} Pa, Instance: {pressure_instance}, Source: {pressure_source}, Success: {send_success}")

    PressureSID = PressureSID + 1
    if PressureSID == 252:
        PressureSID = 0

def send_humidity(can_bus, humidity_percentage, humidity_instance, humidity_source, Source_Address, Priority=5, PGN=130313):
    global HumiditySID
    # Scale the humidity value: value = humidity / 0.004
    Actual_Humidity = int(humidity_percentage / 0.004)
    if Actual_Humidity > 25000:
        Actual_Humidity = 25000  # Cap at 100%

    # Encode as unsigned 16-bit integer, little-endian
    Actual_Humidity_bytes = Actual_Humidity.to_bytes(2, 'little', signed=False)

    # Set Humidity (not used)
    Set_Humidity_bytes = b'\xFF\x7F'  # Data Not Available

    # Reserved byte
    Reserved_byte = 0xFF

    # Construct the data payload
    data_humidity = bytes([
        HumiditySID,
        humidity_instance,
        humidity_source,
        Actual_Humidity_bytes[0],
        Actual_Humidity_bytes[1],
        Set_Humidity_bytes[0],
        Set_Humidity_bytes[1],
        Reserved_byte
    ])

    # Construct the CAN ID
    CAN_ID = (Priority << 26) | (PGN << 8) | Source_Address

    # Create and send the CAN message
    message_humidity = Message(id=CAN_ID, data=data_humidity, extended=True)
    #send_success = can_bus.send(message_humidity)
    #print(f"Sent Humidity: {humidity_percentage:.2f}%, Instance: {humidity_instance}, Source: {humidity_source}, Success: {send_success}")
    #print(f"Sent CAN ID: {hex(CAN_ID)}, Data: {data_humidity.hex()}")
    HumiditySID = HumiditySID + 1
    if HumiditySID == 252:
        HumiditySID = 0

def send_battery_status(can_bus, current_A, battery_instance, Source_Address, Priority=6, PGN=127508):
    global BatterySID

    # Scale voltage and current
    Current = int(current_A / 0.1)   # Convert current to units of 0.1 A

    if Current > 0xFFFF or Current < 0:
        Current_bytes = b'\xFF\xFF'
    else:
        Current_bytes = Current.to_bytes(2, 'little', signed=False)

    # Construct data payload
    data_battery = bytes([
        battery_instance,
        0xFF, 0xFF,  # Voltage set to Data Not Available (since you only want current)
        Current_bytes[0],
        Current_bytes[1],
        0xFF, 0xFF,  # Temperature set to Data Not Available (since you only want current)
        BatterySID
    ])

    # Construct the CAN ID
    CAN_ID = (Priority << 26) | (PGN << 8) | Source_Address

    # Create and send the CAN message
    message_battery = Message(id=CAN_ID, data=data_battery, extended=True)
    send_success = can_bus.send(message_battery)
    # Optionally, print the send status
    # print(f"Sent Battery Status: Current: {Current:.2f}A, Instance: {battery_instance}, Success: {send_success}")

    # Update BatterySID
    BatterySID += 1
    if BatterySID >= 253:
        BatterySID = 0


def send_attitude(can_bus, yaw_radians, pitch_radians, roll_radians, Source_Address, Priority=3, PGN=127257):
    global AttitudeSID

    # Scale the angles (radians to scaled int21eger)
    yaw_scaled = int(yaw_radians * 10000)
    pitch_scaled = int(pitch_radians * 10000)
    roll_scaled = int(roll_radians * 10000)

    # Print the scaled values
    # print(f"Yaw (scaled): {yaw_scaled}")
    # print(f"Pitch (scaled): {pitch_scaled}")
    # print(f"Roll (scaled): {roll_scaled}")


    # Convert scaled values to bytes (little-endian, signed)
    yaw_bytes = yaw_scaled.to_bytes(2, 'little', signed=True)
    pitch_bytes = pitch_scaled.to_bytes(2, 'little', signed=True)
    roll_bytes = roll_scaled.to_bytes(2, 'little', signed=True)

    # Reserved byte
    reserved_byte = 0xFF

    # Construct the data payload
    data = bytes([
        AttitudeSID,
        yaw_bytes[0],
        yaw_bytes[1],
        pitch_bytes[0],
        pitch_bytes[1],
        roll_bytes[0],
        roll_bytes[1],
        reserved_byte
    ])

    # Construct the CAN ID
    CAN_ID = (Priority << 26) | (PGN << 8) | Source_Address

    # Create and send the CAN message
    message = Message(id=CAN_ID, data=data, extended=True)
    #send_success = can_bus.send(message)

    # Uncomment below to print the sent data for debugging
    #print(f"Sent Attitude: Yaw={yaw_radians:.4f} rad, Pitch={pitch_radians:.4f} rad, Roll={roll_radians:.4f} rad")
    #print(f"CAN ID: {hex(CAN_ID)}, Data: {data.hex()}, Send Success: {send_success}")

    # Increment the Sequence ID (SID)
    AttitudeSID += 1
    if AttitudeSID >= 252:
        AttitudeSID = 0


def construct_name(unique_number, manufacturer_code, device_instance_lower,
                   device_instance_upper, device_function, device_class,
                   system_instance, industry_group, arbitrary_address_capable):

    name = (unique_number & 0x1FFFFF)
    name |= (manufacturer_code & 0x7FF) << 21
    name |= (device_instance_lower & 0x7) << 32
    name |= (device_instance_upper & 0x1F) << 35
    name |= (device_function & 0xFF) << 40
    name |= (0 & 0x1) << 48  # NMEA Reserved bit set to 0
    name |= (device_class & 0x7F) << 49
    name |= (system_instance & 0xF) << 56
    name |= (industry_group & 0x7) << 59
    name |= (arbitrary_address_capable & 0x1) << 63
    return name

def claim_address(can_bus, source_address, name):
    # ISO Address Claim PGN
    PGN = 60928
    Priority = 6

    # Construct the CAN ID
    CAN_ID = (Priority << 26) | (PGN << 8) | source_address

    # Convert NAME to bytes (big-endian)
    name_bytes = name.to_bytes(8, 'big')

    # Create the CAN message
    message = Message(id=CAN_ID, data=name_bytes, extended=True)

    # Send the Address Claim message
    send_success = can_bus.send(message)
    #print(f"Sent Address Claim with Source Address {source_address}, Success: {send_success}")

    # Listen for conflicts
    conflict = False
    timeout = time.monotonic() + 0.25  # 250 ms timeout

    with can_bus.listen(timeout=0.25) as listener:
        while time.monotonic() < timeout:
            incoming_message = listener.receive()
            if incoming_message:
                incoming_PGN = (incoming_message.id >> 8) & 0x1FFFF
                if incoming_PGN == PGN:
                    incoming_source = incoming_message.id & 0xFF
                    if incoming_source == source_address:
                        incoming_name = int.from_bytes(incoming_message.data, 'big')
                        if incoming_name < name:
                            # Other device has higher priority
                            #print("Address conflict detected. Selecting a new address.")
                            conflict = True
                            break
                        else:
                            # We have higher priority; resend Address Claim
                            can_bus.send(message)
            else:
                time.sleep(0.01)
    return not conflict

for channel in range(8):
    print(f"Scanning PCA9548 channel {channel}")
    if tca[channel].try_lock():
        try:
            addresses = tca[channel].scan()
            print("Found I2C addresses:", [hex(address) for address in addresses])
        finally:
            tca[channel].unlock()
    time.sleep(0.5)

# Connect to sensor boards via I2C
i2c = board.I2C()
tca = adafruit_tca9548a.TCA9548A(i2c) #For the Multiplexor (PCA9548)

# For temperature/pressure (BMP390)
# Create each BMP390 using the TCA9548A channel instead of the I2C object
# Uncomment the channel that the sensor is plugged into.
# When requesting a value from this sensor put:
# For pressure, pressure_X = bmpX.pressure, in hPa (divide by 10 for kPa)
# For temperature,  temp_X = bmpX.temperature, in degrees C
# Where "X" is the sensor number below.
#bmp0 = adafruit_bmp3xx.BMP3XX_I2C(tca[0])   # TCA Channel 0
bmp1 = adafruit_bmp3xx.BMP3XX_I2C(tca[1])  # TCA Channel 1
#bmp2 = adafruit_bmp3xx.BMP3XX_I2C(tca[2])  # TCA Channel 2
#bmp3 = adafruit_bmp3xx.BMP3XX_I2C(tca[3])   # TCA Channel 3
#bmp4 = adafruit_bmp3xx.BMP3XX_I2C(tca[4])  # TCA Channel 4
#bmp5 = adafruit_bmp3xx.BMP3XX_I2C(tca[5])   # TCA Channel 5
#bmp6 = adafruit_bmp3xx.BMP3XX_I2C(tca[6])  # TCA Channel 6
#bmp7 = adafruit_bmp3xx.BMP3XX_I2C(tca[7])  # TCA Channel 7

# For temperature/humidity (AHT20)
# Create each AHT20 using the TCA9548A channel instead of the I2C object
# Uncomment the channel that the sensor is plugged into.
# When requesting a value from this sensor put:
# For humidity, rel_hum_X = ahtX.relative_humidity
# For temperature, temp_X = ahtX.temperature, in degrees C
# Where "X" is the sensor number below.
#aht0 = adafruit_ahtx0.AHTx0(tca[0])   # TCA Channel 0
#aht1 = adafruit_ahtx0.AHTx0(tca[1])  # TCA Channel 1
aht2 = adafruit_ahtx0.AHTx0(tca[2])  # TCA Channel 2
#aht3 = adafruit_ahtx0.AHTx0(tca[3])   # TCA Channel 3
#aht4 = adafruit_ahtx0.AHTx0(tca[4])  # TCA Channel 4
#aht5 = adafruit_ahtx0.AHTx0(tca[5])   # TCA Channel 5
#aht6 = adafruit_ahtx0.AHTx0(tca[6])  # TCA Channel 6
#aht7 = adafruit_ahtx0.AHTx0(tca[7])  # TCA Channel 7

cs = DigitalInOut(board.CAN_CS)
cs.switch_to_output()
spi = board.SPI()

# CAN Configuration
#can_bus = CAN(
#    spi, cs, loopback=True, silent=True
#)  # use loopback to test without another device

can_bus = CAN(
    spi, cs, loopback=True, silent=True
)  # use loopback to test with another device

# set up name for address claim
unique_number = 0x1FFFFF       # Unique device number (21 bits)
manufacturer_code = 0x1FF      # Manufacturer code (11 bits)
device_instance_lower = 0x0    # Device instance lower (3 bits)
device_instance_upper = 0x0    # Device instance upper (5 bits)
device_function = 0x01         # Device function (8 bits)
device_class = 0x1F            # Device class (7 bits)
system_instance = 0x0          # System instance (4 bits)
industry_group = 0x4           # Industry group (3 bits)
arbitrary_address_capable = 0x1  # Arbitrary address capable (1 bit)

name = construct_name(
    unique_number, manufacturer_code, device_instance_lower,
    device_instance_upper, device_function, device_class,
    system_instance, industry_group, arbitrary_address_capable
)
# get source address
source_address = 217  # Starting address
while source_address <= 253:
    success = claim_address(can_bus, source_address, name)
    if success:
        print(f"Successfully claimed Source Address {source_address}")
        break
    else:
        source_address += 1

if source_address > 253:
    print("Failed to claim any address on the network.")
    # Handle failure (e.g., reset or halt operation)


# Initialize counters and timing variables
attitude_count = 0
temperature_count = 0
pressure_count = 0
humidity_count = 0
current_count = 0
start_time = time.monotonic()
interval = 5  # Interval in seconds to calculate and print frequencies

while True:
    # Initialize max tracking variables
    max_current, max_current1, max_current2 = 0, 0, 0

    max_yaw, max_pitch, max_roll = 0, 0, 0

    for i in range(1):
        for j in range(50):
            # Attitude readings
            roll, pitch, yaw, x_accel, y_accel, z_accel = rvc.heading
            # print("Roll: %2.2f Pitch: %2.2f Yaw: %2.2f Degrees" % (roll, pitch, yaw))
            # Track max values
            max_yaw = max(max_yaw, abs(yaw))
            max_pitch = max(max_pitch, abs(pitch))
            max_roll = max(max_roll, abs(roll))

            # Current readings for sensor 1
            voltage = read_voltage(analog_pin)
            sensor_voltage = voltage * 1.5
            current = (sensor_voltage / 5.0) * 200.0
            max_current = max(max_current, current)
            print(f"Current Sensor 1: {current:.2f} A (Max: {max_current:.2f} A)")

            # Current readings for sensor 2
            voltage1 = read_voltage(analog_pin1)
            sensor_voltage1 = voltage1 * 1.5
            current1 = (sensor_voltage1 / 5.0) * 100.0
            max_current1 = max(max_current1, current1)
            print(f"Current Sensor 2: {current1:.2f} A (Max: {max_current1:.2f} A)")

            # Current readings for sensor 3
            voltage2 = read_voltage(analog_pin2)
            sensor_voltage2 = voltage2 * 1.5
            current2 = (sensor_voltage2 / 5.0) * 100.0
            max_current2 = max(max_current2, current2)
            print(f"Current Sensor 3: {current2:.2f} A (Max: {max_current2:.2f} A)")

        # Convert from degrees to radians
        roll_radians = max_roll * (math.pi / 180.0)
        pitch_radians = max_pitch * (math.pi / 180.0)
        yaw_radians = max_yaw * (math.pi / 180.0)

        # Print the converted values in a formatted way
        # print(f"Roll (radians): {roll_radians:.4f}")
        # print(f"Pitch (radians): {pitch_radians:.4f}")
        # print(f"Yaw (radians): {yaw_radians:.4f}")

        # Send max attitude
        send_attitude(can_bus, yaw_radians, pitch_radians, roll_radians, source_address)
        attitude_count += 1

        # Send max current
        battery_instance = 0x01
        send_battery_status(can_bus, max_current, battery_instance, source_address)
        current_count += 1
        # Send max current
        battery_instance = 0x02
        send_battery_status(can_bus, max_current1, battery_instance, source_address)
        # Send max current
        battery_instance = 0x03
        send_battery_status(can_bus, max_current2, battery_instance, source_address)

        # Reset max tracking variables for next iteration
        max_current, max_current1, max_current2 = 0, 0, 0
        max_yaw, max_pitch, max_roll = 0, 0, 0



    # sending temperature bmp 1
    temperature_celsius = bmp1.temperature  # temperature value in degrees celsius
    temp_instance = 0x01  # bmp1 source = 1
    temp_source = 0x01    # outside temperature
    send_temperature(can_bus, temperature_celsius, temp_instance, temp_source, source_address)
    temperature_count += 1
    # print(f"Temperature in Celsius: {temperature_celsius:.2f}")

    # sending pressure bmp 1
    pressure_pascals = bmp1.pressure * 100  # pressure value in Pa (converted from hPa)
    pressure_instance = 0x01  # bmp1 source = 1
    pressure_source = 0x00    # atmospheric pressure
    send_pressure(can_bus, pressure_pascals, pressure_instance, pressure_source, source_address)
    pressure_count += 1
    # print(f"Pressure in Pa: {pressure_pascals:.2f}")

    # sending humidity aht 2
    humidity_percentage = aht2.relative_humidity  # relative humidity in percent
    humidity_instance = 0x01  # bmp1 source = 1
    humidity_source = 0x00    # inside humidity
    send_humidity(can_bus, humidity_percentage, humidity_instance, humidity_source, source_address)
    # Print the humidity data for debugging
    #print(f"Humidity: {humidity_percentage:.2f}%, Instance: {humidity_instance}, Source: {humidity_source}")
    humidity_count += 1
    #print(f"Humidity: {humidity_percentage:.2f}")


    # Check if it's time to calculate and print frequencies
    current_time = time.monotonic()
    if current_time - start_time >= interval:
        elapsed_time = current_time - start_time
        # Calculate frequencies
        attitude_freq = attitude_count / elapsed_time
        temperature_freq = temperature_count / elapsed_time
        #print(f"count: {temperature_count:.2f}")
        #print(f"time: {elapsed_time:.2f}")
        pressure_freq = pressure_count / elapsed_time
        humidity_freq = humidity_count / elapsed_time
        current_freq = current_count / elapsed_time
        # Print frequencies
        #print(f"Frequencies over {elapsed_time:.2f} seconds:")
        #print(f"Attitude messages per second: {attitude_freq:.2f} Hz")
        #print(f"Temperature messages per second: {temperature_freq:.2f} Hz")
        #print(f"Pressure messages per second: {pressure_freq:.2f} Hz")
        #print(f"Humidity messages per second: {humidity_freq:.2f} Hz")
        #print(f"Current messages per second: {current_freq:.2f} Hz")
        # Reset counters and start_time
        attitude_count = 0
        temperature_count = 0
        pressure_count = 0
        humidity_count = 0
        current_count = 0
        start_time = current_time

