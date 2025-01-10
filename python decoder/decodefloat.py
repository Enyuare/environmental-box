import struct

def decode_pressure(bytes_input):
    """Decodes pressure from 4 bytes in little-endian IEEE 754 format."""
    if len(bytes_input) != 4:
        raise ValueError("Input must be exactly 4 bytes.")
    return struct.unpack('<f', bytes_input)[0]

def decode_current(bytes_input):
    """Decodes current from 2 bytes in little-endian format."""
    if len(bytes_input) != 2:
        raise ValueError("Input must be exactly 2 bytes.")
    raw_value = int.from_bytes(bytes_input, byteorder='little', signed=True)
    return raw_value * 0.1

def decode_attitude(bytes_input):
    """Decodes yaw, pitch, and roll from 6 bytes."""
    if len(bytes_input) != 6:
        raise ValueError("Input must be exactly 6 bytes.")
    yaw_raw = int.from_bytes(bytes_input[0:2], byteorder='little', signed=True)
    pitch_raw = int.from_bytes(bytes_input[2:4], byteorder='little', signed=True)
    roll_raw = int.from_bytes(bytes_input[4:6], byteorder='little', signed=True)
    return {
        "yaw": yaw_raw / 10000.0,
        "pitch": pitch_raw / 10000.0,
        "roll": roll_raw / 10000.0,
    }

def decode_humidity(bytes_input):
    """Decodes humidity from 2 bytes in little-endian format."""
    if len(bytes_input) != 2:
        raise ValueError("Input must be exactly 2 bytes.")
    raw_value = int.from_bytes(bytes_input, byteorder='little', signed=False)
    return raw_value * 0.004

def decode_temperature(bytes_input):
    """Decodes temperature from 3 bytes in little-endian format."""
    if len(bytes_input) != 3:
        raise ValueError("Input must be exactly 3 bytes.")
    raw_value = int.from_bytes(bytes_input, byteorder='little', signed=False)
    kelvin = raw_value * 0.001
    return kelvin - 273.15  # Convert to Celsius

def main():
    while True:
        print("\nWhat would you like to decode?")
        print("1. Pressure (4 bytes)")
        print("2. Current (2 bytes)")
        print("3. Attitude (6 bytes - yaw, pitch, roll)")
        print("4. Humidity (2 bytes)")
        print("5. Temperature (3 bytes)")
        print("6. Exit")

        choice = input("Enter your choice (1-6): ")

        if choice == "6":
            print("Exiting the program. Goodbye!")
            break  # Exit the loop

        try:
            if choice == "1":
                hex_input = input("Enter 4-byte hex data for Pressure (e.g., dc9fc147) Bytes 3-6: ")
                bytes_input = bytes.fromhex(hex_input)
                result = decode_pressure(bytes_input)
                print(f"Decoded Pressure: {result:.2f} Pa")
            elif choice == "2":
                hex_input = input("Enter 2-byte hex data for Current (e.g., 2700) Bytes 3-4: ")
                bytes_input = bytes.fromhex(hex_input)
                result = decode_current(bytes_input)
                print(f"Decoded Current: {result:.2f} A")
            elif choice == "3":
                hex_input = input("Enter 6-byte hex data for Attitude (e.g., d51426000c00) Bytes 3-6: ")
                bytes_input = bytes.fromhex(hex_input)
                result = decode_attitude(bytes_input)
                print(f"Decoded Attitude: Yaw={result['yaw']:.4f} rad, Pitch={result['pitch']:.4f} rad, Roll={result['roll']:.4f} rad")
            elif choice == "4":
                hex_input = input("Enter 2-byte hex data for Humidity (e.g., 9818) Bytes 3-4: ")
                bytes_input = bytes.fromhex(hex_input)
                result = decode_humidity(bytes_input)
                print(f"Decoded Humidity: {result:.2f}%")
            elif choice == "5":
                hex_input = input("Enter 3-byte hex data for Temperature (e.g., ec8204) Bytes 3-5: ")
                bytes_input = bytes.fromhex(hex_input)
                result = decode_temperature(bytes_input)
                print(f"Decoded Temperature: {result:.2f} °C")
            else:
                print("Invalid choice. Please enter a number between 1 and 6.")
        except ValueError as e:
            print(f"Error: {e}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    main()