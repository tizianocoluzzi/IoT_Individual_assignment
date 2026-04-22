import serial
import csv

# Configure serial port
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
OUTPUT_FILE = 'output.csv'

def parse_line(line):
    try:
        parts = line.split('\t')
        if len(parts) != 3:
            return None

        f1 = float(parts[0])
        f2 = float(parts[1])
        i = int(parts[2])

        return f1, f2, i
    except ValueError:
        return None

def main():
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser, \
         open(OUTPUT_FILE, mode='w', newline='') as csvfile:

        writer = csv.writer(csvfile)
        writer.writerow(['float1', 'float2', 'int'])  # header

        print("Listening on serial port...")

        while True:
            try:
                line = ser.readline().decode('utf-8').strip()
                if not line:
                    continue

                parsed = parse_line(line)
                if parsed:
                    writer.writerow(parsed)
                    #print("Saved:", parsed)
                else:
                    print("Invalid format:", line)

            except KeyboardInterrupt:
                print("\nStopped by user.")
                break
            except Exception as e:
                print("Error:", e)

if __name__ == "__main__":
    main()
