import serial
import time

DEVICE = "/dev/tty.usbmodem01"
# DEVICE = "COM7"
BAUDRATE = 115200
LOG_FILE = "log.txt"


def main():
    try:
        with serial.Serial(DEVICE, BAUDRATE, timeout=1) as ser, open(
            LOG_FILE, "a", encoding="utf-8"
        ) as log:
            print(f"Listening on {DEVICE} at {BAUDRATE} baud. Logging to {LOG_FILE}.")
            while True:
                try:
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode(errors="replace").rstrip("\r\n")
                    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                    output = f"[{timestamp}] {line}"
                    # print(output)
                    log.write(output + "\n")
                    log.flush()
                except serial.SerialException as exc:
                    print(f"Serial error: {exc}")
                    break
    except FileNotFoundError:
        print(f"Device not found: {DEVICE}")
    except serial.SerialException as exc:
        print(f"Could not open serial port: {exc}")


if __name__ == "__main__":
    main()
