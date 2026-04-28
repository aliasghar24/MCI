import time
from collections import deque

import matplotlib.pyplot as plt
import serial


SERIAL_PORT = "/dev/ttyUSB0"  # Change if needed
BAUDRATE = 115200
READ_PERIOD_S = 0.010  # 10 ms
PLOT_UPDATE_PERIOD_S = 0.050  # refresh GUI every 50 ms
MAX_SAMPLES = 500


def main() -> None:
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0)
    except serial.SerialException as exc:
        print(f"Failed to open {SERIAL_PORT}: {exc}")
        print("Close other apps using the same COM port and try again.")
        return

    plt.ion()
    fig, ax = plt.subplots()
    ax.set_title("Live accX, gyroX & angle")
    ax.grid(True)
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Degree")

    line_acc, = ax.plot([], [], "r.-", label="accX")
    line_gyro, = ax.plot([], [], "g.-", label="gyroX")
    line_angle, = ax.plot([], [], "b.-", label="angle")
    ax.legend(loc="upper left")

    time_ms = deque(maxlen=MAX_SAMPLES)
    accx_vals = deque(maxlen=MAX_SAMPLES)
    gyrox_vals = deque(maxlen=MAX_SAMPLES)
    angle_vals = deque(maxlen=MAX_SAMPLES)

    next_read = time.perf_counter()
    next_plot = next_read
    sample_time_ms = 0

    print("Press Ctrl+C to stop.")

    try:
        while True:
            if not plt.fignum_exists(fig.number):
                break

            now = time.perf_counter()

            if now < next_read:
                # Short sleep avoids busy-waiting while keeping timing stable.
                time.sleep(min(0.001, next_read - now))
                continue

            next_read += READ_PERIOD_S

            # Catch up if plotting or system delay made us miss one or more ticks.
            if now - next_read > READ_PERIOD_S:
                next_read = now + READ_PERIOD_S

            latest_line = None
            while ser.in_waiting:
                raw = ser.readline()
                if raw:
                    latest_line = raw

            if latest_line:
                try:
                    values = latest_line.decode(errors="ignore").strip().split(",")
                    if len(values) == 3:
                        accx = float(values[0])
                        gyrox = float(values[1])
                        angle = float(values[2])

                        time_ms.append(sample_time_ms)
                        accx_vals.append(accx)
                        gyrox_vals.append(gyrox)
                        angle_vals.append(angle)
                        sample_time_ms += 10
                except ValueError:
                    # Ignore malformed serial lines.
                    pass

            if now >= next_plot and time_ms:
                next_plot = now + PLOT_UPDATE_PERIOD_S

                x = list(time_ms)
                y_acc = list(accx_vals)
                y_gyro = list(gyrox_vals)
                y_angle = list(angle_vals)

                line_acc.set_data(x, y_acc)
                line_gyro.set_data(x, y_gyro)
                line_angle.set_data(x, y_angle)

                ax.set_xlim(x[0], x[-1] + 10)

                ymin = min(min(y_acc), min(y_gyro), min(y_angle))
                ymax = max(max(y_acc), max(y_gyro), max(y_angle))
                margin = max(1.0, (ymax - ymin) * 0.1)
                ax.set_ylim(ymin - margin, ymax + margin)

                fig.canvas.draw()
                plt.pause(0.001)
    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()