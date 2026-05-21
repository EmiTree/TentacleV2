import re
import time
import threading
import serial
import matplotlib.pyplot as plt

SERIAL_PORT = "COM9"
BAUD_RATE = 115200

pattern = re.compile(
    r"angle=([-0-9.]+)\s*\|\s*"
    r"pid=([-0-9.]+)\s*\|\s*"
    r"p=([-0-9.]+)\s*\|\s*"
    r"i=([-0-9.]+)\s*\|\s*"
    r"d=([-0-9.]+)\s*\|\s*"
    r"motor=([-0-9.]+)\s*\|\s*"
    r"pwmA=([-0-9.]+)\s*\|\s*"
    r"pwmB=([-0-9.]+)"
)

data = {
    "time": [],
    "angle": [],
    "pid": [],
    "p": [],
    "i": [],
    "d": [],
    "motor": [],
    "pwmA": [],
    "pwmB": [],
}

recording = False
finished = False
start_time = None
last_terminal_print = 0


def clear_data():
    for key in data:
        data[key].clear()


def serial_reader(ser):
    global recording
    global finished
    global start_time
    global last_terminal_print

    while not finished:
        line = ser.readline().decode(errors="ignore").strip()

        if not line:
            continue

        match = pattern.search(line)

        if not match:
            if "Settings:" in line or "=" in line or "stopped" in line or "STOP" in line:
                print(line)
            continue

        if recording:
            values = [float(value) for value in match.groups()]
            current_time = time.time() - start_time

            data["time"].append(current_time)
            data["angle"].append(values[0])
            data["pid"].append(values[1])
            data["p"].append(values[2])
            data["i"].append(values[3])
            data["d"].append(values[4])
            data["motor"].append(values[5])
            data["pwmA"].append(values[6])
            data["pwmB"].append(values[7])

            if time.time() - last_terminal_print > 0.5:
                last_terminal_print = time.time()
                print(
                    f"t={current_time:.2f}s | "
                    f"angle={values[0]:.2f} | "
                    f"pid={values[1]:.2f} | "
                    f"motor={values[5]:.2f} | "
                    f"pwmA={values[6]:.2f} | "
                    f"pwmB={values[7]:.2f} | "
                    f"samples={len(data['time'])}"
                )


def style_axis(ax, title, ylabel, zero_line=True):
    ax.set_title(title, fontsize=12, fontweight="bold")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.35)

    if zero_line:
        ax.axhline(0, color="black", linewidth=1, alpha=0.7)


def plot_data():
    if len(data["time"]) == 0:
        print("No data recorded, so there is nothing to plot.")
        return

    duration = data["time"][-1] - data["time"][0]

    print(f"\nRecorded {len(data['time'])} samples.")
    print(f"Recording duration: {duration:.2f} seconds.")

    if len(data["time"]) > 1:
        average_dt = duration / (len(data["time"]) - 1)

        print(f"Average graph sample dt: {average_dt:.3f} seconds.")
        print(f"Average graph sample rate: {1.0 / average_dt:.1f} Hz.")

    plt.style.use("seaborn-v0_8-whitegrid")

    fig, axes = plt.subplots(4, 2, figsize=(14, 10), sharex=True)
    axes = axes.flatten()

    plot_config = [
        ("angle", "Angle", "degrees", "#1f77b4"),
        ("pid", "PID Output", "pid", "#d62728"),
        ("p", "P Term", "p", "#ff7f0e"),
        ("i", "I Term", "i", "#2ca02c"),
        ("d", "D Term", "d", "#9467bd"),
        ("motor", "Motor Output", "%", "#8c564b"),
        ("pwmA", "PWM A", "%", "#17becf"),
        ("pwmB", "PWM B", "%", "#e377c2"),
    ]

    for ax, (name, title, ylabel, color) in zip(axes, plot_config):
        ax.plot(data["time"], data[name], color=color, linewidth=1.8)
        style_axis(ax, title, ylabel)

    axes[0].set_ylim(-180, 180)
    axes[5].set_ylim(-105, 105)
    axes[6].set_ylim(-5, 105)
    axes[7].set_ylim(-5, 105)

    axes[-1].set_xlabel("time (s)")
    axes[-2].set_xlabel("time (s)")

    fig.suptitle("Tentacle PID Recording", fontsize=16, fontweight="bold")
    plt.tight_layout()
    plt.show()


def main():
    global recording
    global finished
    global start_time

    print("Close the VS Code serial monitor before running this.")
    print(f"Opening serial port {SERIAL_PORT}...")

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)

    reader_thread = threading.Thread(target=serial_reader, args=(ser,))
    reader_thread.start()

    print("\nReady.")
    print("Type 'start' to start PID and recording.")
    print("Type 's' to stop PID, stop recording, and show graphs.")
    print("Type 'stopmotors' to force motors off immediately.")
    print("Type 'show' to print current settings.")
    print("You can also type: kp 3, ki 0, kd 0.2, maxpidoutput 80\n")

    while True:
        command = input("> ").strip()

        if command == "start":
            clear_data()
            start_time = time.time()
            recording = True

            ser.write(b"start\n")
            print("Recording started.")

        elif command == "s" or command == "stop":
            ser.write(b"stop\n")
            recording = False
            finished = True

            print("Recording stopped.")
            break

        elif command == "stopmotors":
            ser.write(b"stopmotors\n")
            recording = False
            finished = True

            print("Force stop command sent.")
            break

        elif command == "show":
            ser.write(b"settings\n")
            print("Requested current settings from Pico.")

        else:
            ser.write((command + "\n").encode())
            print(f"Sent command: {command}")

    reader_thread.join()
    ser.close()

    plot_data()


if __name__ == "__main__":
    main()