import re
import time
import threading
import serial
import matplotlib.pyplot as plt

SERIAL_PORT = "COM9"
BAUD_RATE = 115200

PRINT_INTERVAL_SECONDS = 0.10

old_line_pattern = re.compile(
    r"angle=([-0-9.]+)\s*\|\s*"
    r"pid=([-0-9.]+)\s*\|\s*"
    r"p=([-0-9.]+)\s*\|\s*"
    r"i=([-0-9.]+)\s*\|\s*"
    r"d=([-0-9.]+)\s*\|\s*"
    r"motor=([-0-9.]+)\s*\|\s*"
    r"pwmA=([-0-9.]+)\s*\|\s*"
    r"pwmB=([-0-9.]+)"
)

key_value_pattern = re.compile(r"^([A-Za-z0-9_]+)\s*=\s*([-0-9.]+)$")

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

serial_lock = threading.Lock()

current_live_values = {}


def clear_data():
    for key in data:
        data[key].clear()


def send_command(ser, command):
    with serial_lock:
        ser.write((command + "\n").encode())


def append_sample(values):
    global last_terminal_print

    if not recording or start_time is None:
        return

    required_keys = [
        "angle",
        "pidOutput",
        "p",
        "i",
        "d",
        "motorOutput",
        "pwmA",
        "pwmB",
    ]

    for key in required_keys:
        if key not in values:
            return

    current_time = time.time() - start_time

    data["time"].append(current_time)
    data["angle"].append(values["angle"])
    data["pid"].append(values["pidOutput"])
    data["p"].append(values["p"])
    data["i"].append(values["i"])
    data["d"].append(values["d"])
    data["motor"].append(values["motorOutput"])
    data["pwmA"].append(values["pwmA"])
    data["pwmB"].append(values["pwmB"])

    if time.time() - last_terminal_print > 0.5:
        last_terminal_print = time.time()
        print(
            f"t={current_time:.2f}s | "
            f"angle={values['angle']:.2f} | "
            f"pid={values['pidOutput']:.2f} | "
            f"motor={values['motorOutput']:.2f} | "
            f"pwmA={values['pwmA']:.2f} | "
            f"pwmB={values['pwmB']:.2f} | "
            f"samples={len(data['time'])}"
        )


def serial_reader(ser):
    global finished
    global current_live_values

    while not finished:
        line = ser.readline().decode(errors="ignore").strip()

        if not line:
            continue

        old_match = old_line_pattern.search(line)

        if old_match:
            values = [float(value) for value in old_match.groups()]

            append_sample(
                {
                    "angle": values[0],
                    "pidOutput": values[1],
                    "p": values[2],
                    "i": values[3],
                    "d": values[4],
                    "motorOutput": values[5],
                    "pwmA": values[6],
                    "pwmB": values[7],
                }
            )

            continue

        key_value_match = key_value_pattern.match(line)

        if key_value_match:
            key = key_value_match.group(1)
            value = float(key_value_match.group(2))

            current_live_values[key] = value

            if key == "pwmB":
                append_sample(current_live_values)
                current_live_values = {}

            continue

        if (
            "settings" in line.lower()
            or "stopped" in line.lower()
            or "started" in line.lower()
            or "FORCE" in line
            or "failed" in line.lower()
            or "mpu" in line.lower()
            or "command" in line.lower()
        ):
            print(line)


def print_requester(ser):
    while not finished:
        if recording:
            send_command(ser, "print")

        time.sleep(PRINT_INTERVAL_SECONDS)


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

    if len(data["time"]) > 1 and duration > 0:
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
    print_thread = threading.Thread(target=print_requester, args=(ser,))

    reader_thread.start()
    print_thread.start()

    print("\nReady.")
    print("Type 'start' to start PID and recording.")
    print("Type 's' to stop PID, stop recording, and show graphs.")
    print("Type 'FORCE' to immediately stop PID, motors, and servos.")
    print("Type 'show' to request all settings.")
    print("Type 'print' to request one live value print.")
    print("You can also type Pico commands directly, for example:")
    print("  kp 3")
    print("  ki 0")
    print("  kd 0.2")
    print("  motor deadband off")
    print("  motor curve 1.5")
    print("  servo status\n")

    while True:
        command = input("> ").strip()

        if command == "start":
            clear_data()
            start_time = time.time()
            recording = True

            send_command(ser, "pid start")
            print("PID started. Recording started.")

        elif command == "s" or command == "stop":
            send_command(ser, "pid stop")
            recording = False
            finished = True

            print("Recording stopped.")
            break

        elif command == "FORCE":
            send_command(ser, "FORCE")
            recording = False
            finished = True

            print("FORCE command sent.")
            break

        elif command == "stopmotors":
            send_command(ser, "stopmotors")
            recording = False
            finished = True

            print("Stop motors command sent.")
            break

        elif command == "show":
            send_command(ser, "settings")
            print("Requested all settings from Pico.")

        elif command == "print":
            send_command(ser, "print")
            print("Requested live values from Pico.")

        elif command == "help":
            send_command(ser, "help")
            print("Requested help from Pico.")

        elif command:
            send_command(ser, command)
            print(f"Sent command: {command}")

    reader_thread.join()
    print_thread.join()
    ser.close()

    plot_data()


if __name__ == "__main__":
    main()