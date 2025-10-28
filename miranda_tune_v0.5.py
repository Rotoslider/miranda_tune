#  v0.4
import time
import errno
from smbus2 import SMBus
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import tkinter.simpledialog as simpledialog

# Configurable retry parameters
I2C_BUS_NUM = 16  # 16 Lattepanda with usb dongle, bus0 for jetom
I2C_MAX_RETRIES = 20
I2C_BASE_SLEEP = 0.002  # 2 ms initial backoff
I2C_MAX_SLEEP = 0.05    # cap backoff to 40 ms

# Variables
#bus = SMBus(16) # 16 Lattepanda with usb dongle, bus0 for jetom
dev_addr = 0x29
bus = None
bus_ok = False
motor_running = False
is_connected = False
conversion_factor = 91.01944445  # deg/s → counts/sec

def wake_motor():
    # 0x1C WAKE UP (0 data bytes)
    ok = i2c_write_block(dev_addr, 0x1C, [])
    if not ok:
        print("[I2C] Wake command failed")
    return ok

def is_calibrated() -> bool:
    # 0x02 CALIBRATION COMPLETE (1 byte: 0x00/0x01/0x02)
    data = i2c_read_block(dev_addr, 0x02, 1)
    if not data:
        return False
    return data[0] == 0x01

def wait_until_ready(timeout_ms=5000, poll_ms=50) -> bool:
    # Try wake, then poll calibration
    wake_motor()
    waited = 0
    while waited <= timeout_ms:
        if is_calibrated():
            return True
        time.sleep(poll_ms / 1000.0)
        waited += poll_ms
    print("[STATE] Calibration not complete within timeout")
    return False

def safe_bus_open():
    global bus, bus_ok
    if bus_ok and bus is not None:
        return
    try:
        if bus is not None:
            try:
                bus.close()
            except Exception:
                pass
        bus = SMBus(I2C_BUS_NUM)
        bus_ok = True
    except Exception as e:
        bus_ok = False
        print(f"[I2C] Failed to open bus {I2C_BUS_NUM}: {e}")

def i2c_should_retry(e):
    msg = str(e).lower()
    if getattr(e, 'errno', None) in (errno.EAGAIN, errno.EIO, errno.ETIMEDOUT, errno.ENXIO, errno.EREMOTEIO):
        return True
    transient_terms = ["input/output error", "remote i/o error", "no such device", "device or resource busy",
                       "timed out", "timeout", "busy"]
    return any(t in msg for t in transient_terms)

def i2c_write_block(address, command, data):
    global bus_ok, bus
    sleep = I2C_BASE_SLEEP
    for attempt in range(1, I2C_MAX_RETRIES + 1):
        try:
            if not bus_ok or bus is None:
                safe_bus_open()
            bus.write_i2c_block_data(address, command, data)
            return True
        except Exception as e:
            if attempt < I2C_MAX_RETRIES and i2c_should_retry(e):
                # print(f"[I2C] Write retry {attempt}/{I2C_MAX_RETRIES} cmd=0x{command:02X} err={e}")
                try:
                    if bus is not None:
                        try:
                            bus.close()
                        except Exception:
                            pass
                    bus_ok = False
                    time.sleep(sleep)
                    safe_bus_open()
                except Exception as open_e:
                    print(f"[I2C] Reopen bus failed: {open_e}")
                time.sleep(min(sleep, I2C_MAX_SLEEP))
                sleep = min(sleep * 2, I2C_MAX_SLEEP)
                continue
            print(f"[I2C] Write failed cmd=0x{command:02X} err={e}")
            return False
    return False

def i2c_read_block(address, command, length):
    global bus_ok, bus
    sleep = I2C_BASE_SLEEP
    for attempt in range(1, I2C_MAX_RETRIES + 1):
        try:
            if not bus_ok or bus is None:
                safe_bus_open()
            data = bus.read_i2c_block_data(address, command, length)
            if data is not None and len(data) == length:
                return data
            raise IOError(f"Short read: got {len(data) if data else 0}, want {length}")
        except Exception as e:
            if attempt < I2C_MAX_RETRIES and i2c_should_retry(e):
                print(f"[I2C] Read retry {attempt}/{I2C_MAX_RETRIES} cmd=0x{command:02X} err={e}")
                try:
                    if bus is not None:
                        try:
                            bus.close()
                        except Exception:
                            pass
                    bus_ok = False
                    time.sleep(sleep)
                    safe_bus_open()
                except Exception as open_e:
                    print(f"[I2C] Reopen bus failed: {open_e}")
                time.sleep(min(sleep, I2C_MAX_SLEEP))
                sleep = min(sleep * 2, I2C_MAX_SLEEP)
                continue
            print(f"[I2C] Read failed cmd=0x{command:02X} err={e}")
            return None
    return None



# Function to convert RPM to motor counts
def rpm_to_counts(rpm):
    # RPM → deg/s
    deg_per_sec = rpm * 6.0
    # deg/s → counts/sec
    motor_counts = int(deg_per_sec * conversion_factor)
    # If you ever allow negative RPM for direction, two's complement safeguard
    if motor_counts < 0:
        motor_counts = 0xFFFF + motor_counts + 1
    # Clamp to 16-bit range expected by the device
    motor_counts = max(0, min(0xFFFF, motor_counts))
    msb = (motor_counts >> 8) & 0xFF
    lsb = motor_counts & 0xFF
    return msb, lsb

def counts_to_rpm(counts):
    # counts/sec -> deg/sec -> RPM
    return (counts / conversion_factor) * 60.0


# Function to write data to the I2C device
def miranda_write(address, command, tx_data):
    ok = i2c_write_block(address, command, tx_data)
    if not ok:
        print(f"[I2C] Write failed to 0x{address:02X} cmd=0x{command:02X}")

# Function to stop the motor
def stop_motor():
    global motor_running
    if not motor_running:
        # Still send a stop, but don't spam UI
        ok = i2c_write_block(dev_addr, 0x07, [0x00, 0x00])
        if not ok:
            print("[I2C] Stop command write failed (motor already not running).")
        return

    ok = i2c_write_block(dev_addr, 0x07, [0x00, 0x00])
    if not ok:
        print("[I2C] Failed to send stop command")
        messagebox.showerror("I2C Error", "Could not write Stop to controller.")
        # Even if the write failed, reflect local state as stopped to keep UI coherent
    motor_running = False
    text_var.set("Motor stopped")

# Function to start the motor
def start_motor():
    global motor_running

    # Ensure motor is awake and calibrated
    if not wait_until_ready(timeout_ms=6000, poll_ms=100):
        messagebox.showerror("Motor Not Ready", "Motor not calibrated/awake. Check power-up and try again.")
        return

    rpm_str = rpm_entry.get().strip()
    if not rpm_str:
        rpm_val = simpledialog.askfloat("Input", "Enter RPM")
        if rpm_val is None:
            messagebox.showerror("Error", "Invalid Input")
            return
        rpm_entry.delete(0, 'end')
        rpm_entry.insert(0, str(rpm_val))
        rpm_str = str(rpm_val)

    try:
        rpm = float(rpm_str)
    except ValueError:
        messagebox.showerror("Error", "Invalid RPM")
        return

    # Clamp to practical range (optional)
    if rpm < 0:
        rpm = 0.0
    if rpm > 1000:
        rpm = 1000.0

    msb, lsb = rpm_to_counts(rpm)
    tx_data = [msb, lsb]
    print(f"[CMD] Start rpm={rpm} -> bytes: {tx_data}")

    ok = i2c_write_block(dev_addr, 0x07, tx_data)  # Travel at velocity
    if not ok:
        print("[I2C] Failed to send speed command")
        messagebox.showerror("I2C Error", "Could not write RPM to controller.")
        return

    # Optionally query movement status (0x03: -1/0/+1). Only for info.
    mv = i2c_read_block(dev_addr, 0x03, 1)
    if mv is not None and len(mv) == 1:
        print(f"[STATE] Is moving: {mv[0]}")
        motor_running = (mv[0] != 0)
    else:
        motor_running = True  # assume moving if write succeeded

    text_var.set(f"Motor {'running' if motor_running else 'stopped'}")
    

# Function to get PID gains via I2C
def get_pid_gains():
    try:
        kp_data = i2c_read_block(dev_addr, 0x0D, 2)
        ki_data = i2c_read_block(dev_addr, 0x0F, 2)
        kd_data = i2c_read_block(dev_addr, 0x11, 2)
        kc_data = i2c_read_block(dev_addr, 0x47, 2)
        if not all([kp_data, ki_data, kd_data, kc_data]):
            print("[I2C] Failed to read one or more PID registers")
            return
        
        kp = (kp_data[0] << 8) | kp_data[1]
        ki = (ki_data[0] << 8) | ki_data[1]
        kd = (kd_data[0] << 8) | kd_data[1]
        kc = (kc_data[0] << 8) | kc_data[1]
        
        kp = kp * 0.005  # Convert to float based on 0.005 resolution
        ki = ki * 0.005  # Convert to float based on 0.005 resolution
        kd = kd * 0.005  # Convert to float based on 0.005 resolution
        kc = kc * 0.005  # Convert to float based on 0.005 resolution
        
        
        kp_entry.delete(0, tk.END)
        ki_entry.delete(0, tk.END)
        kd_entry.delete(0, tk.END)
        kc_entry.delete(0, tk.END)
        
        kp_entry.insert(0, str(kp))
        ki_entry.insert(0, str(ki))
        kd_entry.insert(0, str(kd))
        kc_entry.insert(0, "{:.4f}".format(kc))

    except Exception as e:
        print(f"Error in getting gains: {e}")

# Function to set PID gains via I2C
def set_pid_gains():
    try:
        kp = float(kp_entry.get())
        ki = float(ki_entry.get())
        kd = float(kd_entry.get())
        kc = float(kc_entry.get())
        
        kp = int(kp / 0.005)  # Convert to fixed-point
        ki = int(ki / 0.005)  # Convert to fixed-point
        kd = int(kd / 0.005)  # Convert to fixed-point
        kc = int(kc / 0.005)  # Convert to fixed-point
        
        kp_msb = (kp >> 8) & 0xFF
        kp_lsb = kp & 0xFF
        
        ki_msb = (ki >> 8) & 0xFF
        ki_lsb = ki & 0xFF
        
        kd_msb = (kd >> 8) & 0xFF
        kd_lsb = kd & 0xFF

        kc_msb = (kc >> 8) & 0xFF
        kc_lsb = kc & 0xFF
        
        i2c_write_block(dev_addr, 0x0C, [kp_msb, kp_lsb])
        i2c_write_block(dev_addr, 0x0E, [ki_msb, ki_lsb])
        i2c_write_block(dev_addr, 0x10, [kd_msb, kd_lsb])
        i2c_write_block(dev_addr, 0x46, [kc_msb, kc_lsb])


    except ValueError:
        print("Please enter valid numbers for gains")
    except Exception as e:
        print(f"Error in setting gains: {e}")
           

# Global variable to track previous encoder position
prev_encoder_pos = None 

# Function to get current encoder position
def get_encoder_pos():
    pos_data = i2c_read_block(dev_addr, 0x1E, 2)
    if not pos_data:
        return None
    return (pos_data[0] << 8) | pos_data[1]

# Function to update encoder position display
def update_encoder_display():
    global prev_encoder_pos
    curr_pos = get_encoder_pos()
    if curr_pos is None:
        # Skip update on transient failure; schedule next poll
        root.after(50, update_encoder_display)
        return
    if prev_encoder_pos is None or abs(curr_pos - prev_encoder_pos) > 2:
        encoder_entry.delete(0, 'end')
        encoder_entry.insert(0, str(curr_pos))
        prev_encoder_pos = curr_pos
    root.after(50, update_encoder_display)


# Function to Write PID gains to FLASH
def write_pid_gains():
    i2c_write_block(dev_addr, 0x23, [0x00, 0x00])

# Initialize the Tkinter window
root = tk.Tk()
root.title("Miranda Tuning")

# Add label to show motor status
text_var = tk.StringVar()
text_var.set("Motor stopped")
motor_status_label = ttk.Label(root, textvariable=text_var)
motor_status_label.grid(row=1, columnspan=2, padx=10, pady=10)

# Add Encoder Position
ttk.Label(root, text="Encoder:").grid(row=2, column=0, pady=2)
encoder_entry = ttk.Entry(root, width=6, justify="center")
encoder_entry.grid(row=2, column=1, pady=2)

# Add PID gain controls
ttk.Label(root, text="Kp:").grid(row=3, column=0, pady=2)
kp_entry = ttk.Entry(root, width=6, justify="center")
kp_entry.grid(row=3, column=1, pady=2)

ttk.Label(root, text="Ki:").grid(row=4, column=0, pady=2)
ki_entry = ttk.Entry(root, width=6, justify="center")
ki_entry.grid(row=4, column=1, pady=2)

ttk.Label(root, text="Kd:").grid(row=5, column=0, pady=2)
kd_entry = ttk.Entry(root, width=6, justify="center")
kd_entry.grid(row=5, column=1, pady=2)

ttk.Label(root, text="Kc:").grid(row=6, column=0, pady=2)
kc_entry = ttk.Entry(root, width=6, justify="center")
kc_entry.grid(row=6, column=1, pady=2)

# Start display update
safe_bus_open()
update_encoder_display()

# Add RPM control
ttk.Label(root, text="RPM:").grid(row=7, column=0, pady=2)
rpm_entry = ttk.Entry(root, width=6, justify="center")
rpm_entry.grid(row=7, column=1, pady=2)

# Add buttons to start and stop motor
start_button = ttk.Button(root, text="Start Motor", command=start_motor)
start_button.grid(row=8, column=0, padx=10, pady=10)

stop_button = ttk.Button(root, text="Stop Motor", command=stop_motor)
stop_button.grid(row=8, column=1, padx=10, pady=10)

# Add buttons to set and get PID gains
set_button = ttk.Button(root, text="Set PID Gains", command=set_pid_gains)
set_button.grid(row=9, column=1, padx=10, pady=10)

get_button = ttk.Button(root, text="Get PID Gains", command=get_pid_gains)
get_button.grid(row=9, column=0, padx=10, pady=10)

# Add button write PID gains to FLASH
set_button = ttk.Button(root, text="FLASH PID Gains", command=write_pid_gains)
set_button.grid(row=10, columnspan=2, pady=10)

# Run the Tkinter event loop
root.mainloop()
root.after(50, update_encoder_display)

