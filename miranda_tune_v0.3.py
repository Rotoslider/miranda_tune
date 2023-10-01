from smbus2 import SMBus
import tkinter as tk
from tkinter import ttk

# Variables
dev_addr = 0x29  # Device address
bus = SMBus(0)
motor_running = False
is_connected = False
conversion_factor = 91.01944444444445  # conversion factor from degrees/sec to motor counts/sec

# Function to convert RPM to motor counts
def rpm_to_counts(rpm):
    # Convert RPM to deg/sec
    deg_per_sec = rpm * 6
    
    # Calculate motor counts based on the conversion factor
    motor_counts = int(deg_per_sec * conversion_factor)
    
    # Handle 16-bit signed integer
    if motor_counts < 0:
        motor_counts = 0xFFFF + motor_counts + 1
    
    # Calculate MSB and LSB
    msb = (motor_counts >> 8) & 0xFF
    lsb = motor_counts & 0xFF
    
    return msb, lsb

# Function to convert motor counts/sec to RPM
def counts_to_rpm(counts):
    return (motor_counts / 91.01944444444445) * 60


# Function to write data to the I2C device
def miranda_write(address, command, tx_data):
    bus.write_i2c_block_data(address, command, tx_data)

# Function to stop the motor
def stop_motor():
    global motor_running
    if motor_running:
        motor_running = False
        text_var.set("Motor stopped")
        tx_data = [0x00, 0x00]  # 0 RPM
        miranda_write(dev_addr, 0x07, tx_data)

# Function to start the motor
def start_motor():
    global motor_running
    if not motor_running:
        motor_running = True
        text_var.set("Motor running")
        try:
            rpm = float(rpm_entry.get())
            msb, lsb = rpm_to_counts(rpm)
            
            tx_data = [msb, lsb]
            miranda_write(dev_addr, 0x07, tx_data)
        except ValueError:
            print("Please enter a valid RPM value")

# Function to toggle I2C connection
def toggle_i2c_connection():
    global is_connected
    if is_connected:
        is_connected = False
        connect_button.config(text="Connect")
        status_label.config(text="Disconnected", foreground="red")
    else:
        is_connected = True
        connect_button.config(text="Disconnect")
        status_label.config(text="Connected", foreground="green")

# Function to get PID gains via I2C
def get_pid_gains():
    try:
        kp_data = bus.read_i2c_block_data(dev_addr, 0x0D, 2)
        ki_data = bus.read_i2c_block_data(dev_addr, 0x0F, 2)
        kd_data = bus.read_i2c_block_data(dev_addr, 0x11, 2)
        kc_data = bus.read_i2c_block_data(dev_addr, 0x47, 2)
        
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
        kc_entry.insert(0, str(kc))

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
        
        bus.write_i2c_block_data(dev_addr, 0x0C, [kp_msb, kp_lsb])
        bus.write_i2c_block_data(dev_addr, 0x0E, [ki_msb, ki_lsb])
        bus.write_i2c_block_data(dev_addr, 0x10, [kd_msb, kd_lsb])
        bus.write_i2c_block_data(dev_addr, 0x46, [kc_msb, kc_lsb])
        #bus.write_i2c_block_data(dev_addr, 0x23, [0, 0])


    except ValueError:
        print("Please enter valid numbers for gains")
    except Exception as e:
        print(f"Error in setting gains: {e}")

# Function to Write PID gains to FLASH
def write_pid_gains():
        bus.write_i2c_block_data(dev_addr, 0x23, [0, 0])

# Initialize the Tkinter window
root = tk.Tk()
root.title("Miranda Tuning")

# Add Connect/Disconnect button
connect_button = ttk.Button(root, text="Connect", command=toggle_i2c_connection)
connect_button.grid(row=0, column=0, padx=10, pady=10)

# Add status label
status_label = ttk.Label(root, text="Disconnected", foreground="red")
status_label.grid(row=0, column=1, padx=10, pady=10)

# Add label to show motor status
text_var = tk.StringVar()
text_var.set("Motor stopped")
motor_status_label = ttk.Label(root, textvariable=text_var)
motor_status_label.grid(row=1, columnspan=2)

# Add PID gain controls
ttk.Label(root, text="Kp:").grid(row=2, column=0)
kp_entry = ttk.Entry(root, width=10)
kp_entry.grid(row=2, column=1)

ttk.Label(root, text="Ki:").grid(row=3, column=0)
ki_entry = ttk.Entry(root, width=10)
ki_entry.grid(row=3, column=1)

ttk.Label(root, text="Kd:").grid(row=4, column=0)
kd_entry = ttk.Entry(root, width=10)
kd_entry.grid(row=4, column=1)

ttk.Label(root, text="Kc:").grid(row=5, column=0)
kc_entry = ttk.Entry(root, width=10)
kc_entry.grid(row=5, column=1)


# Add RPM control
ttk.Label(root, text="RPM:").grid(row=6, column=0)
rpm_entry = ttk.Entry(root, width=10)
rpm_entry.grid(row=6, column=1)

# Add buttons to start and stop motor
start_button = ttk.Button(root, text="Start Motor", command=start_motor)
start_button.grid(row=7, column=0, padx=10, pady=10)

stop_button = ttk.Button(root, text="Stop Motor", command=stop_motor)
stop_button.grid(row=7, column=1, padx=10, pady=10)

# Add buttons to set and get PID gains
set_button = ttk.Button(root, text="Set PID Gains", command=set_pid_gains)
set_button.grid(row=8, column=1, padx=10, pady=10)

get_button = ttk.Button(root, text="Get PID Gains", command=get_pid_gains)
get_button.grid(row=8, column=0, padx=10, pady=10)

# Add button write PID gains to FLASH
set_button = ttk.Button(root, text="FLASH PID Gains", command=write_pid_gains)
set_button.grid(row=9, columnspan=2, pady=10)

# Run the Tkinter event loop
root.mainloop()

