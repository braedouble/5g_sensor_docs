# BRAEDEN COOKE MMRI 2025
# ETHERNET/5G DATA COLLECTION FROM ESP32/WT32

#import libraries
import socket
import csv
from datetime import datetime
import os
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import struct

# Global variables for data storage and synchronization
data_lock = threading.Lock()
delta_ts = deque(maxlen=10000)  
x_values = deque(maxlen=10000)
y_values = deque(maxlen=10000)
z_values = deque(maxlen=10000)
cumulative_time = 0  # To track running total time
start_time = None  # To track the absolute start time
time_values = deque(maxlen=10000)  # Stores cumulative time in seconds


# Control flags
running = False
client_socket = None
server_socket = None
animation = None

# Default settings
scale_to_g = True   
scale_csv_to_g = False  
log_to_csv = True   

scale_var = None
csv_scale_var = None
log_var = None


# Sensor configurations
SENSOR_CONFIGS = {
    'ADXL357': {
        'ranges': {
            '10g': 19.5 / 1000000,  # 9.76 µg/LSB for ±10g
            '20g': 39 / 1000000,    # 19.5 µg/LSB for ±20g
            '40g': 78 / 1000000,    # 39.1 µg/LSB for ±40g
        },
        'default_range': '10g',
        'ODR':['4000', '2000', '1000', '500', '250', '125', '62.5', '31.25', '15.625', '7.813', '3.906'], # ODR freq Hz 
        'default_ODR': '4000',
    },
    'ADXL345': {
        'ranges': {
            '2g': 3.9 / 1000,    # 3.9 mg/LSB for ±2g
            '4g': 3.9 / 1000,    # 3.9 mg/LSB for ±4g
            '8g': 3.9 / 1000,   # 3.9 mg/LSB for ±8g
            '16g': 3.9 / 1000,  # 3.9 mg/LSB for ±16g
        },
        'default_range': '2g',
        'ODR':['3200', '1600', '800', '400', '200', '100', '50', '25', '12.5', '6.25', '3.13', '1.56', '0.78', '0.39', '0.20', '0.10'], # LPF corner freq Hz (used to set ODR)
        'default_ODR': '3200',
    },
    'ICM42688': {
        'ranges': {
            '2g': 1 / 16384,    # 2048 LSB/g = 2048 LSB/1000mg = 1000/2048 mg/LSB  ±2g
            '4g': 1 / 8192,    # 4096 LSB/g for ±4g
            '8g': 1 / 4096,    # 8192 LSB/g for ±8g
            '16g': 1 / 2048,  # 16384 LSB/g for ±16g
        },
        'default_range': '16g', # sensor default range
        'ODR':['32000', '16000', '8000', '4000', '2000', '1000', '500', '200', '100', '50', '25', '12.5'], # LPF corner freq Hz (used to set ODR)
        'default_ODR': '1000', ## sensor default ODR
    }
}

selected_sensor = 'ADXL357'  # Default sensor
selected_range = SENSOR_CONFIGS['ADXL357']['default_range']  # Default range

# Function to start the server
# ***PORT MUST MATCH SETTING ON SENSOR***
def start_server(host='0.0.0.0', port=81):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f"Server listening on {host}:{port}")
    return server_socket


# For creating and managing CSV files
def get_desktop_path():
    """Return the path to the 'sensor_readings' folder on the desktop."""
    desktop_path = os.path.join(os.path.expanduser('~'), 'Desktop')
    sensor_folder = os.path.join(desktop_path, 'sensor_readings')
    
    # Create the folder if it doesn't exist
    if not os.path.exists(sensor_folder):
        os.makedirs(sensor_folder)
        print(f"Created folder: {sensor_folder}")
    
    return sensor_folder

def create_csv_file():
    global scale_csv_to_g
    sensor_folder = get_desktop_path()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"esp32_data_{timestamp}_{selected_sensor}_{selected_range}.csv"
    filepath = os.path.join(sensor_folder, filename)
    
    with open(filepath, 'w', newline='') as file:
        file.write(f"# Sensor: {selected_sensor}\n")
        file.write(f"# Range: {selected_range}\n")
        file.write(f"# Scale Factor: {SENSOR_CONFIGS[selected_sensor]['ranges'][selected_range]}\n")
        file.write(f"# Data Scaled?: {scale_csv_to_g}\n")
        file.write(f"# Sampling Frequency (Hz): {freq_dropdown.get()}\n")
        file.write(f"# Start Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        file.write(f"# Delta_T(us),X,Y,Z\n")  # Updated header
    
    print(f"Created CSV file: {filepath}")
    return filepath

# Function to update the plot
def update_plot(frame):
    global start_time
    if not running:
        return
        
    with data_lock:
        plt.cla()
        if time_values and len(time_values) > 0:
            plt.plot(time_values, x_values, label='X')
            plt.plot(time_values, y_values, label='Y')
            plt.plot(time_values, z_values, label='Z')
    
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (g)')
    if scale_to_g:
        scale_factor = SENSOR_CONFIGS[selected_sensor]['ranges'][selected_range]
        resolution = f"{scale_factor * 1e6:.2f} µg/LSB" if selected_sensor == 'ADXL357' else f"{scale_factor * 1e3:.1f} mg/LSB"
        plt.title(f'Live Accelerometer Data - {selected_sensor}\nRange: ±{selected_range}, Resolution: {resolution}')
    else:
        plt.title(f'Live Accelerometer Data - {selected_sensor}\nRange: ±{selected_range}, Unscaled')
    plt.legend()
    plt.grid(True)

# Function to handle data collection
def data_collection(client_socket, csv_filepath):
    global delta_ts, time_values, x_values, y_values, z_values, running, sample_count, start_time, cumulative_time
    sample_count = 0
    scale_factor = SENSOR_CONFIGS[selected_sensor]['ranges'][selected_range]
    cumulative_time = 0

    csv_file = open(csv_filepath, 'a', newline='') if csv_filepath and log_to_csv else None
    csv_writer = csv.writer(csv_file) if csv_file else None

    try:
        buffer = b""  # Byte buffer for binary data
        start_time = time.time()
        while running:
            try:
                data = client_socket.recv(4096)  # Receive bytes
                if not data:
                    print("Connection closed by ESP32")
                    break

                buffer += data
                # Process complete 16-byte samples (NOW 8 BYTES)
                while len(buffer) >= 8:
                #while len(buffer) >= 10:
                    # Unpack 4 integers (little-endian)
                    delta_t, x, y, z = struct.unpack('<Hhhh', buffer[:8])
                    # FOR uint16 deltaT, int16 accelData x3
                    #delta_t, x, y, z = struct.unpack('<Ihhh', buffer[:10])

                    buffer = buffer[8:]  # Remove processed bytes
                    # for new size its 8 bytes instead of 16 (HALVED)
                    #buffer = buffer[10:]  # Remove processed bytes

                    # Update cumulative time in microseconds, convert to seconds
                    cumulative_time += delta_t
                    current_time = cumulative_time / 1e6

                    # Log to CSV
                    if log_to_csv and csv_writer:
                        row = [
                            delta_t,
                            x * scale_factor if scale_csv_to_g else x,
                            y * scale_factor if scale_csv_to_g else y,
                            z * scale_factor if scale_csv_to_g else z
                        ]
                        csv_writer.writerow(row)

                    sample_count += 1

                    # Update plot data
                    with data_lock:
                        delta_ts.append(delta_t)
                        time_values.append(current_time)
                        x_values.append(x * scale_factor if scale_to_g else x)
                        y_values.append(y * scale_factor if scale_to_g else y)
                        z_values.append(z * scale_factor if scale_to_g else z)

                if log_to_csv and csv_file:
                    csv_file.flush()

            except socket.error as e:
                print(f"Socket error: {e}")
                running = False
                break
            except struct.error as e:
                print(f"Struct unpack error: {e}")
                continue  # Skip malformed data
    finally:
        if log_to_csv and csv_file:
            with open(csv_filepath, 'r') as file:
                lines = file.readlines()
            
            with open(csv_filepath, 'w') as file:
                for line in lines:
                    if line.startswith("# Delta_T(us),X,Y,Z"):
                        file.write(f"# End Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                    file.write(line)
            csv_file.close()

    print(f"Data collection stopped. Total samples: {sample_count}")
    return sample_count

# Functions to handle GUI events
def update_range_dropdown():
    range_values = list(SENSOR_CONFIGS[selected_sensor]['ranges'].keys())
    range_dropdown['values'] = range_values
    range_dropdown.set(SENSOR_CONFIGS[selected_sensor]['default_range'])

def on_range_change(event):
    global selected_range
    selected_range = range_dropdown.get()
    # Extract the number from the range string (e.g., '10g' -> '10')
    range_value = selected_range.replace('g', '')
    
    if client_socket:
        try:
            # Send SET_RANGE command to ESP32
            command = f"SET_RANGE{range_value}\n"
            client_socket.send(command.encode())
            print(f"Sent range setting: {command.strip()}")
        except Exception as e:
            print(f"Error sending range setting: {e}")
            messagebox.showerror("Connection Error", f"Failed to set range: {e}")
    else:
        print("No connection to ESP32")
        messagebox.showerror("Connection Error", "No active connection to ESP32")

def update_frequency_dropdown():
    freq_values = SENSOR_CONFIGS[selected_sensor]['ODR']
    freq_dropdown['values'] = freq_values
    freq_dropdown.set(SENSOR_CONFIGS[selected_sensor]['default_ODR'])

def on_frequency_change(event):
    selected_freq = freq_dropdown.get()
    if client_socket:
        try:
            command = f"SET_FREQUENCY{selected_freq}\n"
            client_socket.send(command.encode())
            print(f"Sent frequency setting: {command.strip()}")
        except Exception as e:
            print(f"Error sending frequency setting: {e}")
            messagebox.showerror("Connection Error", f"Failed to set frequency: {e}")
    else:
        print("No connection to ESP32")
        messagebox.showerror("Connection Error", "No active connection to ESP32")

def start_timed_collection(duration):
    try:
        duration = float(duration)
        if duration <= 0:
            messagebox.showerror("Error", "Duration must be positive")
            return
    except ValueError:
        messagebox.showerror("Error", "Invalid duration value")
        return
    
    if client_socket:
        try:
            toggle_collection(timed=True)
            
            def stop_after_duration():
                global cumulative_time
                duration_us = int(duration * 1e6)
                while running:
                    with data_lock:
                        if cumulative_time >= duration_us:
                            root.after(0, lambda: toggle_collection(timed=True))
                            break
                    time.sleep(0.01)

            timer_thread = threading.Thread(target=stop_after_duration)
            timer_thread.daemon = True
            timer_thread.start()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to start timed collection: {str(e)}")
    else:
        messagebox.showerror("Connection Error", "No active connection")

def reset_reader():
    global delta_ts, time_values, x_values, y_values, z_values, start_time
    with data_lock:
        delta_ts.clear()
        time_values.clear()
        x_values.clear()
        y_values.clear()
        z_values.clear()
        start_time = None
        cumulative_time = 0  # Explicitly reset cumulative_tim
    plt.cla()
    print("Reader reset.")

def create_timed_input_frame():
    """Create frame for timed input"""
    global timed_input_frame, duration_entry
    timed_input_frame = tk.Frame(control_frame)
    timed_input_frame.pack(side=tk.LEFT, padx=5)
    
    tk.Label(timed_input_frame, text="Duration (s):").pack(side=tk.LEFT)
    duration_entry = tk.Entry(timed_input_frame, width=6)
    duration_entry.pack(side=tk.LEFT, padx=2)
    duration_entry.insert(0, "10")  # Default value
    
    timed_start_btn = tk.Button(
        timed_input_frame,
        text="Start Timed",
        command=lambda: start_timed_collection(float(duration_entry.get())),
        bg="yellow"
    )
    timed_start_btn.pack(side=tk.LEFT, padx=2)

def toggle_collection(timed=False):
    global running, client_socket, animation, start_time
    
    if not running:
        running = True
        toggle_button.config(text="Stop", bg="red")
        status_label.config(text="Status: Running")
        range_dropdown.config(state='disabled')
        freq_dropdown.config(state='disabled')
        scale_check.config(state='disabled')
        log_check.config(state='disabled')
        csv_scale_check.config(state='disabled')
        
        if timed:
            timed_input_frame.pack_forget()
        else:
            timed_input_frame.pack_forget()
            
        reset_reader()
        
        if client_socket:
            try:
                client_socket.send("START_CONTINUOUS\n".encode())
                print("Sent START_CONTINUOUS command")
                
                csv_filepath = create_csv_file() if log_to_csv else None
                
                data_thread = threading.Thread(target=data_collection, args=(client_socket, csv_filepath))
                data_thread.start()
                
                status_label.config(text="Status: Collecting Data")
            except Exception as e:
                messagebox.showerror("Connection Error", str(e))
                running = False
                toggle_button.config(text="Start", bg="green")
                range_dropdown.config(state='readonly')
                freq_dropdown.config(state='readonly')
                status_label.config(text="Status: Connection Lost")
                timed_input_frame.pack(side=tk.LEFT, padx=5)
        else:
            messagebox.showerror("Connection Error", "No active connection")
            running = False
            toggle_button.config(text="Start", bg="green")
            range_dropdown.config(state='readonly')
            freq_dropdown.config(state='readonly')
            status_label.config(text="Status: No Connection")
            timed_input_frame.pack(side=tk.LEFT, padx=5)
    else:
        running = False
        toggle_button.config(text="Start", bg="green")
        status_label.config(text="Status: Stopped")
        range_dropdown.config(state='readonly')
        freq_dropdown.config(state='readonly')
        timed_input_frame.pack(side=tk.LEFT, padx=5)
        
        scale_check.config(state='normal')
        csv_scale_check.config(state='normal')
        log_check.config(state='normal')
        
        if client_socket:
            try:
                client_socket.send("STOP\n".encode())
                print("Sent STOP command")
            except Exception as e:
                print(f"Error sending stop message: {e}")
        
        if time_values:
            elapsed_time = max(time_values)
            print(f"Collection stopped. Total samples logged: {sample_count}")
            print(f"Elapsed time: {elapsed_time:.2f} seconds")
            print(f"Average sampling rate: {sample_count / elapsed_time:.2f} samples/second")

# Add toggle functions
def toggle_scaling():
    global scale_to_g, scale_var
    scale_to_g = scale_var.get()
    print(f"Scaling to g {'enabled' if scale_to_g else 'disabled'}")

def toggle_csv_scaling():
    global scale_csv_to_g, csv_scale_var
    scale_csv_to_g = csv_scale_var.get()
    print(f"CSV scaling {'enabled' if scale_csv_to_g else 'disabled'}")

def toggle_logging():
    global log_to_csv, log_var
    log_to_csv = log_var.get()
    print(f"CSV logging {'enabled' if log_to_csv else 'disabled'}")

def handle_sensor_info(info_str):
    global selected_sensor, selected_range
    try:
        # Parse the SENSOR_INFO message
        sensor_type, current_range, current_freq = info_str.split(',')
        current_range = f"{current_range}g"
        
        # Update sensor and range
        selected_sensor = sensor_type
        selected_range = current_range
        
        # Update sensor display label
        sensor_label.config(text=f"Sensor: {selected_sensor}")
        
        # Update range dropdown with available ranges and set current
        range_values = list(SENSOR_CONFIGS[selected_sensor]['ranges'].keys())
        range_dropdown['values'] = range_values
        range_dropdown.set(selected_range)
        
        # Update frequency dropdown and set to current
        freq_values = SENSOR_CONFIGS[selected_sensor]['ODR']
        freq_dropdown['values'] = freq_values
        # Find closest matching frequency
        freq_dropdown.set(min(freq_values, key=lambda x: abs(float(x) - float(current_freq))))
        
        print(f"Sensor info received: {selected_sensor}, Range: {selected_range}, Frequency: {current_freq}")
    except Exception as e:
        print(f"Error parsing sensor info: {e}")
    
def auto_connect():
    """Attempt to connect to ESP32 when the script starts"""
    global client_socket, server_socket
    try:
        server_socket = start_server()
        status_label.config(text="Status: Waiting for ESP32...")
        
        def wait_for_connection():
            global client_socket
            try:
                client_socket, address = server_socket.accept()
                print(f"Connected to {address}")
                
                # Wait for and process initial sensor info
                initial_data = client_socket.recv(1024).decode('utf-8')
                if 'SENSOR_INFO:' in initial_data:
                    sensor_info = initial_data.split('SENSOR_INFO:')[1].strip()
                    root.after(0, lambda: handle_sensor_info(sensor_info))
                    
                status_label.config(text=f"Status: Connected to {address[0]}")
            except Exception as e:
                print(f"Connection error: {e}")
                status_label.config(text="Status: Connection Failed")
        
        connection_thread = threading.Thread(target=wait_for_connection)
        connection_thread.start()
    except Exception as e:
        print(f"Server setup error: {e}")
        status_label.config(text="Status: Setup Error")

def main():
    global toggle_button, status_label, animation, sensor_label, range_dropdown, root, freq_dropdown, control_frame, scale_check, log_check, csv_scale_check, scale_var, csv_scale_var, log_var
    
    # Create main window
    root = tk.Tk()
    root.title("ESP32 Data Collection")
    
    # Create control frame
    control_frame = tk.Frame(root)
    control_frame.pack(side=tk.TOP, fill=tk.X)

    # Create sensor display label
    sensor_label = tk.Label(control_frame, text="Sensor: --")
    sensor_label.pack(side=tk.LEFT, padx=5)
    
    # Create range selection dropdown
    range_label = tk.Label(control_frame, text="Range:")
    range_label.pack(side=tk.LEFT, padx=5)
    
    range_dropdown = ttk.Combobox(control_frame, state='readonly')
    update_range_dropdown()  # Initialize range values
    range_dropdown.pack(side=tk.LEFT, padx=5)
    range_dropdown.bind('<<ComboboxSelected>>', on_range_change)

    # Create frequency selection dropdown
    freq_label = tk.Label(control_frame, text="Frequency (Hz):")
    freq_label.pack(side=tk.LEFT, padx=5)
    
    freq_dropdown = ttk.Combobox(control_frame, state='readonly')
    freq_dropdown.pack(side=tk.LEFT, padx=5)
    freq_dropdown.bind('<<ComboboxSelected>>', on_frequency_change)
    
    # Create status label
    status_label = tk.Label(control_frame, text="Status: Stopped")
    status_label.pack(side=tk.LEFT, padx=5)
    
    # Create toggle button
    toggle_button = tk.Button(control_frame, text="Start", command=toggle_collection, bg="green")
    toggle_button.pack(side=tk.LEFT, padx=5)
    
    # Create timed input frame
    create_timed_input_frame()

    scale_var = tk.BooleanVar(value=scale_to_g)
    csv_scale_var = tk.BooleanVar(value=scale_csv_to_g)
    log_var = tk.BooleanVar(value=log_to_csv)

    # Scaling checkbox
    scale_check = ttk.Checkbutton(
        control_frame,
        text="Scale to g",
        command=toggle_scaling,
        variable=scale_var
    )
    scale_check.pack(side=tk.LEFT, padx=5)

    # Add CSV scaling checkbox
    csv_scale_check = ttk.Checkbutton(
        control_frame,
        text="Scale CSV to g",
        command=toggle_csv_scaling,
        variable=csv_scale_var
    )
    csv_scale_check.pack(side=tk.LEFT, padx=5)

    # Logging checkbox
    log_check = ttk.Checkbutton(
        control_frame,
        text="Log to CSV",
        command=toggle_logging,
        variable=log_var
    )
    log_check.pack(side=tk.LEFT, padx=5)
    
    # Create figure and canvas
    fig = plt.figure(figsize=(10, 6))
    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.draw()
    canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
    
    # Start animation
    animation = FuncAnimation(fig, update_plot, interval=100)
    
    # Attempt auto-connect
    auto_connect()
    
    # Start GUI event loop
    root.mainloop()

if __name__ == "__main__":
    main()