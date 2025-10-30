import os
import tkinter as tk
from tkinter.filedialog import askopenfilename
import open3d as o3d
import numpy as np
import csv
import math
import subprocess  # For launching the scan.py script

# Helper Functions
def convert_to_cartesian(angle, distance, rotation):
    angle_rad = math.radians(angle)
    y = distance * math.sin(angle_rad)
    z = distance * math.cos(angle_rad)
    rotation_rad = math.radians(rotation)
    x = z * math.sin(rotation_rad)
    z = z * math.cos(rotation_rad)
    x = -x  # Negate x to match the correct orientation
    return x, y, z

def distance_to_color(distance, min_distance, max_distance):
    normalized = (distance - min_distance) / (max_distance - min_distance)
    normalized = max(0, min(1, normalized))
    r = int(255 * normalized)
    g = int(255 * (1 - abs(normalized - 0.5) * 2))
    b = int(255 * (1 - normalized))
    return r, g, b

def generate_ply(points, filename):
    with open(filename, 'w') as ply_file:
        ply_file.write("ply\n")
        ply_file.write("format ascii 1.0\n")
        ply_file.write(f"element vertex {len(points)}\n")
        ply_file.write("property float x\n")
        ply_file.write("property float y\n")
        ply_file.write("property float z\n")
        ply_file.write("property uchar red\n")
        ply_file.write("property uchar green\n")
        ply_file.write("property uchar blue\n")
        ply_file.write("end_header\n")
        for point in points:
            x, y, z, r, g, b = point
            ply_file.write(f"{x} {y} {z} {r} {g} {b}\n")

def load_csv_data(filepath):
    points = []
    distances = []

    with open(filepath, newline='') as csvfile:
        csvreader = csv.reader(csvfile)
        next(csvreader)  # Skip header
        for row in csvreader:
            try:
                angle = float(row[1])
                distance = float(row[2])
                rotation = float(row[3])
                x, y, z = convert_to_cartesian(angle, distance, rotation)
                distances.append(distance)
                points.append((x, y, z))
            except ValueError as e:
                print(f"Skipping row due to error: {e}")

    if not points:
        raise ValueError("No valid points found in the CSV file.")

    min_distance = min(distances)
    max_distance = max(distances)
    colored_points = [(x, y, z, *distance_to_color(distance, min_distance, max_distance)) for (x, y, z), distance in zip(points, distances)]
    return colored_points, points

def process_and_visualize(csv_file_path, vis=None, log_message=None):
    try:
        # Load data
        if log_message:
            log_message("Loading CSV data...")
        colored_points, _ = load_csv_data(csv_file_path)
        if log_message:
            log_message(f"Loaded {len(colored_points)} points from CSV.")

        # Set the output folder to './PLY', ensuring it's next to the CSV folder
        csv_folder = os.path.dirname(csv_file_path)  # Directory containing the CSV file
        ply_folder = os.path.join(os.path.dirname(csv_folder), 'PLY')  # PLY folder is in the parent directory

        os.makedirs(ply_folder, exist_ok=True)  # Ensure the PLY folder exists
        if log_message:
            log_message(f"Ensured the PLY folder exists: {ply_folder}")

        # Generate the output file path in the PLY folder
        csv_filename = os.path.splitext(os.path.basename(csv_file_path))[0]
        ply_filename = os.path.join(ply_folder, f"{csv_filename}.ply")
        
        # Generate the PLY file
        if log_message:
            log_message(f"Generating PLY file: {ply_filename}")
        generate_ply(colored_points, ply_filename)
        if log_message:
            log_message(f"Generated PLY file: {ply_filename}")

        # Load and adjust point cloud
        point_cloud = o3d.io.read_point_cloud(ply_filename)
        if not point_cloud.has_points():
            if log_message:
                log_message("Error: The PLY file contains no points.")
            return

        # Apply smoothing
        if log_message:
            log_message("Applying voxel downsampling for smoothing...")
        voxel_size = 0.03
        point_cloud_smoothed = point_cloud.voxel_down_sample(voxel_size)

        # Enhance colors for brightness and contrast
        colors = np.asarray(point_cloud_smoothed.colors)
        colors = np.clip(colors ** 1.00, 0, 1)  # Increase contrast dynamically
        point_cloud_smoothed.colors = o3d.utility.Vector3dVector(colors)

        # Visualization
        if vis is None:
            vis = o3d.visualization.VisualizerWithKeyCallback()
            vis.create_window()  # Let Open3D window size be default

        # Toggle background color with 'B'
        def toggle_background_color(vis):
            opt = vis.get_render_option()
            if np.allclose(opt.background_color, [0, 0, 0]):
                opt.background_color = np.array([1, 1, 1])  # Switch to white
            else:
                opt.background_color = np.array([0, 0, 0])  # Switch to black

        vis.register_key_callback(ord("B"), toggle_background_color)

        # Customize render options
        render_option = vis.get_render_option()
        render_option.background_color = np.array([0, 0, 0])  # Default to black
        render_option.point_size = 2.25  # Adjust point size
        render_option.light_on = True  # Ensure lighting is enabled
        render_option.mesh_show_back_face = True  # Show back faces of points

        # Add geometry
        vis.add_geometry(point_cloud_smoothed)
        vis.run()
        vis.destroy_window()

    except Exception as e:
        if log_message:
            log_message(f"Error during processing: {e}")
        print(f"Error during processing: {e}")

# GUI Functions
def main_gui():
    def log_message(message):
        text_box.insert(tk.END, message + "\n")
        text_box.see(tk.END)

    def open_csv():
        log_message("Opening file dialog to select a new CSV file...")
        csv_file_path = askopenfilename(title="Select CSV File", filetypes=[("CSV files", "*.csv")])
        if csv_file_path:
            log_message(f"Selected file: {csv_file_path}")
            try:
                process_and_visualize(csv_file_path, log_message=log_message)
            except Exception as e:
                log_message(f"Error during visualization: {e}")

    root = tk.Tk()
    root.title("Point Cloud Visualizer")
    root.geometry('600x400')

    open_button = tk.Button(root, text="Open CSV", command=open_csv)
    open_button.pack(pady=10)

    frame = tk.Frame(root)
    frame.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)

    canvas = tk.Canvas(frame)
    canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    scrollbar = tk.Scrollbar(frame, orient=tk.VERTICAL, command=canvas.yview)
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
    canvas.configure(yscrollcommand=scrollbar.set)

    text_box = tk.Text(canvas, wrap=tk.WORD)
    text_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    canvas.create_window((0, 0), window=text_box, anchor=tk.NW)

    def on_frame_configure(event):
        canvas.configure(scrollregion=canvas.bbox("all"))

    text_box.bind("<Configure>", on_frame_configure)
    root.mainloop()

# Main Entry Point
if __name__ == "__main__":
    main_gui()
