#!/usr/bin/env python3
"""
Bobbi Control Backend Server

This server provides:
1. REST API for the web interface
2. Docker container management
3. Status monitoring for ROS nodes

Requirements:
- Flask
- Docker SDK for Python
"""

import os
import time
from datetime import datetime
import threading
import json
import subprocess
from flask import Flask, request, jsonify, send_from_directory, send_file
from flask_cors import CORS
import docker


app = Flask(__name__, static_folder='static')
CORS(app)  # Enable CORS for all routes

DATA_DIR = "/mnt/mcap"


DOCKER_IMAGE = "bobbiv2:latest"  

# Global state
system_state = "idle"  # idle, launching, ready, recording
current_mode = None
docker_client = docker.DockerClient(base_url='unix:///var/run/docker.sock')
print(docker_client.version())
container = None
record_process = None


# Docker container management
def launch_container(mode):
    global container, system_state, current_mode
    
    try:
        # Stop any existing container
        if container:
            container.stop()
            container.remove()
            container = None
        
        # Determine launch parameters based on selected mode
        mode_parameters = {
            "mode1": {"use_front_camera": "true", "use_front_lidar": "true"},
            "mode2": {"use_rear_camera": "false", "use_rear_lidar": "true"},
            "mode3": {"use_front_camera": "true", "use_rear_camera": "false"},
            "mode4": {"use_front_lidar": "true", "use_rear_lidar": "true"}, 
            "mode5": {"use_front_lidar": "true", "use_rear_lidar": "true" , "use_front_camera": "true", "use_rear_camera": "false"},  
        }
        
        mode_topics = {
            "mode1": ["fix","zed_multi/front_camera/imu","/zed_multi/front_camera/left/camera_info","/zed_multi/front_camera/left/image_rect_color/compressed","/zed_multi/front_camera/odom","/livox/imu_3WEDJ9H00100551" ,"/livox/lidar_3WEDJ9H00100551"],
            "mode2": ["/livox/imu_3WEDH7600115681", "/livox/lidar_3WEDH7600115681"],
            "mode3": ["/zed_multi/front_camera/imu"],    #Add rear camera 
            "mode4": ["/livox/imu_3WEDH7600115681", "/livox/lidar_3WEDH7600115681"], #Add front lidar
            "mode5": [],  
        }
        
        
        # Get parameters for the current mode
        params = mode_parameters.get(mode, {"use_rear_camera": "false", "use_rear_lidar": "false" , "use_front_camera": "false", "use_rear_camera": "false"})

        param_args = " ".join([f"{key}:={value}" for key, value in params.items()])
        
        # Start the container with the appropriate launch file
        container = docker_client.containers.run(
            DOCKER_IMAGE,
            command=f"ros2 launch master_launch master_launch.py {param_args}",
            runtime="nvidia",
            network="host",
            privileged=True,
            ipc_mode="host",
            pid_mode="host",
            environment={
                "DISPLAY": os.environ.get("DISPLAY", ""),
                "NVIDIA_DRIVER_CAPABILITIES": "all",
            },
            volumes={
                "/tmp": {"bind": "/tmp", "mode": "rw"},
                "/dev": {"bind": "/dev", "mode": "rw"},
                "/var/nvidia/nvcam/settings/": {"bind": "/var/nvidia/nvcam/settings/", "mode": "rw"},
                "/etc/systemd/system/zed_x_daemon.service": {
                    "bind": "/etc/systemd/system/zed_x_daemon.service",
                    "mode": "ro"
                },
                "/usr/local/zed/resources/": {"bind": "/usr/local/zed/resources/", "mode": "rw"},
                "/usr/local/zed/settings/": {"bind": "/usr/local/zed/settings/", "mode": "rw"},
                "/mnt/mcap": {"bind": "/root/data/", "mode": "rw"}
                

            },
            detach=True,
            remove=False,
            tty=True,
            stdin_open=True 
        )

        
        #print(f"Container launched for mode {mode} with launch file {launch_file}")
        current_mode = mode
        
        # Wait for ROS nodes to initialize
        time.sleep(15)  # Simple delay - in production, you'd check node status
        
        container.reload()
        if container.status != "running":
            print("Container exited early. Logs:")
            print(container.logs().decode())
            system_state = "idle"
            return False

        # Verify ROS topics
        exec_result = container.exec_run("bash -c 'source /opt/ros/humble/install/setup.bash && ros2 topic list'")
        topics_output = exec_result.output.decode()
        
        required_topics = mode_topics.get(mode, [])

        if not all(node in topics_output for node in required_topics):
            print("Missing required ROS topics.")
            print("Current topics:", topics_output)
            system_state = "idle"
            return False

        system_state = "ready"
        return True
        
    except Exception as e:
        print(f"Error launching container: {e}")
        system_state = "idle"
        return False

def stop_container():
    global container, system_state, current_mode
    
    # Stop recording first if it's running
    if system_state == "recording":
        stop_recording()
    
    if container:
        try:
            container.stop()
            container.remove()
            container = None
            print("Container stopped")
        except Exception as e:
            print(f"Error stopping container: {e}")
    
    system_state = "idle"
    current_mode = None

# Recording management

def start_recording():
    global record_process, system_state
    
    if system_state != "ready" or not container:
        print(f"Cannot start recording - system: {system_state}, container: {bool(container)}")
        return False
    
    try:
        timestamp = int(time.time())

        dt = datetime.fromtimestamp(timestamp)

        file_date = dt.strftime("%d_%m_%Y")
        
        mode_topics = {
            "mode1": ["/livox/imu_3WEDJ9H00100551", "/livox/lidar_3WEDJ9H00100551"],
            "mode2": ["/livox/imu_3WEDH7600115681", "/livox/lidar_3WEDH7600115681"],
            "mode3": ["/zed_multi/front_camera/imu", "/zed_multi/front_camera/rgb/image_rect_color/compressed"],
        }
        
        topics = mode_topics.get(current_mode, [])
        if not topics:
            print(f"No topics for mode {current_mode}")
            return False
        
        topics_str = " ".join(topics)
        
        cmd = f"bash -c 'cd /root/data && source /opt/ros/humble/install/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 bag record -o bag_{file_date} {topics_str}'"

        print(f"DEBUG: Command: {cmd}")
        # Start recording
        result = container.exec_run(cmd, detach=True)
        
        
        record_process = result
        
        # Quick check - just see if process started
        time.sleep(2)

        check = container.exec_run("pgrep -f 'ros2 bag record'")
        print(f"DEBUG: Process check output: '{check.output.decode()}'")
                
        if check.output.decode().strip():
            system_state = "recording"
            print(f"Recording started - mode {current_mode}")
            return True
        else:
            print("Recording failed to start")
            return False
            
    except Exception as e:
        print(f"Error: {e}")
        return False

def stop_recording():
    global record_process, system_state
    if system_state != "recording":
        print(f"Cannot stop recording - system is in {system_state} state")
        return False
    
    try:
        # Find and kill the ros2 bag process
        kill_result = container.exec_run("bash -c 'pkill -f \"ros2 bag record\"'")
        print(f"Kill command result: {kill_result.exit_code}")
        
        # Wait a moment and verify it's stopped
        time.sleep(2)
        check = container.exec_run("bash -c 'ps aux | grep \"ros2 bag\" | grep -v grep'")
        
        if not check.output.decode().strip():
            print("Recording stopped")
            system_state = "ready"
            return True
        else:
            print("Failed to stop recording")
            return False
            
    except Exception as e:
        print(f"Error stopping recording: {e}")
        return False

# Check ROS nodes status
def check_nodes_status():
    if not container:
        return {"status": "not_running", "nodes": []}
    
    try:
        # Get list of running ROS nodes using ros2 node list
        result = container.exec_run("bash -c 'source /opt/ros/humble/setup.bash && ros2 node list'")
        
        if result.exit_code == 0:
            nodes = result.output.decode().strip().split('\n')
            # Filter out empty lines
            nodes = [node for node in nodes if node.strip()]
            return {"status": "running", "nodes": nodes}
        else:
            return {"status": "error", "message": result.output.decode()}
            
    except Exception as e:
        print(f"Error checking nodes: {e}")
        return {"status": "error", "message": str(e)}
    
def get_file_info(filepath):
    """Get file information including size and modification time"""
    try:
        stat = os.stat(filepath)
        return {
            'size': stat.st_size,
            'modified': stat.st_mtime,
            'is_dir': os.path.isdir(filepath)
        }
    except Exception as e:
        print(f"Error getting file info for {filepath}: {e}")
        return None

def format_file_size(size_bytes):
    """Format file size in human readable format"""
    if size_bytes == 0:
        return "0 B"
    
    size_names = ["B", "KB", "MB", "GB", "TB"]
    i = 0
    while size_bytes >= 1024 and i < len(size_names) - 1:
        size_bytes /= 1024.0
        i += 1
    
    return f"{size_bytes:.1f} {size_names[i]}"

def scan_directory(directory_path):
    """Recursively scan directory and return file structure"""
    items = []
    
    try:
        if not os.path.exists(directory_path):
            return items
            
        for item in os.listdir(directory_path):
            item_path = os.path.join(directory_path, item)
            info = get_file_info(item_path)
            
            if info:
                item_data = {
                    'name': item,
                    'path': item_path,
                    'size': info['size'],
                    'size_formatted': format_file_size(info['size']),
                    'modified': info['modified'],
                    'is_directory': info['is_dir']
                }
                
                # If it's a directory, scan its contents
                if info['is_dir']:
                    item_data['children'] = scan_directory(item_path)
                    # Calculate total size for directories
                    total_size = sum_directory_size(item_path)
                    item_data['size'] = total_size
                    item_data['size_formatted'] = format_file_size(total_size)
                
                items.append(item_data)
                
    except Exception as e:
        print(f"Error scanning directory {directory_path}: {e}")
    
    return sorted(items, key=lambda x: (not x['is_directory'], x['name'].lower()))

def sum_directory_size(directory_path):
    """Calculate total size of a directory"""
    total_size = 0
    
    try:
        for dirpath, dirnames, filenames in os.walk(directory_path):
            for filename in filenames:
                filepath = os.path.join(dirpath, filename)
                if os.path.exists(filepath):
                    total_size += os.path.getsize(filepath)
    except Exception as e:
        print(f"Error calculating directory size for {directory_path}: {e}")
    
    return total_size

# REST API endpoints
@app.route('/')
def index():
    return send_from_directory(app.static_folder, 'index.html')

@app.route('/files')
def files_page():
    return send_from_directory(app.static_folder, 'files.html')

@app.route('/api/files')
def list_files():
    """List all files in the data directory"""
    try:
        files = scan_directory(DATA_DIR)
        return jsonify({
            "success": True,
            "files": files,
            "base_path": DATA_DIR
        })
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@app.route('/api/files/download/<path:filename>')
def download_file(filename):
    """Download a specific file"""
    try:
        file_path = os.path.join(DATA_DIR, filename)
        
        # Security check - ensure file is within DATA_DIR
        if not os.path.abspath(file_path).startswith(os.path.abspath(DATA_DIR)):
            return jsonify({"error": "Access denied"}), 403
            
        if not os.path.exists(file_path):
            return jsonify({"error": "File not found"}), 404
            
        if os.path.isdir(file_path):
            return jsonify({"error": "Cannot download directory"}), 400
            
        return send_file(file_path, as_attachment=True)
        
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/api/files/delete', methods=['POST'])
def delete_files():
    """Delete selected files"""
    try:
        data = request.get_json()
        if not data or 'files' not in data:
            return jsonify({"error": "No files specified"}), 400
            
        deleted_files = []
        errors = []
        
        for file_path in data['files']:
            full_path = os.path.join(DATA_DIR, file_path)
            
            # Security check
            if not os.path.abspath(full_path).startswith(os.path.abspath(DATA_DIR)):
                errors.append(f"Access denied: {file_path}")
                continue
                
            try:
                if os.path.exists(full_path):
                    if os.path.isdir(full_path):
                        import shutil
                        shutil.rmtree(full_path)
                    else:
                        os.remove(full_path)
                    deleted_files.append(file_path)
                else:
                    errors.append(f"File not found: {file_path}")
            except Exception as e:
                errors.append(f"Error deleting {file_path}: {str(e)}")
        
        return jsonify({
            "success": True,
            "deleted": deleted_files,
            "errors": errors
        })
        
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/api/status')
def get_status():
    nodes_status = check_nodes_status() if system_state != "idle" else {"status": "not_running", "nodes": []}
    
    return jsonify({
        "system_state": system_state,
        "current_mode": current_mode,
        "docker_running": container is not None,
        "ros_nodes": nodes_status,
        "recording": system_state == "recording"
    })

@app.route('/api/launch', methods=['POST'])
def launch_mode():
    if not request.json or 'mode' not in request.json:
        return jsonify({"success": False, "error": "No mode specified"}), 400
    
    mode = request.json['mode']
    success = launch_container(mode)
    
    return jsonify({
        "success": success,
        "mode": mode,
        "state": system_state
    })

@app.route('/api/stop', methods=['POST'])
def stop_system():
    stop_container()
    
    return jsonify({
        "success": True,
        "state": system_state
    })

@app.route('/api/start-recording', methods=['POST'])
def start_recording_endpoint():
    success = start_recording()
    
    return jsonify({
        "success": success,
        "state": system_state,
        "mode": current_mode
    })

@app.route('/api/stop-recording', methods=['POST'])
def stop_recording_endpoint():
    success = stop_recording()
    
    return jsonify({
        "success": success,
        "state": system_state,
        "mode": current_mode
    })

# Keep the original toggle endpoint for backward compatibility
@app.route('/api/record', methods=['POST'])
def toggle_recording():
    if system_state == "ready":
        success = start_recording()
        return jsonify({
            "success": success,
            "state": system_state,
            "action": "started"
        })
    elif system_state == "recording":
        success = stop_recording()
        return jsonify({
            "success": success,
            "state": system_state,
            "action": "stopped"
        })
    else:
        return jsonify({
            "success": False,
            "error": f"Cannot toggle recording in {system_state} state"
        }), 400

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, debug=False)
    
