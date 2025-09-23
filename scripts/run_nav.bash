#!/bin/bash
# Navigation control script for your Ackermann robot

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default values
MAP_FILE="/home/tom/carbot_ws/maps/maps.yaml"
WAYPOINT_FILE="/home/tom/carbot_ws/waypoints/waypoints.yaml"
AUTO_START=false

# Function to print colored messages
print_msg() {
    echo -e "${GREEN}[NAV]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

# Function to check if a ROS2 node is running
check_node() {
    ros2 node list | grep -q "$1"
    return $?
}

# Function to start navigation
start_navigation() {
    print_msg "Starting navigation stack..."
    
    # Check if already running
    if check_node "amcl"; then
        print_warn "Navigation appears to be already running"
        return 1
    fi
    
    # Launch navigation
    ros2 launch your_navigation_package complete_navigation.launch.py \
        map_file:="$MAP_FILE" \
        waypoint_file:="$WAYPOINT_FILE" \
        serial_port:="$SERIAL_PORT" \
        auto_start:="$AUTO_START" &
    
    # Wait for nodes to start
    sleep 5
    
    # Check if key nodes are running
    if check_node "amcl" && check_node "controller_server"; then
        print_msg "Navigation stack started successfully!"
        print_msg "Key nodes running:"
        ros2 node list | grep -E "(amcl|controller|planner|bt_navigator|waypoint)"
    else
        print_error "Failed to start navigation stack"
        return 1
    fi
}

# Function to stop navigation
stop_navigation() {
    print_msg "Stopping navigation..."
    
    # Send stop command to waypoint sender
    ros2 topic pub --once /stop_navigation std_msgs/msg/Bool "data: true"
    
    # Kill navigation launch
    pkill -f "complete_navigation.launch.py"
    
    sleep 2
    print_msg "Navigation stopped"
}

# Function to send navigation commands
send_nav_command() {
    case $1 in
        start)
            print_msg "Starting waypoint navigation..."
            ros2 topic pub --once /start_navigation std_msgs/msg/Bool "data: true"
            ;;
        stop)
            print_msg "Stopping waypoint navigation..."
            ros2 topic pub --once /stop_navigation std_msgs/msg/Bool "data: true"
            ;;
        pause)
            print_msg "Pausing navigation..."
            ros2 action cancel /navigate_to_pose
            ;;
        *)
            print_error "Unknown command: $1"
            ;;
    esac
}

# Function to monitor navigation status
monitor_status() {
    print_msg "Monitoring navigation status (Ctrl+C to stop)..."
    ros2 topic echo /navigation_status
}

# Function to check system
check_system() {
    print_msg "Checking system..."
    
    # Check serial port
    if [ -e "$SERIAL_PORT" ]; then
        print_msg "✓ Serial port $SERIAL_PORT exists"
    else
        print_error "✗ Serial port $SERIAL_PORT not found"
    fi
    
    # Check map file
    if [ -f "$MAP_FILE" ]; then
        print_msg "✓ Map file exists"
    else
        print_error "✗ Map file not found: $MAP_FILE"
    fi
    
    # Check waypoint file
    if [ -f "$WAYPOINT_FILE" ]; then
        print_msg "✓ Waypoint file exists"
        # Count waypoints
        WP_COUNT=$(grep -c "position:" "$WAYPOINT_FILE" || echo "0")
        print_msg "  Found $WP_COUNT waypoints"
    else
        print_error "✗ Waypoint file not found: $WAYPOINT_FILE"
    fi
    
    # Check if LIDAR is running
    if check_node "ldlidar_node"; then
        print_msg "✓ LIDAR node is running"
    else
        print_warn "⚠ LIDAR node not detected"
    fi
    
    # Check TF tree
    print_msg "Checking TF tree..."
    timeout 2 ros2 run tf2_tools view_frames --no-exe 2>/dev/null
    if [ -f "frames.pdf" ]; then
        print_msg "✓ TF tree generated (see frames.pdf)"
    fi
}

# Main menu
show_menu() {
    echo ""
    echo "======================================"
    echo "    Ackermann Robot Navigation Control"
    echo "======================================"
    echo "1) Start navigation stack"
    echo "2) Stop navigation stack"
    echo "3) Start waypoint following"
    echo "4) Stop waypoint following"
    echo "5) Monitor status"
    echo "6) Check system"
    echo "7) Set map file"
    echo "8) Set waypoint file"
    echo "9) Exit"
    echo ""
    echo -n "Select option: "
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --map)
            MAP_FILE="$2"
            shift 2
            ;;
        --waypoints)
            WAYPOINT_FILE="$2"
            shift 2
            ;;
        --port)
            SERIAL_PORT="$2"
            shift 2
            ;;
        --auto-start)
            AUTO_START=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Main loop
while true; do
    show_menu
    read -r choice
    
    case $choice in
        1)
            start_navigation
            ;;
        2)
            stop_navigation
            ;;
        3)
            send_nav_command start
            ;;
        4)
            send_nav_command stop
            ;;
        5)
            monitor_status
            ;;
        6)
            check_system
            ;;
        7)
            echo -n "Enter map file path: "
            read -r MAP_FILE
            ;;
        8)
            echo -n "Enter waypoint file path: "
            read -r WAYPOINT_FILE
            ;;
        9)
            print_msg "Exiting..."
            exit 0
            ;;
        *)
            print_error "Invalid option"
            ;;
    esac
done