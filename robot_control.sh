#!/bin/bash

# Raspberry Pi Information:
USER="sfr"
IP="sfr.local"

# Function to print help
function print_help
{
    echo "Usage: bash ./robot_control.sh command" 
    echo "Available Commands" 
    echo "=================="
    echo "- shutdown: Turn off the robot"
    echo "- sync: Sync your local files with the robot"
    echo "- FileName: Sync only one file"
    echo "- help: Print this menu"
    printf "\n"
}

# If fail we print a message
function handle_error {
        if [ "$?" -ne "0" ]
        then
            echo "Could not complete sync. Are you connected to the same network of the robot?";
            echo "Error message:";
            cat  /tmp/err;
        else
            echo "Done!";
        fi
}

# If no arguments are provided to the script
if [ ! $# -ge 1 ]
then
    echo "incorrect number of arguments: $0"
    print_help
    exit 1
fi

# Check hostname and if is on the rpi throw error message
if [ "$(hostname)" == $USER ]
then
    echo "You are on the robot. Please run this script from your computer."
    exit 1
fi

action=$1

case $action in
    "shutdown")
        echo "Shutting down $IP..."
        ssh $USER@$IP 'sudo shutdown -h now' 1> /dev/null 2> /tmp/err;
        handle_error
        ;;
    "sync")
        if [ "$2" == 'rosws' ]
        then
            echo "Synchronizing dev ROS2 workspace to robot ROS2 workspace"
            rsync -av -e ssh --exclude='.git*' /workspaces/msc-thesis/src $USER@$IP:/home/$USER/sfr_ros2_ws/ 1> /dev/null 2> /tmp/err;
            handle_error
        elif [ "$2" == 'util' ]
        then
            echo "Synchronizing util to robot"
            rsync -av -e ssh --exclude='.git*' /workspaces/msc-thesis/util/ $USER@$IP:/home/$USER/util 1> /dev/null 2> /tmp/err;
        elif [ "$#" -eq 3 ] # Synchronizes $2 to target path $3
        then
            echo "Synchronizing $2 to $3... This might take a while." # Synchronize one file
            rsync -av -e ssh --exclude='.git*' "$2" $USER@$IP:/home/$USER/"$3" 1> /dev/null 2> /tmp/err;
            handle_error
        elif [ "$#" -eq 2 ] # Synchronizes $2 to root directory
        then
            echo "Synchronizing $2... This might take a while." # Synchronize one file
            rsync -av -e ssh --exclude='.git*' "$2" $USER@$IP:/home/$USER 1> /dev/null 2> /tmp/err;
            handle_error
        else
            echo "Unable to synchronize. You must choose a command, file or a folder to sync!"
            echo "Available Commands" 
            echo "=================="
            echo "- rosws: Synchronize ROS2 workspace to the robot"
            echo "- util: Synchronize util to the root directory of the robot"
            echo "- \$2 to \$3 on robot"
            echo "- \$2 to the root directory of the robot"
            printf "\n"
        fi
        ;;
    "help" | "--help" | "-h")
        print_help
        ;;
    *)
        echo "Invalid action $1"
        print_help
        ;;
esac
