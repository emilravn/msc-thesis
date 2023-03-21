#!/bin/bash

# Set this:
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
    printf '%s'
}

function handle_error {
     # If fail we print a message
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

# Check hostname and if is "raspberrypi" throw error message
if [ `hostname` == $USER ]
then
    echo "You are on the robot. Please run this script from your computer."
    exit 1
fi

action=$1

# rsync -a pi@dreamdestroyer.local:/home/pi/robot_tag /c/Code/ITU/adv-robotics/exam-project

case $action in
    "shutdown")
        echo "Shutting down SFR..."
        ssh $USER@$IP 'sudo shutdown -h now' 1> /dev/null 2> /tmp/err;
        handle_error
        ;;
    "sync")
        if [ $# -ne 2 ] # Synchronizes everything
        then
            echo "Synchronizing... This might take a while."
            rsync -av -e ssh --exclude='.git*' --exclude='*pics*' --exclude='*sokoban*' `pwd`/* $USER@$IP:/home/pi 1> /dev/null 2> /tmp/err;
            handle_error
        else
            echo "Synchronizing $2... This might take a while." # Synchronize one file
            rsync -av -e ssh --exclude='.git*' $2 $USER@$IP:/home/pi 1> /dev/null 2> /tmp/err;
            handle_error
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
