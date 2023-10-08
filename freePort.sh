#!/bin/bash

# Get the port number from the command-line argument (Use $1)
port_number=5000

# Find the process ID (PID) using the specified port
pid=$(lsof -t -i:$port_number)

# Check if the port is in use
if [[ -z $pid ]]; then
  echo "Port $port_number is free."
else
  # Terminate the process associated with the port
  kill $pid

  # Check if the process was successfully terminated
  if [[ $? -eq 0 ]]; then
    echo "Process with PID $pid using port $port_number has been terminated."
  else
    echo "Failed to terminate the process with PID $pid using port $port_number."
  fi
fi