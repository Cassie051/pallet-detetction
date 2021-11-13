#!/usr/bin/env bash

usage() { echo "Usage: $0 -w <window_name> [-p <pane_name>]" 1>&2; exit 1; }

# Check if tmux session is active (main roslaunch was opened in tmux)
if tmux info &> /dev/null; then 
  :
else
  echo "To use tmux terminal first open tmux session (eg. in shell: $ tmux)" 
  exit 1
fi 

# Get arguments
while getopts ":w:p:" option; do
  got_params=1
  case $option in
    w ) 
      window_name=$OPTARG
      ;;
    p ) 
      pane_name=$OPTARG
      ;;
    \? )
      usage
      exit 1
      ;;
    : ) 
      usage
      exit 1
      ;;
  esac
done
shift $((OPTIND-1))

# Check mandatory args
if [ ! "$window_name" ]
then
  usage
  exit 1
fi

# If pane_name is not defined then check if node name can be loadem from argument list
# if not set pane name same as window name
if [ ! "$pane_name" ]
then
  pane_name=`echo ${*} | sed -E 's/.*?__name:=//' | awk '{print $1}'` 
  if [ ! "$pane_name" ]; then
    pane_name=$window_name
  fi
fi

# Check if current session is named "ros"
# if not rename current name to "ros"
if tmux has -t ros 2>/dev/null; then
  tmux switch -t ros
else
  current_session_name=`tmux display -p '#{session_name}'`
  tmux rename-session -t "$current_session_name" ros
fi

# Check if another window is opening, if not, then lock
LOCK_DIR=/tmp/add_window.lock
i=0
max_iterations=30
until mkdir "$LOCK_DIR" &> /dev/null; do
  sleep 0.1
  ((++i))
  if ((i > max_iterations )); then
    max_period=$((max_iterations / 10))
    echo "$0: Cannot add new pane for ${max_period}s, because lock folder exists."
    echo "Removing $LOCK_DIR and letting open pane"
    rm -rf "$LOCK_DIR"
  fi
done

# Open new window with specified name
# then set pane name and roslaunch file
tmux switch -t ros \; new-window -n $window_name "${*} __ns:=$ROS_NAMESPACE; bash -i" \; \
select-pane -T ${pane_name} \; last-window \; 

# Remove lock
rm -rf "$LOCK_DIR"