#!/bin/bash

usage() {
    echo "  options:"
    echo "      -p: platorm. Default: ms. Choices:"
    echo "        ms: multirotor simulator"
    echo "        gz: gazebo"
    echo "      -b: behaviors. Default: False."
    echo "          If True, enables behavior-based control."
}

# Initialize variables with default values
platform="ms"
behaviors="false"

# Arg parser
while getopts "p:b" opt; do
  case ${opt} in
    p )
      platform="${OPTARG}"
      ;;
    b )
      behaviors="true"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[wrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

# Launch aerostack2
source drone_course_ws/install/setup.bash
eval "tmuxinator start -n drone -p config/tmuxinator.yaml platform=$platform behaviors=$behaviors"
