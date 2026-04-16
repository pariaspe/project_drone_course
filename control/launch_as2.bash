#!/bin/bash

usage() {
    echo "  options:"
    echo "      -p: platorm. Default: ms. Choices:"
    echo "        ms: multirotor simulator"
    echo "        gz: gazebo"
    echo "      -b: behaviors. Default: False."
    echo "          If True, enables behavior-based control."
    echo "      -c: gate config file. Default: config/gates_config.yaml"
    echo ""
}

# Initialize variables with default values
platform="ms"
behaviors="false"
gates_config="config/gates_config.yaml"

# Arg parser
while getopts "p:bc:" opt; do
  case ${opt} in
    p )
      platform="${OPTARG}"
      ;;
    b )
      behaviors="true"
      ;;
    c )
      gates_config="${OPTARG}"
      ;;
    \? )
      echo "Invalid option: -${OPTARG}" >&2
      usage
      exit 1
      ;;
    : )
      echo "Option -${OPTARG} requires an argument." >&2
      usage
      exit 1
      ;;
  esac
done

# Launch aerostack2
source drone_course_ws/install/setup.bash
eval "tmuxinator start -n drone -p config/tmuxinator.yaml platform=$platform behaviors=$behaviors gates_config=$gates_config"
