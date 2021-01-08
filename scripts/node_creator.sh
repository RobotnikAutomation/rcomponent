#!/usr/bin/env bash

# USEFUL FORMAT PARAMETERS

bold="\e[1m" 
italic="\e[3m"
underline="\e[4m"
reset="\e[0m"

black="\e[1;30m"
blue="\e[1;34m"
cyan="\e[1;36m"
green="\e[1;32m"
orange="\e[1;33m"
purple="\e[1;35m"
red="\e[1;31m"
white="\e[1;37m"
yellow="\e[1;33m"

# SCRIPT INTRODUCTION

clear
echo -e "\n${bold}Welcome to the node creator assistant."

echo -e -n "\n${reset}The goal of this script is to simplify the creation of new "
echo -e    "rcomponent-based nodes."

# LOAD PARAMETERS

# Load destination path
src_path=`echo $CMAKE_PREFIX_PATH | awk '{split($1, a, "/devel:"); print a[1];}'`"/src"
echo -e -n "\n${reset}Destination folder of the node. It is required to start with a slash "
echo -e    "[${bold}${italic}${src_path}${reset} by default]: "
read dest_path
case $dest_path in
/*)
    ;;
*)
    dest_path=$src_path
    ;;
esac

# Load rc_package name
echo -e -n "\n${reset}Name of the new package using snake_case (e.g. "
echo -e    "${italic}odometry_calibration${reset}): "
read rc_package
if [ -z "$rc_package" ]
then
    echo -e "${bold}You have skipped a required argument. Stopping the script."
    exit 1
fi

# Load RCNode name
echo -e -n "\n${reset}Name of the main class using PascalCase (e.g. "
echo -e    "${italic}OdomCalibration${reset}): "
read RCNode
if [ -z "$RCNode" ]
then
    echo -e "${bold}You have skipped a required argument. Stopping the script."
    exit 1
fi

# Load rc_node name
echo -e -n "\n${reset}Name of the main class using snake_case (e.g. "
echo -e    "${italic}odom_calibration${reset}): "
read rc_node
if [ -z "$rc_node" ]
then
    echo -e "${bold}You have skipped a required argument. Stopping the script."
    exit 1
fi

# Load RC_NODE name
RC_NODE=${rc_node^^}

# Load author name
echo -e -n "\n${reset}Author name (e.g. "
echo -e    "${italic}José Gómez${reset}): "
read authorName

# Load author email
echo -e -n "\n${reset}Author email (e.g. "
echo -e    "${italic}jgomez@robotnik.es${reset}): "
read authorEmail

# Load node description
echo -e -n "\n${reset}Description of the node (not required, but recommended): "
read brief

# CREATE PACKAGE
TODO