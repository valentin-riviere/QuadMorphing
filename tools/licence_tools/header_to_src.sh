#!/bin/bash

usage="$(basename "$0") [-h] header_path dir_src... -- program to put licence header to src/headers files

where:
	-h	Show this help text
	header_path	Path of the licence header
	dir_src	Directory where sources/headers are"

while getopts ':h' option; do
  case "$option" in
    h) echo "$usage"
       exit
       ;;
  esac
done
shift "$((OPTIND - 1))"	# Shift arguments

if [ $# -lt 2 ]; then
	printf "Must have at least 2 arguments\n"	
	echo "$usage"
	exit
fi

# Save header path and shift arguments
header=$1
shift

# For every specify directory
for d in $*; do
	# For every file
	echo $d
	for f in $(find $d -name '*.cpp' -or -name '*.c' -or -name '*.h' -or -name '*.hpp'); do
		echo $f
		cat $header $f > $f.new
		mv $f.new $f
	done
done
