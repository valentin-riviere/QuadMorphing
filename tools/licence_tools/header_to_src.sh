: '
This file is part of QuadMorphing project
Copyright (C) 2018 - Institute of Movement Sciences (Marseille, FRANCE)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'
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
		echo -e "/*\n$(cat $header)\n*/\n$(cat $f)" > $f.new
		mv $f.new $f
	done
	for f in $(find $d -name '*.sh'); do
		echo $f
		echo -e ": '\n$(cat $header)\n'\n$(cat $f)" > $f.new
		mv $f.new $f
	done
done
