#!/bin/bash

# Define bag and csv filename:
VIDEO=false
filename=
input=$1
if [ -n "$input" ]
then
    filename=$1
else
    # read in args:
    if [ -a ".recordargs" ]
    then
	argv=$(cat .recordargs)
	eval set -- "$argv"
	while getopts ":xf:" opt; do
	    case "$opt" in
		x)
		    VIDEO=true
		    ;;
		f)
		    filename=${OPTARG}
		    ;;
	    esac
	done
    else
	echo "File .recordargs not found!"
	exit 1
    fi
fi
echo "Using '${filename}' as filename"

# If filename.* exists, move to backup#.*
if [ -a ${filename}.bag ]
then
    echo "Moving old bag files"
    num=`ls | grep "${filename}.*bag" | wc -l`
    mv ${filename}.bag ${filename}_backup_"$num".bag
fi

# Wait for a user to press a button:
echo "Press any button to start recording data..."
read -n 1 -s
echo "Beginning recording..."
# Start recording bag file:
rosbag record --quiet -O ${filename}.bag -e "(.*)/robot/accelerometer/right_accelerometer/state" \
    "(.*)/robot/limb/right/endpoint_state" "(.*)/robot/limb/right/interaction_controller/state" "(.*)mda_topic" \
    "(.*)visualization_marker_array"& #"(.*)acceptance"&

sleep 1
echo "Now press any button to stop recording..."
read -n 1 -s
echo "Stopping recording, now killing recording process..."
# Stop recording:
killall -2 record
sleep 2
echo "Generating csv file..."
# Generate default csv files:
info=`rosbag info ${filename}.bag`
endpoint=`echo $info | grep -e '\ \/[^ ]*' -o |grep -e '/robot/limb/right/endpoint_state'`
imu=`echo $info | grep -e '\ \/[^ ]*' -o |grep -e '/robot/accelerometer/right_accelerometer/state'`
interaction=`echo $info | grep -e '\ \/[^ ]*' -o |grep -e '/robot/limb/right/interaction_controller/state'`
mda=`echo $info | grep -e '\ \/[^ ]*' -o |grep -e 'mda_topic'`

rostopic echo -p -b ${filename}.bag $endpoint > ${filename}_ep.txt 
rostopic echo -p -b ${filename}.bag $imu > ${filename}_imu.csv 
rostopic echo -p -b ${filename}.bag $interaction > ${filename}_interact.txt 
rostopic echo -p -b ${filename}.bag $mda > ${filename}_mda.csv 
#rostopic echo -p -b ${filename}.bag $mda > ${filename}_mda.txt 
echo "Done creating bag and csv file"
