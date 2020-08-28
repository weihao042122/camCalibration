#!/bin/bash

if [[ "$#" != "3" ]]; then
	echo "$0 PATH1 PATH2 OUT_PATH"
	exit -1
fi

PATH1=$1
PATH2=$2
PARM_OUT_PATH=$3

set -x
./camCalibr $PATH1
./camCalibr $PATH2

./extCalibra $PATH1 $PATH2

mv $PATH1/calibResult.txt $PARM_OUT_PATH/calibResult0.txt
mv $PATH2/calibResult.txt $PARM_OUT_PATH/calibResult1.txt
mv ./extrinsics.txt $PARM_OUT_PATH/extrinsics.txt
