#!/bin/bash

while true; do
	if pgrep electricTree
	then
		echo "OK";
	else
		/home/zac/build-electricTree-Desktop_Qt_5_7_0_GCC_64bit-Default/electricTree &
	fi
	sleep 1
done
