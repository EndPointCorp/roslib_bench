#!/usr/bin/env bash

source /catkin_ws/devel/setup.bash

twistd -o web --path=/src/webapp

roslaunch /src/bench.launch
