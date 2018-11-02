#!/usr/bin/env bash

docker build -t roslib_bench .
docker run -it --rm --net=host roslib_bench
