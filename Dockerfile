FROM ros:melodic

RUN apt-get update \
 && apt-get -y upgrade \
 && apt-get -y install \
    ros-melodic-tf2-web-republisher \
    python-twisted \
 && rm -rf /var/lib/apt/lists/*

ENV CATKIN_DIR=/catkin_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
 && mkdir -p $CATKIN_DIR/src \
 && cd $CATKIN_DIR/src \
 && catkin_init_workspace

COPY rosbridge_suite/rosbridge_suite/package.xml $CATKIN_DIR/src/rosbridge_suite/rosbridge_suite/
COPY rosbridge_suite/rosbridge_library/package.xml $CATKIN_DIR/src/rosbridge_suite/rosbridge_library/
COPY rosbridge_suite/rosbridge_server/package.xml $CATKIN_DIR/src/rosbridge_suite/rosbridge_server/
COPY rosbridge_suite/rosapi/package.xml $CATKIN_DIR/src/rosbridge_suite/rosapi/

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
 && cd $CATKIN_DIR \
 && apt-get update \
 && rosdep update \
 && rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro $ROS_DISTRO \
    -y \
 && rm -rf /var/lib/apt/lists/*

COPY rosbridge_suite $CATKIN_DIR/src/rosbridge_suite

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
 && cd $CATKIN_DIR \
 && catkin_make

COPY webapp /src/webapp

COPY bench.launch /src/
COPY in_container_startup.sh /

CMD bash /in_container_startup.sh
