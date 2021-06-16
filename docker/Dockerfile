FROM ros:melodic-ros-core-bionic



RUN mkdir catkin_ws \
    && mkdir catkin_ws/src \
    && mkdir catkin_ws/src/firos \
    && mkdir catkin_ws/devel \
    && mkdir catkin_ws/build

COPY ./firos /catkin_ws/src/firos
COPY ./sick_scan /catkin_ws/src/sick_scan
COPY ./betterfactory_bringup /catkin_ws/src/betterfactory_bringup
WORKDIR /catkin_ws

# install ros packages and dependencies
RUN apt-get update && apt-get install -y ros-melodic-ros-base python python-pip python-rosdep

RUN apt install ros-melodic-sick-scan -y
RUN apt install ros-melodic-tf2-msgs -y
RUN apt install ros-melodic-geometry-msgs -y
RUN apt install ros-melodic-sensor-msgs -y
RUN apt install ros-melodic-xacro -y

RUN pip install -r  /catkin_ws/src/firos/requirements.txt

RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'

EXPOSE 10100
CMD /bin/bash -c '. /opt/ros/melodic/setup.bash; . /catkin_ws/devel/setup.bash; roslaunch betterfactory_bringup firos_bringup.launch'

