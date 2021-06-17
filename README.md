## Installation steps
```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone --recursive https://github.com/ipa-rar/betterfactory_lidar_firos.git
cd ~/catkin_ws
catkin_make
source /devel/setup.bash

```
## About this repository
```
.
├── betterfactory_bringup
├── docker
├── firos
├── orion_context_broker
├── README.md
└── sick_scan

```
- `betterfactory_bringup` contains the launch files and configuration for launching the firos, tf and laserscanner.
- `docker` contains the docker related files to dockerize the application
- `firos` is used to transform ROS msgs to NGSIv2 message type. Here we define the configurations about the topics that needs to be publisher or subscribed.
- `sick_scan` is the official ros driver for sick_tim_5xx laser scanner. This can be replaced by the new lidar driver. 

## Starting up the entire system
 - Launch the Orion Context Broker(OCB) in the docker container with port `1026` exposed.
    ```
    - sudo docker-compose up
    ```
- Bring up the hardware driver nodes and other application nodes before running the firos node. Firos will not transform any new topics that arrive after launching firos node.
    ```
    sudo ifconfig eth0 192.168.1.121 netmask 255.255.255.0
    ```
    ```
    - roslaunch betterfactory_bringup firos_bringup.launch
    ```

## Demo of Sub and Pub
- This will start one firos instance that will publish the data to the ocb and another firos instance that will subscribe to the data from the ocb. Since we already use port 10100 for firos instance we have to use a differnt port for other instance. 
```
    roscore
    rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map cloud 10
    rosrun firos core.py --conf ./lidar_config
    rosrun firos core.py --conf ./lidar_reverse_config -P 10101 --ros-node-name firos2
```

## Usage
Once you have started up the entire system you can check the contents in the OCB. You can do this by using terminal, postman and web-browser. Web browser is most convenient way as it pretty format the json contents.
- To list all the available endpoints
    ```
    http://localhost:1026/v2
    ```
    - Expected result

        ```
        {
        "entities_url": "/v2/entities",
        "types_url": "/v2/types",
        "subscriptions_url": "/v2/subscriptions",
        "registrations_url": "/v2/registrations"
        }
        ```
- To list the content that are subscribed by the OCB
    ```
    http://localhost:1026/v2/entities
    ```
- To list the type of messages that are subscribed by the OCB
    ```
    http://localhost:1026/v2/types
    ```
- To check the status of FIROS
    ```
    http://localhost:10100

- To list the topics subscribed/published by FIROS
    ```
    http://localhost:10100/topics


## Setting up the physical sick tim 571 Lidar
- Install the [SOPAS tool](https://www.sick.com/ag/en/sopas-engineering-tool/p/p367244) from the SICK Sensor Intelligence to configure the LIDAR IP address
- Inorder to identify the LIDAR in windows 10
    - Open up `Control pannel -> Network and internet -> Netowork connections -> Ethernet`
    - Right click on the Ethernet and select properties. This needs admin rights
    - Then select `Internet Protocol Version 4 (TCP/IPv4)`
    - Set IP address of series `192.168.1.xxx` and Subnet mask to `255.255.255.0` and save it.
- Now your SOPAS tool can identify your LIDAR
    - Search for the device using the ethernet interface search setting
    - Edit the IP address to the series `192.168.1.xxx` and Subnet mask to `255.255.255.0`
    - This will remove the connection with your LIDAR and you need to search again using the same settings to identify the lidar
- Now you can start using your LIDAR on Ubuntu System
    - Change your IP address to the same series of LIDAR
    - `sudo ifconfig eth0 192.168.1.xxx netmask 255.255.255.0`
- ROS system setup
    - `roslaunch sick_scan sick_tim_5xx.launch hostname:=192.168.1.1`
    - `rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map cloud 10`
    - `rosrun rviz rviz`
    - Laser data is published on `/scan`

## Setting up the Lidar publisher node
### Conguration files
- Setup the `config.json`. Here server is the `firos` node and `context_broker` is responsible of transforming ROS messages to the NGSIv2 and interfacing the ROS world with non-ROS world via the OCB.
    ````
    {
        "environment": "local",
        "local": {
            "server": {
                "port": 10100
            },
            "contextbroker": {
                "address": "localhost",
                "port": 1026
            }
        }
    }
    ````
- The config file `topics.json` defines the topic that needs to be interfaced with OCB. The message type and topic names are defined here. The `publisher` and `subscriber` terminology is at the non-ROS world. The `/scan` and `/tf` are being subscribed by the OCB or in other words these topics are published to the OCB.

    ````
    {
        "/scan": ["sensor_msgs/LaserScan", "subscriber"],
        "/tf": ["tf2_msgs/TFMessage", "subscriber"]
    }
    ````
- As the name suggests, the `whitelist.json` functions as a whitelist to let FIROS know which messages it should keep track of. Given an environment where already ROS-Applications are running, FIROS will not automatically subscribe to all available topics if no whitelist.json is given. In a small ROS-World with few ROS-Applications, it can be desirable to subscribe to all topics.
    ```
    {}
    ```
#

## Docker container launch
**Work In Progress!**
```
docker network create finet --subnet=19.168.1.123/16 -d ipvlan
```
## Troubleshooting
- Laserscanner does not get detected - refer to Setting up the physical sick tim 571 Lidar section and verify its `ip address`
- firos fails to launch - run the orion_context_broker first before running the firos
- Nothing is subscribed/published - firos should be the last one to be started as it works by creating the snapshop of the current running system.


## References
- [FIROS readthedocs](https://firos.readthedocs.io/en/latest/)
- [Swagger firos API endpoints reference](https://swagger.lab.fiware.org/?url=https://raw.githubusercontent.com/Fiware/specifications/master/OpenAPI/ngsiv2/ngsiv2-openapi.json#/)