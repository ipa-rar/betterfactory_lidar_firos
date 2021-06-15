## Settting up the physical sick tim 571 Lidar
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
## Starting up the entire system
- Bring up the hardware driver nodes and other application nodes before running the firos node. Firos will not transform any new topics that arrive after launching firos node.
    ```
    - sudo ifconfig eth0 192.168.1.121 netmask 255.255.255.0
    ```
    ```
    - roslaunch sick_scan sick_tim_5xx.launch hostname:=192.168.1.1
    ```
    ```
    - rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map cloud 10
    ```
 - Launch the Orion Context Broker(OCB) in the docker container with port `1026` exposed.
    ```
    - sudo service docker start
    ```
    ```
         - sudo docker-compose up
    ```
- Finally, launch the firos node.
    ```
    - roslaunch firos firos.launch
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
## Docker launch
```
docker network create finet --subnet=19.168.1.123/16 -d ipvlan
```


## References
- [FIROS readthedocs](https://firos.readthedocs.io/en/latest/)
- [Swagger firos API endpoints reference](https://swagger.lab.fiware.org/?url=https://raw.githubusercontent.com/Fiware/specifications/master/OpenAPI/ngsiv2/ngsiv2-openapi.json#/)