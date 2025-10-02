for the map to work, we had to :

docker exec -it iw_hub_app-ros1-ros2-bridge-1 bash
vim config/parameter_bridge_config.yaml 

and add : 


  topic: /lts_ng/map
  type:  nav_msgs/msg/OccupancyGrid 
  queue_size: 1

the reason behind this change is that the map is being published on  /lts_ng/map which is a ros1 topic.