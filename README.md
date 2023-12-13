# multi_robile_navigation_experiments
# multi_robile_navigation_experiments

# Commands

In separate windows
``` bash
------ Window 1 Gazebo
$> ros2 launch robile_gazebo multi_robile_sim.launch.py
------ EDDI
$> ros2 launch eddi_monitor_launcher two_robile_eddi_with_adaptation.launch
------ Navigation stack
$> ros2 launch robile_navigation multi_nav2_bringup.launch.py
------ Robile 0 navigation behaviour starting as leader
$> cd multi_robile_navigation_experiments/multi_robile_navigation_experiments
$> python3 robile_0_behaviour.py
------ Robile 1 navigation behaviour starting as Follower
$> cd multi_robile_navigation_experiments/multi_robile_navigation_experiments
$> python3 robile_1_behaviour.py
``` 


```mermaid
sequenceDiagram
    participant robile_0
    participant eddi
    participant robile_1
    loop Every second
        robile_0->>eddi: /robile_0/robile_position 1 (Leader)
        robile_1->>eddi: /robile_1/robile_position 0 (Follower)

        
            robile_0->>eddi: /robile_0/consert/camera True (Working)
            robile_0->>eddi: /robile_0/consert/lidar True (Working)
            robile_0->>eddi: /robile_0/consert/navigation True (Working)
            eddi-->>robile_0: /robile_0/adaptation 'highly_connected_platooning_3'
         
            robile_1->>eddi: /robile_1/consert/camera True (Working)
            robile_1->>eddi: /robile_1/consert/lidar True (Working)
            robile_1->>eddi: /robile_1/consert/navigation True (Working)
            eddi-->>robile_1: /robile_1/adaptation 'highly_connected_platooning_3'
        
    end
    Note over robile_0: Failure in Leader
    robile_0->>eddi: /robile_0/consert/navigation False (Failure)
    eddi->>robile_0: /robile_0/adaptation 'follower_takeover'
```
