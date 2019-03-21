# CognitiveRoboticsLab2019 - Detecting and Tracking a Quadcopter with Radar

## Environment
- Ubuntu 18.04LTS
- ROS Melodic
## Project Structure
- packages
    - beginner_tutorials: Detection/Tracking package
        - src/cluster_extraction: tracking pipeline
        - launch/cluster_extraction.launch: run tracking on certain recorded bagfile
        - To run: ```roslaunch beginner_tutorials cluster_extraction ```
        
    - ti_mmwave_rospkg: Sensor Visualization
        - https://github.com/radar-lab/ti_mmwave_rospkg
        - To run: ```roslaunch ti_mmwave_rospkg 1642es2_short_range.launch  ```


## Configuration
- ti_mmwave_rospkg/cfg: configurations from experiments
 ![configurations](./cfg_explanation.png)


## Bagfiles
- beginner_tutorials/indoor_experiment_bag: human data recorded in HoersaalZentrum 1 floor with different config
- beginner_tutorials/last_fly_bag: drone data recorded with different config
    - short: 1642es2_short_range.cfg
    - mid: 8.cfg
    - long: 1642es2_long_range.cfg

