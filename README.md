# MacRM_Standard_2023
McMaster Robomaster Standard Infantry Robot Code for 2023 Steering-wheel-chassis Model

Developed based on "RoboMaster competition robot 2020 self-assembly version A". Refer to user manuals posted on
> "File download: [RoboMaster Competition Robot 2020 Self-Assembled Version A Type Related Documents](https://rm-static.djicdn.com/documents/26898/57bcb7163d7bd1575980335867998835.7z)"

on 
> RM's Official website: https://www.robomaster.com/zh-CN/resource/pages/announcement/1081.

# Mechanical Features
- Steering motor chassis: four M6020 steering motors and four M3508 speed motors

# Firmware Environment
- InfantryMain controller: Type C Development Board
    - FreeRTOS
- InfantrySteer controller: Type A Development Board
    - Type C Board sends target encoder values (absolute angle) of steering motors to Type A Board with CAN ID 0x112;
    - Type A Board controls steering motors with PID

# Firmware Architecture
Same architecture to 2019:
Except a Type A Development Board is added to the system with CAN ID 0x112. This board only controls four steering motors on an isolated CAN bus.
![image](https://user-images.githubusercontent.com/57267209/185773597-4cd07a38-2232-4443-a679-13531dbe4313.png)

# Branches
- test_no_ref: test all other features without referee system by ignoring ref errrors
