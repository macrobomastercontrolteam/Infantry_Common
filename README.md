# Common Infantry Code for MacRM
Robot configs:
- INFANTRY_2024_MECANUM
- INFANTRY_2023_MECANUM
- INFANTRY_2023_SWERVE
- INFANTRY_BIPED
- SENTRY_2023_MECANUM

Developed based on "RoboMaster competition robot 2020 self-assembly version A", which was for Type A Dev Board. Refer to user manuals posted on
> "File download: [RoboMaster Competition Robot 2020 Self-Assembled Version A Type Related Documents](https://rm-static.djicdn.com/documents/26898/57bcb7163d7bd1575980335867998835.7z)"

on 
> RM's Official website: https://www.robomaster.com/zh-CN/resource/pages/announcement/1081.

# Firmware Environment
- Type C Development Board
- FreeRTOS

# Firmware Architecture
WIP on branch misc/Electrical_Assembly_Diagram

# Special Branches
- test_no_ref: test all other features without referee system by ignoring ref errrors
- release/sentry_head: code for upper head on dual-head sentry.
