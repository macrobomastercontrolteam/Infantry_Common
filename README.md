# Swerve robot controller on the lower platform
Abbreviated as the "lower board" in the code.

# Firmware Environment
- Type C Development Board
- FreeRTOS

# Firmware Architecture
Takes in commands from the upper head to control steering motor and hip motor. Those motors are isolated from the main chassis CAN network from upper board.
