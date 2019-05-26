# microclimate-lorawan-firmware
Unofficial firmware for microclimate.network device in Arduino environment, under development, use not recommended. See microclimate.network for official content.

Basic operation:
 * Send at 180s for first 50 frames to trigger ADR
 * Then send at defined intervals based on data rate/ttn fair use policy

# LoraWAN configuration
OTAA activation with ADR and auto-rejoin upon link failures. First join after boot retry is ever 180s.

# Known issues
All corner cases not tested for hands.

# Requirements:
Core: https://github.com/GrumpyOldPizza/ArduinoCore-stm32l0
Note you need to upgrade to the latest git version as release 0.0.8 does not have all necessary fixes.

# TODO:
Evaluate OTAA join retry tiemouts and process.