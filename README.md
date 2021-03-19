<div align="center">
    <h1>
        <br>
            Pymixel
        <br>
    </h1>
</div>

`Pymixel` is a simple and easy way to deal with the Dynamixel X Series. It's fast and based on the `DynamixelSDK`. 

## Features:
- [X] fast and easy to understand
- [X] support for group writes
- [X] most of the common features implemented
- [X] [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) as dependency

## License:
[![License](https://img.shields.io/badge/license-GPLv3-blue.svg?longCache=true&style=flat)](https://github.com/Vinz1911/Pymixel/blob/master/LICENSE)

## Python & Pypi:
[![Python](https://img.shields.io/badge/Python-v3.8-blue.svg?logo=python&style=flat)](https://www.python.org) [![PyPi](https://img.shields.io/badge/PyPi-Support-blue.svg?logo=pypi&style=flat)](https://pypi.org)

## Install & Upgrade:
```shell
# install via pypi
pip3 install pymixel
# upgrade
pip3 install --upgrade pymixel
```

## Import:
```python
from pymixel import Dynamixel
```

## Usage:
### Examples:
```python
from pymixel import Dynamixel
import time

DXL_ID_ONE = 1      # dynamixel id
VELOCITY_MODE = 1   # velocity mode

dynamixel = Dynamixel()                                 # create instance
dynamixel.open()                                        # open serial communication
dynamixel.set_operating_mode(DXL_ID_ONE, VELOCITY_MODE) # set dynamixel mode
dynamixel.set_torque(DXL_ID_ONE, True)                  # enable torque
dynamixel.set_led(DXL_ID_ONE, True)                     # enable led
dynamixel.set_goal_velocity(DXL_ID_ONE, 100)            # start moving
...
```

## Author:
👨🏼‍💻 [Vinzenz Weist](https://github.com/Vinz1911)