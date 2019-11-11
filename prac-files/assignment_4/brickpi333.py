#!/usr/bin/env python

import time
from brickpi3 import *

class BrickPi333(BrickPi3):
    def __init__(self):
        super().__init__()
        time.sleep(1)

    def __del__(self):
        self.reset_all()
        time.sleep(1)
