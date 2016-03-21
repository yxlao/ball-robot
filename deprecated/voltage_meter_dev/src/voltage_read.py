#!/usr/bin/env python
import os

acm0 = os.open("/dev/ttyACM0", os.O_RDONLY)
for i in xrange(50):
    a = os.read(acm0, 100)
    print a
