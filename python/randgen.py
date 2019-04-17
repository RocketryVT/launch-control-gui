#!/usr/bin/python3

import transmission as tr
import numpy as np
import time
import sys

if len(sys.argv) < 2:
    print(sys.argv[0], ": requires filename")
    exit(1)

filename = sys.argv[1]
file = open(filename, "wb")
now = time.time()
wait = now + 1

while True:

    len = np.random.randint(5,50)
    packet = tr.buildPacket(np.random.randint(1,30),
        np.random.randint(1,255,len))
    print(tr.packet2str(packet))
    file.write(bytearray(packet))
    file.flush()   
 
    while now < wait:
        now = time.time()

    wait = wait + 1
