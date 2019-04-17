#!/usr/bin/python3

import sys
import transmission

if len(sys.argv) < 2:
    print(sys.argv[0], ": requires filename")
    exit(1)

packets = transmission.fromFile(sys.argv[1])
line = 0
for p in packets:
    linestr = "{:04x}".format(line)
    print(linestr + "\t" + transmission.packet2str(p))
    line = line + 1
