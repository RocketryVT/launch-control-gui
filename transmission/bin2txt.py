#!/usr/bin/python3

import sys
import transmission

if len(sys.argv) < 2:
    print(sys.argv[0], ": requires filename")
    exit(1)

filename = sys.argv[1];
if filename[-4:] != ".rvt":
    print("Warning! Input file may not be a .rvt log file") 

packets = transmission.fromFile(filename)
line = 0
for p in packets:
    linestr = "{:04x}".format(line)
    print(linestr + "\t" + transmission.packet2str(p))
    line = line + 1
