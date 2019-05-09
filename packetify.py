
import sys
import rvtr

if len(sys.argv) < 2:
    packstr = '56.34d 78u16 -4573n32 #Hello!'
    print(sys.argv[0] + ": requires packet str")
    print("For example:")
    print("> python3 packetify.py '" + packstr + "'");
else:
    packstr = sys.argv[1];

bytes = rvtr.packetify(packstr)
print(" ".join("{:02x}".format(x) for x in bytes))
