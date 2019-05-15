import time

f = open("./recivedpackets.txt", 'r')

while True:
    for line in f:
        print(line)
    time.sleep(3)

