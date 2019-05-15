import time


f = open("./recivedpackets.txt", 'w')
i = 0
while True:
    time.sleep(2)
    s = "sent packet #" + str(i)
    f.write(s)
    i += 1

