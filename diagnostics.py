import subprocess, socket, re

net = "192.168.1.{}"

name_pattern = re.compile(".+(?=, )")
pos_pattern = re.compile("(?<=, )\d+")

while True:

    directory = []
    with open("ipconfig.txt") as ipconfig:
        for line in ipconfig:
            try:
                directory.append((name_pattern.search(line).group(),
                      int(pos_pattern.search(line).group())))
            except:
                print()
                print("Troublesome ipconfig line: " + line)

    print()
    for name, pos in sorted(directory, key=lambda x: x[1]):

        addr = net.format(pos)

        response = subprocess.run("ping " + addr + " -n 1 -w 2000",
            capture_output=True)

        if response.returncode == 0:
            print(name.upper().ljust(20)+ addr.ljust(16)  + "...SUCCESS")
            print()

        else:
            print(name.upper().ljust(20)+ addr.ljust(16)  + "...FAILURE [X]")
            print()

    input("Press ENTER to rerun. ============================ ")
