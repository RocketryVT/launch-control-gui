# transmission.py

import numpy as np
from collections import deque
import struct


def parse(bytes):
    parsing = True
    packets = []

    while len(bytes) > 0 and parsing:
        while len(bytes) > 0 and bytes[0] != 0xAA:
            bytes.popleft()
        if len(bytes) < 2:
            continue
        if bytes[1] != 0x14:
            bytes.popleft()
            continue
        if len(bytes) < 4:
            parsing = False
            continue
        data_len = bytes[2]
        if len(bytes) < data_len + 6:
            parsing = False
            continue
        packet = []
        for i in range(0, 4 + data_len):
            packet.append(bytes[i])

        c0_true, c1_true = xorchecksum(packet)
        c0 = bytes[4 + data_len]
        c1 = bytes[5 + data_len]

        if c0 != c0_true or c1 != c1_true:
            print("Checksum error: got (" + "{:02x}".format(c0) +
                  ", {:02x}".format(c1) + "), expected " +
                  "({:02x}".format(c0_true) + ", {:02x}".format(c1_true) + ")")
            bytes.popleft()
            continue
        packet.append(c0)
        packet.append(c1)

        packets.append(packet)
        for i in range(0, len(packet)):
            bytes.popleft()
    return packets


def xorchecksum(packet):
    c0 = 0
    c1 = 0
    i = 0
    while i < len(packet) - 1:
        c0 ^= packet[i]
        c1 ^= packet[i + 1]
        i += 2
    if i < len(packet):
        c0 ^= packet[i]
    return c0, c1


def fromFile(filename):
    bytes = open(filename, "rb").read()
    return parse(deque(bytes))


def packet2str(packet):
    str = ""
    if len(packet) < 6:
        return "Packet not long enough"
    for i in range(0, len(packet)):
        str = str + "{:02x}".format(packet[i])
        if i < len(packet) - 1:
            str = str + " "
    return str


def appendChecksum(packet):
    c0, c1 = xorchecksum(packet)
    packet.append(c0)
    packet.append(c1)


def buildPacket(id, message):
    packet = [0xAA, 0x14, len(message), id]
    for c in message:
        packet.append(c)
    appendChecksum(packet)
    return packet


def packetify(formatted):
    ret = bytes(0);

    for token in formatted.split(' '):

        try:

            if token.endswith('n8'):
                num = int(token[0:-2])
                ret += struct.pack(">b", num)

            elif token.endswith('n16'):
                num = int(token[0:-3])
                ret += struct.pack(">h", num)

            elif token.endswith('n32'):
                num = int(token[0:-3])
                ret += struct.pack(">l", num)

            elif token.endswith('n64'):
                num = int(token[0:-3])
                ret += struct.pack(">q", num)

            elif token.endswith('u8'):
                num = int(token[0:-2])
                ret += struct.pack(">c", num)

            elif token.endswith('u16'):
                num = int(token[0:-3])
                ret += struct.pack(">H", num)

            elif token.endswith('u32'):
                num = int(token[0:-3])
                ret += struct.pack(">L", num)

            elif token.endswith('u64'):
                num = int(token[0:-3])
                ret += struct.pack(">Q", num)

            elif token.endswith('f'):
                num = float(token[0:-1])
                print(num)
                ret += struct.pack(">f", num)

            elif token.endswith('d'):
                num = float(token[0:-1])
                ret += struct.pack(">d", num)

            elif token.startswith('\"') and token.endswith('\"'):
                string = token[1:-1];
                ret += bytes(string, 'utf-8')

            else:
                print("Unrecognized token: " + token)

        except ValueError:
            print("Encountered an error parsing token: " + token)
            continue

    return list(ret);
