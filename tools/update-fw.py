# Lishui ebike controller firmware updater
#
# This is Linux friendly replacement for Windows tool 'LS_Controller_Update.exe'.
#
# Script can handle Intel HEX file format (output of 'arm-none-eabi-objcopy -O ihex')
# or Lishui LSH format. Conversion of HEX to LSH code taken from Java hexToLsh.java code.
#
# Author: Hinko Kocevar 2023
# Version: 0.1

from intelhex import IntelHex
from io import StringIO
import sys
import os
import serial
import time
from tqdm import trange


def hex2lsh(filename):

    ih = IntelHex()
    ih.loadhex(filename)
    # print('HEX input addresses: 0x%08X - 0x%08X' % (ih.minaddr(), ih.maxaddr()))

    limitAddress = 16 * ((ih.maxaddr() // 16) + 2)
    # print('Limit address: 0x%08X' % limitAddress)

    sio = StringIO()
    sio.write('\x00' * (limitAddress - ih.maxaddr() - 1))
    sio.seek(0)
    ih2 = IntelHex()
    ih2.loadbin(sio, offset=ih.maxaddr()+1)

    ih.merge(ih2)
    # print('HEX output addresses: 0x%08X - 0x%08X' % (ih.minaddr(), ih.maxaddr()))

    sio = StringIO()
    ih.write_hex_file(sio, byte_count=16)
    sio.seek(0)
    rawlines = sio.readlines()

    key = [ 0x81, 0x30, 0x00, 0x5a, 0x7f, 0xcb, 0x37, 0x13, 0x32, 0x85,
            0x20, 0x4b, 0xc8, 0xf3, 0x10, 0x2e, 0x1c, 0xa7, 0xc2, 0xa3 ]

    print('converting HEX to LSH..')
    lsh_data = []
    for line in rawlines:
        rawhex = line[3:-1]
        if rawhex[4:6] == '00':
            rawbin = bytes.fromhex(rawhex)
            idx = 0
            encbin = b''
            for b in rawbin:
                encbin += (b ^ key[idx]).to_bytes(1, 'little')
                idx += 1
            lsh_data.append(encbin.hex().upper() + '\n')

    return lsh_data


def wait_for_msg(port, timeout):
    rx_msg = b''
    while True:
        char = port.read()
        if char:
            # print('RX:', char)
            if char.isalpha():
                rx_msg += char

        # print('rx_msg', rx_msg)
        if rx_msg.find(b'MPC') != -1:
            return 'MPC'
        elif rx_msg.find(b'OK') != -1:
            return 'OK'
        elif rx_msg.find(b'ER') != -1:
            return 'ER'
        elif rx_msg.find(b'ET') != -1:
            return 'ET'

        if timeout is not None:
            timeout -= 1
            if timeout <= 0:
                return ''


def flash(portname, data):

    port = serial.Serial(portname, 38400, timeout=1)
    if not port.is_open:
        print('ERROR: serial port %s not opened!' % portname)
        return -1

    if wait_for_msg(port, None) != 'MPC':
        print('ERROR: failed to get MPC message')
        return -1

    print('starting xfer..')

    # begin transfer pattern
    tx_msg = b'D'*20
    port.write(tx_msg)
    # sleep for a bit
    time.sleep(0.1)
    port.reset_input_buffer()

    flash_bytes = 0
    for i in trange(len(data)):
        line = data[i].strip()
        # line = line.strip()
        tx_msg = bytes.fromhex(line)
        port.write(tx_msg)

        # MANDATORY
        # if we are on page boundary wait for flash page erase to finish!
        # MANDATORY
        if (flash_bytes & 0x3ff) == 0:
            # results in ~50ms on scope for some reson!?!
            time.sleep(0.11)
        flash_bytes += 16

        # controller will send 'ER' if data line crc is bad
        if port.in_waiting and wait_for_msg(port, 0) == 'ER':
            print('ERROR: got ER from the controller')
            return -1

    # end transfer pattern
    tx_msg = b'U'*20
    port.write(tx_msg)

    # controller will send 'OK' or 'ER' within couple of ms
    status = wait_for_msg(port, 2)
    port.close()
    if status == 'OK':
        print('xfer SUCCESS!')
        return 0
    else:
        print('xfer FAILED!')
        return -1


def main():

    if len(sys.argv) < 3:
        print('%s <port> <file.hex|file.lsh>' % sys.argv[0])
        return -1

    portname = sys.argv[1]
    filename = sys.argv[2]
    print('using serial port %s' % portname)
    print('using file %s' % filename)
    if not os.path.exists(portname):
        print('ERROR: serial port %s does not exist!' % portname)
        return -1
    if not os.path.exists(filename):
        print('ERROR: file %s does not exist!' % filename)
        return -1

    ext = os.path.splitext(filename)
    if ext[1] == '.hex':
        lsh_data = hex2lsh(filename)
    elif ext[1] == '.lsh':
        fp = open(filename, 'r')
        lsh_data = fp.readlines()
        fp.close()
    else:
        print('ERROR: unknown file type')
        return -1

    ret = flash(portname, lsh_data)
    return ret


if __name__ == '__main__':
    ret = main()
    exit(ret)
