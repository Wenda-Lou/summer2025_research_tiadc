"""
Python script for Fast 32 KByte UDP ethernet data packet capture
Dump 128 bit blocks, split the block into 16 bit binary numbers and dynamically plot

Author : Jingling Hou
"""

import socket
import matplotlib.pyplot as pyplt
import numpy  # Using numpy for high efficiency data organization and computation
import time
from enum import Enum

## Start of User parameters
BOARD_IP = "192.168.1.10"  # Sender IP --> Configured in Vitis
UDP_PORT = 5002  # Port --> Same as above
PKT_MAX =  512 # Max bytes per datagram
TOTAL_BYTES = 512  # Capture size (32 KB)
SOCKET_RCVBUF_KB = 512  # OS socket RX buffer size (KB)
TIMEOUT_FIRST = 10  # Seconds to wait for very first packet
WAIT_IDLE_MS = 200  # Stop if idle this long after buffer full
HEX_FILE = "received_128bit_hex.txt"
PLOT_FILE = "first16_plot.png"
DMA_RXBASE = 0x1300000


## Start of user function definition
def detect_sample_packet_header(word_array: numpy.ndarray):
    """
        Finding the starting point of the incoming data packet, ensuring the data sequence
        of the received packet aligns with the actual memory addr inside PS

        NOTE** This function is only for detecting the header of the testing samples

        :param word_array:
        :return: the starting point index of the incoming data packet
        """
    array_element_difference = numpy.diff(word_array.astype(numpy.int32))
    # finding the difference between each array element. arr[i+1] - arr[i]
    # converting the type of array elements into int32 to prevent data overflow
    threshold_check_list = numpy.where((numpy.abs(array_element_difference) > 1000))[0]
    # making sure return val is a 1D array
    # if the difference between the two elements is larger than 1000, return True index
    # there should only be two or one such value for the testing sample
    starting_index = 0
    if threshold_check_list.size:
        starting_index = int(threshold_check_list[0] + 1)
        # Plus one to compensate the index misalignment due to the difference
    return starting_index


def socket_init():
    """
    Create and init a python socket instance
    :return: an initialized socket instance
    """
    socket_inst = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Init socket instance & Specify used Protocols
    # AF_INET -> IPv4 SOCK_DGRAM -> UDP
    socket_inst.bind(("", UDP_PORT))
    # "" == IPADDR_ANY
    socket_inst.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, SOCKET_RCVBUF_KB * 1024)
    # Set socket option -> this socket instance -> change receive buffer size to 512K byte
    socket_inst.settimeout(None)
    # Enable socket blocking mode
    # System will wait until the data packet is received
    socket_inst.connect((BOARD_IP, UDP_PORT))
    print(f"Socket Init finished\n"
          f"Listening {UDP_PORT}, Receive Buffer = {SOCKET_RCVBUF_KB} KB")
    connected = False
    for i in range(32):
        dummy = socket_inst.recv(1024)

    if not connected:
        connected = True
        print(f"Connected to the Server\n"
              f"IP: {BOARD_IP} | Port: {UDP_PORT}")
    socket_inst.settimeout(0.1)
    return socket_inst


def main():
    socket_inst = socket_init()

    # Allocate a 32KB capture buffer
    capture_buffer = bytearray(TOTAL_BYTES)
    memory_view = memoryview(capture_buffer)
    # Enable socket to directly write into the allocated memory location of the capture buffer
    abort_flag = False
    try:
        while not abort_flag:
            write_ptr = 0
            w_r_select = input(f"Input w for UDP Transmit\n"
                               f"Input r for UDP Receive\n")
            if w_r_select == "r" or (write_ptr < TOTAL_BYTES and write_ptr > 1024):
                print("Socket will receive UDP packages")
                while write_ptr < TOTAL_BYTES:
                    remain = TOTAL_BYTES - write_ptr
                    max_data_read = PKT_MAX if remain >= PKT_MAX else remain
                    try:
                        num_received_bytes = socket_inst.recv_into(memory_view[write_ptr:], max_data_read)
                    except socket.timeout:
                        if (write_ptr > 1024):
                            write_ptr = 0
                            print("Data misaligned, reset write pointer\n"
                                  "Abort this data package and restart\n")
                        continue
                    # Read max_data_read amount of bytes of this data packet into the allocated memory location
                    # Allowed space starts from the position pointed by the write
                    # script will keep monitoring the board IP and the server port

                    write_ptr += num_received_bytes
                    last_rx = time.time()
                    print(f"[{last_rx}] Received Package #{write_ptr // 1024}")

                print(f"[✓] Captured {write_ptr} bytes")
                check_abort = input(f"Press t to terminate transmission\n"
                                    f"Press anything else to receive a new packet\n")
                if check_abort == 't':
                    abort_flag = True

            elif w_r_select == "w":
                print("Socket will send out UDP packages")
                t_buf = bytearray(range(3))  # init an empty array
                try:

                    sent = socket_inst.sendto(t_buf, (BOARD_IP, UDP_PORT))
                    print(f"Testing array:\n{t_buf.hex(' ')}\n"
                          f"was sent to [IP: {BOARD_IP} | PORT: {UDP_PORT}]\n")
                except OSError as err:
                    print("TX ERROR", err)

            else:
                print(f"User input incorrect\n"
                      f"You should input either r or w\n")
                continue

            with open(HEX_FILE, "w", encoding="utf-8") as output_file:
                for i in range(0, TOTAL_BYTES, 16):
                    block_128 = capture_buffer[i: i + 16][::-1]  # Write the bytes to the 128 bit block in Big endian
                    # Step From i + 16 to i
                    output_file.write(f"#{i // 16} : {block_128.hex().upper()}\n")

            print(f"[i] Hex blocks saved → {HEX_FILE}")

            # 3) Plot the first 16 bit each block
            # first16 = numpy.frombuffer(capture_buffer, dtype="<u2")[::8]  # step = 16 B / 2 B
            # fig, ax = pyplt.subplots()
            # ax.plot(first16)
            # ax.set_title("LSB-16 of each 128-bit block (32 KiB capture)")
            # ax.set_xlabel("Block index")
            # ax.set_ylabel("Value (0-65535)")
            # ax.grid(True)
            # fig.tight_layout()
            # fig.savefig(PLOT_FILE, dpi=300)
            # print(f"[i] Plot saved → {PLOT_FILE}")

    except KeyboardInterrupt:
        print("User Abort")
    finally:
        socket_inst.close()

    # Post Data Receive Processing
    # 1) Check header and reorganize data packet


    pyplt.show()


if __name__ == "__main__":
    main()
