"""
Python script for Fast 32 KByte UDP ethernet data packet capture

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
PKT_MAX = 1024  # Max bytes per datagram
TOTAL_BYTES = 32 * 1024  # Capture size (32 KB)
SOCKET_RCVBUF_KB = 512  # OS socket RX buffer size (KB)
TIMEOUT_FIRST = 10  # Seconds to wait for very first packet
WAIT_IDLE_MS = 200  # Stop if idle this long after buffer full
PLOT_FILE = "adc_sample.png"
HEX_FILE = "adc_sample16bit.txt"
DMA_RXBASE = 0x1300000
ADC_SAMPLE_RATE_HZ = 1_300_000_000.0


class AD9695_clk_delay_mode(Enum):
    AD9695_NO_CLOCK_DELAY = 0
    AD9695_FINE_DELAY_16 = 2
    AD9695_FINE_DELAY_16_LOW_JITTER = 3
    AD9695_FINE_DELAY_192 = 4
    AD9695_SUPERFINE_DELAY = 6


class AD9695_Channel_index_select(Enum):
    CHANNEL_A = 1  # 0x01
    CHANNEL_B = 2  # 0x02
    CHANNEL_A_B = 3  # 0x03


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

    # init an empty array for ADC clk config
    adc_clk_delay_mode = AD9695_clk_delay_mode.AD9695_NO_CLOCK_DELAY
    adc_fine_delay = 0
    adc_super_fine_delay = 0
    config_object = AD9695_Channel_index_select.CHANNEL_A  # default channel 1
    clk_cfg = [adc_clk_delay_mode, adc_fine_delay, adc_super_fine_delay, config_object]

    try:
        while not abort_flag:
            write_ptr = 0
            w_r_select = input(f"Input w for UDP Transmit\n"
                               f"Input r for UDP Receive\n")
            if w_r_select == "r" or (write_ptr < TOTAL_BYTES and write_ptr > 1024):
                print("Socket waiting for UDP data packages ...")
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
                print("Visualizing received data ...")
                int16_t_array = numpy.frombuffer(capture_buffer, dtype="<i2")
                figure, axes = pyplt.subplots(3, 2)
                sample_channel_a = numpy.empty(128, dtype="<i2")
                sample_channel_b = numpy.empty(128, dtype="<i2")
                count = 0
                for i in range(0, 127, 4):
                    sample_channel_a[i : i + 4] = int16_t_array[count : count + 4]
                    count += 4
                    sample_channel_b[i : i + 4] = int16_t_array[count : count + 4]
                    count += 4
                #for some reason channel a samples are all reversed, need to correct it
                sample_channel_a = -sample_channel_a
                Ts = 1 / ADC_SAMPLE_RATE_HZ
                ch_a_fft = numpy.fft.fft(sample_channel_a)
                ch_b_fft = numpy.fft.fft(sample_channel_b)
                ch_a_freq = numpy.fft.fftfreq(len(sample_channel_a), d=Ts)
                ch_b_freq = numpy.fft.fftfreq(len(sample_channel_b), d=Ts)

                axes[0, 0].plot(sample_channel_a)
                axes[0, 0].set_title("ADC Sample Plot (Channel A)")
                axes[0, 0].set_xlabel("ADC Sample Index")
                axes[0, 0].set_ylabel("Digital Amplitude")

                axes[0, 1].plot(ch_a_freq, numpy.abs(ch_a_fft))
                axes[0, 1].set_title("FFT Plot (Channel A)")
                axes[0, 1].set_xlabel("Frequency")
                axes[0, 1].set_ylabel("Magnitude")

                axes[1, 0].plot(sample_channel_b)
                axes[1, 0].set_title("ADC Sample Plot (Channel B)")
                axes[1, 0].set_xlabel("ADC Sample Index")
                axes[1, 0].set_ylabel("Digital Amplitude")

                axes[1, 1].plot(ch_b_freq, numpy.abs(ch_b_fft))
                axes[1, 1].set_title("FFT Plot (Channel B)")
                axes[1, 1].set_xlabel("Frequency")
                axes[1, 1].set_ylabel("Magnitude")

                axes[2, 0].hist(sample_channel_a)
                axes[2, 0].set_title("ADC Sample Distribution (Channel A)")

                axes[2, 1].hist(sample_channel_b)
                axes[2, 1].set_title("ADC Sample Distribution (Channel B)")

                pyplt.tight_layout()
                print("Please close the window to proceed")
                with open(HEX_FILE, "w", encoding="utf-8") as output_file:
                    for i in range(0, 512, 2):
                        block_128 = capture_buffer[i: i + 2][
                                    ::-1]  # Write the bytes to the 128 bit block in Big endian
                        # Step From i + 16 to i
                        output_file.write(f"#{i // 2} : {block_128.hex().upper()}\n")

                print(f"[i] Hex blocks saved → {HEX_FILE}")
                pyplt.show()
                figure.savefig(PLOT_FILE)



            elif w_r_select == "w":
                print("Socket will send out UDP packages")
                try:
                    # get clk delay type, fine delay amount, super fine delay amount from user
                    print("Prompting user to enter adc sampling clock config ...")
                    send_flag = False
                    while not send_flag:
                        print(f"Current clk config:\n"
                              f"AD9695 Clk Delay Mode (NOTE: THIS WILL AFFECT BOTH CHANNELS): {clk_cfg[0].name}\n"
                              f"Fine Clk Delay Steps: {clk_cfg[1]}\n"
                              f"Super Fine Clk Delay Steps: {clk_cfg[2]}\n"
                              f"Total Clk Delay: {clk_cfg[1] * 1.725} ps + {clk_cfg[2] * 0.25} ps = {clk_cfg[1] * 1.725 + clk_cfg[2] * 0.25} ps\n"
                              f"Channel(s) to receive the above settings: {clk_cfg[3].name}\n")
                        config_type = int(input("Please choose what to configure: \n"
                                                "Type 1 > for AD9695 Clk Delay mode\n"
                                                "Type 2 > for AD9695 fine delay amount (Step size = 1.725 ps)\n"
                                                "Type 3 > for AD9695 super fine delay amount (Step size = 0.25 ps)\n"
                                                "Type 4 > to select the configuration object (Ch A or Ch B or both)\n\n"
                                                "Type 0 > to send\n"))
                        match config_type:
                            case 1:
                                adc_clk_delay_mode = int(input("Type\n"
                                                               "1 > NO Clock Delay\n"
                                                               "2 > Enable Fine Delay\n"
                                                               "3 > Enable Fine Delay and Super Fine Delay\n"))
                                match adc_clk_delay_mode:
                                    case 1:
                                        adc_clk_delay_mode = AD9695_clk_delay_mode.AD9695_NO_CLOCK_DELAY
                                    case 2:
                                        adc_clk_delay_mode = AD9695_clk_delay_mode.AD9695_FINE_DELAY_192
                                    case 3:
                                        adc_clk_delay_mode = AD9695_clk_delay_mode.AD9695_SUPERFINE_DELAY
                                    case _:
                                        print("Option invalid. Abort Selection")
                                        continue
                                clk_cfg[0] = adc_clk_delay_mode
                            case 2:
                                adc_fine_delay = int(input("Enter fine delay steps (0 <= step <= 192)\n"))
                                if (adc_fine_delay < 0) or (adc_fine_delay > 192):
                                    print("Option invalid. Abort Selection")
                                    continue
                                clk_cfg[1] = adc_fine_delay
                            case 3:
                                adc_super_fine_delay = int(input("Enter super fine delay steps (0 <= step <= 128)\n"))
                                if (adc_super_fine_delay < 0) or (adc_super_fine_delay > 192):
                                    print("Option invalid. Abort Selection")
                                    continue
                                clk_cfg[2] = adc_super_fine_delay
                            case 0:
                                send_flag = True
                            case 4:
                                config_object = input("Choose which channel(s) to be configured:\n"
                                                      "a/A   > Channel A\n"
                                                      "b/B   > Channel B\n"
                                                      "ab/AB > Both Channels\n")
                                match config_object:
                                    case "a" | "A":
                                        config_object = AD9695_Channel_index_select.CHANNEL_A
                                    case "b" | "B":
                                        config_object = AD9695_Channel_index_select.CHANNEL_B
                                    case "ab" | "AB" | "Ab" | "aB":
                                        config_object = AD9695_Channel_index_select.CHANNEL_A_B
                                    case _:
                                        print("Option invalid. Abort Selection")
                                        continue
                                clk_cfg[3] = config_object
                            case _:
                                print("Option invalid. Abort Selection")
                                continue
                    t_buf = bytearray([clk_cfg[0].value] + clk_cfg[1:3] + [clk_cfg[3].value])
                    sent = socket_inst.sendto(t_buf, (BOARD_IP, UDP_PORT))
                    print(f"clk config array:\n{t_buf.hex(' ')}\n"
                          f"was sent to [IP: {BOARD_IP} | PORT: {UDP_PORT}]\n")
                except OSError as err:
                    print("TX ERROR", err)

            else:
                print(f"User input incorrect\n"
                      f"You should input either r or w\n")
                continue

    except KeyboardInterrupt:
        print("User Abort")
    finally:
        socket_inst.close()

if __name__ == "__main__":
    main()
