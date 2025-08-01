#!/usr/bin/env python3
#
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2025, Raspberry Pi Ltd.
#
# udp_object_detection.py - This utility file implements a UDP receiver
# designed to process object detection information. It's specifically
# tailored to receive data sent by `object_detect_udp_stage.cpp`,
# but can also be used with generic UDP senders like `nc -ulp 12347`
# for debugging raw data.

import socket
import struct
import sys
from dataclasses import dataclass

# --- Data Structures ---


@dataclass
class ParsedDetection:
    """A data class to hold the parsed detection data received over UDP."""
    x: int
    y: int
    width: int
    height: int
    name: str
    confidence: float

# --- UDP_AI_Receiver Class Definition ---


class UDP_AI_Receiver:
    """
    The UDP_AI_Receiver class handles the establishment of a UDP socket,
    receiving incoming data, and parsing it into the ParsedDetection format.
    """
    MAX_BUFFER_SIZE = 1024

    def __init__(self, port: int):
        """
        Constructor: Initializes the UDP receiver.
        :param port: The UDP port number to listen on.
        """
        try:
            # Create a UDP socket.
            # AF_INET specifies the IPv4 address family.
            # SOCK_DGRAM specifies a UDP (datagram) socket.
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Bind the created socket to the specified IP address and port.
            self.sock.bind(('', port))
        except socket.error as e:
            print(f"Failed to create or bind socket: {e}", file=sys.stderr)
            sys.exit(1)

        print(f"Listening for UDP detection packets on port {port}...")

    def __del__(self):
        """Destructor: Cleans up resources by closing the socket."""
        self.sock.close()

    def receive_detection(self) -> ParsedDetection | None:
        """
        Method to receive and parse a single detection packet.
        :return: A ParsedDetection object if a packet was successfully received and parsed,
                 None otherwise.
        """
        try:
            buffer, addr = self.sock.recvfrom(self.MAX_BUFFER_SIZE)
            return self._parse_detection(buffer)
        except socket.error as e:
            print(f"Failed to receive UDP message: {e}", file=sys.stderr)
            return None

    @staticmethod
    def _parse_detection(buffer: bytes) -> ParsedDetection | None:
        """
        Helper method to parse the raw byte buffer into a ParsedDetection object.
        :param buffer: A byte string containing the received data.
        :return: A ParsedDetection object if parsing was successful, None otherwise.
        """
        START_DELIMITER_LE = 0xDDCCBBAA

        # Define the minimum expected size of a valid packet, excluding the name string length.
        # Delimiter (4) + x (4) + y (4) + width (4) + height (4) + name_length (1) + confidence (4)
        MIN_PACKET_SIZE = 4 + 4 + 4 + 4 + 4 + 1 + 4

        if len(buffer) < MIN_PACKET_SIZE:
            print("Packet too small to be a valid detection.", file=sys.stderr)
            return None

        # Check for the presence of the start delimiter.
        delimiter = struct.unpack('<I', buffer[0:4])[0]
        if delimiter != START_DELIMITER_LE:
            print("Invalid packet: delimiter not found.", file=sys.stderr)
            return None

        offset = 4  # Start after the delimiter

        try:
            # Unpack the integer coordinates and dimensions.
            x, y, width, height = struct.unpack('<iiii', buffer[offset:offset + 16])
            offset += 16

            # Unpack the length of the object's name.
            name_length = struct.unpack('<B', buffer[offset:offset + 1])[0]
            offset += 1

            # Check for a malformed packet where the reported name length
            # would exceed the received buffer size.
            if offset + name_length + 4 > len(buffer):
                print("Invalid packet: name length exceeds buffer size.",
                      file=sys.stderr)
                return None

            # Unpack the name string.
            name = buffer[offset:offset + name_length].decode('utf-8')
            offset += name_length

            # Unpack the confidence score.
            confidence = struct.unpack('<f', buffer[offset:offset + 4])[0]

            return ParsedDetection(x, y, width, height, name, confidence)

        except struct.error as e:
            print(f"Error unpacking packet data: {e}", file=sys.stderr)
            return None

# --- Example Usage of the Receiver Class ---


if __name__ == "__main__":
    port = 12347  # The UDP port number this receiver will listen on.
    # Ensure this matches the port your sender is transmitting to.
    receiver = UDP_AI_Receiver(port)

    # Enter an infinite loop to continuously receive and process detection packets.
    while True:
        detection = receiver.receive_detection()
        if detection:
            # If a packet was successfully received and parsed, print its contents.
            print("Received Detection:")
            print(
                f"  Box: ({detection.x}, {detection.y}, {detection.width}, {detection.height})")
            print(f"  Name: {detection.name}")
            print(f"  Confidence: {detection.confidence}")
