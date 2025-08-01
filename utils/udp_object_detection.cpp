/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * udp_object_detection.cpp - This utility file implements a UDP receiver
 * designed to process object detection information. It's specifically
 * tailored to receive data sent by `object_detect_draw_cv_stage.cpp`,
 * but can also be used with generic UDP senders like `nc -ulp 12347`
 * for debugging raw data.
 *
 * To compile this file:
 * g++ udp_object_detection.cpp -o udp_object_detection
 */

#include <iostream>  // For input/output operations (e.g., std::cout, std::cerr)
#include <string>    // For std::string manipulation (e.g., object name)
#include <vector>    // Not directly used in this version but often useful for collections
#include <sstream>   // Not directly used in this version but useful for string parsing/formatting
#include <cstring>   // For memcpy and memset, used for memory operations

// Required for networking functionalities
#include <sys/socket.h>  // For socket creation and manipulation
#include <netinet/in.h>  // For Internet address structures (e.g., sockaddr_in)
#include <unistd.h>      // For close() function
#include <arpa/inet.h>   // For INADDR_ANY and other network address conversions

// --- Data Structures ---

// A struct to hold the parsed detection data received over UDP.
// This structure defines the format of the object detection information.
struct ParsedDetection {
    int x, y;          // Top-left corner coordinates of the detected object's bounding box
    int width, height; // Dimensions of the detected object's bounding box
    std::string name;  // The name or label of the detected object (e.g., "person", "car")
    float confidence;  // The confidence score of the detection (0.0 to 1.0)
};

// --- UDP_AI_Receiver Class Definition ---

// The UDP_AI_Receiver class handles the establishment of a UDP socket,
// receiving incoming data, and parsing it into the ParsedDetection format.
class UDP_AI_Receiver {
public:
    // Constructor: Initializes the UDP receiver.
    // @param port The UDP port number to listen on.
    UDP_AI_Receiver(int port) {
        // Create a UDP socket.
        // AF_INET specifies the IPv4 address family.
        // SOCK_DGRAM specifies a UDP (datagram) socket.
        // 0 specifies the default protocol (UDP in this case).
        if ((sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            perror("Failed to create socket"); // Print error message if socket creation fails
            exit(EXIT_FAILURE);                 // Terminate the program
        }

        // Initialize the server address structure with zeros.
        memset(&servaddr_, 0, sizeof(servaddr_));
        // Set the address family to IPv4.
        servaddr_.sin_family = AF_INET;
        // Bind to any available network interface. INADDR_ANY allows receiving
        // packets sent to any of the host's IP addresses.
        servaddr_.sin_addr.s_addr = INADDR_ANY;
        // Convert the port number to network byte order and assign it.
        servaddr_.sin_port = htons(port);

        // Bind the created socket to the specified IP address and port.
        // This makes the socket listen for incoming UDP packets on that port.
        if (bind(sockfd_, (const struct sockaddr *)&servaddr_, sizeof(servaddr_)) < 0) {
            perror("Failed to bind socket"); // Print error message if binding fails
            exit(EXIT_FAILURE);                // Terminate the program
        }
    }

    // Destructor: Cleans up resources when the UDP_AI_Receiver object is destroyed.
    ~UDP_AI_Receiver() {
        // Close the socket to release system resources.
        close(sockfd_);
    }

    // Method to receive and parse a single detection packet.
    // @param detection_data A reference to a ParsedDetection struct where the received
    //                       and parsed data will be stored.
    // @return true if a detection packet was successfully received and parsed, false otherwise.
    bool receiveDetection(ParsedDetection& detection_data) {
        char buffer[MAX_BUFFER_SIZE]; // Buffer to hold incoming UDP data
        socklen_t len = sizeof(clientaddr_); // Size of the client address structure

        // Receive data from the socket.
        // The received data is placed into 'buffer'.
        // 'n' will store the number of bytes received.
        ssize_t n = recvfrom(sockfd_, (char*)buffer, MAX_BUFFER_SIZE, 0,
                             (struct sockaddr *)&clientaddr_, &len);

        if (n < 0) {
            perror("Failed to receive UDP message"); // Print error if reception fails
            return false;
        }

        // Attempt to parse the received data.
        return parseDetection(buffer, n, detection_data);
    }

private:
    // Helper method to parse the raw byte buffer into a ParsedDetection struct.
    // @param buffer A pointer to the raw byte buffer containing the received data.
    // @param size The number of bytes in the buffer.
    // @param detection_data A reference to a ParsedDetection struct to populate.
    // @return true if parsing was successful, false otherwise (e.g., malformed packet).
    bool parseDetection(const char* buffer, size_t size, ParsedDetection& detection_data) {
        // Define a start delimiter used to validate the beginning of a valid packet.
        // This helps in discarding malformed or unrelated UDP traffic.
        const uint32_t START_DELIMITER = 0xDDCCBBAA;
        // Define the minimum expected size of a valid packet, excluding the name string length.
        const size_t MIN_PACKET_SIZE = sizeof(START_DELIMITER) + sizeof(int32_t) * 4 + sizeof(uint8_t) + sizeof(float);

        // Check if the received packet is too small to contain basic detection data.
        if (size < MIN_PACKET_SIZE) {
            std::cerr << "Packet too small to be a valid detection." << std::endl;
            return false;
        }

        // Check for the presence of the start delimiter at the beginning of the packet.
        // This ensures the packet adheres to the expected format.
        if (*(uint32_t*)buffer != START_DELIMITER) {
            std::cerr << "Invalid packet: delimiter not found." << std::endl;
            return false;
        }

        size_t offset = sizeof(START_DELIMITER); // Initialize offset after the delimiter

        // Read x, y, width, and height (each as a 32-bit integer).
        // Data is directly cast and read from the buffer at the current offset.
        detection_data.x = *(int32_t*)(buffer + offset);
        offset += sizeof(int32_t);
        detection_data.y = *(int32_t*)(buffer + offset);
        offset += sizeof(int32_t);
        detection_data.width = *(int32_t*)(buffer + offset);
        offset += sizeof(int32_t);
        detection_data.height = *(int32_t*)(buffer + offset);
        offset += sizeof(int32_t);

        // Read the length of the object's name (as an 8-bit unsigned integer).
        uint8_t name_length = *(uint8_t*)(buffer + offset);
        offset += sizeof(uint8_t);

        // Check if the reported name length would exceed the received buffer size,
        // which indicates a corrupted or malicious packet.
        if (offset + name_length + sizeof(float) > size) {
            std::cerr << "Invalid packet: name length exceeds buffer size." << std::endl;
            return false;
        }
        // Extract the name string using the `assign` method of `std::string`,
        // which copies `name_length` characters starting from `buffer + offset`.
        detection_data.name.assign(buffer + offset, name_length);
        offset += name_length;

        // Read the confidence score (as a 32-bit float).
        detection_data.confidence = *(float*)(buffer + offset);

        return true; // Parsing successful
    }

private:
    int sockfd_;                        // Socket file descriptor
    struct sockaddr_in servaddr_;       // Server address structure
    struct sockaddr_in clientaddr_;     // Client address structure (for `recvfrom` to store sender's info)
    static const int MAX_BUFFER_SIZE = 1024; // Maximum size for the UDP receive buffer
};

// --- Example Usage of the Receiver Class ---

int main() {
    int port = 12347; // The UDP port number this receiver will listen on.
                      // Ensure this matches the port your sender is transmitting to.
    UDP_AI_Receiver receiver(port); // Create an instance of the UDP_AI_Receiver

    std::cout << "Listening for UDP detection packets on port " << port << "..." << std::endl;

    // Enter an infinite loop to continuously receive and process detection packets.
    while (true) {
        ParsedDetection detection; // Declare a ParsedDetection struct to hold the received data
        if (receiver.receiveDetection(detection)) { // Attempt to receive and parse a packet
            // If a packet was successfully received and parsed, print its contents.
            std::cout << "Received Detection:" << std::endl;
            std::cout << "  Box: (" << detection.x << ", " << detection.y << ", " << detection.width << ", " << detection.height << ")" << std::endl;
            std::cout << "  Name: " << detection.name << std::endl;
            std::cout << "  Confidence: " << detection.confidence << std::endl;
        }
        // In a real-world application, you might add a small delay here
        // (e.g., `usleep(10000);`) to prevent the loop from consuming 100% CPU
        // if packets are not arriving continuously.
    }

    return 0; // Program exits (though in the infinite loop, this line is unreachable)
}