#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fcntl.h>
#include <termios.h>  /* POSIX terminal control definitions */
#include <unistd.h>   /* UNIX standard function definitions */


#define BUFFER_SIZE 12 // Data size -> Higher values for high sized files.
#define HEADER_SIZE 2  // Size of the header/preamble
#define FRAME_DELAY_US 10000  // 10 milliseconds delay between frames

void printUsage() {
    printf("Usage: file_transfer -p <serial_port> -f <file_path>\n");
}

// Function to calculate the checksum of a frame                    // Better to use more complex checksum calculation. Exp: MD5 Hash
unsigned char calculateChecksum(const char* data, size_t size) {
    unsigned char checksum = 0;
    for (size_t i = 0; i < size; i++) {
        checksum += data[i];
    }
    return checksum;
}

// Send every frame using serial port.
void sendOverRS232(const char* data, size_t size, int rs232_interface) {

    // Write each frame
    write(rs232_interface, data, size);

    // Delay between frames
    usleep(FRAME_DELAY_US);
}


// Arrange termios settings. More info on serial comm. guide: https://www.ing.iac.es/%7Edocs/external/serial/serial.pdf
int openSerialPort(char* portName) {
    int serialPort = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serialPort == -1) {
        perror("Error opening serial port");
        exit(EXIT_FAILURE);
    }

    struct termios options;
    tcgetattr(serialPort, &options);

    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    tcsetattr(serialPort, TCSANOW, &options);

    return serialPort;
}

// Frame convention:  <Header/Preamble:"AB"><Data><Checksum><Packet ID>, One should parse each frame at the receiver.

void sendFile(FILE* file, int serialPort) {
    char buffer[BUFFER_SIZE];
    size_t bytesRead;
    unsigned char count = 32;  // starts from ascii.
    // Read and send data in chunks
    do {
        bytesRead = fread(buffer, 1, sizeof(buffer), file);
        count++;
        if (bytesRead > 0) {
            // Create a frame with the header/preamble, data, and checksum for each frame
            char frame[BUFFER_SIZE + HEADER_SIZE + 1 + 1 ];  // +1 for the checksum, +1 packet number(id), {you can even add packet length for data integrity.}
            memcpy(frame, "AB", HEADER_SIZE);
            memcpy(frame + HEADER_SIZE, buffer, bytesRead);
            frame[bytesRead + HEADER_SIZE ] = calculateChecksum(buffer, bytesRead);
            frame[bytesRead + HEADER_SIZE + 1  ] = count; // packet ID

            //for(char k=0; k<16;k++){  // Debugging for sent frame.
            //    printf("%c",frame[k]);
            //}
            //printf("\n");

            sendOverRS232(frame, bytesRead + HEADER_SIZE + 1 + 1 , serialPort);
        }
    } while (bytesRead > 0);

}

// Main func
int main(int argc, char* argv[]) {
    char* portName = NULL;
    char* filePath = NULL;

    // Parse command-line arguments
    for (int i = 1; i < argc; i += 2) {
        if (i + 1 < argc) {
            if (strcmp(argv[i], "-p") == 0) { // Port parser
                portName = argv[i + 1];
            } else if (strcmp(argv[i], "-f") == 0) { // File parser
                filePath = argv[i + 1];
            } else {
                printUsage();
                return -1;
            }
        } else {
            printUsage();
            return -1;
        }
    }

    if (portName == NULL || filePath == NULL) {
        printUsage();
        return -1;
    }

    // Open serial port.
    int serialPort = openSerialPort(portName);

    FILE* file = fopen(filePath, "rb");
    if (file == NULL) {
        perror("Error opening file");
        close(serialPort);
        return -1;
    }

    // Read and send whole file.
    sendFile(file, serialPort);

    // Close serial port and file after transmission.
    fclose(file);
    close(serialPort);

    printf("File transfer complete.\n");

    return 0;
}
