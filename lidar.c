#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <signal.h>
#include <bcm2835.h>

// Define constants based on the LiDAR data protocol
#define START_CHARACTER 0x54
#define DATA_LENGTH 47

int serial_port;

// Global variable to control the main loop
volatile sig_atomic_t keep_running = 1;

// Signal handler for SIGINT
void handle_sigint(int sig) {
    keep_running = 0;
}

// Function to initialize the LiDAR sensor
void initializeLidar() {
    struct termios tty;

    // Initialize BCM2835
    if (!bcm2835_init()) return 1;

    // Set pin mode for GPIO pin 17 to out
    bcm2835_gpio_fsel(RPI_GPIO_P1_11, BCM2835_GPIO_FSEL_OUTP);

    // Open the serial port in read/write mode
    serial_port = open("/dev/ttyS0", O_RDWR);
    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        exit(1);
    }

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        exit(1);
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 230400
    cfsetispeed(&tty, B230400);
    cfsetospeed(&tty, B230400);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        exit(1);
    }
}

// Function to read a data packet from the LiDAR sensor
void readLidarDataPacket(unsigned char *dataPacket) {
    static int packetIndex = 0;
    static int isReadingPacket = 0;

    unsigned char read_byte;
    int bytesRead;

    while ((bytesRead = read(serial_port, &read_byte, 1)) > 0) {

        if (read_byte == START_CHARACTER) {
            printf("Starting new packet.\n");
            packetIndex = 0;
            isReadingPacket = 1;
        }

        if (isReadingPacket) {
            dataPacket[packetIndex] = read_byte;
            packetIndex++;

            if (packetIndex == DATA_LENGTH) {
                printf("End of current packet, processing data...\n");
                processLidarData(dataPacket);
                isReadingPacket = 0;
                packetIndex = 0;
            }
        }
    }

    if (bytesRead < 0) {
        printf("Error %i from read: %s\n", errno, strerror(errno));
    }
}

// Function to process the LiDAR data and convert it into a top-down view
void processLidarData(unsigned char *data) {
    FILE *file = fopen("/home/pi/lidar/lidar_data.txt", "a"); // Open the file in append mode
    if (file == NULL) {
        printf("Error opening file!\n");
        exit(1);
    }

    // Implement data processing logic here
    int startAngle = (data[5] << 8) | data[4];
    // Map the startAngle value between 0 and 360
    startAngle = startAngle / 100;
    fprintf(file, "Start angle: %d\n", startAngle);
    unsigned char groups[12][3];
    for (int i = 0; i < 12; i++) {
        groups[i][0] = data[6 + i*3];
        groups[i][1] = data[7 + i*3];
        groups[i][2] = data[8 + i*3];
    }
    for (int i = 0; i < 12; i++) {
        int distance = (groups[i][1] << 8) | groups[i][0];
        int quality = groups[i][2];
        // Compute the angle for each data point
        float angleIncrement = 360.0 / 12.0;
        float angle = startAngle + (i * angleIncrement);
        if (angle >= 360.0) {
            angle -= 360.0;
        }

        // Flash LED if something is close
        if (distance <= 100) {
            bcm2835_gpio_write(RPI_GPIO_P1_11, HIGH);
        } else {
            bcm2835_gpio_write(RPI_GPIO_P1_11, LOW);
        }

        fprintf(file, "Angle: %.2f, Distance: %d, Quality: %d\n", angle, distance, quality);
    }

    // Check if we have completed a rotation
    static int lastEndAngle = 0;
    int endAngle = (data[43] << 8) | data[42];
    endAngle = endAngle / 100;

    // If the end angle is less than the last end angle, we assume a rotation has completed
    if (endAngle < lastEndAngle) {
        fprintf(file, "Rotation Complete\n");
    }

    lastEndAngle = endAngle;
    fclose(file); // Close the file
}

int main() {
    // Set up signal handler
    signal(SIGINT, handle_sigint);

    // Initialize LiDAR
    initializeLidar();

    unsigned char dataPacket[DATA_LENGTH]; // Adjust size as necessary based on the data packet structure

    while (keep_running) {
        readLidarDataPacket(dataPacket);
        // processLidarData(dataPacket);

        usleep(10); // Example: 10us delay
    }

    // Close the serial port and terminate GPIO
    close(serial_port);
    bcm2835_close();

    return 0;
}
