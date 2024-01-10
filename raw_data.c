#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int main() {
    int serial_port = open("/dev/ttyS0", O_RDONLY);
    if (serial_port < 0) {
        return 1;
    }

    struct termios tty;
    if(tcgetattr(serial_port, &tty) != 0) {
        return 1;
    }

    cfsetispeed(&tty, B230400);
    cfsetospeed(&tty, B230400);

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

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        return 1;
    }

    // Allocate memory for read buffer, set size according to your needs
    unsigned char read_buf [8];

    while (1) {
        memset(&read_buf, '\0', sizeof(read_buf));

        // Normally you would do at least one read to get data from the serial port
        int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

        // n is the number of bytes read into the buffer
        if (num_bytes < 0) {
            continue;
        }

        // Print what was read in hexadecimal and clear the terminal if 54 is printed
        for (int i = 0; i < num_bytes; ++i) {
            if (read_buf[i] == 0x54) {
                system("clear");
            }
            printf("%02x ", read_buf[i]);
            printf("\n");
        }
    }

    close(serial_port);
    return 0;
}
