# Serial-communication
RS232 serial communication in C.

Usage:
To compile: ```gcc -o serial_com serial_com.c```
Running example : ```./serial_com -f your_file.txt -p /dev/tty```

I tested the code only on terminal device "/dev/tty" due to absence of an serial connection. One should select a real serial port like "/dev/ttyS0" or "/dev/ttyUSB0".
