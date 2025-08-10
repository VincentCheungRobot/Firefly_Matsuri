#include "mbed.h"

BufferedSerial USB_uart(PA_2, PA_3, 115200);  // TX, RX, 波特率

BufferedSerial uart(PA_9, PA_10, 115200);  // TX, RX, 波特率

// 緩衝區大小
#define BUFFER_SIZE 100
// char rx_buffer[BUFFER_SIZE];
char buffer[BUFFER_SIZE];

int main() {
    const char message[] = "Hello, World!";
    ssize_t bytes_written = uart.write(message, sizeof(message) - 1);  // 不包括結尾的 '\0'

    if (bytes_written < 0) {
        printf("Error writing data: %ld\n", bytes_written);
    } else {
        printf("Successfully written %ld bytes\n", bytes_written);
    }

    while (true) {
        // Main loop
        if (USB_uart.readable()) {
        size_t bytes_read = USB_uart.read(buffer, sizeof(buffer));

        // 將數據通過 PA_9, PA_10 發送
        if (bytes_read > 0) {
            uart.write(buffer, bytes_read);
            ThisThread::sleep_for(10ms);  // 防止過高的輪詢頻率
        }
    }
}
}