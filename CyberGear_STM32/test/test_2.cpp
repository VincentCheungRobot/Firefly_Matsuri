#include "mbed.h"

// 定義兩個串口對象
BufferedSerial pc(PA_2, PA_3, 115200);  // TX, RX, 波特率

BufferedSerial uart2(PA_9, PA_10, 115200);  // TX, RX, 波特率

// 緩衝區大小和接收緩衝區
#define BUFFER_SIZE 32
char buffer[BUFFER_SIZE];


int initMessage(){
    const char message[] = "Starting UART forwarding...\r\n";
    ssize_t bytes_written = uart2.write(message, sizeof(message) - 1);  // 不包括結尾的 '\0'

    if (bytes_written < 0) {
        printf("Error writing data: %ld\n", bytes_written);
    } else {
        printf("Successfully written %ld bytes\n", bytes_written);
    }
}


int main() {

    initMessage();

//     while (true) {
//         // 檢查是否有數據從Python發送過來
//         if (pc.readable()) {
//             int index = 0;

//             // 接收數據，並確保不超過緩衝區大小
//             while (pc.readable() && index < BUFFER_SIZE) {
//                 buffer[index] = pc.getc(); // 從UART讀取一個字節
//                 index++;
//             }

//             // 將接收到的數據轉發到另一個UART
//             for (int i = 0; i < index; i++) {
//                 uart2.putc(buffer[i]); // 將數據發送到第二個UART
//             }

//             // 回顯接收到的數據（可選，用於調試）
//             buffer[index] = '\0'; // 確保緩衝區以NULL終止
//             pc.printf("Forwarded data: %s\r\n", buffer);
//         }
//     }
// }


while (true) {
    // 檢查是否有數據從Python發送過來
    if (pc.readable()) {
        int index = 0;

        // 接收數據，並確保不超過緩衝區大小
        while (pc.readable() && index < BUFFER_SIZE) {
            char c;
            pc.read(&c, 1);  // 從UART讀取一個字節
            buffer[index++] = c;
        }

        // 將接收到的數據轉發到另一個UART
        for (int i = 0; i < index; i++) {
            uart2.write(&buffer[i], 1);  // 將數據發送到第二個UART
        }

        // 回顯接收到的數據（可選，用於調試）
        buffer[index] = '\0'; // 確保緩衝區以NULL終止
        pc.write("Forwarded data: ", 17);
        pc.write(buffer, index);
        pc.write("\r\n", 2);
    }
}
}