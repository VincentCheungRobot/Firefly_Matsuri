#include "mbed.h"
#include "CAN3.h"
#include "vesc.h"
#include "motor_property.h"
// 假設使用的串口
// BufferedSerial pc(PA_2, PA_3, 115200); // TX, RX, Baud
// BufferedSerial pc(PA_2, PA_3, 921600); // TX, RX, Baud
BufferedSerial pc(PA_2, PA_3, 256000); // TX, RX, Baud


SPI spi(PA_7, PA_6, PA_5); // MOSI,MISO,SCLK
CAN3 can(spi, PB_6); //SSEL

//VESC_init
vesc L(39, &can, 1000000, Motor_type::m3508); 
vesc R(25, &can, 1000000, Motor_type::m3508);

// 定義幀格式
#define FRAME_HEADER_1 0xAA
#define FRAME_HEADER_2 0x55
// #define FRAME_MID 0xF1
#define FRAME_END 0x0A
#define FRAME_SIZE 11 // 總共的幀長度 (2字節標頭 + 2個float + 1字節中間標記)

// 缓冲区
uint8_t buffer[FRAME_SIZE] = {0}; // 清零初始化


float value1 = 900.0;
// void* ptr = &value1;  // Create a pointer to the float
float value2 = 1000.0;

// 函數：將4字節的數據轉換為float
float bytesToFloat(uint8_t *bytes) {
    float value;
    memcpy(&value, bytes, 4); // 使用內存拷貝進行轉換
    return value;
}

int main() {
    pc.set_format(8, BufferedSerial::None, 1); // 設置串口格式：8數據位，無校驗，1停止位

    can.frequency(CAN_1MBPS_8MHZ);

    const char message[] = "Hello, World!";
    pc.write(message, sizeof(message) - 1);  // 不包括結尾的 '\0'


    while (true) {


        // float value1 = bytesToFloat(&buffer[2]); // 第1個float值
        // float value2 = bytesToFloat(&buffer[7]); // 第2個float值

        // L.comm_can_set_mrpm(value1);
        // R.comm_can_set_mrpm(value2);




        memset(buffer, 0, FRAME_SIZE); // 將緩衝區的所有字節設置為0
    
        // 讀取幀數據
        if (pc.readable()){
    
            // ThisThread::sleep_for(1ms);

        // < FRAME_SIZE) {
        //         ThisThread::sleep_for(1ms); // 等待數據準備好
        //     }else{
            
            size_t bytesRead = pc.read(buffer, FRAME_SIZE);
            // if (bytesRead == FRAME_SIZE) {
                // if (memcpy(&value2, &buffer[2], sizeof(float)) == 0 || memcpy(&value2, &buffer[2], sizeof(float)) == 0){
                //     sprintf(output, "Memcpy Fail!");
                //     pc.write(output, strlen(output));
                // }else{
                //     sprintf(output, "Memcpy Ok!");
                //     pc.write(output, strlen(output));            
                // }

                char output[50]; // 用於存儲輸出字符串，并且把每一個緩存的字節都打印出來，用這個方法可以防止數據丟失。

                // Print First Float
                // sprintf(output, "Received float 1 in x: %d\r\n", value1);
                // pc.write(output, strlen(output));
                // Print Second Float
                // sprintf(output, "Received float 2 in lf: %f\r\n", value2);
                // pc.write(output, strlen(output));
                sprintf(output, "Received bytesRead: %d\r\n", bytesRead);
                pc.write(output, strlen(output));
                sprintf(output, "Received bytes0: %d\r\n", buffer[0]);
                pc.write(output, strlen(output));
                sprintf(output, "Received bytes1: %d\r\n", buffer[1]);
                pc.write(output, strlen(output));
                sprintf(output, "Received bytes2: %d\r\n", buffer[2]);
                pc.write(output, strlen(output));
                sprintf(output, "Received bytes3: %d\r\n", buffer[3]);
                pc.write(output, strlen(output));
                sprintf(output, "Received bytes4: %d\r\n", buffer[4]);
                pc.write(output, strlen(output));
                sprintf(output, "Received bytes5: %d\r\n", buffer[5]);
                pc.write(output, strlen(output));
                sprintf(output, "Received bytes6: %d\r\n", buffer[6]);
                pc.write(output, strlen(output));
                sprintf(output, "Received bytes7: %d\r\n", buffer[7]);
                pc.write(output, strlen(output));
                sprintf(output, "Received bytes8: %d\r\n", buffer[8]);
                pc.write(output, strlen(output));
                sprintf(output, "Received bytes9: %d\r\n", buffer[9]);
                pc.write(output, strlen(output));
                sprintf(output, "Received bytes10: %d\r\n", buffer[10]);
                pc.write(output, strlen(output));
                sprintf(output, "Received bytes11: %d\r\n", buffer[11]);
                pc.write(output, strlen(output));
                // sprintf(output, "Received bytes12: %d\r\n", buffer[12]);
                // pc.write(output, strlen(output));
                // sprintf(output, "Received bytes13: %d\r\n", buffer[13]);
                // pc.write(output, strlen(output));
                // sprintf(output, "Received bytes14: %d\r\n", buffer[14]);
                // pc.write(output, strlen(output));
                // sprintf(output, "Received bytes15: %d\r\n", buffer[15]);
                // pc.write(output, strlen(output));
                
                // 驗證幀標頭和中間標記，這非常重要，有時候傳輸數據出問題了，可能就會導致memcpy了一個錯誤的數值給vesc的速度控制器，有可能因此導致硬件損壞。
                if (buffer[0] == FRAME_HEADER_1 && buffer[1] == FRAME_HEADER_2 && buffer[10] == FRAME_END) {
                    // 解碼兩個float
                    // value1 = bytesToFloat(&buffer[2]); // 第1個float值
                    // value2 = bytesToFloat(&buffer[7]); // 第2個float值

                    memcpy(&value1, &buffer[2], 4);
                    memcpy(&value2, &buffer[6], 4);

                    // memcpy(&ptr, &buffer[2], sizeof(float));
                    // value1 = *(float*)ptr;
// }

                L.comm_can_set_mrpm(value1);
                R.comm_can_set_mrpm(value2);                    
                }
            }
        }
    }
// }