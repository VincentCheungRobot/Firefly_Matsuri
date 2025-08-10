#include "mbed.h"
#include "CAN3.h"
#include "math.h"

#include "vesc.h"
#include "motor_property.h"

#include "xm_motor.h"
// 假設使用的串口
// BufferedSerial pc(PA_2, PA_3, 19200); // TX, RX, Baud
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


float value1 = 0.0;
float value2 = 0.0;
//about cybergear motor
xm_motor cybergear(1, &can);
float position_set = 0;

// int motor_id = 1; //motor id also is the CAN ID ,You can set the ID of the cybeargear in the official debugger.

void motor_init(int motor_id){
    cybergear.motor_enable(motor_id);
    cybergear.motor_pos_value(motor_id, 0); //Be careful not to get hit, because every time it is powered on it will record the current position as the zero point.
    cybergear.motor_pos_zero(motor_id); // Zeroing the encoder, it will register four revolutions, from -4pi to 4pi.
    cybergear.motor_mode(motor_id, 1);  // 1 position mode, 2 speed mode, 3 current mode, 0 motion mode
    // cybergear.motor_pow_value(motor_id, 10, 20, 0.2, 0.13); // id = 1 , speed = 10 rad/s , curr limit = 20 , Kp = 0.2, Kd = 0.13
    cybergear.motor_pow_value(motor_id, 30, 23, 30, 1, 0.13); // motor CAN id, 位置模式速度限制 0~30rad/s , 速度位置模式电流限制 0~23A , 位置的kp (默认值30) , 电流的Kp (默认值0.125) , 电流的Ki (默认值0.0158)
};

void test_position(float angle1 = 0, float angle2 = 0, float angle3 = 0, float angle4 = 0){
    cybergear.motor_pos_value(1, angle1);
    cybergear.motor_pos_value(2, angle2);
    cybergear.motor_pos_value(3, angle3);
    cybergear.motor_pos_value(4, angle4);
}

int main() {

    can.frequency(CAN_1MBPS_8MHZ);

    const char message[] = "Hello, World!";
    pc.write(message, sizeof(message) - 1);  // 不包括結尾的 '\0'

    ThisThread::sleep_for(233ms); // wait for 233ms (0.233s) For unknown reasons, my IDE displays an error, but the program actually runs normally

    motor_init(1);
    motor_init(2);
    motor_init(3);
    motor_init(4);
    test_position(position_set,position_set,position_set,position_set);

    pc.set_format(8, BufferedSerial::None, 1); // 設置串口格式：8數據位，無校驗，1停止位

    while (true) {

        memset(buffer, 0, FRAME_SIZE); // 將緩衝區的所有字節設置為0
    
        // 讀取幀數據
        if (pc.readable()){

            // ThisThread::sleep_for(89ms);

            size_t bytesRead = pc.read(buffer, FRAME_SIZE);
            // pc.read(buffer, FRAME_SIZE);
            
            // ThisThread::sleep_for(64ms);


                char output[50]; // 用於存儲輸出字符串，并且把每一個緩存的字節都打印出來，用這個方法可以防止數據丟失。

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

                    memcpy(&value1, &buffer[2], 4);
                    memcpy(&value2, &buffer[6], 4);


                // L.comm_can_set_mrpm(value1);
                // R.comm_can_set_mrpm(value2);

                test_position(0, value1/1000, value2/1000, 0);
                }
                
            }
        }
    }
// }