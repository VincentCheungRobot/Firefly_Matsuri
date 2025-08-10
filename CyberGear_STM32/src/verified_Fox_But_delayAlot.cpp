#include "mbed.h"
#include "CAN3.h"
#include "math.h"

// #include "vesc.h"
// #include "motor_property.h"

#include "xm_motor.h"

#include "PS5.h"

// #include "pin_def.h"
// #include "setting_def.h"

// 緩衝區大小
// #define BUFFER_SIZE 4
// #define BUFFER_SIZE 100
// char rx_buffer[BUFFER_SIZE];
// char buffer[BUFFER_SIZE];


SPI spi(PA_7, PA_6, PA_5); // MOSI,MISO,SCLK
CAN3 can(spi, PB_6); //SSEL

xm_motor cybergear(1, &can);

float position_set = 0;


//VESC_init
    // vesc L(39, &can, 1000000, Motor_type::m3508); 
    // vesc R(25, &can, 1000000, Motor_type::m3508);

    float red_speed = 0;

//uart communicate
// BufferedSerial USB_uart(PA_2, PA_3, 115200);  // TX, RX, 波特率
// BufferedSerial USB_uart(esp_tx, esp_rx, esp_baud);
BufferedSerial USB_uart(PA_0, PA_1, 115200);
// BufferedSerial USB_uart(PC_12, PD_2, 115200);
// BufferedSerial USB_uart(PC_10, PC_11, 115200);
PS5 _ps5(&USB_uart);


void flush_uart_buffer()
{
    // 清空所有殘留資料
    while (USB_uart.readable()) {
        char dump[8];
        USB_uart.read(dump, sizeof(dump));
    }
}

void motor_init(int motor_id){
    cybergear.motor_enable(motor_id);
    cybergear.motor_pos_value(motor_id, 0); //Be careful not to get hit, because every time it is powered on it will record the current position as the zero point.
    cybergear.motor_pos_zero(motor_id); // Zeroing the encoder, it will register four revolutions, from -4pi to 4pi.
    cybergear.motor_mode(motor_id, 1);  // 1 position mode, 2 speed mode, 3 current mode, 0 motion mode
    cybergear.motor_pow_value(motor_id, 10, 20, 13, 0.2, 0.13); // id = 1 , speed = 10 rad/s , curr limit = 20 , Kp = 0.2, Kd = 0.13
    // cybergear.motor_pow_value(motor_id, 30, 23, 30, 1, 0.13); // motor CAN id, 位置模式速度限制 0~30rad/s , 速度位置模式电流限制 0~23A , 位置的kp (默认值30) , 电流的Kp (默认值0.125) , 电流的Ki (默认值0.0158)
};

void test_position(float angle1 = 0, float angle2 = 0, float angle3 = 0, float angle4 = 0){
    cybergear.motor_pos_value(1, -angle1);
    cybergear.motor_pos_value(2, angle2);
    cybergear.motor_pos_value(3, -angle3);
    cybergear.motor_pos_value(4, angle4);
}

 
// int steps = 314;
int steps = 32;
int motor_id = 2; //motor id also is the CAN ID ,You can set the ID of the cybeargear in the official debugger.

    int main(){
        // USB_uart.set_blocking(false);  // 非阻塞模式

        // HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
        // HAL_NVIC_EnableIRQ(UART4_IRQn);

        // RCC->CR |= RCC_CR_HSEON;                      // 啟用 HSE
        // while (!(RCC->CR & RCC_CR_HSERDY));           // 等待 HSE 穩定

        // RCC->PLLCFGR = (8 << RCC_PLLCFGR_PLLM_Pos)    // HSE / 8
        //             | (360 << RCC_PLLCFGR_PLLN_Pos)  // *360
        //             | (0 << RCC_PLLCFGR_PLLP_Pos)    // /2
        //             | RCC_PLLCFGR_PLLSRC_HSE;        // 使用 HSE 作為 PLL 輸入

        // RCC->CR |= RCC_CR_PLLON;                      // 啟用 PLL
        // while (!(RCC->CR & RCC_CR_PLLRDY));           // 等待 PLL 穩定

        // RCC->CFGR |= RCC_CFGR_SW_PLL;                 // 使用 PLL 作為系統時鐘
        // while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // 等待切換完成


        // const char message[] = "Hello, World!";
        // ssize_t bytes_written = uart.write(message, sizeof(message) - 1);  // not include the ending '\0'
    
        // if (bytes_written < 0) {
        //     printf("Error writing data: %ld\n", bytes_written);
        // } else {
        //     printf("Successfully written %ld bytes\n", bytes_written);
        // }

        ThisThread::sleep_for(2333ms); // wait for 2333ms (2.333s) For unknown reasons, my IDE displays an error, but the program actually runs normally


        bool Shake = 0;
        bool HeadMove = 1;

        can.frequency(CAN_1MBPS_8MHZ);

        motor_init(1);
        motor_init(2);
        motor_init(3);
        motor_init(4);

        /// printf("%d, %d, %d, %d, %d, %d, %d, %d\n", cybergear.canMsg.data[0], cybergear.canMsg.data[1], cybergear.canMsg.data[2], cybergear.canMsg.data[3], cybergear.canMsg.data[4], cybergear.canMsg.data[5], cybergear.canMsg.data[6], cybergear.canMsg.data[7]);
        test_position(position_set,position_set,position_set,position_set);
        
        // while (1){

        //     if (USB_uart.readable()) {
        //         size_t bytes_read = USB_uart.read(buffer, sizeof(buffer));
        //         // float red_speed = USB_uart.read(buffer, sizeof(buffer));
        //         red_speed = strtof(buffer, NULL);
                
        //         //Extremely large or small values: If the string represents a number that exceeds the range of float (for example, "1.0e500"), strtof may return positive infinity (+∞) or negative infinity (-∞).
        //         if (red_speed > 14000){
        //             red_speed = 14000;
        //         } else if (red_speed < 1 && red_speed > -1){
        //             red_speed = 0;
        //         }

        //         // const char* str = reinterpret_cast<const char*>(bytes_read);
        //     }
            
        //     if (red_speed < 2000){
        //         red_speed ++;
        //         L.comm_can_set_mrpm(red_speed);
        //         R.comm_can_set_mrpm(red_speed);
        //         ThisThread::sleep_for(6ms); // wait for 2333ms (2.333s) For unknown reasons, my IDE displays an error, but the program actually runs normally
        //     }else{
        //         red_speed = -2000;
        //         L.comm_can_set_mrpm(red_speed);
        //         R.comm_can_set_mrpm(red_speed);
        //         ThisThread::sleep_for(6ms);
        //     }

        // }






        //  while (1){
        //     for (int i=0;i<=steps;i++){
        //     cybergear.motor_pos_value(motor_id, 0.3*(M_PI*i/steps));
        //     ThisThread::sleep_for(1ms); // wait for 4ms (0.004s)
        //     }

        //     for (int i=0;i<=steps;i++){
        //     cybergear.motor_pos_value(motor_id, 0.3*(M_PI-M_PI*i/steps));
        //     ThisThread::sleep_for(1ms); // wait for 4ms (0.004s)
        //     }

        //     }





         while (1){

            // turn the cmd to persentage
            // float x_pre = (float)(abs(_ps5.getRX() * M_PI / 256.0f));
            // float y_pre = (float)(abs;(_ps5.getRY() * M_PI / 256.0f));

            for (int i=0;i<=steps;i++){
                // get PS5 command
                _ps5.decode();

                if(_ps5.isLeftPressed()){
                    HeadMove = 1;
                }else if(_ps5.isUpPressed()){
                    HeadMove = 0;
                }

                if(_ps5.isTrianglePressed()){
                    Shake =1;
                }else if(_ps5.isCirclePressed()){
                    Shake =0;
                }

                if (HeadMove == 1){
                    float x_pre = (float)(_ps5.getRX() * M_PI/4 / 128.0f);
                    float y_pre = (float)(_ps5.getRY() * M_PI/4 / 128.0f);
                    cybergear.motor_pos_value(2, x_pre);
                    cybergear.motor_pos_value(3, y_pre);
                }else{
                    float x_pre = x_pre;
                    float y_pre = y_pre;
                }
                
                if (Shake == 1){
                    cybergear.motor_pos_value(1, M_PI/4 * sin(i* 4*M_PI/steps));
                }
            // ThisThread::sleep_for(4ms); // wait for 4ms (0.004s) For unknown reasons, my IDE displays an error, but the program actually runs normally
            
            // 清空舊資料，只保留最新一筆
            flush_uart_buffer();
            }

            }
        }
    