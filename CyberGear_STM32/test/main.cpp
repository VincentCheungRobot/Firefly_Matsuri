#include "mbed.h"
#include "CAN3.h"
#include "math.h"

#include "vesc.h"
#include "motor_property.h"

#include "xm_motor.h"

SPI spi(PA_7, PA_6, PA_5); // MOSI,MISO,SCLK
CAN3 can(spi, PB_6); //SSEL

xm_motor cybergear(1, &can);


//VESC_init
    vesc L(39, &can, 1000000, Motor_type::m3508); 
    vesc R(25, &can, 1000000, Motor_type::m3508);

void motor_init(int motor_id){
    cybergear.motor_enable(motor_id);
    cybergear.motor_pos_value(motor_id, 0); //Be careful not to get hit, because every time it is powered on it will record the current position as the zero point.
    cybergear.motor_pos_zero(motor_id); // Zeroing the encoder, it will register four revolutions, from -4pi to 4pi.
    cybergear.motor_mode(motor_id, 1);  // 1 position mode, 2 speed mode, 3 current mode, 0 motion mode
    // cybergear.motor_pow_value(motor_id, 10, 20, 0.2, 0.13); // id = 1 , speed = 10 rad/s , curr limit = 20 , Kp = 0.2, Kd = 0.13
    cybergear.motor_pow_value(motor_id, 30, 23, 30, 1, 0.13); // motor CAN id, 位置模式速度限制 0~30rad/s , 速度位置模式电流限制 0~23A , 位置的kp (默认值30) , 电流的Kp (默认值0.125) , 电流的Ki (默认值0.0158)
};

void test_position(int angle = 0){
    cybergear.motor_pos_value(1, -angle);
    cybergear.motor_pos_value(2, angle);
    cybergear.motor_pos_value(3, -angle);
    cybergear.motor_pos_value(4, angle);
}

int steps = 1000;
int motor_id = 1; //motor id also is the CAN ID ,You can set the ID of the cybeargear in the official debugger.

    int main(){
        ThisThread::sleep_for(2333ms); // wait for 2333ms (2.333s) For unknown reasons, my IDE displays an error, but the program actually runs normally

        can.frequency(CAN_1MBPS_8MHZ);

        motor_init(1);
        motor_init(2);
        motor_init(3);
        motor_init(4);

        /// printf("%d, %d, %d, %d, %d, %d, %d, %d\n", cybergear.canMsg.data[0], cybergear.canMsg.data[1], cybergear.canMsg.data[2], cybergear.canMsg.data[3], cybergear.canMsg.data[4], cybergear.canMsg.data[5], cybergear.canMsg.data[6], cybergear.canMsg.data[7]);

        test_position(0);
        
        while (1){  
            L.comm_can_set_mrpm(1000);
            R.comm_can_set_mrpm(1000);
        }
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
            
        }