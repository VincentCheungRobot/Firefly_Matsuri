#ifndef _PIN_DEF_H_
#define _PIN_DEF_H_

// bluetooth
#define esp_tx  PA_0
#define esp_rx  PA_1
#define esp_baud    115200

// CAN Bus / SPI
#define mcp2515_mosi_pin    PA_7
#define mcp2515_miso_pin    PA_6
#define mcp2515_clk_pin     PA_5
#define mcp2515_cs_pin      PB_6

// VESCs' ID
#define front_speed_ID      7
#define front_direction_ID  30
#define left_speed_ID       53
#define left_direction_ID   18
#define right_speed_ID      47
#define right_direction_ID  24

//  module's abs homing encoder
#define encoder_miso_pin        PC_11
#define encoder_clk_pin         PC_10
#define front_encoder_cs_pin    PC_0
#define left_encoder_cs_pin     PB_0
#define right_encoder_cs_pin    PC_1

// home encoder
// #define front_zero_deg      200.0f
// #define left_zero_deg       223.0f
// #define right_zero_deg      12.0f
#define front_zero_deg      143.0f
#define left_zero_deg       99.0f
#define right_zero_deg      101.0f

#endif