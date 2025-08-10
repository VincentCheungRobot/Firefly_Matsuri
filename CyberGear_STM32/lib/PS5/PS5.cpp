#include "mbed.h"
#include "PS5.h"

PS5::PS5(){}

PS5::PS5(BufferedSerial* ser){
    this->ser = ser;
}

void PS5::encode(){

}

void PS5::decode(){
    readData();

    this->incomingMsg = buffer;

    if(incomingMsg[0] != start_byte){
        return;
    }

    // if(this->CRCverify() != 0){
    //     return;
    // }

    //clear
    controller = {};

    uint8_t op1 = incomingMsg[7];
    uint8_t op2 = incomingMsg[8];

    controller.Lx = (int8_t)incomingMsg[1];
    controller.Ly = (int8_t)incomingMsg[2];
    controller.Rx = (int8_t)incomingMsg[3];
    controller.Ry = (int8_t)incomingMsg[4];
    controller.L2 = (uint8_t)incomingMsg[5];
    controller.R2 = (uint8_t)incomingMsg[6];

    controller.Right = (op1 & Right_butt_mask) != 0;
    controller.Left = (op1 & Left_butt_mask) != 0;
    controller.Up = (op1 & Up_butt_mask) != 0;          
    controller.Down = (op1 & Down_butt_mask) != 0;      
    controller.Square = (op1 & Square_butt_mask) != 0;  
    controller.Cross = (op1 & Cross_butt_mask) != 0;    
    controller.Circle = (op1 & Circle_butt_mask) != 0;  
    controller.Triangle = (op1 & Trangle_butt_mask) != 0; 

    controller.L1 = (op2 & L1_butt_mask) != 0;
    controller.R1 = (op2 & R1_butt_mask) != 0;
    controller.Share = (op2 & Share_butt_mask) != 0;          
    controller.Options = (op2 & Options_butt_mask) != 0;      
    controller.PSButton = (op2 & PsButton_butt_mask) != 0;  
    controller.PSTouchpad= (op2 & PsTouchpad_butt_mask) != 0;    
    controller.L3 = (op2 & L3_butt_mask) != 0;  
    controller.R3 = (op2 & R3_butt_mask) != 0; 
    
    

}

// int PS5::CRCverify(){
//     int result = 0;
    
//     return result;
// }


bool PS5::isXpressed() {
    return controller.Cross; 
}

bool PS5::isSquarePressed() {
    return controller.Square; 
}

bool PS5::isCirclePressed() {
    return controller.Circle; 
}

bool PS5::isTrianglePressed() {
    return controller.Triangle; 
}

bool PS5::isLeftPressed() {
    return controller.Left; 
}

bool PS5::isRightPressed() {
    return controller.Right; 
}

bool PS5::isUpPressed() {
    return controller.Up; 
}

bool PS5::isDownPressed() {
    return controller.Down; 
}

bool PS5::isL1Pressed() {
    return controller.L1; 
}

bool PS5::isR1Pressed() {
    return controller.R1; 
}

bool PS5::isSharePressed() {
    return controller.Share; 
}

bool PS5::isOptionsPressed() {
    return controller.Options; 
}

bool PS5::isPSButtonPressed() {
    return controller.PSButton; 
}

bool PS5::isPSTouchpadPressed() {
    return controller.PSTouchpad; 
}

bool PS5::isL3Pressed() {
    return controller.L3; 
}

bool PS5::isR3Pressed() {
    return controller.R3; 
}

uint8_t PS5::getL2Value() {
    return controller.L2; 
}

uint8_t PS5::getR2Value() {
    return controller.R2; 
}

int8_t PS5::getLX() {
    return controller.Lx; 
}

int8_t PS5::getLY() {
    return controller.Ly; 
}

int8_t PS5::getRX() {
    return controller.Rx; 
}

int8_t PS5::getRY() {
    return controller.Ry; 
}
void PS5::readData(){
    ser->read(buffer, sizeof(buffer));
}