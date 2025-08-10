#include "mbed.h"


void Error_Handler(void)
{
    // 你可以在這裡加入錯誤提示，例如閃燈或進入死迴圈
    while (1)
    {
        // 假設你有 LED 接在某個腳位，可以在這裡閃爍
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(500);

    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** 設定電壓調節器為 Scale 1 模式（支援 180 MHz） **/
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // /** 啟用 HSE 並設定 PLL **/
    // RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    // RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    // RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    // RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;  // HSI = 16 MHz



    // 假設 HSE = 8 MHz
    // RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 360;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;  // 360 / 2 = 180 MHz
    RCC_OscInitStruct.PLL.PLLQ = 7;              // USB clock (optional)

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** 啟用 OverDrive 模式（某些晶片需要，但 F446 不需要） **/
    // HAL_PWREx_EnableOverDrive();  // F446 不需要這行

    /** 設定時鐘分配 **/
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;     // HCLK = 180 MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;      // PCLK1 = 45 MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;      // PCLK2 = 90 MHz

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}


BufferedSerial serial(PA_2, PA_3, 115200);  // TX, RX
// BufferedSerial serial(PA_0, PA_1, 115200);  // TX, RX

int main() {

    SystemClock_Config();

    char buf[64];
    while (true) {
        if (serial.readable()) {
            size_t len = serial.read(buf, sizeof(buf));
            if (len > 0) {
                serial.write(buf, len);  // Echo back
            }
        }
    }
}
