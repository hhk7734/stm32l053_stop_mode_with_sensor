#include "bridge.h"

#include "gpio.h"
#include "usart.h"

void setup() {
    // C/C++ 코드 사용
}

void loop() {
    // C/C++ 코드 사용
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    HAL_Delay(200);
}

#ifdef __cplusplus
extern "C" int _write(int32_t file, uint8_t *ptr, int32_t len) {
#else
int _write(int32_t file, uint8_t *ptr, int32_t len) {
#endif
    if(HAL_UART_Transmit(&huart1, ptr, len, len) == HAL_OK) return len;
    else
        return 0;
}