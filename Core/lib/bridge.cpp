#include "bridge.h"

void setup() {
    // C/C++ 코드 사용
}

void loop() {
    // C/C++ 코드 사용
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    HAL_Delay(200);
}
