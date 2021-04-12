#include "bridge.h"

#include "adc.h"
#include "gpio.h"
#include "hal_bme280.h"
#include "hal_ccs811.h"
#include "hal_tcs34725.h"
#include "i2c.h"
#include "main.h"
#include "rtc.h"
#include "usart.h"

#include <cstdio>

CCS811   ccs811;
BME280   bme280;
TCS34725 tcs34725;

RTC_TimeTypeDef  sTime;
RTC_DateTypeDef  sDate;
RTC_AlarmTypeDef sAlarm;

void check_battery_and_enter_stop();

void setup() {
    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
    HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

    if(! ccs811.begin(&hi2c1)) { printf("Failed to init CCS811\n"); }
    if(! bme280.begin(&hi2c1)) { printf("Failed to init BME280\n"); }
    if(! tcs34725.begin(&hi2c1)) { printf("Failed to init TCS34725\n"); }
}

void loop() {
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    HAL_Delay(200);

    if(ccs811.dataAvailable()) {
        uint16_t r, g, b, c;

        ccs811.readAlgorithmResults();
        printf("CO2: %d\n", ccs811.getCO2());
        printf("TVOC: %d\n", ccs811.getTVOC());
        printf("Temp: %.2f\n", bme280.readTemperature());
        printf("Press: %.2f\n", bme280.readPressure() / 100.0f);
        printf("Humid: %.2f\n", bme280.readHumidity());
        tcs34725.getRawData(&r, &g, &b, &c);
        printf("RGBC: (%d, %d, %d, %d)\n", r, g, b, c);
        printf("ColorTemp: %d\n",
               tcs34725.calculateColorTemperature_dn40(r, g, b, c));
        printf("lux: %d\n", tcs34725.calculateLux(r, g, b));
        printf("\n");

        check_battery_and_enter_stop();
    }
}

void check_battery_and_enter_stop() {
    HAL_ADC_Start(&hadc);
    HAL_ADC_PollForConversion(&hadc, 1000);
    float v_bat = 3.3f * (HAL_ADC_GetValue(&hadc) / 4096.0);
    HAL_ADC_Stop(&hadc);

    printf("BAT: %.2f V\n", v_bat);
    if(v_bat < 2.1f) {
        HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
        printf("%02d:%02d:%02d:%02d\n",
               sTime.Hours,
               sTime.Minutes,
               sTime.Seconds,
               100
                   - (int)(100.0f * (sTime.SubSeconds)
                           / (sTime.SecondFraction + 1)));

        sAlarm.AlarmTime.Seconds        = (sTime.Seconds + 5) % 60;
        sAlarm.AlarmTime.SubSeconds     = sTime.SubSeconds;
        sAlarm.AlarmTime.SecondFraction = sTime.SecondFraction;
        sAlarm.AlarmMask          = RTC_ALARMMASK_ALL & ~RTC_ALARMMASK_SECONDS;
        sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_SS14_3;
        sAlarm.Alarm              = RTC_ALARM_A;
        HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN);

        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
        printf("Stop mode\n\n");
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);

        SystemClock_Config();

        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
        printf("Wake up from Stop mode\n");

        HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
        printf("\n");
    }
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