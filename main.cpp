/**
 Interfacing DS18B20 KY-001 Temperature sensor using nrf52840dk and reading temperature value in serial Terminal
 @author:   Vishnu Priya P
 @PSno: 40019211
 @date: 23.11.2023
*/

#include <stdio.h>
#include "D:/VISHNUPRIYA_WORKSPACE-1/SDK52_DeviceDownload/nRF5_SDK_17.1.0_ddde560/components/libraries/delay/nrf_delay.h"
#include "D:/VISHNUPRIYA_WORKSPACE-1/SDK52_DeviceDownload/nRF5_SDK_17.1.0_ddde560/modules/nrfx/hal/nrf_gpio.h"
#include <zephyr/kernel.h> 
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include "sensor.cpp"
#include "sensor.h"

int main(void)
{
    static float temp = 0;
    DS18B20 DS18B20;
    DS18B20.ds18b20_setResolution(12);

    

    while(1)
    {
        

        temp = DS18B20.getTemperature();
        printk("Temperature: %.3f \r\n", temp);
        nrf_delay_ms(1000);

    }
}