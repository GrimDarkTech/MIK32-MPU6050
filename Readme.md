# MIK32-MPU6050

## Установка

Перейдите в директорию **libs**, затем скопируйте ее содержимое в **libs** вашего PlatformIO проекта.

Содерижимое **libs**:
- I2Cdev - MIK32 порт STM32 HAL порта Arduino библиотек с набором дополнительных функции для работы с I2C. Дополнительно содержит STM32 HAL Connector - набор функций, которых нет в MIK32 HAL, но оны нужны для работы I2Cdev;
- MPU6050 - драйвер для базовой работы с MPU6050, в том числе с DMP;
- MotionApps - набор библиотек, расширяющих возможности работы с MPU6050, в том числе с DMP;
    - helper_3dmath - математика векторов и кватернионов;
    - MotionApps20 - расширенные функции для работы с MPU6050, в основном DMP.
- Tityformat - функция форматирования с-string, работает с примитивными типами (не обязательна к установке в ваш проект)

## Использование

```c
/*
Ваши заголовки
*/
#include "MotionApps20.h"
#include "MPU6050.h"
#include "I2Cdev.h"

/*
Ваши глобальные переменные
*/
uint16_t packetSize = 42;    // размер DMP пакета (по умолчанию 42 байта)
uint8_t fifoBuffer[64]; // буфер FIFO

/*
Ваши объявления функций
*/

int main()
{
    /*
    Ваш код, в том числе с инициализацией I2C контроллера (в рассматриваемом примере hi2c0)
    */

    //Настройка указателя на I2C
    I2Cdev_init(&hi2c0);

    //Настройка адреса I2C для MPU6050 можно определить I2C сканером или через Datasheet
    MPU6050_setAddress(MPU6050_ADDRESS_AD0_HIGH);

    //Работу MPU6050 на шине можно проверить через вызов MPU6050_testConnection(), функция вернет True, если устройство доступно по адресу, заданному выше через MPU6050_setAddress(dev_address);

    // Задайте количество попыток запуска проверки подключения. Обычно, если все хорошо - подключение проходит тест с первого раза
    uint8_t timeout = 50;
    while(!MPU6050_testConnection() && timeout > 2)
    {
        // ID устройства можно получить через вызов MPU6050_getDeviceID();

        // Задайте время между попытками запуска проверки подключения
        HAL_DelayMs(500);
        timeout -= 1;
    };

    // Инициализация драйвера MPU6050
    MPU6050_initialize();
    HAL_DelayMs(10);

    //Настройка адреса I2C для MPU6050 можно определить I2C сканером или через Datasheet
    MotionApps20_setAddress(MPU6050_ADDRESS_AD0_HIGH);

    //Настройка и инициализация DMP
    uint8_t status = MotionApps20_dmpInitialize();
    HAL_DelayMs(100);
    if(status != 0)
    {
        //Здесь логика обработки ошибок или вывод сообщения о том, что инициализация DMP не завершилась успешно
    }

    //Запись активного состояния в регистр DMP. Можно включить/выключить DMP
    MPU6050_setDMPEnabled(true);


    /*
    Ваш остальной код до запуска основного цикла
    */

    //Опционально, переменные для значений с MPU6050
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    while (1)
    {
        HAL_DelayMs(500);

        //Получение сырых данных IMU (raw data)
        MPU6050_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        //Примение мастабных коэффициентов
        float accX = ax / 16384.0;
        float accY = ay / 16384.0;
        float accZ = az / 16384.0;

        float fgX = gx / 131.0f;
        float fgY = gy / 131.0f;
        float fgZ = gz / 131.0f;

        //Получение температуры
        int16_t temp = MPU6050_getTemperature();

        //Получение текущий размер буфера DMP в байтах (максимальный размер 1024 байта).
        int16_t fifoC = MPU6050_getFIFOCount();

        //Получение актуального пакета из буфера DMP и запись его в буфер fifoBuffer
        uint8_t fifoPacket = MPU6050_getCurrentFIFOPacket(fifoBuffer, packetSize);

        //Если удалось что-то прочитать, то можем обработать
        if (fifoPacket != 0)
        {
            //Получение кватерниона с ориентацией
            Quaternion q;
            MotionApps20_dmpGetQuaternion_qauternion(&q, fifoBuffer);

            //углы Эйлера работают, но очень сомнительно
            float eulerAngles[3] = {0.0f, 0.0f, 0.0f};
            MotionApps20_dmpGetEuler(eulerAngles, &q);
        }
    }
}

/*
Ваш остальной код
*/

```
