#include "mik32_hal_usart.h"
#include "mik32_hal_i2c.h"

#include "MotionApps20.h"
#include "MPU6050.h"
#include "I2Cdev.h"

#include "tinyformat.h"

#define BUFFER_LENGTH   50
USART_HandleTypeDef husart0;
I2C_HandleTypeDef hi2c0;

void SystemClock_Config(void);
void USART_Init();
void I2C0_Init(uint32_t clock_frequency, uint32_t i2c_frequency);


uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffe

int main()
{
    SystemClock_Config();
    USART_Init();
    HAL_USART_Print(&husart0, "[1T-Rexboard]:[UART firmware v. 1.0.12.25]: Started\n", USART_TIMEOUT_DEFAULT);

    I2C0_Init(32000000, 100000);
    HAL_USART_Print(&husart0, "[1T-Rexboard]: I2C0 init done\n", USART_TIMEOUT_DEFAULT);

    I2Cdev_init(&hi2c0);
    HAL_USART_Print(&husart0, "[1T-Rexboard]: I2C0dev init done\n", USART_TIMEOUT_DEFAULT);

    MPU6050_setAddress(MPU6050_ADDRESS_AD0_HIGH);
    HAL_USART_Print(&husart0, "[1T-Rexboard]: MPU6050 address 0x69 setted\n", USART_TIMEOUT_DEFAULT);
    
    HAL_USART_Print(&husart0, "[1T-Rexboard]: Scanning\n", USART_TIMEOUT_DEFAULT);
    HAL_StatusTypeDef result;
    for (uint8_t i = 0; i < 0x80; i++)
    {
        uint8_t dummy;
        result = HAL_I2C_Master_Receive(&hi2c0, i, &dummy, 1, 10000);
        if (result == HAL_OK)
        {
            char message[64] = "[1T-Rexboard]:[I2C-Scanner]:";
            tinfmt_format(message, sizeof(message), "[1T-Rexboard]:[I2C-Scanner]: Device on %d\n", i);
            HAL_USART_Print(&husart0, message, USART_TIMEOUT_DEFAULT);

            if(MPU6050_ADDRESS_AD0_HIGH == i)
            {
                HAL_USART_Print(&husart0, "[1T-Rexboard]:[I2C-Scanner]: MPU6050 detected on 0x69\n", USART_TIMEOUT_DEFAULT);
            }
        }
    }

    if (hi2c0.Init.AutoEnd == I2C_AUTOEND_DISABLE) 
    {
        hi2c0.Instance->CR2 |= I2C_CR2_STOP_M;
    }
    I2C0_Init(32000000, 100000);
    
    HAL_USART_Print(&husart0, "[1T-Rexboard]: Waiting for MPU6050 test connection\n", USART_TIMEOUT_DEFAULT);

    uint8_t timeout = 100;
    while(!MPU6050_testConnection() && timeout > 2)
    {
        uint8_t deviceID = MPU6050_getDeviceID();
        char message[64] = "[1T-Rexboard]:[I2C-Scanner]:";
        tinfmt_format(message, sizeof(message), "[1T-Rexboard]: I2C: %d; Device ID: %d\n", &hi2c0, deviceID);
        HAL_USART_Print(&husart0, message, USART_TIMEOUT_DEFAULT);

        HAL_DelayMs(500);
        timeout -= 2;
    };
    HAL_USART_Print(&husart0, "[1T-Rexboard]: MPU6050 test connection done\n", USART_TIMEOUT_DEFAULT);

    MPU6050_initialize();
    HAL_DelayMs(10);
    HAL_USART_Print(&husart0, "[1T-Rexboard]: MPU6050 init done\n", USART_TIMEOUT_DEFAULT);

    MotionApps20_setAddress(MPU6050_ADDRESS_AD0_HIGH);
    HAL_USART_Print(&husart0, "[1T-Rexboard]: MPU6050 DMP address 0x69 setted\n", USART_TIMEOUT_DEFAULT);

    uint8_t status = MotionApps20_dmpInitialize();
    HAL_DelayMs(100);
    if(status != 0)
    {
        HAL_USART_Print(&husart0, "[1T-Rexboard]: MPU6050 DMP init error\n", USART_TIMEOUT_DEFAULT);  
    }

    MPU6050_setDMPEnabled(true);
    packetSize = 42;

    HAL_USART_Print(&husart0, "[1T-Rexboard]: MPU6050 DMP init done\n", USART_TIMEOUT_DEFAULT);

    char message[128] = "";
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    while (1)
    {
        // HAL_DelayMs(500);

        MPU6050_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        float accX = ax / 16384.0;
        float accY = ay / 16384.0;
        float accZ = az / 16384.0;

        float fgX = gx / 131.0f;
        float fgY = gy / 131.0f;
        float fgZ = gz / 131.0f;

        int16_t temp = MPU6050_getTemperature();

        HAL_USART_Print(&husart0, "[1T-Rexboard]:[MPU6050]\n", USART_TIMEOUT_DEFAULT);
        
        tinfmt_format(message, sizeof(message), "[MPU6050]:[Temp] (%d)\n", temp);
        HAL_USART_Print(&husart0, message, USART_TIMEOUT_DEFAULT);
        tinfmt_format(message, sizeof(message), "[MPU6050]:[Acc(x, y, z)] (%f, %f, %f)\n", accX, accY, accZ);
        HAL_USART_Print(&husart0, message, USART_TIMEOUT_DEFAULT);
        tinfmt_format(message, sizeof(message), "[MPU6050]:[Gyro(x, y, z)] (%f, %f, %f)\n", fgX, fgY, fgZ);
        HAL_USART_Print(&husart0, message, USART_TIMEOUT_DEFAULT);



        int16_t fifoC = MPU6050_getFIFOCount();
        tinfmt_format(message, sizeof(message), "[MPU6050]:[FIFO Count]: %d\n", fifoC);
        HAL_USART_Print(&husart0, message, USART_TIMEOUT_DEFAULT);

        uint8_t fifoPacket = MPU6050_getCurrentFIFOPacket(fifoBuffer, 42);

        tinfmt_format(message, sizeof(message), "[MPU6050]:[fifoPacket]: %d\n", fifoPacket);
        HAL_USART_Print(&husart0, message, USART_TIMEOUT_DEFAULT);
        if (fifoPacket != 0)
        {
            Quaternion q;
            MotionApps20_dmpGetQuaternion_qauternion(&q, fifoBuffer);

            tinfmt_format(message, sizeof(message), "[MPU6050]:[Quat(w, x, y, z)] (%f, %f, %f, %f)\n", q.w, q.x, q.y, q.z);
            HAL_USART_Print(&husart0, message, USART_TIMEOUT_DEFAULT);

            float eulerAngles[3] = {0.0f, 0.0f, 0.0f};
            MotionApps20_dmpGetEuler(eulerAngles, &q);

            tinfmt_format(message, sizeof(message), "[MPU6050]:[Euler(x, y, z)] (%f, %f, %f)\n", eulerAngles[0], eulerAngles[1], eulerAngles[2]);
            HAL_USART_Print(&husart0, message, USART_TIMEOUT_DEFAULT);
        }
    }
}
void SystemClock_Config(void)
{
    PCC_InitTypeDef PCC_OscInit = {0};
    PCC_OscInit.OscillatorEnable = PCC_OSCILLATORTYPE_ALL;
    PCC_OscInit.FreqMon.OscillatorSystem = PCC_OSCILLATORTYPE_OSC32M;
    PCC_OscInit.FreqMon.ForceOscSys = PCC_FORCE_OSC_SYS_UNFIXED;
    PCC_OscInit.FreqMon.Force32KClk = PCC_FREQ_MONITOR_SOURCE_OSC32K;
    PCC_OscInit.AHBDivider = 0;
    PCC_OscInit.APBMDivider = 0;
    PCC_OscInit.APBPDivider = 0;
    PCC_OscInit.HSI32MCalibrationValue = 128;
    PCC_OscInit.LSI32KCalibrationValue = 8;
    PCC_OscInit.RTCClockSelection = PCC_RTC_CLOCK_SOURCE_AUTO;
    PCC_OscInit.RTCClockCPUSelection = PCC_CPU_RTC_CLOCK_SOURCE_OSC32K;
    HAL_PCC_Config(&PCC_OscInit);
}
void USART_Init()
{
    husart0.Instance = UART_0;
    husart0.transmitting = Enable;
    husart0.receiving = Enable;
    husart0.frame = Frame_8bit;
    husart0.parity_bit = Disable;
    husart0.parity_bit_inversion = Disable;
    husart0.bit_direction = LSB_First;
    husart0.data_inversion = Disable;
    husart0.tx_inversion = Disable;
    husart0.rx_inversion = Disable;
    husart0.swap = Disable;
    husart0.lbm = Disable;
    husart0.stop_bit = StopBit_1;
    husart0.mode = Asynchronous_Mode;
    husart0.xck_mode = XCK_Mode3;
    husart0.last_byte_clock = Disable;
    husart0.overwrite = Disable;
    husart0.rts_mode = AlwaysEnable_mode;
    husart0.dma_tx_request = Disable;
    husart0.dma_rx_request = Disable;
    husart0.channel_mode = Duplex_Mode;
    husart0.tx_break_mode = Disable;
    husart0.Interrupt.ctsie = Disable;
    husart0.Interrupt.eie = Disable;
    husart0.Interrupt.idleie = Disable;
    husart0.Interrupt.lbdie = Disable;
    husart0.Interrupt.peie = Disable;
    husart0.Interrupt.rxneie = Disable;
    husart0.Interrupt.tcie = Disable;
    husart0.Interrupt.txeie = Disable;
    husart0.Modem.rts = Disable; //out
    husart0.Modem.cts = Disable; //in
    husart0.Modem.dtr = Disable; //out
    husart0.Modem.dcd = Disable; //in
    husart0.Modem.dsr = Disable; //in
    husart0.Modem.ri = Disable;  //in
    husart0.Modem.ddis = Disable;//out
    husart0.baudrate = 115200;
    HAL_USART_Init(&husart0);
}

void I2C0_Init(uint32_t clock_frequency, uint32_t i2c_frequency)
{
    hi2c0.Instance = I2C_0;
    hi2c0.Init.Mode = HAL_I2C_MODE_MASTER;
    hi2c0.Init.DigitalFilter = I2C_DIGITALFILTER_OFF;
    hi2c0.Init.AnalogFilter = I2C_ANALOGFILTER_DISABLE;
    hi2c0.Init.AutoEnd = I2C_AUTOEND_DISABLE;

    if (clock_frequency == 32000000)
    {
        if (i2c_frequency == 100000) 
        {
            hi2c0.Clock.PRESC = 0; //на 100 кГц
            hi2c0.Clock.SCLDEL = 8;
            hi2c0.Clock.SDADEL = 2;
            hi2c0.Clock.SCLH = 144;
            hi2c0.Clock.SCLL = 166;
        }

        if (i2c_frequency == 400000) 
        {
            hi2c0.Clock.PRESC = 0; //на 400 кГц
            hi2c0.Clock.SCLDEL = 3;
            hi2c0.Clock.SDADEL = 2;
            hi2c0.Clock.SCLH = 31;
            hi2c0.Clock.SCLL = 35;
        }
    }

    if (HAL_I2C_Init(&hi2c0) != HAL_OK)
    {
        HAL_USART_Print(&husart0, "[1T-Rexboard]: I2C init error\n", USART_TIMEOUT_DEFAULT);
    }
}