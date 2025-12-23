#include "stm32_hal_connector.h"

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    if(Timeout == 0)
    {
        Timeout = I2C_TIMEOUT_DEFAULT;
    }

    if (MemAddSize != I2C_MEMADD_SIZE_8BIT)
        return HAL_ERROR;

    uint8_t buffer[1 + Size];
    buffer[0] = (uint8_t)MemAddress;

    for (uint16_t i = 0; i < Size; i++)
        buffer[1 + i] = pData[i];

    HAL_StatusTypeDef res = HAL_I2C_Master_Transmit(
            hi2c,
            DevAddress,
            &buffer,
            1 + Size,
            Timeout
        );

    if (hi2c->Init.AutoEnd == I2C_AUTOEND_DISABLE) 
    {
        hi2c->Instance->CR2 |= I2C_CR2_STOP_M;
    }
    
    return res;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    (void)Timeout;

    if (MemAddSize != I2C_MEMADD_SIZE_8BIT)
        return HAL_ERROR;

    uint8_t reg = (uint8_t)MemAddress;

    /* Фаза 1: передаём адрес регистра */
    HAL_StatusTypeDef res = HAL_I2C_Master_Transmit(
            hi2c,
            DevAddress,
            &reg,
            1,
            Timeout);

    if (hi2c->Init.AutoEnd == I2C_AUTOEND_DISABLE) 
    {
        hi2c->Instance->CR2 |= I2C_CR2_STOP_M;
    }

    if(res != HAL_OK)
    {
        return res;
    }

    /* Фаза 2: читаем данные */
    res =  HAL_I2C_Master_Receive(
            hi2c,
            DevAddress,
            pData,
            Size,
            Timeout
        );

    if (hi2c->Init.AutoEnd == I2C_AUTOEND_DISABLE) 
    {
        hi2c->Instance->CR2 |= I2C_CR2_STOP_M;
    }

    return res;
}

