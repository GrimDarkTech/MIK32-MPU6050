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
    uint8_t buffer[2 + Size];
    uint16_t idx = 0;

    /* Формируем адрес регистра */
    if (MemAddSize == I2C_MEMADD_SIZE_8BIT)
    {
        buffer[idx++] = (uint8_t)(MemAddress & 0xFF);
    }
    else if (MemAddSize == I2C_MEMADD_SIZE_16BIT)
    {
        buffer[idx++] = (uint8_t)((MemAddress >> 8) & 0xFF);
        buffer[idx++] = (uint8_t)(MemAddress & 0xFF);
    }
    else
    {
        return HAL_ERROR;
    }

    /* Копируем данные */
    for (uint16_t i = 0; i < Size; i++)
    {
        buffer[idx++] = pData[i];
    }

    HAL_I2C_Reset(hi2c);

    /* Обычная передача мастером */
    return HAL_I2C_Master_Transmit(
        hi2c,
        DevAddress,
        buffer,
        idx,
        Timeout
    );
}
