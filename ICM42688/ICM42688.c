#include "main.h"
#include "ICM42688.h"

/* 外部引用 CubeMX 生成的句柄 */
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;

/* CS宏改为HAL操作 */
#define ICM_CS_LOW()   HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET)
#define ICM_CS_HIGH()  HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET)

/* 底层字节收发 [核心变更] */
static uint8_t SPI_ReadWriteByte(uint8_t TxData)
{
    uint8_t rxData;
    HAL_SPI_TransmitReceive(&hspi1, &TxData, &rxData, 1, 10);
    return rxData;
}

/* 以下 ICM42688_ReadReg / WriteReg / ReadRegs 与原标准库版完全相同 */
uint8_t ICM42688_ReadReg(uint8_t regAddr)
{
    uint8_t readData;
    ICM_CS_LOW();
    SPI_ReadWriteByte(regAddr | 0x80);
    readData = SPI_ReadWriteByte(0xFF);
    ICM_CS_HIGH();
    return readData;
}

void ICM42688_WriteReg(uint8_t regAddr, uint8_t writeData)
{
    ICM_CS_LOW();
    SPI_ReadWriteByte(regAddr & 0x7F);
    SPI_ReadWriteByte(writeData);
    ICM_CS_HIGH();
}

void ICM42688_ReadRegs(uint8_t regAddr, uint8_t *pData, uint16_t len)
{
    uint16_t i;
    ICM_CS_LOW();
    SPI_ReadWriteByte(regAddr | 0x80);
    for(i = 0; i < len; i++)
        pData[i] = SPI_ReadWriteByte(0xFF);
    ICM_CS_HIGH();
}

/* 初始化 [去除原生SPI/GPIO初始化] */
uint8_t ICM42688_Init(void)
{
    uint8_t id;

    __HAL_SPI_ENABLE(&hspi1);   // 安全起见多写一句

    HAL_Delay(100);

    ICM_CS_HIGH();
    HAL_Delay(1);
    ICM_CS_LOW();
    HAL_Delay(1);
    ICM_CS_HIGH();
    HAL_Delay(10);

    ICM42688_WriteReg(ICM42688_REG_BANK_SEL, 0x00);
    id = ICM42688_ReadReg(ICM42688_WHO_AM_I);
    if(id != 0x47)
        return 1;

    ICM42688_WriteReg(ICM42688_DEVICE_CONFIG, 0x01);
    HAL_Delay(10);

    ICM42688_WriteReg(ICM42688_PWR_MGMT0, PWR_MGMT0_ACCEL_LN_GYRO_LN);
    HAL_Delay(50);

    ICM42688_WriteReg(ICM42688_GYRO_CONFIG0, GYRO_ROBOT_2000_1KHZ);
    ICM42688_WriteReg(ICM42688_ACCEL_CONFIG0, ACCEL_ROBOT_16G_1KHZ);
    ICM42688_WriteReg(ICM42688_GYRO_CONFIG1, FILTER_BW_100HZ_3RD);
    ICM42688_WriteReg(ICM42688_ACCEL_CONFIG1, FILTER_BW_100HZ_3RD);

    ICM42688_WriteReg(ICM42688_PWR_MGMT0, PWR_MGMT0_ACCEL_LN_GYRO_LN);
    HAL_Delay(50);

    return 0;
}

/* 6轴+温度读取 [无需改动] */
void ICM42688_Read_6Axis_Temp(ICM42688_Data_t *data)
{
    uint8_t buffer[14];
    ICM42688_ReadRegs(ICM42688_TEMP_DATA1, buffer, 14);
    data->temp   = (float)((int16_t)((buffer[0] << 8) | buffer[1])) / 132.48f + 25.0f;
    data->accel_x = (float)((int16_t)((buffer[2] << 8) | buffer[3])) / 2048.0f;
    data->accel_y = (float)((int16_t)((buffer[4] << 8) | buffer[5])) / 2048.0f;
    data->accel_z = (float)((int16_t)((buffer[6] << 8) | buffer[7])) / 2048.0f;
    data->gyro_x  = (float)((int16_t)((buffer[8] << 8) | buffer[9])) / 16.384f;
    data->gyro_y  = (float)((int16_t)((buffer[10] << 8) | buffer[11])) / 16.384f;
    data->gyro_z  = (float)((int16_t)((buffer[12] << 8) | buffer[13])) / 16.384f;
}



