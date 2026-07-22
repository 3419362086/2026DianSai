#include "icm20608_iic.h"


extern I2C_HandleTypeDef hi2c1;  // 在外部定义的 HAL I2C 句柄
extern I2C_HandleTypeDef hi2c2;  // 根据实际使用的 I2C 修改

/***********************************************************
@函数名：Init_I2C
@入口参数：无
@出口参数：无
功能描述：STM32 I2C 初始化（已在 main 中通过 HAL 初始化）
@说明：STM32 的 I2C 初始化通常在 HAL 初始化阶段完成，此处留空
*************************************************************/
void Init_I2C(void) {
    // STM32 的 I2C 初始化通常在 HAL_I2C_MspInit 中完成
    // 包含 GPIO 配置和 I2C 外设时钟使能
    // 此函数保留为空或用于其他初始化
}

/***********************************************************
@函数名：i2cWriteData
@功能：I2C 写多字节数据
@参数：addr-7位设备地址, regAddr-寄存器地址, data-数据指针, length-数据长度
*************************************************************/
void i2cWriteData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length) {
    HAL_I2C_Mem_Write(&hi2c1, addr << 1, regAddr, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

/***********************************************************
@函数名：i2cRead
@功能：I2C 读取单字节
@参数：addr-7位设备地址, regAddr-寄存器地址
@返回值：读取到的数据
*************************************************************/
uint8_t i2cRead(uint8_t addr, uint8_t regAddr) {
    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c1, addr << 1, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    return data;
}

/***********************************************************
@函数名：i2cWrite
@功能：I2C 写单字节数据
@参数：addr-7位设备地址, regAddr-寄存器地址, data-待写入数据
*************************************************************/
void i2cWrite(uint8_t addr, uint8_t regAddr, uint8_t data) {
    HAL_I2C_Mem_Write(&hi2c1, addr << 1, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

/***********************************************************
@函数名：i2cReadData
@功能：I2C 读取多字节数据
@参数：addr-7位设备地址, regAddr-寄存器地址, data-数据缓冲区, length-读取长度
*************************************************************/
void i2cReadData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length) {
    HAL_I2C_Mem_Read(&hi2c1, addr << 1, regAddr, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

/***********************************************************
@函数名：Single_WriteI2C
@功能：封装单字节写入
*************************************************************/
void Single_WriteI2C(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data) {
    i2cWrite(SlaveAddress, REG_Address, REG_data);
}

/***********************************************************
@函数名：Single_ReadI2C
@功能：封装单字节读取
@返回值：读取到的数据
*************************************************************/
uint8_t Single_ReadI2C(uint8_t SlaveAddress, uint8_t REG_Address) {
    return i2cRead(SlaveAddress, REG_Address);
}

/***********************************************************
@函数名：Double_ReadI2C
@功能：读取两个字节并组合为16位数据
@返回值：组合后的16位数据
*************************************************************/
int16_t Double_ReadI2C(uint8_t SlaveAddress, uint8_t REG_Address) {
    uint8_t data[2];
    i2cReadData(SlaveAddress, REG_Address, data, 2);
    return (int16_t)((data[0] << 8) | data[1]);
}
