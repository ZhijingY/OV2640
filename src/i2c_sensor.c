#include "main.h"

void write_SCCB(I2C_HandleTypeDef hi2c, uint8_t addr, uint8_t data) {
	HAL_I2C_Mem_Write(&hi2c, 0x60, addr, 1, &data, 1, HAL_MAX_DELAY);
}

void read_SCCB(I2C_HandleTypeDef hi2c, uint8_t addr, uint8_t data) {
	HAL_I2C_Mem_Read(&hi2c, 0x61, addr, 1, &data, 1, HAL_MAX_DELAY);
}

void write_sensor_reg(I2C_HandleTypeDef hi2c, const struct sensor_reg regs[]) {
	uint16_t reg_addr = 0;
	uint16_t reg_data = 0;
	const struct sensor_reg *next = regs;
	while((reg_addr != 0xff) | (reg_data != 0xff)) {
		reg_addr = next->addr;
		reg_data = next->data;
		write_SCCB(hi2c, reg_addr, reg_data);
		next++;
	}
}
