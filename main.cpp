/* mbed Microcontroller Library
 * Copyright (c) 2023 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "mext_encoder_stm32/Encoder.h"
#include "eaux/eaux.h"

//#define DEBUG

UnbufferedSerial ser(CONSOLE_TX, CONSOLE_RX, 115200);

enum struct i2c_transaction_result {
	ok,
	err,
};

i2c_transaction_result read_bytes(I2C &i2c, std::uint8_t sa, std::uint8_t ra, std::size_t n, unsigned char *dest) {

	char reg_address = static_cast<char>(ra);

	if (i2c.write(sa << 1, &reg_address, 1, true) != 0) {
		return i2c_transaction_result::err;
	}

	if (i2c.read((sa << 1) | 1, reinterpret_cast<char *>(dest), n) != 0) {
		return i2c_transaction_result::err;
	}

	return i2c_transaction_result::ok;
}

i2c_transaction_result read_byte(I2C &i2c, std::uint8_t sa, std::uint8_t ra, unsigned char *dest) {
	return read_bytes(i2c, sa, ra, 1, dest);
}

i2c_transaction_result write_byte(I2C &i2c, std::uint8_t sa, std::uint8_t ra, unsigned char byte) {

	char buf[] = { static_cast<char>(ra), static_cast<char>(byte) };

	if (i2c.write(sa << 1, buf, sizeof buf) != 0) {
		return i2c_transaction_result::err;
	}

	return i2c_transaction_result::ok;
}

struct info_t {
	std::uint8_t chip_id;
	std::uint8_t acc_id;
	std::uint8_t mag_id;
	std::uint8_t gyr_id;
	std::uint16_t sw_rev_id;
	std::uint8_t bl_rev_id;
	std::uint8_t page_id;
};

struct gyr_t {
	std::int16_t gyr_data_x;
	std::int16_t gyr_data_y;
	std::int16_t gyr_data_z;
};

struct acc_t {
	std::int16_t acc_data_x;
	std::int16_t acc_data_y;
	std::int16_t acc_data_z;
};

mext::Qei enc_x(PB_4, PB_5);
mext::Qei enc_y(PA_8, PA_9);

gyr_t g_gyr;
acc_t g_acc;

volatile mstd::atomic<std::uint16_t> g_data;

#define ENCODER_RESOLUTION (2048 * 4)
#define WHEEL_RADIUS 14
#define GYRO_DATA_UNIT 900
#define ACC_DATA_UNIT 100

constexpr float enc_ticks_to_disp(int ticks) {
	return static_cast<float>(ticks) / ENCODER_RESOLUTION
			* 2 * eaux::numbers::pi<float>::value * WHEEL_RADIUS;
}

void on_timer() {
	int enc_x_val = enc_x.counter();
	int enc_y_val = enc_y.counter();

	enc_x.write_zero();
	enc_y.write_zero();

	/*
	float dx = enc_ticks_to_disp(enc_x_val);
	float dy = enc_ticks_to_disp(enc_y_val);
	*/

	printf("%d,%d,%d,%d,%d,%d,%d,%d\n",
			enc_x_val, enc_y_val,
			g_gyr.gyr_data_x, g_gyr.gyr_data_y, g_gyr.gyr_data_z,
			g_acc.acc_data_x, g_acc.acc_data_y, g_acc.acc_data_z
		  );
	
	fflush(stdout);
}

Ticker tim;

int main() {

	for (;;) {
		char c;
		ser.read(&c, 1);
		if (c == ' ') {
			break;
		}
	}

	I2C i2c(PB_7, PB_6);
	i2c.frequency(400'000);

	thread_sleep_for(400);

	unsigned char sys_trigger = 0x20;
	write_byte(i2c, 0x28, 0x3F, sys_trigger);
	thread_sleep_for(650);

	unsigned char data[8];
	read_bytes(i2c, 0x28, 0x00, sizeof data, data);

#ifdef DEBUG
	info_t info;
	info.chip_id = data[0];
	info.acc_id = data[1];
	info.mag_id = data[2];
	info.gyr_id = data[3];
	info.sw_rev_id = data[5] << 8 | data[4];
	info.bl_rev_id = data[6];
	info.page_id = data[7];

	printf("\n");

	printf("CHIP_ID: 0x%X\n", info.chip_id);
	printf("ACC_ID: 0x%X\n", info.acc_id);
	printf("MAG_ID: 0x%X\n", info.mag_id);
	printf("GYR_ID: 0x%X\n", info.gyr_id);
	printf("SW_REV_ID: 0x%X\n", info.sw_rev_id);
	printf("BL_REV_ID: 0x%X\n", info.bl_rev_id);
	printf("PAGE_ID: 0x%X\n", info.page_id);

	printf("\n");

	unsigned char st_result = 0;
	read_byte(i2c, 0x28, 0x36, &st_result);
	printf("ST_RESULT: 0x%X\n", st_result);

	unsigned char sys_status = 0;
	read_byte(i2c, 0x28, 0x39, &sys_status);
	printf("SYS_STATUS: %d\n", sys_status);

	sys_trigger = 0;
	read_byte(i2c, 0x28, 0x3F, &sys_trigger);
	printf("SYS_TRIGGER: 0x%X\n", sys_trigger);
	sys_trigger = 1;
	write_byte(i2c, 0x28, 0x3F, sys_trigger);

	thread_sleep_for(400);

	st_result = 0;
	read_byte(i2c, 0x28, 0x36, &st_result);
	printf("ST_RESULT: 0x%X\n", st_result);

	unsigned char sys_err = 0;
	read_byte(i2c, 0x28, 0x3A, &sys_err);
	printf("SYS_ERR: %d\n", sys_err);

	printf("\n");
#endif

	unsigned char unit_sel = 0b10000010;
	write_byte(i2c, 0x28, 0x3B, unit_sel);

	unsigned char opr_mode = 0;
	read_byte(i2c, 0x28, 0x3D, &opr_mode);
#ifdef DEBUG
	printf("OPR_MODE: 0x%x\n", opr_mode);
#endif
	opr_mode = (opr_mode & 0xF0) | 0b1100;
	write_byte(i2c, 0x28, 0x3D, opr_mode);
	thread_sleep_for(7);
	opr_mode = 0;
#ifdef DEBUG
	read_byte(i2c, 0x28, 0x3D, &opr_mode);
	printf("OPR_MODE: 0x%x\n", opr_mode);

	printf("\n");
#endif

	tim.attach(on_timer, 10ms);

	for (;;) {
		{
		unsigned char buf[6];

		read_bytes(i2c, 0x28, 0x14, sizeof buf, buf);

		core_util_critical_section_enter();
		g_gyr.gyr_data_x = buf[1] << 8 | buf[0];
		g_gyr.gyr_data_y = buf[3] << 8 | buf[2];
		g_gyr.gyr_data_z = buf[5] << 8 | buf[4];
		core_util_critical_section_exit();
		}

		{
		unsigned char buf[6];

		read_bytes(i2c, 0x28, 0x08, sizeof buf, buf);

		core_util_critical_section_enter();
		g_acc.acc_data_x = buf[1] << 8 | buf[0];
		g_acc.acc_data_y = buf[3] << 8 | buf[2];
		g_acc.acc_data_z = buf[5] << 8 | buf[4];
		core_util_critical_section_exit();
		}

		thread_sleep_for(5);
	}
}
