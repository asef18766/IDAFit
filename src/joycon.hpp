#include <bitset>
#include <chrono>
#include "hidapi.h"
#include "tools.hpp"
#include <cstring>

#define JOYCON_VENDOR 0x057e
#define JOYCON_L_BT 0x2006
#define JOYCON_R_BT 0x2007
#define PRO_CONTROLLER 0x2009
#define JOYCON_CHARGING_GRIP 0x200e
#define L_OR_R(lr) (lr == 1 ? 'L' : (lr == 2 ? 'R' : '?'))

u8 mcu_crc8_calc(u8* buf, u8 size) { //CTCAER
	u8 crc8 = 0x0;

	for (int i = 0; i < size; ++i) {
		crc8 = mcu_crc8_table[(u8)(crc8 ^ buf[i])];
	}
	return crc8;
}

u8 ringmcu_crc8_calc(u8* buf, u8 size) {
	u8 crc8 = 0x0;

	for (int i = 0; i < size; ++i) {
		crc8 = ringmcu_crc8_table[(u8)(crc8 ^ buf[i])];
	}
	return crc8;
}



class Joycon {

public:
	bool running;
	bool squatting;
	
	hid_device* handle;
	wchar_t* serial;

	std::string name;

	int deviceNumber = 0;// left(0) or right(1)
	//int VigemNumber = 0;// vigem device number / device group number

	bool bluetooth = true;
	bool ringconattached = false;

	int left_right = 0;// 1: left joycon, 2: right joycon, 3: pro controller

	uint16_t buttons = 0;
	uint16_t buttons2 = 0;// for pro controller

	int ringcon = 0x0A; //Ringcon data. Packet[40]. Fully pulled = 0x00, rest = 0x0a, fully pushed = 0x14.

	struct btn_states {
		// left:
		int up = 0;
		int down = 0;
		int left = 0;
		int right = 0;
		int l = 0;
		int zl = 0;
		int minus = 0;
		int capture = 0;

		// right:
		int a = 0;
		int b = 0;
		int x = 0;
		int y = 0;
		int r = 0;
		int zr = 0;
		int plus = 0;
		int home = 0;

		// shared:
		int sl = 0;
		int sr = 0;
		int stick_button = 0;

		// pro controller:
		int stick_button2 = 0;// pro controller

	} btns;

	int8_t dstick;
	uint8_t battery;

	int global_count = 0;

	struct Stick {
		uint16_t x = 0;
		uint16_t y = 0;
		float CalX = 0;
		float CalY = 0;
	};

	Stick stick;
	Stick stick2;// Pro Controller

	struct Gyroscope {
		// relative:
		float pitch = 0;
		float yaw = 0;
		float roll = 0;

		struct Offset {
			int n = 0;

			// absolute:
			float pitch = 0;
			float yaw = 0;
			float roll = 0;
		} offset;
	} gyro;

	struct Accelerometer {
		float prevX = 0;
		float prevY = 0;
		float prevZ = 0;
		float x = 0;
		float y = 0;
		float z = 0;
	} accel;


	// calibration data:

	struct brcm_hdr {
		uint8_t cmd;
		uint8_t rumble[9];
	};

	unsigned char timestampbuffer[64] = { 0 };
	// Convert long long to byte array
	void IntToByteArray(long long value)
	{
		for (int i = 0; i < 64; i++)
		{
			timestampbuffer[i] = ((value >> (8 * i)) & 0XFF);
		}
	}

	//struct brcm_cmd_01 {
	//	uint8_t subcmd;
	//	union {

	//		struct {
	//			uint32_t offset;
	//			uint8_t size;
	//		} spi_read;

	//		struct {
	//			uint32_t address;
	//		} hax_read;
	//	};
	//};

	struct brcm_cmd_01 {
		uint8_t subcmd;
		uint32_t offset;
		uint8_t size;
	};

	int timing_byte = 0x0;

	//struct brcm_hdr {
	//	uint8_t cmd;
	//	uint8_t timer;
	//	uint8_t rumble_l[4];
	//	uint8_t rumble_r[4];
	//};

	//struct brcm_cmd_01 {
	//	uint8_t subcmd;
	//	union {
	//		struct {
	//			uint32_t offset;
	//			uint8_t size;
	//		} spi_read;

	//		struct {
	//			uint8_t arg1;
	//			uint8_t arg2;
	//		} subcmd_arg;
	//	};
	//};

	//// Used to order the packets received in Joy-Con internally. Range 0x0-0xF.
	//uint8_t timing_byte = 0x0;



	float acc_cal_coeff[3];
	float gyro_cal_coeff[3];
	float cal_x[1] = { 0.0f };
	float cal_y[1] = { 0.0f };

	bool has_user_cal_stick_l = false;
	bool has_user_cal_stick_r = false;
	bool has_user_cal_sensor = false;

	unsigned char factory_stick_cal[0x12];
	unsigned char user_stick_cal[0x16];
	unsigned char sensor_model[0x6];
	unsigned char stick_model[0x24];
	unsigned char factory_sensor_cal[0x18];
	unsigned char user_sensor_cal[0x1A];
	uint16_t factory_sensor_cal_calm[0xC];
	uint16_t user_sensor_cal_calm[0xC];
	int16_t sensor_cal[0x2][0x3];
	uint16_t stick_cal_x_l[0x3];
	uint16_t stick_cal_y_l[0x3];
	uint16_t stick_cal_x_r[0x3];
	uint16_t stick_cal_y_r[0x3];


public:


	Joycon(struct hid_device_info* dev) {

		if (dev->product_id == JOYCON_CHARGING_GRIP) {

			if (dev->interface_number == 0 || dev->interface_number == -1) {
				this->name = std::string("Joy-Con (R)");
				this->left_right = 2;// right joycon
			}
			else if (dev->interface_number == 1) {
				this->name = std::string("Joy-Con (L)");
				this->left_right = 1;// left joycon
			}
		}

		if (dev->product_id == JOYCON_L_BT) {
			this->name = std::string("Joy-Con (L)");
			this->left_right = 1;// left joycon
		}
		else if (dev->product_id == JOYCON_R_BT) {
			this->name = std::string("Joy-Con (R)");
			this->left_right = 2;// right joycon
		}
		else if (dev->product_id == PRO_CONTROLLER) {
			this->name = std::string("Pro Controller");
			this->left_right = 3;// left joycon
		}

		this->serial = _wcsdup(dev->serial_number);

		//printf("Found joycon %c %i: %ls %s\n", L_OR_R(this->left_right), joycons.size(), this->serial, dev->path);
		printf("Found joycon %c: %ls %s\n", L_OR_R(this->left_right), this->serial, dev->path);
		this->handle = hid_open_path(dev->path);


		if (this->handle == nullptr) {
			//printf("Could not open serial %ls: %s\n", this->serial, strerror(errno));
			throw;
		}
	}

	void hid_exchange(hid_device* handle, unsigned char* buf, int len) {
		if (!handle) return;

		int res;

		res = hid_write(handle, buf, len);

		//if (res < 0) {
		//	printf("Number of bytes written was < 0!\n");
		//} else {
		//	printf("%d bytes written.\n", res);
		//}

		//// set non-blocking:
		//hid_set_nonblocking(handle, 1);

		res = hid_read(handle, buf, 0x40);

		//if (res < 1) {
		//	printf("Number of bytes read was < 1!\n");
		//} else {
		//	printf("%d bytes read.\n", res);
		//}
	}


	void send_command(int command, uint8_t* data, int len) {
		unsigned char buf[0x40];
		memset(buf, 0, 0x40);

		if (!bluetooth) {
			buf[0x00] = 0x80;
			buf[0x01] = 0x92;
			buf[0x03] = 0x31;
		}

		buf[bluetooth ? 0x0 : 0x8] = command;
		if (data != nullptr && len != 0) {
			memcpy(buf + (bluetooth ? 0x1 : 0x9), data, len);
		}

		hid_exchange(this->handle, buf, len + (bluetooth ? 0x1 : 0x9));

		if (data) {
			memcpy(data, buf, 0x40);
		}
	}

	void send_subcommand(int command, int subcommand, uint8_t* data, int len) {
		unsigned char buf[0x40];
		memset(buf, 0, 0x40);

		uint8_t rumble_base[9] = { (++global_count) & 0xF, 0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40 };
		memcpy(buf, rumble_base, 9);

		if (global_count > 0xF) {
			global_count = 0x0;
		}

		// set neutral rumble base only if the command is vibrate (0x01)
		// if set when other commands are set, might cause the command to be misread and not executed
		//if (subcommand == 0x01) {
		//	uint8_t rumble_base[9] = { (++global_count) & 0xF, 0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40 };
		//	memcpy(buf + 10, rumble_base, 9);
		//}

		buf[9] = subcommand;
		if (data && len != 0) {
			memcpy(buf + 10, data, len);
		}

		send_command(command, buf, 10 + len);

		if (data) {
			memcpy(data, buf, 0x40); //TODO
		}
	}

	void rumble(int frequency, int intensity) {

		unsigned char buf[0x400];
		memset(buf, 0, 0x40);

		// intensity: (0, 8)
		// frequency: (0, 255)

		//	 X	AA	BB	 Y	CC	DD
		//[0 1 x40 x40 0 1 x40 x40] is neutral.


		//for (int j = 0; j <= 8; j++) {
		//	buf[1 + intensity] = 0x1;//(i + j) & 0xFF;
		//}

		buf[1 + 0 + intensity] = 0x1;
		buf[1 + 4 + intensity] = 0x1;

		// Set frequency to increase
		if (this->left_right == 1) {
			buf[1 + 0] = frequency;// (0, 255)
		}
		else {
			buf[1 + 4] = frequency;// (0, 255)
		}

		// set non-blocking:
		hid_set_nonblocking(this->handle, 1);

		send_command(0x10, (uint8_t*)buf, 0x9);
	}

	void rumble2(uint16_t hf, uint8_t hfa, uint8_t lf, uint16_t lfa) {
		unsigned char buf[0x400];
		memset(buf, 0, 0x40);


		//int hf		= HF;
		//int hf_amp	= HFA;
		//int lf		= LF;
		//int lf_amp	= LFA;
		// maybe:
		//int hf_band = hf + hf_amp;

		int off = 0;// offset
		if (this->left_right == 2) {
			off = 4;
		}


		// Byte swapping
		buf[0 + off] = hf & 0xFF;
		buf[1 + off] = hfa + ((hf >> 8) & 0xFF); //Add amp + 1st byte of frequency to amplitude byte

												 // Byte swapping
		buf[2 + off] = lf + ((lfa >> 8) & 0xFF); //Add freq + 1st byte of LF amplitude to the frequency byte
		buf[3 + off] = lfa & 0xFF;


		// set non-blocking:
		hid_set_nonblocking(this->handle, 1);

		send_command(0x10, (uint8_t*)buf, 0x9);
	}

	void rumble3(float frequency, uint8_t hfa, uint16_t lfa) {

		//Float frequency to hex conversion
		if (frequency < 0.0f) {
			frequency = 0.0f;
		}
		else if (frequency > 1252.0f) {
			frequency = 1252.0f;
		}
		uint8_t encoded_hex_freq = (uint8_t)round(log2((double)frequency / 10.0) * 32.0);

		//uint16_t encoded_hex_freq = (uint16_t)floor(-32 * (0.693147f - log(frequency / 5)) / 0.693147f + 0.5f); // old

		//Convert to Joy-Con HF range. Range in big-endian: 0x0004-0x01FC with +0x0004 steps.
		uint16_t hf = (encoded_hex_freq - 0x60) * 4;
		//Convert to Joy-Con LF range. Range: 0x01-0x7F.
		uint8_t lf = encoded_hex_freq - 0x40;

		rumble2(hf, hfa, lf, lfa);
	}

	void rumble4(float real_LF, float real_HF, uint8_t hfa, uint16_t lfa) {

		real_LF = clamp(real_LF, 40.875885f, 626.286133f);
		real_HF = clamp(real_HF, 81.75177, 1252.572266f);

		////Float frequency to hex conversion
		//if (frequency < 0.0f) {
		//	frequency = 0.0f;
		//} else if (frequency > 1252.0f) {
		//	frequency = 1252.0f;
		//}
		//uint8_t encoded_hex_freq = (uint8_t)round(log2((double)frequency / 10.0)*32.0);

		//uint16_t encoded_hex_freq = (uint16_t)floor(-32 * (0.693147f - log(frequency / 5)) / 0.693147f + 0.5f); // old

		////Convert to Joy-Con HF range. Range in big-endian: 0x0004-0x01FC with +0x0004 steps.
		//uint16_t hf = (encoded_hex_freq - 0x60) * 4;
		////Convert to Joy-Con LF range. Range: 0x01-0x7F.
		//uint8_t lf = encoded_hex_freq - 0x40;



		uint16_t hf = ((uint8_t)round(log2((double)real_HF * 0.01) * 32.0) - 0x60) * 4;
		uint8_t lf = (uint8_t)round(log2((double)real_LF * 0.01) * 32.0) - 0x40;

		rumble2(hf, hfa, lf, lfa);
	}


	void rumble_freq(uint16_t hf, uint8_t hfa, uint8_t lf, uint16_t lfa) {
		unsigned char buf[0x400];
		memset(buf, 0, 0x40);


		//int hf		= HF;
		//int hf_amp	= HFA;
		//int lf		= LF;
		//int lf_amp	= LFA;
		// maybe:
		//int hf_band = hf + hf_amp;

		int off = 0;// offset
		if (this->left_right == 2) {
			off = 4;
		}


		// Byte swapping
		buf[0 + off] = hf & 0xFF;
		buf[1 + off] = hfa + ((hf >> 8) & 0xFF); //Add amp + 1st byte of frequency to amplitude byte

												 // Byte swapping
		buf[2 + off] = lf + ((lfa >> 8) & 0xFF); //Add freq + 1st byte of LF amplitude to the frequency byte
		buf[3 + off] = lfa & 0xFF;


		// set non-blocking:
		hid_set_nonblocking(this->handle, 1);

		send_command(0x10, (uint8_t*)buf, 0x9);
	}


	void setGyroOffsets() {
		float thresh = 0.1;
		if (abs(this->gyro.roll) > thresh || abs(this->gyro.pitch) > thresh || abs(this->gyro.yaw) > thresh) {
			return;
		}

		//average = current + ((newData - current) / n);
		this->gyro.offset.n += 1;
		this->gyro.offset.roll = this->gyro.offset.roll + ((this->gyro.roll - this->gyro.offset.roll) / this->gyro.offset.n);
		this->gyro.offset.pitch = this->gyro.offset.pitch + ((this->gyro.pitch - this->gyro.offset.pitch) / this->gyro.offset.n);
		this->gyro.offset.yaw = this->gyro.offset.yaw + ((this->gyro.yaw - this->gyro.offset.yaw) / this->gyro.offset.n);
		//this->gyro.offset.roll	= this->gyro.roll;
		//this->gyro.offset.pitch = this->gyro.pitch;
		//this->gyro.offset.yaw	= this->gyro.yaw;
	}

	void set_ext_config(int a, int b, int c, int d) {
		unsigned char buf[0x400];
		memset(buf, 0, 0x40);

		while (1) {
			printf("Set Ext Config 58\n");

			static int output_buffer_length = 49; //CTCAER - USE THIS - for some reason the pkt sub command was positioned right but all the subcommand 21 21 stuff was offset plus one byte
			int res = 0;

			memset(buf, 0, sizeof(buf));
			auto hdr = (brcm_hdr*)buf;
			hdr->cmd = 0x01;
			hdr->rumble[0] = timing_byte & 0xF;
			timing_byte++;
			buf[10] = 0x58;
			buf[11] = a;
			buf[12] = b;
			buf[13] = c;
			buf[14] = d;

			/*for (int i = 0; i <= 48; i++) {
				printf("%i: %02x ", i, buf[i]);
			}
			printf("\n");
			printf("\n");*/

			res = hid_write(handle, buf, output_buffer_length);
			int retries = 0;

			/*for (int i = 0; i < 50; i++) {
				res = hid_read_timeout(handle, buf, sizeof(buf), 64);
				for (int i = 0; i <= 100; i++) {
					printf("%i: %02x ", i, buf[i]);
				}
				printf("\n");
				printf("\n");*/
			

			//while (1) {
			//	buf[0] != 21;
			//}

			//while (1) {}

			while (1) {
				res = hid_read_timeout(handle, buf, sizeof(buf), 64);
				//for (int i = 0; i <= 60; i++) {
				//	printf("%i: %02x ", i, buf[i]);
				//}
				//printf("\n");
				//printf("\n");
				if (buf[0] == 0x21) {
					if (buf[14] == 0x58) {
						return;
					}
				}
				retries++;
				if (retries > 8 || res == 0) {
					break;
				}
			}
		}
	}

	void set_vib_config(int a, int b, int c, int d) {
		unsigned char buf[0x400];
		memset(buf, 0, 0x40);

		static int output_buffer_length = 49; //CTCAER - USE THIS - for some reason the pkt sub command was positioned right but all the subcommand 21 21 stuff was offset plus one byte
		int res = 0;

		memset(buf, 0, sizeof(buf));
		auto hdr = (brcm_hdr*)buf;
		hdr->cmd = 0x10;
		hdr->rumble[0] = timing_byte & 0xF;
		timing_byte++;
		buf[6] = a;
		buf[7] = b;
		buf[8] = c;
		buf[9] = d;

		/*for (int i = 0; i <= 48; i++) {
			printf("%i: %02x ", i, buf[i]);
		}
		printf("\n");
		printf("\n");*/

		res = hid_write(handle, buf, output_buffer_length);

		res = hid_read_timeout(handle, buf, sizeof(buf), 64);
		for (int i = 0; i <= 100; i++) {
			printf("%i: %02x ", i, buf[i]);
		}
	}


	int init_bt() {

		this->bluetooth = true;

		unsigned char buf[0x40];
		memset(buf, 0, 0x40);


		// set blocking to ensure command is recieved:
		hid_set_nonblocking(this->handle, 0);

		// Enable vibration
		printf("Enabling vibration...\n");
		buf[0] = 0x01; // Enabled
		send_subcommand(0x1, 0x48, buf, 1);

		// Enable IMU data
		printf("Enabling IMU data...\n");
		buf[0] = 0x01; // Enabled
		send_subcommand(0x01, 0x40, buf, 1);


		// Set input report mode (to push at 60hz)
		// x00	Active polling mode for IR camera data. Answers with more than 300 bytes ID 31 packet
		// x01	Active polling mode
		// x02	Active polling mode for IR camera data.Special IR mode or before configuring it ?
		// x21	Unknown.An input report with this ID has pairing or mcu data or serial flash data or device info
		// x23	MCU update input report ?
		// 30	NPad standard mode. Pushes current state @60Hz. Default in SDK if arg is not in the list
		// 31	NFC mode. Pushes large packets @60Hz
		printf("Set input report mode to 0x30...\n");
		buf[0] = 0x30;
		send_subcommand(0x01, 0x03, buf, 1);

		GetCalibrationData();
		
		if (this->left_right == 1 || this->left_right == 3) {
			printf("Successfully initialized %s!\n", this->name.c_str());
			return 0;
		}

		int Ringconretries = 0;

	step1:

		// Enable MCU data
		int retries2 = 0;
		while (1) {
			printf("Enabling MCU data 22 1\n");
			buf[0] = 0x01; // Enabled - 00 = suspend, 01 = resume
			send_subcommand(0x01, 0x22, buf, 1);

			int retries = 0;
			while (1) {
				int res = hid_read_timeout(handle, buf, sizeof(buf), 64);
				/*for (int i = 0; i <= 100; i++) {
					printf("%i: %02x ", i, buf[i]);
				}
				printf("\n");
				printf("\n");*/
				if (*(u16*)&buf[0xD] == 0x2280)
					goto step3;

				retries++;
				if (retries > 8 || res == 0)
					break;
			}
			retries2++;
			/*if (retries2 > 8) {
				goto step20;
			}	*/
		}

	step2:
		// Request MCU mode status - This step doesnt work when using 0x30
		while (1) {
			printf("Requesting MCU data 11 1 ...\n");
			buf[0] = 0x00; //Output report x11, sub command 0x01, no arguments
			send_subcommand(0x11, 0x01, buf, 0);

			int retries = 0;
			while (1) {
				int res = hid_read_timeout(handle, buf, sizeof(buf), 64);
				/*for (int i = 0; i <= 60; i++) {
					printf("%i: %02x ", i, buf[i]);
				}
				printf("\n");
				printf("\n");*/
				if (buf[0] == 0x31) {
					for (int i = 49; i <= 56; i++) {
						printf("%i: %02x ", i, buf[i]);
					}
					//printf("%02x ", buf[49]);
					//if (buf[49] == 0x01 && buf[56] == 0x06) // MCU state is Initializing
					// *(u16*)buf[52]LE x04 in lower than 3.89fw, x05 in 3.89
					// *(u16*)buf[54]LE x12 in lower than 3.89fw, x18 in 3.89
					// buf[56]: mcu mode state
					if (buf[49] == 0x01 && buf[56] == 0x01) // MCU state is Standby
						goto step3;

				}
				retries++;
				if (retries > 8 || res == 0)
					break;
			}
		}

	step3:

		// Enable MCU polling
		//printf("Enabling MCU polling...\n");
		//buf[0] = 0x01; // Enabled empty 0, 5, 6 Not 1 49:1 56:1 2 49:2A 56:0
		//send_subcommand(0x01, 0x24, buf, 1);
		retries2 = 0;
		//Set MCU Mode
		while (1) {
			printf("Enabling MCU data 21 21 0 3...\n");
			static int output_buffer_length = 49; //CTCAER - USE THIS - for some reason the pkt sub command was positioned right but all the subcommand 21 21 stuff was offset plus one byte
			int res = 0;

			memset(buf, 0, sizeof(buf));
			auto hdr = (brcm_hdr*)buf;
			//auto pkt = (brcm_cmd_01*)(hdr+1);
			hdr->cmd = 0x01;
			hdr->rumble[0] = timing_byte & 0xF;
			timing_byte++;
			buf[10] = 0x21;
			buf[11] = 0x21;
			buf[12] = 0x00;
			buf[13] = 0x03;
			//pkt->subcmd_21_21.mcu_cmd = 0x21; // Set MCU mode cmd
			//pkt->subcmd_21_21.mcu_subcmd = 0x00; // Set MCU mode cmd
			//pkt->subcmd_21_21.mcu_mode = 0x03; // MCU mode - 1: Standby, 4: NFC, 5: IR, 6: Initializing/FW Update?

			buf[48] = mcu_crc8_calc(buf + 12, 36);
			/*for (int i = 0; i <= 48; i++) {
				printf("%i: %02x ", i, buf[i]);
			}
			printf("\n");*/

			res = hid_write(handle, buf, output_buffer_length);
			int retries = 0;

			while (1) {
				res = hid_read_timeout(handle, buf, sizeof(buf), 64);
				/*for (int i = 0; i <= 100; i++) {
					printf("%i: %02x ", i, buf[i]);
				}
				printf("\n");*/
				if (buf[0] == 0x21) {
					// *(u16*)buf[18]LE x04 in lower than 3.89fw, x05 in 3.89
					// *(u16*)buf[20]LE x12 in lower than 3.89fw, x18 in 3.89
					if (buf[15] == 0x01 && buf[22] == 0x03) // Mcu mode is standby
						goto step5;
				}
				retries++;
				if (retries > 8 || res == 0) {
					break;
				}
			}
			retries2++;
			/*if (retries2 > 8) {
				goto step20;
			}*/
		}

	step4:
		// Request MCU mode status
		while (1) {
			printf("Enabling MCU data 11 1...\n");
			static int output_buffer_length = 49;
			int res = 0;
			memset(buf, 0, sizeof(buf));
			auto hdr = (brcm_hdr*)buf;
			auto pkt = (brcm_cmd_01*)(hdr + 1);
			hdr->cmd = 0x11;
			hdr->rumble[0] = timing_byte & 0xF;
			timing_byte++;
			pkt->subcmd = 0x01; //subcmd is in the right byte unlike the union stuff

			/*for (int i = 0; i <= 48; i++) {
				printf("%i: %02x ", i, buf[i]);
			}
			printf("\n");*/

			res = hid_write(handle, buf, output_buffer_length);
			int retries = 0;
			while (1) {
				res = hid_read_timeout(handle, buf, sizeof(buf), 64);
				/*for (int i = 0; i <= 60; i++) {
					printf("%i: %02x ", i, buf[i]);
				}
				printf("\n");
				printf("\n");*/
				if (buf[0] == 0x31) {
					// *(u16*)buf[52]LE x04 in lower than 3.89fw, x05 in 3.89
					// *(u16*)buf[54]LE x12 in lower than 3.89fw, x18 in 3.89
					if (buf[49] == 0x01 && buf[56] == 0x03) // Mcu mode is Ringcon
						goto step5;
				}
				retries++;
				if (retries > 8 || res == 0)
					break;
			}
		}

	step5:

		//Set MCU Mode
		while (1) {
			printf("Enabling MCU data 21 21 1 1...\n");

			static int output_buffer_length = 49; //CTCAER
			int res = 0;

			memset(buf, 0, sizeof(buf));
			auto hdr = (brcm_hdr*)buf;
			hdr->cmd = 0x01;
			hdr->rumble[0] = timing_byte & 0xF;
			timing_byte++;
			buf[10] = 0x21;
			buf[11] = 0x21;
			buf[12] = 0x01;
			buf[13] = 0x01;

			buf[48] = mcu_crc8_calc(buf + 12, 36);

			/*for (int i = 0; i <= 48; i++) {
				printf("%i: %02x ", i, buf[i]);
			}
			printf("\n");
			printf("\n");*/

			res = hid_write(handle, buf, output_buffer_length);
			int retries = 0;

			while (1) {
				res = hid_read_timeout(handle, buf, sizeof(buf), 64);
				/*for (int i = 0; i <= 100; i++) {
					printf("%i: %02x ", i, buf[i]);
				}
				printf("\n");
				printf("\n");*/
				if (buf[0] == 0x21) {
					if (buf[15] == 0x09 && buf[17] == 0x01) // Mcu mode is external ready
						goto step6;
				}
				retries++;
				if (retries > 8 || res == 0)
					break;
			}
		}

	step6:

		retries2 = 0;
		printf("Get ext data 59.");
		while (1) {
			printf(".");

			static int output_buffer_length = 49; //CTCAER
			int res = 0;

			memset(buf, 0, sizeof(buf));
			auto hdr = (brcm_hdr*)buf;
			hdr->cmd = 0x01;
			hdr->rumble[0] = timing_byte & 0xF;
			timing_byte++;
			buf[10] = 0x59;

			/*for (int i = 0; i <= 48; i++) {
				printf("%i: %02x ", i, buf[i]);
			}
			printf("\n");
			printf("\n");*/

			res = hid_write(handle, buf, output_buffer_length);
			int retries = 0;

			while (1) {
				res = hid_read_timeout(handle, buf, sizeof(buf), 64);
				/*for (int i = 0; i <= 100; i++) {
					printf("%i: %02x ", i, buf[i]);
				}
				printf("\n");
				printf("\n");*/
				if (buf[0] == 0x21) {
					if (buf[14] == 0x59 && buf[16] == 0x20) { // Mcu mode is ringcon ready (Note:0x20 is the Ringcon ext device id - it may also be the fm_main_ver) No ringcon is buf[15]=fe. With ringcon buf[15]=0
						goto step7;
					}
				}
				retries++;
				if (retries > 8 || res == 0)
					break;
			}
			retries2++;
			if (retries2 > 28 || res == 0) {
				printf("Enabling IMU data...\n");
				buf[0] = 0x01; // Enabled
				send_subcommand(0x01, 0x40, buf, 1);
				//GetCalibrationData();

				printf("Successfully initialized but no Ringcon detected %s!\n", this->name.c_str());
				return 0;
			}
		}


	step7:

		ringconattached = true;
		// Enable IMU data
		printf("Enabling IMU data 3...\n");
		buf[0] = 0x03; // Ringcon IMU enabled 
		send_subcommand(0x01, 0x40, buf, 1);
		int res = 0;
		int retries = 0;
		while (1) {
			res = hid_read_timeout(handle, buf, sizeof(buf), 64);
			/*for (int i = 0; i <= 60; i++) {
				printf("%i: %02x ", i, buf[i]);
			}
			printf("\n");
			printf("\n");*/
			if (buf[0] == 0x21) {
				if (buf[14] == 0x40) {
					//break;
					printf("Enabling IMU data 1...\n");
					buf[0] = 0x02;
					send_subcommand(0x01, 0x40, buf, 1);
					buf[0] = 0x01;  
					send_subcommand(0x01, 0x40, buf, 1);
					Ringconretries++; //Was having issues with the Gyro being screwy sometimes.
					if (Ringconretries <= 1) {
						GetCalibrationData();
						goto step1;
					}
					else {
						break;
					}
				}
			}
			retries++;
			if (retries > 20 || res == 0)
			break;
		}

		//unsigned long long timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		//IntToByteArray(timestamp);

		while (1) {
			printf("Get ext dev in format config 5C...\n");

			static int output_buffer_length = 49; //CTCAER 
			int res = 0;

			memset(buf, 0, sizeof(buf));
			auto hdr = (brcm_hdr*)buf;
			hdr->cmd = 0x01;
			hdr->rumble[0] = timing_byte & 0xF;
			timing_byte++;
			buf[10] = 0x5C;
			buf[11] = 0x06;
			buf[12] = 0x03;
			buf[13] = 0x25;
			buf[14] = 0x06;
			buf[19] = 0x1C;
			buf[20] = 0x16;
			buf[21] = 237;// 0xCD;//+
			buf[22] = 52;// 0xA6;//+
			buf[23] = 54;// 0x2B;//+ These change but appear to be connected with the last values
			buf[27] = 10;//timestampbuffer[0];// 27-32 = timestamp
			buf[28] = 100;//timestampbuffer[1];
			buf[29] = 11;//timestampbuffer[2];
			buf[30] = 230;//timestampbuffer[3];
			buf[31] = 169;//timestampbuffer[4];
			buf[32] = 34;//timestampbuffer[5];
			buf[33] = 0x00;//timestampbuffer[6];
			buf[34] = 0x00;//timestampbuffer[7]; 
			buf[35] = 0x04;
			buf[43] = 0x90;
			buf[44] = 0xA8;
			buf[45] = 225;// 0xC1;//+
			buf[46] = 52;// 0x34;// 0xA6;//+
			buf[47] = 54;// 0x36;// 0x2B;//+ These numbers have something to do with the input report from 40 3 */

			//21/10/20  92 6 3 37 6 0 0 0 0 28 22 205 166 43 0 0 0 197 20 147 187 160 13 0 0 4 0 0 0 0 0 0 0 144 168 193 166 43 0
			//08/10/20  92 6 3 37 6 0 0 0 0 28 22 237 52  54 0 0 0 10 100  11 230 169 34 0 0 4 0 0 0 0 0 0 0 144 168 225 52  54 0
 
  			/*for (int i = 0; i <= 48; i++) {
				printf("%i: %02x ", i, buf[i]);
			}
			printf("\n");
			printf("\n");*/

			res = hid_write(handle, buf, output_buffer_length);
			int retries = 0;
			while (1) {
				res = hid_read_timeout(handle, buf, sizeof(buf), 64);
				/*for (int i = 0; i <= 60; i++) {
					printf("%i: %02x ", i, buf[i]);
				}
				printf("\n");
				printf("\n");*/
				if (buf[0] == 0x21) {
					if (buf[14] == 0x5C) { // Ringcon config set
						goto step8;
					}
					/*if (buf[14] == 0x40) { // Sensor put to sleep for some reason. Try again.
						printf("Enabling IMU data 1...\n");
						buf[0] = 0x01; // Ringcon IMU enabled 
						send_subcommand(0x01, 0x40, buf, 1); 
						goto step1;
					}*/
				}
				retries++;
				if (retries > 8 || res == 0)
					break;
			}
		}

	step8:
		while (1) {
			printf("Start external polling 5A...\n"); //This may only be for the config stuff. The app turns it off after config then calls the MCU again. If you call the 5c command when the polling is enbled, it will fail.

			static int output_buffer_length = 49; //CTCAER
			int res = 0;

			memset(buf, 0, sizeof(buf));
			auto hdr = (brcm_hdr*)buf;
			hdr->cmd = 0x01;
			hdr->rumble[0] = timing_byte & 0xF;
			timing_byte++;
			buf[10] = 0x5A;
			buf[11] = 0x04;
			buf[12] = 0x01;
			buf[13] = 0x01;
			buf[14] = 0x02;

			/*for (int i = 0; i <= 48; i++) {
				printf("%i: %02x ", i, buf[i]);
			}
			printf("\n");
			printf("\n");*/

			res = hid_write(handle, buf, output_buffer_length);
			int retries = 0;

			while (1) {
				res = hid_read_timeout(handle, buf, sizeof(buf), 64);
				/*for (int i = 0; i <= 63; i++) {
					printf("%i: %02x ", i, buf[i]);
				}
				printf("\n");
				printf("\n");*/
				if (buf[0] == 0x21) {
					if (buf[14] == 0x5A) {// Mcu mode is ringcon polling
						goto step13;
					}
				}
				retries++;
				if (retries > 8 || res == 0)
					break;
			}
		}

	step13:

		//Set ext config
		/*printf("Set Ext Config 58 - 4 5 1 2...\n");
		set_ext_config(0x04, 0x05, 0x01, 0x02);
		printf("Set Ext Config 58 - 4 0 0 2...\n");
		set_ext_config(0x04, 0x00, 0x00, 0x02);
		printf("Set Ext Config 58 - 4 4 5 2...\n");
		set_ext_config(0x04, 0x04, 0x05, 0x02);
		printf("Set Ext Config 58 - 4 0 0 2...\n");
		set_ext_config(0x04, 0x00, 0x00, 0x02);
		printf("Set Ext Config 58 - 4 4 1 2...\n");
		set_ext_config(0x04, 0x04, 0x01, 0x02);
		printf("Set Ext Config 58 - 4 4 2 2...\n");
		set_ext_config(0x04, 0x04, 0x02, 0x02);
		printf("Set Ext Config 58 - 4 4 4 2...\n");
		set_ext_config(0x04, 0x04, 0x04, 0x02);
		printf("Set Ext Config 58 - 4 4 3 2...\n");
		set_ext_config(0x04, 0x04, 0x03, 0x02);
		printf("Set Ext Config 58 - 4 4 32 2...\n");
		set_ext_config(0x04, 0x04, 0x32, 0x02);
		printf("Set Ext Config 58 - 4 0 1 2...\n");
		set_ext_config(0x04, 0x00, 0x01, 0x02);
		printf("Set Ext Config 58 - 4 0 0 2...\n");
		set_ext_config(0x04, 0x00, 0x00, 0x02);
		printf("Set Ext Config 58 - 4 4 1 2...\n");
		set_ext_config(0x04, 0x04, 0x01, 0x02);
		printf("Set Ext Config 58 - 4 4 2 2...\n");
		set_ext_config(0x04, 0x04, 0x02, 0x02);
		printf("Set Ext Config 58 - 4 4 4 2...\n");
		set_ext_config(0x04, 0x04, 0x04, 0x02);
		printf("Set Ext Config 58 - 4 4 3 2...\n");
		set_ext_config(0x04, 0x04, 0x03, 0x02);
		printf("Set Ext Config 58 - 4 0 0 2...\n");
		set_ext_config(0x04, 0x00, 0x00, 0x02);
		printf("Set Ext Config 58 - 4 4 11 2...\n");
		set_ext_config(0x04, 0x04, 0x11, 0x02);*/
		printf("Set Ext Config 58 - 4 4 12 2...\n");
		set_ext_config(0x04, 0x04, 0x12, 0x02);
		/*printf("Set Ext Config 58 - 4 4 13 2...\n");
		set_ext_config(0x04, 0x04, 0x13, 0x02);*/

		goto step20;

	/*step14:
		//Set MCU Mode
		while (1) {
			printf("Disable ext polling 5b...\n");

			static int output_buffer_length = 49;
			int res = 0;

			memset(buf, 0, sizeof(buf));
			auto hdr = (brcm_hdr*)buf;
			hdr->cmd = 0x01;
			hdr->rumble[0] = timing_byte & 0xF;
			timing_byte++;
			buf[10] = 0x5b;

			for (int i = 0; i <= 48; i++) {
				printf("%i: %02x ", i, buf[i]);
			}
			printf("\n");
			printf("\n");

			res = hid_write(handle, buf, output_buffer_length);
			int retries = 0;
			Sleep(300);

			/*
			for (int i = 0; i < 50; i++) {
				res = hid_read_timeout(handle, buf, sizeof(buf), 64);
				for (int i = 0; i <= 100; i++) {
					printf("%i: %02x ", i, buf[i]);
				}
				printf("\n");
				printf("\n");
			}/

			while (1) {
				res = hid_read_timeout(handle, buf, sizeof(buf), 64);
				Sleep(300);
				for (int i = 0; i <= 60; i++) {
					printf("%i: %02x ", i, buf[i]);
				}
				printf("\n");
				printf("\n");
				if (buf[0] == 0x21) {
					if (buf[14] == 0x5b) { // Mcu mode is ringcon ready
						goto step15;
					}
				}
				retries++;
				if (retries > 8 || res == 0) {
					break;
				}
			}
		}

	step15:
		// Enable IMU data
		printf("Enabling IMU data 2...\n");
		buf[0] = 0x02;
		send_subcommand(0x01, 0x40, buf, 1);

		while (1) {
			res = hid_read_timeout(handle, buf, sizeof(buf), 64);
			for (int i = 0; i <= 63; i++) {
				printf("%i: %02x ", i, buf[i]);
			}
			printf("\n");
			printf("\n");
			if (buf[0] == 0x21) {
				if (buf[14] == 0x40) {
					break;
				}
			}
		}

		timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		IntToByteArray(timestamp);

		while (1) {
			printf("Get ext dev in format config 5C...\n");

			static int output_buffer_length = 49; //CTCAER 
			int res = 0;

			memset(buf, 0, sizeof(buf));
			auto hdr = (brcm_hdr*)buf;
			hdr->cmd = 0x01;
			hdr->rumble[0] = timing_byte & 0xF;
			timing_byte++;
			buf[10] = 0x5C;
			buf[13] = 0xB0;
			buf[14] = 0xA6;
			buf[15] = 0x2B;
			buf[19] = 0x1C;
			buf[20] = 0x16;
			buf[21] = 0xCD;
			buf[22] = 0xA6;
			buf[23] = 0x2B;
			buf[27] = timestampbuffer[0];// 0x74;//27-34 is the timestamp
			buf[28] = timestampbuffer[1];//0x70;//
			buf[29] = timestampbuffer[2];//0xA0;//
			buf[30] = timestampbuffer[3];//0xBA;//
			buf[31] = timestampbuffer[4];//0x91;//
			buf[32] = timestampbuffer[5];//0x0A;//
			buf[33] = timestampbuffer[6];//0x00;//
			buf[34] = timestampbuffer[7];//0x00;//
			buf[35] = 0x60;
			buf[36] = 0x31;
			buf[37] = 0xD4;
			buf[38] = 0xA6;
			buf[39] = 0x2B;
			buf[44] = 0xF2;
			buf[45] = 0x05;
			buf[46] = 0x2A;
			buf[47] = 0x01;

			//21/10/20 - 92 0 0 176 166 43 0 0 0 28 22 205 166 43 0 0 0 55 35 55 184 160 13 0 0 96 49 212 166 43 0 0 0 0 242 5 42 1 0

			for (int i = 0; i <= 48; i++) {
				printf("%i: %02x ", i, buf[i]);
			}
			printf("\n");
			printf("\n");

			res = hid_write(handle, buf, output_buffer_length);
			int retries = 0;

			while (1) {
				res = hid_read_timeout(handle, buf, sizeof(buf), 64);
				for (int i = 0; i <= 60; i++) {
					printf("%i: %02x ", i, buf[i]);
				}
				printf("\n");
				printf("\n");
				if (buf[0] == 0x21) {
					if (buf[14] == 0x5C) // External config set
						goto step16;
				}
				retries++;
				if (retries > 8 || res == 0)
					break;
			}
		}
		
	step16:
		while (1) {
			printf("Disabling MCU data 21 21 1 0...\n"); // This changes the coninfo back to 8A when 15.09 17.01. It also has a report after ththe 8a change  of 15.09 17.00

			static int output_buffer_length = 49; //CTCAER - USE THIS - for some reason the pkt sub command was positioned right but all the subcommand 21 21 stuff was offset plus one byte
			int res = 0;

			memset(buf, 0, sizeof(buf));
			auto hdr = (brcm_hdr*)buf;
			hdr->cmd = 0x01;
			hdr->rumble[0] = timing_byte & 0xF;
			timing_byte++;
			buf[10] = 0x21;
			buf[11] = 0x21;
			buf[12] = 0x01;
			buf[13] = 0x00;

			buf[48] = mcu_crc8_calc(buf + 12, 36);

			for (int i = 0; i <= 48; i++) {
				printf("%i: %02x ", i, buf[i]);
			}
			printf("\n");
			printf("\n");

			res = hid_write(handle, buf, output_buffer_length);
			int retries = 0;
			Sleep(100);

			while (1) {
				res = hid_read_timeout(handle, buf, sizeof(buf), 64);
				for (int i = 0; i <= 100; i++) {
					printf("%i: %02x ", i, buf[i]);
				}
				printf("\n");
				printf("\n");
				if (buf[0] == 0x21) {
					if (buf[15] == 0x09 && buf[17] == 0x01) {
						goto step17;
					}
				}
				retries++;
				if (retries > 8 || res == 0)
					break;
			}
		}


	step17:
		// Disable MCU
		while (1) {
			printf("Disabling MCU data 22 0...\n");
			buf[0] = 0x00; // Enabled - 00 = suspend, 01 = resume
			send_subcommand(0x01, 0x22, buf, 1);
			int retries = 0;
			while (1) {
				int res = hid_read_timeout(handle, buf, sizeof(buf), 64);
				for (int i = 0; i <= 63; i++) {
					printf("%i: %02x ", i, buf[i]);
				}
				printf("\n");
				printf("\n");
				if (buf[14] == 0x22) {
					//goto step1701;
					goto step1;
				}

				retries++;
				if (retries > 8 || res == 0)
					//break;
					goto step1;
			}
		}

	step1701:
		// Enable MCU data
		while (1) {
			printf("Enabling MCU data 22 1...\n");
			buf[0] = 0x01; // Enabled - 00 = suspend, 01 = resume
			send_subcommand(0x01, 0x22, buf, 1);
			int retries = 0;
			while (1) {
				int res = hid_read_timeout(handle, buf, sizeof(buf), 64);
				for (int i = 0; i <= 63; i++) {
					printf("%i: %02x ", i, buf[i]);
				}
				printf("\n");
				printf("\n");
				if (*(u16*)&buf[0xD] == 0x2280)
					goto step18;

				retries++;
				if (retries > 8 || res == 0)
					break;
			}
		}

	step18:

		while (1) {
			printf("Enabling MCU data 21 21 0 3...\n");
			static int output_buffer_length = 49; //CTCAER - USE THIS - for some reason the pkt sub command was positioned right but all the subcommand 21 21 stuff was offset plus one byte
			int res = 0;

			memset(buf, 0, sizeof(buf));
			auto hdr = (brcm_hdr*)buf;
			//auto pkt = (brcm_cmd_01*)(hdr+1);
			hdr->cmd = 0x01;
			hdr->rumble[0] = timing_byte & 0xF;
			timing_byte++;
			buf[10] = 0x21;
			buf[11] = 0x21;
			buf[12] = 0x00;
			buf[13] = 0x03;
			//pkt->subcmd_21_21.mcu_cmd = 0x21; // Set MCU mode cmd
			//pkt->subcmd_21_21.mcu_subcmd = 0x00; // Set MCU mode cmd
			//pkt->subcmd_21_21.mcu_mode = 0x03; // MCU mode - 1: Standby, 4: NFC, 5: IR, 6: Initializing/FW Update?

			buf[48] = mcu_crc8_calc(buf + 12, 36);
			for (int i = 0; i <= 48; i++) {
				printf("%i: %02x ", i, buf[i]);
			}
			printf("\n");

			res = hid_write(handle, buf, output_buffer_length);
			int retries = 0;

			while (1) {
				res = hid_read_timeout(handle, buf, sizeof(buf), 64);
				for (int i = 0; i <= 48; i++) {
					printf("%i: %02x ", i, buf[i]);
				}
				printf("\n");
				if (buf[0] == 0x21) {
					// *(u16*)buf[18]LE x04 in lower than 3.89fw, x05 in 3.89
					// *(u16*)buf[20]LE x12 in lower than 3.89fw, x18 in 3.89
					if (buf[15] == 0x01 && buf[22] == 0x03) // Mcu mode is standby
						goto step19;
				}
				break;
			}
		}

	step19:
		while (1) {
			printf("Enabling MCU data 21 21 1 1...\n");

			static int output_buffer_length = 49; //CTCAER
			int res = 0;

			memset(buf, 0, sizeof(buf));
			auto hdr = (brcm_hdr*)buf;
			hdr->cmd = 0x01;
			hdr->rumble[0] = timing_byte & 0xF;
			timing_byte++;
			buf[10] = 0x21;
			buf[11] = 0x21;
			buf[12] = 0x01;
			buf[13] = 0x01;

			buf[48] = mcu_crc8_calc(buf + 12, 36);

			for (int i = 0; i <= 48; i++) {
				printf("%i: %02x ", i, buf[i]);
			}
			printf("\n");
			printf("\n");

			res = hid_write(handle, buf, output_buffer_length);
			int retries = 0;
			Sleep(50);

			while (1) {
				res = hid_read_timeout(handle, buf, sizeof(buf), 64);
				Sleep(100);
				for (int i = 0; i <= 60; i++) {
					printf("%i: %02x ", i, buf[i]);
				}
				printf("\n");
				printf("\n");
				if (buf[2] == 0x8b || buf[2] == 0x6b || buf[2] == 0x4b || buf[2] == 0x2b) {//(buf[15] == 0x01 && buf[16] == 0x32) { // No idea, is 16 an error code with 0x32?
					goto step20;
				}
				retries++;
				if (retries > 8 || res == 0) {

				}
			}
		}*/

step20:

	// Enable IMU data
		//printf("Enabling IMU data...\n");
		//buf[0] = 0x01; // Enabled
		//send_subcommand(0x01, 0x40, buf, 1);
		//GetCalibrationData();
		// Enable IMU data
		//printf("Enabling IMU data 2...\n");
		//buf[0] = 0x02;
		//send_subcommand(0x01, 0x40, buf, 1);


		printf("Successfully initialized %s!\n", this->name.c_str());
		return 0;
	}

	void init_usb() {

		this->bluetooth = false;

		unsigned char buf[0x400];
		memset(buf, 0, 0x400);

		// set blocking:
		// this insures we get the MAC Address
		hid_set_nonblocking(this->handle, 0);

		//Get MAC Left
		printf("Getting MAC...\n");
		memset(buf, 0x00, 0x40);
		buf[0] = 0x80;
		buf[1] = 0x01;
		hid_exchange(this->handle, buf, 0x2);

		if (buf[2] == 0x3) {
			printf("%s disconnected!\n", this->name.c_str());
		}
		else {
			printf("Found %s, MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", this->name.c_str(), buf[9], buf[8], buf[7], buf[6], buf[5], buf[4]);
		}

		// set non-blocking:
		//hid_set_nonblocking(jc->handle, 1);

		// Do handshaking
		printf("Doing handshake...\n");
		memset(buf, 0x00, 0x40);
		buf[0] = 0x80;
		buf[1] = 0x02;
		hid_exchange(this->handle, buf, 0x2);

		// Switch baudrate to 3Mbit
		printf("Switching baudrate...\n");
		memset(buf, 0x00, 0x40);
		buf[0] = 0x80;
		buf[1] = 0x03;
		hid_exchange(this->handle, buf, 0x2);

		//Do handshaking again at new baudrate so the firmware pulls pin 3 low?
		printf("Doing handshake...\n");
		memset(buf, 0x00, 0x40);
		buf[0] = 0x80;
		buf[1] = 0x02;
		hid_exchange(this->handle, buf, 0x2);

		//Only talk HID from now on
		printf("Only talk HID...\n");
		memset(buf, 0x00, 0x40);
		buf[0] = 0x80;
		buf[1] = 0x04;
		hid_exchange(this->handle, buf, 0x2);

		// Enable vibration
		printf("Enabling vibration...\n");
		memset(buf, 0x00, 0x400);
		buf[0] = 0x01; // Enabled
		send_subcommand(0x1, 0x48, buf, 1);

		// Enable IMU data
		printf("Enabling IMU data...\n");
		memset(buf, 0x00, 0x400);
		buf[0] = 0x01; // Enabled
		send_subcommand(0x1, 0x40, buf, 1);

		printf("Successfully initialized %s!\n", this->name.c_str());
	}

	void deinit_usb() {
		unsigned char buf[0x40];
		memset(buf, 0x00, 0x40);

		//Let the Joy-Con talk BT again    
		buf[0] = 0x80;
		buf[1] = 0x05;
		hid_exchange(this->handle, buf, 0x2);
		printf("Deinitialized %s\n", this->name.c_str());
	}


	// calibrated sticks:
	// Credit to Hypersect (Ryan Juckett)
	// http://blog.hypersect.com/interpreting-analog-sticks/
	void CalcAnalogStick() {

		if (this->left_right == 1) {
			CalcAnalogStick2(
				this->stick.CalX,
				this->stick.CalY,
				this->stick.x,
				this->stick.y,
				this->stick_cal_x_l,
				this->stick_cal_y_l);

		}
		else if (this->left_right == 2) {
			CalcAnalogStick2(
				this->stick.CalX,
				this->stick.CalY,
				this->stick.x,
				this->stick.y,
				this->stick_cal_x_r,
				this->stick_cal_y_r);

		}
		else if (this->left_right == 3) {
			CalcAnalogStick2(
				this->stick.CalX,
				this->stick.CalY,
				this->stick.x,
				this->stick.y,
				this->stick_cal_x_l,
				this->stick_cal_y_l);

			CalcAnalogStick2(
				this->stick2.CalX,
				this->stick2.CalY,
				this->stick2.x,
				this->stick2.y,
				this->stick_cal_x_r,
				this->stick_cal_y_r);
		}
	}


	void CalcAnalogStick2
	(
		float& pOutX,       // out: resulting stick X value
		float& pOutY,       // out: resulting stick Y value
		uint16_t x,              // in: initial stick X value
		uint16_t y,              // in: initial stick Y value
		uint16_t x_calc[3],      // calc -X, CenterX, +X
		uint16_t y_calc[3]       // calc -Y, CenterY, +Y
	)
	{

		float x_f, y_f;
		// Apply Joy-Con center deadzone. 0xAE translates approx to 15%. Pro controller has a 10% () deadzone
		float deadZoneCenter = 0.15f;
		// Add a small ammount of outer deadzone to avoid edge cases or machine variety.
		float deadZoneOuter = 0.10f;

		// convert to float based on calibration and valid ranges per +/-axis
		x = clamp(x, x_calc[0], x_calc[2]);
		y = clamp(y, y_calc[0], y_calc[2]);
		if (x >= x_calc[1]) {
			x_f = (float)(x - x_calc[1]) / (float)(x_calc[2] - x_calc[1]);
		}
		else {
			x_f = -((float)(x - x_calc[1]) / (float)(x_calc[0] - x_calc[1]));
		}
		if (y >= y_calc[1]) {
			y_f = (float)(y - y_calc[1]) / (float)(y_calc[2] - y_calc[1]);
		}
		else {
			y_f = -((float)(y - y_calc[1]) / (float)(y_calc[0] - y_calc[1]));
		}

		// Interpolate zone between deadzones
		float mag = sqrtf(x_f * x_f + y_f * y_f);
		if (mag > deadZoneCenter) {
			// scale such that output magnitude is in the range [0.0f, 1.0f]
			float legalRange = 1.0f - deadZoneOuter - deadZoneCenter;
			float normalizedMag = std::min(1.0f, (mag - deadZoneCenter) / legalRange);
			float scale = normalizedMag / mag;
			pOutX = (x_f * scale);
			pOutY = (y_f * scale);
		}
		else {
			// stick is in the inner dead zone
			pOutX = 0.0f;
			pOutY = 0.0f;
		}
	}

	// SPI (@CTCaer):

	int get_spi_data(uint32_t offset, const uint16_t read_len, uint8_t* test_buf) {
		int res;
		uint8_t buf[0x100];
		while (1) {
			memset(buf, 0, sizeof(buf));
			auto hdr = (brcm_hdr*)buf;
			auto pkt = (brcm_cmd_01*)(hdr + 1);
			hdr->cmd = 1;
			hdr->rumble[0] = timing_byte;

			buf[1] = timing_byte;

			timing_byte++;
			if (timing_byte > 0xF) {
				timing_byte = 0x0;
			}
			pkt->subcmd = 0x10;
			pkt->offset = offset;
			pkt->size = read_len;

			for (int i = 11; i < 22; ++i) {
				buf[i] = buf[i + 3];
			}

			res = hid_write(handle, buf, sizeof(*hdr) + sizeof(*pkt));

			res = hid_read(handle, buf, sizeof(buf));

			if ((*(uint16_t*)&buf[0xD] == 0x1090) && (*(uint32_t*)&buf[0xF] == offset)) {
				break;
			}
		}
		if (res >= 0x14 + read_len) {
			for (int i = 0; i < read_len; i++) {
				test_buf[i] = buf[0x14 + i];
			}
		}

		return 0;
	}

	int write_spi_data(uint32_t offset, const uint16_t write_len, uint8_t* test_buf) {
		int res;
		uint8_t buf[0x100];
		int error_writing = 0;
		while (1) {
			memset(buf, 0, sizeof(buf));
			auto hdr = (brcm_hdr*)buf;
			auto pkt = (brcm_cmd_01*)(hdr + 1);
			hdr->cmd = 1;
			hdr->rumble[0] = timing_byte;
			timing_byte++;
			if (timing_byte > 0xF) {
				timing_byte = 0x0;
			}
			pkt->subcmd = 0x11;
			pkt->offset = offset;
			pkt->size = write_len;
			for (int i = 0; i < write_len; i++) {
				buf[0x10 + i] = test_buf[i];
			}
			res = hid_write(handle, buf, sizeof(*hdr) + sizeof(*pkt) + write_len);

			res = hid_read(handle, buf, sizeof(buf));

			if (*(uint16_t*)&buf[0xD] == 0x1180)
				break;

			error_writing++;
			if (error_writing == 125) {
				return 1;
			}
		}

		return 0;

	}



	//int write_spi_data(uint32_t offset, const uint16_t write_len, uint8_t* test_buf) {
	//	int res;
	//	uint8_t buf[0x100];
	//	int error_writing = 0;
	//	while (1) {
	//		memset(buf, 0, sizeof(buf));
	//		auto hdr = (brcm_hdr *)buf;
	//		auto pkt = (brcm_cmd_01 *)(hdr + 1);
	//		hdr->cmd = 1;
	//		hdr->timer = timing_byte & 0xF;
	//		timing_byte++;
	//		pkt->subcmd = 0x11;
	//		pkt->spi_read.offset = offset;
	//		pkt->spi_read.size = write_len;
	//		for (int i = 0; i < write_len; i++) {
	//			buf[0x10 + i] = test_buf[i];
	//		}

	//		res = hid_write(handle, buf, sizeof(*hdr) + sizeof(*pkt) + write_len);

	//		res = hid_read(handle, buf, sizeof(buf));
	//		if (*(uint16_t*)&buf[0xD] == 0x1180) {
	//			break;
	//		}
	//		error_writing++;
	//		if (error_writing == 125) {
	//			return 1;
	//		}
	//	}

	//	return 0;
	//}



	//int get_spi_data(uint32_t offset, const uint16_t read_len, uint8_t* test_buf) {
	//	int res;
	//	uint8_t buf[0x100];
	//	while (1) {
	//		memset(buf, 0, sizeof(buf));
	//		auto hdr = (brcm_hdr *)buf;
	//		auto pkt = (brcm_cmd_01 *)(hdr + 1);
	//		hdr->cmd = 1;
	//		hdr->timer = timing_byte & 0xF;
	//		timing_byte++;
	//		pkt->subcmd = 0x10;
	//		pkt->spi_read.offset = offset;
	//		pkt->spi_read.size = read_len;
	//		res = hid_write(handle, buf, sizeof(*hdr) + sizeof(*pkt));

	//		res = hid_read(handle, buf, sizeof(buf));
	//		if ((*(u16*)&buf[0xD] == 0x1090) && (*(uint32_t*)&buf[0xF] == offset)) {
	//			break;
	//		}
	//	}
	//	if (res >= 0x14 + read_len) {
	//		for (int i = 0; i < read_len; i++) {
	//			test_buf[i] = buf[0x14 + i];
	//		}
	//	}

	//	return 0;
	//}
	void GetCalibrationData() {
		printf("Getting calibration data...\n");
		memset(factory_stick_cal, 0, 0x12);
		memset(user_stick_cal, 0, 0x16);
		memset(sensor_model, 0, 0x6);
		memset(stick_model, 0, 0x12);
		memset(factory_sensor_cal, 0, 0x18);
		memset(user_sensor_cal, 0, 0x1A);
		memset(factory_sensor_cal_calm, 0, 0xC);
		memset(user_sensor_cal_calm, 0, 0xC);
		memset(sensor_cal, 0, sizeof(sensor_cal));
		memset(stick_cal_x_l, 0, sizeof(stick_cal_x_l));
		memset(stick_cal_y_l, 0, sizeof(stick_cal_y_l));
		memset(stick_cal_x_r, 0, sizeof(stick_cal_x_r));
		memset(stick_cal_y_r, 0, sizeof(stick_cal_y_r));


		get_spi_data(0x6020, 0x18, factory_sensor_cal);
		get_spi_data(0x603D, 0x12, factory_stick_cal);
		get_spi_data(0x6080, 0x6, sensor_model);
		get_spi_data(0x6086, 0x12, stick_model);
		get_spi_data(0x6098, 0x12, &stick_model[0x12]);
		get_spi_data(0x8010, 0x16, user_stick_cal);
		get_spi_data(0x8026, 0x1A, user_sensor_cal);


		// get stick calibration data:

		// factory calibration:

		if (this->left_right == 1 || this->left_right == 3) {
			stick_cal_x_l[1] = (factory_stick_cal[4] << 8) & 0xF00 | factory_stick_cal[3];
			stick_cal_y_l[1] = (factory_stick_cal[5] << 4) | (factory_stick_cal[4] >> 4);
			stick_cal_x_l[0] = stick_cal_x_l[1] - ((factory_stick_cal[7] << 8) & 0xF00 | factory_stick_cal[6]);
			stick_cal_y_l[0] = stick_cal_y_l[1] - ((factory_stick_cal[8] << 4) | (factory_stick_cal[7] >> 4));
			stick_cal_x_l[2] = stick_cal_x_l[1] + ((factory_stick_cal[1] << 8) & 0xF00 | factory_stick_cal[0]);
			stick_cal_y_l[2] = stick_cal_y_l[1] + ((factory_stick_cal[2] << 4) | (factory_stick_cal[2] >> 4));

		}

		if (this->left_right == 2 || this->left_right == 3) {
			stick_cal_x_r[1] = (factory_stick_cal[10] << 8) & 0xF00 | factory_stick_cal[9];
			stick_cal_y_r[1] = (factory_stick_cal[11] << 4) | (factory_stick_cal[10] >> 4);
			stick_cal_x_r[0] = stick_cal_x_r[1] - ((factory_stick_cal[13] << 8) & 0xF00 | factory_stick_cal[12]);
			stick_cal_y_r[0] = stick_cal_y_r[1] - ((factory_stick_cal[14] << 4) | (factory_stick_cal[13] >> 4));
			stick_cal_x_r[2] = stick_cal_x_r[1] + ((factory_stick_cal[16] << 8) & 0xF00 | factory_stick_cal[15]);
			stick_cal_y_r[2] = stick_cal_y_r[1] + ((factory_stick_cal[17] << 4) | (factory_stick_cal[16] >> 4));
		}


		// if there is user calibration data:
		if ((user_stick_cal[0] | user_stick_cal[1] << 8) == 0xA1B2) {
			stick_cal_x_l[1] = (user_stick_cal[6] << 8) & 0xF00 | user_stick_cal[5];
			stick_cal_y_l[1] = (user_stick_cal[7] << 4) | (user_stick_cal[6] >> 4);
			stick_cal_x_l[0] = stick_cal_x_l[1] - ((user_stick_cal[9] << 8) & 0xF00 | user_stick_cal[8]);
			stick_cal_y_l[0] = stick_cal_y_l[1] - ((user_stick_cal[10] << 4) | (user_stick_cal[9] >> 4));
			stick_cal_x_l[2] = stick_cal_x_l[1] + ((user_stick_cal[3] << 8) & 0xF00 | user_stick_cal[2]);
			stick_cal_y_l[2] = stick_cal_y_l[1] + ((user_stick_cal[4] << 4) | (user_stick_cal[3] >> 4));
			//FormJoy::myform1->textBox_lstick_ucal->Text = String::Format(L"L Stick User:\r\nCenter X,Y: ({0:X3}, {1:X3})\r\nX: [{2:X3} - {4:X3}] Y: [{3:X3} - {5:X3}]",
				//stick_cal_x_l[1], stick_cal_y_l[1], stick_cal_x_l[0], stick_cal_y_l[0], stick_cal_x_l[2], stick_cal_y_l[2]);
		}
		else {
			//FormJoy::myform1->textBox_lstick_ucal->Text = L"L Stick User:\r\nNo calibration";
			//printf("no user Calibration data for left stick.\n");
		}

		if ((user_stick_cal[0xB] | user_stick_cal[0xC] << 8) == 0xA1B2) {
			stick_cal_x_r[1] = (user_stick_cal[14] << 8) & 0xF00 | user_stick_cal[13];
			stick_cal_y_r[1] = (user_stick_cal[15] << 4) | (user_stick_cal[14] >> 4);
			stick_cal_x_r[0] = stick_cal_x_r[1] - ((user_stick_cal[17] << 8) & 0xF00 | user_stick_cal[16]);
			stick_cal_y_r[0] = stick_cal_y_r[1] - ((user_stick_cal[18] << 4) | (user_stick_cal[17] >> 4));
			stick_cal_x_r[2] = stick_cal_x_r[1] + ((user_stick_cal[20] << 8) & 0xF00 | user_stick_cal[19]);
			stick_cal_y_r[2] = stick_cal_y_r[1] + ((user_stick_cal[21] << 4) | (user_stick_cal[20] >> 4));
			//FormJoy::myform1->textBox_rstick_ucal->Text = String::Format(L"R Stick User:\r\nCenter X,Y: ({0:X3}, {1:X3})\r\nX: [{2:X3} - {4:X3}] Y: [{3:X3} - {5:X3}]",
				//stick_cal_x_r[1], stick_cal_y_r[1], stick_cal_x_r[0], stick_cal_y_r[0], stick_cal_x_r[2], stick_cal_y_r[2]);
		}
		else {
			//FormJoy::myform1->textBox_rstick_ucal->Text = L"R Stick User:\r\nNo calibration";
			//printf("no user Calibration data for right stick.\n");
		}

		// get gyro / accelerometer calibration data:

		// factory calibration:

		// Acc cal origin position
		sensor_cal[0][0] = uint16_to_int16(factory_sensor_cal[0] | factory_sensor_cal[1] << 8);
		sensor_cal[0][1] = uint16_to_int16(factory_sensor_cal[2] | factory_sensor_cal[3] << 8);
		sensor_cal[0][2] = uint16_to_int16(factory_sensor_cal[4] | factory_sensor_cal[5] << 8);

		// Gyro cal origin position
		sensor_cal[1][0] = uint16_to_int16(factory_sensor_cal[0xC] | factory_sensor_cal[0xD] << 8);
		sensor_cal[1][1] = uint16_to_int16(factory_sensor_cal[0xE] | factory_sensor_cal[0xF] << 8);
		sensor_cal[1][2] = uint16_to_int16(factory_sensor_cal[0x10] | factory_sensor_cal[0x11] << 8);

		// user calibration:
		if ((user_sensor_cal[0x0] | user_sensor_cal[0x1] << 8) == 0xA1B2) {
			//FormJoy::myform1->textBox_6axis_ucal->Text = L"6-Axis User (XYZ):\r\nAcc:  ";
			//for (int i = 0; i < 0xC; i = i + 6) {
			//	FormJoy::myform1->textBox_6axis_ucal->Text += String::Format(L"{0:X4} {1:X4} {2:X4}\r\n      ",
			//		user_sensor_cal[i + 2] | user_sensor_cal[i + 3] << 8,
			//		user_sensor_cal[i + 4] | user_sensor_cal[i + 5] << 8,
			//		user_sensor_cal[i + 6] | user_sensor_cal[i + 7] << 8);
			//}
			// Acc cal origin position
			sensor_cal[0][0] = uint16_to_int16(user_sensor_cal[2] | user_sensor_cal[3] << 8);
			sensor_cal[0][1] = uint16_to_int16(user_sensor_cal[4] | user_sensor_cal[5] << 8);
			sensor_cal[0][2] = uint16_to_int16(user_sensor_cal[6] | user_sensor_cal[7] << 8);
			//FormJoy::myform1->textBox_6axis_ucal->Text += L"\r\nGyro: ";
			//for (int i = 0xC; i < 0x18; i = i + 6) {
			//	FormJoy::myform1->textBox_6axis_ucal->Text += String::Format(L"{0:X4} {1:X4} {2:X4}\r\n      ",
			//		user_sensor_cal[i + 2] | user_sensor_cal[i + 3] << 8,
			//		user_sensor_cal[i + 4] | user_sensor_cal[i + 5] << 8,
			//		user_sensor_cal[i + 6] | user_sensor_cal[i + 7] << 8);
			//}
			// Gyro cal origin position
			sensor_cal[1][0] = uint16_to_int16(user_sensor_cal[0xE] | user_sensor_cal[0xF] << 8);
			sensor_cal[1][1] = uint16_to_int16(user_sensor_cal[0x10] | user_sensor_cal[0x11] << 8);
			sensor_cal[1][2] = uint16_to_int16(user_sensor_cal[0x12] | user_sensor_cal[0x13] << 8);
		}
		else {
			//FormJoy::myform1->textBox_6axis_ucal->Text = L"\r\n\r\nUser:\r\nNo calibration";
		}

		// Use SPI calibration and convert them to SI acc unit
		acc_cal_coeff[0] = (float)(1.0 / (float)(16384 - uint16_to_int16(sensor_cal[0][0]))) * 4.0f * 9.8f;
		acc_cal_coeff[1] = (float)(1.0 / (float)(16384 - uint16_to_int16(sensor_cal[0][1]))) * 4.0f * 9.8f;
		acc_cal_coeff[2] = (float)(1.0 / (float)(16384 - uint16_to_int16(sensor_cal[0][2]))) * 4.0f * 9.8f;

		// Use SPI calibration and convert them to SI gyro unit
		gyro_cal_coeff[0] = (float)(936.0 / (float)(13371 - uint16_to_int16(sensor_cal[1][0])) * 0.01745329251994);
		gyro_cal_coeff[1] = (float)(936.0 / (float)(13371 - uint16_to_int16(sensor_cal[1][1])) * 0.01745329251994);
		gyro_cal_coeff[2] = (float)(936.0 / (float)(13371 - uint16_to_int16(sensor_cal[1][2])) * 0.01745329251994);
	}
};
