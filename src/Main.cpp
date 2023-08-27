#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#pragma comment(lib, "user32.lib")

#include <bitset>
#include <random>
#include "stdafx.h"
#include <string.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <fstream>

#include <hidapi.h>

#include "packet.h"
#include "joycon.hpp"
#include "MouseController.hpp"
#include "tools.hpp"

#if defined(_WIN32)
#include <Windows.h>
#include <Lmcons.h>
#include <shlobj.h>
#endif

// #include "public.h"

// sio:
// #include "sio_client.h"

// Vigem
#include <Xinput.h>
// #include <ViGEm/Client.h>
#pragma comment(lib, "setupapi.lib")

// GLM - Used for the complementary filter (gyro/accel)
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#pragma warning(disable : 4996)

#define JOYCON_VENDOR 0x057e
#define JOYCON_L_BT 0x2006
#define JOYCON_R_BT 0x2007
#define PRO_CONTROLLER 0x2009
#define JOYCON_CHARGING_GRIP 0x200e
#define SERIAL_LEN 18
#define PI 3.14159265359
#define L_OR_R(lr) (lr == 1 ? 'L' : (lr == 2 ? 'R' : '?'))

std::vector<Joycon> joycons;
MouseController MC;
unsigned char buf[65];
int res = 0;

// sio:
// sio::client myClient;

int subloop = 0;

int Ringcon = 0x0A;
int prevRingcon = 0x0A;
int ringconcounter = 0;

#define runarraylength 50
int runningindex[runarraylength] = {0};
int runvalue = 0;
int squatvalue = 0;
bool running = false;
bool ringconattached = false;
bool squatting = false;

float validroll = 0.00;
float validpitch = 0.00;

bool leftmousedown = false;
bool rightmousedown = false;
float squatmousemult = 1; // Slow down mouse movement depending on how far you squat. Makes Ringcon clicking easier.

bool p1ready = false;
bool p2ready = false;

// Init VigEm
// const auto client = vigem_alloc();
// const auto retval = vigem_connect(client);
// PVIGEM_TARGET pad1 = 0;
// PVIGEM_TARGET pad2 = 0;
// XUSB_REPORT report;
WORD remappedbtnsr = 0;
WORD remappedbtnsl = 0;
BYTE RightTrigger = 0;
BYTE LeftTrigger = 0;
int MaxStick = 32767;

LONG sThumbLX = 0;
LONG sThumbLY = 0;
SHORT sThumbRX = 0;
SHORT sThumbRY = 0;

struct Settings
{

	// Enabling this combines both JoyCons to a single Vigem Device
	// when combineJoyCons == false:
	// JoyCon(L) is mapped to Vigem Device #1
	// JoyCon(R) is mapped to Vigem Device #2
	// when combineJoyCons == true:
	// JoyCon(L) and JoyCon(R) are mapped to Vigem Device #1
	bool combineJoyCons = false;

	bool reverseX = false; // reverses joystick x (both sticks)
	bool reverseY = false; // reverses joystick y (both sticks)

	bool usingGrip = false;
	bool usingBluetooth = true;
	bool disconnect = false;

	// enables motion controls
	bool enableGyro = false;
	bool squatSlowsMouse = false;

	// gyroscope (mouse) sensitivity:
	float gyroSensitivityX = 150.0f;
	float gyroSensitivityY = 150.0f;

	// prefer the left joycon for gyro controls
	bool preferLeftJoyCon = false;

	// combo code to set key combination to disable gyroscope for quick turning in games. -1 to disable.
	int gyroscopeComboCode = 4;

	// toggles between two different toggle types
	// disabled = traditional toggle
	// enabled = while button(s) are held gyro is enabled
	bool quickToggleGyro = false;

	// inverts the above function
	bool invertQuickToggle = false;

	// for dolphin, mostly:
	// bool dolphinPointerMode = false;

	// so that you don't rapidly toggle the gyro controls every frame:
	bool canToggleGyro = true;

	// plays a version of the mario theme by vibrating
	// the first JoyCon connected.
	bool marioTheme = false;

	// bool to restart the program
	bool restart = false;

	// auto start the program
	bool autoStart = false;

	// Run presses button
	bool Runpressesbutton = false;

	// Some Ringcons do not default to 10 when not being flexed. This is a manual adjustment to make it happen:
	float RingconFix = 0.0f;

	// debug file:
	FILE *outputFile;

	// Use Ringcon with full set of buttons, right handed::
	bool RingconFullRH = false;
	// where to connect:
	std::string host = "";
	// string to send:
	std::string controllerState = "";
	// Use Ringcon with full set of buttons, left handed:
	bool RingconToAnalog = false;

	// poll options:

	// running unlocks the gyro:
	bool rununlocksgyro = false;

	// times to poll per second per joycon:
	float pollsPerSec = 30.0f;

	// time to sleep (in ms) between polls:
	float timeToSleepMS = 1.0f;

	// version number
	std::string version = "1.03";

} settings;

struct Tracker
{

	int var1 = 0;
	int var2 = 0;
	int counter1 = 0;

	float low_freq = 200.0f;
	float high_freq = 500.0f;

	float relX = 0;
	float relY = 0;

	float anglex = 0;
	float angley = 0;
	float anglez = 0;

	glm::fquat quat = glm::angleAxis(0.0f, glm::vec3(1.0, 0.0, 0.0));
	// get current time
	// std::chrono::high_resolution_clock tNow;
	// std::chrono::steady_clock::time_point tPoll = std::chrono::high_resolution_clock::now();
	std::vector<std::chrono::steady_clock::time_point> tPolls;
	// Tracker(int value) : tPolls(100, std::chrono::high_resolution_clock::now()) {}
	// auto tSleepStart = std::chrono::high_resolution_clock::now();

	float previousPitch = 0;
} tracker;

void handle_input(Joycon *jc, uint8_t *packet, int len)
{

	// bluetooth button pressed packet:
	if (packet[0] == 0x3F)
	{

		uint16_t old_buttons = jc->buttons;
		int8_t old_dstick = jc->dstick;

		jc->dstick = packet[3];
		// todo: get button states here aswell:
	}

	// input update packet:
	// 0x21 is just buttons, 0x30 includes gyro, 0x31 includes NFC (large packet size)
	if (packet[0] == 0x30 || packet[0] == 0x31 || packet[0] == 0x32)
	{

		// offset for usb or bluetooth data:
		/*int offset = settings.usingBluetooth ? 0 : 10;*/
		int offset = jc->bluetooth ? 0 : 10;

		uint8_t *btn_data = packet + offset + 3;

		// get button states:
		{
			uint16_t states = 0;
			uint16_t states2 = 0;

			// Left JoyCon:
			if (jc->left_right == 1)
			{
				states = (btn_data[1] << 8) | (btn_data[2] & 0xFF);
				// Right JoyCon:
			}
			else if (jc->left_right == 2)
			{
				states = (btn_data[1] << 8) | (btn_data[0] & 0xFF);
				// Pro Controller:
			}
			else if (jc->left_right == 3)
			{
				states = (btn_data[1] << 8) | (btn_data[2] & 0xFF);
				states2 = (btn_data[1] << 8) | (btn_data[0] & 0xFF);
			}

			jc->buttons = states;
			// Pro Controller:
			if (jc->left_right == 3)
			{
				jc->buttons2 = states2;

				// fix some non-sense the Pro Controller does
				// clear nth bit
				// num &= ~(1UL << n);
				jc->buttons &= ~(1UL << 9);
				jc->buttons &= ~(1UL << 10);
				jc->buttons &= ~(1UL << 12);
				jc->buttons &= ~(1UL << 14);

				jc->buttons2 &= ~(1UL << 8);
				jc->buttons2 &= ~(1UL << 11);
				jc->buttons2 &= ~(1UL << 13);
			}
		}

		// get stick data:
		uint8_t *stick_data = packet + offset;
		if (jc->left_right == 1)
		{
			stick_data += 6;
		}
		else if (jc->left_right == 2)
		{
			stick_data += 9;
		}

		uint16_t stick_x = stick_data[0] | ((stick_data[1] & 0xF) << 8);
		uint16_t stick_y = (stick_data[1] >> 4) | (stick_data[2] << 4);
		jc->stick.x = stick_x;
		jc->stick.y = stick_y;

		// use calibration data:
		jc->CalcAnalogStick();

		// pro controller:
		if (jc->left_right == 3)
		{
			stick_data += 6;
			uint16_t stick_x = stick_data[0] | ((stick_data[1] & 0xF) << 8);
			uint16_t stick_y = (stick_data[1] >> 4) | (stick_data[2] << 4);
			jc->stick.x = (int)(unsigned int)stick_x;
			jc->stick.y = (int)(unsigned int)stick_y;
			stick_data += 3;
			uint16_t stick_x2 = stick_data[0] | ((stick_data[1] & 0xF) << 8);
			uint16_t stick_y2 = (stick_data[1] >> 4) | (stick_data[2] << 4);
			jc->stick2.x = (int)(unsigned int)stick_x2;
			jc->stick2.y = (int)(unsigned int)stick_y2;

			// calibration data:
			jc->CalcAnalogStick();
		}

		jc->battery = (stick_data[1] & 0xF0) >> 4;
		// printf("JoyCon battery: %d\n", jc->battery);

		// Accelerometer:
		// Accelerometer data is absolute (m/s^2)
		{

			// get accelerometer X:
			jc->accel.x = (float)(uint16_to_int16(packet[13] | (packet[14] << 8) & 0xFF00)) * jc->acc_cal_coeff[0];

			// get accelerometer Y:
			jc->accel.y = (float)(uint16_to_int16(packet[15] | (packet[16] << 8) & 0xFF00)) * jc->acc_cal_coeff[1];

			// get accelerometer Z:
			jc->accel.z = (float)(uint16_to_int16(packet[17] | (packet[18] << 8) & 0xFF00)) * jc->acc_cal_coeff[2];
		}

		// Gyroscope:
		// Gyroscope data is relative (rads/s)
		if (jc->left_right == 2 && ringconattached)
		{
			{

				// get roll:
				jc->gyro.roll = (float)((uint16_to_int16(packet[35] | (packet[36] << 8) & 0xFF00)) - jc->sensor_cal[1][0]) * jc->gyro_cal_coeff[0]; // 23 24 was working

				// get pitch:
				jc->gyro.pitch = (float)((uint16_to_int16(packet[31] | (packet[32] << 8) & 0xFF00)) - jc->sensor_cal[1][1]) * jc->gyro_cal_coeff[1]; // 19 20 was working

				// get yaw:
				jc->gyro.yaw = (float)((uint16_to_int16(packet[33] | (packet[34] << 8) & 0xFF00)) - jc->sensor_cal[1][2]) * jc->gyro_cal_coeff[2]; // 21 22 was working

				// Note: All of the below orientations are from the point of view of the ringcon. May not line up with official terminology.
				// 13-14 Roll
				// 15-16 Pitch centred at horizontal
				// 17-18 Pitch centred at vertical
				// 19-20 Gyro pitch - Forward = +, Backward = -
				// 21-22 Gyro yaw (needed for running) - When running, stepping down = +, stepping up = -
				// 23-24 Gyro roll - Clockwise = +, Anticlockwise = -
				// 25-26 Roll anticlockwise +, clockwise -
				// 27-28 Pitch centred at horizontal - up = -, down = +
				// 29-30 Pitch centred at vertical - up = -, down = +
				// 31-32, 33-34, 35-36 arebouncing around but have something to do with the gyro. maybe i need a single byte?
				// printf("%f      %f     %f", jc->gyro.roll, jc->gyro.yaw, jc->gyro.pitch);
			}
		}
		else
		{

			// get roll:
			jc->gyro.roll = (float)((uint16_to_int16(packet[19] | (packet[20] << 8) & 0xFF00)) - jc->sensor_cal[1][0]) * jc->gyro_cal_coeff[0]; // 23 24 was working, now not so much

			// get pitch:
			jc->gyro.pitch = (float)((uint16_to_int16(packet[21] | (packet[22] << 8) & 0xFF00)) - jc->sensor_cal[1][1]) * jc->gyro_cal_coeff[1]; // 19 20 was working

			// get yaw:
			jc->gyro.yaw = (float)((uint16_to_int16(packet[23] | (packet[24] << 8) & 0xFF00)) - jc->sensor_cal[1][2]) * jc->gyro_cal_coeff[2]; // 21 22 was working
		}

		// offsets:
		{
			jc->setGyroOffsets();

			jc->gyro.roll -= jc->gyro.offset.roll;
			jc->gyro.pitch -= jc->gyro.offset.pitch;
			jc->gyro.yaw -= jc->gyro.offset.yaw;
		}
	}

	// handle button combos:
	{
		bool lightpress = false;
		bool lightpull = false;
		bool heavypress = false;
		bool heavypull = false;

		// right:
		if (jc->left_right == 2 && ringconattached)
		{
			// Ringcon logic - Default values - int prevringcon = 0x0A; int ringconcounter = 0;

			Ringcon = packet[40];

			if (Ringcon == 0x00)
			{ // The Ringcon reading has started randomly putting zero in to the reading, I must not be initializing it properly. This is a hack to get around that.
				Ringcon = prevRingcon;
			}

			Ringcon = Ringcon + settings.RingconFix;

			if (Ringcon >= 100)
			{
				Ringcon = Ringcon - 255;
			}

			if (Ringcon != prevRingcon)
			{
				printf("%i\n", Ringcon);
			}

			if (settings.RingconFullRH)
			{ // The sensor readings change if it is being held sideways
				if (Ringcon == 0x0A || Ringcon == 0x09 || Ringcon == 0x08 || Ringcon == 0x07)
				{ // Deadzone
					ringconcounter = 0;
				}

				if (Ringcon == 0x01 || Ringcon == 0xFF || Ringcon == 0xFE)
				{
					heavypress = false; // turn off heavy press, may damage Ringcon as it goes outside the flex range
					// ringconcounter = -1;
				}
				if (Ringcon == 0x0D || Ringcon == 0x0E || Ringcon == 0x0F)
				{
					heavypull = true;
					ringconcounter = -1;
				}
				if (Ringcon >= 0x02 && Ringcon <= 0x06 && ringconcounter != -1)
				{
					/*if (Ringcon < prevringcon && ringconcounter < 10) {
						ringconcounter = 0;
					}
					else if (Ringcon == prevringcon && ringconcounter < 10) {
						ringconcounter++;
					}
					else {*/
					lightpress = true;
					ringconcounter = 20;
					//}
				}
				if (Ringcon <= 0x0C && Ringcon >= 0x0B && ringconcounter != -1)
				{
					if (Ringcon > prevRingcon && ringconcounter < 10)
					{
						ringconcounter = 0;
					}
					else if (Ringcon == prevRingcon && ringconcounter < 10)
					{
						ringconcounter++;
					}
					else
					{
						lightpull = true;
						ringconcounter = 20;
					}
				}
			}
			else
			{
				if (Ringcon == 0x0A || Ringcon == 0x09 || Ringcon == 0x08 || Ringcon == 0x0B)
				{ // Deadzone
					ringconcounter = 0;
				}

				if (Ringcon >= 0x11)
				{
					heavypress = true;
					ringconcounter = -1;
				}
				if (Ringcon <= 0x03 && Ringcon != 0x00)
				{
					heavypull = true;
					ringconcounter = -1;
				}
				if (Ringcon >= 0x0C && Ringcon <= 0x10 && ringconcounter != -1)
				{
					if (Ringcon > prevRingcon && ringconcounter < 10)
					{
						ringconcounter = 0;
					}
					else if (Ringcon == prevRingcon && ringconcounter < 10)
					{
						ringconcounter++;
					}
					else
					{
						lightpress = true;
						ringconcounter = 20;
					}
				}
				if (Ringcon <= 0x07 && Ringcon >= 0x04 && ringconcounter != -1)
				{
					if (Ringcon < prevRingcon && ringconcounter < 10)
					{
						ringconcounter = 0;
					}
					else if (Ringcon == prevRingcon && ringconcounter < 10)
					{
						ringconcounter++;
					}
					else
					{
						lightpull = true;
						ringconcounter = 20;
					}
				}
			}

			prevRingcon = Ringcon;
			// printf("%i \n\n", Ringcon);
		}

		// left:
		if (jc->left_right == 1)
		{

			// Determine whether the left joycon is telling us we are running
			runningindex[runvalue % runarraylength] = jc->gyro.pitch;
			runvalue++;
			int sum = 0;
			int average = 0;
			for (int i = 0; i < runarraylength; i++)
			{
				if (runningindex[i] >= 0)
				{
					sum += (runningindex[i] * 2);
				}
				else
				{
					sum -= (runningindex[i] * 2); // Too many zeros means the average will be 0 even when there are quite a lot of numbers with values. This seems to be a good value with arraylength at 50.
				}
			}

			average = sum / runarraylength;

			// printf("%i\n", average); //walk 0-1, jog 1-2, run 2-3, sprint 3-4
			if (average > 0)
			{
				running = true;
				if (settings.Runpressesbutton)
				{
					jc->buttons |= 1U << 4; // sr = run
				}
			}
			else
			{
				running = false;
			}
			jc->running = running;
			// jc->btns.sl = (jc->buttons & (1 << 5)) ? 1 : 0; // set a bit: *ptr |= 1 << index;
			// sprint button
			if (average >= 3)
			{							// sprint
				jc->buttons |= 1U << 5; // sl = sprint
			}
			// int squatvalue = 0;
			// printf("%f", jc->accel.z); //9.8 when horizontal. 0 when vertical. Goes to minus when facing down or backwards.
			if (jc->accel.z > 6.0 && jc->accel.z < 12.0)
			{
				squatvalue++;
				if (squatvalue >= 20 && !settings.squatSlowsMouse)
				{
					jc->buttons |= 1U << 8; // jc->btns.minus = (jc->buttons & (1 << 8)) ? 1 : 0;
				}
			}
			else
			{
				squatvalue = 0;
			}

			if (jc->accel.z > 9.5 && jc->accel.z < 12.0)
			{
				squatting = true;
			}
			else
			{
				squatting = false;
			}
			jc->squatting = squatting;

			if (settings.squatSlowsMouse && !running)
			{
				if (jc->accel.z <= 0.1)
				{
					squatmousemult = 1;
				}
				else if (jc->accel.z >= 9.0)
				{
					squatmousemult = 0.1;
				}
				else
				{
					squatmousemult = 1 - (jc->accel.z * 0.1);
				}
			}
			else
			{
				squatmousemult = 1;
			}

			// Mouse buttons
			if (settings.enableGyro && !settings.combineJoyCons)
			{
				if (jc->buttons & (1 << 7) && !leftmousedown)
				{ // ZL controls left mouse button
					MC.LeftClickDown();
					leftmousedown = true;
				}
				if (!(jc->buttons & (1 << 7)) && leftmousedown)
				{ // ZL controls left mouse button
					MC.LeftClickUp();
					leftmousedown = false;
				}
				if (jc->buttons & (1 << 6) && !rightmousedown)
				{ // L controls right mouse button
					MC.RightClickDown();
					rightmousedown = true;
				}
				if (!(jc->buttons & (1 << 6)) && rightmousedown)
				{ // L controls right mouse button
					MC.RightClickUp();
					rightmousedown = false;
				}
			}

			jc->btns.down = (jc->buttons & (1 << 0)) ? 1 : 0;
			jc->btns.up = (jc->buttons & (1 << 1)) ? 1 : 0;
			jc->btns.right = (jc->buttons & (1 << 2)) ? 1 : 0;
			jc->btns.left = (jc->buttons & (1 << 3)) ? 1 : 0;
			jc->btns.sr = (jc->buttons & (1 << 4)) ? 1 : 0;
			jc->btns.sl = (jc->buttons & (1 << 5)) ? 1 : 0;
			jc->btns.l = (jc->buttons & (1 << 6)) ? 1 : 0;
			jc->btns.zl = (jc->buttons & (1 << 7)) ? 1 : 0;
			jc->btns.minus = (jc->buttons & (1 << 8)) ? 1 : 0;
			jc->btns.stick_button = (jc->buttons & (1 << 11)) ? 1 : 0;
			jc->btns.capture = (jc->buttons & (1 << 13)) ? 1 : 0;
		}

		// right:
		if (jc->left_right == 2)
		{

			// Ringcon stuff

			if (lightpress == true)
			{
				jc->buttons |= 1U << 4;
			}

			if (heavypress == true)
			{
				jc->buttons |= 1U << 7;
			}

			if (lightpull == true)
			{
				jc->buttons |= 1U << 5;
			}

			if (heavypull == true)
			{
				jc->buttons |= 1U << 6;
			}

			// Mouse buttons
			// printf("%i\n", Ringcon);
			if (settings.enableGyro)
			{
				if ((jc->buttons & (1 << 7) || Ringcon >= 0x0C) && !leftmousedown)
				{ // ZR controls left mouse button
					MC.LeftClickDown();
					leftmousedown = true;
				}
				if (!(jc->buttons & (1 << 7) || Ringcon >= 0x0C) && leftmousedown)
				{
					MC.LeftClickUp();
					leftmousedown = false;
				}
				if ((jc->buttons & (1 << 6) || lightpull || heavypull) && !rightmousedown)
				{ // R controls right mouse button
					MC.RightClickDown();
					rightmousedown = true;
				}
				if (!(jc->buttons & (1 << 6) || lightpull || heavypull) && rightmousedown)
				{
					MC.RightClickUp();
					rightmousedown = false;
				}
			}

			jc->btns.y = (jc->buttons & (1 << 0)) ? 1 : 0;
			jc->btns.x = (jc->buttons & (1 << 1)) ? 1 : 0;
			jc->btns.b = (jc->buttons & (1 << 2)) ? 1 : 0;
			jc->btns.a = (jc->buttons & (1 << 3)) ? 1 : 0;
			jc->btns.sr = (jc->buttons & (1 << 4)) ? 1 : 0;
			jc->btns.sl = (jc->buttons & (1 << 5)) ? 1 : 0;
			jc->btns.r = (jc->buttons & (1 << 6)) ? 1 : 0;
			jc->btns.zr = (jc->buttons & (1 << 7)) ? 1 : 0;
			jc->btns.plus = (jc->buttons & (1 << 9)) ? 1 : 0;
			jc->btns.stick_button = (jc->buttons & (1 << 10)) ? 1 : 0;
			jc->btns.home = (jc->buttons & (1 << 12)) ? 1 : 0;
		}

		// pro controller:
		if (jc->left_right == 3)
		{

			// left:
			jc->btns.down = (jc->buttons & (1 << 0)) ? 1 : 0;
			jc->btns.up = (jc->buttons & (1 << 1)) ? 1 : 0;
			jc->btns.right = (jc->buttons & (1 << 2)) ? 1 : 0;
			jc->btns.left = (jc->buttons & (1 << 3)) ? 1 : 0;
			jc->btns.sr = (jc->buttons & (1 << 4)) ? 1 : 0;
			jc->btns.sl = (jc->buttons & (1 << 5)) ? 1 : 0;
			jc->btns.l = (jc->buttons & (1 << 6)) ? 1 : 0;
			jc->btns.zl = (jc->buttons & (1 << 7)) ? 1 : 0;
			jc->btns.minus = (jc->buttons & (1 << 8)) ? 1 : 0;
			jc->btns.stick_button = (jc->buttons & (1 << 11)) ? 1 : 0;
			jc->btns.capture = (jc->buttons & (1 << 13)) ? 1 : 0;

			// right:
			jc->btns.y = (jc->buttons2 & (1 << 0)) ? 1 : 0;
			jc->btns.x = (jc->buttons2 & (1 << 1)) ? 1 : 0;
			jc->btns.b = (jc->buttons2 & (1 << 2)) ? 1 : 0;
			jc->btns.a = (jc->buttons2 & (1 << 3)) ? 1 : 0;
			jc->btns.sr = (jc->buttons2 & (1 << 4)) ? 1 : 0;
			jc->btns.sl = (jc->buttons2 & (1 << 5)) ? 1 : 0;
			jc->btns.r = (jc->buttons2 & (1 << 6)) ? 1 : 0;
			jc->btns.zr = (jc->buttons2 & (1 << 7)) ? 1 : 0;
			jc->btns.plus = (jc->buttons2 & (1 << 9)) ? 1 : 0;
			jc->btns.stick_button2 = (jc->buttons2 & (1 << 10)) ? 1 : 0;
			jc->btns.home = (jc->buttons2 & (1 << 12)) ? 1 : 0;
		}
		/*
		if (lightpress) {
			puts("light press");
		}
		else if (heavypress) {
			puts("heavy press");
		}
		else if (lightpull) {
			puts("light pull");
		}
		else if (heavypull) {
			puts("heavy pull");
		}
		*/
	}
}

void parseSettings2()
{

	// setupConsole("Debug");

	std::map<std::string, std::string> cfg = LoadConfig("config.txt");

	settings.combineJoyCons = (bool)stoi(cfg["combineJoyCons"]);
	settings.enableGyro = (bool)stoi(cfg["gyroControls"]);

	settings.gyroSensitivityX = stof(cfg["gyroSensitivityX"]);
	settings.gyroSensitivityY = stof(cfg["gyroSensitivityY"]);

	settings.squatSlowsMouse = (bool)stoi(cfg["squatSlowsMouse"]);
	settings.marioTheme = (bool)stoi(cfg["marioTheme"]);

	settings.reverseX = (bool)stoi(cfg["reverseX"]);
	settings.reverseY = (bool)stoi(cfg["reverseY"]);

	settings.preferLeftJoyCon = (bool)stoi(cfg["preferLeftJoyCon"]);
	settings.quickToggleGyro = (bool)stoi(cfg["quickToggleGyro"]);
	settings.invertQuickToggle = (bool)stoi(cfg["invertQuickToggle"]);

	// settings.dolphinPointerMode = (bool)stoi(cfg["dolphinPointerMode"]);

	settings.gyroscopeComboCode = stoi(cfg["gyroscopeComboCode"]);

	settings.Runpressesbutton = (bool)stoi(cfg["runPressesButton"]);
	settings.RingconFix = (bool)stoi(cfg["ringconfix"]);

	settings.rununlocksgyro = (bool)stoi(cfg["rununlocksgyro"]);

	settings.RingconFullRH = (bool)stoi(cfg["ringconfullrh"]);
	settings.host = cfg["host"];
	settings.RingconToAnalog = (bool)stoi(cfg["ringcontoanalog"]);

	settings.autoStart = (bool)stoi(cfg["autoStart"]);
}

void start();

void pollLoop()
{
	// poll joycons:
	for (int i = 0; i < joycons.size(); ++i)
	{

		Joycon *jc = &joycons[i];

		// choose a random joycon to reduce bias / figure out the problem w/input lag:
		// Joycon *jc = &joycons[rand_range(0, joycons.size()-1)];

		if (!jc->handle)
		{
			continue;
		}

		hid_set_nonblocking(jc->handle, 1);

		// get input:
		memset(buf, 0, 65);

		// get current time
		std::chrono::steady_clock::time_point tNow = std::chrono::steady_clock::now();

		auto timeSincePoll = std::chrono::duration_cast<std::chrono::microseconds>(tNow - tracker.tPolls[i]);

		// time spent sleeping (0):
		double timeSincePollMS = timeSincePoll.count() / 1000.0;

		if (timeSincePollMS > (1000.0 / settings.pollsPerSec))
		{
			jc->send_command(0x1E, buf, 0);
			tracker.tPolls[i] = std::chrono::steady_clock::now();
		}

		// hid_read(jc->handle, buf, 0x40);
		hid_read_timeout(jc->handle, buf, 0x40, 20);

		handle_input(jc, buf, 0x40);
	}

	accurateSleep(settings.timeToSleepMS);

	if (settings.restart)
	{
		settings.restart = false;
		start();
	}
}

void start()
{
	// set infinite reconnect attempts
	// myClient.set_reconnect_attempts(999999999999);

	int read;	 // number of bytes read
	int written; // number of bytes written
	const char *device_name;

	// Enumerate and print the HID devices on the system
	struct hid_device_info *devs, *cur_dev;

	res = hid_init();

	// hack:
	for (int i = 0; i < 100; ++i)
	{
		tracker.tPolls.push_back(std::chrono::steady_clock::now());
	}

init_start:

	devs = hid_enumerate(JOYCON_VENDOR, 0x0);
	cur_dev = devs;
	while (cur_dev)
	{

		// identify by vendor:
		if (cur_dev->vendor_id == JOYCON_VENDOR)
		{

			// bluetooth, left / right joycon:
			if (cur_dev->product_id == JOYCON_L_BT || cur_dev->product_id == JOYCON_R_BT)
			{
				Joycon jc = Joycon(cur_dev);
				joycons.push_back(jc);
			}

			// pro controller:
			if (cur_dev->product_id == PRO_CONTROLLER)
			{
				Joycon jc = Joycon(cur_dev);
				joycons.push_back(jc);
			}
		}

		cur_dev = cur_dev->next;
	}
	hid_free_enumeration(devs);

	// init joycons:
	if (settings.usingGrip)
	{
		for (int i = 0; i < joycons.size(); ++i)
		{
			joycons[i].init_usb();
		}
	}
	else
	{
		for (int i = 0; i < joycons.size(); ++i)
		{
			joycons[i].init_bt();
		}
	}

	if (settings.combineJoyCons)
	{
		int counter = 0;
		for (int i = 0; i < joycons.size(); ++i)
		{
			// joycons[i].VigemNumber = (counter / 2) + 1;
			joycons[i].deviceNumber = (counter % 2 ? 1 : 0);
			counter++;
		}
	}
	else
	{
		for (int i = 0; i < joycons.size(); ++i)
		{
			// joycons[i].VigemNumber = i + 1;
			joycons[i].deviceNumber = 0; // left
		}
	}

	// initial poll:
	pollLoop();

	// set lights:
	printf("setting LEDs...\n");
	for (int r = 0; r < 5; ++r)
	{
		for (int i = 0; i < joycons.size(); ++i)
		{
			Joycon *jc = &joycons[i];
			// Player LED Enable
			memset(buf, 0x00, 0x40);
			if (i == 0)
			{
				buf[0] = 0x0 | 0x0 | 0x0 | 0x1; // solid 1
			}
			if (i == 1)
			{
				if (settings.combineJoyCons)
				{
					buf[0] = 0x0 | 0x0 | 0x0 | 0x1; // solid 1
				}
				else if (!settings.combineJoyCons)
				{
					buf[0] = 0x0 | 0x0 | 0x2 | 0x0; // solid 2
				}
			}
			// buf[0] = 0x80 | 0x40 | 0x2 | 0x1; // Flash top two, solid bottom two
			// buf[0] = 0x8 | 0x4 | 0x2 | 0x1; // All solid
			// buf[0] = 0x80 | 0x40 | 0x20 | 0x10; // All flashing
			// buf[0] = 0x80 | 0x00 | 0x20 | 0x10; // All flashing except 3rd light (off)

			jc->send_subcommand(0x01, 0x30, buf, 1);
		}
	}

	// Set Ringcon IMU state
	for (int i = 0; i < joycons.size(); ++i)
	{
		Joycon *jc = &joycons[i];
		if (jc->left_right == 2 && jc->ringconattached)
		{
			ringconattached = true;
		}
	}

	// give a small rumble to all joycons:
	printf("vibrating JoyCon(s).\n");
	for (int k = 0; k < 1; ++k)
	{
		for (int i = 0; i < joycons.size(); ++i)
		{
			joycons[i].rumble(100, 1);
			Sleep(20);
			joycons[i].rumble(10, 3);
		}
	}

	// Mfosses clever Mario theme
	// Plays the Mario theme on the JoyCons:
	// I'm bad with music I just did this by
	// using a video of a piano version of the mario theme.
	// maybe eventually I'll be able to play something like sound files.

	// notes arbitrarily defined:
#define C3 110
#define D3 120
#define E3 130
#define F3 140
#define G3 150
#define G3A4 155
#define A4 160
#define A4B4 165
#define B4 170
#define C4 180
#define D4 190
#define D4E4 195
#define E4 200
#define F4 210
#define F4G4 215
#define G4 220
#define A5 230
#define B5 240
#define C5 250

	if (settings.marioTheme)
	{
		for (int i = 0; i < 1; ++i)
		{

			printf("Playing mario theme...\n");

			float spd = 1;
			float spd2 = 1;

			// goto N1;

			Joycon joycon = joycons[0];

			Sleep(1000);

			joycon.rumble(mk_odd(E4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E2
			Sleep(100 / spd2);
			joycon.rumble(mk_odd(E4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E2
			Sleep(100 / spd2);
			joycon.rumble(mk_odd(E4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E2
			Sleep(100 / spd2);
			joycon.rumble(mk_odd(C4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // C2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(E4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E2
			Sleep(100 / spd2);
			joycon.rumble(mk_odd(G4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G2
			Sleep(400 / spd2);

			joycon.rumble(mk_odd(A4), 1);
			Sleep(400 / spd);
			joycon.rumble(1, 3); // too low for joycon
			Sleep(50 / spd2);

			joycon.rumble(mk_odd(C4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // C2
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(G3), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G1
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(E3), 2);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E1
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(A4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // A2

			Sleep(100 / spd2);
			joycon.rumble(mk_odd(B4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // B2

			Sleep(50 / spd2);
			joycon.rumble(mk_odd(A4B4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // A2-B2?
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(A4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // A2
			Sleep(100 / spd2);
			joycon.rumble(mk_odd(G3), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G1

			Sleep(100 / spd2);
			joycon.rumble(mk_odd(E4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E2
			Sleep(100 / spd2);
			joycon.rumble(mk_odd(G4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G2
			Sleep(100 / spd2);
			joycon.rumble(mk_odd(A5), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // A3

			Sleep(200 / spd2);
			joycon.rumble(mk_odd(F4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // F2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(G4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G2

			Sleep(200 / spd2);
			joycon.rumble(mk_odd(E4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E2

			Sleep(50 / spd2);
			joycon.rumble(mk_odd(C4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // C2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(D4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // D2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(B4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // B2

			Sleep(200 / spd2);
			joycon.rumble(mk_odd(C4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // C2
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(G3), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G1
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(E3), 2);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E1

			Sleep(200 / spd2);
			joycon.rumble(mk_odd(A4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // A2
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(B4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // B2
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(A4B4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // A2-B2?
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(A4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // A2
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(G4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G2

			Sleep(100 / spd2);
			joycon.rumble(mk_odd(E4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E2
			Sleep(100 / spd2);
			joycon.rumble(mk_odd(G4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G2
			Sleep(100 / spd2);
			joycon.rumble(mk_odd(A5), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // A3
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(F4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // F2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(G4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G2
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(E4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E2

			Sleep(200 / spd2);
			joycon.rumble(mk_odd(C4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // C2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(D4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // D2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(B4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // B2

			// new:

			Sleep(500 / spd2);

			joycon.rumble(mk_odd(G4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(F4G4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // F2-G2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(F4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // F2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(D4E4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // D2-E2
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(E4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E2

			Sleep(200 / spd2);

			joycon.rumble(mk_odd(G3A4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G1-A2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(A4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // A2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(C4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // C2

			Sleep(200 / spd2);
			joycon.rumble(mk_odd(A4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // A2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(C4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // C2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(D4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // D2

			Sleep(300 / spd2);

			joycon.rumble(mk_odd(G4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(F4G4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // F2-G2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(F4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // F2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(D4E4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // D2-E2
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(E4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E2

			// three notes:
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(C5), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // C3
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(C3), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // C3
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(C3), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // C3

		N1:

			Sleep(500 / spd2);
			joycon.rumble(mk_odd(G4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(F4G4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // F2G2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(F4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // F2

			Sleep(50 / spd2);
			joycon.rumble(mk_odd(D4E4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // D2E2

			Sleep(200 / spd2);
			joycon.rumble(mk_odd(E4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E2

			Sleep(200 / spd2);
			joycon.rumble(mk_odd(G3A4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G1A2

			Sleep(50 / spd2);
			joycon.rumble(mk_odd(A4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // A2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(C4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // C2
			Sleep(100 / spd2);
			joycon.rumble(mk_odd(A4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // A2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(C4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // C2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(D4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // D2

			Sleep(300 / spd2);
			joycon.rumble(mk_odd(D4E4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // D2E2
			Sleep(300 / spd2);
			joycon.rumble(mk_odd(D4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // D2
			Sleep(300 / spd2);
			joycon.rumble(mk_odd(C4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // C2

			Sleep(800 / spd2);

			joycon.rumble(mk_odd(G4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(F4G4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // F2G2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(F4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // F2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(D4E4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // D2E2
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(E4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E2

			Sleep(200 / spd2);

			joycon.rumble(mk_odd(G3A4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G1A2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(A4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // A2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(C4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // C2

			Sleep(200 / spd2);

			joycon.rumble(mk_odd(A4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // A2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(C4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // C2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(D4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // D2

			Sleep(300 / spd2);

			joycon.rumble(mk_odd(G4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(F4G4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // F2G2
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(F4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // F2

			Sleep(50 / spd2);
			joycon.rumble(mk_odd(D4E4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // D2E2
			Sleep(100 / spd2);
			joycon.rumble(mk_odd(E4), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E2

			// 30 second mark

			// three notes:

			Sleep(300 / spd2);
			joycon.rumble(mk_odd(C5), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // C3
			Sleep(200 / spd2);
			joycon.rumble(mk_odd(C5), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // C3
			Sleep(50 / spd2);
			joycon.rumble(mk_odd(C5), 1);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // C3

			Sleep(1000);
		}
	}

#define MusicOffset 600

	// notes in hertz:
#define C3 131 + MusicOffset
#define D3 146 + MusicOffset
#define E3 165 + MusicOffset
#define F3 175 + MusicOffset
#define G3 196 + MusicOffset
#define G3A4 208 + MusicOffset
#define A4 440 + MusicOffset
#define A4B4 466 + MusicOffset
#define B4 494 + MusicOffset
#define C4 262 + MusicOffset
#define D4 294 + MusicOffset
#define D4E4 311 + MusicOffset
#define E4 329 + MusicOffset
#define F4 349 + MusicOffset
#define F4G4 215 + MusicOffset
#define G4 392 + MusicOffset
#define A5 880 + MusicOffset
#define B5 988 + MusicOffset
#define C5 523 + MusicOffset

#define hfa 0xb0   // 8a
#define lfa 0x006c // 8062

	if (false)
	{
		for (int i = 0; i < 1; ++i)
		{

			printf("Playing mario theme...\n");

			float spd = 1;
			float spd2 = 1;

			Joycon joycon = joycons[0];

			Sleep(1000);

			joycon.rumble3(E4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E2
			Sleep(100 / spd2);
			joycon.rumble3(E4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E2
			Sleep(100 / spd2);
			joycon.rumble3(E4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E2
			Sleep(100 / spd2);
			joycon.rumble3(C4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // C2
			Sleep(50 / spd2);
			joycon.rumble3(E4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E2
			Sleep(100 / spd2);
			joycon.rumble3(G4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G2
			Sleep(400 / spd2);

			joycon.rumble3(A4, hfa, lfa);
			Sleep(400 / spd);
			joycon.rumble(1, 3); // too low for joycon
			Sleep(50 / spd2);

			joycon.rumble3(C4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // C2
			Sleep(200 / spd2);
			joycon.rumble3(G3, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G1
			Sleep(200 / spd2);
			joycon.rumble3(E3, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E1
			Sleep(200 / spd2);
			joycon.rumble3(A4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // A2

			Sleep(100 / spd2);
			joycon.rumble3(B4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // B2

			Sleep(50 / spd2);
			joycon.rumble3(A4B4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // A2-B2?
			Sleep(50 / spd2);
			joycon.rumble3(A4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // A2
			Sleep(100 / spd2);
			joycon.rumble3(G3, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G1

			Sleep(100 / spd2);
			joycon.rumble3(E4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E2
			Sleep(100 / spd2);
			joycon.rumble3(G4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G2
			Sleep(100 / spd2);
			joycon.rumble3(A5, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // A3

			Sleep(200 / spd2);
			joycon.rumble3(F4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // F2
			Sleep(50 / spd2);
			joycon.rumble3(G4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G2

			Sleep(200 / spd2);
			joycon.rumble3(E4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E2

			Sleep(50 / spd2);
			joycon.rumble3(C4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // C2
			Sleep(50 / spd2);
			joycon.rumble3(D4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // D2
			Sleep(50 / spd2);
			joycon.rumble3(B4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // B2

			Sleep(200 / spd2);
			joycon.rumble3(C4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // C2
			Sleep(200 / spd2);
			joycon.rumble3(G3, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G1
			Sleep(200 / spd2);
			joycon.rumble3(E3, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // E1

			Sleep(200 / spd2);
			joycon.rumble3(A4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // A2
			Sleep(200 / spd2);
			joycon.rumble3(B4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // B2
			Sleep(200 / spd2);
			joycon.rumble3(A4B4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // A2-B2?
			Sleep(50 / spd2);
			joycon.rumble3(A4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // A2
			Sleep(200 / spd2);
			joycon.rumble3(G4, hfa, lfa);
			Sleep(200 / spd);
			joycon.rumble(1, 3); // G2

			Sleep(1000);
		}
	}

	printf("Done.\n");

	if (settings.RingconFullRH)
	{
		printf("\n CAUTION: Do not use heavy press when the Ringcon is sideways. It may damage the flex sensor.");
	};
}

void actuallyQuit()
{
	/*
	for (int i = 0; i < joycons.size(); ++i) {
		buf[0] = 0x0; // disconnect
		joycons[i].send_subcommand(0x01, 0x06, buf, 1);
	}*/

	if (settings.usingGrip)
	{
		for (int i = 0; i < joycons.size(); ++i)
		{
			joycons[i].deinit_usb();
		}
	}
	// Finalize the hidapi library
	res = hid_exit();
	//puts("eeeeexit ~~~");
}

// ----------------------------------------------------------------------------
// constants
// ----------------------------------------------------------------------------

// int main(int argc, char *argv[]) {
// int wWinMain(HINSTANCE hInstance, HINSTANCE prevInstance, LPWSTR cmdLine, int cmdShow) {

#include <process.h> // For CRT atexit functions
#include <shellapi.h> // For ShellExecute


extern "C"
{
	__declspec(dllexport) void ringcon_init()
	{
		parseSettings2();
		start();
		atexit(actuallyQuit); // prevent broke next use
	}
	
	__declspec(dllexport) void poll_ringcon()
	{
		pollLoop();
		static int ringcon_index = -1;
		static int leg_joycon_index = -1;
		if (ringcon_index == -1 || leg_joycon_index == -1)
			for (auto i = 0; i < joycons.size(); ++i)
			{
				if (joycons[i].name == "Joy-Con (R)")
				{
					ringcon_index = i;
					std::cout << "[ringcon index:" << i << "]\n";
				}
				else if (joycons[i].name == "Joy-Con (L)")
				{
					leg_joycon_index = i;
					std::cout << "[leg_joycon index:" << i << "]\n";
				}
			}
		
		
		
		// serialize to bytes data to pass back
		auto &ringcon = joycons[ringcon_index];
		auto &leg_joycon = joycons[leg_joycon_index];
		
		if (leg_joycon.squatting)
		{
			printf("s: %f\n", leg_joycon.accel.z);
		}
		else
		{
			printf("ns: %f\n", leg_joycon.accel.z);
		}
	}
	/*
	__declspec(dllexport) void start_daemon()
	{
		parseSettings2();
		start();

		while (true)
		{
			pollLoop();
			// puts("tick");
		}
	}
	*/
}