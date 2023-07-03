/*
Library for Ecoride C2 display

Copyright (c) 2023 Hinko Kocevar
Based on display_kingmeter.c by Michael Fabry (Michael@Fabry.de)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Includes

#include "config.h"
#include "main.h"
#include "display_ecoride.h"
#include "stm32f1xx_hal.h"
#include "print.h"

#if (DISPLAY_TYPE & DISPLAY_TYPE_ECORIDE)

// Definitions
// #define RXSTATE_STARTCODE 0
// #define RXSTATE_SENDTXMSG 1
// #define RXSTATE_MSGBODY 2
// #define RXSTATE_DONE 3

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

// Hashtable used for handshaking in C2 protocol
#if (DISPLAY_TYPE == DISPLAY_TYPE_ECORIDE_C2)
const uint8_t C2_HANDSHAKE[64] =
	{
		137, 159, 134, 249,  88,  11, 250,  61,  33, 150,   3, 193, 118, 141, 209,  94,
		226,  68, 146, 158, 145, 127, 216, 	62, 116, 230, 101, 211, 251,  54, 229, 247,
		20,  222,  59,  63,  35, 252, 142, 238,  23, 197,  84,  77, 147, 173, 210,  57,
		142, 223, 157,  97,  36, 160, 229, 237,  75,  80,  37, 113, 154,  88,  23, 120
	};
#endif

// Local function prototypes
#if (DISPLAY_TYPE == DISPLAY_TYPE_ECORIDE_C2)
static void C2_Service(ECORIDE_t *ctx);
#endif

uint8_t lowByte(uint16_t word);
uint8_t highByte(uint16_t word);

uint8_t pas_tolerance = 0;
uint8_t wheel_magnets = 1;
uint8_t vcutoff = 30;
// uint16_t wheel_circumference = 2200;
uint8_t spd_max1 = 25;
// uint8_t ui8_RxLength = 1;

/* Public functions (Prototypes declared by display_ecoride.h) */

/****************************************************************************************************
 * KingMeter_Init() - Initializes the display object
 *
 ****************************************************************************************************/

void Ecoride_Init(ECORIDE_t *ctx)

{
	uint8_t i;

	// ctx->RxState = RXSTATE_STARTCODE;
	// ctx->DirectSetpoint = 0xFF; // hier aktuellen Timerwert als Startzeitpunkt

	for (i = 0; i < ECORIDE_MAX_RXBUFF; i++)
	{
		ctx->RxBuff[i] = 0x00;
	}
	ctx->RxCnt = 0;

	// Settings received from display:
	// ctx->Settings.PAS_RUN_Direction = KM_PASDIR_FORWARD;
	// ctx->Settings.PAS_SCN_Tolerance = (uint8_t)pas_tolerance;
	ctx->Settings.PAS_N_Ratio = 255;
	// ctx->Settings.HND_HL_ThrParam = KM_HND_HL_NO;
	// ctx->Settings.HND_HF_ThrParam = KM_HND_HF_NO;
	// ctx->Settings.SYS_SSP_SlowStart = 1;
	// ctx->Settings.SPS_SpdMagnets = (uint8_t)wheel_magnets;
	// ctx->Settings.VOL_1_UnderVolt_x10 = (uint16_t)(vcutoff * 10);
	// ctx->Settings.WheelSize_mm = (uint16_t)(WHEEL_CIRCUMFERENCE * 1000);
	ctx->Settings.WheelSize_mm = (uint16_t)(KM_WHEELSIZE_28 * 1000);

	// Parameters received from display in operation mode:

#if (DISPLAY_TYPE == DISPLAY_TYPE_ECORIDE_C2)
	ctx->Rx.AssistLevel = 128; // MK5S Level 0...255
#endif

	ctx->Rx.Headlight = KM_HEADLIGHT_OFF;
	ctx->Rx.Battery = KM_BATTERY_NORMAL;
	// ctx->Rx.PushAssist = KM_PUSHASSIST_OFF;
	// ctx->Rx.PowerAssist = KM_POWERASSIST_ON;
	// ctx->Rx.Throttle = KM_THROTTLE_ON;
	// ctx->Rx.CruiseControl = KM_CRUISE_OFF;
	// ctx->Rx.OverSpeed = KM_OVERSPEED_NO;
	// ctx->Rx.SPEEDMAX_Limit = (uint16_t)(spd_max1 * 10);
	// ctx->Rx.CUR_Limit_mA = 150;

	// Parameters to be send to display in operation mode:
	ctx->Tx.Battery = KM_BATTERY_NORMAL;
	ctx->Tx.Wheeltime_ms = KM_MAX_WHEELTIME;
	ctx->Tx.Error = KM_ERROR_NONE;
	ctx->Tx.Current_x10 = 0;

#if (DISPLAY_TYPE == DISPLAY_TYPE_ECORIDE_C2)
	// Start UART with DMA
	if (HAL_UART_Receive_DMA(&huart1, (uint8_t *)ctx->RxBuff, ECORIDE_MAX_RXBUFF) != HAL_OK)
	{
		Error_Handler();
	}
#endif
	// HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&buffer, KM_MAX_RXBUFF);
}

/****************************************************************************************************
 * KingMeter_Service() - Communicates data from and to the display
 *
 ***************************************************************************************************/
void Ecoride_Service(ECORIDE_t *ctx)
{
#if (DISPLAY_TYPE == DISPLAY_TYPE_ECORIDE_C2)
	C2_Service(ctx);
#endif
}

/* Local functions */
#if (DISPLAY_TYPE == DISPLAY_TYPE_ECORIDE_C2)
/****************************************************************************************************
 * KM_901U_Service() - Communication protocol of 901U firmware
 *
 ***************************************************************************************************/
static void C2_Service(ECORIDE_t *ctx)
{
	static uint8_t TxBuffer[ECORIDE_MAX_TXBUFF];
	static uint8_t m;
	static uint8_t last_pointer_position;
	static uint8_t recent_pointer_position;

	uint16_t CheckSum;

	static uint8_t TxCnt;
	static uint8_t Rx_message_length;
	static uint8_t KM_Message[32];

	recent_pointer_position = ECORIDE_MAX_RXBUFF - DMA1_Channel5->CNDTR;

	if (recent_pointer_position > last_pointer_position)
	{
		Rx_message_length = recent_pointer_position - last_pointer_position;
		// printf_("groesser %d, %d, %d \n ",recent_pointer_position,last_pointer_position, Rx_message_length);
		// HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		memcpy(KM_Message, ctx->RxBuff + last_pointer_position, Rx_message_length);
		// HAL_UART_Transmit(&huart3, (uint8_t *)&KM_Message, Rx_message_length,50);
	}
	else
	{
		Rx_message_length = recent_pointer_position + ECORIDE_MAX_RXBUFF - last_pointer_position;
		memcpy(KM_Message, ctx->RxBuff + last_pointer_position, ECORIDE_MAX_RXBUFF - last_pointer_position);
		memcpy(KM_Message + ECORIDE_MAX_RXBUFF - last_pointer_position, ctx->RxBuff, recent_pointer_position);
		//  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	}
	last_pointer_position = recent_pointer_position;
	// HAL_UART_Transmit(&huart3, (uint8_t *)&KM_Message, Rx_message_length,50);

	CheckSum = 0x0000;
	for (m = 1; m < (4 + KM_Message[3]); m++)
	{
		CheckSum = CheckSum + KM_Message[m]; // Calculate CheckSum
	}
	CheckSum -= (KM_Message[m] + ((KM_Message[m + 1]) << 8));

	switch (KM_Message[2])
	{
	case 0x52:		   // Operation mode
		if (!CheckSum) // low-byte and high-byte
		{
			// HAL_UART_Transmit(&huart3, (uint8_t *)&KM_Message, Rx_message_length,50);
			// Decode Rx message

			ctx->Rx.AssistLevel = KM_Message[4];					// 0..255
			// ctx->Rx.Headlight = (KM_Message[5] & 0xC0) >> 6;		// KM_HEADLIGHT_OFF / KM_HEADLIGHT_ON / KM_HEADLIGHT_LOW / KM_HEADLIGHT_HIGH
			ctx->Rx.Headlight = (KM_Message[5] & 0x80) >> 7;		// KM_HEADLIGHT_OFF / KM_HEADLIGHT_ON
			ctx->Rx.Battery = (KM_Message[5] & 0x20) >> 5;		// KM_BATTERY_NORMAL / KM_BATTERY_LOW
			ctx->Rx.AssistMode = (KM_Message[5] & 0x0F);	// assist mode (0, 2, 4, 6, 8)
			// ctx->Rx.PushAssist = (KM_Message[5] & 0x10) >> 4;	// KM_PUSHASSIST_OFF / KM_PUSHASSIST_ON
			// ctx->Rx.PowerAssist = (KM_Message[5] & 0x08) >> 3;	// KM_POWERASSIST_OFF / KM_POWERASSIST_ON
			// ctx->Rx.Throttle = (KM_Message[5] & 0x04) >> 2;		// KM_THROTTLE_OFF / KM_THROTTLE_ON
			// ctx->Rx.CruiseControl = (KM_Message[5] & 0x02) >> 1; // KM_CRUISE_OFF / KM_CRUISE_ON
			// ctx->Rx.OverSpeed = (KM_Message[5] & 0x01);			// KM_OVERSPEED_NO / KM_OVERSPEED_YES
		}
		else
		{ // printf_("Checksum fail! \n ");
		}

		ecoride_update();

		// C2 0 kmh
		// 3A 1A 52 05 00 00 0D AC 00 2A 01 0D 0A
		// C2 max assist
		// 3a 1a 52 05 00 29 0d ac 00 53 01 0d 0a

		// Prepare Tx message
		TxBuffer[0] = 0X3A; // StartCode
		TxBuffer[1] = 0x1A; // SrcAdd:  Controller
		TxBuffer[2] = 0x52; // CmdCode
		TxBuffer[3] = 0x05; // DataSize

		if (ctx->Tx.Battery == KM_BATTERY_LOW)
		{
			// TxBuffer[4] = 0x40; // State data (only UnderVoltage bit has influence on display)
			// C2: when low voltage 0x80; tested value on the bike!
			TxBuffer[4] = 0x80; // State data (only UnderVoltage bit has influence on display)
		}
		else
		{							  // Byte7 ist autocruise Symbol, Byte6 ist Battery low Symbol
			// C2: 0x00 when not low voltage
			TxBuffer[4] = 0b00000000; // State data (only UnderVoltage bit has influence on display)
		}

		// C2: changes from 0x00 .. 0x29 ; tested on the bike
		TxBuffer[5] = (uint8_t)((ctx->Tx.Current_x10 * 3) / 10); // Current low Strom in 1/3 Ampere, nur ein Byte
		// C2: changes from 0D AC (0 kmh) .. 01 38 (25 kmh); tested on the bike
		TxBuffer[6] = highByte(ctx->Tx.Wheeltime_ms);			// WheelSpeed high Hinweis
		TxBuffer[7] = lowByte(ctx->Tx.Wheeltime_ms);				// WheelSpeed low
		// C2: usually 0x00; chnaged to 0x26 on low battery!; tested on the bike
		TxBuffer[8] = ctx->Tx.Error;								// Error

		TxCnt = 9;
		break;

	case 0x53: // Settings mode

		// Decode Rx message
		if (!CheckSum) // low-byte and high-byte
		{
			ecoride_update();
			// ctx->Settings.PAS_RUN_Direction = (KM_Message[4] & 0x80) >> 7; // KM_PASDIR_FORWARD / KM_PASDIR_BACKWARD
			// ctx->Settings.PAS_SCN_Tolerance = KM_Message[5];				  // 2..9
			ctx->Settings.PAS_N_Ratio = KM_Message[6];					  // 0..255
			// ctx->Settings.HND_HL_ThrParam = (KM_Message[7] & 0x80) >> 7;	  // KM_HND_HL_NO / KM_HND_HL_YES
			// ctx->Settings.HND_HF_ThrParam = (KM_Message[7] & 0x40) >> 6;	  // KM_HND_HF_NO / KM_HND_HF_YES
			// ctx->Settings.SYS_SSP_SlowStart = KM_Message[8];				  // 1..9
			// ctx->Settings.SPS_SpdMagnets = KM_Message[9];				  // 1..4
			// ctx->Settings.VOL_1_UnderVolt_x10 = (((uint16_t)KM_Message[11]) << 8) | KM_Message[11];
			// ctx->Settings.WheelSize_mm = (((uint16_t)KM_Message[12]) << 8) | KM_Message[13];
			// ctx->Rx.SPEEDMAX_Limit = KM_Message[11];
			// ctx->Rx.CUR_Limit_mA = (KM_Message[8] & 0x3F) * 500;

			// if (ctx->Rx.CUR_Limit_mA == 21500)
			// 	autodetect();
			// if (ctx->Rx.CUR_Limit_mA == 20500)
			// 	get_internal_temp_offset();
		}

		// C2
		// 3A 1A 53 05 00 00 0D 5E 00 DD 00 0D 0A

		// Prepare Tx message with handshake code
		TxBuffer[0] = 0X3A; // StartCode
		TxBuffer[1] = 0x1A; // SrcAdd:  Controller
		TxBuffer[2] = 0x53; // CmdCode
		TxBuffer[3] = 0x05; // Number of Databytes
		TxBuffer[4] = 0x00;
		TxBuffer[5] = 0x00;
		TxBuffer[6] = 0x0D;
		TxBuffer[7] = C2_HANDSHAKE[KM_Message[9]];
		TxBuffer[8] = 0x00;
		TxBuffer[9] = 0x0C;
		TxBuffer[10] = 0x01;
		TxBuffer[11] = 0x0D;
		TxBuffer[12] = 0x0A;

		// 3A 1A 53 05 00 00 0D 91 00 10 01 0D 0A
		// 3A 1A 53 05 80 00 0D 91 26 B6 01 0D 0A
		// 3A 1A 53 05 00 00 0D 91 00 10 01 0D 0A
		// 3A 1A 53 05 00 00 0D 8D 00 0C 01 0D 0A
		// DataSize
		// TxBuffer[5] = KM_901U_HANDSHAKE[ctx->RxBuff[14]];      // Handshake answer
		TxCnt = 9;
		break;

	// C2: Not seen?!
	// case 0x54:		   // Operation mode
	// 	if (!CheckSum) // low-byte and high-byte
	// 	{
	// 		ctx->DirectSetpoint = KM_Message[4];
	// 	}
	// 	break;

	default:
		TxCnt = 0;
	}

	// Send prepared message
	if (TxCnt && !CheckSum)
	{
		CheckSum = 0x0000;

		for (m = 1; m < TxCnt; m++)
		{

			CheckSum = CheckSum + TxBuffer[m]; // Calculate CheckSum
		}
		TxBuffer[TxCnt + 0] = lowByte(CheckSum);  // Low Byte of checksum
		TxBuffer[TxCnt + 1] = highByte(CheckSum); // High Byte of checksum
		TxBuffer[TxCnt + 2] = 0x0D;
		TxBuffer[TxCnt + 3] = 0x0A;

		HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&TxBuffer, TxCnt + 4);
		// HAL_UART_Transmit(&huart3, (uint8_t *)&TxBuffer, TxCnt+4,50);
		// printf_("%d, %d \n ",TxCnt+4,KM_Message[2]);
	}
}

#endif

uint8_t lowByte(uint16_t word)
{
	return word & 0xFF;
}

uint8_t highByte(uint16_t word)
{
	return word >> 8;
}

#endif // (DISPLAY_TYPE & DISPLAY_TYPE_ECORIDE)
