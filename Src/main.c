/******************************************************************************
 * @file           : main.c
 * @brief          : CG2028 Assignment - ElderCare Wearable Safety Companion
 * @author         : Hou Linxin
 * (c) CG2028 Teaching Team
 ******************************************************************************/

/*--------------------------- Includes ---------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <math.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "wifi.h"
#include <stdlib.h>	// for rand(). Can be removed if valid sensor data is sent instead

#define MAX_LENGTH 400	// adjust it depending on the max size of the packet you expect to send or receive
#define WIFI_READ_TIMEOUT 10000
#define WIFI_WRITE_TIMEOUT 10000
//#define USING_IOT_SERVER // This line should be commented out if using Packet Sender (not IoT server connection)

/*--------------------------- Configuration ----------------------------------*/
#define EWMA_ALPHA_ACCEL_PERCENT   40 //old config: 25
#define EWMA_ALPHA_GYRO_PERCENT    40 //old config: 25
#define NORMAL_LED_DELAY_MS       1000
#define FALL_LED_DELAY_MS          150

/*Thresholds*/
#define ACCEL_DEV_THRESHOLD_MPS2    3.0f  /* |SVM - 1 g| above this = abnormal accel */
#define GYRO_THRESHOLD_DPS         40.0f  /* gyro SVM above this = abnormal rotation */

/*Timing*/
#define SAMPLE_PERIOD_MS             50   /* 20 Hz sampling, independent of the LED */
#define PRINT_EVERY_N_SAMPLES        10
#define FALL_CONFIRM_SAMPLES         2
#define WARMUP_SAMPLES               10
#define OLED_I2C_TIMING       0x00702681U
#define OLED_UI_RECTANGLE_WIDTH      7
#define BUTTON_DEBOUNCE_MS           60


typedef enum { STATE_WARMUP, STATE_NORMAL, STATE_NEAR_FALL, STATE_FALL } State;
static const char *const state_name[] = { "WARMUP", "NORMAL", "NEAR-FALL", "FALL" };


const char* WiFi_SSID = "cxk";				// Replacce mySSID with WiFi SSID for your router / Hotspot
const char* WiFi_password = "ruozhicaiyong12";	// Replace myPassword with WiFi password for your router / Hotspot
const WIFI_Ecn_t WiFi_security = WIFI_ECN_WPA2_PSK;	// WiFi security your router / Hotspot. No need to change it unless you use something other than WPA2 PSK
const uint16_t SOURCE_PORT = 1234;	// source port, which can be almost any 16 bit number

uint8_t ipaddr[4] = {18, 138, 224, 16}; // IP address of your laptop wireless lan adapter, which is the one you successfully used to test Packet Sender above.
									// If using IoT platform, this will be overwritten by DNS lookup, so the values of x and y doesn't matter
											//(it should still be filled in with numbers 0-255 to avoid compilation errors)

#ifdef USING_IOT_SERVER
	const char* SERVER_NAME = "demo.thingsboard.io"; 	// domain name of the IoT server used
	const uint16_t DEST_PORT = 80;			// 'server' port number. Change according to application layer protocol. 80 is the destination port for HTTP protocol.
#else
	const uint16_t DEST_PORT = 2028;		// 'server' port number - this is the port Packet Sender listens to (as you set in Packer Sender)
												// and should be allowed by the OS firewall
#endif
SPI_HandleTypeDef hspi3;

static volatile uint8_t button_debounce_pending = 0;
static volatile uint8_t button_press_pending = 0;
static volatile uint32_t button_last_edge_tick = 0;
static void UART1_Init(void);
static void BUTTON_Init(void);
static void OLED_I2C1_Init(void);
static void OLED_draw(char* line1, char* line2);
static void UART_Send(const char *text);
static float magnitude(const float v[3]);
static State classify(float accel_dev, float gyro_svm, int *consecutive_fall_samples);
static void Buzzer_Init(void);
static void Buzzer_Pattern(State state, int fall_latched, int fall_new);

/* Buzzer pattern times in ms. */
enum { HEARTBEAT_MS = 10000, LONG_BEEP_MS = 1000, SHORT_BEEP_MS = 200, FALL_PULSE_MS = 100 };

extern int ewma_filter(int new_data, int old_output, int alpha_percent);
//int ewma_filter_C(int new_data, int old_output, int alpha_percent);

UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c1;

int main(void)
{
    HAL_Init();
    UART1_Init();
    BUTTON_Init();
    BSP_LED_Init(LED2);
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();
    BSP_LED_Off(LED2);
    Buzzer_Init();
    OLED_I2C1_Init();
    ssd1306_Init();
    ssd1306_SetCursor(0, 0);
    uint8_t req[MAX_LENGTH];	// request packet
	uint8_t resp[MAX_LENGTH];	// response pa  cket
	uint16_t Datalen;
	WIFI_Status_t WiFi_Stat;
	UART_Send("Program started\r\n");
	OLED_draw("Program", "started");

	WiFi_Stat = WIFI_Init();
	if (WiFi_Stat == WIFI_STATUS_OK) {
		UART_Send("WiFi INIT finished, try to connect WiFi\r\n");
	} else {
		while (WiFi_Stat != WIFI_STATUS_OK) {
			UART_Send("WiFi INIT failed, try to do it again\r\n");
			WiFi_Stat = WIFI_Init();
		}
	}
	WiFi_Stat = WIFI_Connect(WiFi_SSID, WiFi_password, WiFi_security);
	while (WiFi_Stat!=WIFI_STATUS_OK) {
		UART_Send("Wifi not Connected\r\n");
		WiFi_Stat = WIFI_Connect(WiFi_SSID, WiFi_password, WiFi_security);
	};
	if (WiFi_Stat == WIFI_STATUS_OK) {
		UART_Send("Wifi connected successfully, trying to connect to server\r\n");
	}
	// WiFi_Stat = WIFI_Ping(ipaddr, 3, 200);					// Optional ping 3 times in 200 ms intervals
	WiFi_Stat = WIFI_OpenClientConnection(1, WIFI_TCP_PROTOCOL, "conn", ipaddr, DEST_PORT, SOURCE_PORT); // Make a TCP connection.
																	  // "conn" is just a name and serves no functional purpose

	if (WiFi_Stat != WIFI_STATUS_OK)
	{
	    UART_Send("WIFI_OpenClientConnection returned ERROR\r\n");
	    OLED_draw("Server", "connect failed");

	    while (1)
	    {
	    }
	}
	// halt computations if a connection could not be established with the server
	OLED_draw("Connected to", "wifi and server");
	HAL_Delay(1000);
	OLED_draw("NORMAL", NULL);
    /* Previous EWMA outputs. The first test/application sample starts from 0. */
    int accel_ewma_asm[3] = {0, 0, 0};
    int gyro_ewma_asm[3]  = {0, 0, 0};

    /* Reference C states are kept separately for assembly verification. */
    int accel_ewma_c[3] = {0, 0, 0};
    int gyro_ewma_c[3]  = {0, 0, 0};

    unsigned long sample_number = 0;

    /* Fall-detection state (persist across loop iterations). */
    int consecutive_fall_samples = 0;   /* consecutive samples with accel AND gyro abnormal */
    int fall_latched = 0;               /* alarm flag: stays 1 after a fall (drives LED only, not the printed state) */

    uint32_t next_sample_tick = HAL_GetTick();  /* when the next sample is due */
    uint32_t last_led_tick    = HAL_GetTick();  /* when the LED last toggled */

    State worst_state = STATE_WARMUP;   /* worst state since the last report */
    float peak_accel_dev = 0.0f;
    float peak_gyro_svm  = 0.0f;


    while (1)
    {

        int16_t accel_raw_i16[3] = {0, 0, 0};
        float gyro_raw_float[3] = {0.0f, 0.0f, 0.0f};
        int gyro_raw_int[3] = {0, 0, 0};


        BSP_ACCELERO_AccGetXYZ(accel_raw_i16);
        BSP_GYRO_GetXYZ(gyro_raw_float);

        /* The supplied BSP reports gyroscope readings as floating-point raw
         * values. Convert them to signed integers before passing them to the
         * integer assembly routine. */
        for (int axis = 0; axis < 3; axis++)
        {
            gyro_raw_int[axis] = (int)gyro_raw_float[axis];

            accel_ewma_asm[axis] = ewma_filter(
                (int)accel_raw_i16[axis],
                accel_ewma_asm[axis],
                EWMA_ALPHA_ACCEL_PERCENT);

            gyro_ewma_asm[axis] = ewma_filter(
                gyro_raw_int[axis],
                gyro_ewma_asm[axis],
                EWMA_ALPHA_GYRO_PERCENT);

            accel_ewma_c[axis] = ewma_filter_C(
                (int)accel_raw_i16[axis],
                accel_ewma_c[axis],
                EWMA_ALPHA_ACCEL_PERCENT);

            gyro_ewma_c[axis] = ewma_filter_C(
                gyro_raw_int[axis],
                gyro_ewma_c[axis],
                EWMA_ALPHA_GYRO_PERCENT);
        }

        /* Accelerometer filtered readings are in meters per second squared. */
        float accel_mps2[3] = {
            accel_ewma_asm[0] * (9.80665f / 1000.0f),
            accel_ewma_asm[1] * (9.80665f / 1000.0f),
            accel_ewma_asm[2] * (9.80665f / 1000.0f)
        };

        /* Gyroscope filtered readings are in degrees per second. */
        float gyro_dps[3] = {
            gyro_ewma_asm[0] / 1000.0f,
            gyro_ewma_asm[1] / 1000.0f,
            gyro_ewma_asm[2] / 1000.0f
        };

        char buffer[320];
        int print_now = ((sample_number % PRINT_EVERY_N_SAMPLES) == 0);

        if (print_now)
        {
            snprintf(buffer, sizeof(buffer),
                     "Sample %lu\r\n"
                     "Accel EWMA ASM [m/s^2]: X=%8.3f Y=%8.3f Z=%8.3f\r\n"
                     "Gyro  EWMA ASM [dps]  : X=%8.3f Y=%8.3f Z=%8.3f\r\n",
                     sample_number,
                     accel_mps2[0], accel_mps2[1], accel_mps2[2],
                     gyro_dps[0], gyro_dps[1], gyro_dps[2]);
            UART_Send(buffer);
        }

        /* Optional debugging check. This confirms that the assembly routine
         * matches the reference C routine for the current samples. */
        if ((accel_ewma_asm[0] != accel_ewma_c[0]) ||
            (accel_ewma_asm[1] != accel_ewma_c[1]) ||
            (accel_ewma_asm[2] != accel_ewma_c[2]) ||
            (gyro_ewma_asm[0] != gyro_ewma_c[0]) ||
            (gyro_ewma_asm[1] != gyro_ewma_c[1]) ||
            (gyro_ewma_asm[2] != gyro_ewma_c[2]))
        {
            UART_Send("WARNING: Assembly and C EWMA outputs do not match.\r\n");
        }

        /**************** Elderly wearable state logic starts here************************
         * Compulsory requirements:
         * 1. Use filtered accelerometer AND gyroscope readings.
         * 2. Distinguish normal activity, near-fall movements, and a real fall.
         * 3. Use a slow LED blink for normal operation and a fast blink after
         *    a fall is detected.
         *********************************************************************/


	/* Distance from 1 g: ~0 at rest, ~9.8 in free-fall, positive on impact. */
	float accel_dev = fabsf(magnitude(accel_mps2) - 9.81f);
	float gyro_svm  = magnitude(gyro_dps)/10.0f;

	State state = (sample_number < WARMUP_SAMPLES)
						? STATE_WARMUP
						: classify(accel_dev, gyro_svm, &consecutive_fall_samples);

	if (state > worst_state)        { worst_state    = state; }
	if (accel_dev > peak_accel_dev) { peak_accel_dev = accel_dev; }
	if (gyro_svm  > peak_gyro_svm)  { peak_gyro_svm  = gyro_svm; }


	int fall_new = (state == STATE_FALL) && !fall_latched;
	if (fall_new)
    {
	    fall_latched = 1;
	    UART_Send("ALERT: fall detected!\r\n");
	    sprintf((char*)req, "FallDetected\r");

	    WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
	    OLED_draw("Fall Detected", "Contact Relatives");

    }
	if (button_debounce_pending != 0)
	{
		uint32_t now = HAL_GetTick();

		if ((uint32_t)(now - button_last_edge_tick) >= BUTTON_DEBOUNCE_MS)
		{
			button_debounce_pending = 0;

			// if the button is still pressed
			if (HAL_GPIO_ReadPin(
					BUTTON_EXTI13_GPIO_Port,
					BUTTON_EXTI13_Pin) == GPIO_PIN_RESET)
			{
				button_press_pending = 1;
			}
		}
	}
	if (button_press_pending != 0U)
	{
	    button_press_pending = 0U;

	    if (fall_latched != 0)
	    {
	        fall_latched = 0;
	        consecutive_fall_samples = 0;

	        OLED_draw("Assistance", "is here");
	        HAL_Delay(1000U);
	        OLED_draw("NORMAL", NULL);

	        next_sample_tick = HAL_GetTick();
	    }
	}
	Buzzer_Pattern(state, fall_latched, fall_new);
	if (print_now)
	{
	    snprintf(buffer, sizeof(buffer), "State: %s%s (accel_dev=%.2f, gyro_svm=%.1f)\r\n",
	             state_name[worst_state], fall_latched ? " [ALARM LATCHED]" : "",
	             peak_accel_dev, peak_gyro_svm);
	    UART_Send(buffer);
	    worst_state    = STATE_WARMUP;   /* start a fresh 1 s window */
		peak_accel_dev = 0.0f;
		peak_gyro_svm  = 0.0f;
	}

	   /* The LED blinks on its own timer, so it does not set the sample rate. */
		  uint32_t now = HAL_GetTick();
		  if ((now - last_led_tick) >= (fall_latched ? FALL_LED_DELAY_MS : NORMAL_LED_DELAY_MS))
		  {
			  BSP_LED_Toggle(LED2);
			  last_led_tick = now;
		  }

		  sample_number++;

		  /* Fixed-period pacing: wait only for what is left of this slot, so a
		   * slow UART print does not stretch every later sample. */
		  next_sample_tick += SAMPLE_PERIOD_MS;
		  int32_t wait_ms = (int32_t)(next_sample_tick - HAL_GetTick());
		  if (wait_ms > 0)
		  {
			  HAL_Delay((uint32_t)wait_ms);
		  }
  }
}
	/* NORMAL    : neither accel_dev nor gyro is above its threshold.
	 * NEAR-FALL : one of them is above its threshold (a jolt or a fast rotation).
	 * FALL      : both are above their thresholds for FALL_CONFIRM_SAMPLES in a row. */
static State classify(float accel_dev, float gyro_svm, int *consecutive_fall_samples)
{
	int accel_abnormal = (accel_dev > ACCEL_DEV_THRESHOLD_MPS2);
	int gyro_abnormal  = (gyro_svm  > GYRO_THRESHOLD_DPS);

	if (accel_abnormal && gyro_abnormal)
	{
		(*consecutive_fall_samples)++;
		return (*consecutive_fall_samples >= FALL_CONFIRM_SAMPLES) ? STATE_FALL
															   : STATE_NEAR_FALL;
	}

	*consecutive_fall_samples = 0;
	return (accel_abnormal || gyro_abnormal) ? STATE_NEAR_FALL : STATE_NORMAL;
}

/* ---- Buzzer (active type): PB0 = Arduino D3, plain GPIO, HIGH = sound ---- */
static uint32_t beep_start_tick;   /* when the last beep started                */
static uint32_t beep_length_ms;    /* length of the beep playing, 0 = silent    */
static int      beep_pulsed;       /* 1 = pulsing on/off, 0 = one steady sound  */
static uint32_t near_beep_tick;    /* when the last near-fall beep started      */

static void Buzzer_Set(int on)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Buzzer_Beep(uint32_t length_ms, int pulsed)
{
	beep_start_tick = HAL_GetTick();
	beep_length_ms  = length_ms;
	beep_pulsed     = pulsed;
}

static void Buzzer_Init(void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin   = GPIO_PIN_0;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	Buzzer_Set(0);

	beep_start_tick = HAL_GetTick();                       /* first heartbeat after 10 s */
	near_beep_tick  = HAL_GetTick() - 2U* LONG_BEEP_MS;   /* near-fall beep allowed at once after 2s, U is unsigned*/
}

/* Call once per sample.
 * NORMAL    : short beep every 10 s.
 * NEAR-FALL : 2 s steady beep (at most one every 4 s while it lasts).
 * FALL      : 2 s of rapid beeping when confirmed, then repeated every 10 s
 *             until reset. An active buzzer has one fixed pitch, so "urgent"
 *             is rapid pulsing instead of a higher pitch. */
static void Buzzer_Pattern(State state, int fall_latched, int fall_new)
{
	uint32_t since = HAL_GetTick() - beep_start_tick;

	if ((beep_length_ms != 0) && (since >= beep_length_ms))      /* beep finished */
	{
		beep_length_ms = 0;
	}

	if (fall_new)                                                /* interrupts anything */
	{
		Buzzer_Beep(LONG_BEEP_MS, 1);
	}
	else if (beep_length_ms == 0)                                /* only start a beep when silent */
	{
		if (fall_latched)
		{
			if (since >= HEARTBEAT_MS)
			{
				Buzzer_Beep(LONG_BEEP_MS, 1);
			}
		}
		else if (state == STATE_NEAR_FALL)
		{
			if ((HAL_GetTick() - near_beep_tick) >= (2U * LONG_BEEP_MS))
			{
				near_beep_tick = HAL_GetTick();
				Buzzer_Beep(LONG_BEEP_MS, 0);
			}
		}
		else if (state == STATE_NORMAL)
		{
			if (since >= HEARTBEAT_MS)
			{
				Buzzer_Beep(SHORT_BEEP_MS, 0);
			}
		}
	}

	/* Drive the pin: steady during a beep, or on/off every FALL_PULSE_MS if pulsed. */
	int on = 0;
	if (beep_length_ms != 0)
	{
		on = beep_pulsed ? ((((HAL_GetTick() - beep_start_tick) / FALL_PULSE_MS) % 2U) == 0U) : 1;
	}
	Buzzer_Set(on);
}

/* Vector magnitude: orientation-independent, unlike per-axis values. */
static float magnitude(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}



int ewma_filter_C(int new_data, int old_output, int alpha_percent)
{
    /* Reference implementation for verification only. The assembly routine
     * must be used in the actual sensor-processing and detection pipeline. */
    int numerator = alpha_percent * new_data
                  + (100 - alpha_percent) * old_output;
    return numerator / 100;
}

static void UART_Send(const char *text)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)text, strlen(text), HAL_MAX_DELAY);
}
static void OLED_I2C1_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = OLED_I2C_TIMING;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        while (1) { }
    }

    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        while (1) { }
    }

    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        while (1) { }
    }
}
static void UART1_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        while (1) { }
    }
}
static void BUTTON_Init(void)
{
	// configure push button for EXIT and GPIO
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        BUTTON_EXTI13_GPIO_Port,
        &GPIO_InitStruct
    );

    __HAL_GPIO_EXTI_CLEAR_IT(BUTTON_EXTI13_Pin);

    HAL_NVIC_SetPriority( BUTTON_EXTI13_EXTI_IRQn, 15U, 0U );

    HAL_NVIC_EnableIRQ(BUTTON_EXTI13_EXTI_IRQn);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_1)
    {
        SPI_WIFI_ISR();
    }

    if (GPIO_Pin == BUTTON_EXTI13_Pin)
    {
        button_last_edge_tick = HAL_GetTick();
        button_debounce_pending = 1;
    }
}
static void OLED_WriteCenteredSingleLine(char *text)
{
    uint16_t text_width = (uint16_t)strlen(text) * Font_7x10.width;

    uint8_t x = 0U;
    uint8_t y = 0U;

    if (text_width < SSD1306_WIDTH)
    {
        x = (uint8_t)((SSD1306_WIDTH - text_width) / 2U);
    }

    y = (uint8_t)((SSD1306_HEIGHT - Font_7x10.height) / 2U);

    ssd1306_SetCursor(x, y);
    ssd1306_WriteString(text, Font_7x10, White);
}
static void OLED_WriteCenteredTwoLines(char *line1, char *line2)
{
    uint16_t line1_width = (uint16_t)strlen(line1) * Font_7x10.width;

    uint16_t line2_width = (uint16_t)strlen(line2) * Font_7x10.width;

    uint16_t total_height = (2U * Font_7x10.height) + 2U;
    uint8_t x1 = 0U;
    uint8_t x2 = 0U;
    uint8_t y1 = 0U;
    uint8_t y2 = 0U;

    if (line1_width < SSD1306_WIDTH)
    {
        x1 = (uint8_t)(
            (SSD1306_WIDTH - line1_width) / 2U
        );
    }

    if (line2_width < SSD1306_WIDTH)
    {
        x2 = (uint8_t)((SSD1306_WIDTH - line2_width) / 2U);
    }

    y1 = (uint8_t)((SSD1306_HEIGHT - total_height) / 2U);

    y2 = (uint8_t)(y1 + Font_7x10.height + 2U);

    ssd1306_SetCursor(x1, y1);
    ssd1306_WriteString(line1, Font_7x10, White);

    ssd1306_SetCursor(x2, y2);
    ssd1306_WriteString(line2, Font_7x10, White);
}
static void OLED_draw(char *line1, char *line2)
{
    ssd1306_Fill(Black);

    ssd1306_SetCursor(0, 0);
    ssd1306_DrawRectangle(0, 0, SSD1306_WIDTH - 1, SSD1306_HEIGHT - 1, White);
    ssd1306_DrawRectangle(0 + OLED_UI_RECTANGLE_WIDTH, 0 + OLED_UI_RECTANGLE_WIDTH,
    		SSD1306_WIDTH - 1 - OLED_UI_RECTANGLE_WIDTH, SSD1306_HEIGHT - 1 - OLED_UI_RECTANGLE_WIDTH,
			White
    );
    if (line2 == NULL) {
    	OLED_WriteCenteredSingleLine(line1);
    } else {
    	OLED_WriteCenteredTwoLines(line1, line2);
    }

    ssd1306_UpdateScreen();
}
/* Do not modify these lines. They suppress UART-related warnings. */
int _write(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    return len;
}
int _read(int file, char *ptr, int len) { (void)file; (void)ptr; (void)len; return 0; }
int _fstat(int file, struct stat *st) { (void)file; (void)st; return 0; }
int _lseek(int file, int ptr, int dir) { (void)file; (void)ptr; (void)dir; return 0; }
int _isatty(int file) { (void)file; return 1; }
int _close(int file) { (void)file; return -1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { (void)pid; (void)sig; return -1; }
