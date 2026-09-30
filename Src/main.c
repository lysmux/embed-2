/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "kb.h"
#include "sdk_uart.h"
#include "pca9538.h"
#include "oled.h"
#include "fonts.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	OP_ADD = 0,
	OP_SUB,
	OP_MUL,
	OP_DIV,
	OP_COUNT
} Calc_Op;

typedef enum {
	STATE_FIRST,	/* ввод первого операнда                   */
	STATE_SECOND,	/* операция выбрана, ввод второго операнда */
	STATE_RESULT	/* показан результат или ошибка            */
} Calc_State;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEY_NONE		0xFF
#define KEY_OP			10		/* нижний ряд, левая кнопка: выбор операции */
#define KEY_EQ			11		/* нижний ряд, правая кнопка: "=" / сброс   */

#define MAX_DIGITS		6		/* максимум цифр в операнде                  */
#define FRAC_SCALE		1000	/* 3 знака после запятой */

#define SCAN_PERIOD_MS	10		/* период опроса клавиатуры */
#define DEBOUNCE_SCANS	2		/* сколько одинаковых опросов подряд нужно */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*
 * Раскладка клавиатуры (ROW1 - верхний ряд, ROW4 - нижний):
 *   ROW1:  1   2   3
 *   ROW2:  4   5   6
 *   ROW3:  7   8   9
 *   ROW4:  OP  0   =
 * Check_Row() возвращает 0x04 - левый, 0x02 - средний, 0x01 - правый столбец.
 */
static const uint8_t KB_Rows[4] = {ROW1, ROW2, ROW3, ROW4};
static const uint8_t KB_Map[4][3] = {
	{ 1,      2, 3      },
	{ 4,      5, 6      },
	{ 7,      8, 9      },
	{ KEY_OP, 0, KEY_EQ }
};

static const char Op_Symbols[OP_COUNT] = {'+', '-', '*', '/'};

static Calc_State calc_state;
static Calc_Op    calc_op;
static uint32_t   calc_a, calc_b;
static uint8_t    calc_a_len, calc_b_len;
static int64_t    calc_result;	/* результат */
static uint8_t    calc_error;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void oled_Reset( void );
static uint8_t KB_Scan( void );
static uint8_t KB_GetKey( void );
static void Calc_Reset( void );
static void Calc_AppendDigit( uint32_t *value, uint8_t *len, uint8_t digit );
static void Calc_HandleKey( uint8_t key );
static void Calc_Compute( void );
static void Calc_Draw( void );
static char *U64_ToStr( uint64_t v, char *buf );
static void Fixed_ToStr( int64_t v, char *buf );
/* USER CODE END PFP */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  oled_Init();
  UART_Transmit( (uint8_t*)"Calculator start\r\n" );
  Calc_Reset();
  Calc_Draw();



  /* Infinite loop */
  while (1)
  {
	  uint8_t key = KB_GetKey();
	  if ( key != KEY_NONE ) {
		  Calc_HandleKey( key );
		  Calc_Draw();
	  }
	  HAL_Delay( SCAN_PERIOD_MS );
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}


/* Один проход по всем рядам. Возвращает код кнопки 0..11 или KEY_NONE. */
static uint8_t KB_Scan( void ) {
	for ( uint8_t r = 0; r < 4; r++ ) {
		uint8_t col = Check_Row( KB_Rows[r] );
		if ( col == 0x04 ) return KB_Map[r][0];
		if ( col == 0x02 ) return KB_Map[r][1];
		if ( col == 0x01 ) return KB_Map[r][2];
	}
	return KEY_NONE;
}

/*
 * Антидребезг
 */
static uint8_t KB_GetKey( void ) {
	static uint8_t last_raw = KEY_NONE;
	static uint8_t stable_cnt = 0;
	static uint8_t reported = KEY_NONE;

	uint8_t raw = KB_Scan();
	if ( raw == last_raw ) {
		if ( stable_cnt < DEBOUNCE_SCANS ) stable_cnt++;
	} else {
		last_raw = raw;
		stable_cnt = 1;
	}

	if ( stable_cnt < DEBOUNCE_SCANS || raw == reported ) {
		return KEY_NONE;
	}
	reported = raw;
	return raw;	/* при отпускании вернётся KEY_NONE */
}


static void Calc_Reset( void ) {
	calc_state = STATE_FIRST;
	calc_op = OP_ADD;
	calc_a = calc_b = 0;
	calc_a_len = calc_b_len = 0;
	calc_result = 0;
	calc_error = 0;
}

static void Calc_AppendDigit( uint32_t *value, uint8_t *len, uint8_t digit ) {
	if ( *len == 1 && *value == 0 ) {
		*value = digit;			/* ведущий ноль заменяется */
	} else if ( *len < MAX_DIGITS ) {
		*value = *value * 10 + digit;
		(*len)++;
	}
}

static void Calc_HandleKey( uint8_t key ) {
	char msg[16] = "Key: ";

	if ( key <= 9 ) {
		msg[5] = '0' + key;
		msg[6] = 0;
	} else {
		strcat( msg, key == KEY_OP ? "OP" : "=" );
	}
	strcat( msg, "\r\n" );
	UART_Transmit( (uint8_t*)msg );

	if ( key <= 9 ) {
		if ( calc_state == STATE_RESULT ) {
			Calc_Reset();		/* результат: цифра начинает новое выражение */
		}
		if ( calc_state == STATE_FIRST ) {
			Calc_AppendDigit( &calc_a, &calc_a_len, key );
		} else {
			Calc_AppendDigit( &calc_b, &calc_b_len, key );
		}
	} else if ( key == KEY_OP ) {
		if ( calc_state == STATE_FIRST && calc_a_len > 0 ) {
			calc_state = STATE_SECOND;
			calc_op = OP_ADD;
		} else if ( calc_state == STATE_SECOND ) {
			calc_op = (Calc_Op)( ( calc_op + 1 ) % OP_COUNT );
		}
		/* без первого операнда и после результата кнопка игнорируется */
	} else if ( key == KEY_EQ ) {
		if ( calc_state == STATE_SECOND && calc_b_len > 0 ) {
			Calc_Compute();
		} else {
			Calc_Reset();		/* неполное выражение или "=" после результата - сброс */
			UART_Transmit( (uint8_t*)"Reset\r\n" );
		}
	}
}

static void Calc_Compute( void ) {
	int64_t a = calc_a, b = calc_b;
	char line[48], num[24];

	calc_state = STATE_RESULT;
	switch ( calc_op ) {
		case OP_ADD: calc_result = ( a + b ) * FRAC_SCALE; break;
		case OP_SUB: calc_result = ( a - b ) * FRAC_SCALE; break;
		case OP_MUL: calc_result = ( a * b ) * FRAC_SCALE; break;
		case OP_DIV:
			if ( b == 0 ) {
				calc_error = 1;
			} else {
				/* округление до ближайшего */
				calc_result = ( a * FRAC_SCALE * 2 / b + 1 ) / 2;
			}
			break;
		default: break;
	}

	U64_ToStr( calc_a, line );
	num[0] = Op_Symbols[calc_op];
	num[1] = 0;
	strcat( line, num );
	strcat( line, U64_ToStr( calc_b, num ) );
	strcat( line, "=" );
	if ( calc_error ) {
		strcat( line, "Error: div by 0" );
	} else {
		Fixed_ToStr( calc_result, num );
		strcat( line, num );
	}
	strcat( line, "\r\n" );
	UART_Transmit( (uint8_t*)line );
}

/*
 * Экран:
 *   y=0   заголовок и текущая операция
 *   y=16  выражение
 *   y=32  результат
 *   y=54  подсказка по кнопкам
 */
static void Calc_Draw( void ) {
	char line[24], num[24];

	oled_Fill( Black );

	oled_SetCursor( 0, 0 );
	strcpy( line, "CALC   op: " );
	num[0] = ( calc_state == STATE_FIRST ) ? '-' : Op_Symbols[calc_op];
	num[1] = 0;
	strcat( line, num );
	oled_WriteString( line, Font_7x10, White );

	line[0] = 0;
	if ( calc_a_len > 0 ) {
		U64_ToStr( calc_a, line );
	}
	if ( calc_state != STATE_FIRST ) {
		num[0] = Op_Symbols[calc_op];
		num[1] = 0;
		strcat( line, num );
		if ( calc_b_len > 0 ) {
			strcat( line, U64_ToStr( calc_b, num ) );
		}
	}
	if ( calc_state != STATE_RESULT ) {
		strcat( line, "_" );	/* курсор ввода */
	}
	oled_SetCursor( 0, 16 );
	oled_WriteString( line, Font_7x10, White );

	if ( calc_state == STATE_RESULT ) {
		if ( calc_error ) {
			oled_SetCursor( 0, 32 );
			oled_WriteString( "Div by 0!", Font_11x18, White );
		} else {
			line[0] = '=';
			Fixed_ToStr( calc_result, line + 1 );
			if ( strlen( line ) * Font_11x18.FontWidth < OLED_WIDTH ) {
				oled_SetCursor( 0, 32 );
				oled_WriteString( line, Font_11x18, White );
			} else {
				oled_SetCursor( 0, 36 );
				oled_WriteString( line, Font_7x10, White );
			}
		}
	}

	oled_SetCursor( 0, 53 );
	oled_WriteString( "L:op  R:=/C", Font_7x10, White );

	oled_UpdateScreen();
}

static char *U64_ToStr( uint64_t v, char *buf ) {
	char tmp[21];
	int i = 0, j = 0;

	do {
		tmp[i++] = '0' + ( v % 10 );
		v /= 10;
	} while ( v );
	while ( i ) {
		buf[j++] = tmp[--i];
	}
	buf[j] = 0;
	return buf;
}

/* Число с фиксированной точкой */
static void Fixed_ToStr( int64_t v, char *buf ) {
	uint64_t abs_v;
	uint32_t frac;
	char *p = buf;

	if ( v < 0 ) {
		*p++ = '-';
		abs_v = (uint64_t)( -v );
	} else {
		abs_v = (uint64_t)v;
	}
	frac = (uint32_t)( abs_v % FRAC_SCALE );
	U64_ToStr( abs_v / FRAC_SCALE, p );
	p += strlen( p );

	if ( frac ) {
		*p++ = '.';
		for ( uint32_t d = FRAC_SCALE / 10; d && frac; d /= 10 ) {
			*p++ = '0' + frac / d;
			frac %= d;
		}
	}
	*p = 0;
}

void oled_Reset( void ) {
	oled_Fill(Black);
	oled_SetCursor(0, 0);
	oled_UpdateScreen();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
