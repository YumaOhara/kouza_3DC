
#include <main.h>
#include "stm32f1xx_hal_uart.h"		// SBDBTとUART通信をするためのライブラリ

// UART用構造体変数
extern UART_HandleTypeDef huart2;
extern uint8_t RxBuffer[8];

extern uint8_t B;
extern uint8_t A;
extern uint8_t X;
extern uint8_t Y;
extern uint8_t RIGHT;
extern uint8_t DOWN;
extern uint8_t LEFT;
extern uint8_t UP;
extern uint8_t R1;
extern uint8_t R2;
extern uint8_t L1;
extern uint8_t L2;
extern uint8_t START;
extern uint8_t BACK;
extern uint8_t RightAxisX;
extern uint8_t RightAxisY;
extern uint8_t LeftAxisX;
extern uint8_t LeftAxisY;

// タイマ用構造体変数
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

//ロボット半径[m]
double R = 0.18;

double Vx = 0;
double Vy = 0;
double Omega = 0;


double v1 = 0;
double v2 = 0;
double v3 = 0;

int32_t V1 = 0;
int32_t V2 = 0;
int32_t V3 = 0;

//スティックの値をDutyに変換
void update_wheel_vel(){
	Vx = (LeftAxisX - 64)*1000/64;
	Vy = (64 - LeftAxisY)*1000/64;
	Omega = L1 - R1;

	v1 = Vx + R*Omega;
	v2 = -0.5*Vx + 1.732/2*Vy + R*Omega;
	v3 = -0.5*Vx - 1.732/2*Vy + R*Omega;

	V1 = (int32_t)v1;
	V2 = (int32_t)v2;
	V3 = (int32_t)v3;

}

void DC1(){
	if(v1 > 0){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, V1);
	    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
	}

	else if(v1 < 0){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -V1);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
	}

	else{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	}
}

void DC2(){
	if(v2 > 0){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, V2);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
	}

	else if(v2 < 0){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -V2);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
	}

	else{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

	}
}

void DC3(){
	if(v3 > 0){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, V3);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);
	}

	else if(v3 < 0){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, V3);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);
	}

	else{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	}
}

// メイン関数
void main_cpp(void)
{
	// UART開始
	HAL_UART_Receive_IT(&huart2, RxBuffer, 8);

	// LED緑を点灯
	HAL_GPIO_WritePin(GPIOC, GREEN_LED_Pin, GPIO_PIN_SET);

	// 無効化後、黄色LEDを消灯、赤色LEDを点灯
	HAL_GPIO_WritePin(GPIOC, YELLOW_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, RED_LED_Pin, GPIO_PIN_SET);

	// メインループ
	while(1)
	{
		// もし、BACKボタンが押されている(BACK == 1)なら、
		if(BACK == 1)
		{

			// 無効化後、黄色LEDを消灯、赤色LEDを点灯
			HAL_GPIO_WritePin(GPIOC, YELLOW_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, RED_LED_Pin, GPIO_PIN_SET);

			// LED緑を点灯
			HAL_GPIO_WritePin(GPIOC, GREEN_LED_Pin, GPIO_PIN_SET);

			//PWM無効化
			//tim1
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
			//tim2
			HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);


		}

		// もし、STARTボタンが押されている(START == 1)なら、
		if(START == 1)
		{
			// 有効化後、黄色LEDを点灯、赤色LEDを消灯
			HAL_GPIO_WritePin(GPIOC, YELLOW_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, RED_LED_Pin, GPIO_PIN_RESET);

			// LED緑を点灯
			HAL_GPIO_WritePin(GPIOC, GREEN_LED_Pin, GPIO_PIN_SET);

			//PWM有効化
			//tim1
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
			//tim2
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);

			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,0);
		}

		update_wheel_vel();


		//DC更新&回転
		DC1();
		DC2();
		DC3();
	}
}
