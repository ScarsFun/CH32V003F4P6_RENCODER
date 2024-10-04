/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : ScarsFun
 * Version            : V1.0.0
 * Date               : 2024/10/02
 * Description        : Simple Timers test.
 *********************************************************************************/

#include "debug.h"
#include "ULWOS2.h"
#include "oled_display.h"
#include "fonts.h"

/* Global defines */

typedef enum {
    DISPLAY_STATUS_SLEEP,
    DISPLAY_STATUS_UPDATE
} display_status_t;

typedef enum {
    BUTTON_STATUS_HZ,
    BUTTON_STATUS_DUTY
} button_status_t;


/* Global Variables */

volatile int8_t encoder_status = DISPLAY_STATUS_SLEEP;
volatile int16_t encoder_steps_hz = 1, encoder_steps_duty = 50;
int8_t displayMode = BUTTON_STATUS_HZ;
uint16_t ARRvalue = 10000;
uint16_t CH1CVRvalue = 5000;
uint16_t Hz, Duty;
char str[20];


// --------------- peripheral configuration ----------------------
// Timer 2 in Encoder mode
void TIM2_init(void)
{
    GPIO_InitTypeDef gpio_cfg;
    TIM_TimeBaseInitTypeDef TIMER_InitStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    // PD3 (TIM2_CH2) (encoder pin A), PD4 (TIM2_CH1) (encoder pin B)
    gpio_cfg.GPIO_Mode = GPIO_Mode_IPU;
    gpio_cfg.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 ;
    gpio_cfg.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOD, &gpio_cfg);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseStructInit(&TIMER_InitStructure);
    // setting TIM_Period = 1 Timer will reloaded every step and Interrupt triggered
    TIMER_InitStructure.TIM_Prescaler = 0;
    TIMER_InitStructure.TIM_Period = 1; // ARR
    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up | TIM_CounterMode_Down;
    TIM_TimeBaseInit(TIM2, &TIMER_InitStructure);

    //debounce filter
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0F;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICFilter = 0x0F;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        if (displayMode == BUTTON_STATUS_HZ)
        {
            //Il registro TIM2_CTLR1 bit TIM_DIR sarà la direzione di rotazione dell'encoder
            encoder_steps_hz += (TIM2->CTLR1 & TIM_DIR) ? -1 : 1;
            encoder_steps_hz = (encoder_steps_hz <1) ? 1 : encoder_steps_hz ;
            encoder_steps_hz = (encoder_steps_hz >111) ? 100 : encoder_steps_hz ;
        }
        else if (displayMode == BUTTON_STATUS_DUTY)
        {
            encoder_steps_duty += (TIM2->CTLR1 & TIM_DIR) ? -1 : 1;
            encoder_steps_duty = (encoder_steps_duty <1) ? 1 : encoder_steps_duty ;
            encoder_steps_duty = (encoder_steps_duty >100) ? 100 : encoder_steps_duty ;
        }
        encoder_status = DISPLAY_STATUS_UPDATE;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
/*********************************************************************
 * @fn      TIM1_OutCompare_Init
 *
 * @brief   Initializes TIM1 output compare.
 *
 * @param   arr - the period value.
 *          psc - the prescaler value.
 *          ccp - the pulse value.
 *
 * @return  none
 */
void TIM1_PWMOut_Init(u16 arr, u16 psc, u16 ccp)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    TIM_OCInitTypeDef TIM_OCInitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD | RCC_APB2Periph_TIM1, ENABLE );

    // LED Output PD2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOD, &GPIO_InitStructure );

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);


    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;


    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init( TIM1, &TIM_OCInitStructure );

    TIM_CtrlPWMOutputs(TIM1, ENABLE );
    TIM_OC1PreloadConfig( TIM1, TIM_OCPreload_Disable );
    TIM_ARRPreloadConfig( TIM1, ENABLE );
    TIM_Cmd( TIM1, ENABLE );
}


// init I2C interface
void I2C_User_Init(uint32_t bound, uint16_t address) {
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };
    I2C_InitTypeDef I2C_InitTSturcture = { 0 };

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE);

    //SCL - PC2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure);

    //SDA - PC1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure);

    I2C_InitTSturcture.I2C_ClockSpeed = bound;
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitTSturcture.I2C_OwnAddress1 = address;
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init( I2C1, &I2C_InitTSturcture);

    I2C_Cmd( I2C1, ENABLE);
    I2C_AcknowledgeConfig( I2C1, ENABLE);

}

/*********************************************************************
 * @fn      EXTI0_INT_INIT
 *
 * @brief   Initializes EXTI0 collection. PD0 Connected to Encoder Switch Button
 *
 * @return  none
 */
void EXTI0_INT_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* GPIOD ----> EXTI_Line0 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource0);
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      EXTI0_IRQHandler
 *
 * @brief   This function handles EXTI0 Handler.
 *
 * @return  none
 */
void EXTI7_0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        if (displayMode == BUTTON_STATUS_HZ)
            displayMode = BUTTON_STATUS_DUTY;
        else
            displayMode = BUTTON_STATUS_HZ;
        
        encoder_status = DISPLAY_STATUS_UPDATE;
        EXTI_ClearITPendingBit(EXTI_Line0);     /* Clear Flag */
    }
}

// Update Display task using ULWOS2
void updateDisplay(void)
{
    ULWOS2_THREAD_START();
    while (1)
    {
        ULWOS2_THREAD_SLEEP_MS(50);
        if (encoder_status == DISPLAY_STATUS_UPDATE)
        {
            if (displayMode == BUTTON_STATUS_HZ)
            {
                ssd1306_Fill(Black);
                ssd1306_SetCursor(0, 0);
                sprintf(str, "%d", encoder_steps_hz);
                ssd1306_WriteString(str, FONT_UGE, White);
                sprintf(str, "HZ");
                ssd1306_SetCursor(ssd1306_GetCurrentX()+5, 10);
                ssd1306_WriteString(str, FONT_BIG, White);
                ssd1306_UpdateScreen();

                TIM_SetAutoreload(TIM1, ARRvalue / encoder_steps_hz); //TIM1->ATRLR
                TIM_SetCompare1(TIM1, ARRvalue / encoder_steps_hz / 100 * encoder_steps_duty ); //TIM1->CH1CVR

            }
            else if (displayMode == BUTTON_STATUS_DUTY)
            {
                ssd1306_Fill(Black);
                ssd1306_SetCursor(0, 0);
                sprintf(str, "%d", encoder_steps_duty);
                ssd1306_WriteString(str, FONT_UGE, White);
                sprintf(str, "DUTY");
                ssd1306_SetCursor(ssd1306_GetCurrentX()+5, 10);
                ssd1306_WriteString(str, FONT_BIG, White);
                ssd1306_UpdateScreen();

                TIM_SetCompare1(TIM1, ARRvalue / encoder_steps_hz / 100 * encoder_steps_duty ); //TIM1->CH1CVR
            }
            encoder_status = DISPLAY_STATUS_SLEEP;
        }
    }
}
/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    SysTick_INIT();

    TIM2_init();
    TIM1_PWMOut_Init( ARRvalue, 4800 - 1, CH1CVRvalue);
    I2C_User_Init(100000, 0x02);
    EXTI0_INT_INIT();
    ssd1306_Init();

#if (SDI_PRINT == SDI_PR_OPEN)
    SDI_Printf_Enable();
#endif

    ULWOS2_INIT();
    ULWOS2_THREAD_CREATE(updateDisplay, 10);
    ULWOS2_START_SCHEDULER();
}
