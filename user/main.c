/**
 * @file    bldc_driver.c
 * @brief   无刷电机驱动 (3极对, 12V, 带电流采样，含零电流校准)
 *          使用 DMA 读取三相电流，PWM 定时器同步触发 ADC
 *          代码 C89/90 标准
 */

#include "at32f415_board.h"
#include "at32f415_clock.h"
#include "at32f415_crm.h"
#include "at32f415_gpio.h"
#include "at32f415_tmr.h"
#include "at32f415_misc.h"
#include "at32f415_adc.h"
#include "at32f415_dma.h"

/* ======================== 宏定义 ======================== */

#define MOTOR_POLE_PAIRS            3
#define HALL_EDGES_PER_REV          (MOTOR_POLE_PAIRS * 6)
#define TWO_PI_E6                   6283185.307f
#define TMR_PRESCALER_1US           119u        /* 120 MHz / 120 = 1 MHz */
#define TMR2_PERIOD_MAX             0xFFFFFFFFu
#define PWM_FREQ_HZ                 20000u      //20k
#define PWM_PERIOD                  (1000000u / PWM_FREQ_HZ - 1u)
#define CURRENT_SENSE_R             0.0003f     /* 0.3mΩ */
#define V_REF                       3.33f

/* GPIO 引脚 */
#define HALL_U_PORT                 GPIOB
#define HALL_U_PIN                  GPIO_PINS_10
#define HALL_V_PORT                 GPIOA
#define HALL_V_PIN                  GPIO_PINS_15
#define HALL_W_PORT                 GPIOB
#define HALL_W_PIN                  GPIO_PINS_3

#define HS_U_PORT                   GPIOA
#define HS_U_PIN                    GPIO_PINS_10     /* TMR1_CH3 */
#define HS_V_PORT                   GPIOA
#define HS_V_PIN                    GPIO_PINS_9      /* TMR1_CH2 */
#define HS_W_PORT                   GPIOA
#define HS_W_PIN                    GPIO_PINS_8      /* TMR1_CH1 */

#define LS_U_PORT                   GPIOB
#define LS_U_PIN                    GPIO_PINS_15
#define LS_V_PORT                   GPIOB
#define LS_V_PIN                    GPIO_PINS_14
#define LS_W_PORT                   GPIOB
#define LS_W_PIN                    GPIO_PINS_13

/* ADC 通道宏 */
#define ADC_CH_U                    ADC_CHANNEL_7
#define ADC_CH_V                    ADC_CHANNEL_8
#define ADC_CH_W                    ADC_CHANNEL_9

#define CURRENT_SENSE_GAIN          12.0f   /* 运放放大倍数，电流分辨率0.223A*/

/* ======================== 数据类型 ======================== */

typedef enum {
    PHASE_U = 0,
    PHASE_V = 1,
    PHASE_W = 2
} phase_t;

typedef struct {
    phase_t high_phase;
    phase_t low_phase;
} commutation_t;

/* ======================== 全局变量 ======================== */

volatile float g_motor_speed_rad_s = 0.0f;
volatile uint32_t g_hall_last_cap     = 0u;
volatile uint32_t g_hall_interval     = 0u;
volatile uint8_t  g_hall_state_prev   = 0xFFu;

uint8_t  current_high = 0xFF;
uint8_t  current_low  = 0xFF;
uint8_t  last_valid_hall = 0xFF;
uint8_t  g_commutated_hall = 0xFF;

uint32_t cap_now;
uint32_t interval;
uint8_t  hall_u, hall_v, hall_w, hall_state;

/* 三相电流全局变量 */
volatile float g_phase_current_u;
volatile float g_phase_current_v;
volatile float g_phase_current_w;

volatile float g_bus_current;   /* 母线电流（A） */

/* 零电流偏移校准值（ADC原始值） */
float g_adc_offset_u = 0.0f;
float g_adc_offset_v = 0.0f;
float g_adc_offset_w = 0.0f;

uint16_t adc_val_u, adc_val_v, adc_val_w;
float v_u, v_v, v_w;

float pwm_p = 0.1f;

/* DMA 缓冲区：存放ADC 转换结果 */
uint16_t adc_dma_buffer[4];



volatile uint16_t adc_pa2_value = 0;
static float f_pa2_value = 0.0f;    // 原始电压
static float f_pa2_percent = 0.0f;  // 百分比 0~100%



// /* 正转换相表 (霍尔码: U<<2|V<<1|W) */
// static const commutation_t comm_table_forward[8] = {
//     {PHASE_U, PHASE_V},   /* 0 无效 */
//     {PHASE_W, PHASE_V},   /* 1 */
//     {PHASE_V, PHASE_U},   /* 2 */
//     {PHASE_W, PHASE_U},   /* 3 */
//     {PHASE_U, PHASE_W},   /* 4 */
//     {PHASE_U, PHASE_V},   /* 5 */
//     {PHASE_V, PHASE_W},   /* 6 */
//     {PHASE_U, PHASE_V}    /* 7 无效 */
// };

// 反转
static const commutation_t comm_table_forward[8] = {
    {PHASE_V, PHASE_U},   /* 0 无效 */
    {PHASE_V, PHASE_W},   /* 1 : 正转 W+ V- -> 反转 V+ W- */
    {PHASE_U, PHASE_V},   /* 2 : 正转 V+ U- -> 反转 U+ V- */
    {PHASE_U, PHASE_W},   /* 3 : 正转 W+ U- -> 反转 U+ W- */
    {PHASE_W, PHASE_U},   /* 4 : 正转 U+ W- -> 反转 W+ U- */
    {PHASE_V, PHASE_U},   /* 5 : 正转 U+ V- -> 反转 V+ U- */
    {PHASE_W, PHASE_V},   /* 6 : 正转 V+ W- -> 反转 W+ V- */
    {PHASE_V, PHASE_U}    /* 7 无效 */
};

// V-W+ 1 (001)
// U-V+ 2 (010)
// U-W+ 3 (011)
// U+W- 4 (100)
// U+V- 5 (101)
// V+W- 6 (110)

// 正转 1 → 5 → 4 → 6 → 2 → 3 → 1

// V-W+ 1 (001)
// U+V- 5 (101)
// U+W- 4 (100)
// V+W- 6 (110)
// U-V+ 2 (010)
// U-W+ 3 (011)
// V-W+ 1 (001)

// 反转 1 → 3 → 2 → 6 → 4 → 5 → 1

/* ======================== 函数声明 ======================== */

static void hall_speed_init(void);
static void motor_pwm_init(void);
static void adc_init(void);
static void current_offset_calibration(void);
static void current_sample(void);
static void set_high_phase(phase_t phase, float duty);
static void set_low_phase(phase_t phase, uint8_t on);
static void motor_commutate(uint8_t hall_state, float duty);

/* ======================== 初始化函数 ======================== */

static void hall_speed_init(void)
{
    gpio_init_type        gpio_cfg;
    tmr_input_config_type tmr_in_cfg;

    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK,  TRUE);

    gpio_pin_remap_config(SWJTAG_MUX_010, TRUE);

    gpio_default_para_init(&gpio_cfg);
    gpio_cfg.gpio_mode = GPIO_MODE_INPUT;
    gpio_cfg.gpio_pull = GPIO_PULL_UP;

    gpio_cfg.gpio_pins = HALL_V_PIN;
    gpio_init(HALL_V_PORT, &gpio_cfg);
    gpio_cfg.gpio_pins = HALL_U_PIN | HALL_W_PIN;
    gpio_init(HALL_U_PORT, &gpio_cfg);

    gpio_pin_remap_config(TMR2_MUX_11, TRUE);

    tmr_base_init(TMR2, TMR2_PERIOD_MAX, TMR_PRESCALER_1US);
    tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
    tmr_32_bit_function_enable(TMR2, TRUE);

    tmr_channel1_input_select(TMR2, TMR_CHANEL1_2_3_CONNECTED_C1IRAW_XOR);
    tmr_hall_select(TMR2, TRUE);

    tmr_input_default_para_init(&tmr_in_cfg);
    tmr_in_cfg.input_channel_select  = TMR_SELECT_CHANNEL_1;
    tmr_in_cfg.input_polarity_select  = TMR_INPUT_BOTH_EDGE;
    tmr_in_cfg.input_mapped_select    = TMR_CC_CHANNEL_MAPPED_DIRECT;
    tmr_in_cfg.input_filter_value     = 0x06;
    tmr_input_channel_init(TMR2, &tmr_in_cfg, TMR_CHANNEL_INPUT_DIV_1);
}

static void motor_pwm_init(void)
{
    gpio_init_type          gpio_cfg;
    tmr_output_config_type  tmr_out_cfg;
    tmr_brkdt_config_type   brkdt_cfg;

    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK,  TRUE);
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio_cfg);
    gpio_cfg.gpio_mode  = GPIO_MODE_MUX;
    gpio_cfg.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_cfg.gpio_pull  = GPIO_PULL_DOWN;
    gpio_cfg.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_cfg.gpio_pins = HS_W_PIN | HS_V_PIN | HS_U_PIN;
    gpio_init(GPIOA, &gpio_cfg);

    gpio_cfg.gpio_mode  = GPIO_MODE_OUTPUT;
    gpio_cfg.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_cfg.gpio_pull  = GPIO_PULL_DOWN;
    gpio_cfg.gpio_pins  = LS_U_PIN | LS_V_PIN | LS_W_PIN;
    gpio_init(GPIOB, &gpio_cfg);
    gpio_bits_reset(GPIOB, LS_U_PIN | LS_V_PIN | LS_W_PIN);

    tmr_base_init(TMR1, PWM_PERIOD, TMR_PRESCALER_1US);
    tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);

    tmr_output_default_para_init(&tmr_out_cfg);
    tmr_out_cfg.oc_mode         = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_out_cfg.oc_idle_state   = FALSE;
    tmr_out_cfg.occ_idle_state  = FALSE;
    tmr_out_cfg.oc_polarity     = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_out_cfg.occ_polarity    = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_out_cfg.oc_output_state = TRUE;
    tmr_out_cfg.occ_output_state= FALSE;

    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &tmr_out_cfg);
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_2, &tmr_out_cfg);
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_3, &tmr_out_cfg);

    /* 配置通道4用于产生ADC触发信号（无物理输出） */
    {
        tmr_output_config_type tmr_out_cfg4;
        tmr_output_default_para_init(&tmr_out_cfg4);
        tmr_out_cfg4.oc_mode         = TMR_OUTPUT_CONTROL_PWM_MODE_B;
        tmr_out_cfg4.oc_idle_state   = FALSE;
        tmr_out_cfg4.occ_idle_state  = FALSE;
        tmr_out_cfg4.oc_polarity     = TMR_OUTPUT_ACTIVE_LOW;
        tmr_out_cfg4.occ_polarity    = TMR_OUTPUT_ACTIVE_LOW;
        tmr_out_cfg4.oc_output_state = FALSE;   /* 不输出到引脚 */
        tmr_out_cfg4.occ_output_state= FALSE;
        tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_4, &tmr_out_cfg4);
        tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, (PWM_PERIOD + 1) / 2);
    }

    /* 设置 TRGO 输出为 OC4REF (使用寄存器操作，值 4) */
    TMR1->ctrl2_bit.ptos = 4;       // 正确字段名

    tmr_brkdt_default_para_init(&brkdt_cfg);
    brkdt_cfg.deadtime         = 120;
    brkdt_cfg.brk_polarity     = TMR_BRK_INPUT_ACTIVE_HIGH;
    brkdt_cfg.wp_level         = TMR_WP_OFF;
    brkdt_cfg.auto_output_enable = TRUE;
    brkdt_cfg.fcsoen_state     = FALSE;
    brkdt_cfg.fcsodis_state    = FALSE;
    brkdt_cfg.brk_enable       = FALSE;
    tmr_brkdt_config(TMR1, &brkdt_cfg);

    tmr_output_enable(TMR1, TRUE);

    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, 0);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, 0);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, 0);

    tmr_counter_enable(TMR1, TRUE);
}

static void adc_init(void)
{

    gpio_init_type    gpio_cfg;
    adc_base_config_type adc_cfg;
    dma_init_type     dma_cfg;

    /* 1. 时钟使能 */
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

    /* 2. 配置 PA7 / PB0 / PB1 为模拟输入电流采样 ，配置 PA2 为模拟输入转把信号*/
    gpio_default_para_init(&gpio_cfg);
    gpio_cfg.gpio_mode = GPIO_MODE_ANALOG;
    gpio_cfg.gpio_pull = GPIO_PULL_NONE;

    gpio_cfg.gpio_pins = GPIO_PINS_7 |GPIO_PINS_2;
    gpio_init(GPIOA, &gpio_cfg);

    gpio_cfg.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1;
    gpio_init(GPIOB, &gpio_cfg);
    


    /* 3. ADC 复位与基本配置 */
    adc_reset(ADC1);

    adc_base_default_para_init(&adc_cfg);
    adc_cfg.sequence_mode = TRUE;      // 序列模式使能
    adc_cfg.repeat_mode = FALSE;       // 单次模式
    adc_cfg.data_align = ADC_RIGHT_ALIGNMENT; // 数据右对齐
    adc_cfg.ordinary_channel_length = 4; //4个输入
    adc_base_config(ADC1, &adc_cfg);

    /* 4. 配置普通通道顺序 (IN7 -> IN8 -> IN9) */
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_7, 1, ADC_SAMPLETIME_239_5);
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_2, 2, ADC_SAMPLETIME_239_5);
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_8, 3, ADC_SAMPLETIME_239_5);
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_9, 4, ADC_SAMPLETIME_239_5);
    


    /* 5. 设置触发源为 TMR1 TRGO（外部触发） */
    adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_TMR1TRGOUT, TRUE);

    /* 6. 使能 ADC 并执行校准 */
    adc_enable(ADC1, TRUE);
    adc_calibration_init(ADC1);
    while (adc_calibration_init_status_get(ADC1));
    adc_calibration_start(ADC1);
    while (adc_calibration_status_get(ADC1));

    /* 7. 使能 ADC 的 DMA 模式 */
    adc_dma_mode_enable(ADC1, TRUE);

    /* 8. 初始化 DMA1 通道1，用于 ADC1 扫描结果的自动搬运 */
    dma_reset(DMA1_CHANNEL1);
    dma_default_para_init(&dma_cfg);
    dma_cfg.peripheral_base_addr  = (uint32_t)&ADC1->odt;
    dma_cfg.memory_base_addr      = (uint32_t)adc_dma_buffer;
    dma_cfg.direction             = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_cfg.buffer_size           = 4;
    dma_cfg.peripheral_inc_enable = FALSE;
    dma_cfg.memory_inc_enable     = TRUE;
    dma_cfg.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_cfg.memory_data_width     = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_cfg.loop_mode_enable      = TRUE;
    dma_cfg.priority              = DMA_PRIORITY_HIGH;
    dma_init(DMA1_CHANNEL1, &dma_cfg);

    dma_flexible_config(DMA1, FLEX_CHANNEL1, DMA_FLEXIBLE_ADC1);
    dma_channel_enable(DMA1_CHANNEL1, TRUE);

}

/* ======================== 零电流偏移校准 ======================== */

static void current_offset_calibration(void)
{
    uint16_t adc_u, adc_v, adc_w;
    float sum_u, sum_v, sum_w;
    int i;

    /* 临时切换为软件触发，确保能主动采集 */
    adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_SOFTWARE, TRUE);

    sum_u = 0.0f;
    sum_v = 0.0f;
    sum_w = 0.0f;

    dma_flag_clear(DMA1_FDT1_FLAG);

    for (i = 0; i < 3000; i++)
    {
        adc_ordinary_software_trigger_enable(ADC1, TRUE);
        while (dma_flag_get(DMA1_FDT1_FLAG) == RESET);
        dma_flag_clear(DMA1_FDT1_FLAG);

        adc_u = adc_dma_buffer[0];
        adc_v = adc_dma_buffer[2];
        adc_w = adc_dma_buffer[3];

        sum_u += (float)adc_u;
        sum_v += (float)adc_v;
        sum_w += (float)adc_w;
    }

    g_adc_offset_u = sum_u / 3000.0f;
    g_adc_offset_v = sum_v / 3000.0f;
    g_adc_offset_w = sum_w / 3000.0f;

    /* 恢复为外部触发（TMR1 TRGO） */
    adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_TMR1TRGOUT, TRUE);
}

/* ======================== 中断 ======================== */

void TMR2_GLOBAL_IRQHandler(void)
{
    if (tmr_interrupt_flag_get(TMR2, TMR_C1_FLAG) != RESET)
    {
        tmr_flag_clear(TMR2, TMR_C1_FLAG);
        cap_now = tmr_channel_value_get(TMR2, TMR_SELECT_CHANNEL_1);

        hall_u = gpio_input_data_bit_read(HALL_U_PORT, HALL_U_PIN) ? 1u : 0u;
        hall_v = gpio_input_data_bit_read(HALL_V_PORT, HALL_V_PIN) ? 1u : 0u;
        hall_w = gpio_input_data_bit_read(HALL_W_PORT, HALL_W_PIN) ? 1u : 0u;
        hall_state = (hall_u << 2) | (hall_v << 1) | hall_w;

        if (hall_state == 0 || hall_state == 7)
        {
            if (last_valid_hall != 0xFF && last_valid_hall != 0 && last_valid_hall != 7)
                hall_state = last_valid_hall;
            else
                return;
        }
        else
        {
            last_valid_hall = hall_state;
        }

        interval = cap_now - g_hall_last_cap;
        if (interval > 0u)
        {
            g_hall_interval = interval;
            g_motor_speed_rad_s = (TWO_PI_E6 / ((float)interval * (float)HALL_EDGES_PER_REV));
        }
        g_hall_last_cap = cap_now;

        if (hall_state != g_commutated_hall)
        {
            g_commutated_hall = hall_state;
            motor_commutate(hall_state, pwm_p);
        }
    }
}

/* ======================== PWM 换相 ======================== */
/*
 * 换相安全时序（配合死区 120 @ 120 MHz ≈ 1 us）：
 *   1. 先全部关闭下桥臂（3 个 LS 引脚拉低）
 *   2. 再切换上桥臂 PWM（旧相占空比清 0，新相写入目标占空比）
 *   3. 最后打开新下桥臂
 * 这样可确保任意时刻同一相的上下管不会同时导通。
 */

/**
 * @brief 设置上桥臂 PWM 输出
 * @param phase 目标导通相（PHASE_U / PHASE_V / PHASE_W）
 * @param duty  占空比 0.0 ~ 1.0
 * 
 * 通道映射（由硬件引脚决定）：
 *   TMR1_CH1 (PA8) -> HS_W -> PHASE_W
 *   TMR1_CH2 (PA9) -> HS_V -> PHASE_V
 *   TMR1_CH3 (PA10)-> HS_U -> PHASE_U
 * 
 * 若当前已有其他相在导通，则先将旧相占空比清零（关闭上管），
 * 再写入新相比较值，避免两相上桥臂同时导通造成母线直通。
 */
static void set_high_phase(phase_t phase, float duty)
{
    uint32_t cmp;
    cmp = (uint32_t)(duty * (float)(PWM_PERIOD + 1u));
    if (cmp > (uint32_t)(PWM_PERIOD + 1u))
        cmp = (uint32_t)(PWM_PERIOD + 1u);

    /* 如果当前已有上桥臂导通且与目标不同，先将旧相 PWM 关断 */
    if (current_high != 0xFF && current_high != phase)
    {
        if      (current_high == PHASE_U) tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, 0);
        else if (current_high == PHASE_V) tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, 0);
        else if (current_high == PHASE_W) tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, 0);
    }

    /* 写入目标相比较值，PWM 通道与相的对应关系由硬件连线决定 */
    if      (phase == PHASE_U) tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, cmp);
    else if (phase == PHASE_V) tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, cmp);
    else if (phase == PHASE_W) tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, cmp);

    current_high = phase;
}

/**
 * @brief 设置下桥臂 GPIO 输出
 * @param phase 目标相（PHASE_U / PHASE_V / PHASE_W）
 * @param on    1=开启该相下桥臂；0=全部关闭
 * 
 * 注意：无论 phase 传入何值，函数内部都会先将 U/V/W 三个下桥臂
 * 全部拉低，起到硬件互锁作用；仅在 on==1 时才把指定相拉高。
 * 因此 motor_commutate() 里调用 set_low_phase(PHASE_U, 0) 时，
 * PHASE_U 仅作占位，实际效果是"关闭所有下桥臂"。
 */
static void set_low_phase(phase_t phase, uint8_t on)
{
    /* 先无条件关闭全部下桥臂，防止换相期间出现共态导通 */
    gpio_bits_reset(LS_U_PORT, LS_U_PIN);
    gpio_bits_reset(LS_V_PORT, LS_V_PIN);
    gpio_bits_reset(LS_W_PORT, LS_W_PIN);

    if (on)
    {
        if      (phase == PHASE_U) gpio_bits_set(LS_U_PORT, LS_U_PIN);
        else if (phase == PHASE_V) gpio_bits_set(LS_V_PORT, LS_V_PIN);
        else if (phase == PHASE_W) gpio_bits_set(LS_W_PORT, LS_W_PIN);
        current_low = phase;
    }
    else
    {
        current_low = 0xFF;   /* 标记为"无下桥臂导通" */
    }
}

/**
 * @brief 根据霍尔码执行六步换相
 * @param hall_state 当前霍尔组合码（U<<2 | V<<1 | W），范围 1~6
 * @param duty       目标占空比
 * 
 * 处理流程：
 *   1. 过滤非法码 0/7（全 0 或全 1，通常是霍尔故障或转子恰在边界），
 *      用 last_valid_hall 保持上一个有效扇区，防止换相抖动。
 *   2. 查表得到应导通的上/下相。
 *   3. 若目标相与当前已导通的相相同，仅更新占空比（避免不必要的 GPIO 翻转）。
 *   4. 若需换相，按安全时序执行：
 *        关全部下桥臂 -> 切换上桥臂 PWM -> 开新下桥臂
 */
static void motor_commutate(uint8_t hall_state, float duty)
{
    commutation_t comm;

    /* ---- 非法霍尔码容错：000(0) 和 111(7) 不是有效扇区 ---- */
    if (hall_state == 0 || hall_state == 7)
    {
        /* 若从未捕获过有效码，直接忽略；否则保持上次有效扇区 */
        if (last_valid_hall == 0xFF || last_valid_hall == 0 || last_valid_hall == 7)
            return;
        hall_state = last_valid_hall;
    }
    else
    {
        last_valid_hall = hall_state;
    }

    /* ---- 查换相表：霍尔码 -> (上桥臂相, 下桥臂相) ---- */
    comm = comm_table_forward[hall_state];

    /* ---- 若上/下相均未改变，只需平滑调节占空比，无需换相 ---- */
    if (comm.high_phase == current_high && comm.low_phase == current_low)
    {
        set_high_phase(comm.high_phase, duty);
        return;
    }

    /* ---- 执行换相：先关下桥臂，再切上桥臂，最后开新下桥臂 ----
     * 第 1 步：关闭所有下桥臂（PHASE_U 在此仅作占位，函数内部会全关）
     * 第 2 步：set_high_phase 内部先关旧上桥臂，再开新上桥臂
     * 第 3 步：开启目标下桥臂
     */
    set_low_phase(PHASE_U, 0);              /* 关闭全部下桥臂 */
    set_high_phase(comm.high_phase, duty);  /* 切换上桥臂 PWM */
    set_low_phase(comm.low_phase, 1);       /* 开启目标下桥臂 */
}



// 转把控制
static void Speed_set(void)
{
    float v_pa2;

    // 读取 ADC 原始值
    adc_pa2_value = adc_dma_buffer[1];

    // 转换为实际电压
    f_pa2_value = (float)adc_pa2_value * (V_REF / 4095.0f)/0.76;

    // 归一化百分比 (0.8V -> 0%, 3.8V -> 100%)
    f_pa2_percent = (f_pa2_value - 0.8f) / (2.8f - 0.8f) * 100.0f;

    // 限幅
    if (f_pa2_percent < 0.0f) f_pa2_percent = 0.0f;
    if (f_pa2_percent > 100.0f) f_pa2_percent = 100.0f;

    pwm_p=(f_pa2_percent/100*0.9f)+0.1f;

}

/* ======================== 电流采样（同步触发） ======================== */

static void current_sample(void)
{
    float v_shunt_u, v_shunt_v, v_shunt_w;
    float raw_u, raw_v, raw_w;
    float sum_u, sum_v, sum_w;
    int   i;
    static float filt_u = 0.0f;      /* 一阶滤波历史值 */
    static float filt_v = 0.0f;
    static float filt_w = 0.0f;
    static uint8_t first_call = 1;   /* 首次调用标志 */
    float alpha = 0.05f;             /* 低通滤波系数，越小越平滑（对应约 100 Hz @ 20kHz 采样） */

    /* ---- 1. 过采样：采集 8 帧求平均，降低噪声 ---- */
    sum_u = 0.0f;
    sum_v = 0.0f;
    sum_w = 0.0f;

    for (i = 0; i < 8; i++)
    {
        /* 等待 TMR1 自动触发 ADC 完成扫描，DMA 传输完成标志置位 */
        while (dma_flag_get(DMA1_FDT1_FLAG) == RESET);
        dma_flag_clear(DMA1_FDT1_FLAG);

        sum_u += (float)adc_dma_buffer[0];
        sum_v += (float)adc_dma_buffer[2];
        sum_w += (float)adc_dma_buffer[3];
    }

    /* 平均 ADC 值 */
    raw_u = sum_u / 8.0f;
    raw_v = sum_v / 8.0f;
    raw_w = sum_w / 8.0f;

    /* ---- 2. 减去零电流偏移，计算分流电阻上的差分电压 ---- */
    v_shunt_u = (raw_u - g_adc_offset_u) * (V_REF / 4095.0f);
    v_shunt_v = (raw_v - g_adc_offset_v) * (V_REF / 4095.0f);
    v_shunt_w = (raw_w - g_adc_offset_w) * (V_REF / 4095.0f);

    /* ---- 3. 转换为相电流（A），并钳位负值为 0 ---- */
    raw_u = v_shunt_u / (CURRENT_SENSE_R * CURRENT_SENSE_GAIN);
    raw_v = v_shunt_v / (CURRENT_SENSE_R * CURRENT_SENSE_GAIN);
    raw_w = v_shunt_w / (CURRENT_SENSE_R * CURRENT_SENSE_GAIN);

    if (raw_u < 0.0f) raw_u = 0.0f;
    if (raw_v < 0.0f) raw_v = 0.0f;
    if (raw_w < 0.0f) raw_w = 0.0f;

    /* ---- 4. 一阶低通滤波（首次调用直接赋值） ---- */
    if (first_call)
    {
        filt_u = raw_u;
        filt_v = raw_v;
        filt_w = raw_w;
        first_call = 0;
    }
    else
    {
        filt_u = filt_u + alpha * (raw_u - filt_u);
        filt_v = filt_v + alpha * (raw_v - filt_v);
        filt_w = filt_w + alpha * (raw_w - filt_w);
    }

    /* 更新全局相电流（平滑后） */
    g_phase_current_u = filt_u;
    g_phase_current_v = filt_v;
    g_phase_current_w = filt_w;

    /* ---- 5. 母线电流：取当前低侧相的平滑电流（已保证 >= 0） ---- */
    switch (current_low)
    {
        case PHASE_U:
            g_bus_current = filt_u;
            break;
        case PHASE_V:
            g_bus_current = filt_v;
            break;
        case PHASE_W:
            g_bus_current = filt_w;
            break;
        default:
            g_bus_current = 0.0f;
            break;
    }

}


/* ======================== 主函数 ======================== */

int main(void)
{
    uint8_t start_comm_done;
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    system_clock_config();
    at32_board_init();

    hall_speed_init();
    motor_pwm_init();
    adc_init();

    /* 执行零电流偏移校准（电机未通电） */
    current_offset_calibration();

    /* 校准完成后，启动霍尔定时器和中断 */
    tmr_interrupt_enable(TMR2, TMR_C1_INT, TRUE);
    nvic_irq_enable(TMR2_GLOBAL_IRQn, 1, 0);
    tmr_counter_enable(TMR2, TRUE);

    start_comm_done = 0;

    while (1)
    {
        current_sample();
        Speed_set();
        if (!start_comm_done)
        {
            if (last_valid_hall != 0xFF && last_valid_hall != 0 && last_valid_hall != 7)
            {
                motor_commutate(last_valid_hall, pwm_p);
                start_comm_done = 1;
            }
        }
    }
}

/* end of file */