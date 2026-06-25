#include <stdio.h>
#include <math.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "iot_gpio_ex.h"
#include "hi_io.h"
#include "hi_time.h"
#include "radar_sg90.h"
#include "car_pwm_motor.h"

// ==================== 常量 ====================
// 需根据实际小车行驶情况进行调整
#define MIN_DUTY                50          // 电机最小占空比（低于此值自动钳位）
#define BACKWARD_DURATION_MS    400         // 避障时后退持续时间(ms)
#define TURN_DURATION_MS        80          // ?????????(ms)
#define FILTER_COUNT            3           // 滑动平均滤波采样次数
#define DIST_CHANGE_THRESH      2.0f        // 距离变化阈值(cm)，超过才打印日志
// ==================== 全局变量 ====================
// 超声波模块配置（引脚、采样间隔、障碍物阈值）
// 需根据实际小车行驶情况进行调整
static RadarConfig g_cfg = {
    .trig_gpio = RADAR_TRIG_GPIO,
    .echo_gpio = RADAR_ECHO_GPIO,
    .sample_interval_ms = 10,
    .obstacle_distance = OBSTACLE_DISTANCE_CM
};
// 超声波当前状态（距离、运行状态、障碍物状态）
static RadarStatus g_status = {0};
// 模块运行标志：1=运行中，0=停止
static volatile int g_running = 0;
// 避障任务线程ID
static osThreadId_t g_task_id = NULL;
// 避障状态机阶段：空闲/后退/转向
typedef enum { PHASE_IDLE, PHASE_BACKWARD, PHASE_TURN } Phase;
// 当前避障阶段
static Phase g_phase = PHASE_IDLE;
// 阶段开始时间戳（用于计时后退/转向时长）
static uint32_t g_phase_start = 0;
// 滑动平均滤波缓存数组
static float g_distance_filter[FILTER_COUNT] = {0};
// 滤波缓存当前索引
static int g_filter_idx = 0;
// 滤波缓存是否填满标志：0=未填满，1=已填满
static int g_filter_filled = 0;
// 上一次打印的距离值（用于判断是否需要打印）
static float g_last_printed_dist = -1.0f;
// ==================== 辅助函数 ====================
/**
 * @brief  占空比钳位函数（保底函数）
 * @param  d: 输入的原始占空比
 * @return 处理后的合法占空比
 * @note   逻辑：如果输入 < 50，强制返回50；否则返回原值
 *         作用：防止电机因占空比过低无法启动、堵转
 */
static uint8_t clamp_duty(uint8_t d) {
    return (d < MIN_DUTY) ? MIN_DUTY : d;
}
// ==================== GPIO 初始化 ====================
/**
 * @brief  超声波模块GPIO初始化
 * @return 无
 * @note   初始化Trig为输出（发脉冲），Echo为输入（读回波）
 */
static void init_gpio(void) {
    // 初始化Trig引脚：输出模式，默认低电平
    IoTGpioInit(g_cfg.trig_gpio);
    IoSetFunc(g_cfg.trig_gpio, 0);
    IoTGpioSetDir(g_cfg.trig_gpio, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(g_cfg.trig_gpio, IOT_GPIO_VALUE0);

    // 初始化Echo引脚：输入模式，接收超声波回波信号
    IoTGpioInit(g_cfg.echo_gpio);
    IoSetFunc(g_cfg.echo_gpio, 0);
    IoTGpioSetDir(g_cfg.echo_gpio, IOT_GPIO_DIR_IN);
}
// ==================== 超声波测距（精确微秒级） ====================
/**
 * @brief  发送超声波触发信号
 * @return 无
 * @note   给Trig引脚一个10us的高电平脉冲，启动模块测距
 */
static void trig_pulse(void) {
    IoTGpioSetOutputVal(g_cfg.trig_gpio, IOT_GPIO_VALUE1);
    hi_udelay(10);  // 持续10微秒
    IoTGpioSetOutputVal(g_cfg.trig_gpio, IOT_GPIO_VALUE0);
}
/**
 * @brief  等待Echo引脚变为指定电平（带超时）
 * @param  level: 等待的电平（高/低）
 * @param  timeout_us: 超时时间（微秒）
 * @return 0=成功，-1=超时
 * @note   用于精确测量Echo高电平持续时间
 */
static int wait_echo(IotGpioValue level, uint32_t timeout_us) {
    IotGpioValue val;
    uint32_t start = hi_get_us();  // 记录开始时间

    // 循环读取引脚，直到超时或匹配目标电平
    while ((hi_get_us() - start) < timeout_us) {
        IoTGpioGetInputVal(g_cfg.echo_gpio, &val);
        if (val == level) return 0;  // 匹配成功
        osDelay(0);   // 让出CPU，避免系统阻塞
    }
    return -1;  // 超时
}
/**
 * @brief  执行一次超声波测距，返回距离（cm）
 * @return 成功返回距离值，失败返回-1
 * @note   计算公式：距离 = 高电平时间 * 0.034 / 2
 *         0.034：声速cm/us，除以2：往返路程
 */
float radar_measure_distance(void) {
    trig_pulse();  // 发触发脉冲

    // 等待Echo变高（开始计时）
    if (wait_echo(IOT_GPIO_VALUE1, ULTRASOUND_TIMEOUT_US) != 0) return -1;
    uint32_t start = hi_get_us();

    // 等待Echo变低（结束计时）
    if (wait_echo(IOT_GPIO_VALUE0, ULTRASOUND_TIMEOUT_US) != 0) return -1;
    uint32_t end = hi_get_us();

    uint32_t pulse_us = end - start;
    if (pulse_us == 0) return -1;

    // 计算实际距离
    float dist = (float)pulse_us * 0.034f / 2.0f;

    // 限制测距范围（2cm~400cm）
    if (dist < 2.0f) dist = 2.0f;
    if (dist > 400.0f) dist = 400.0f;

    return dist;
}
/**
 * @brief  滑动平均滤波，获取稳定的距离值
 * @return 滤波后的距离
 * @note   连续采样3次取平均，减少噪声抖动
 */
static float get_filtered_distance(void) {
    float raw = radar_measure_distance();  // 读取原始值
    if (raw < 0) return g_status.distance_cm;  // 读取失败，返回上一次有效值

    // 将新采样值存入滤波数组
    g_distance_filter[g_filter_idx] = raw;
    g_filter_idx = (g_filter_idx + 1) % FILTER_COUNT;  // 循环索引

    // 标记数组是否已填满（必须采满3次才开始平均）
    if (!g_filter_filled && g_filter_idx == 0)
        g_filter_filled = 1;

    // 数组填满 → 求平均值
    if (g_filter_filled) {
        float sum = 0;
        for (int i = 0; i < FILTER_COUNT; i++)
            sum += g_distance_filter[i];
        g_status.distance_cm = sum / FILTER_COUNT;
    } else {
        // 未填满 → 直接用原始值
        g_status.distance_cm = raw;
    }

    return g_status.distance_cm;
}
// ==================== 任务线程 ====================
// ==================== 对外接口 ====================

static void radar_task(void *arg);
/**
 * @brief  雷达模块初始化
 * @param  cfg: 配置结构体（可为NULL）
 * @return 0=成功
 * @note   初始化GPIO、状态变量
 */
int radar_module_init(const RadarConfig *cfg) {
    if (cfg)
        g_cfg = *cfg;  // 传入自定义配置

    init_gpio();                          // 初始化引脚

    // 初始化状态
    g_status.distance_cm = 0;
    g_status.state = RADAR_STATE_IDLE;
    g_status.is_running = 0;

    printf("[RADAR] Init OK, obstacle=%d cm\n", g_cfg.obstacle_distance);
    return 0;
}
/**
 * @brief  启动雷达测距任务
 * @return 无
 * @note   创建线程，开始循环测距+避障
 */
void radar_start(void) {
    if (g_running) return;  // 已运行则直接返回

    // 重置运行标志与状态
    g_running = 1;
    g_status.is_running = 1;
    g_phase = PHASE_IDLE;
    g_filter_filled = 0;
    g_filter_idx = 0;
    g_last_printed_dist = -1.0f;

    // 创建任务线程
    osThreadAttr_t attr = {
        .name = "RadarTask",
        .stack_size = 4096,
        .priority = 24
    };
    g_task_id = osThreadNew(radar_task, NULL, &attr);

    if (!g_task_id) {
        g_running = 0;
        g_status.is_running = 0;
        printf("[RADAR] Task create failed\n");
    } else {
        printf("[RADAR] Started\n");
    }
}
/**
 * @brief  停止雷达测距任务
 * @return 无
 * @note   停止线程，停车，重置状态
 */
void radar_stop(void) {
    if (!g_running) return;

    g_running = 0;  // 让循环退出

    if (g_task_id) {
        osDelay(100);  // 等待线程安全退出
        g_task_id = NULL;
    }

    g_status.is_running = 0;
    car_stop();
    printf("[RADAR] Stopped\n");
}
/**
 * @brief  获取模块是否正在运行
 * @return 1=运行，0=停止
 */
int radar_is_running(void) {
    return g_running;
}

/**
 * @brief  获取当前雷达状态（距离、工作状态）
 * @param  st: 输出状态结构体
 * @return 无
 */
void radar_get_status(RadarStatus *st) {
    if (st)
        *st = g_status;
}

// ==================== 避障动作 ====================
/**
 * @brief  避障核心逻辑（状态机）
 * @param  dist: 当前滤波后距离
 * @return 无
 * @note   状态：空闲 → 检测到障碍 → 后退 → 转向 → 恢复前进
 */
static void do_avoidance(float dist) {
    uint32_t now = osKernelGetTickCount();  // 获取当前系统时间

    // ==================== 状态机正在执行避障动作 ====================
    if (g_phase != PHASE_IDLE) {
        uint32_t elapsed = now - g_phase_start;  // 计算当前阶段已执行时间
        switch (g_phase) {
            case PHASE_BACKWARD:
                // ?????? ? ????
                if (elapsed < BACKWARD_DURATION_MS) {
                    car_backward(clamp_duty(70));
                    break;
                }
                // ???? ? ???????
                g_phase = PHASE_TURN;
                g_phase_start = now;
                car_stop();
                break;

            case PHASE_TURN: {
                // ?????? ? ????
                if (elapsed < TURN_DURATION_MS) {
                    static int turn_dir = 0;
                    if (turn_dir) {
                        car_turn_left(clamp_duty(50), clamp_duty(70));
                    } else {
                        car_turn_right(clamp_duty(70), clamp_duty(50));
                    }
                    break;
                }
                // ???? ? ?????????
                {
                    static int turn_toggle = 0;
                    turn_toggle = !turn_toggle;
                }
                g_phase = PHASE_IDLE;
                car_forward(clamp_duty(70));
                break;
            }

            default:
                break;
        }
        return;
    }

    // ==================== ????????? ====================
    // ???? ? ????
    if (dist < 0) {
        car_forward(clamp_duty(70));
        return;
    }

    // ?????? ? ??????? ? ??????
    if (dist < g_cfg.obstacle_distance) {
        printf("[RADAR] Obstacle %.1f cm -> backward\n", dist);
        car_stop();
        g_phase = PHASE_BACKWARD;
        g_phase_start = now;
        g_last_printed_dist = -1.0f;
        return;
    }

    // ???? ? ????
    car_forward(clamp_duty(70));

    // ??????2cm??????????
    if (g_last_printed_dist < 0 || fabs(dist - g_last_printed_dist) >= DIST_CHANGE_THRESH) {
        printf("[RADAR] Distance: %.1f cm, Forward\n", dist);
        g_last_printed_dist = dist;
    }
}

// ==================== 任务线程 ====================
/**
 * @brief  超声波避障主任务线程
 * @param  arg: 线程参数（未使用）
 * @return 无
 * @note   循环：测距 → 更新状态 → 执行避障 → 延时
 */
static void radar_task(void *arg) {
    (void)arg;
    printf("[RADAR] Task started\n");

    // 模块运行时一直循环
    while (g_running) {
        float d = get_filtered_distance();  // 获取滤波后距离

        // 更新雷达状态（供外部读取）
        if (d < 10)
            g_status.state = RADAR_STATE_OBSTACLE_BACK;
        else if (d < g_cfg.obstacle_distance)
            g_status.state = RADAR_STATE_OBSTACLE_FRONT;
        else
            g_status.state = RADAR_STATE_FRONT_CLEAR;

        do_avoidance(d);                  // 执行避障逻辑
        osDelay(g_cfg.sample_interval_ms); // 采样间隔
    }

    // 任务停止 → 停车
    car_stop();
    g_status.state = RADAR_STATE_IDLE;
    printf("[RADAR] Task stopped\n");
}
