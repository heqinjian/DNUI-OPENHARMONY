#include <stdio.h>
#include <unistd.h>

#include "ohos_init.h"
#include "cmsis_os2.h"

#include "trace_model.h"
#include "car_pwm_motor.h"
#include "iot_gpio.h"
#include "iot_gpio_ex.h"
#include "hi_io.h"

// ==================== 模块内部变量 ====================
static TraceConfig g_trace_config = {
    .sensor_left_gpio = TRACE_SENSOR_LEFT_PIN,
    .sensor_right_gpio = TRACE_SENSOR_RIGHT_PIN,
    .duty = 60,                     // 前进占空 ?0%
    .mode = TRACE_MODE_LINE_FOLLOWING,
    .sample_interval_ms = 2
};
static TraceStatus g_trace_status = {
    .left_sensor = TRACE_SENSOR_WHITE,
    .right_sensor = TRACE_SENSOR_WHITE,
    .trace_state = CAR_TRACE_STOP,
    .left_duty = 0,
    .right_duty = 0
};

static volatile int g_trace_running = 0;
static osThreadId_t g_trace_task_id = NULL;

// 丢失前的运动状态（保留但未用于丢失后处理，为扩展保留）
typedef enum {
    LAST_MOVE_FORWARD = 0,
    LAST_MOVE_LEFT,
    LAST_MOVE_RIGHT
} LastMoveState;
static volatile LastMoveState g_last_move_state = LAST_MOVE_FORWARD;

static CarTraceState g_last_printed_state = CAR_TRACE_STOP;
static TraceSensorState read_left_sensor(void)
{
    IotGpioValue value;
    IoTGpioGetInputVal(g_trace_config.sensor_left_gpio, &value);
    // 低电 ?检测到黑线，高电平=白色
    return (value == IOT_GPIO_VALUE0) ? TRACE_SENSOR_BLACK : TRACE_SENSOR_WHITE;
}
static TraceSensorState read_right_sensor(void)
{
    IotGpioValue value;
    IoTGpioGetInputVal(g_trace_config.sensor_right_gpio, &value);
  return (value == IOT_GPIO_VALUE0) ? TRACE_SENSOR_BLACK : TRACE_SENSOR_WHITE;
}
static void control_motors_by_state(TraceSensorState left, TraceSensorState right)
{
    uint8_t forward_duty = g_trace_config.duty;   // 前进占空比（用户可调 ?
   // 需根据小车和轨道实际情况调整转向占空比和丢线占空比
    uint8_t turn_duty = 37;                      // 转向内侧轮占空比（低速，增大差速提升过弯能力）
     uint8_t slow_duty = 90;                     // 丢线后减速直行占空比

    CarTraceState final_action = CAR_TRACE_STOP;
uint8_t final_left_duty = 0, final_right_duty = 0;

// 后续小车循迹行驶，停止等逻辑（黄色标注部分）
// 需根据每台小车不同的情况进行调整，
// 代码给出的为基础逻辑，不能保证循迹功能的稳定实现

// ========== 1. 双黑：立即停 ?==========
    if (left == TRACE_SENSOR_BLACK && right == TRACE_SENSOR_BLACK) {
        // 双传感器均为黑，判定为停止线，立即停 ?
        final_action = CAR_TRACE_STOP_LINE;
        final_left_duty = 0;
        final_right_duty = 0;
    }
    // ========== 2. 单边黑：循迹修正 ==========
    else if (right == TRACE_SENSOR_BLACK) {
        // 左白右黑：小车偏左，右转修正
        final_action = CAR_TRACE_RIGHT_ADJUST;
        final_left_duty = forward_duty;
        final_right_duty = turn_duty;
        g_last_move_state = LAST_MOVE_RIGHT;
    }
    else if (left == TRACE_SENSOR_BLACK) {
        // 左黑右白：小车偏右，左转修正
        final_action = CAR_TRACE_LEFT_ADJUST;
        final_left_duty = turn_duty;
        final_right_duty = forward_duty;
        g_last_move_state = LAST_MOVE_LEFT;
    }
    // ========== 3. 双白：正常直行（传感器跨在黑线两侧，双白表示在轨道上 ?==========
    else { // left == WHITE && right == WHITE
        // 双白说明小车位于黑线正上方，保持直行
        final_action = CAR_TRACE_FORWARD;
        final_left_duty = forward_duty;
        final_right_duty = forward_duty;
    }

    // 更新全局状态（供外部查询）
    g_trace_status.trace_state = final_action;
    g_trace_status.left_duty = final_left_duty;
    g_trace_status.right_duty = final_right_duty;

    // 执行电机控制
    switch (final_action) {
        case CAR_TRACE_FORWARD:
            car_forward(final_left_duty);
            break;
        case CAR_TRACE_LEFT_ADJUST:
            car_turn_left(final_left_duty, final_right_duty);
            break;
        case CAR_TRACE_RIGHT_ADJUST:
            car_turn_right(final_left_duty, final_right_duty);
            break;
        case CAR_TRACE_STOP_LINE:
        default:
            car_stop();
            break;
    }

    // 仅当状态改变时打印，避免刷 ?
    if (g_last_printed_state != final_action) {
        g_last_printed_state = final_action;
        const char* action_name[] = {"STOP", "FORWARD", "LEFT_ADJ", "RIGHT_ADJ", "LOST", "STOP_LINE"};
        printf("==>[TRACE] State=%s, L_duty=%d%%, R_duty=%d%%\n",
               action_name[final_action], final_left_duty, final_right_duty);
    }

}
int trace_module_init(const TraceConfig *config)
{
    if (config != NULL) {
        g_trace_config.sensor_left_gpio = config->sensor_left_gpio;
        g_trace_config.sensor_right_gpio = config->sensor_right_gpio;
        g_trace_config.duty = config->duty;
        g_trace_config.mode = config->mode;
        g_trace_config.sample_interval_ms = config->sample_interval_ms;
    }

    // 初始化左红外传感器GPIO - 输入模式
 IoTGpioInit(g_trace_config.sensor_left_gpio);
 IoSetFunc(g_trace_config.sensor_left_gpio, IOT_IO_FUNC_GPIO_11_GPIO);
 IoTGpioSetDir(g_trace_config.sensor_left_gpio, IOT_GPIO_DIR_IN);

    // 初始化右红外传感器GPIO - 输入模式
 IoTGpioInit(g_trace_config.sensor_right_gpio);
 IoSetFunc(g_trace_config.sensor_right_gpio, IOT_IO_FUNC_GPIO_12_GPIO);
 IoTGpioSetDir(g_trace_config.sensor_right_gpio, IOT_GPIO_DIR_IN);


printf("==>[TRACE] Init: Left=GPIO%d, Right=GPIO%d, Duty=%d%%\n",
           g_trace_config.sensor_left_gpio,
           g_trace_config.sensor_right_gpio,
           g_trace_config.duty);
    return 0;
}
void trace_get_status(TraceStatus *status)
{
    if (status != NULL) {
        status->left_sensor = g_trace_status.left_sensor;
        status->right_sensor = g_trace_status.right_sensor;
        status->trace_state = g_trace_status.trace_state;
        status->left_duty = g_trace_status.left_duty;
        status->right_duty = g_trace_status.right_duty;
    }
}

void trace_set_mode(TraceMode mode)
{
    g_trace_config.mode = mode;
   printf("==>[TRACE] Mode changed to: %s (edge detection not fully implemented)\n",
       mode == TRACE_MODE_LINE_FOLLOWING ? "Line Following" : "Edge Detection");
}

void trace_set_speed(uint8_t speed)
{
    if (speed > 99) speed = 99;
    g_trace_config.duty = speed;
}
void trace_task(void *arg)
{
   (void)arg;
    TraceSensorState left_sensor, right_sensor;

    printf("==>[TRACE] Trace task started\n");

    while (g_trace_running) {
        // 读取左右传感器状 ?
        left_sensor = read_left_sensor();
        right_sensor = read_right_sensor();

        // 更新传感器状态（供外部查询）
        g_trace_status.left_sensor = left_sensor;
        g_trace_status.right_sensor = right_sensor;

        // 根据传感器状态控制电 ?
        control_motors_by_state(left_sensor, right_sensor);

        // 延时采样
        osDelay(g_trace_config.sample_interval_ms);
    }

    // 退出前停止电机
    car_stop();
    printf("==>[TRACE] Trace task stopped\n");
}
void trace_start(void)
{
     osThreadAttr_t attr;

    printf("==>[TRACE] trace_start called\n");

    if (g_trace_running) {
        printf("==>[TRACE] Warning: Trace already running\n");
        return;
    }

    // 重置所有计数器和状态，避免残留值影响新的一次循 ?
    g_last_move_state = LAST_MOVE_FORWARD;
    g_last_printed_state = CAR_TRACE_STOP;

    g_trace_running = 1;

    // 创建循迹任务 - 增大栈空间避免溢 ?
    attr.name = "TraceTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 12;   // 12KB
    attr.priority = 24;            // 稍高于控制任 ?

    g_trace_task_id = osThreadNew(trace_task, NULL, &attr);
    if (g_trace_task_id == NULL) {
        printf("==>[TRACE] Error: Failed to create trace task\n");
        g_trace_running = 0;
        return;
    }

    printf("==>[TRACE] Trace started, task_id=%p\n", g_trace_task_id);
}
void trace_stop(void)
{
    if (!g_trace_running) {
        printf("==>[TRACE] Warning: Trace not running\n");
        return;
    }

    g_trace_running = 0;

    // 等待任务退 ?
    if (g_trace_task_id != NULL) {
        osDelay(50); // 等待任务自然退 ?
        g_trace_task_id = NULL;
    }

    // 确保电机停止
    car_stop();

    printf("==>[TRACE] Trace stopped\n");
}

int trace_is_running(void)
{
    return g_trace_running;
}