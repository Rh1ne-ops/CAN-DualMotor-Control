#include "state_machine.h"

#include "App.h"
#include "Key.h"
#include "OLED.h"


/* =========================================================
 *                       参数区
 * ========================================================= */

/* 官方参数 + 之前实测基本吻合：输出轴1圈约5775 count */
#define UI_COUNTS_PER_TURN          5775L

/* 圈数设置范围 */
#define UI_TURN_MIN                 1
#define UI_TURN_MAX                 20

/*
 * target_speed单位仍然是 count / 10ms。
 * 目前10已经验证稳定；考核时间要求后面再实测调到合适值。
 */
#define UI_SPEED_MIN                1
#define UI_SPEED_MAX                60

/* 等待时间：DELAY模式的启动延时 / COMBO模式的中间等待 */
#define UI_DELAY_MIN_MS             0
#define UI_DELAY_MAX_MS             5000
#define UI_DELAY_STEP_MS            500

/* 按键扫描 */
#define UI_KEY_SCAN_PERIOD_MS       10U

/* 运行数据刷新：只更新变化数字，不整页重画 */
#define UI_REFRESH_PERIOD_MS        200U

/* 菜单第三页 / FAULT页的CAN状态刷新 */
#define UI_SLOW_REFRESH_MS          500U


/* =========================================================
 *                       UI状态
 * ========================================================= */

typedef enum
{
    UI_STATE_MENU = 0,
    UI_STATE_RUNNING,
    UI_STATE_STOPPING,
    UI_STATE_RESULT,
    UI_STATE_FAULT
} UI_State_t;


/* =========================================================
 *                       菜单项目
 * ========================================================= */

typedef enum
{
    MENU_MODE = 0,

    MENU_S1_TURN,
    MENU_S1_DIR,
    MENU_S1_SPEED,

    MENU_S2_TURN,
    MENU_S2_DIR,
    MENU_S2_SPEED,

    MENU_DELAY,

    MENU_START,

    MENU_ITEM_COUNT
} Menu_Item_t;


/* =========================================================
 *                       UI变量
 * ========================================================= */

static UI_State_t ui_state = UI_STATE_MENU;
static uint8_t menu_index = MENU_MODE;
static uint8_t edit_mode = 0;

/* 运行时PAGE可切换 MAIN / DETAIL 两页 */
static uint8_t run_page = 0;

/* ---------------- 菜单参数 ---------------- */

static App_Mode_t ui_mode = APP_MODE_SYNC;

static uint8_t ui_s1_turns = 1;
static uint8_t ui_s2_turns = 1;

static int8_t ui_s1_direction = 1;
static int8_t ui_s2_direction = 1;

static int16_t ui_s1_speed = 30;
static int16_t ui_s2_speed = 30;

static uint16_t ui_delay_ms = 2000;

/* 交给App层的配置 */
static App_Config_t ui_config;

/* 本轮开始以后保存一份配置快照，运行页面只看这一份 */
static App_Config_t run_config;

/* App运行信息 */
static App_Runtime_t ui_runtime;

/* ---------------- 时间 ---------------- */

static uint32_t last_key_scan_tick = 0;
static uint32_t last_refresh_tick = 0;
static uint32_t last_slow_refresh_tick = 0;

/*
 * UI自己的运行计时器：
 * START成功 -> 开始
 * DONE / STOP / FAULT -> 定格
 */
static uint32_t run_start_ms = 0;
static uint32_t run_elapsed_ms = 0;
static uint8_t run_timer_active = 0;

/* COMBO标题阶段发生变化时，只更新第1行，不整屏刷新 */
static App_Phase_t last_drawn_phase = APP_PHASE_NONE;


/* =========================================================
 *                    内部函数声明
 * ========================================================= */

static void UI_ClearLine(uint8_t line);

static void UI_ApplyConfig(void);
static void UI_Start(void);
static void UI_EnterMenu(void);

static void UI_DrawMenu(void);
static void UI_DrawMenuItem(uint8_t line, Menu_Item_t item);
static void UI_UpdateMenuCan(void);
static void UI_AdjustCurrentItem(int8_t direction);

static void UI_DrawRunningFrame(void);
static void UI_UpdateRunning(void);
static void UI_DrawRunTitle(void);
static void UI_DrawStopping(void);
static void UI_DrawResult(void);
static void UI_DrawFault(void);
static void UI_UpdateFaultCan(void);

static uint32_t UI_Abs32(int32_t value);
static uint32_t UI_CountToTurnX10(int32_t count);
static uint8_t UI_TargetCountToTurns(int32_t target_count);
static void UI_ShowTurnX10(uint8_t line, uint8_t column, uint32_t turn_x10);
static void UI_ShowElapsed(uint8_t line, uint8_t column);
static void UI_UpdateTimer(uint32_t now);
static void UI_FreezeTimer(uint32_t now);


/* =========================================================
 *                       基础工具
 * ========================================================= */

static void UI_ClearLine(uint8_t line)
{
    OLED_ShowString(line, 1, "                ");
}


static uint32_t UI_Abs32(int32_t value)
{
    if (value < 0)
    {
        return (uint32_t)(-value);
    }

    return (uint32_t)value;
}


/* count -> 圈数×10，例如3.2圈返回32 */
static uint32_t UI_CountToTurnX10(int32_t count)
{
    uint32_t abs_count;

    abs_count = UI_Abs32(count);

    return (abs_count * 10U + (uint32_t)UI_COUNTS_PER_TURN / 2U) / (uint32_t)UI_COUNTS_PER_TURN;
}


/* target_count本身就是5775的整数倍，所以直接换回整数圈数 */
static uint8_t UI_TargetCountToTurns(int32_t target_count)
{
    uint32_t turns;

    turns = UI_Abs32(target_count) / (uint32_t)UI_COUNTS_PER_TURN;

    if (turns > 99U)
    {
        turns = 99U;
    }

    return (uint8_t)turns;
}


/* 显示两位整数 + 一位小数，例如03.2 */
static void UI_ShowTurnX10(uint8_t line, uint8_t column, uint32_t turn_x10)
{
    uint32_t integer_part;
    uint32_t decimal_part;

    integer_part = turn_x10 / 10U;
    decimal_part = turn_x10 % 10U;

    if (integer_part > 99U)
    {
        integer_part = 99U;
        decimal_part = 9U;
    }

    OLED_ShowNum(line, column, integer_part, 2);
    OLED_ShowChar(line, column + 2, '.');
    OLED_ShowNum(line, column + 3, decimal_part, 1);
}


/* 显示 xxx.x 秒；TIME运行中增加，停止后保持最后值 */
static void UI_ShowElapsed(uint8_t line, uint8_t column)
{
    uint32_t tenth_second;
    uint32_t sec;
    uint32_t tenth;

    tenth_second = run_elapsed_ms / 100U;
    sec = tenth_second / 10U;
    tenth = tenth_second % 10U;

    if (sec > 999U)
    {
        sec = 999U;
        tenth = 9U;
    }

    OLED_ShowNum(line, column, sec, 3);
    OLED_ShowChar(line, column + 3, '.');
    OLED_ShowNum(line, column + 4, tenth, 1);
}


static void UI_UpdateTimer(uint32_t now)
{
    if (run_timer_active)
    {
        run_elapsed_ms = now - run_start_ms;
    }
}


static void UI_FreezeTimer(uint32_t now)
{
    if (run_timer_active)
    {
        run_elapsed_ms = now - run_start_ms;
        run_timer_active = 0;
    }
}


/* =========================================================
 *                    参数转换给App层
 * ========================================================= */

static void UI_ApplyConfig(void)
{
    ui_config.mode = ui_mode;

    ui_config.slave1.target_count = (int32_t)ui_s1_turns * UI_COUNTS_PER_TURN;
    ui_config.slave1.target_speed = ui_s1_speed;
    ui_config.slave1.direction = ui_s1_direction;

    ui_config.slave2.target_count = (int32_t)ui_s2_turns * UI_COUNTS_PER_TURN;
    ui_config.slave2.target_speed = ui_s2_speed;
    ui_config.slave2.direction = ui_s2_direction;

    ui_config.async_delay_ms = ui_delay_ms;

    /*
     * SYNC模式的含义明确为：
     * 两机同目标、同方向、同速度，同时启动。
     * 用户只需要设置S1参数，S2自动跟随。
     * 若要两机不同圈数/速度，选择ASYNC模式。
     */
    if (ui_mode == APP_MODE_SYNC)
    {
        ui_config.slave2 = ui_config.slave1;
    }
}


/* =========================================================
 *                       菜单绘制
 * ========================================================= */

static void UI_DrawMenuItem(uint8_t line, Menu_Item_t item)
{
    UI_ClearLine(line);

    if (menu_index == item)
    {
        if (edit_mode)
        {
            OLED_ShowString(line, 1, "*");
        }
        else
        {
            OLED_ShowString(line, 1, ">");
        }
    }
    else
    {
        OLED_ShowString(line, 1, " ");
    }

    switch (item)
    {
        case MENU_MODE:

            OLED_ShowString(line, 2, "MODE:");

            switch (ui_mode)
            {
                case APP_MODE_S1_ONLY: OLED_ShowString(line, 7, "S1ONLY"); break;
                case APP_MODE_S2_ONLY: OLED_ShowString(line, 7, "S2ONLY"); break;
                case APP_MODE_SYNC:    OLED_ShowString(line, 7, "SYNC");   break;
                case APP_MODE_ASYNC:   OLED_ShowString(line, 7, "ASYNC");  break;
                case APP_MODE_DELAY:   OLED_ShowString(line, 7, "DELAY");  break;
                case APP_MODE_COMBO:   OLED_ShowString(line, 7, "COMBO");  break;
                default:               OLED_ShowString(line, 7, "?");      break;
            }

            break;


        case MENU_S1_TURN:

            OLED_ShowString(line, 2, "S1TURN:");
            OLED_ShowNum(line, 9, ui_s1_turns, 2);
            break;


        case MENU_S1_DIR:

            OLED_ShowString(line, 2, "S1DIR:");
            OLED_ShowString(line, 8, (ui_s1_direction > 0) ? "FWD" : "REV");
            break;


        case MENU_S1_SPEED:

            OLED_ShowString(line, 2, "S1SPD:");
            OLED_ShowNum(line, 8, ui_s1_speed, 2);
            break;


        case MENU_S2_TURN:

            OLED_ShowString(line, 2, "S2TURN:");
            OLED_ShowNum(line, 9, ui_s2_turns, 2);
            break;


        case MENU_S2_DIR:

            OLED_ShowString(line, 2, "S2DIR:");
            OLED_ShowString(line, 8, (ui_s2_direction > 0) ? "FWD" : "REV");
            break;


        case MENU_S2_SPEED:

            OLED_ShowString(line, 2, "S2SPD:");
            OLED_ShowNum(line, 8, ui_s2_speed, 2);
            break;


        case MENU_DELAY:

            OLED_ShowString(line, 2, "WAIT:");
            OLED_ShowNum(line, 7, ui_delay_ms, 4);
            OLED_ShowString(line, 11, "ms");
            break;


        case MENU_START:

            OLED_ShowString(line, 2, "START");
            break;


        default:
            break;
    }
}


/*
 * 菜单分3页：
 * PAGE1：模式、S1圈数、方向、速度
 * PAGE2：S2圈数、方向、速度、等待时间
 * PAGE3：START + 两个CAN节点在线状态
 */
static void UI_DrawMenu(void)
{
    if (menu_index <= MENU_S1_SPEED)
    {
        UI_DrawMenuItem(1, MENU_MODE);
        UI_DrawMenuItem(2, MENU_S1_TURN);
        UI_DrawMenuItem(3, MENU_S1_DIR);
        UI_DrawMenuItem(4, MENU_S1_SPEED);
    }
    else if (menu_index <= MENU_DELAY)
    {
        UI_DrawMenuItem(1, MENU_S2_TURN);
        UI_DrawMenuItem(2, MENU_S2_DIR);
        UI_DrawMenuItem(3, MENU_S2_SPEED);
        UI_DrawMenuItem(4, MENU_DELAY);
    }
    else
    {
        UI_ClearLine(1);
        UI_ClearLine(2);
        UI_ClearLine(3);
        UI_ClearLine(4);

        OLED_ShowString(1, 1, ">START");
        OLED_ShowString(2, 1, "CAN1:");
        OLED_ShowString(3, 1, "CAN2:");
        OLED_ShowString(4, 1, "OK:RUN PAGE:SET");

        UI_UpdateMenuCan();
    }
}


static void UI_UpdateMenuCan(void)
{
    OLED_ShowString(2, 6, ui_runtime.slave1_online ? "ON " : "OFF");
    OLED_ShowString(3, 6, ui_runtime.slave2_online ? "ON " : "OFF");
}


/* direction：+1 = UP，-1 = DOWN */
static void UI_AdjustCurrentItem(int8_t direction)
{
    switch (menu_index)
    {
        case MENU_MODE:

            if (direction > 0)
            {
                if (ui_mode == 0)
                {
                    ui_mode = (App_Mode_t)(APP_MODE_COUNT - 1);
                }
                else
                {
                    ui_mode = (App_Mode_t)(ui_mode - 1);
                }
            }
            else
            {
                ui_mode = (App_Mode_t)(ui_mode + 1);

                if (ui_mode >= APP_MODE_COUNT)
                {
                    ui_mode = APP_MODE_S1_ONLY;
                }
            }

            break;


        case MENU_S1_TURN:

            if ((direction > 0) && (ui_s1_turns < UI_TURN_MAX)) ui_s1_turns++;
            if ((direction < 0) && (ui_s1_turns > UI_TURN_MIN)) ui_s1_turns--;
            break;


        case MENU_S1_DIR:

            ui_s1_direction = -ui_s1_direction;
            break;


        case MENU_S1_SPEED:

            if ((direction > 0) && (ui_s1_speed < UI_SPEED_MAX)) ui_s1_speed++;
            if ((direction < 0) && (ui_s1_speed > UI_SPEED_MIN)) ui_s1_speed--;
            break;


        case MENU_S2_TURN:

            if ((direction > 0) && (ui_s2_turns < UI_TURN_MAX)) ui_s2_turns++;
            if ((direction < 0) && (ui_s2_turns > UI_TURN_MIN)) ui_s2_turns--;
            break;


        case MENU_S2_DIR:

            ui_s2_direction = -ui_s2_direction;
            break;


        case MENU_S2_SPEED:

            if ((direction > 0) && (ui_s2_speed < UI_SPEED_MAX)) ui_s2_speed++;
            if ((direction < 0) && (ui_s2_speed > UI_SPEED_MIN)) ui_s2_speed--;
            break;


        case MENU_DELAY:

            if ((direction > 0) &&
                (ui_delay_ms <= (UI_DELAY_MAX_MS - UI_DELAY_STEP_MS)))
            {
                ui_delay_ms += UI_DELAY_STEP_MS;
            }

            if ((direction < 0) &&
                (ui_delay_ms >= (UI_DELAY_MIN_MS + UI_DELAY_STEP_MS)))
            {
                ui_delay_ms -= UI_DELAY_STEP_MS;
            }

            break;


        default:
            break;
    }
}


/* =========================================================
 *                       运行页面
 * ========================================================= */

/* 第1行标题：只在进页 / COMBO阶段变化时更新 */
static void UI_DrawRunTitle(void)
{
    UI_ClearLine(1);

    switch (ui_runtime.mode)
    {
        case APP_MODE_S1_ONLY: OLED_ShowString(1, 1, "S1 ONLY RUN"); break;
        case APP_MODE_S2_ONLY: OLED_ShowString(1, 1, "S2 ONLY RUN"); break;
        case APP_MODE_SYNC:    OLED_ShowString(1, 1, "SYNC RUN");    break;
        case APP_MODE_ASYNC:   OLED_ShowString(1, 1, "ASYNC RUN");   break;
        case APP_MODE_DELAY:   OLED_ShowString(1, 1, "DELAY RUN");   break;

        case APP_MODE_COMBO:

            if (ui_runtime.phase == APP_PHASE_WAIT)
            {
                OLED_ShowString(1, 1, "COMBO WAIT");
            }
            else if (ui_runtime.phase == APP_PHASE_SECONDARY)
            {
                OLED_ShowString(1, 1, "COMBO REV");
            }
            else
            {
                OLED_ShowString(1, 1, "COMBO FWD");
            }

            break;

        default:
            OLED_ShowString(1, 1, "RUN");
            break;
    }

    last_drawn_phase = ui_runtime.phase;
}


/*
 * 运行页只在：
 * 1. 刚进入RUN
 * 2. 用户按PAGE切页
 * 时完整画一次。
 * 之后每200ms只改数字。
 */
static void UI_DrawRunningFrame(void)
{
    UI_ClearLine(1);
    UI_ClearLine(2);
    UI_ClearLine(3);
    UI_ClearLine(4);

    if (run_page == 0)
    {
        UI_DrawRunTitle();

        if (ui_runtime.mode == APP_MODE_S1_ONLY)
        {
            OLED_ShowString(2, 1, "S1:00.0/00");
            OLED_ShowString(3, 1, "S2:IDLE");
            OLED_ShowNum(2, 9, UI_TargetCountToTurns(run_config.slave1.target_count), 2);
        }
        else if (ui_runtime.mode == APP_MODE_S2_ONLY)
        {
            OLED_ShowString(2, 1, "S1:IDLE");
            OLED_ShowString(3, 1, "S2:00.0/00");
            OLED_ShowNum(3, 9, UI_TargetCountToTurns(run_config.slave2.target_count), 2);
        }
        else
        {
            OLED_ShowString(2, 1, "S1:00.0/00");
            OLED_ShowString(3, 1, "S2:00.0/00");
            OLED_ShowNum(2, 9, UI_TargetCountToTurns(run_config.slave1.target_count), 2);
            OLED_ShowNum(3, 9, UI_TargetCountToTurns(run_config.slave2.target_count), 2);
        }

        OLED_ShowString(4, 1, "T:000.0 CAN:OK");
    }
    else
    {
        OLED_ShowString(1, 1, "RUN DETAIL");
        OLED_ShowString(2, 1, "V1:+000 V2:+000");
        OLED_ShowString(3, 1, "DIF:00000");
        OLED_ShowString(4, 1, "PAGE:MAIN");
    }

    UI_UpdateRunning();
}


/* 运行过程中只更新变化数字，不清屏、不整页重画 */
static void UI_UpdateRunning(void)
{
    uint32_t s1_turn_x10;
    uint32_t s2_turn_x10;
    uint32_t diff_count;
    uint32_t s1_abs;
    uint32_t s2_abs;

    if ((ui_runtime.mode == APP_MODE_COMBO) &&
        (ui_runtime.phase != last_drawn_phase) &&
        (run_page == 0))
    {
        UI_DrawRunTitle();
    }

    if (run_page == 0)
    {
        s1_turn_x10 = UI_CountToTurnX10(ui_runtime.slave1_status.total_count);
        s2_turn_x10 = UI_CountToTurnX10(ui_runtime.slave2_status.total_count);

        if (ui_runtime.mode != APP_MODE_S2_ONLY)
        {
            UI_ShowTurnX10(2, 4, s1_turn_x10);
        }

        if (ui_runtime.mode != APP_MODE_S1_ONLY)
        {
            UI_ShowTurnX10(3, 4, s2_turn_x10);
        }

        UI_ShowElapsed(4, 3);
    }
    else
    {
        OLED_ShowSignedNum(2, 4, (int32_t)ui_runtime.slave1_status.delta_count, 3);
        OLED_ShowSignedNum(2, 12, (int32_t)ui_runtime.slave2_status.delta_count, 3);

        s1_abs = UI_Abs32(ui_runtime.slave1_status.total_count);
        s2_abs = UI_Abs32(ui_runtime.slave2_status.total_count);

        if (s1_abs >= s2_abs)
        {
            diff_count = s1_abs - s2_abs;
        }
        else
        {
            diff_count = s2_abs - s1_abs;
        }

        if (diff_count > 99999U)
        {
            diff_count = 99999U;
        }

        OLED_ShowNum(3, 5, diff_count, 5);
    }
}


/* =========================================================
 *                       停止页面
 * ========================================================= */

static void UI_DrawStopping(void)
{
    UI_ClearLine(1);
    UI_ClearLine(2);
    UI_ClearLine(3);
    UI_ClearLine(4);

    OLED_ShowString(1, 1, "STOPPING");

    OLED_ShowString(2, 1, "S1:");
    UI_ShowTurnX10(2, 4, UI_CountToTurnX10(ui_runtime.slave1_status.total_count));

    OLED_ShowString(3, 1, "S2:");
    UI_ShowTurnX10(3, 4, UI_CountToTurnX10(ui_runtime.slave2_status.total_count));

    OLED_ShowString(4, 1, "T:000.0s");
    UI_ShowElapsed(4, 3);
}


/* =========================================================
 *                       完成页面
 * ========================================================= */

static void UI_DrawResult(void)
{
    UI_ClearLine(1);
    UI_ClearLine(2);
    UI_ClearLine(3);
    UI_ClearLine(4);

    OLED_ShowString(1, 1, "FINISHED");

    if (ui_runtime.mode == APP_MODE_S1_ONLY)
    {
        OLED_ShowString(2, 1, "S1:");
        UI_ShowTurnX10(2, 4, UI_CountToTurnX10(ui_runtime.slave1_status.total_count));
        OLED_ShowString(3, 1, "S2:IDLE");
    }
    else if (ui_runtime.mode == APP_MODE_S2_ONLY)
    {
        OLED_ShowString(2, 1, "S1:IDLE");
        OLED_ShowString(3, 1, "S2:");
        UI_ShowTurnX10(3, 4, UI_CountToTurnX10(ui_runtime.slave2_status.total_count));
    }
    else
    {
        OLED_ShowString(2, 1, "S1:");
        UI_ShowTurnX10(2, 4, UI_CountToTurnX10(ui_runtime.slave1_status.total_count));
        OLED_ShowString(3, 1, "S2:");
        UI_ShowTurnX10(3, 4, UI_CountToTurnX10(ui_runtime.slave2_status.total_count));
    }

    OLED_ShowString(4, 1, "T:000.0 OK:MENU");
    UI_ShowElapsed(4, 3);
}


/* =========================================================
 *                       故障页面
 * ========================================================= */

static void UI_DrawFault(void)
{
    UI_ClearLine(1);
    UI_ClearLine(2);
    UI_ClearLine(3);
    UI_ClearLine(4);

    OLED_ShowString(1, 1, "FAULT T:000.0");
    UI_ShowElapsed(1, 9);

    switch (ui_runtime.fault)
    {
        case APP_FAULT_SLAVE1_OFFLINE: OLED_ShowString(2, 1, "S1 OFFLINE");   break;
        case APP_FAULT_SLAVE2_OFFLINE: OLED_ShowString(2, 1, "S2 OFFLINE");   break;
        case APP_FAULT_SLAVE1_NO_ACK:  OLED_ShowString(2, 1, "S1 NO ACK");    break;
        case APP_FAULT_SLAVE2_NO_ACK:  OLED_ShowString(2, 1, "S2 NO ACK");    break;
        case APP_FAULT_SLAVE1:         OLED_ShowString(2, 1, "S1 FAULT");     break;
        case APP_FAULT_SLAVE2:         OLED_ShowString(2, 1, "S2 FAULT");     break;
        case APP_FAULT_CAN_TX:         OLED_ShowString(2, 1, "CAN TX ERROR"); break;
        default:                       OLED_ShowString(2, 1, "UNKNOWN");      break;
    }

    OLED_ShowString(3, 1, "C1:ON  C2:ON ");
    OLED_ShowString(4, 1, "OK:CLEAR");

    UI_UpdateFaultCan();
}


/* FAULT页只更新CAN那一行，不整页重画 */
static void UI_UpdateFaultCan(void)
{
    OLED_ShowString(3, 4, ui_runtime.slave1_online ? "ON " : "OFF");
    OLED_ShowString(3, 11, ui_runtime.slave2_online ? "ON " : "OFF");
}


/* =========================================================
 *                       启动任务
 * ========================================================= */

static void UI_Start(void)
{
    uint32_t now;

    UI_ApplyConfig();

    if (!App_SetConfig(&ui_config))
    {
        OLED_ShowString(4, 1, "CONFIG ERROR    ");
        return;
    }

    if (App_RequestStart() == APP_START_OK)
    {
        now = HAL_GetTick();

        /* 保存本轮真正使用的参数 */
        App_GetConfig(&run_config);

        run_start_ms = now;
        run_elapsed_ms = 0;
        run_timer_active = 1;

        run_page = 0;
        last_drawn_phase = APP_PHASE_NONE;

        ui_state = UI_STATE_RUNNING;

        last_refresh_tick = now;

        App_GetRuntime(&ui_runtime);
        UI_DrawRunningFrame();
    }
    else
    {
        /* 最常见就是CAN节点没全部在线。 */
        OLED_ShowString(4, 1, "START FAILED    ");
    }
}


/* =========================================================
 *                     返回主菜单
 * ========================================================= */

static void UI_EnterMenu(void)
{
    ui_state = UI_STATE_MENU;
    menu_index = MENU_MODE;
    edit_mode = 0;
    run_page = 0;

    UI_DrawMenu();
}


/* =========================================================
 *                       初始化
 * ========================================================= */

void StateMachine_Init(void)
{
    Key_Init();

    ui_mode = APP_MODE_SYNC;

    ui_s1_turns = 1;
    ui_s2_turns = 1;

    ui_s1_direction = 1;
    ui_s2_direction = 1;

    ui_s1_speed = 30;
    ui_s2_speed = 30;

    ui_delay_ms = 2000;

    ui_state = UI_STATE_MENU;
    menu_index = MENU_MODE;
    edit_mode = 0;
    run_page = 0;

    run_start_ms = 0;
    run_elapsed_ms = 0;
    run_timer_active = 0;

    last_key_scan_tick = HAL_GetTick();
    last_refresh_tick = HAL_GetTick();
    last_slow_refresh_tick = HAL_GetTick();

    App_GetRuntime(&ui_runtime);

    UI_ApplyConfig();
    App_SetConfig(&ui_config);

    UI_DrawMenu();
}


/* =========================================================
 *                       主状态机
 * ========================================================= */

void StateMachine_Task(void)
{
    uint32_t now;

    now = HAL_GetTick();

    /* ---------------- 按键扫描 ---------------- */
    if ((uint32_t)(now - last_key_scan_tick) >= UI_KEY_SCAN_PERIOD_MS)
    {
        last_key_scan_tick = now;
        Key_Update();
    }

    App_GetRuntime(&ui_runtime);

    /* 运行计时只在active时增加 */
    UI_UpdateTimer(now);


    /* =====================================================
     * 全局STOP
     * ===================================================== */
    if (Key_WasPressed(KEY_STOP))
    {
        if (ui_state == UI_STATE_RUNNING)
        {
            UI_FreezeTimer(now);
            App_RequestStop();

            ui_state = UI_STATE_STOPPING;
            UI_DrawStopping();
            return;
        }
    }


    /* =====================================================
     * 全局FAULT
     * ===================================================== */
    if ((ui_runtime.state == APP_STATE_FAULT) &&
        (ui_state != UI_STATE_FAULT))
    {
        UI_FreezeTimer(now);

        ui_state = UI_STATE_FAULT;
        last_slow_refresh_tick = now;

        UI_DrawFault();
        return;
    }


    /* =====================================================
     * MENU
     * ===================================================== */
    if (ui_state == UI_STATE_MENU)
    {
        /* 第3页CAN状态只改两处文字，不整页重画 */
        if ((menu_index == MENU_START) &&
            ((uint32_t)(now - last_slow_refresh_tick) >= UI_SLOW_REFRESH_MS))
        {
            last_slow_refresh_tick = now;
            UI_UpdateMenuCan();
        }


        /* PAGE：在三页菜单之间快速跳转 */
        if (Key_WasPressed(KEY_PAGE))
        {
            if (edit_mode == 0)
            {
                if (menu_index <= MENU_S1_SPEED)
                {
                    menu_index = MENU_S2_TURN;
                }
                else if (menu_index <= MENU_DELAY)
                {
                    menu_index = MENU_START;
                }
                else
                {
                    menu_index = MENU_MODE;
                }

                UI_DrawMenu();
            }

            return;
        }


        /* BACK：编辑时退出编辑 */
        if (Key_WasPressed(KEY_BACK))
        {
            if (edit_mode)
            {
                edit_mode = 0;
                UI_DrawMenu();
            }

            return;
        }


        /* OK：参数进入/退出编辑；START则运行 */
        if (Key_WasPressed(KEY_OK))
        {
            if (menu_index == MENU_START)
            {
                UI_Start();
            }
            else
            {
                edit_mode = !edit_mode;
                UI_DrawMenu();
            }

            return;
        }


        /* UP：编辑时加参数；否则移动光标 */
        if (Key_WasPressed(KEY_UP))
        {
            if (edit_mode)
            {
                UI_AdjustCurrentItem(+1);
            }
            else
            {
                if (menu_index == 0)
                {
                    menu_index = MENU_ITEM_COUNT - 1;
                }
                else
                {
                    menu_index--;
                }
            }

            UI_DrawMenu();
            return;
        }


        /* DOWN：编辑时减参数；否则移动光标 */
        if (Key_WasPressed(KEY_DOWN))
        {
            if (edit_mode)
            {
                UI_AdjustCurrentItem(-1);
            }
            else
            {
                menu_index++;

                if (menu_index >= MENU_ITEM_COUNT)
                {
                    menu_index = 0;
                }
            }

            UI_DrawMenu();
            return;
        }


        return;
    }


    /* =====================================================
     * RUNNING
     * ===================================================== */
    if (ui_state == UI_STATE_RUNNING)
    {
        /* 完成：先冻结时间，再画最终页 */
        if (ui_runtime.state == APP_STATE_FINISHED)
        {
            UI_FreezeTimer(now);

            ui_state = UI_STATE_RESULT;
            UI_DrawResult();
            return;
        }


        /* PAGE：MAIN / DETAIL切换；只在切页这一刻完整画一次 */
        if (Key_WasPressed(KEY_PAGE))
        {
            run_page = !run_page;
            UI_DrawRunningFrame();
            return;
        }


        /* BACK：人工停止 */
        if (Key_WasPressed(KEY_BACK))
        {
            UI_FreezeTimer(now);
            App_RequestStop();

            ui_state = UI_STATE_STOPPING;
            UI_DrawStopping();
            return;
        }


        /* 只更新变化数据，不再整页刷新 */
        if ((uint32_t)(now - last_refresh_tick) >= UI_REFRESH_PERIOD_MS)
        {
            last_refresh_tick = now;
            UI_UpdateRunning();
        }

        return;
    }


    /* =====================================================
     * STOPPING
     * ===================================================== */
    if (ui_state == UI_STATE_STOPPING)
    {
        /* STOP页无需不断刷新；时间已经冻结。 */
        if (ui_runtime.state == APP_STATE_IDLE)
        {
            UI_EnterMenu();
        }

        return;
    }


    /* =====================================================
     * RESULT
     * ===================================================== */
    if (ui_state == UI_STATE_RESULT)
    {
        if (Key_WasPressed(KEY_OK) || Key_WasPressed(KEY_BACK))
        {
            App_AcknowledgeFinished();
            UI_EnterMenu();
        }

        return;
    }


    /* =====================================================
     * FAULT
     * ===================================================== */
    if (ui_state == UI_STATE_FAULT)
    {
        /* 只更新CAN在线文字，不整页重画 */
        if ((uint32_t)(now - last_slow_refresh_tick) >= UI_SLOW_REFRESH_MS)
        {
            last_slow_refresh_tick = now;
            UI_UpdateFaultCan();
        }

        if (Key_WasPressed(KEY_OK))
        {
            App_ClearFault();
            UI_EnterMenu();
        }

        return;
    }
}
