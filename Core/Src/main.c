/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 海泰 HT-J-2E(配 ZE300) 位置控制 —— 无串口 / LinkScope 调试版
  *                   平台: 大疆 RM 开发板A (STM32F427IIH) + CAN1(1MHz)
  *
  *   特点:
  *   - 不依赖串口, 上电即跑; 用 LinkScope 看下面的全局变量绘图
  *   - 异步状态机(不阻塞): 每 AUTO_INTERVAL_MS 给一个随机绝对位置(0xDA 梯形)
  *   - 抱闸时序: 先发"当前位置"保持 -> 松闸(0xCE=0) -> 0xDA 到位 -> 合闸(0xCE=1)
  *   - 零点: 第一次上电把当前位置记成 0 点(0xB1, 驱动断电保存)+STM32 存标记;
  *           之后每次上电自动回 0 点
  *
  *   LinkScope 可看的变量(全局, 都在本文件底部/顶部):
  *   g_tgt_cnt / g_tgt_deg   目标位置
  *   g_now_cnt / g_now_deg   实际位置(0xA3 回读)
  *   g_spd_rpm               实际速度
  *   g_st                    状态: 0空闲 1送保持 2松闸 3运动 4到位
  *   g_brk                   抱闸: 1闭合 0松开
  *   g_auto                  自动随机开关
  *   g_run_ms / g_rx_cnt / g_move_cycles / g_move_timeouts
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "gpio.h"
#include "drv_can.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ZE300_ADDR              (2u)   /* 设备地址(CAN ID) */

/* 第一次上电"把当前位置设成0点"只在无标记时执行; 之后每次上电回0点 */
/* HT6010-J36-2E: 额定电流8.8A, 峰值30.17A, 减速36:1, 输出最高~93rpm */
#define ACCEL_RPMPS             (150)  /* 梯形加速度 rpm/s (0xD0) —— 太大起停会震, 150起步 */
#define DECEL_RPMPS             (150)  /* 梯形减速度 rpm/s (0xD1) */
#define POS_MAX_RPM             (90)   /* 位置模式最大速度 rpm (0xB2); 输出上限就~93, 设再大没用 */
#define IQ_LIMIT_A              (8.8f) /* 最大Q轴电流A (0xB3): 用额定8.8A, 别长期超; 短时想更快可到12 */

/* 抱闸时序 */
#define ENGAGE_HOLD_MS          (60)   /* 松闸前"保持当前位置"时长 */
#define BRAKE_RELEASE_MS        (200)  /* 松闸到发运动指令之间的响应 */
#define BRAKE_ENGAGE_MS         (80)   /* 到位后合闸(仅计数用, 无阻塞) */

/* 位置 / 到位判定 */
#define SETTLE_TOL_CNT          (16)   /* 16384/圈, ±16counts≈±0.35° */
#define ARRIVE_MAX_MS           (30000)/* 有反馈时的到位最长时间 */
#define NO_ACK_TIMEOUT_MS       (3000) /* 发目标后一直无任何回包 -> 判定没接电机, 跳走 */

/* 自动随机 */
#define ENABLE_AUTO_RANDOM      (0)    /* 0=不做自动随机, 目标由 LinkScope 变量 pos_cmd_deg 给 */
#define AUTO_INTERVAL_MS        (2000)
#define AUTO_START_DELAY_MS     (1500)
#define AUTO_RANGE_DEG          (180)
#define HOLD_ZERO_MS            (5000) /* 上电回零到位后, 先在0点保持5s再开始随机 */

/* 主循环节拍 */
#define LOOP_PERIOD_MS          (5)
#define POS_POLL_MS             (20)   /* 定期 0xA3 读位置 */

/* 首次零点标记(STM32 flash 扇区4) */
#define PERSIST_ADDR            (0x08010000u)
#define PERSIST_MAGIC           (0x5A5A5A01u)

/* ====== 在线调 PID(用 LinkScope 写下面的变量即可) ======
 * 0xB6 位置环Kp  0xB7 位置环Ki  0xB8 速度环Kp  0xB9 速度环Ki (float, 断电不保存) */
#define PID_POLL_MS             (300)   /* PID 下发/比较周期 */
#define RD_PERIOD_MS            (100)   /* 参数回读: 每100ms只发一条, 一条一条轮询 */

/* USER CODE END PD */

/* ============================================================
 * LinkScope 变量(全局符号)
 * 中断(ISR)写的加 volatile; 其余主循环每拍刷新
 * ============================================================ */
volatile int32_t  g_now_cnt  = 0;      /* 实际位置 counts(0xA3, ISR 写) */
volatile int32_t  g_spd_x100 = 0;      /* 实际速度 0.01rpm(ISR 写) */
volatile uint32_t g_rx_cnt   = 0;      /* 收到的 CAN 帧数(ISR 写) */

volatile int32_t  g_local_tgt  = 0;    /* 目标相对零点的局部counts(限±8192) */
volatile int32_t  g_tgt_cnt   = 0;     /* 实际下发给驱动的绝对目标 counts */
volatile float    g_tgt_deg   = 0.0f;  /* 当前目标 度(相对零点, 限±180) */
volatile float    g_now_deg   = 0.0f;  /* 实际位置 度 */
volatile float    g_spd_rpm   = 0.0f;  /* 实际速度 rpm */
volatile int32_t  g_st        = 0;     /* 状态机状态 */
volatile uint8_t  g_brk       = 1;     /* 抱闸: 1=闭合 0=松开 (逻辑用原始值) */
volatile int32_t  g_brk_plot  = 50;    /* 给LinkScope画的抱闸: 合=50 松=0, 台阶明显 */
volatile uint8_t  g_auto      = ENABLE_AUTO_RANDOM;
volatile uint32_t g_run_ms    = 0;     /* 开机后运行毫秒 */
volatile uint32_t g_move_cycles   = 0; /* 完成/超时次数 */
volatile uint32_t g_move_timeouts = 0; /* 无回包超时次数 */

volatile int32_t  g_base_cnt     = 0;   /* 零点基准 count(开机回零到位后锁定) */
volatile float    g_peak_rpm     = 0.0f; /* 运行以来实测的最大|速度| rpm(判断提速有没有生效) */
volatile int32_t  g_cfg_acc_x100 = 0;   /* 回读 0xD0 加速度(x0.01rpm/s) */
volatile int32_t  g_cfg_dec_x100 = 0;   /* 回读 0xD1 减速度 */
volatile int32_t  g_cfg_vmax_x100= 0;   /* 回读 0xB2 位置最大速度(x0.01rpm) */
volatile int32_t  g_cfg_iq_ma    = 0;   /* 回读 0xB3 最大电流(mA) */

/* ====== 在线调 PID: LinkScope 里改这些变量, 程序自动写入驱动 ======
 * 驱动这一组"运动控制参数"共6个:
 *  pos_kp(B6)/pos_ki(B7)/pos_out_limit(B2,单位rpm)/vel_kp(B8)/vel_ki(B9)/vel_out_limit(B3,单位A) */
volatile float pid_pos_kp = 0.0f;      /* 目标 位置环Kp(0xB6) */
volatile float pid_pos_ki = 0.0f;      /* 目标 位置环Ki(0xB7) */
volatile float pid_pos_lim_rpm = (float)POS_MAX_RPM; /* 目标 位置环输出限=位置最大速度rpm(0xB2) */
volatile float pid_vel_kp = 0.0f;      /* 目标 速度环Kp(0xB8) */
volatile float pid_vel_ki = 0.0f;      /* 目标 速度环Ki(0xB9) */
volatile float pid_vel_lim_A = IQ_LIMIT_A; /* 目标 速度环输出限=最大电流A(0xB3) */

/* ====== 手动给位置: LinkScope 里写 pos_cmd_deg(度, 相对零点), 电机就走过去 ====== */
volatile float pos_cmd_deg = 0.0f;

/* ====== 只读: 驱动当前实际值(每次回读实时刷新, 用于反馈确认) ====== */
volatile float cur_pos_kp = 0.0f;
volatile float cur_pos_ki = 0.0f;
volatile float cur_pos_lim_rpm = 0.0f;
volatile float cur_vel_kp = 0.0f;
volatile float cur_vel_ki = 0.0f;
volatile float cur_vel_lim_A = 0.0f;

/* 私有 */
static uint8_t g_first_hold = 1;       /* 第一次(开机回零)到位后多保持 HOLD_ZERO_MS */
static uint8_t  g_rd_idx = 0;          /* 参数轮询指针 */
static uint32_t g_last_rd_ms = 0;      /* 参数管理器时间 */
static float    g_last_cmd_deg = 0.0f; /* 上一次已执行的目标角度(去抖用) */
/* 驱动回读的 PID 现值(ISR 写) */
static volatile float rb_pkp = 0.0f, rb_pki = 0.0f, rb_vkp = 0.0f, rb_vki = 0.0f;
static uint8_t f_pkp = 0, f_pki = 0, f_vkp = 0, f_vki = 0;   /* 是否已收到各自回读 */
static uint8_t c_pkp = 0, c_pki = 0, c_vkp = 0, c_vki = 0;   /* 是否已拷给 pid_* */
static uint8_t f_d0 = 0, f_d1 = 0, f_b2 = 0, f_b3 = 0;       /* D0/D1/B2/B3 是否已读到过 */
static uint8_t pid_loaded = 0;         /* 已把驱动现值复制给 pid_* (可安全开始写) */

/* ---------- 私有 ---------- */
static uint32_t g_auto_next_ms = 0;    /* 下次自动给位置的时刻 */
static uint32_t g_last_ack_cnt = 0;    /* 本次运动开始时已收到的回包数 */
static uint32_t g_st_t0 = 0;           /* 进入当前状态的时刻 */
static uint32_t g_last_poll_ms = 0;

static uint32_t g_rng = 0x9E3779B9u;   /* 伪随机种子 */

static uint8_t  g_ba_addr = 0;         /* 0xBA 回包里的地址 */
static uint8_t  g_ba_ok = 0;

/* ---------------- 伪随机 ---------------- */
static uint32_t rng_next(void)
{
    g_rng = g_rng * 1664525u + 1013904223u;
    return g_rng;
}
static int32_t rng_in_range(int32_t amp)
{
    return (int32_t)(rng_next() % (uint32_t)(2u * (uint32_t)amp + 1u)) - amp;
}

/* ---------------- CAN/协议小工具 ---------------- */
static void can_send(uint16_t sid, const uint8_t *d, uint8_t len)
{
    CAN_Send_Data(&hcan1, sid, (uint8_t *)d, len);
}

static void put_i32le(uint8_t *p, int32_t v)
{
    p[0]=(uint8_t)(v & 0xFF); p[1]=(uint8_t)((v>>8)&0xFF);
    p[2]=(uint8_t)((v>>16)&0xFF); p[3]=(uint8_t)((v>>24)&0xFF);
}
static int32_t get_i32le(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1]<<8) |
                     ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24));
}
static float get_f32le(const uint8_t *p)
{
    uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1]<<8) |
                 ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
    union { uint32_t u; float f; } c;
    c.u = u;
    return c.f;
}
static void put_f32le(uint8_t *p, float v)
{
    union { uint32_t u; float f; } c;
    c.f = v;
    p[0] = (uint8_t)(c.u & 0xFF); p[1] = (uint8_t)((c.u>>8)&0xFF);
    p[2] = (uint8_t)((c.u>>16)&0xFF); p[3] = (uint8_t)((c.u>>24)&0xFF);
}

/* 0xBA 设地址 / 0x00 重启 */
static void ze300_set_addr(uint8_t a)
{
    uint8_t d[2] = {0xBA, a};
    can_send(0xFF, d, 2);
}
static void ze300_restart(void)
{
    uint8_t d[8] = {0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF};
    can_send(0xFF, d, 8);
}
/* 0xB1 设当前位置为原点(断电保存) */
static void ze300_set_origin(void)
{
    uint8_t d[1] = {0xB1};
    can_send(ZE300_ADDR, d, 1);
}
/* 0xDA 梯形绝对位置 */
static void ze300_target_cnt(int32_t cnt)
{
    uint8_t d[6] = {0xDA, 0x00, 0, 0, 0, 0};
    put_i32le(&d[2], cnt);
    can_send(ZE300_ADDR, d, 6);
}
/* 0xA3 读当前位置 */
static void ze300_read_pos(void)
{
    uint8_t d[1] = {0xA3};
    can_send(ZE300_ADDR, d, 1);
}
/* 0xCE 抱闸 */
static void brake(uint8_t close)
{
    uint8_t d[2];
    d[0] = 0xCE;
    d[1] = close ? 0x01u : 0x00u;
    can_send(ZE300_ADDR, d, 2);
    g_brk = close;
}
/* 0xD0/0xD1 梯形加/减速度 */
static void trap_acc(uint8_t cmd, int32_t v_x100)
{
    uint8_t d[5] = {cmd, 0, 0, 0, 0};
    put_i32le(&d[1], v_x100);
    can_send(ZE300_ADDR, d, 5);
}
/* 0xB2 位置模式最大速度(0.01rpm) */
static void pos_max_speed_rpm(int32_t rpm)
{
    uint8_t d[5] = {0xB2, 0, 0, 0, 0};
    put_i32le(&d[1], rpm * 100);
    can_send(ZE300_ADDR, d, 5);
}
/* 0xB3 位置/速度模式最大Q轴电流(0.001A) */
static void pos_iq_limit_ma(uint32_t ma)
{
    uint8_t d[5] = {0xB3, 0, 0, 0, 0};
    put_i32le(&d[1], (int32_t)ma);
    can_send(ZE300_ADDR, d, 5);
}
/* 在线写 PID: cmd = 0xB6/0xB7/0xB8/0xB9, 值为 float */
static void pid_write(uint8_t cmd, float v)
{
    uint8_t d[5] = {cmd, 0, 0, 0, 0};
    put_f32le(&d[1], v);
    can_send(ZE300_ADDR, d, 5);
}

/* ---------------- flash 首次零点标记 ---------------- */
static int persist_get(void)
{
    return (*(volatile const uint32_t *)PERSIST_ADDR) == PERSIST_MAGIC;
}
static int persist_set(void)
{
    FLASH_EraseInitTypeDef er;
    uint32_t err = 0;
    HAL_FLASH_Unlock();
    er.TypeErase = FLASH_TYPEERASE_SECTORS;
    er.Sector = FLASH_SECTOR_4;
    er.NbSectors = 1;
    er.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    if (HAL_FLASHEx_Erase(&er, &err) != HAL_OK) { HAL_FLASH_Lock(); return 0; }
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PERSIST_ADDR, PERSIST_MAGIC) != HAL_OK) { HAL_FLASH_Lock(); return 0; }
    HAL_FLASH_Lock();
    return 1;
}

/* ---------------- 异步运动状态机 ---------------- */
/* 把 count 归一到 ±半圈(±180°)内. 多圈 count 会越界走动, 显示/比较都用它,
 * 这样"角度"永远只在 ±180 里, 不会被多圈累积带飞到几百 */
static int32_t norm180(int32_t cnt)
{
    int32_t c = cnt % 16384;
    if (c > 8191)      c -= 16384;
    else if (c < -8191) c += 16384;
    return c;
}

/* 当前实际位置 相对零点(基准) 的局部角 counts, 永远限在 ±半圈内 */
static int32_t local_now_cnt(void)
{
    return norm180(g_now_cnt - g_base_cnt);
}

/* 启动一次运动: 目标 = 相对零点的局部角 local_cnt(±半圈内, 自动走最短方向).
 * 关键: 命令总是"当前 count + 小增量"(不去追多圈层), 到位判断也用局部角.
 * 这样即使驱动多圈 count 越界走动, 目标/实际都永远显示 ±180, 不会发散. */
static void start_move(int32_t local_cnt)
{
    int32_t nowl, delta, eff;
    if (g_st != 0) return;               /* 忙就忽略 */

    nowl  = local_now_cnt();                     /* 当前局部角 */
    delta = norm180(local_cnt - nowl);           /* 需要的局部增量(自动取短方向) */
    eff   = g_now_cnt + delta;                   /* 绝对命令 = 当前count + 小增量 */

    g_local_tgt = local_cnt;
    g_tgt_cnt = eff;
    g_tgt_deg = (float)norm180(local_cnt) * 360.0f / 16384.0f;  /* 显示目标(限±180) */
    g_last_ack_cnt = g_rx_cnt;           /* 记录起点回包数, 用于"没接电机"判定 */
    g_st = 1;                            /* 送当前位置保持 */
    g_st_t0 = g_run_ms;
    ze300_target_cnt(g_now_cnt);         /* 先发当前位置让驱动建保持, 抵消松闸下坠 */
}

/* 开机专用回零(阻塞一次): 送当前位置保持 -> 松闸 -> 0xC4 最短回原点 -> 等静止 -> 合闸
 * 注意: 只有上电这一次, 之后都走上面的异步状态机 */
static void home_boot(void)
{
    uint8_t d;
    uint32_t t0, last_chg;
    int32_t prev;

    ze300_target_cnt(g_now_cnt);         /* 让驱动先建立保持, 防松闸瞬间下坠 */
    HAL_Delay(ENGAGE_HOLD_MS);
    brake(0);
    HAL_Delay(BRAKE_RELEASE_MS);

    d = 0xC4;                            /* 最短距离回原点(≤180°) */
    can_send(ZE300_ADDR, &d, 1);

    /* 位置连续不变约400ms -> 认为到位; 一直无回包(没接电机)1.2s退出 */
    prev = g_now_cnt;
    t0 = HAL_GetTick();
    last_chg = t0;
    while (1)
    {
        uint8_t a = 0xA3;
        if (HAL_GetTick() - t0 > 2500) break;
        can_send(ZE300_ADDR, &a, 1);
        HAL_Delay(20);
        if (g_now_cnt != prev)
        {
            prev = g_now_cnt;
            last_chg = HAL_GetTick();
        }
        else if (HAL_GetTick() - last_chg >= 400)
        {
            break;                       /* 静止到位 */
        }
        if (g_rx_cnt == 0 && HAL_GetTick() - t0 > 1200) break;
    }
    brake(1);
}

static void st_machine(void)
{
    uint32_t t = g_run_ms;

    switch (g_st)
    {
    case 0:                             /* 空闲 */
        if (g_auto && (int32_t)(t - g_auto_next_ms) >= 0)
        {
            int32_t local = rng_in_range(AUTO_RANGE_DEG) * 16384 / 360; /* 相对零点随机(度) */
            start_move(local);                /* 只给局部角, 由 start_move 转成小增量命令 */
        }
        break;

    case 1:                             /* 送保持, 等一小会 */
        if (t - g_st_t0 >= ENGAGE_HOLD_MS)
        {
            brake(0);                    /* 松闸 */
            g_st = 2;
            g_st_t0 = t;
        }
        break;

    case 2:                             /* 松闸响应中 */
        if (t - g_st_t0 >= BRAKE_RELEASE_MS)
        {
            ze300_target_cnt(g_tgt_cnt); /* 真正下发目标 */
            g_st = 3;
            g_st_t0 = t;
        }
        break;

    case 3:                             /* 运动中 */
    {
        int32_t err = local_now_cnt() - g_local_tgt;   /* 用局部角判到位, 不追多圈层 */
        if (err > -SETTLE_TOL_CNT && err < SETTLE_TOL_CNT)
        {
            g_st = 4;
            g_st_t0 = t;
        }
        else if (g_rx_cnt == g_last_ack_cnt && (t - g_st_t0) > NO_ACK_TIMEOUT_MS)
        {
            /* 发目标后一直无任何回包: 认为没接电机, 不等了 */
            g_st = 4;
            g_st_t0 = t;
            g_move_timeouts++;
        }
        else if (g_rx_cnt != g_last_ack_cnt && (t - g_st_t0) > ARRIVE_MAX_MS)
        {
            g_st = 4;                    /* 有反馈但一直没到位, 兜底 */
            g_st_t0 = t;
            g_move_timeouts++;
        }
        break;
    }

    case 4:                             /* 到位/兜底 -> 合闸锁死 */
        if (t - g_st_t0 >= BRAKE_ENGAGE_MS)
        {
            uint32_t hold = g_first_hold ? (uint32_t)HOLD_ZERO_MS : (uint32_t)AUTO_INTERVAL_MS;
            g_first_hold = 0;           /* 只有开机那次回零要多保持5s */
            brake(1);
            g_st = 0;
            g_move_cycles++;
            g_auto_next_ms = t + hold;  /* 到位后再等 hold 才给下一个随机位置 */
        }
        break;
    }
}

/* ---------------- CAN RX 回调 ---------------- */
void CAN_Motor_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    const uint8_t *d = Rx_Buffer->Data;
    g_rx_cnt++;

    if (d[0] == 0xBA)
    {
        g_ba_addr = d[1];
        g_ba_ok = 1;
    }
    else if (d[0] == 0xB2) { g_cfg_vmax_x100 = get_i32le(&d[1]); f_b2 = 1; }
    else if (d[0] == 0xD0) { g_cfg_acc_x100  = get_i32le(&d[1]); f_d0 = 1; }
    else if (d[0] == 0xD1) { g_cfg_dec_x100  = get_i32le(&d[1]); f_d1 = 1; }
    else if (d[0] == 0xB3) { g_cfg_iq_ma     = get_i32le(&d[1]); f_b3 = 1; }
    else if (d[0] == 0xB6) { rb_pkp = get_f32le(&d[1]); f_pkp = 1; cur_pos_kp = rb_pkp; }
    else if (d[0] == 0xB7) { rb_pki = get_f32le(&d[1]); f_pki = 1; cur_pos_ki = rb_pki; }
    else if (d[0] == 0xB8) { rb_vkp = get_f32le(&d[1]); f_vkp = 1; cur_vel_kp = rb_vkp; }
    else if (d[0] == 0xB9) { rb_vki = get_f32le(&d[1]); f_vki = 1; cur_vel_ki = rb_vki; }
    else if (d[0] == 0xA3 || d[0] == 0xC2 || d[0] == 0xC3 || d[0] == 0xC4 ||
             d[0] == 0xDA || d[0] == 0xDC)
    {
        g_now_cnt = get_i32le(&d[3]);   /* 与 0xA3 相同, 多圈绝对值在[3..6] */
    }
    else if (d[0] == 0xA2 || d[0] == 0xC1)
    {
        g_spd_x100 = get_i32le(&d[1]);
    }
}

/**
  * @brief  The application entry point.
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CAN1_Init();                     /* CAN1 = 1MHz, PD0/PD1 */

    CAN_Init(&hcan1, CAN_Motor_Call_Back);
    g_run_ms = 0;

    /* ---- 1. 设备地址自动配置(发公共地址0xFF) ---- */
    {
        uint8_t need_set = 0;
        uint8_t d_ba[1] = {0xBA};
        uint32_t t0;
        g_ba_ok = 0;
        can_send(0xFF, d_ba, 1);
        t0 = HAL_GetTick();
        while (!g_ba_ok && (HAL_GetTick() - t0) < 300) HAL_Delay(2);
        if (!g_ba_ok) need_set = 1;
        else if (g_ba_addr != ZE300_ADDR) need_set = 1;
        if (need_set)
        {
            ze300_set_addr(ZE300_ADDR);
            HAL_Delay(150);
            ze300_restart();
            HAL_Delay(3500);            /* 等驱动重启 */
        }
    }

    /* ---- 2. 梯形加减速 ----
     * 注意: 重物没接/没测过前先别放大; 也可用官方上位机设 0xB3 限流 */
    trap_acc(0xD0, (int32_t)ACCEL_RPMPS * 100);   /* 梯形加速度 */
    trap_acc(0xD1, (int32_t)DECEL_RPMPS * 100);   /* 梯形减速度 */
    pos_max_speed_rpm(POS_MAX_RPM);               /* 位置最大速度上限(防被默认值压住) */
    pos_iq_limit_ma((uint32_t)(IQ_LIMIT_A * 1000.0f)); /* 最大电流限制 */

    /* ---- 3. 零点 / 回零 / 基准锁定 ----
     * 第一次: 当前位置=0点(0xB1, 驱动断电保存)+STM32标记;
     * 以后:   每次上电 0xC4 回原点;
     * 到位后把该点 count 记为 g_base_cnt: 之后所有位置都是"基准+相对量",
     * 多圈count不会越滚越大, 回0永远走小圈 */
    if (!persist_get())
    {
        uint8_t a;
        ze300_set_origin();             /* 0xB1: 当前位置设为原点 */
        persist_set();
        a = 0xA3; can_send(ZE300_ADDR, &a, 1);  /* 读一次位置当基准 */
        HAL_Delay(150);
        g_base_cnt = g_now_cnt;
    }
    else
    {
        home_boot();                    /* 真正回零并等静止 */
        g_base_cnt = g_now_cnt;         /* 锁定零点基准 */
    }
    g_first_hold = 0;                   /* 保持5s已在上面完成, 之后随机按AUTO_INTERVAL */

    /* 种子, 让每次上电随机序列不同 */
    g_rng = HAL_GetTick();

    g_auto_next_ms = g_run_ms + HOLD_ZERO_MS;   /* 回零后在0点保持5s再开始随机 */

    /* ---- 4. 主循环(纯变量, 无串口) ---- */
    while (1)
    {
        uint32_t now = HAL_GetTick();
        g_run_ms = now;

        /* 派生量: 度 / rpm, 供 LinkScope 看更直观 (角度相对零点并归一化±180, 不会越滚越大) */
        g_now_deg = (float)norm180(g_now_cnt - g_base_cnt) * 360.0f / 16384.0f;
        g_spd_rpm = (float)g_spd_x100 * 0.01f;
        if (g_spd_rpm < 0.0f) { if (-g_spd_rpm > g_peak_rpm) g_peak_rpm = -g_spd_rpm; }
        else                 { if ( g_spd_rpm > g_peak_rpm) g_peak_rpm =  g_spd_rpm; }
        g_brk_plot = g_brk ? 50 : 0;    /* 抱闸绘图: 合=50 松=0, 台阶明显 */

        /* === 参数管理器: 每 100ms 只做一件事 ===
         * 优先: 找一条“与目标不一致”的参数写下去;
         * 否则: 读下一条参数(不能连发, 连发驱动只回第一条) */
        if (now - g_last_rd_ms >= RD_PERIOD_MS)
        {
            static const uint8_t rd_cmds[8] =
                {0xD0, 0xD1, 0xB2, 0xB3, 0xB6, 0xB7, 0xB8, 0xB9};
            uint8_t i, sent = 0;
            g_last_rd_ms = now;

            /* 刷新只读 + 第一次把驱动原值灌进 pid_* */
            cur_pos_lim_rpm = (float)g_cfg_vmax_x100 * 0.01f;
            cur_vel_lim_A   = (float)g_cfg_iq_ma    * 0.001f;

            if (!pid_loaded)
            {
                if (f_pkp) { pid_pos_kp = rb_pkp; f_pkp = 0; c_pkp = 1; }
                if (f_pki) { pid_pos_ki = rb_pki; f_pki = 0; c_pki = 1; }
                if (f_vkp) { pid_vel_kp = rb_vkp; f_vkp = 0; c_vkp = 1; }
                if (f_vki) { pid_vel_ki = rb_vki; f_vki = 0; c_vki = 1; }
                if (c_pkp && c_pki && c_vkp && c_vki) pid_loaded = 1;
            }

            /* 扫描第一个需要写的参数(一次只发一条) */
            for (i = 0; i < 8 && !sent; i++)
            {
                switch (i)
                {
                case 0: /* 梯形加速度 D0 */
                    if (f_d0 && g_cfg_acc_x100 != (int32_t)ACCEL_RPMPS * 100) { trap_acc(0xD0, (int32_t)ACCEL_RPMPS * 100); sent = 1; }
                    break;
                case 1: /* 梯形减速度 D1 */
                    if (f_d1 && g_cfg_dec_x100 != (int32_t)DECEL_RPMPS * 100) { trap_acc(0xD1, (int32_t)DECEL_RPMPS * 100); sent = 1; }
                    break;
                case 2: /* 位置最大速度 B2 */
                    if (f_b2 && (cur_pos_lim_rpm < pid_pos_lim_rpm - 0.5f || cur_pos_lim_rpm > pid_pos_lim_rpm + 0.5f))
                    { pos_max_speed_rpm((int32_t)pid_pos_lim_rpm); sent = 1; }
                    break;
                case 3: /* 最大电流 B3 */
                    if (f_b3 && (cur_vel_lim_A < pid_vel_lim_A - 0.05f || cur_vel_lim_A > pid_vel_lim_A + 0.05f))
                    { pos_iq_limit_ma((uint32_t)(pid_vel_lim_A * 1000.0f)); sent = 1; }
                    break;
                case 4:
                    if (pid_loaded && pid_pos_kp != rb_pkp) { pid_write(0xB6, pid_pos_kp); sent = 1; }
                    break;
                case 5:
                    if (pid_loaded && pid_pos_ki != rb_pki) { pid_write(0xB7, pid_pos_ki); sent = 1; }
                    break;
                case 6:
                    if (pid_loaded && pid_vel_kp != rb_vkp) { pid_write(0xB8, pid_vel_kp); sent = 1; }
                    break;
                case 7:
                    if (pid_loaded && pid_vel_ki != rb_vki) { pid_write(0xB9, pid_vel_ki); sent = 1; }
                    break;
                }
            }

            if (!sent)   /* 没有要写的, 就读下一条(轮询) */
            {
                uint8_t c = rd_cmds[g_rd_idx];
                g_rd_idx = (g_rd_idx + 1) % 8;
                can_send(ZE300_ADDR, &c, 1);
            }
        }

        /* 定期读一次实际位置 */
        if (now - g_last_poll_ms >= POS_POLL_MS)
        {
            g_last_poll_ms = now;
            ze300_read_pos();
        }

        st_machine();

        /* ===== 手动目标: 改 pos_cmd_deg 就去(度, 相对零点; 空闲时执行) ===== */
        if (pos_cmd_deg != g_last_cmd_deg && g_st == 0)
        {
            g_last_cmd_deg = pos_cmd_deg;   /* 记下本次, 忙碌时下一次空闲再补跑 */
            g_auto = 0;                     /* 手动接管, 关随机 */
            start_move((int32_t)(pos_cmd_deg * 16384.0f / 360.0f));
        }

        HAL_Delay(LOOP_PERIOD_MS);
    }
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  if (HAL_PWREx_EnableOverDrive() != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
