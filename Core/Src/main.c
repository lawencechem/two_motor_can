/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 海泰 HT-J-2E(配 ZE300) 双电机位置控制 —— LinkScope 调参 + USART2 回传/指令
  *                   平台: 大疆 RM 开发板A (STM32F427IIH) + CAN1(1MHz)
  *
  *   特点(双电机, CAN1 总线, 设备地址 1 / 2 均已设好, 不再自动配地址):
  *   - LinkScope(SWD)看/改下面的全局变量; USART2(1M) ~50Hz 发 VOFA+ JustFloat 波形 + ASCII 指令控制
  *   - 两台电机完全独立: 各自有目标位置/回读/抱闸/状态机/参数管理器/在线 PID
  *   - 上电: 每台首次(本固件)设原点(0xB1), 之后每次 0xC4 回各自原点
  *   - 收包按回包 CAN StdId(=设备自身地址 1/2) 分拣到对应电机
  *   - 串口下行指令: m1 <deg> / m2 <deg> / stop(在发送区回车)
  *
  *   LinkScope 可看的变量(每台一套, 全局):
  *   g1_* / g2_*:
  *     tgt_cnt / tgt_deg    目标位置      now_cnt / now_deg  实际位置(0xA3 回读)
  *     spd_rpm 实际速度      st 状态:0空闲 1送保持 2松闸 3运动 4到位
  *     brk / brk_plot 抱闸   auto 自动随机开关  rx_cnt 收到的本机回包帧数
  *     move_cycles / move_timeouts        peak_rpm 运行以来最大|速度|
  *     cfg_acc_x100 / cfg_dec_x100 / cfg_vmax_x100 / cfg_iq_ma  驱动参数回读
  *     base_cnt 零点基准    cur_pos_kp/ki, cur_vel_kp/ki, cur_pos_lim_rpm, cur_vel_lim_A  驱动现值(只读)
  *     可写: pos_cmd_deg 手动目标角 / auto / pid_pos_kp/ki, pid_vel_kp/ki, pid_pos_lim_rpm, pid_vel_lim_A
  *   共享:  g_run_ms 开机运行毫秒(时间轴)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "gpio.h"
#include "drv_can.h"
#include "dma.h"
#include "usart.h"
#include <stdio.h>

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MOT_COUNT               (2)    /* 电机数量 */
#define MOT1_ADDR               (1u)   /* 电机1 设备地址(CAN ID) */
#define MOT2_ADDR               (2u)   /* 电机2 设备地址(CAN ID) */

/* 首次上电"把当前位置设成0点"只在无标记时执行; 之后每次上电回0点 */
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
#define CMD_REPEAT_MS           (300)  /* 运动中周期性重发目标, 抗偶发丢帧/总线错误 */

/* 自动随机(每台独立开关 g1_auto/g2_auto) */
#define ENABLE_AUTO_RANDOM      (0)    /* 0=不做自动随机, 目标由 pos_cmd_deg 给 */
#define AUTO_INTERVAL_MS        (2000)
#define AUTO_START_DELAY_MS     (1500)
#define AUTO_RANGE_DEG          (180)
#define HOLD_ZERO_MS            (5000) /* 上电回零到位后, 先在0点保持5s再开始随机 */

/* 主循环节拍 */
#define LOOP_PERIOD_MS          (5)
#define POS_POLL_MS             (20)   /* 定期 0xA3 读位置 */

/* USART2 串口(1M 波特率): 上行回传 = VOFA+ JustFloat(不堆文本); 下行 = ASCII 指令控制 */
#define UART_TX_PERIOD_MS       (20)   /* 回传周期(ms) */
#define UART_TX_CHANNELS        (8)    /* JustFloat 通道数: 6路电机数据 + 2路串口诊断 */
#define UART_RX_BUF_SIZE        (128)  /* DMA 空闲接收缓冲 */
#define UART_ACC_SIZE           (120)  /* 下行行累积缓冲 */
#define UART_CMD_MAX            (64)   /* 单条指令最大长度 */

/* 首次零点标记(STM32 flash 扇区4, 每电机一个 32bit 字)
 * 注: 魔数已从旧单机版(0x5A5A5A01)换成新值 —— 本固件首次上电会把两台都当作"第一次"各设一次原点 */
#define PERSIST_SECTOR          (FLASH_SECTOR_4)
#define PERSIST_ADDR_M1         (0x08010000u)  /* 电机1(地址1) 首次原点标记字 */
#define PERSIST_ADDR_M2         (0x08010004u)  /* 电机2(地址2) 首次原点标记字 */
#define PERSIST_MAGIC           (0x5A5A5A02u)

/* ====== 在线调 PID(用 LinkScope 写变量即可, 每台一套) ======
 * 0xB6 位置环Kp  0xB7 位置环Ki  0xB8 速度环Kp  0xB9 速度环Ki (float, 断电不保存) */
#define PID_POLL_MS             (300)   /* PID 下发/比较周期 */
#define RD_PERIOD_MS            (100)   /* 参数回读: 每100ms只发一条, 一条一条轮询 */

/* USER CODE END PD */

/* ============================================================
 * LinkScope 变量(全局符号, 每台电机一套 g1_* / g2_*)
 * 中断(ISR)写的加 volatile; 其余主循环每拍刷新
 * ============================================================ */
volatile uint32_t g_run_ms  = 0;        /* 开机后运行毫秒(两机共用时间轴) */

/* ---- 电机1(地址1) ---- */
volatile int32_t  g1_now_cnt  = 0;      /* 实际位置 counts(0xA3, ISR 写) */
volatile uint32_t g1_rx_cnt   = 0;      /* 收到的 CAN 帧数(ISR 写) */
volatile int32_t  g1_tgt_cnt  = 0;      /* 实际下发给驱动的绝对目标 counts */
volatile float    g1_tgt_deg  = 0.0f;   /* 当前目标 度(相对零点, 限±180) */
volatile float    g1_now_deg  = 0.0f;   /* 实际位置 度 */
volatile float    g1_spd_rpm  = 0.0f;   /* 实际速度 rpm */
volatile int32_t  g1_st       = 0;      /* 状态机状态 */
volatile uint8_t  g1_brk      = 1;      /* 抱闸: 1=闭合 0=松开 */
volatile int32_t  g1_brk_plot = 50;     /* 给LinkScope画的抱闸: 合=50 松=0 */
volatile uint8_t  g1_auto     = ENABLE_AUTO_RANDOM;
volatile uint32_t g1_move_cycles   = 0; /* 完成/超时次数 */
volatile uint32_t g1_move_timeouts = 0; /* 无回包超时次数 */
volatile int32_t  g1_base_cnt      = 0; /* 零点基准 count */
volatile float    g1_peak_rpm      = 0.0f; /* 实测的最大|速度| rpm */
volatile int32_t  g1_cfg_acc_x100  = 0; /* 回读 0xD0 加速度(x0.01rpm/s) */
volatile int32_t  g1_cfg_dec_x100  = 0; /* 回读 0xD1 减速度 */
volatile int32_t  g1_cfg_vmax_x100 = 0; /* 回读 0xB2 位置最大速度(x0.01rpm) */
volatile int32_t  g1_cfg_iq_ma     = 0; /* 回读 0xB3 最大电流(mA) */
/* 可写: 在线调参 / 手动目标 */
volatile float g1_pos_cmd_deg = 0.0f;
volatile float g1_pid_pos_kp = 0.0f, g1_pid_pos_ki = 0.0f, g1_pid_vel_kp = 0.0f, g1_pid_vel_ki = 0.0f;
volatile float g1_pid_pos_lim_rpm = (float)POS_MAX_RPM;
volatile float g1_pid_vel_lim_A   = IQ_LIMIT_A;
volatile float g1_accel_rpmps = (float)ACCEL_RPMPS;   /* 可写: 梯形加速度 rpm/s(0xD0) */
volatile float g1_decel_rpmps = (float)DECEL_RPMPS;   /* 可写: 梯形减速度 rpm/s(0xD1) */
/* 只读: 驱动当前实际值 */
volatile float g1_cur_pos_kp = 0.0f, g1_cur_pos_ki = 0.0f, g1_cur_vel_kp = 0.0f, g1_cur_vel_ki = 0.0f;
volatile float g1_cur_pos_lim_rpm = 0.0f, g1_cur_vel_lim_A = 0.0f;

/* ---- 电机2(地址2) ---- */
volatile int32_t  g2_now_cnt  = 0;
volatile uint32_t g2_rx_cnt   = 0;
volatile int32_t  g2_tgt_cnt  = 0;
volatile float    g2_tgt_deg  = 0.0f;
volatile float    g2_now_deg  = 0.0f;
volatile float    g2_spd_rpm  = 0.0f;
volatile int32_t  g2_st       = 0;
volatile uint8_t  g2_brk      = 1;
volatile int32_t  g2_brk_plot = 50;
volatile uint8_t  g2_auto     = ENABLE_AUTO_RANDOM;
volatile uint32_t g2_move_cycles   = 0;
volatile uint32_t g2_move_timeouts = 0;
volatile int32_t  g2_base_cnt      = 0;
volatile float    g2_peak_rpm      = 0.0f;
volatile int32_t  g2_cfg_acc_x100  = 0;
volatile int32_t  g2_cfg_dec_x100  = 0;
volatile int32_t  g2_cfg_vmax_x100 = 0;
volatile int32_t  g2_cfg_iq_ma     = 0;
volatile float g2_pos_cmd_deg = 0.0f;
volatile float g2_pid_pos_kp = 0.0f, g2_pid_pos_ki = 0.0f, g2_pid_vel_kp = 0.0f, g2_pid_vel_ki = 0.0f;
volatile float g2_pid_pos_lim_rpm = (float)POS_MAX_RPM;
volatile float g2_pid_vel_lim_A   = IQ_LIMIT_A;
volatile float g2_accel_rpmps = (float)ACCEL_RPMPS;   /* 可写: 梯形加速度 rpm/s(0xD0) */
volatile float g2_decel_rpmps = (float)DECEL_RPMPS;   /* 可写: 梯形减速度 rpm/s(0xD1) */
volatile float g2_cur_pos_kp = 0.0f, g2_cur_pos_ki = 0.0f, g2_cur_vel_kp = 0.0f, g2_cur_vel_ki = 0.0f;
volatile float g2_cur_pos_lim_rpm = 0.0f, g2_cur_vel_lim_A = 0.0f;

/* ============================================================
 * 每台电机的完整状态(含 ISR 写的反馈量)
 * ============================================================ */
typedef struct
{
    uint8_t addr;                       /* 设备地址: 1 或 2 */

    /* ---- ISR(CAN 回包)写 ---- */
    volatile int32_t  now_cnt;          /* 实际位置 counts */
    volatile int32_t  spd_x100;         /* 实际速度 0.01rpm */
    volatile uint32_t rx_cnt;           /* 收到本机回包帧数 */
    volatile float    rb_pkp, rb_pki, rb_vkp, rb_vki;  /* 驱动回读 PID 现值 */
    volatile int32_t  cfg_acc_x100, cfg_dec_x100;      /* 回读 0xD0/0xD1 */
    volatile int32_t  cfg_vmax_x100, cfg_iq_ma;        /* 回读 0xB2/0xB3 */
    volatile uint8_t  f_d0, f_d1, f_b2, f_b3;          /* 是否已读到 D0/D1/B2/B3 */
    volatile uint8_t  f_pkp, f_pki, f_vkp, f_vki;      /* 是否已收到各自回读 */

    /* ---- 可写目标(每拍从 LinkScope 的 g1_* / g2_* 镜像同步进来) ---- */
    float pid_pos_kp, pid_pos_ki, pid_vel_kp, pid_vel_ki; /* 目标 PID(0xB6~B9) */
    float pid_pos_lim_rpm, pid_vel_lim_A;                 /* 目标 输出限(0xB2/B3) */
    float acc_rpmps, dec_rpmps;                           /* 目标 梯形加速度/减速度 rpm/s(0xD0/D1) */
    uint8_t c_pkp, c_pki, c_vkp, c_vki;   /* 是否已拷给 pid_* */
    uint8_t pid_loaded;                   /* 已把驱动现值复制给 pid_* (可安全开始写) */
    float pos_cmd_deg, last_cmd_deg;      /* 手动目标角(LinkScope 给) 及去抖 */

    /* ---- 主循环算/显示 ---- */
    int32_t  local_tgt, tgt_cnt, base_cnt;
    float    tgt_deg, now_deg, spd_rpm, peak_rpm;
    int32_t  st;                          /* 状态机 */
    uint8_t  brk;                         /* 抱闸逻辑值 */
    int32_t  brk_plot;
    uint8_t  auto_on;                     /* 本机自动随机开关 */
    uint8_t  first_hold;
    uint32_t move_cycles, move_timeouts;

    /* ---- 私有时序 ---- */
    uint32_t st_t0, auto_next_ms, last_ack_cnt, last_poll_ms, last_rd_ms;
    uint32_t cmd_ms;                      /* 上次(重)发目标指令的时刻 */
    uint32_t brk_ms;                      /* 上次发松闸/抱闸指令的时刻 */
    int32_t  last_acc_x100, last_dec_x100;/* 上次下发的 加减速(x0.01rpm/s), 目标变就发 */
    int32_t  last_vmax_x100, last_iq_ma;  /* 上次下发的 速度上限(x0.01rpm)/最大电流(mA) */
    uint8_t  rd_idx;                      /* 参数轮询指针 */
} MOT;

static MOT mot[MOT_COUNT];              /* 全零初始化, 地址等字段在 main() 开头再填 */

static uint32_t g_rng = 0x9E3779B9u;     /* 伪随机种子 */
static uint8_t  g_poll_phase = 0;         /* 位置/速度轮询分相 */

/* ---- 串口链路诊断(LinkScope 可看, 也回传为 JustFloat 第7/8通道) ---- */
volatile uint32_t g_uart_rx_bytes = 0;   /* 从 PC 收到的字节总数 */
volatile uint32_t g_uart_cmd_cnt  = 0;   /* 成功执行的下行指令条数 */

/* ---- USART2 下行接收(ISR 写 / 主循环读) ---- */
static uint8_t  s_uart_rx[UART_RX_BUF_SIZE];     /* DMA 收 */
static uint8_t  s_uart_acc[UART_ACC_SIZE];       /* 行累积 */
static uint16_t s_uart_acc_len = 0;
static volatile uint8_t  s_uart_cmd[UART_CMD_MAX]; /* 一条完整指令 */
static volatile uint16_t s_uart_cmd_len = 0;
static volatile uint8_t  s_uart_cmd_ready = 0;      /* 1=有指令待主循环执行 */

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

/* 0xB1 设当前位置为原点(断电保存) */
static void ze300_set_origin(uint8_t addr)
{
    uint8_t d[1] = {0xB1};
    can_send(addr, d, 1);
}
/* 0xDA 梯形绝对位置 */
static void ze300_target_cnt(uint8_t addr, int32_t cnt)
{
    uint8_t d[6] = {0xDA, 0x00, 0, 0, 0, 0};
    put_i32le(&d[2], cnt);
    can_send(addr, d, 6);
}
/* 0xA3 读当前位置 */
static void ze300_read_pos(uint8_t addr)
{
    uint8_t d[1] = {0xA3};
    can_send(addr, d, 1);
}
/* 0xA2 读实时速度 */
static void ze300_read_spd(uint8_t addr)
{
    uint8_t d[1] = {0xA2};
    can_send(addr, d, 1);
}
/* 0xCE 抱闸 */
static void brake(uint8_t addr, uint8_t close)
{
    uint8_t d[2];
    d[0] = 0xCE;
    d[1] = close ? 0x01u : 0x00u;
    can_send(addr, d, 2);
}
/* 0xD0/0xD1 梯形加/减速度 */
static void trap_acc(uint8_t addr, uint8_t cmd, int32_t v_x100)
{
    uint8_t d[5] = {cmd, 0, 0, 0, 0};
    put_i32le(&d[1], v_x100);
    can_send(addr, d, 5);
}
/* 0xB2 位置模式最大速度(0.01rpm) */
static void pos_max_speed_rpm(uint8_t addr, int32_t rpm)
{
    uint8_t d[5] = {0xB2, 0, 0, 0, 0};
    put_i32le(&d[1], rpm * 100);
    can_send(addr, d, 5);
}
/* 0xB3 位置/速度模式最大Q轴电流(0.001A) */
static void pos_iq_limit_ma(uint8_t addr, uint32_t ma)
{
    uint8_t d[5] = {0xB3, 0, 0, 0, 0};
    put_i32le(&d[1], (int32_t)ma);
    can_send(addr, d, 5);
}
/* 在线写 PID: cmd = 0xB6/0xB7/0xB8/0xB9, 值为 float */
static void pid_write(uint8_t addr, uint8_t cmd, float v)
{
    uint8_t d[5] = {cmd, 0, 0, 0, 0};
    put_f32le(&d[1], v);
    can_send(addr, d, 5);
}

/* ---------------- flash 首次零点标记(每电机一个字, 尽量不整区擦) ---------------- */
static void persist_program_word(uint32_t addr)
{
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, PERSIST_MAGIC);
    HAL_FLASH_Lock();
}
static void persist_erase_sector(void)
{
    FLASH_EraseInitTypeDef er;
    uint32_t err = 0;
    HAL_FLASH_Unlock();
    er.TypeErase = FLASH_TYPEERASE_SECTORS;
    er.Sector = PERSIST_SECTOR;
    er.NbSectors = 1;
    er.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASHEx_Erase(&er, &err);
    HAL_FLASH_Lock();
}

/* ---------------- 把 count 归一到 ±半圈(±180°)内 ---------------- */
static int32_t norm180(int32_t cnt)
{
    int32_t c = cnt % 16384;
    if (c > 8191)      c -= 16384;
    else if (c < -8191) c += 16384;
    return c;
}

/* 当前实际位置 相对零点(基准) 的局部角 counts, 永远限在 ±半圈内 */
static int32_t local_now_cnt(MOT *m)
{
    return norm180(m->now_cnt - m->base_cnt);
}

/* 启动一次运动: 目标 = 相对零点的局部角 local_cnt(±半圈内, 自动走最短方向).
 * 命令总是"当前 count + 小增量", 到位判断也用局部角, 不会发散. */
static void start_move(MOT *m, int32_t local_cnt)
{
    int32_t nowl, delta, eff;
    if (m->st != 0) return;              /* 忙就忽略 */

    nowl  = local_now_cnt(m);            /* 当前局部角 */
    delta = norm180(local_cnt - nowl);   /* 需要的局部增量(自动取短方向) */
    eff   = m->now_cnt + delta;          /* 绝对命令 = 当前count + 小增量 */

    m->local_tgt = local_cnt;
    m->tgt_cnt   = eff;
    m->tgt_deg   = (float)norm180(local_cnt) * 360.0f / 16384.0f;
    m->last_ack_cnt = m->rx_cnt;         /* 记录起点回包数, 用于"没接电机"判定 */
    m->st  = 1;
    m->st_t0 = g_run_ms;
    m->cmd_ms = g_run_ms;
    m->brk_ms = g_run_ms;
    ze300_target_cnt(m->addr, m->now_cnt);  /* 先发当前位置让驱动建保持, 抵消松闸下坠 */
}

/* 上电专用回零(阻塞一次): 送当前位置保持 -> 松闸 -> 0xC4 最短回原点 -> 等静止 -> 合闸 */
static void home_boot(MOT *m)
{
    uint8_t d;
    uint32_t t0, last_chg;
    int32_t prev;

    ze300_target_cnt(m->addr, m->now_cnt);   /* 让驱动先建立保持, 防松闸瞬间下坠 */
    HAL_Delay(ENGAGE_HOLD_MS);
    brake(m->addr, 0);
    HAL_Delay(BRAKE_RELEASE_MS);

    d = 0xC4;                            /* 最短距离回原点(≤180°) */
    can_send(m->addr, &d, 1);

    /* 位置连续不变约400ms -> 认为到位; 一直无回包(没接电机)1.2s退出 */
    prev = m->now_cnt;
    t0 = HAL_GetTick();
    last_chg = t0;
    while (1)
    {
        uint8_t a = 0xA3;
        if (HAL_GetTick() - t0 > 2500) break;
        can_send(m->addr, &a, 1);
        HAL_Delay(20);
        if (m->now_cnt != prev)
        {
            prev = m->now_cnt;
            last_chg = HAL_GetTick();
        }
        else if (HAL_GetTick() - last_chg >= 400)
        {
            break;                       /* 静止到位 */
        }
        if (m->rx_cnt == 0 && HAL_GetTick() - t0 > 1200) break;
    }
    brake(m->addr, 1);
}

/* ---------------- 异步运动状态机(每台一套) ---------------- */
static void mot_st_machine(MOT *m)
{
    uint32_t t = g_run_ms;

    switch (m->st)
    {
    case 0:                             /* 空闲 */
        if (m->auto_on && (int32_t)(t - m->auto_next_ms) >= 0)
        {
            int32_t local = rng_in_range(AUTO_RANGE_DEG) * 16384 / 360; /* 相对零点随机(度) */
            start_move(m, local);        /* 只给局部角, 由 start_move 转成小增量命令 */
        }
        break;

    case 1:                             /* 送保持, 等一小会 */
        if (t - m->st_t0 >= ENGAGE_HOLD_MS)
        {
            brake(m->addr, 0);           /* 松闸 */
            m->brk_ms = t;
            m->st = 2;
            m->st_t0 = t;
        }
        break;

    case 2:                             /* 松闸响应中 */
        /* 松闸帧可能被总线吞掉, 每 100ms 补发一次, 确保真松开 */
        if ((int32_t)(t - m->brk_ms) >= 100)
        {
            brake(m->addr, 0);
            m->brk_ms = t;
        }
        if (t - m->st_t0 >= BRAKE_RELEASE_MS)
        {
            ze300_target_cnt(m->addr, m->tgt_cnt);   /* 真正下发目标 */
            m->cmd_ms = t;
            m->st = 3;
            m->st_t0 = t;
        }
        break;

    case 3:                             /* 运动中 */
    {
        int32_t err;

        /* 周期性重发目标 + 保持松闸: 单帧丢失/总线偶发错误也不至于刹死不动 */
        if ((int32_t)(t - m->cmd_ms) >= (int32_t)CMD_REPEAT_MS)
        {
            ze300_target_cnt(m->addr, m->tgt_cnt);
            brake(m->addr, 0);           /* 保持松闸, 防之前松闸帧丢了刹死 */
            m->cmd_ms = t;
            m->brk_ms = t;
        }
        err = local_now_cnt(m) - m->local_tgt;          /* 用局部角判到位, 不追多圈层 */
        if (err > -SETTLE_TOL_CNT && err < SETTLE_TOL_CNT)
        {
            m->st = 4;
            m->st_t0 = t;
        }
        else if (m->rx_cnt == m->last_ack_cnt && (t - m->st_t0) > NO_ACK_TIMEOUT_MS)
        {
            /* 发目标后一直无任何回包: 认为没接电机, 不等了 */
            m->st = 4;
            m->st_t0 = t;
            m->move_timeouts++;
        }
        else if (m->rx_cnt != m->last_ack_cnt && (t - m->st_t0) > ARRIVE_MAX_MS)
        {
            m->st = 4;                   /* 有反馈但一直没到位, 兜底 */
            m->st_t0 = t;
            m->move_timeouts++;
        }
        break;
    }

    case 4:                             /* 到位/兜底 -> 合闸锁死 */
        if (t - m->st_t0 >= BRAKE_ENGAGE_MS)
        {
            uint32_t hold = m->first_hold ? (uint32_t)HOLD_ZERO_MS : (uint32_t)AUTO_INTERVAL_MS;
            m->first_hold = 0;           /* 只有开机那次回零要多保持5s */
            brake(m->addr, 1);
            m->st = 0;
            m->move_cycles++;
            m->auto_next_ms = t + hold;  /* 到位后再等 hold 才给下一个随机位置 */
        }
        break;
    }
}

/* ---------------- 参数管理器(每台独立: 每100ms只做一件事) ----------------
 * 加减速/限速/限流(D0/D1/B2/B3): 目标一变就下发, 不依赖驱动是否回读;
 * 若回读可用, 发现驱动值不符会自愈再发一次。
 * PID(B6~B9): 仍要先收到一次回读把驱动现值灌进目标, 才允许在线写(避免误写成0)。
 * 没有要写的就轮询读下一条, 让 gN_cfg_* 能刷新。 */
static void mot_manager(MOT *m)
{
    static const uint8_t rd_cmds[8] =
        {0xD0, 0xD1, 0xB2, 0xB3, 0xB6, 0xB7, 0xB8, 0xB9};
    uint8_t i, sent = 0;
    int32_t want_acc, want_dec, want_vmax, want_iq, rpm_lim;
    uint32_t now = g_run_ms;

    if (now - m->last_rd_ms < RD_PERIOD_MS) return;
    m->last_rd_ms = now;

    /* 各参数目标值(x0.01 或 mA) */
    want_acc  = (int32_t)(m->acc_rpmps * 100.0f + 0.5f);
    want_dec  = (int32_t)(m->dec_rpmps * 100.0f + 0.5f);
    rpm_lim   = (int32_t)m->pid_pos_lim_rpm;
    want_vmax = rpm_lim * 100;
    want_iq   = (int32_t)(m->pid_vel_lim_A * 1000.0f + 0.5f);

    if (!m->pid_loaded)
    {
        /* 第一次把驱动原值灌进 pid_* 目标, 保证不乱写 */
        if (m->f_pkp) { m->pid_pos_kp = m->rb_pkp; m->f_pkp = 0; m->c_pkp = 1; }
        if (m->f_pki) { m->pid_pos_ki = m->rb_pki; m->f_pki = 0; m->c_pki = 1; }
        if (m->f_vkp) { m->pid_vel_kp = m->rb_vkp; m->f_vkp = 0; m->c_vkp = 1; }
        if (m->f_vki) { m->pid_vel_ki = m->rb_vki; m->f_vki = 0; m->c_vki = 1; }
        if (m->c_pkp && m->c_pki && m->c_vkp && m->c_vki) m->pid_loaded = 1;
    }

    /* 扫描第一个需要写的参数(一次只发一条) */
    for (i = 0; i < 8 && !sent; i++)
    {
        switch (i)
        {
        case 0: /* 梯形加速度 D0 */
            if (want_acc != m->last_acc_x100 || (m->f_d0 && m->cfg_acc_x100 != want_acc))
            { trap_acc(m->addr, 0xD0, want_acc); m->last_acc_x100 = want_acc; sent = 1; }
            break;
        case 1: /* 梯形减速度 D1 */
            if (want_dec != m->last_dec_x100 || (m->f_d1 && m->cfg_dec_x100 != want_dec))
            { trap_acc(m->addr, 0xD1, want_dec); m->last_dec_x100 = want_dec; sent = 1; }
            break;
        case 2: /* 位置最大速度 B2 */
            if (want_vmax != m->last_vmax_x100 || (m->f_b2 && m->cfg_vmax_x100 != want_vmax))
            { pos_max_speed_rpm(m->addr, rpm_lim); m->last_vmax_x100 = want_vmax; sent = 1; }
            break;
        case 3: /* 最大电流 B3 */
            if (want_iq != m->last_iq_ma || (m->f_b3 && m->cfg_iq_ma != want_iq))
            { pos_iq_limit_ma(m->addr, (uint32_t)want_iq); m->last_iq_ma = want_iq; sent = 1; }
            break;
        case 4:
            if (m->pid_loaded && m->pid_pos_kp != m->rb_pkp) { pid_write(m->addr, 0xB6, m->pid_pos_kp); sent = 1; }
            break;
        case 5:
            if (m->pid_loaded && m->pid_pos_ki != m->rb_pki) { pid_write(m->addr, 0xB7, m->pid_pos_ki); sent = 1; }
            break;
        case 6:
            if (m->pid_loaded && m->pid_vel_kp != m->rb_vkp) { pid_write(m->addr, 0xB8, m->pid_vel_kp); sent = 1; }
            break;
        case 7:
            if (m->pid_loaded && m->pid_vel_ki != m->rb_vki) { pid_write(m->addr, 0xB9, m->pid_vel_ki); sent = 1; }
            break;
        }
    }

    if (!sent)   /* 没有要写的, 就读下一条(轮询, 刷新 gN_cfg_*) */
    {
        uint8_t c = rd_cmds[m->rd_idx];
        m->rd_idx = (m->rd_idx + 1) % 8;
        can_send(m->addr, &c, 1);
    }
}

/* ---------------- 派生量刷新(每台) ---------------- */
static void mot_derive(MOT *m)
{
    int32_t now = m->now_cnt;
    float spd;

    m->now_deg = (float)norm180(now - m->base_cnt) * 360.0f / 16384.0f;
    m->spd_rpm = (float)m->spd_x100 * 0.01f;
    spd = m->spd_rpm;
    if (spd < 0.0f) { if (-spd > m->peak_rpm) m->peak_rpm = -spd; }
    else            { if ( spd > m->peak_rpm) m->peak_rpm =  spd; }
    m->brk_plot = m->brk ? 50 : 0;       /* 抱闸绘图: 合=50 松=0, 台阶明显 */
}

/* ---------------- LinkScope 镜像: 可写变量抄进来 / 状态量抄出去 ---------------- */
static void sync_in(MOT *m)
{
    if (m->addr == MOT1_ADDR)
    {
        m->auto_on          = g1_auto;
        m->pos_cmd_deg      = g1_pos_cmd_deg;
        m->pid_pos_kp       = g1_pid_pos_kp;
        m->pid_pos_ki       = g1_pid_pos_ki;
        m->pid_vel_kp       = g1_pid_vel_kp;
        m->pid_vel_ki       = g1_pid_vel_ki;
        m->pid_pos_lim_rpm  = g1_pid_pos_lim_rpm;
        m->pid_vel_lim_A    = g1_pid_vel_lim_A;
        m->acc_rpmps        = g1_accel_rpmps;
        m->dec_rpmps        = g1_decel_rpmps;
    }
    else
    {
        m->auto_on          = g2_auto;
        m->pos_cmd_deg      = g2_pos_cmd_deg;
        m->pid_pos_kp       = g2_pid_pos_kp;
        m->pid_pos_ki       = g2_pid_pos_ki;
        m->pid_vel_kp       = g2_pid_vel_kp;
        m->pid_vel_ki       = g2_pid_vel_ki;
        m->pid_pos_lim_rpm  = g2_pid_pos_lim_rpm;
        m->pid_vel_lim_A    = g2_pid_vel_lim_A;
        m->acc_rpmps        = g2_accel_rpmps;
        m->dec_rpmps        = g2_decel_rpmps;
    }
}

static void sync_out(MOT *m)
{
    volatile int32_t *p_now, *p_tgt, *p_base, *p_st, *p_brk_plot;
    volatile int32_t *p_cfg_acc, *p_cfg_dec, *p_cfg_vmax, *p_cfg_iq;
    volatile float   *p_tgt_deg, *p_now_deg, *p_spd_rpm, *p_peak_rpm;
    volatile uint32_t *p_rx, *p_cycles, *p_timeouts;
    volatile uint8_t *p_brk, *p_auto;
    volatile float   *p_pos_kp, *p_pos_ki, *p_vel_kp, *p_vel_ki, *p_lim_rpm, *p_lim_a;
    volatile float   *p_ckp, *p_cki, *p_cvk, *p_cvi, *p_clr, *p_cla;

    if (m->addr == MOT1_ADDR)
    {
        p_now=&g1_now_cnt; p_tgt=&g1_tgt_cnt; p_base=&g1_base_cnt; p_st=&g1_st; p_brk_plot=&g1_brk_plot;
        p_cfg_acc=&g1_cfg_acc_x100; p_cfg_dec=&g1_cfg_dec_x100; p_cfg_vmax=&g1_cfg_vmax_x100; p_cfg_iq=&g1_cfg_iq_ma;
        p_tgt_deg=&g1_tgt_deg; p_now_deg=&g1_now_deg; p_spd_rpm=&g1_spd_rpm; p_peak_rpm=&g1_peak_rpm;
        p_rx=&g1_rx_cnt; p_cycles=&g1_move_cycles; p_timeouts=&g1_move_timeouts;
        p_brk=&g1_brk; p_auto=&g1_auto;
        p_pos_kp=&g1_pid_pos_kp; p_pos_ki=&g1_pid_pos_ki; p_vel_kp=&g1_pid_vel_kp; p_vel_ki=&g1_pid_vel_ki;
        p_lim_rpm=&g1_pid_pos_lim_rpm; p_lim_a=&g1_pid_vel_lim_A;
        p_ckp=&g1_cur_pos_kp; p_cki=&g1_cur_pos_ki; p_cvk=&g1_cur_vel_kp; p_cvi=&g1_cur_vel_ki;
        p_clr=&g1_cur_pos_lim_rpm; p_cla=&g1_cur_vel_lim_A;
    }
    else
    {
        p_now=&g2_now_cnt; p_tgt=&g2_tgt_cnt; p_base=&g2_base_cnt; p_st=&g2_st; p_brk_plot=&g2_brk_plot;
        p_cfg_acc=&g2_cfg_acc_x100; p_cfg_dec=&g2_cfg_dec_x100; p_cfg_vmax=&g2_cfg_vmax_x100; p_cfg_iq=&g2_cfg_iq_ma;
        p_tgt_deg=&g2_tgt_deg; p_now_deg=&g2_now_deg; p_spd_rpm=&g2_spd_rpm; p_peak_rpm=&g2_peak_rpm;
        p_rx=&g2_rx_cnt; p_cycles=&g2_move_cycles; p_timeouts=&g2_move_timeouts;
        p_brk=&g2_brk; p_auto=&g2_auto;
        p_pos_kp=&g2_pid_pos_kp; p_pos_ki=&g2_pid_pos_ki; p_vel_kp=&g2_pid_vel_kp; p_vel_ki=&g2_pid_vel_ki;
        p_lim_rpm=&g2_pid_pos_lim_rpm; p_lim_a=&g2_pid_vel_lim_A;
        p_ckp=&g2_cur_pos_kp; p_cki=&g2_cur_pos_ki; p_cvk=&g2_cur_vel_kp; p_cvi=&g2_cur_vel_ki;
        p_clr=&g2_cur_pos_lim_rpm; p_cla=&g2_cur_vel_lim_A;
    }

    *p_now = m->now_cnt;  *p_tgt = m->tgt_cnt;  *p_base = m->base_cnt;
    *p_st = m->st;  *p_brk_plot = m->brk_plot;
    *p_cfg_acc = m->cfg_acc_x100; *p_cfg_dec = m->cfg_dec_x100;
    *p_cfg_vmax = m->cfg_vmax_x100; *p_cfg_iq = m->cfg_iq_ma;
    *p_tgt_deg = m->tgt_deg; *p_now_deg = m->now_deg;
    *p_spd_rpm = m->spd_rpm; *p_peak_rpm = m->peak_rpm;
    *p_rx = m->rx_cnt; *p_cycles = m->move_cycles; *p_timeouts = m->move_timeouts;
    *p_brk = m->brk; *p_auto = m->auto_on;
    *p_pos_kp = m->pid_pos_kp; *p_pos_ki = m->pid_pos_ki;
    *p_vel_kp = m->pid_vel_kp; *p_vel_ki = m->pid_vel_ki;
    *p_lim_rpm = m->pid_pos_lim_rpm; *p_lim_a = m->pid_vel_lim_A;
    *p_ckp = m->rb_pkp; *p_cki = m->rb_pki;
    *p_cvk = m->rb_vkp; *p_cvi = m->rb_vki;
    *p_clr = (float)m->cfg_vmax_x100 * 0.01f;
    *p_cla = (float)m->cfg_iq_ma    * 0.001f;
}

/* ---------------- USART2 回传(VOFA+ JustFloat, ~50Hz) ----------------
 * 帧 = 8×float 小端 + 帧尾 00 00 80 7F
 * 通道1-6: 两电机各 目标角°/实际角°/速度rpm
 * 通道7-8: 串口诊断 收到字节数 / 已执行指令条数(排查"发指令没反应"用)
 * VOFA+ 选 JustFloat 协议即可画曲线, 不会把数据当文本堆进发送区 */
static void uart_send_telemetry(void)
{
    static uint8_t jf[UART_TX_CHANNELS * 4 + 4];   /* static: DMA 发送期间必须有效 */
    float v;
    uint8_t *p = jf;
    int i;
    const float vals[UART_TX_CHANNELS] =
        { g1_tgt_deg, g1_now_deg, g1_spd_rpm,
          g2_tgt_deg, g2_now_deg, g2_spd_rpm,
          (float)g_uart_rx_bytes, (float)g_uart_cmd_cnt };

    for (i = 0; i < UART_TX_CHANNELS; i++)
    {
        v = vals[i];
        put_f32le(p, v);
        p += 4;
    }
    jf[UART_TX_CHANNELS * 4 + 0] = 0x00;
    jf[UART_TX_CHANNELS * 4 + 1] = 0x00;
    jf[UART_TX_CHANNELS * 4 + 2] = 0x80;
    jf[UART_TX_CHANNELS * 4 + 3] = 0x7F;            /* JustFloat 帧尾 */
    if (huart2.gState == HAL_UART_STATE_READY)
    {
        HAL_UART_Transmit_DMA(&huart2, jf, UART_TX_CHANNELS * 4 + 4);
    }
}

/* ---------------- USART2 下行指令(在发送区输入, 回车发送) ----------------
 *  m1 <角度°>   电机1转到目标角(相对自己的零点, ±180)
 *  m2 <角度°>   电机2转到目标角
 *  stop         两台停在当前位置 */
static void serial_set_target(uint8_t idx, float deg)
{
    volatile float *gp;
    MOT *m;

    if (deg > 180.0f) deg = 180.0f;
    else if (deg < -180.0f) deg = -180.0f;

    m = (idx == 0) ? &mot[0] : &mot[1];
    gp = (idx == 0) ? &g1_pos_cmd_deg : &g2_pos_cmd_deg;

    m->auto_on = 0;                     /* 手动接管, 关随机 */
    *gp = deg;                          /* 写到 LinkScope 同源变量, 主循环 sync_in 会同步 */
    if (m->st == 0)
    {
        m->last_cmd_deg = deg;          /* 立即执行, 让手动逻辑不重复触发 */
        start_move(m, (int32_t)(deg * 16384.0f / 360.0f));
    }
    /* 忙碌: 目标已写入, 空闲后主循环手动逻辑会补跑 */
    g_uart_cmd_cnt++;                   /* 诊断: 成功执行一条指令就+1 */
}

/* 手写浮点解析: 符号+整数+小数, 不依赖 printf/scanf 库(防微库 %f 失效) */
static float parse_num(const char **pp)
{
    const char *p = *pp;
    long ip = 0;
    float fp = 0.0f, scale = 0.1f;
    int neg = 0, has = 0;

    while (*p == ' ' || *p == '\t') p++;
    if (*p == '-') { neg = 1; p++; }
    else if (*p == '+') p++;
    while (*p >= '0' && *p <= '9') { ip = ip * 10 + (*p - '0'); has = 1; p++; }
    if (*p == '.')
    {
        p++;
        while (*p >= '0' && *p <= '9')
        {
            fp += scale * (float)(*p - '0');
            scale *= 0.1f;
            has = 1;
            p++;
        }
    }
    *pp = p;
    if (!has) return 0.0f;
    return neg ? -((float)ip + fp) : ((float)ip + fp);
}

/* 下行指令: "电机1角度 电机2角度" —— 两个数值, 空格或逗号分隔, 单位度(绝对, 相对各自零点)
 * 例: "30 -45"  → 电机1去30°, 电机2去-45°;  两者同时下发 */
static void serial_execute(const char *line)
{
    const char *p = line;
    float d1, d2;
    int cnt = 0;

    while (*p)
    {
        if (*p == '-' || *p == '+' || (*p >= '0' && *p <= '9'))
        {
            float v = parse_num(&p);
            if (cnt == 0)      d1 = v;
            else if (cnt == 1) d2 = v;
            cnt++;
        }
        else
        {
            p++;                            /* 空格/逗号/其它分隔符跳过 */
        }
    }

    if (cnt >= 2)                           /* 两个数: 两台同时给绝对目标 */
    {
        serial_set_target(0, d1);
        serial_set_target(1, d2);
    }
}

/* ---------------- USART2 接收 ISR(DMA 空闲): 攒到换行给一条指令 ---------------- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint16_t i, j, n;
    uint8_t b;

    if (huart->Instance != USART2) return;
    g_uart_rx_bytes += Size;            /* 诊断: 只要收到任何字节就累加 */

    for (i = 0; i < Size; i++)
    {
        b = s_uart_rx[i];
        if (b == '\n' || b == '\r')
        {
            if (s_uart_acc_len > 0 && !s_uart_cmd_ready)
            {
                n = s_uart_acc_len;
                if (n > UART_CMD_MAX - 1) n = UART_CMD_MAX - 1;
                for (j = 0; j < n; j++) s_uart_cmd[j] = s_uart_acc[j];
                s_uart_cmd[n] = 0;
                s_uart_cmd_len = n;
                s_uart_cmd_ready = 1;
            }
            s_uart_acc_len = 0;
        }
        else
        {
            if (s_uart_acc_len < UART_ACC_SIZE)
            {
                s_uart_acc[s_uart_acc_len] = b;
                s_uart_acc_len++;
            }
            else s_uart_acc_len = 0;    /* 超长丢弃整行 */
        }
    }

    /* 没收到换行但也静默了(上位机没发\r\n): 也把已收到的一行当作指令提交 */
    if (s_uart_acc_len > 0 && !s_uart_cmd_ready)
    {
        n = s_uart_acc_len;
        if (n > UART_CMD_MAX - 1) n = UART_CMD_MAX - 1;
        for (j = 0; j < n; j++) s_uart_cmd[j] = s_uart_acc[j];
        s_uart_cmd[n] = 0;
        s_uart_cmd_len = n;
        s_uart_cmd_ready = 1;
        s_uart_acc_len = 0;
    }

    HAL_UARTEx_ReceiveToIdle_DMA(huart, s_uart_rx, UART_RX_BUF_SIZE);
}

/* ---------------- USART2 出错急救(避免 ORE 后 DMA 永久停收) ---------------- */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) return;
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    HAL_UART_AbortReceive(huart);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, s_uart_rx, UART_RX_BUF_SIZE);
}

/* ---------------- CAN RX 回调(按回包 StdId = 电机自身地址 分拣) ---------------- */
static MOT *mot_of_stdid(uint16_t stdid)
{
    if (stdid == mot[0].addr) return &mot[0];
    if (stdid == mot[1].addr) return &mot[1];
    return 0;                            /* 其它 ID 忽略(如广播残留) */
}

void CAN_Motor_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    const uint8_t *d = Rx_Buffer->Data;
    MOT *m = mot_of_stdid(Rx_Buffer->Header.StdId);

    if (m == 0) return;
    m->rx_cnt++;

    if (d[0] == 0xB2) { m->cfg_vmax_x100 = get_i32le(&d[1]); m->f_b2 = 1; }
    else if (d[0] == 0xD0) { m->cfg_acc_x100 = get_i32le(&d[1]); m->f_d0 = 1; }
    else if (d[0] == 0xD1) { m->cfg_dec_x100 = get_i32le(&d[1]); m->f_d1 = 1; }
    else if (d[0] == 0xB3) { m->cfg_iq_ma = get_i32le(&d[1]); m->f_b3 = 1; }
    else if (d[0] == 0xB6) { m->rb_pkp = get_f32le(&d[1]); m->f_pkp = 1; }
    else if (d[0] == 0xB7) { m->rb_pki = get_f32le(&d[1]); m->f_pki = 1; }
    else if (d[0] == 0xB8) { m->rb_vkp = get_f32le(&d[1]); m->f_vkp = 1; }
    else if (d[0] == 0xB9) { m->rb_vki = get_f32le(&d[1]); m->f_vki = 1; }
    else if (d[0] == 0xA3 || d[0] == 0xC2 || d[0] == 0xC3 || d[0] == 0xC4 ||
             d[0] == 0xDA || d[0] == 0xDC)
    {
        m->now_cnt = get_i32le(&d[3]);  /* 与 0xA3 相同, 多圈绝对值在[3..6] */
    }
    else if (d[0] == 0xA2 || d[0] == 0xC1)
    {
        m->spd_x100 = get_i32le(&d[1]);
    }
}

/**
  * @brief  The application entry point.
  */
int main(void)
{
    int i;
    uint32_t w1, w2;
    int need1, need2, need_erase;
    static uint32_t uart_last_ms = 0;   /* 串口回传节拍 */

    /* 填电机固定参数(开中断前先设好, 避免漏收早到的回包) */
    for (i = 0; i < MOT_COUNT; i++)
    {
        mot[i].addr            = (i == 0) ? MOT1_ADDR : MOT2_ADDR;
        mot[i].brk             = 1;                  /* 抱闸初始闭合 */
        mot[i].auto_on         = ENABLE_AUTO_RANDOM;
        mot[i].first_hold      = 1;
        mot[i].pid_pos_lim_rpm = (float)POS_MAX_RPM;
        mot[i].pid_vel_lim_A   = IQ_LIMIT_A;
        mot[i].acc_rpmps       = (float)ACCEL_RPMPS;
        mot[i].dec_rpmps       = (float)DECEL_RPMPS;
        mot[i].last_acc_x100   = (int32_t)ACCEL_RPMPS * 100;
        mot[i].last_dec_x100   = (int32_t)DECEL_RPMPS * 100;
        mot[i].last_vmax_x100  = POS_MAX_RPM * 100;
        mot[i].last_iq_ma      = (int32_t)(IQ_LIMIT_A * 1000.0f);
    }

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    /* USART2 串口(1M): 上行 JustFloat 回传 / 下行 ASCII 指令 */
    MX_DMA_Init();                      /* 必须先开 DMA 时钟 */
    MX_USART2_UART_Init();
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_uart_rx, UART_RX_BUF_SIZE); /* 开始收指令 */

    MX_CAN1_Init();                     /* CAN1 = 1MHz, PD0/PD1 */

    CAN_Init(&hcan1, CAN_Motor_Call_Back);
    g_run_ms = 0;

    /* ---- 设备地址 1/2 已在出厂/上位机设好, 不再自动配地址 ----
     * 上电只需: 首次(无 flash 标记)设一次原点, 之后 0xC4 回原点 */

    /* ---- 梯形加减速 / 限速限流(两台都发, 不接的电机自然不理会) ---- */
    for (i = 0; i < MOT_COUNT; i++)
    {
        trap_acc(mot[i].addr, 0xD0, (int32_t)(mot[i].acc_rpmps * 100.0f + 0.5f));
        trap_acc(mot[i].addr, 0xD1, (int32_t)(mot[i].dec_rpmps * 100.0f + 0.5f));
        pos_max_speed_rpm(mot[i].addr, POS_MAX_RPM);
        pos_iq_limit_ma(mot[i].addr, (uint32_t)(IQ_LIMIT_A * 1000.0f));
    }

    /* ---- 每台零点 / 回零 / 基准锁定(独立 flash 标记) ---- */
    w1 = *(volatile const uint32_t *)PERSIST_ADDR_M1;
    w2 = *(volatile const uint32_t *)PERSIST_ADDR_M2;
    need1 = (w1 != PERSIST_MAGIC);
    need2 = (w2 != PERSIST_MAGIC);

    for (i = 0; i < MOT_COUNT; i++)
    {
        MOT *m = &mot[i];
        int first = (i == 0) ? need1 : need2;

        if (first)
        {
            /* 该机第一次: 把当前位置设成0点(0xB1, 驱动断电保存) */
            ze300_set_origin(m->addr);
            HAL_Delay(200);
            ze300_read_pos(m->addr);    /* 读一次位置当基准 */
            HAL_Delay(150);
            brake(m->addr, 1);
        }
        else
        {
            home_boot(m);               /* 真正回零并等静止 */
        }
        m->base_cnt = m->now_cnt;       /* 锁定零点基准 */
        m->first_hold = 0;              /* 保持5s由 auto_next_ms 承担 */
        m->auto_next_ms = g_run_ms + HOLD_ZERO_MS;
    }

    /* 写"首次设原点"标记; 只有发现某字非空白(旧残留)才整区擦一次, 之后两字补写 */
    need_erase = 0;
    if (need1 && w1 != 0xFFFFFFFFu) need_erase = 1;
    if (need2 && w2 != 0xFFFFFFFFu) need_erase = 1;
    if (need1 || need2)
    {
        if (need_erase)
        {
            persist_erase_sector();
            need1 = need2 = 1;          /* 擦了就把两个标记都补上 */
        }
        if (need1) persist_program_word(PERSIST_ADDR_M1);
        if (need2) persist_program_word(PERSIST_ADDR_M2);
    }

    /* 种子, 让每次上电随机序列不同 */
    g_rng = HAL_GetTick();

    /* 首拍把结构体状态同步到 LinkScope 变量(让界面不是全0) */
    for (i = 0; i < MOT_COUNT; i++)
    {
        sync_out(&mot[i]);
    }

    /* ---- 4. 主循环(每 5ms 对两台各做一遍; 含 USART2 收指令/发回传) ---- */
    while (1)
    {
        uint32_t now = HAL_GetTick();
        g_run_ms = now;

        /* 执行串口下行指令(在 VOFA+ 发送区输入, 回车发送) */
        if (s_uart_cmd_ready)
        {
            char line[UART_CMD_MAX];
            uint16_t j;
            s_uart_cmd_ready = 0;
            for (j = 0; j < s_uart_cmd_len && j < UART_CMD_MAX - 1; j++)
                line[j] = (char)s_uart_cmd[j];
            line[j] = 0;
            serial_execute(line);
        }

        for (i = 0; i < MOT_COUNT; i++)
        {
            MOT *m = &mot[i];

            sync_in(m);                  /* LinkScope 可写变量抄进来 */

            /* 派生量: 度 / rpm (角度相对零点归一化±180) */
            mot_derive(m);

            /* 参数管理器(每机每100ms只发一条) */
            mot_manager(m);

            /* 定期读实际位置(20ms); 速度每5拍(100ms)读一次 —— 两查询不连发, 避免刷爆驱动 */
            if (now - m->last_poll_ms >= POS_POLL_MS)
            {
                m->last_poll_ms = now;
                ze300_read_pos(m->addr);
                if ((g_poll_phase & 3u) == 0u) ze300_read_spd(m->addr);
                g_poll_phase++;
            }

            mot_st_machine(m);

            /* ===== 手动目标: 改 g1/g2_pos_cmd_deg 就去(度, 相对零点; 空闲时执行) ===== */
            if (m->pos_cmd_deg != m->last_cmd_deg && m->st == 0)
            {
                m->last_cmd_deg = m->pos_cmd_deg;   /* 记下本次, 忙碌时下一次空闲再补跑 */
                m->auto_on = 0;                     /* 手动接管, 关随机 */
                start_move(m, (int32_t)(m->pos_cmd_deg * 16384.0f / 360.0f));
            }

            /* 状态量抄给 LinkScope */
            sync_out(m);
        }

        /* 串口回传电机信息(~50Hz) */
        if ((int32_t)(now - uart_last_ms) >= (int32_t)UART_TX_PERIOD_MS)
        {
            uart_last_ms = now;
            uart_send_telemetry();
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
