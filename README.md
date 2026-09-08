# ZE300_Pos_Home —— 位置控制(无串口, LinkScope 看变量) · 海泰 HT-J-2E + ZE300

平台：大疆 RoboMaster **开发板 A**（STM32F427IIH）+ **CAN1**（1 MHz）。
本版**不依赖任何串口**，上电即跑，用 **LinkScope**（SWD 连上后）看全局变量实时绘图。

## 上电行为
1. 自动把设备地址配成 1（发 0xFF 公共地址）。
2. 设梯形加减速（20 rpm/s，可改宏）。
3. **零点**：
   - 第一次上电：把当前位置记成 0 点（`0xB1`，ZE300 断电保存），并写 STM32 flash 标记；
   - 之后每次上电：自动回 0 点。
4. **自动随机位置测试**：回零后就绪，每 **2 s** 随机给一个绝对位置（±180°），用
   `0xDA` 梯形位置闭环走过去；异步状态机，**不阻塞主循环**。

## LinkScope 变量（全局符号，直接加曲线）
| 变量 | 含义 | 建议绘图 |
|---|---|---|
| `g_tgt_deg` | 目标位置(度) | ✓ |
| `g_now_deg` | 实际位置(度, 0xA3 回读) | ✓ |
| `g_spd_rpm` | 实际速度(rpm) | ✓(看跟不跟得上) |
| `g_tgt_cnt` | 目标位置(counts) | |
| `g_now_cnt` | 实际位置(counts) | |
| `g_st` | 状态 0空闲 1送保持 2松闸 3运动 4到位 | |
| `g_brk` | 抱闸 1闭合 0松开(逻辑原始值) | |
| `g_brk_plot` | 抱闸绘图值 **合=50 / 松=0**，台阶明显 | ✓ |
| `g_auto` | 自动随机开关(改 0 可停) | |
| `g_run_ms` | 开机运行毫秒 | ✓(时间轴) |
| `g_rx_cnt` | 收到 CAN 帧数 | |
| `g_move_cycles` / `g_move_timeouts` | 完成次数 / 无回包超时次数 | |
| `g_peak_rpm` | 运行以来实测最大\|速度\|(rpm)，**看提速是否生效** | ✓ |
| `g_cfg_acc_x100` / `g_cfg_dec_x100` | 驱动回读的加速度/减速度(x0.01rpm/s) | 确认参数生效 |
| `g_cfg_vmax_x100` / `g_cfg_iq_ma` | 回读的位置最大速度/最大电流 | |
| `g_base_cnt` | 零点基准 count（开机回零到位后锁定）；目标/实际角度都相对它显示，不会越滚越大 | | |
| `pid_pos_kp` | **可写**：位置环 Kp（0xB6），驱动当前值会先拷进来，改它=在线改驱动 | ✓ |
| `pid_pos_ki` | **可写**：位置环 Ki（0xB7） | ✓ |
| `pid_vel_kp` | **可写**：速度环 Kp（0xB8） | ✓ |
| `pid_vel_ki` | **可写**：速度环 Ki（0xB9） | ✓ |
| `pos_cmd_deg` | **可写**：手动目标角度(度, 相对零点)——改它电机就走过去(已关自动随机) | ✓ |
| `pid_pos_lim_rpm` | **可写**：位置环输出限＝位置最大速度 rpm（0xB2） | ✓ |
| `pid_vel_lim_A` | **可写**：速度环输出限＝最大电流 A（0xB3） | ✓ |
| `cur_pos_kp/ki`、`cur_vel_kp/ki`、`cur_pos_lim_rpm`、`cur_vel_lim_A` | **只读**：驱动当前实际那 6 个值，每次回读实时刷新，用于反馈确认 | ✓ |

### 在线调 PID（不用串口，用 LinkScope 写变量）
1. 上电后等约 1~2s，`pid_pos_kp/ki/vel_kp/ki` 会自动变成**驱动当前值**（这是安全起点，不会误覆盖成 0）；
2. 在 LinkScope 里**双击直接改**某个 pid 变量的值（例如位置环 Kp 调小来止震）；
3. 程序每 300ms 轮询，检测到不一致就把新值写进驱动（0xB6~0xB9，float），写完后回读对齐；
4. 看 `g_now_deg/g_tgt_deg/g_spd_rpm` 的反应再改下一个参数。

> 提示：这些 PID **断电不保存**，每次上电从驱动默认开始。方向感：到位震荡大→**位置环 Kp 调小**；有稳态误差/慢爬→适当加 Ki；速度跟不平→调速度环 Kp/Ki。

> 状态量都是全局且 `volatile`，每主循环(5ms)刷新；编译已开 Debug 信息。

## 还没接电机时的表现（只上电）
发目标后若一直没有任何 CAN 回包，3 s（`NO_ACK_TIMEOUT_MS`）判定"没接电机"，
状态回空闲并继续计时给下一个随机目标。此时你会看到：
- `g_tgt_deg` 每 ~几秒跳一个随机值；
- `g_rx_cnt` 停在 0（无回包）；
- `g_st` 在 0~4 之间循环；
- 接上电机后 `g_now_deg` 会跟随 `g_tgt_deg`，`g_rx_cnt`/`g_now_cnt` 开始增长。

## 编译烧录
1. Keil 打开 `MDK-ARM\test_feedback.uvprojx`（STM32F427IIH，AC5）。
2. F7 编译（应 0 错误 0 警告），产物 `MDK-ARM\test_feedback\test_feedback.hex`。
3. SWD 烧录 → 上电 → LinkScope 连接后把上面变量拉曲线。

## 关键宏（`Core/Src/main.c` 顶部）
- `AUTO_INTERVAL_MS`=2000 给位置间隔；`AUTO_RANGE_DEG`=180 随机范围；
- `ACCEL_RPMPS/DECEL_RPMPS`=150 梯形加/减速度（0xD0/D1；HT6010输出上限~93rpm，太大起停会震）；
- `POS_MAX_RPM`=90 位置模式最大速度(0xB2)（本型号减速36:1，输出最高~93rpm，设更大无效）；
- `IQ_LIMIT_A`=8.8 **位置/速度模式最大电流 A(0xB3)**（HT6010额定8.8A；短时可放宽到12，别长期超）；
- `HOLD_ZERO_MS`=5000 **上电回零到位后先在 0 点保持 5s**，再开始随机给位置；
- `SETTLE_TOL_CNT`=16 到位容差(≈0.35°)；`NO_ACK_TIMEOUT_MS`=3000 无回包判定；
- `ARRIVE_MAX_MS`=30000 有反馈但久不到位兜底。
- 想停自动随机：改 `ENABLE_AUTO_RANDOM` 0；或运行时在 LinkScope 里把 `g_auto` 写 0。

## 提醒
- 重物正式跑前用官方上位机把 **最大电流(0xB3)** 限好，先小角度小加减速试；
- 换电机/机构想重设 0 点：删掉 STM32 里那个标记即可（烧录时如果 Keil 是
  "Erase Full Chip"，烧一次就会回到"第一次上电设零点"状态；否则见 main.c 的
  `PERSIST_ADDR`，把该地址擦一下或全片擦）。
- 垂直重物：电机没接/刹车时序没验证前别挂负载；`g_brk` 到 1(合闸)后负载才安全。
