#include "tune.h"
#include "balance.h"

/* =====================================================================
 * 无线串口调参（UART_2，P10.5 发 / P10.6 收，115200）
 *
 * 上位机发命令（每条以 \n 或 \r 结尾）：
 *   P300      设置 KP
 *   D8        设置 KD
 *   Z2.5      设置平衡零点
 *   V0.02     设置速度环 KP
 *   W0.005    设置速度环 KI
 *   S1 / S-1  设置速度环叠加符号
 *   ?         立刻回传一帧状态
 *
 * 下位机每 100ms 自动回传：
 *   $ang=1.23,P=650.00,D=50.00,Z=1.78,duty=120,spd=3,sl=1.00,sr=2.00,corr=0.00,
 *    v=0.02,w=0.00,tgt=1.78,sg=-1.00,f=0,arm=1
 * ===================================================================== */

static uint8 g_dump_req = 0;    /* A3：立即导出 */
static uint8 g_rec_kind = 0;    /* 0 = 诊断记录(A1)；1 = 运行时记录(A2) */

#define TUNE_UART           (UART_2)
#define TUNE_BAUD           (115200)
#define TUNE_TX_PIN         (UART2_TX_P10_5)
#define TUNE_RX_PIN         (UART2_RX_P10_6)

static char   s_buf[64];      /* 只放发送用的小字符串 */

static char   s_rx[64];       /* 只放收到的命令行，和发送分开 */

static uint8  s_len = 0;

/* ---------- 发送小工具（不用 printf，避免占用调试串口） ---------- */
static void tx_str(const char *s)
{
    uart_write_string(TUNE_UART, s);
}

static void tx_int(int32 v)
{
    char tmp[12];
    uint8 i = 0;
    uint8 neg = 0;
    uint8 n = 0;

    if (v < 0) { neg = 1; v = -v; }
    if (v == 0) { tmp[i++] = '0'; }
    while (v > 0) { tmp[i++] = (char)('0' + (v % 10)); v /= 10; }
    if (neg) tmp[i++] = '-';
    while (i > 0) { s_buf[n++] = tmp[--i]; }
    s_buf[n] = '\0';
    tx_str(s_buf);
}

/* 两位小数输出，例如 300.00 */
static void tx_float2(float v)
{
    char tmp[16];
    uint8 i = 0;
    uint8 n = 0;
    int32 whole;
    int32 frac;
    uint8 neg = 0;

    if (v < 0.0f) { neg = 1; v = -v; }
    whole = (int32)v;
    frac  = (int32)((v - (float)whole) * 100.0f + 0.5f);
    if (frac >= 100) { frac -= 100; whole += 1; }

    if (whole == 0) tmp[i++] = '0';
    while (whole > 0) { tmp[i++] = (char)('0' + (whole % 10)); whole /= 10; }
    if (neg) tmp[i++] = '-';
    while (i > 0) { s_buf[n++] = tmp[--i]; }
    s_buf[n++] = '.';
    s_buf[n++] = (char)('0' + (frac / 10));
    s_buf[n++] = (char)('0' + (frac % 10));
    s_buf[n] = '\0';
    tx_str(s_buf);
}

/* 速度环小增益输出四位小数，避免 0.004 被显示成 0.00。 */
static void tx_float4(float v)
{
    char tmp[16];
    uint8 i = 0, n = 0, neg = 0;
    int32 whole, frac;
    if (v < 0.0f) { neg = 1; v = -v; }
    whole = (int32)v;
    frac = (int32)((v - (float)whole) * 10000.0f + 0.5f);
    if (frac >= 10000) { frac -= 10000; whole += 1; }
    if (whole == 0) tmp[i++] = '0';
    while (whole > 0) { tmp[i++] = (char)('0' + (whole % 10)); whole /= 10; }
    if (neg) tmp[i++] = '-';
    while (i > 0) s_buf[n++] = tmp[--i];
    s_buf[n++] = '.';
    s_buf[n++] = (char)('0' + frac / 1000);
    s_buf[n++] = (char)('0' + (frac / 100) % 10);
    s_buf[n++] = (char)('0' + (frac / 10) % 10);
    s_buf[n++] = (char)('0' + frac % 10);
    s_buf[n] = '\0';
    tx_str(s_buf);
}

/* 简单浮点解析："-12.34" -> -12.34，返回是否成功 */
static uint8 parse_float(const char *s, float *out)
{
    float v = 0.0f;
    float f = 0.1f;
    uint8 neg = 0;
    uint8 dot = 0;
    uint8 has = 0;

    while (*s == ' ') s++;
    if (*s == '-') { neg = 1; s++; }
    else if (*s == '+') { s++; }

    while (*s)
    {
        char c = *s;
        if (c >= '0' && c <= '9')
        {
            has = 1;
            if (!dot) v = v * 10.0f + (float)(c - '0');
            else      { v += (float)(c - '0') * f; f *= 0.1f; }
        }
        else if (c == '.' && !dot) { dot = 1; }
        else break;
        s++;
    }
    if (!has) return 0;
    *out = neg ? -v : v;
    return 1;
}

/* ---------- 命令处理 ---------- */
static void handle_line(char *line)
{
    char cmd = line[0];
    float v;

    if (cmd == 'P' || cmd == 'p')
    {
        if (parse_float(&line[1], &v)) { balance_set_kp(v); tx_str("OK P\r\n"); }
    }
    else if (cmd == 'D' || cmd == 'd')
    {
        if (parse_float(&line[1], &v)) { balance_set_kd(v); tx_str("OK D\r\n"); }
    }
    else if (cmd == 'Z' || cmd == 'z')
    {
        if (parse_float(&line[1], &v)) { balance_set_zero(v); tx_str("OK Z\r\n"); }
    }
    else if (cmd == 'V' || cmd == 'v')
    {
        if (parse_float(&line[1], &v)) { balance_set_speed_kp(v); tx_str("OK V\r\n"); }
    }
    else if (cmd == 'W' || cmd == 'w')
    {
        if (parse_float(&line[1], &v)) { balance_set_speed_ki(v); tx_str("OK W\r\n"); }
    }
    else if (cmd == 'S' || cmd == 's')
    {
        if (parse_float(&line[1], &v)) { balance_set_speed_sign(v); tx_str("OK S\r\n"); }
    }
    else if (cmd == 'X' || cmd == 'x')
    {
        if (parse_float(&line[1], &v)) { balance_set_pulse_duty(v); tx_str("OK X\r\n"); }
    }
    else if (cmd == 'A' || cmd == 'a')
    {
        if (parse_float(&line[1], &v))
        {
            if (v >= 3.0f)
            {
                g_dump_req = 1U;
                tx_str("OK A3 (dump now)\r\n");
            }
            else if (v >= 2.0f)
            {
                balance_set_diag(0);
                balance_set_run_rec(1);
                g_rec_kind = 1U;
                tx_str("OK A2 (run rec, 20ms x 800 = 16s)\r\n");
            }
            else if (v >= 1.0f)
            {
                balance_set_run_rec(0);
                balance_set_diag(1);
                g_rec_kind = 0U;
                tx_str("OK A1 (diag rec, motors OFF)\r\n");
            }
            else
            {
                balance_set_diag(0);
                balance_set_run_rec(0);
                tx_str("OK A0 (record off)\r\n");
            }
        }
    }
    else if (cmd == 'G' || cmd == 'g')
    {
        if (parse_float(&line[1], &v)) { balance_set_gain_r(v); tx_str("OK GR\r\n"); }
    }
    else if (cmd == 'O' || cmd == 'o')
    {
        if (parse_float(&line[1], &v)) { balance_set_offset_r(v); tx_str("OK OR\r\n"); }
    }
    else if (cmd == 'K' || cmd == 'k')
    {
        if (parse_float(&line[1], &v)) { balance_set_deadzone(v); tx_str("OK K\r\n"); }
    }
    else if (cmd == 'R' || cmd == 'r')
    {
        if (balance_clear_rearm()) tx_str("OK R (rearm allowed)\r\n");
        else                       tx_str("ERR R (calib not valid)\r\n");
    }
    else if (cmd == 'B' || cmd == 'b')
    {
        balance_restart_gyro_calib();
        tx_str("OK B (hold still 2s)\r\n");
    }
    else if (cmd == '?')
    {
        tune_send_telemetry();
    }
}

/* 打印一行 CFG，保证记录能还原每帧控制计算 */
static void dump_cfg(void)
{
    tx_str("CFG,Px10=");   tx_int((int32)(balance_get_kp()    * 10.0f));
    tx_str(",Dx10=");      tx_int((int32)(balance_get_kd()    * 10.0f));
    tx_str(",Zx100=");     tx_int((int32)(balance_get_zero()  * 100.0f));
    tx_str(",Vx1000=");    tx_int((int32)(balance_get_speed_kp()   * 1000.0f));
    tx_str(",Sx100=");     tx_int((int32)(balance_get_speed_sign() * 100.0f));
    tx_str("\r\n");
}

/* 诊断记录 / 运行时记录共用：缓存满后一次性打印 CSV */
static void acc_dump_body(uint8 run_kind)
{
    uint16 i, n;
    const balance_acc_row_t *r;

    r = balance_acc_rows();
    n = balance_acc_row_count();

    tx_str((run_kind != 0U) ? "RUNREC n=" : "ACCREC n=");
    tx_int((int32)n);
    tx_str("\r\n");
    dump_cfg();
    tx_str("t_ms,ang_x100,aa_x100,ga_x100,ac_x1000,rate_x10,ax,ay,az,sl,sr,duty,raw,tgt_x100,fault,arm\r\n");

    for (i = 0; i < n; i++)
    {
        tx_int((int32)r[i].t_ms);      tx_str(",");
        tx_int((int32)r[i].ang_x100);  tx_str(",");
        tx_int((int32)r[i].aa_x100);   tx_str(",");
        tx_int((int32)r[i].ga_x100);   tx_str(",");
        tx_int((int32)r[i].ac_x1000);  tx_str(",");
        tx_int((int32)r[i].rate_x10);  tx_str(",");
        tx_int((int32)r[i].ax_raw);    tx_str(",");
        tx_int((int32)r[i].ay_raw);    tx_str(",");
        tx_int((int32)r[i].az_raw);    tx_str(",");
        tx_int((int32)r[i].sl);        tx_str(",");
        tx_int((int32)r[i].sr);        tx_str(",");
        tx_int((int32)r[i].duty);      tx_str(",");
        tx_int((int32)r[i].raw);       tx_str(",");
        tx_int((int32)r[i].tgt_x100);  tx_str(",");
        tx_int((int32)r[i].fault);     tx_str(",");
        tx_int((int32)r[i].armed);     tx_str("\r\n");
    }
    tx_str((run_kind != 0U) ? "RUNREC end\r\n" : "ACCREC end\r\n");

    balance_acc_clear();
}

static void acc_dump(void)
{
    uint8 ready = 0;

    if (g_dump_req && balance_acc_row_count() > 0U)            ready = 1U;
    else if (g_rec_kind == 0U && balance_diag_is_done())       ready = 1U;
    else if (g_rec_kind == 1U && balance_run_rec_full())       ready = 1U;

    if (!ready) return;

    g_dump_req = 0;
    acc_dump_body(g_rec_kind);
}

void tune_pulse_dump(void)
{
    uint16 i, n;
    const balance_pulse_row_t *r;

    acc_dump();

    if (!balance_pulse_is_done()) return;

    r = balance_pulse_rows();
    n = balance_pulse_row_count();

    tx_str("PULSE8 duty=");
    tx_int((int32)balance_get_pulse_duty());
    tx_str(" n=");
    tx_int((int32)n);
    tx_str("\r\n");
    tx_str("t_ms,phase,ang_x100,rate_raw_x10,rate_x10,u_prev,u_cmd,sl,sr,reason,acc_ang_x100,gyro_angle_x100,acc_corr_x100\r\n");

    for (i = 0; i < n; i++)
    {
        tx_int((int32)r[i].t_ms);           tx_str(",");
        tx_int((int32)r[i].phase);          tx_str(",");
        tx_int((int32)r[i].ang_x100);       tx_str(",");
        tx_int((int32)r[i].rate_raw_x10);   tx_str(",");
        tx_int((int32)r[i].rate_x10);       tx_str(",");
        tx_int((int32)r[i].u_prev);         tx_str(",");
        tx_int((int32)r[i].u_cmd);          tx_str(",");
        tx_int((int32)r[i].sl);             tx_str(",");
        tx_int((int32)r[i].sr);             tx_str(",");
        tx_int((int32)r[i].reason);         tx_str(",");
        tx_int((int32)r[i].acc_ang_x100);   tx_str(",");
        tx_int((int32)r[i].gyro_angle_x100);tx_str(",");
        tx_int((int32)r[i].acc_corr_x100);  tx_str("\r\n");
    }
    tx_str("PULSE8 end\r\n");

    balance_pulse_clear();
}

void tune_init(void)
{
    uart_init(TUNE_UART, TUNE_BAUD, TUNE_TX_PIN, TUNE_RX_PIN);
    s_len = 0;
}

void tune_poll(void)
{
    uint8 c;

    while (uart_query_byte(TUNE_UART, &c))
    {
        if (c == '\n' || c == '\r')
        {
            if (s_len > 0)
            {
                s_rx[s_len] = '\0';
                handle_line(s_rx);
                s_len = 0;
            }
        }
        else if (s_len < (uint8)(sizeof(s_rx) - 1))
        {
            s_rx[s_len++] = (char)c;
        }
        else
        {
            s_len = 0;      /* 超长丢弃 */
        }
    }
}

void tune_send_telemetry(void)
{
    float ang, kp, kd, zero, speed, sl, sr, corr, skp, ski, target, sign, rate;
    float acc_ang_d, gyro_ang_d, acc_corr_d, bias_d, raw_d;
    int16 duty;
    uint8 fault, armed, mode;
    uint32 irq_state;

    /* CPU0 控制 ISR 与本函数共享状态：先短时关中断取快照，再恢复后发送。
     * 使用逐飞 interrupt_global_disable/enable 保存并恢复原中断状态。
     * 关中断区内绝不进行 UART 发送。
     */
    irq_state = interrupt_global_disable();
    ang = balance_get_angle();
    kp = balance_get_kp();
    kd = balance_get_kd();
    zero = balance_get_zero();
    duty = balance_get_duty();
    speed = balance_get_speed();
    sl = balance_get_sl();
    sr = balance_get_sr();
    corr = balance_get_corr();
    skp = balance_get_speed_kp();
    ski = balance_get_speed_ki();
    target = balance_get_target();
    sign = balance_get_speed_sign();
    fault = balance_get_fault();
    armed = balance_is_armed();
    mode = balance_get_test_mode();
    rate = balance_get_rate();
    acc_ang_d  = balance_get_acc_ang();
    gyro_ang_d = balance_get_gyro_angle();
    acc_corr_d = balance_get_acc_corr();
    bias_d     = balance_get_gyro_bias();
    raw_d      = balance_get_raw_out();
    interrupt_global_enable(irq_state);

    tx_str("$ang="); tx_float2(ang);
    tx_str(",P="); tx_float2(kp);
    tx_str(",D="); tx_float2(kd);
    tx_str(",Z="); tx_float2(zero);
    tx_str(",duty="); tx_int(duty);
    tx_str(",spd="); tx_float2(speed);
    tx_str(",sl="); tx_float2(sl);
    tx_str(",sr="); tx_float2(sr);
    tx_str(",gr="); tx_float2(balance_get_gain_r());
    tx_str(",or="); tx_float2(balance_get_offset_r());
    tx_str(",dz="); tx_float2(balance_get_deadzone());
    tx_str(",corr="); tx_float2(corr);
    tx_str(",v="); tx_float4(skp);
    tx_str(",w="); tx_float4(ski);
    tx_str(",si="); tx_float4(balance_get_speed_i());
    tx_str(",tgt="); tx_float2(target);
    tx_str(",sg="); tx_float2(sign);
    tx_str(",f="); tx_int((int32)fault);
    tx_str(",arm="); tx_int((int32)armed);
    tx_str(",rate="); tx_float2(rate);
    tx_str(",mode="); tx_int((int32)mode);
    tx_str(",build="); tx_str(balance_get_build());
    tx_str(",aa=");   tx_float2(acc_ang_d);
    tx_str(",ga=");   tx_float2(gyro_ang_d);
    tx_str(",ac=");   tx_float4(acc_corr_d);
    tx_str(",gb=");   tx_float2(bias_d);
    tx_str(",raw=");  tx_float2(raw_d);
tx_str("\r\n");
}
