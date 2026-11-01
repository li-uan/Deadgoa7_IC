/* ============================================================
 *  vision_process.c
 *  适配：MT9V034 (灰度) + 两轮差速平衡车 + TC264 双核架构
 * ============================================================ */

#include "vision.h"
#include <math.h>
#include <string.h>

/* 【接口1】声明逐飞库摄像头数组 (请确保你的库中名字一致) */
extern uint8_t mt9v03x_image[120][188]; 
extern uint8_t mt9v03x_finish_flag;

/* 定义双核共享数据结构，供 CPU0 读取 */
vision_t g_vision_data = {0, 0, 0};

/* ============================================================
 * 尺寸与全局缓存
 * ============================================================ */
#define IMG_W   188
#define IMG_H   120
#define IPM_W   120
#define IPM_H   140
#define EW      IPM_W
#define EH      IPM_H

#define gray_img mt9v03x_image 

/* 将视觉算法用到的所有大数组分配到 CPU1 的专属 RAM 中，避免跨核访问卡顿 */
#pragma section all "cpu1_dsram"

uint8_t  ipm_gray [IPM_H][IPM_W];
uint8_t  ipm_bin  [IPM_H][IPM_W];
int16_t left_edge  [IPM_H];
int16_t right_edge [IPM_H];
int16_t center_line[IPM_H];
int16_t center_filt[IPM_H];
int16_t center_final[IPM_H];

static uint8_t map_x[IPM_H][IPM_W];
static uint8_t map_y[IPM_H][IPM_W];

#pragma section all restore

/* ============================================================
 * 系统参数
 * ============================================================ */
#define EDGE_STEP      2      /* 八邻域行步长（隔行加速） */
#define MAX_JUMP       10     /* 中线限幅滤波跳变阈值 */
#define FILTER_WIN     5      /* 中线滑动平均窗口 */
#define TARGET_COL     (EW / 2)
#define CAR_HW         15     /* 车身半宽(IPM像素) */

/* ============================================================
 * 一、逆透视变换 IPM
 * ============================================================ */
/* ⚠️ 警告：此处必须现场标定赛道四个角 */
static const float src_pts[4][2] = {
    {  50.0f,  40.0f },   // 左上(远左)
    { 138.0f,  40.0f },   // 右上(远右)
    {   0.0f, 119.0f },   // 左下(近左)
    { 187.0f, 119.0f }    // 右下(近右)
};
static const float dst_pts[4][2] = {
    {   0.0f,        0.0f },
    { IPM_W - 1,     0.0f },
    {   0.0f,    IPM_H - 1 },
    { IPM_W - 1, IPM_H - 1 }
};

static int solve_homography(const float src[4][2], const float dst[4][2], float H[3][3])
{
    float A[8][8], b[8];
    for (int i = 0; i < 4; i++) {
        float sx = src[i][0], sy = src[i][1];
        float dx = dst[i][0], dy = dst[i][1];
        A[i*2][0]=sx; A[i*2][1]=sy; A[i*2][2]=1;
        A[i*2][3]=0;  A[i*2][4]=0;  A[i*2][5]=0;
        A[i*2][6]=-sx*dx; A[i*2][7]=-sy*dx; b[i*2]=dx;
        A[i*2+1][0]=0;  A[i*2+1][1]=0;  A[i*2+1][2]=0;
        A[i*2+1][3]=sx; A[i*2+1][4]=sy; A[i*2+1][5]=1;
        A[i*2+1][6]=-sx*dy; A[i*2+1][7]=-sy*dy; b[i*2+1]=dy;
    }
    for (int i = 0; i < 8; i++) {
        int mr = i;
        for (int r = i + 1; r < 8; r++)
            if (fabsf(A[r][i]) > fabsf(A[mr][i])) mr = r;
        if (mr != i) {
            for (int c = 0; c < 8; c++) { float t=A[i][c];A[i][c]=A[mr][c];A[mr][c]=t; }
            float t = b[i]; b[i] = b[mr]; b[mr] = t;
        }
        if (fabsf(A[i][i]) < 1e-8f) return -1;
        for (int r = 0; r < 8; r++) {
            if (r == i) continue;
            float f = A[r][i] / A[i][i];
            for (int c = i; c < 8; c++) A[r][c] -= f * A[i][c];
            b[r] -= f * b[i];
        }
    }
    float h[8];
    for (int i = 0; i < 8; i++) h[i] = b[i] / A[i][i];
    H[0][0]=h[0];H[0][1]=h[1];H[0][2]=h[2];
    H[1][0]=h[3];H[1][1]=h[4];H[1][2]=h[5];
    H[2][0]=h[6];H[2][1]=h[7];H[2][2]=1.0f;
    return 0;
}

static void ipm_init(void)
{
    float H[3][3];
    if (solve_homography(dst_pts, src_pts, H) != 0) {
        for (int y = 0; y < IPM_H; y++)
            for (int x = 0; x < IPM_W; x++) {
                map_x[y][x] = (uint8_t)(x * IMG_W / IPM_W);
                map_y[y][x] = (uint8_t)(y * IMG_H / IPM_H);
            }
        return;
    }
    for (int v = 0; v < IPM_H; v++) {
        for (int u = 0; u < IPM_W; u++) {
            float den = H[2][0]*u + H[2][1]*v + H[2][2];
            if (fabsf(den) < 1e-6f) den = 1e-6f;
            float fx = (H[0][0]*u + H[0][1]*v + H[0][2]) / den;
            float fy = (H[1][0]*u + H[1][1]*v + H[1][2]) / den;
            int ix = (int)(fx + 0.5f);
            int iy = (int)(fy + 0.5f);
            if (ix < 0) ix = 0; if (ix >= IMG_W) ix = IMG_W - 1;
            if (iy < 0) iy = 0; if (iy >= IMG_H) iy = IMG_H - 1;
            map_x[v][u] = (uint8_t)ix;
            map_y[v][u] = (uint8_t)iy;
        }
    }
}

static void ipm_run(void)
{
    for (int v = 0; v < IPM_H; v++) {
        const uint8_t *rx = map_x[v];
        const uint8_t *ry = map_y[v];
        uint8_t       *d  = ipm_gray[v];
        for (int u = 0; u < IPM_W; u++) d[u] = gray_img[ry[u]][rx[u]];
    }
}

/* ============================================================
 * 二、二值化
 * ============================================================ */
static uint8_t otsu_th(const uint8_t *src, uint32_t len)
{
    uint32_t hist[256]; memset(hist, 0, sizeof(hist));
    for (uint32_t i = 0; i < len; i++) hist[src[i]]++;
    uint32_t total = len, sum = 0;
    for (int i = 0; i < 256; i++) sum += (uint32_t)i * hist[i];
    uint32_t sumB = 0, wB = 0; float vMax = 0; uint8_t th = 0;
    for (int t = 0; t < 256; t++) {
        wB += hist[t]; if (wB == 0) continue;
        uint32_t wF = total - wB; if (wF == 0) break;
        sumB += (uint32_t)t * hist[t];
        float mB = (float)sumB / wB;
        float mF = (float)(sum - sumB) / wF;
        float v = (float)wB * wF * (mB - mF) * (mB - mF);
        if (v > vMax) { vMax = v; th = (uint8_t)t; }
    }
    return th;
}

static void binarize(void)
{
    for (int y = 0; y < IPM_H; y++) {
        uint8_t th = otsu_th(ipm_gray[y], IPM_W);
        for (int x = 0; x < IPM_W; x++) ipm_bin[y][x] = (ipm_gray[y][x] > th) ? 255 : 0;
    }
}

/* ============================================================
 * 三、八邻域爬线 + 中线计算
 * ============================================================ */
static const int8_t NB_DR[8] = {-1, -1, -1,  0,  1,  1,  1,  0};
static const int8_t NB_DC[8] = {-1,  0,  1,  1,  1,  0, -1, -1};

static void trace_edge(int sr, int sc, int16_t *edge, int is_left)
{
    for (int i = 0; i < EH; i++) edge[i] = -1;
    int r = sr, c = sc, last = is_left ? 7 : 3;
    while (r >= 0 && r < EH && c >= 0 && c < EW) {
        edge[r] = (int16_t)c;
        int found = 0;
        for (int k = 0; k < 8; k++) {
            int d = is_left ? (last + k) % 8 : (last + 8 - k) % 8;
            if (NB_DR[d] > 0) continue;
            int nr = r + NB_DR[d] * EDGE_STEP;
            int nc = c + NB_DC[d] * EDGE_STEP;
            if (nr < 0 || nr >= EH || nc < 0 || nc >= EW || ipm_bin[nr][nc] != 0) continue;
            if (EDGE_STEP >= 2) {
                int mr = r + NB_DR[d], mc = c + NB_DC[d];
                if (mr >= 0 && mr < EH && mc >= 0 && mc < EW && ipm_bin[mr][mc] != 0) continue;
            }
            int span = r - nr;
            if (span > 1) for (int t = 1; t < span; t++) edge[r - t] = (int16_t)(c + (nc - c) * t / span);
            r = nr; c = nc; last = d; found = 1; break;
        }
        if (!found) break;
    }
}

static void edge_extract(void)
{
    int mid = EW / 2, sr = EH - 3;
    int sl = -1, srr = -1;
    for (int x = mid; x >= 0; x--) if (ipm_bin[sr][x] == 0) { sl = x; break; }
    for (int x = mid; x < EW; x++) if (ipm_bin[sr][x] == 0) { srr = x; break; }
    if (sl  >= 0) trace_edge(sr, sl,  left_edge,  1);
    if (srr >= 0) trace_edge(sr, srr, right_edge, 0);

    for (int y = 0; y < EH; y++) {
        if (left_edge[y] < 0) {
            if (y > 0 && left_edge[y-1] >= 0) left_edge[y] = left_edge[y-1];
            else for (int k = y + 1; k < EH; k++) if (left_edge[k] >= 0) { left_edge[y] = left_edge[k]; break; }
        }
        if (right_edge[y] < 0) {
            if (y > 0 && right_edge[y-1] >= 0) right_edge[y] = right_edge[y-1];
            else for (int k = y + 1; k < EH; k++) if (right_edge[k] >= 0) { right_edge[y] = right_edge[k]; break; }
        }
    }
}

static void calc_center(void)
{
    for (int y = 0; y < EH; y++) {
        int16_t l = left_edge[y], r = right_edge[y];
        if (l >= 0 && r >= 0)      center_line[y] = (l + r) / 2;
        else if (l >= 0)           center_line[y] = l + EW / 4;
        else if (r >= 0)           center_line[y] = r - EW / 4;
        else                       center_line[y] = -1;
    }
    int prev = -1;
    for (int y = 0; y < EH; y++) {
        if (center_line[y] >= 0) {
            if (prev >= 0 && y - prev > 1) {
                int16_t v1 = center_line[prev], v2 = center_line[y];
                for (int k = prev + 1; k < y; k++)
                    center_line[k] = (int16_t)(v1 + (v2 - v1) * (k - prev) / (y - prev));
            }
            prev = y;
        }
    }
    if (prev >= 0) for (int y = prev + 1; y < EH; y++) center_line[y] = center_line[prev];
    for (int y = 0; y < EH; y++) if (center_line[y] >= 0) { for (int k = 0; k < y; k++) center_line[k] = center_line[y]; break; }

    for (int y = 1; y < EH; y++) {
        if (center_line[y] < 0 || center_line[y-1] < 0) continue;
        int diff = center_line[y] - center_line[y-1];
        if (diff > MAX_JUMP || diff < -MAX_JUMP) center_line[y] = center_line[y-1];
    }
    int half = FILTER_WIN / 2;
    for (int y = 0; y < EH; y++) {
        int sum = 0, cnt = 0;
        for (int k = -half; k <= half; k++) {
            int yy = y + k;
            if (yy < 0 || yy >= EH || center_line[yy] < 0) continue;
            sum += center_line[yy]; cnt++;
        }
        center_filt[y] = cnt ? (int16_t)(sum / cnt) : center_line[y];
    }
}

/* ============================================================
 * 四、元素识别与场景分类
 * ============================================================ */
typedef enum { RD_STRAIGHT, RD_LEFT, RD_RIGHT, RD_S, RD_CROSS } RoadType_t;
static RoadType_t road_type = RD_STRAIGHT;

static void classify_road(void)
{
    int cb = center_filt[EH - 5], ct = center_filt[30], cm = center_filt[EH / 2];
    int16_t s = (cb >= 0 && ct >= 0) ? (ct - cb) : 0;
    int16_t curv = (cb >= 0 && ct >= 0 && cm >= 0) ? (cm - (cb + (ct - cb) / 2)) : 0;
    int ll = 0, rl = 0;
    for (int y = 15; y < EH - 15; y++) {
        if (left_edge[y]  < 0) ll++;
        if (right_edge[y] < 0) rl++;
    }
    if (ll > 30 && rl > 30) { road_type = RD_CROSS; return; }
    if (abs(s) < 6 && abs(curv) < 8) { road_type = RD_STRAIGHT; return; }
    int16_t sb = (cb >= 0 && cm >= 0) ? (cm - cb) : 0;
    int16_t st = (cm >= 0 && ct >= 0) ? (ct - cm) : 0;
    if ((sb > 8 && st < -8) || (sb < -8 && st > 8)) { road_type = RD_S; return; }
    if (s > 20) road_type = RD_RIGHT;
    else if (s < -20) road_type = RD_LEFT;
    else road_type = (s > 0) ? RD_RIGHT : RD_LEFT;
}

/* --- 环岛状态机 --- */
typedef enum { RA_NONE, RA_FOUND, RA_ENTERING, RA_INSIDE, RA_LEAVING, RA_DONE } RaState_t;
typedef enum { RA_DIR_NONE, RA_DIR_L, RA_DIR_R } RaDir_t;
static RaState_t ra_state = RA_NONE;
static RaDir_t   ra_dir   = RA_DIR_NONE;
static int       ra_timer = 0;

static int calc_bulge(const int16_t *edge, int y1, int y2, int is_left)
{
    int sm = 0, cm = 0;
    for (int y = y1; y <= y2; y++) if (edge[y] >= 0) { sm += edge[y]; cm++; }
    if (cm < 3) return 0;
    int am = sm / cm;
    int sb = 0, cb = 0;
    for (int y = EH-10; y < EH; y++) if (edge[y] >= 0) { sb += edge[y]; cb++; }
    if (cb < 3) return 0;
    int ab = sb / cb;
    return is_left ? (am - ab) : (ab - am);
}

static void roundabout_process(void)
{
    ra_timer++;
    switch (ra_state) {
    case RA_NONE:
        int bl = calc_bulge(left_edge,  20, 55, 1);
        int br = calc_bulge(right_edge, 20, 55, 0);
        if (bl > 25 && abs(br) < 12)      { ra_state = RA_FOUND; ra_dir = RA_DIR_L; ra_timer = 0; }
        else if (br > 25 && abs(bl) < 12) { ra_state = RA_FOUND; ra_dir = RA_DIR_R; ra_timer = 0; }
        break;
    case RA_FOUND:
        if (ra_timer > 3) {
            int l = left_edge[EH-5], r = right_edge[EH-5];
            if (l >= 0 && r >= 0 && (r - l) > EW * 2 / 3) { ra_state = RA_ENTERING; ra_timer = 0; }
            else if (ra_timer > 30) { ra_state = RA_NONE; ra_dir = RA_DIR_NONE; }
        }
        break;
    case RA_ENTERING:
        if (ra_dir == RA_DIR_L) {
            int16_t base = left_edge[EH-5] >= 0 ? left_edge[EH-5] : EW/2;
            for (int y = 0; y < EH; y++) if (right_edge[y] < 0) right_edge[y] = base + EW/4;
        } else {
            int16_t base = right_edge[EH-5] >= 0 ? right_edge[EH-5] : EW/2;
            for (int y = 0; y < EH; y++) if (left_edge[y] < 0) left_edge[y] = base - EW/4;
        }
        int ll = 0, rl = 0;
        for (int y = 30; y < EH-30; y++) {
            if (left_edge[y]  < 0) ll++;
            if (right_edge[y] < 0) rl++;
        }
        if ((ra_dir == RA_DIR_L && rl > 20) || (ra_dir == RA_DIR_R && ll > 20)) { ra_state = RA_INSIDE; ra_timer = 0; }
        else if (ra_timer > 40) { ra_state = RA_NONE; ra_dir = RA_DIR_NONE; }
        break;
    case RA_INSIDE:
        for (int y = 0; y < EH; y++) {
            if (ra_dir == RA_DIR_L && left_edge[y] >= 0) center_final[y] = left_edge[y] + EW/8;
            else if (ra_dir == RA_DIR_R && right_edge[y] >= 0) center_final[y] = right_edge[y] - EW/8;
        }
        int valid = 0;
        for (int y = 40; y < EH-20; y++) if (left_edge[y] >= 0 && right_edge[y] >= 0) valid++;
        if (ra_timer > 15 && valid > 40) { ra_state = RA_LEAVING; ra_timer = 0; }
        break;
    case RA_LEAVING:
        if (ra_timer > 10) { ra_state = RA_DONE; ra_timer = 0; }
        break;
    case RA_DONE:
        if (ra_timer > 50) { ra_state = RA_NONE; ra_dir = RA_DIR_NONE; ra_timer = 0; }
        break;
    }
    if (ra_state != RA_NONE && ra_state != RA_DONE) g_vision_data.elements |= BIT_ROUND;
}

/* --- 障碍物状态机 --- */
typedef enum { OB_NONE, OB_DET, OB_CONF, OB_AVOID, OB_RECOVER, OB_DONE } ObState_t;
typedef enum { OB_DIR_NONE, OB_DIR_L, OB_DIR_R } ObDir_t;
static ObState_t ob_state = OB_NONE;
static ObDir_t   ob_dir   = OB_DIR_NONE;
static int       ob_timer = 0;
static int16_t   ob_col = -1, ob_row = -1;

static int find_dark_seg(int y, int16_t *sl, int16_t *sr)
{
    int le = left_edge[y], re = right_edge[y];
    if (le < 0 || re < 0) return 0;
    int xs = le + 3, xe = re - 3;
    if (xs >= xe) return 0;
    const uint8_t *row = ipm_bin[y];
    int bl = 0, bl_l = -1, bl_r = -1, cl = 0, cl_l = -1;
    for (int x = xs; x <= xe; x++) {
        if (row[x] == 0) { if (cl == 0) cl_l = x; cl++; }
        else { if (cl > bl) { bl = cl; bl_l = cl_l; bl_r = x - 1; } cl = 0; }
    }
    if (cl > bl) { bl = cl; bl_l = cl_l; bl_r = xe; }
    if (bl >= 6 && bl <= 60) { *sl = bl_l; *sr = bl_r; return bl; }
    return 0;
}

static int obstacle_detect(void)
{
    int dr = 0, sc = 0, xr = 0, mr = 0, found = 0;
    for (int y = 40; y <= EH - 15; y++) {
        int16_t sl, sr;
        if (find_dark_seg(y, &sl, &sr) > 0) {
            if (!found) { mr = y; found = 1; }
            xr = y; dr++; sc += (sl + sr) / 2;
        }
    }
    if (dr < 5) return 0;
    int tot_w = 0, cnt = 0;
    for (int y = mr; y <= xr; y++) {
        int16_t sl, sr;
        if (find_dark_seg(y, &sl, &sr) > 0) { tot_w += sr - sl; cnt++; }
    }
    if ((cnt ? tot_w / cnt : 0) * (xr - mr + 1) < 60) return 0;
    ob_col = (int16_t)(sc / dr); ob_row = (int16_t)xr;
    return 1;
}

static void obstacle_process(void)
{
    ob_timer++;
    switch (ob_state) {
    case OB_NONE:
        if (obstacle_detect()) { ob_state = OB_DET; ob_timer = 0; }
        break;
    case OB_DET:
        if (ob_timer > 2) {
            if (obstacle_detect()) { ob_state = OB_CONF; ob_timer = 0; }
            else { ob_state = OB_NONE; ob_col = -1; ob_row = -1; }
        }
        break;
    case OB_CONF:
        int y = (ob_row >= 0 && ob_row < EH) ? ob_row : (EH - 20);
        int l = left_edge[y], r = right_edge[y];
        if (l < 0 || r < 0) ob_dir = (ob_col < EW/2) ? OB_DIR_R : OB_DIR_L;
        else {
            int sl = ob_col - l, sr = r - ob_col;
            if (sl < CAR_HW * 2) ob_dir = OB_DIR_R;
            else if (sr < CAR_HW * 2) ob_dir = OB_DIR_L;
            else ob_dir = (sl > sr) ? OB_DIR_L : OB_DIR_R;
        }
        if (ob_timer > 2) { ob_state = OB_AVOID; ob_timer = 0; }
        break;
    case OB_AVOID:
        if (ob_dir != OB_DIR_NONE) {
            int off_max = (ob_dir == OB_DIR_L) ? -20 : 20;
            for (int yy = 0; yy < EH; yy++) {
                if (center_final[yy] < 0) continue;
                float w = (yy >= ob_row) ? 1.0f : (1.0f - (float)(ob_row - yy) / 40.0f);
                if (w > 0) center_final[yy] += (int16_t)(off_max * w);
            }
        }
        if (ob_row > EH - 20 || ob_timer > 60) { ob_state = OB_RECOVER; ob_timer = 0; }
        break;
    case OB_RECOVER:
        if (ob_timer > 15) { ob_state = OB_DONE; ob_timer = 0; }
        break;
    case OB_DONE:
        if (ob_timer > 40) { ob_state = OB_NONE; ob_dir = OB_DIR_NONE; ob_col = -1; ob_row = -1; ob_timer = 0; }
        break;
    }
    if (ob_state == OB_AVOID) g_vision_data.elements |= BIT_OBS;
}

/* --- 坡道与斑马线检测 --- */
static uint8_t slope_state = 0; 
static uint8_t zebra_state = 0; 

static void slope_detect(void)
{
    uint32_t st = 0, sb = 0;
    for (int y = 0; y < EH/2; y++) for (int x = 0; x < EW; x++) st += ipm_gray[y][x];
    for (int y = EH/2; y < EH; y++) for (int x = 0; x < EW; x++) sb += ipm_gray[y][x];
    st /= (EH/2 * EW); sb /= (EH/2 * EW);
    int diff = (int)sb - (int)st;
    if (diff > 50) slope_state = 1; else if (diff < -50) slope_state = 2; else slope_state = 0;
    if (slope_state != 0) g_vision_data.elements |= BIT_SLOPE;
}

static void zebra_detect(void)
{
    int cnt[IPM_H], valid[IPM_H], avg_w = 0, tcnt = 0;
    for (int y = 0; y < EH; y++) {
        cnt[y] = 0; valid[y] = 0;
        int l = left_edge[y], r = right_edge[y];
        if (l < 0 || r < 0 || (r - l) < 20) continue;
        int c = 0;
        for (int x = l + 2; x < r - 2; x++) if (ipm_bin[y][x] == 0) c++;
        cnt[y] = c; valid[y] = 1; avg_w += (r - l); tcnt++;
    }
    if (tcnt == 0) { zebra_state = 0; return; }
    avg_w /= tcnt;
    int stripes = 0, state = 0;
    for (int y = 0; y < EH; y++) {
        if (!valid[y]) continue;
        int s = (cnt[y] > avg_w / 2) ? 1 : (cnt[y] < avg_w / 5 ? 0 : -1);
        if (s < 0) continue;
        if (s != state) { if (s == 1) stripes++; state = s; }
    }
    zebra_state = (stripes >= 3) ? 1 : 0;
    if (zebra_state) g_vision_data.elements |= BIT_ZEBRA;
}

/* ============================================================
 * 五、综合决策与误差获取
 * ============================================================ */
static void calc_center_by_scene(void)
{
    classify_road();
    calc_center();
    for (int y = 0; y < EH; y++) if (center_filt[y] >= 0) center_final[y] = center_filt[y];

    if (road_type == RD_CROSS) {
        g_vision_data.elements |= BIT_CROSS;
        for (int y = 0; y < EH; y++) center_final[y] = TARGET_COL;
        return;
    }
    if (road_type == RD_S) {
        g_vision_data.elements |= BIT_S;
        int mid = EH / 2;
        if (center_filt[mid] >= 0 && center_filt[EH - 10] >= 0 && center_filt[30] >= 0) {
            for (int y = mid; y < EH; y++)
                center_final[y] = center_filt[EH-10] + (center_filt[mid] - center_filt[EH-10]) * (EH-10 - y) / (EH-10 - mid);
            for (int y = 0; y < mid; y++)
                center_final[y] = center_filt[30] + (center_filt[mid] - center_filt[30]) * y / mid;
        }
    }
    roundabout_process();
    if (ra_state == RA_NONE || ra_state == RA_DONE) obstacle_process();
}

static int16_t get_error(void)
{
    int32_t sum = 0, ws = 0;
    int start_y = EH - 60, end_y = EH - 20;
    for (int y = start_y; y <= end_y; y++) {
        if (y < 0 || y >= EH) continue;
        int16_t c = center_final[y];
        if (c < 0) continue;
        int w = y - start_y + 1;
        sum += (int32_t)(c - TARGET_COL) * w; ws += w;
    }
    return ws ? (int16_t)(sum / ws) : 0;
}

/* ============================================================
 * 对外接口
 * ============================================================ */
void vision_init(void)
{
    ipm_init();
    memset(&g_vision_data, 0, sizeof(g_vision_data));
    ra_state = RA_NONE; ra_dir = RA_DIR_NONE; ra_timer = 0;
    ob_state = OB_NONE; ob_dir = OB_DIR_NONE; ob_timer = 0; ob_col = -1; ob_row = -1;
}

void vision_task(void)
{
    g_vision_data.elements = 0;
    
    ipm_run();
    binarize();
    edge_extract();
    
    slope_detect();
    zebra_detect();
    
    calc_center_by_scene();
    
    g_vision_data.offset = get_error();
    g_vision_data.valid = (center_final[EH-10] >= 0) ? 1 : 0;
}