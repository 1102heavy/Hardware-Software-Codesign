/******************************************************************************
 * xscugic_tapp_example.c
 *
 * Combined performance benchmark: HLS FFT-Only IP vs RTL FFT IP.
 *
 * Hardware data paths:
 *   DMA0:  DDR (TxBuf0) --> AXI DMA0 MM2S --> s_axis --> HLS FFT-Only IP
 *          HLS FFT-Only IP --> m_axis --> AXI DMA0 S2MM --> DDR (RxBuf0)
 *
 *   DMA1:  DDR (TxBuf1) --> AXI DMA1 MM2S --> s_axis --> RTL FFT IP
 *          RTL FFT IP   --> m_axis --> AXI DMA1 S2MM --> DDR (RxBuf1)
 *
 * AXI-Stream word format:
 *   HLS FFT-Only IP  : bits[15: 0] = real (Q1.15),  bits[31:16] = imag (Q1.15)
 *   RTL FFT IP      : bits[31:16] = real (Q1.15),  bits[15: 0] = imag (Q1.15)
 *
 * Tests run on both IPs:
 *   TEST 1 — DC input       : all x[n] = 0x7FFF + j0
 *   TEST 2 — Impulse        : x[0] = 0x7FFF, x[1..N-1] = 0
 *   TEST 3 — Cosine tone    : x[n] = cos(2π·TEST_BIN·n/N), Q1.15
 *
 * RTL FFT scaling (10 stages of per-stage >>1):
 *   DC peak       ~ 32767,   Impulse bins ~ 31,   Cosine peaks ~ 16383
 *
 * Cache requirements (Cortex-A53, non-coherent DMA port):
 *   TX: D-cache FLUSH  before DMA start  (DDR must hold latest CPU writes)
 *   RX: D-cache INVALIDATE after DMA done (CPU must not read stale prefetch)
 *
 * Build requirements:
 *   - AXI DMA 0 + AXI DMA 1 IPs in Vivado block design (simple DMA, no S/G)
 *   - AXI Timer IP in Vivado block design (32-bit free-running counter)
 *   - HLS IP exported and added to Vivado block design
 *   - xaxidma, xtmrctr, xmy_fft_only_dma_ip drivers in BSP
 *   - 'm' in USER_LINK_LIBRARIES (UserConfig.cmake) for cosf()
 ******************************************************************************/

/* ── Standard / BSP includes ─────────────────────────────────────────────── */
#include <math.h>
#include "xparameters.h"
#include "xaxidma.h"
#include "xtmrctr.h"
#include "xil_cache.h"
#include "xstatus.h"
#include "xil_printf.h"
#include "xmy_fft_only_dma_ip.h"        /* HLS FFT-Only IP driver                 */

/* ═══════════════════════════════════════════════════════════════════════════
 * Configuration constants
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ── FFT / signal parameters ─────────────────────────────────────────────── */
#define FFT_N           1024            /* number of FFT points              */
#define FFT_LOG2N       10              /* log2(FFT_N) = number of stages    */
#define TEST_BIN        4               /* cosine test frequency bin         */
#define Q15_SCALE       32767.0f        /* full-scale Q1.15 amplitude        */
#ifndef M_PI
#define M_PI            3.14159265358979f
#endif

/* ── RTL FFT expected output magnitudes ──────────────────────────────────── */
/* (10 stages of per-stage >>1 right-shift scaling)                          */
#define RTL_EXP_DC_BIN0     32767       /* X[0] for all-ones DC input        */
#define RTL_EXP_IMPULSE     31          /* 32767 >> 10, flat across all bins */
#define RTL_EXP_COSINE_PEAK 16383       /* A/2 at +/- frequency bin          */

/* ── Verification tolerances ─────────────────────────────────────────────── */
#define TOL_DC_BIN0     2000
#define TOL_DC_OTHERS   5
#define TOL_IMPULSE     5
#define TOL_COSINE      1000            /* cosine peak detection threshold    */

/* ── AXI DMA base addresses ──────────────────────────────────────────────── */
/* Vitis 2025.2 SDT mode: XAxiDma_LookupConfig() takes a BASE ADDRESS,      */
/* not a device ID. Use XPAR_XAXIDMA_x_BASEADDR from xparameters.h.         */
/* DMA0: connected to HLS FFT-Only IP                                        */
/* DMA1: connected to RTL FFT IP                                             */
#ifndef XPAR_XAXIDMA_0_BASEADDR
#define XPAR_XAXIDMA_0_BASEADDR     0xA0000000U /* fallback for clangd       */
#endif
#ifndef XPAR_XAXIDMA_1_BASEADDR
#define XPAR_XAXIDMA_1_BASEADDR     0xA0010000U /* fallback for clangd       */
#endif
#define DMA0_BASE       XPAR_XAXIDMA_0_BASEADDR
#define DMA1_BASE       XPAR_XAXIDMA_1_BASEADDR

/* ── HLS IP base addresses ───────────────────────────────────────────────── */
/* Match the addresses assigned in your Vivado block design address editor.  */
#ifndef XPAR_MY_FFT_ONLY_DMA_IP_0_S_AXI_AXILITES_BASEADDR
#define XPAR_MY_FFT_ONLY_DMA_IP_0_S_AXI_AXILITES_BASEADDR  XPAR_XMY_FFT_ONLY_DMA_IP_0_BASEADDR
#endif
#define HLS_VOCODER_BASE    XPAR_MY_FFT_ONLY_DMA_IP_0_S_AXI_AXILITES_BASEADDR

/* ── AXI Timer ───────────────────────────────────────────────────────────── */
#ifndef XPAR_TMRCTR_0_DEVICE_ID
#define XPAR_TMRCTR_0_DEVICE_ID     0U  /* fallback for clangd               */
#endif
#ifndef XPAR_TMRCTR_0_CLOCK_FREQ_HZ
#define XPAR_TMRCTR_0_CLOCK_FREQ_HZ 100000000U   /* 100 MHz fallback         */
#endif
#define TIMER_DEV_ID        XPAR_TMRCTR_0_DEVICE_ID
#define TIMER_IDX           0
#define TIMER_CLK_HZ        XPAR_TMRCTR_0_CLOCK_FREQ_HZ

/* ── DMA polling timeout (loop iterations) ───────────────────────────────── */
#define DMA_TIMEOUT         10000000U

/* ═══════════════════════════════════════════════════════════════════════════
 * Buffers
 *
 * Each IP gets its own TX/RX pair so results remain independent.
 * Aligned to 64 bytes (one Cortex-A53 D-cache line) for safe cache ops.
 * ═══════════════════════════════════════════════════════════════════════════ */
static u32 HlsTxBuf[FFT_N] __attribute__((aligned(64)));  /* HLS input       */
static u32 HlsRxBuf[FFT_N] __attribute__((aligned(64)));  /* HLS output      */
static u32 RtlTxBuf[FFT_N] __attribute__((aligned(64)));  /* RTL FFT input   */
static u32 RtlRxBuf[FFT_N] __attribute__((aligned(64)));  /* RTL FFT output  */
static u32 SwRxBuf [FFT_N] __attribute__((aligned(64)));  /* SW FFT output   */

/* ── Software FFT working buffer ─────────────────────────────────────────── */
/* float complex pair — kept static to avoid 8 KB stack allocation           */
typedef struct { float r; float i; } CplxF;
static CplxF SwWorkBuf[FFT_N];

/* ── Driver instances ────────────────────────────────────────────────────── */
static XAxiDma        Dma0Inst;     /* DMA0 — HLS FFT-Only IP                 */
static XAxiDma        Dma1Inst;     /* DMA1 — RTL FFT IP                     */
static XTmrCtr        TimerInst;    /* AXI Timer                             */
static XMy_fft_only_dma_ip FftOnlyInst; /* HLS FFT-Only IP control                */

/* ═══════════════════════════════════════════════════════════════════════════
 * Function prototypes
 * ═══════════════════════════════════════════════════════════════════════════ */
static int  DmaInit     (XAxiDma *Dma, UINTPTR BaseAddr, const char *Name);
static int  TimerInit   (XTmrCtr *Tmr);
static int  HlsIpInit   (XMy_fft_only_dma_ip *Ip, u32 BaseAddr);

/* Signal generators (IsPacked=1 → RTL {real@31:16}, =0 → HLS {real@15:0}) */
static void GenDC       (u32 *Buf, int N, int IsPacked);
static void GenImpulse  (u32 *Buf, int N, int IsPacked);
static void GenCosine   (u32 *Buf, int N, int Bin, int IsPacked);

/* Run one complete FFT transaction and measure latency in timer ticks       */
static int  HlsRun      (XAxiDma *Dma, XMy_fft_only_dma_ip *Ip, XTmrCtr *Tmr,
                         u32 *Tx, u32 *Rx, int N,
                         const char *Tag, u32 *ElapsedTicksOut);
static int  RtlRun      (XAxiDma *Dma, XTmrCtr *Tmr,
                         u32 *Tx, u32 *Rx, int N,
                         const char *Tag, u32 *ElapsedTicksOut);
static int  SwRun       (XTmrCtr *Tmr, const u32 *Tx, u32 *Rx, int N,
                         const char *Tag, u32 *ElapsedTicksOut);

/* Result checkers — IsPacked selects real/imag byte lane                   */
static int  CheckDC     (const u32 *Buf, int N, int IsPacked, const char *Tag);
static int  CheckImpulse(const u32 *Buf, int N, int IsPacked, const char *Tag);
static int  CheckCosine (const u32 *Buf, int N, int Bin,
                         int IsPacked, const char *Tag);

/* Inline helpers to unpack a 32-bit FFT word into signed 16-bit components */
static inline s16 WordReal(u32 W, int IsPacked)
{
    /* RTL FFT  (IsPacked=1): bits[31:16] = real                             */
    /* HLS IP   (IsPacked=0): bits[15: 0] = real                             */
    return IsPacked ? (s16)(W >> 16) : (s16)(W & 0xFFFFU);
}
static inline s16 WordImag(u32 W, int IsPacked)
{
    return IsPacked ? (s16)(W & 0xFFFFU) : (s16)(W >> 16);
}
static inline u32 PackWord(s16 Re, s16 Im, int IsPacked)
{
    return IsPacked
        ? (((u32)(u16)Re << 16) | (u16)Im)   /* RTL FFT  format             */
        : (((u32)(u16)Im << 16) | (u16)Re);  /* HLS IP   format             */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * main
 * ═══════════════════════════════════════════════════════════════════════════ */
int main(void)
{
    int Status;
    int HlsPass, RtlPass, SwPass;

    /* Per-test cycle counts — stored for the final comparison table         */
    u32 HlsCycles[3], RtlCycles[3], SwCycles[3];
    int HlsResult[3], RtlResult[3], SwResult[3];

    int TotalPass = 0, TotalFail = 0;

    /* ── Banner ──────────────────────────────────────────────────────────── */
    xil_printf("\r\n");
    xil_printf("============================================================\r\n");
    xil_printf("  FFT Performance Benchmark: HLS FFT-Only vs RTL FFT        \r\n");
    xil_printf("  N=%d  LOG2N=%d  TEST_BIN=%d  Timer=%lu MHz              \r\n",
               FFT_N, FFT_LOG2N, TEST_BIN,
               (unsigned long)(TIMER_CLK_HZ / 1000000U));
    xil_printf("============================================================\r\n\r\n");

    /* ════════════════════════════════════════════════════════════════════════
     * INITIALISATION
     * ════════════════════════════════════════════════════════════════════════ */
    xil_printf("--- INIT ---------------------------------------------------\r\n");

    Status = TimerInit(&TimerInst);
    if (Status != XST_SUCCESS) {
        xil_printf("[INIT] FAIL: AXI Timer\r\n");
        return XST_FAILURE;
    }

    Status = DmaInit(&Dma0Inst, DMA0_BASE, "DMA0 (HLS)");
    if (Status != XST_SUCCESS) return XST_FAILURE;

    Status = DmaInit(&Dma1Inst, DMA1_BASE, "DMA1 (RTL)");
    if (Status != XST_SUCCESS) return XST_FAILURE;

    Status = HlsIpInit(&FftOnlyInst, HLS_VOCODER_BASE);
    if (Status != XST_SUCCESS) return XST_FAILURE;

    xil_printf("--- INIT DONE ----------------------------------------------\r\n\r\n");

    /* ════════════════════════════════════════════════════════════════════════
     * TEST 1 — DC Input
     *   x[n] = full-scale real sample for all n
     *   RTL FFT : X[0] ~ 32767,  all other bins ~ 0
     *   HLS IP  : same expected output (both compute the DFT)
     * ════════════════════════════════════════════════════════════════════════ */
    xil_printf("============================================================\r\n");
    xil_printf(" TEST 1 — DC Input  (x[n] = 0x7FFF + j0  for all n)       \r\n");
    xil_printf("============================================================\r\n");

    /* ── HLS FFT-Only on DMA0 ─────────────────────────────────────────────── */
    xil_printf("\r\n[HLS] ");
    GenDC(HlsTxBuf, FFT_N, 0 /*HLS packing*/);
    Status = HlsRun(&Dma0Inst, &FftOnlyInst, &TimerInst,
                    HlsTxBuf, HlsRxBuf, FFT_N, "HLS-DC", &HlsCycles[0]);
    if (Status != XST_SUCCESS) {
        xil_printf("[HLS] TEST 1 FAIL — DMA error\r\n");
        HlsResult[0] = 0; TotalFail++;
    } else {
        HlsPass = CheckDC(HlsRxBuf, FFT_N, 0 /*HLS packing*/, "HLS-DC");
        HlsResult[0] = HlsPass;
        xil_printf("[HLS] TEST 1 %s  (%lu cycles)\r\n\r\n",
                   HlsPass ? "PASS" : "FAIL", (unsigned long)HlsCycles[0]);
        if (HlsPass) TotalPass++; else TotalFail++;
    }

    /* ── RTL FFT on DMA1 ─────────────────────────────────────────────────── */
    xil_printf("[RTL] ");
    GenDC(RtlTxBuf, FFT_N, 1 /*RTL packing*/);
    Status = RtlRun(&Dma1Inst, &TimerInst,
                    RtlTxBuf, RtlRxBuf, FFT_N, "RTL-DC", &RtlCycles[0]);
    if (Status != XST_SUCCESS) {
        xil_printf("[RTL] TEST 1 FAIL — DMA error\r\n");
        RtlResult[0] = 0; TotalFail++;
    } else {
        RtlPass = CheckDC(RtlRxBuf, FFT_N, 1 /*RTL packing*/, "RTL-DC");
        RtlResult[0] = RtlPass;
        xil_printf("[RTL] TEST 1 %s  (%lu cycles)\r\n\r\n",
                   RtlPass ? "PASS" : "FAIL", (unsigned long)RtlCycles[0]);
        if (RtlPass) TotalPass++; else TotalFail++;
    }

    /* ── Software FFT (reference, no DMA) ───────────────────────────────────── */
    xil_printf("[SW]  ");
    /* Reuse RtlTxBuf — same DC signal, RTL packing, generated above         */
    Status = SwRun(&TimerInst, RtlTxBuf, SwRxBuf, FFT_N, "SW-DC", &SwCycles[0]);
    if (Status != XST_SUCCESS) {
        xil_printf("[SW]  TEST 1 FAIL\r\n");
        SwResult[0] = 0; TotalFail++;
    } else {
        SwPass = CheckDC(SwRxBuf, FFT_N, 1 /*RTL packing*/, "SW-DC");
        SwResult[0] = SwPass;
        xil_printf("[SW]  TEST 1 %s  (%lu cycles)\r\n\r\n",
                   SwPass ? "PASS" : "FAIL", (unsigned long)SwCycles[0]);
        if (SwPass) TotalPass++; else TotalFail++;
    }

    /* ════════════════════════════════════════════════════════════════════════
     * TEST 2 — Impulse Input
     *   x[0] = 0x7FFF + j0,  x[1..N-1] = 0
     *   RTL FFT : all bins = 31 +/- 5  (32767 >> 10)
     *   HLS IP  : all bins equal (magnitude depends on HLS scaling)
     * ════════════════════════════════════════════════════════════════════════ */
    xil_printf("============================================================\r\n");
    xil_printf(" TEST 2 — Impulse  (x[0]=0x7FFF, x[1..%d]=0)              \r\n",
               FFT_N - 1);
    xil_printf("============================================================\r\n");

    /* ── HLS FFT-Only on DMA0 ─────────────────────────────────────────────── */
    xil_printf("\r\n[HLS] ");
    GenImpulse(HlsTxBuf, FFT_N, 0);
    Status = HlsRun(&Dma0Inst, &FftOnlyInst, &TimerInst,
                    HlsTxBuf, HlsRxBuf, FFT_N, "HLS-Imp", &HlsCycles[1]);
    if (Status != XST_SUCCESS) {
        xil_printf("[HLS] TEST 2 FAIL — DMA error\r\n");
        HlsResult[1] = 0; TotalFail++;
    } else {
        HlsPass = CheckImpulse(HlsRxBuf, FFT_N, 0, "HLS-Imp");
        HlsResult[1] = HlsPass;
        xil_printf("[HLS] TEST 2 %s  (%lu cycles)\r\n\r\n",
                   HlsPass ? "PASS" : "FAIL", (unsigned long)HlsCycles[1]);
        if (HlsPass) TotalPass++; else TotalFail++;
    }

    /* ── RTL FFT on DMA1 ─────────────────────────────────────────────────── */
    xil_printf("[RTL] ");
    GenImpulse(RtlTxBuf, FFT_N, 1);
    Status = RtlRun(&Dma1Inst, &TimerInst,
                    RtlTxBuf, RtlRxBuf, FFT_N, "RTL-Imp", &RtlCycles[1]);
    if (Status != XST_SUCCESS) {
        xil_printf("[RTL] TEST 2 FAIL — DMA error\r\n");
        RtlResult[1] = 0; TotalFail++;
    } else {
        RtlPass = CheckImpulse(RtlRxBuf, FFT_N, 1, "RTL-Imp");
        RtlResult[1] = RtlPass;
        xil_printf("[RTL] TEST 2 %s  (%lu cycles)\r\n\r\n",
                   RtlPass ? "PASS" : "FAIL", (unsigned long)RtlCycles[1]);
        if (RtlPass) TotalPass++; else TotalFail++;
    }

    /* ── Software FFT (reference) ────────────────────────────────────────── */
    xil_printf("[SW]  ");
    Status = SwRun(&TimerInst, RtlTxBuf, SwRxBuf, FFT_N, "SW-Imp", &SwCycles[1]);
    if (Status != XST_SUCCESS) {
        xil_printf("[SW]  TEST 2 FAIL\r\n");
        SwResult[1] = 0; TotalFail++;
    } else {
        SwPass = CheckImpulse(SwRxBuf, FFT_N, 1, "SW-Imp");
        SwResult[1] = SwPass;
        xil_printf("[SW]  TEST 2 %s  (%lu cycles)\r\n\r\n",
                   SwPass ? "PASS" : "FAIL", (unsigned long)SwCycles[1]);
        if (SwPass) TotalPass++; else TotalFail++;
    }

    /* ════════════════════════════════════════════════════════════════════════
     * TEST 3 — Real Cosine Tone at bin k = TEST_BIN
     *   x[n] = cos(2π·TEST_BIN·n/N),  Q1.15, imag = 0
     *   Expected: peaks at X[TEST_BIN] and X[N-TEST_BIN], rest near zero
     * ════════════════════════════════════════════════════════════════════════ */
    xil_printf("============================================================\r\n");
    xil_printf(" TEST 3 — Cosine Tone at bin k=%d                          \r\n",
               TEST_BIN);
    xil_printf("============================================================\r\n");

    /* ── HLS FFT-Only on DMA0 ─────────────────────────────────────────────── */
    xil_printf("\r\n[HLS] ");
    GenCosine(HlsTxBuf, FFT_N, TEST_BIN, 0);
    Status = HlsRun(&Dma0Inst, &FftOnlyInst, &TimerInst,
                    HlsTxBuf, HlsRxBuf, FFT_N, "HLS-Cos", &HlsCycles[2]);
    if (Status != XST_SUCCESS) {
        xil_printf("[HLS] TEST 3 FAIL — DMA error\r\n");
        HlsResult[2] = 0; TotalFail++;
    } else {
        HlsPass = CheckCosine(HlsRxBuf, FFT_N, TEST_BIN, 0, "HLS-Cos");
        HlsResult[2] = HlsPass;
        xil_printf("[HLS] TEST 3 %s  (%lu cycles)\r\n\r\n",
                   HlsPass ? "PASS" : "FAIL", (unsigned long)HlsCycles[2]);
        if (HlsPass) TotalPass++; else TotalFail++;
    }

    /* ── RTL FFT on DMA1 ─────────────────────────────────────────────────── */
    xil_printf("[RTL] ");
    GenCosine(RtlTxBuf, FFT_N, TEST_BIN, 1);
    Status = RtlRun(&Dma1Inst, &TimerInst,
                    RtlTxBuf, RtlRxBuf, FFT_N, "RTL-Cos", &RtlCycles[2]);
    if (Status != XST_SUCCESS) {
        xil_printf("[RTL] TEST 3 FAIL — DMA error\r\n");
        RtlResult[2] = 0; TotalFail++;
    } else {
        RtlPass = CheckCosine(RtlRxBuf, FFT_N, TEST_BIN, 1, "RTL-Cos");
        RtlResult[2] = RtlPass;
        xil_printf("[RTL] TEST 3 %s  (%lu cycles)\r\n\r\n",
                   RtlPass ? "PASS" : "FAIL", (unsigned long)RtlCycles[2]);
        if (RtlPass) TotalPass++; else TotalFail++;
    }

    /* ── Software FFT (reference) ────────────────────────────────────────── */
    xil_printf("[SW]  ");
    Status = SwRun(&TimerInst, RtlTxBuf, SwRxBuf, FFT_N, "SW-Cos", &SwCycles[2]);
    if (Status != XST_SUCCESS) {
        xil_printf("[SW]  TEST 3 FAIL\r\n");
        SwResult[2] = 0; TotalFail++;
    } else {
        SwPass = CheckCosine(SwRxBuf, FFT_N, TEST_BIN, 1, "SW-Cos");
        SwResult[2] = SwPass;
        xil_printf("[SW]  TEST 3 %s  (%lu cycles)\r\n\r\n",
                   SwPass ? "PASS" : "FAIL", (unsigned long)SwCycles[2]);
        if (SwPass) TotalPass++; else TotalFail++;
    }

    /* ════════════════════════════════════════════════════════════════════════
     * PERFORMANCE COMPARISON TABLE
     *
     * Columns: HLS FFT-Only (DMA0)  |  RTL FFT (DMA1)  |  SW FFT (Cortex-A53)
     * Speedup shows how many times faster the fastest HW impl is vs SW.
     * ════════════════════════════════════════════════════════════════════════ */
    xil_printf("============================================================\r\n");
    xil_printf("  PERFORMANCE COMPARISON  (AXI Timer @ %lu MHz)            \r\n",
               (unsigned long)(TIMER_CLK_HZ / 1000000U));
    xil_printf("  Test      HLS(%s)  RTL(%s)   SW(%s)   HW Speedup\r\n",
               HlsResult[0]?"OK":"--", RtlResult[0]?"OK":"--", SwResult[0]?"OK":"--");
    xil_printf("  -------  ----------  ----------  ----------  ----------  \r\n");

    {
        const char *TestName[3] = {"DC     ", "Impulse", "Cosine "};
        int i;
        for (i = 0; i < 3; i++) {
            /* Best hardware = min(HLS, RTL); speedup vs software             */
            u32 best_hw = (HlsCycles[i] < RtlCycles[i]) ? HlsCycles[i] : RtlCycles[i];
            u32 speedup = (best_hw > 0U) ? (SwCycles[i] / best_hw) : 0U;

            xil_printf("  %s  %10lu  %10lu  %10lu   ~%lux faster\r\n",
                       TestName[i],
                       (unsigned long)HlsCycles[i],
                       (unsigned long)RtlCycles[i],
                       (unsigned long)SwCycles[i],
                       (unsigned long)speedup);
        }
    }

    xil_printf("============================================================\r\n");
    xil_printf("  OVERALL : %d PASS,  %d FAIL  (9 tests: HLS + RTL + SW)  \r\n",
               TotalPass, TotalFail);
    xil_printf("  %s\r\n",
               (TotalFail == 0)
               ? "*** ALL TESTS PASSED ***"
               : "*** FAILURES DETECTED — check IP configuration ***");
    xil_printf("============================================================\r\n");

    return (TotalFail == 0) ? XST_SUCCESS : XST_FAILURE;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * DmaInit
 *
 * Looks up AXI DMA by device ID, initialises the driver, verifies simple
 * (non-scatter-gather) mode, and resets the engine.
 * ═══════════════════════════════════════════════════════════════════════════ */
static int DmaInit(XAxiDma *Dma, UINTPTR BaseAddr, const char *Name)
{
    XAxiDma_Config *Cfg;
    int Status;

    /* SDT (Vitis 2025.2): LookupConfig takes the AXI DMA base address.      */
    Cfg = XAxiDma_LookupConfig(BaseAddr);
    if (!Cfg) {
        xil_printf("[%s] FAIL: No config at base 0x%08X.\r\n"
                   "  Check XPAR_XAXIDMA_x_BASEADDR in xparameters.h\r\n"
                   "  and verify the DMA IP exists in your Vivado design.\r\n",
                   Name, (unsigned int)BaseAddr);
        return XST_FAILURE;
    }

    /* If this hangs the PL bitstream is not loaded. In Vitis:               */
    /* Run Configuration -> enable "Program FPGA" before launch.             */
    Status = XAxiDma_CfgInitialize(Dma, Cfg);
    if (Status != XST_SUCCESS) {
        xil_printf("[%s] FAIL: CfgInitialize returned %d\r\n", Name, Status);
        return Status;
    }

    if (XAxiDma_HasSg(Dma)) {
        xil_printf("[%s] FAIL: Scatter-gather mode detected.\r\n"
                   "  Set 'Enable Scatter Gather Engine' = OFF in Vivado IP.\r\n",
                   Name);
        return XST_FAILURE;
    }

    XAxiDma_Reset(Dma);
    while (!XAxiDma_ResetIsDone(Dma));

    xil_printf("[%s] OK  BaseAddr=0x%08X\r\n",
               Name, (unsigned int)Cfg->BaseAddr);
    return XST_SUCCESS;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TimerInit
 *
 * Initialises AXI Timer counter 0 in free-running up-count mode.
 * ═══════════════════════════════════════════════════════════════════════════ */
static int TimerInit(XTmrCtr *Tmr)
{
    int Status;

    Status = XTmrCtr_Initialize(Tmr, TIMER_DEV_ID);
    if (Status != XST_SUCCESS) {
        xil_printf("[Timer] FAIL: Initialize returned %d\r\n"
                   "  Verify XPAR_TMRCTR_0_DEVICE_ID in xparameters.h.\r\n",
                   Status);
        return Status;
    }

    XTmrCtr_SetOptions(Tmr, TIMER_IDX, XTC_AUTO_RELOAD_OPTION);
    xil_printf("[Timer] OK  DevId=%d  Clk=%lu MHz\r\n",
               TIMER_DEV_ID, (unsigned long)(TIMER_CLK_HZ / 1000000U));
    return XST_SUCCESS;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * HlsIpInit
 *
 * Initialises the HLS FFT-Only IP using its AXI-Lite control port.
 * The base address must match the address assigned in Vivado's address editor.
 * ═══════════════════════════════════════════════════════════════════════════ */
static int HlsIpInit(XMy_fft_only_dma_ip *Ip, u32 BaseAddr)
{
    XMy_fft_only_dma_ip_Config *Cfg;
    int Status;

    Cfg = XMy_fft_only_dma_ip_LookupConfig(BaseAddr);
    if (!Cfg) {
        xil_printf("[HLS-IP] FAIL: No config at base 0x%08X.\r\n"
                   "  Verify XPAR_MY_FFT_ONLY_DMA_IP_0_S_AXI_AXILITES_BASEADDR.\r\n",
                   (unsigned int)BaseAddr);
        return XST_FAILURE;
    }

    Status = XMy_fft_only_dma_ip_CfgInitialize(Ip, Cfg);
    if (Status != XST_SUCCESS) {
        xil_printf("[HLS-IP] FAIL: CfgInitialize returned %d\r\n", Status);
        return Status;
    }

    xil_printf("[HLS-IP] OK  BaseAddr=0x%08X\r\n", (unsigned int)BaseAddr);
    return XST_SUCCESS;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Signal generators
 *
 * IsPacked = 1  →  RTL FFT  word format: {real @ bits[31:16], imag @ bits[15:0]}
 * IsPacked = 0  →  HLS IP   word format: {imag @ bits[31:16], real @ bits[15:0]}
 * ═══════════════════════════════════════════════════════════════════════════ */

/* GenDC — all samples = 0x7FFF + j0 */
static void GenDC(u32 *Buf, int N, int IsPacked)
{
    int n;
    for (n = 0; n < N; n++)
        Buf[n] = PackWord(0x7FFF, 0x0000, IsPacked);
}

/* GenImpulse — x[0] = 0x7FFF + j0,  x[1..N-1] = 0 */
static void GenImpulse(u32 *Buf, int N, int IsPacked)
{
    int n;
    Buf[0] = PackWord(0x7FFF, 0x0000, IsPacked);
    for (n = 1; n < N; n++)
        Buf[n] = 0x00000000U;
}

/* GenCosine — x[n] = cos(2π·Bin·n/N) Q1.15, imag = 0 */
static void GenCosine(u32 *Buf, int N, int Bin, int IsPacked)
{
    int n;
    float angle;
    s16 re;

    for (n = 0; n < N; n++) {
        angle = 2.0f * (float)M_PI * (float)Bin * (float)n / (float)N;
        re    = (s16)(cosf(angle) * Q15_SCALE);
        Buf[n] = PackWord(re, 0x0000, IsPacked);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * HlsRun
 *
 * Runs one FFT transaction through the HLS FFT-Only IP via DMA0:
 *   1. Flush TX cache → DDR visible to DMA
 *   2. Invalidate RX cache → stale data discarded
 *   3. Start the HLS IP (sets ap_start)
 *   4. Arm S2MM (RX) first so DMA is ready when IP produces output
 *   5. Fire MM2S (TX) to push samples into IP
 *   6. Poll TX done, poll RX done
 *   7. Re-invalidate RX cache → CPU reads fresh DMA data
 *   8. Return elapsed timer ticks
 * ═══════════════════════════════════════════════════════════════════════════ */
static int HlsRun(XAxiDma *Dma, XMy_fft_only_dma_ip *Ip, XTmrCtr *Tmr,
                  u32 *Tx, u32 *Rx, int N,
                  const char *Tag, u32 *ElapsedTicksOut)
{
    u32 ByteLen = (u32)N * (u32)sizeof(u32);
    int Status;
    u32 Timeout;
    u32 TStart, TEnd;

    xil_printf("Running %s (%lu bytes)...\r\n", Tag, (unsigned long)ByteLen);

    /* ── Step 1: Flush TX — ensure DDR holds latest CPU-written data ──────── */
    Xil_DCacheFlushRange((UINTPTR)Tx, ByteLen);

    /* ── Step 2: Invalidate RX — discard stale cache lines ──────────────── */
    Xil_DCacheInvalidateRange((UINTPTR)Rx, ByteLen);

    /* ── Step 3: Start timer ──────────────────────────────────────────────── */
    XTmrCtr_Reset(Tmr, TIMER_IDX);
    XTmrCtr_Start(Tmr, TIMER_IDX);
    TStart = XTmrCtr_GetValue(Tmr, TIMER_IDX);

    /* ── Step 4: Assert ap_start on the HLS IP ───────────────────────────── */
    XMy_fft_only_dma_ip_Start(Ip);

    /* ── Step 5: Arm RX (S2MM) BEFORE TX — prevents missed output beats ──── */
    Status = XAxiDma_SimpleTransfer(Dma, (UINTPTR)Rx, ByteLen,
                                    XAXIDMA_DEVICE_TO_DMA);
    if (Status != XST_SUCCESS) {
        xil_printf("[%s] FAIL: RX DMA start (err=%d)\r\n", Tag, Status);
        return Status;
    }

    /* ── Step 6: Fire TX (MM2S) to feed the HLS IP ───────────────────────── */
    Status = XAxiDma_SimpleTransfer(Dma, (UINTPTR)Tx, ByteLen,
                                    XAXIDMA_DMA_TO_DEVICE);
    if (Status != XST_SUCCESS) {
        xil_printf("[%s] FAIL: TX DMA start (err=%d)\r\n", Tag, Status);
        return Status;
    }

    /* ── Step 7: Poll TX done ──────────────────────────────────────────────── */
    Timeout = DMA_TIMEOUT;
    while (XAxiDma_Busy(Dma, XAXIDMA_DMA_TO_DEVICE)) {
        if (--Timeout == 0U) {
            xil_printf("[%s] FAIL: TX timeout — check HLS IP s_axis tready\r\n",
                       Tag);
            return XST_FAILURE;
        }
    }

    /* ── Step 8: Poll RX done ──────────────────────────────────────────────── */
    Timeout = DMA_TIMEOUT;
    while (XAxiDma_Busy(Dma, XAXIDMA_DEVICE_TO_DMA)) {
        if (--Timeout == 0U) {
            xil_printf("[%s] FAIL: RX timeout — HLS IP did not produce output\r\n",
                       Tag);
            return XST_FAILURE;
        }
    }

    /* ── Step 9: Capture and stop timer ──────────────────────────────────── */
    TEnd = XTmrCtr_GetValue(Tmr, TIMER_IDX);
    XTmrCtr_Stop(Tmr, TIMER_IDX);

    /* ── Step 10: Invalidate RX again — discard speculative prefetch ──────── */
    Xil_DCacheInvalidateRange((UINTPTR)Rx, ByteLen);

    /* Unsigned subtraction handles 32-bit timer wrap correctly              */
    *ElapsedTicksOut = TEnd - TStart;

    xil_printf("[%s] Done.\r\n", Tag);
    return XST_SUCCESS;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * RtlRun
 *
 * Runs one FFT transaction through the RTL FFT IP via DMA1.
 * Same cache + timer discipline as HlsRun; no ap_start needed (RTL FFT
 * auto-starts on s_axis_tlast from the DMA).
 * ═══════════════════════════════════════════════════════════════════════════ */
static int RtlRun(XAxiDma *Dma, XTmrCtr *Tmr,
                  u32 *Tx, u32 *Rx, int N,
                  const char *Tag, u32 *ElapsedTicksOut)
{
    u32 ByteLen = (u32)N * (u32)sizeof(u32);
    int Status;
    u32 Timeout;
    u32 TStart, TEnd;

    xil_printf("Running %s (%lu bytes)...\r\n", Tag, (unsigned long)ByteLen);

    /* ── Cache management: flush TX, invalidate RX ───────────────────────── */
    Xil_DCacheFlushRange((UINTPTR)Tx, ByteLen);
    Xil_DCacheInvalidateRange((UINTPTR)Rx, ByteLen);

    /* ── Timer start ──────────────────────────────────────────────────────── */
    XTmrCtr_Reset(Tmr, TIMER_IDX);
    XTmrCtr_Start(Tmr, TIMER_IDX);
    TStart = XTmrCtr_GetValue(Tmr, TIMER_IDX);

    /* ── Arm RX (S2MM) first ──────────────────────────────────────────────── */
    Status = XAxiDma_SimpleTransfer(Dma, (UINTPTR)Rx, ByteLen,
                                    XAXIDMA_DEVICE_TO_DMA);
    if (Status != XST_SUCCESS) {
        xil_printf("[%s] FAIL: RX DMA start (err=%d)\r\n", Tag, Status);
        return Status;
    }

    /* ── Fire TX (MM2S) ───────────────────────────────────────────────────── */
    Status = XAxiDma_SimpleTransfer(Dma, (UINTPTR)Tx, ByteLen,
                                    XAXIDMA_DMA_TO_DEVICE);
    if (Status != XST_SUCCESS) {
        xil_printf("[%s] FAIL: TX DMA start (err=%d)\r\n", Tag, Status);
        return Status;
    }

    /* ── Poll TX done ─────────────────────────────────────────────────────── */
    Timeout = DMA_TIMEOUT;
    while (XAxiDma_Busy(Dma, XAXIDMA_DMA_TO_DEVICE)) {
        if (--Timeout == 0U) {
            xil_printf("[%s] FAIL: TX timeout — check RTL FFT s_axis tready\r\n",
                       Tag);
            return XST_FAILURE;
        }
    }

    /* ── Poll RX done ─────────────────────────────────────────────────────── */
    Timeout = DMA_TIMEOUT;
    while (XAxiDma_Busy(Dma, XAXIDMA_DEVICE_TO_DMA)) {
        if (--Timeout == 0U) {
            xil_printf("[%s] FAIL: RX timeout — RTL FFT did not produce output\r\n",
                       Tag);
            return XST_FAILURE;
        }
    }

    /* ── Timer stop + final cache invalidation ────────────────────────────── */
    TEnd = XTmrCtr_GetValue(Tmr, TIMER_IDX);
    XTmrCtr_Stop(Tmr, TIMER_IDX);
    Xil_DCacheInvalidateRange((UINTPTR)Rx, ByteLen);

    *ElapsedTicksOut = TEnd - TStart;

    xil_printf("[%s] Done.\r\n", Tag);
    return XST_SUCCESS;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * CheckDC
 *
 * Verifies DC test output:
 *   X[0].real >= RTL_EXP_DC_BIN0 - TOL_DC_BIN0
 *   X[0].imag within +/- TOL_DC_OTHERS
 *   All X[k!=0] within +/- TOL_DC_OTHERS
 *
 * Returns 1 (PASS) or 0 (FAIL).
 * ═══════════════════════════════════════════════════════════════════════════ */
static int CheckDC(const u32 *Buf, int N, int IsPacked, const char *Tag)
{
    int k;
    s16 re, im;
    int Fail = 0;

    re = WordReal(Buf[0], IsPacked);
    im = WordImag(Buf[0], IsPacked);
    xil_printf("[%s-DC] X[   0] = %6d + j*%6d  (expect ~%d + j*0)  %s\r\n",
               Tag, (int)re, (int)im, RTL_EXP_DC_BIN0,
               (re >= RTL_EXP_DC_BIN0 - TOL_DC_BIN0 &&
                im >= -TOL_DC_OTHERS && im <= TOL_DC_OTHERS) ? "PASS" : "FAIL");

    if (re < RTL_EXP_DC_BIN0 - TOL_DC_BIN0 ||
        im < -TOL_DC_OTHERS || im > TOL_DC_OTHERS)
        Fail = 1;

    for (k = 1; k < N; k++) {
        re = WordReal(Buf[k], IsPacked);
        im = WordImag(Buf[k], IsPacked);
        if (re > TOL_DC_OTHERS || re < -TOL_DC_OTHERS ||
            im > TOL_DC_OTHERS || im < -TOL_DC_OTHERS) {
            xil_printf("[%s-DC] X[%4d] = %6d + j*%6d  UNEXPECTED\r\n",
                       Tag, k, (int)re, (int)im);
            Fail = 1;
        }
    }
    return !Fail;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * CheckImpulse
 *
 * Verifies impulse test output:
 *   All bins: real = RTL_EXP_IMPULSE +/- TOL_IMPULSE,  imag = 0 +/- TOL_IMPULSE
 *
 * Returns 1 (PASS) or 0 (FAIL).
 * ═══════════════════════════════════════════════════════════════════════════ */
static int CheckImpulse(const u32 *Buf, int N, int IsPacked, const char *Tag)
{
    int k;
    s16 re, im;
    int Fail = 0;
    int BadCount = 0;

    xil_printf("[%s-Imp] Checking all bins = %d +/-%d...\r\n",
               Tag, RTL_EXP_IMPULSE, TOL_IMPULSE);

    for (k = 0; k < N; k++) {
        re = WordReal(Buf[k], IsPacked);
        im = WordImag(Buf[k], IsPacked);

        if (re < RTL_EXP_IMPULSE - TOL_IMPULSE ||
            re > RTL_EXP_IMPULSE + TOL_IMPULSE ||
            im < -TOL_IMPULSE || im > TOL_IMPULSE) {
            if (BadCount < 8)
                xil_printf("[%s-Imp] X[%4d] = %6d + j*%6d  FAIL\r\n",
                           Tag, k, (int)re, (int)im);
            else if (BadCount == 8)
                xil_printf("[%s-Imp] ... (further failures suppressed)\r\n", Tag);
            BadCount++;
            Fail = 1;
        }
    }

    if (!Fail)
        xil_printf("[%s-Imp] All %d bins within tolerance.\r\n", Tag, N);
    else
        xil_printf("[%s-Imp] %d bin(s) out of tolerance.\r\n", Tag, BadCount);

    return !Fail;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * CheckCosine
 *
 * Verifies cosine tone test output:
 *   X[Bin]   : |real| or |imag| >= TOL_COSINE
 *   X[N-Bin] : |real| or |imag| >= TOL_COSINE
 *   All others below threshold
 *
 * Returns 1 (PASS) or 0 (FAIL).
 * ═══════════════════════════════════════════════════════════════════════════ */
static int CheckCosine(const u32 *Buf, int N, int Bin, int IsPacked,
                       const char *Tag)
{
    int k;
    s16 re, im;
    int PeakPos = 0, PeakNeg = 0;
    int SpuriousCount = 0;
    int Fail = 0;

    xil_printf("[%s-Cos] Non-zero bins (|re| or |im| > %d):\r\n",
               Tag, TOL_COSINE);

    for (k = 0; k < N; k++) {
        re = WordReal(Buf[k], IsPacked);
        im = WordImag(Buf[k], IsPacked);

        if (re >  (s16)TOL_COSINE || re < -(s16)TOL_COSINE ||
            im >  (s16)TOL_COSINE || im < -(s16)TOL_COSINE) {

            xil_printf("  X[%4d] = %6d + j*%6d", k, (int)re, (int)im);

            if (k == Bin) {
                xil_printf("  <-- positive-freq peak\r\n");
                PeakPos = 1;
            } else if (k == N - Bin) {
                xil_printf("  <-- negative-freq peak\r\n");
                PeakNeg = 1;
            } else {
                xil_printf("  SPURIOUS\r\n");
                SpuriousCount++;
                Fail = 1;
            }
        }
    }

    if (!PeakPos) { xil_printf("[%s-Cos] FAIL: no peak at X[%d]\r\n",   Tag, Bin);   Fail = 1; }
    if (!PeakNeg) { xil_printf("[%s-Cos] FAIL: no peak at X[%d]\r\n",   Tag, N-Bin); Fail = 1; }
    if (SpuriousCount)
        xil_printf("[%s-Cos] WARNING: %d spurious peak(s)\r\n", Tag, SpuriousCount);

    xil_printf("[%s-Cos] X[%d]: %s  X[%d]: %s\r\n",
               Tag, Bin, PeakPos ? "PASS" : "FAIL",
               N-Bin, PeakNeg ? "PASS" : "FAIL");

    return !Fail;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * sw_bit_rev — return bit-reversed index for LOG2N-bit value
 * ═══════════════════════════════════════════════════════════════════════════ */
static unsigned int sw_bit_rev(unsigned int x)
{
    unsigned int result = 0;
    int i;
    for (i = 0; i < FFT_LOG2N; i++) {
        result = (result << 1) | (x & 1U);
        x >>= 1;
    }
    return result;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * sw_fft_compute
 *
 * In-place Cooley-Tukey radix-2 DIT FFT on SwWorkBuf[0..N-1].
 * Applies the same per-stage ÷2 scaling as the RTL FFT so that output
 * magnitudes are directly comparable (total scale = 1/1024 for N=1024).
 *
 * Algorithm:
 *   1. Bit-reversal permutation
 *   2. For each stage s=1..LOG2N:
 *        group size  m  = 2^s
 *        span           = m/2
 *        twiddle step   W_m = e^(-j*2*pi/m)
 *        For each group starting at k:
 *          For each pair index j=0..span-1:
 *            t         = W^j * x[k+j+span]
 *            x[k+j]       = (x[k+j] + t) * 0.5   (÷2 scaling)
 *            x[k+j+span] = (x[k+j] - t) * 0.5
 * ═══════════════════════════════════════════════════════════════════════════ */
static void sw_fft_compute(CplxF *x, int N)
{
    int s, k, j;

    /* ── Step 1: Bit-reversal permutation ───────────────────────────────── */
    for (k = 0; k < N; k++) {
        unsigned int rev = sw_bit_rev((unsigned int)k);
        if (rev > (unsigned int)k) {
            CplxF tmp = x[k];
            x[k]      = x[rev];
            x[rev]    = tmp;
        }
    }

    /* ── Step 2: Butterfly stages ────────────────────────────────────────── */
    for (s = 1; s <= FFT_LOG2N; s++) {
        int    m      = 1 << s;          /* group size (2, 4, 8, ..., N)    */
        int    span   = m >> 1;          /* half-group = butterfly span      */
        float  angle  = -2.0f * (float)M_PI / (float)m;
        float  wm_r   = cosf(angle);     /* twiddle factor increment W_m     */
        float  wm_i   = sinf(angle);

        for (k = 0; k < N; k += m) {
            float w_r = 1.0f;            /* running twiddle W^j, starts at 1 */
            float w_i = 0.0f;

            for (j = 0; j < span; j++) {
                /* t = W^j * x[k+j+span] */
                float t_r = w_r * x[k+j+span].r - w_i * x[k+j+span].i;
                float t_i = w_r * x[k+j+span].i + w_i * x[k+j+span].r;

                float u_r = x[k+j].r;
                float u_i = x[k+j].i;

                /* Per-stage ÷2 scaling applied at write-back (matches RTL)  */
                x[k+j].r        = (u_r + t_r) * 0.5f;
                x[k+j].i        = (u_i + t_i) * 0.5f;
                x[k+j+span].r   = (u_r - t_r) * 0.5f;
                x[k+j+span].i   = (u_i - t_i) * 0.5f;

                /* Advance twiddle: w *= wm */
                float new_wr = w_r * wm_r - w_i * wm_i;
                float new_wi = w_r * wm_i + w_i * wm_r;
                w_r = new_wr;
                w_i = new_wi;
            }
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SwRun
 *
 * Runs the software FFT on the Cortex-A53 (no DMA, no hardware IP):
 *   1. Unpack Q1.15 input words (RTL packing) into float complex array.
 *   2. Start timer.
 *   3. Run sw_fft_compute (in-place, with per-stage ÷2 scaling).
 *   4. Stop timer.
 *   5. Round float results to int16_t and pack into RxBuf (RTL packing).
 *
 * Input TxBuf must be in RTL packing format (bits[31:16]=real, bits[15:0]=imag)
 * Output RxBuf is written in the same RTL packing format.
 * ElapsedTicksOut counts only the FFT compute time, not pack/unpack overhead.
 * ═══════════════════════════════════════════════════════════════════════════ */
static int SwRun(XTmrCtr *Tmr, const u32 *Tx, u32 *Rx, int N,
                 const char *Tag, u32 *ElapsedTicksOut)
{
    int    k;
    u32    TStart, TEnd;
    float  re_f, im_f;
    s16    re_i, im_i;

    xil_printf("Running %s (software, Cortex-A53)...\r\n", Tag);

    /* ── Unpack Q1.15 input into float complex (outside timed region) ─────── */
    for (k = 0; k < N; k++) {
        /* RTL packing: bits[31:16]=real, bits[15:0]=imag                    */
        SwWorkBuf[k].r = (float)(s16)(Tx[k] >> 16)   / Q15_SCALE;
        SwWorkBuf[k].i = (float)(s16)(Tx[k] & 0xFFFFU) / Q15_SCALE;
    }

    /* ── Time only the FFT computation ───────────────────────────────────── */
    XTmrCtr_Reset(Tmr, TIMER_IDX);
    XTmrCtr_Start(Tmr, TIMER_IDX);
    TStart = XTmrCtr_GetValue(Tmr, TIMER_IDX);

    sw_fft_compute(SwWorkBuf, N);

    TEnd = XTmrCtr_GetValue(Tmr, TIMER_IDX);
    XTmrCtr_Stop(Tmr, TIMER_IDX);

    *ElapsedTicksOut = TEnd - TStart;

    /* ── Round float result to Q1.15 and pack into RxBuf (RTL format) ─────── */
    for (k = 0; k < N; k++) {
        re_f = SwWorkBuf[k].r * Q15_SCALE;
        im_f = SwWorkBuf[k].i * Q15_SCALE;

        /* Saturate to int16 range                                            */
        if (re_f >  32767.0f) re_f =  32767.0f;
        if (re_f < -32768.0f) re_f = -32768.0f;
        if (im_f >  32767.0f) im_f =  32767.0f;
        if (im_f < -32768.0f) im_f = -32768.0f;

        re_i = (s16)re_f;
        im_i = (s16)im_f;

        /* RTL packing: bits[31:16]=real, bits[15:0]=imag                    */
        Rx[k] = ((u32)(u16)re_i << 16) | (u16)im_i;
    }

    xil_printf("[%s] Done.\r\n", Tag);
    return XST_SUCCESS;
}
