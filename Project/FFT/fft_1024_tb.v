// ─────────────────────────────────────────────────────────────────────────
// fft_1024_tb.v  -  1024-point FFT testbench (7 tests)
//
// DUT: fft_8_top with N=1024, LOG2N=10, DWIDTH=16 (Q1.15)
//
// All expected values derived analytically and verified with Python DFT.
// The hardware computes the NORMALISED DFT: X[k] = (1/N) * Σ x[n]·W^{nk}
// so every output stays in the same Q1.15 range as the inputs.
//
// ── TEST 1: Real DC ──────────────────────────────────────────────────────
//   x[n] = 0x7FFF + j·0  for all n
//   X[0].real ≈ 32767,  X[0].imag = 0,  X[k≠0] = 0
//   (DC amplitude is preserved exactly under normalised DFT)
//
// ── TEST 2: Real impulse ─────────────────────────────────────────────────
//   x[0] = 0x7FFF, x[1..1023] = 0
//   bit_rev(0) = 0 → impulse lands at mem[0]
//   Every B=0 butterfly: X=Y=(A+0)>>1=A>>1, regardless of twiddle
//   After 10×>>1:  32767 >> 10 = 31  for ALL bins
//
// ── TEST 3: Imaginary DC ─────────────────────────────────────────────────
//   x[n] = 0 + j·0x7FFF  for all n
//   Tests the imaginary datapath independently.
//   X[0].real = 0,  X[0].imag = 32767,  X[k≠0] = 0
//
// ── TEST 4: Complex DC ───────────────────────────────────────────────────
//   x[n] = 0x4000 + j·0x4000 = 0.5 + j·0.5  for all n
//   Both real and imaginary datapaths active simultaneously.
//   X[0].real = 16384,  X[0].imag = 16384,  X[k≠0] = 0
//
// ── TEST 5: Nyquist alternating tone ─────────────────────────────────────
//   x[n] = +32767 (n even),  -32767 (n odd),  imag = 0
//   This is cos(π·n) - a real cosine exactly at the Nyquist frequency.
//   Energy concentrates entirely in bin 512 (the Nyquist bin).
//   X[512].real = 32767,  X[512].imag = 0,  X[k≠512] = 0
//
// ── TEST 6: Complex impulse ──────────────────────────────────────────────
//   x[0] = 0x4000 + j·0x4000,  x[1..1023] = 0
//   Same mechanism as Test 2 but both real and imaginary active.
//   After 10×>>1:  16384 >> 10 = 16  for ALL bins, both components.
//   X[k].real = 16,  X[k].imag = 16  for all k
//
// ── TEST 7: Conjugate symmetry - real cosine at bin 1 ───────────────────
//   x[n] = round(0x4000 · cos(2π·n/1024)),  imag = 0
//   For any real-valued input: X[N-k] = conj(X[k])
//   A cosine at bin 1 splits its energy equally into bins 1 and 1023.
//   X[1].real  = X[1023].real  = 8192
//   X[1].imag  = -X[1023].imag ≈ 0   (self-consistent for bin 1)
//   All other bins = 0
//
// ── Timing budget ────────────────────────────────────────────────────────
//   S_LOAD:      1024 cycles =  10.24 µs
//   Compute: 5120 × 5 cyc  = 256.00 µs
//   S_OUTPUT:    1024 cycles =  10.24 µs
//   Per run ≈ 277 µs → 7 runs ≈ 1940 µs.  Watchdog at 2200 µs.
// ─────────────────────────────────────────────────────────────────────────
`timescale 1ns/1ps

module fft_1024_tb;

    localparam N          = 1024;
    localparam LOG2N      = 10;
    localparam DWIDTH     = 16;
    localparam CLK_PERIOD = 10;    // 100 MHz

    // ── DUT ports ─────────────────────────────────────────────────────
    reg  clk, rst_n;

    reg                  s_axis_tvalid;
    wire                 s_axis_tready;
    reg  [2*DWIDTH-1:0]  s_axis_tdata;
    reg                  s_axis_tlast;

    wire                 m_axis_tvalid;
    reg                  m_axis_tready;
    wire [2*DWIDTH-1:0]  m_axis_tdata;
    wire                 m_axis_tlast;

    wire done;

    // ── DUT ───────────────────────────────────────────────────────────
    fft_1024_top #(
        .N(N), .LOG2N(LOG2N), .DWIDTH(DWIDTH)
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .s_axis_tvalid  (s_axis_tvalid),
        .s_axis_tready  (s_axis_tready),
        .s_axis_tdata   (s_axis_tdata),
        .s_axis_tlast   (s_axis_tlast),
        .m_axis_tvalid  (m_axis_tvalid),
        .m_axis_tready  (m_axis_tready),
        .m_axis_tdata   (m_axis_tdata),
        .m_axis_tlast   (m_axis_tlast),
        .done           (done)
    );

    // ── Clock ─────────────────────────────────────────────────────────
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ── Output capture ────────────────────────────────────────────────
    integer out_idx;
    reg signed [DWIDTH-1:0] out_real [0:N-1];
    reg signed [DWIDTH-1:0] out_imag [0:N-1];

    always @(posedge clk) begin
        if (m_axis_tvalid && m_axis_tready) begin
            out_real[out_idx] <= $signed(m_axis_tdata[2*DWIDTH-1 : DWIDTH]);
            out_imag[out_idx] <= $signed(m_axis_tdata[DWIDTH-1   : 0]);
            out_idx           <= out_idx + 1;
        end
    end

    // ── Shared TX buffer - fill this before calling send_and_wait ─────
    reg signed [DWIDTH-1:0] tx_real [0:N-1];
    reg signed [DWIDTH-1:0] tx_imag [0:N-1];

    // ── Shared check variables ────────────────────────────────────────
    integer ci, pass_cnt, fail_cnt, x_cnt, total_pass, total_fail;
    integer exp_r, exp_i, act_r, act_i;

    // ── Send task: iterates tx_real/tx_imag, waits for done ──────────
    integer tx_i;
    task send_and_wait;
        begin
            out_idx = 0;
            for (tx_i = 0; tx_i < N; tx_i = tx_i + 1) begin
                s_axis_tvalid = 1'b1;
                s_axis_tdata  = {tx_real[tx_i][DWIDTH-1:0],
                                 tx_imag[tx_i][DWIDTH-1:0]};
                s_axis_tlast  = (tx_i == N - 1);

                @(negedge clk);
                while (!s_axis_tready) @(negedge clk);
                @(posedge clk);
                #1;

                if (tx_i < 4 || tx_i >= N-4)
                    $display("  TX[%4d]: real=0x%04h(%6d)  imag=0x%04h(%6d)  tlast=%0b",
                             tx_i,
                             tx_real[tx_i] & 16'hFFFF, $signed(tx_real[tx_i]),
                             tx_imag[tx_i] & 16'hFFFF, $signed(tx_imag[tx_i]),
                             s_axis_tlast);
                else if (tx_i == 4)
                    $display("  TX[   4..%0d]: ... (omitted) ...", N-5);
            end
            s_axis_tvalid = 1'b0;
            s_axis_tlast  = 1'b0;
            s_axis_tdata  = {2*DWIDTH{1'b0}};

            @(posedge done);
            @(posedge clk);
            // The rev5 controller asserts done on the same edge that launches
            // the last output beat. Our capture process stores that final beat
            // with nonblocking assignments on the following clock edge, so
            // wait one delta before reading out_idx / out_real / out_imag.
            #1;
            $display("  FFT done - %0d bins captured.", out_idx);
        end
    endtask

    // ── Check task: compare out_real/imag against exp_real/exp_imag ──
    // Tolerance: abs(actual - expected) <= tol for each component.
    // Only bins in the "non-trivial" set are checked strictly; rest use tol_rest.
    // For simplicity: caller sets exp_r/exp_i per bin inside the loop.
    // This task just prints the header/footer and runs the loop.
    // The loop body is inlined per test for clarity.

    // ── Print helpers ─────────────────────────────────────────────────
    task print_header;
        begin
            $display(" %4s | %9s | %9s | %9s | %9s | Status",
                     "Bin", "Act_Real", "Act_Imag", "Exp_Real", "Exp_Imag");
            $display(" -----|-----------|-----------|-----------|-----------|-------");
        end
    endtask

    task check_bin;
        input integer bin;
        input integer e_r, e_i;
        input integer tol_r, tol_i;   // abs tolerance per component
        input integer always_print;   // 1 = always print; 0 = only print failures
        begin
            if ((^out_real[bin] === 1'bx) || (^out_imag[bin] === 1'bx)) begin
                $display(" %4d | X(unknown) | X(unknown) | %9d | %9d | FAIL(X)",
                         bin, e_r, e_i);
                x_cnt    = x_cnt + 1;
                fail_cnt = fail_cnt + 1;
            end else begin
                act_r = $signed(out_real[bin]);
                act_i = $signed(out_imag[bin]);
                if ((act_r >= e_r - tol_r) && (act_r <= e_r + tol_r) &&
                    (act_i >= e_i - tol_i) && (act_i <= e_i + tol_i)) begin
                    if (always_print)
                        $display(" %4d | %9d | %9d | %9d | %9d | PASS",
                                 bin, act_r, act_i, e_r, e_i);
                    pass_cnt = pass_cnt + 1;
                end else begin
                    $display(" %4d | %9d | %9d | %9d | %9d | FAIL",
                             bin, act_r, act_i, e_r, e_i);
                    fail_cnt = fail_cnt + 1;
                end
            end
        end
    endtask

    // ── Loop helper: check all N bins against a flat expected value ───
    // Bins in except_lo..except_hi get (e_r_hot, e_i_hot, tol_hot).
    // All other bins get (0, 0, tol_zero).
    // Prints all "hot" bins; only prints passing zero bins at 0,1,2,512,1021,1022,1023.
    integer li;
    task check_all_bins;
        input integer except_lo, except_hi;   // hot bin range (inclusive)
        input integer e_r_hot, e_i_hot;
        input integer tol_hot;
        input integer tol_zero;
        begin
            for (li = 0; li < N; li = li + 1) begin
                if (li >= except_lo && li <= except_hi) begin
                    check_bin(li, e_r_hot, e_i_hot, tol_hot, tol_hot, 1);
                end else begin
                    // zero-expected bin: only print a few representatives
                    if (li <= 2 || li == 512 || li >= N-3)
                        check_bin(li, 0, 0, tol_zero, tol_zero, 1);
                    else
                        check_bin(li, 0, 0, tol_zero, tol_zero, 0);
                end
            end
        end
    endtask

    // ── Loop helper: check all bins same expected value ───────────────
    integer ai;
    task check_all_same;
        input integer e_r, e_i, tol;
        begin
            for (ai = 0; ai < N; ai = ai + 1) begin
                if (ai <= 2 || ai == 512 || ai >= N-3)
                    check_bin(ai, e_r, e_i, tol, tol, 1);
                else
                    check_bin(ai, e_r, e_i, tol, tol, 0);
            end
        end
    endtask

    // ── Init helpers ──────────────────────────────────────────────────
    integer ii;
    task fill_dc_real;
        begin
            for (ii = 0; ii < N; ii = ii + 1) begin
                tx_real[ii] = 16'h7FFF;
                tx_imag[ii] = 16'h0000;
            end
        end
    endtask

    task fill_impulse_real;
        begin
            tx_real[0] = 16'h7FFF; tx_imag[0] = 16'h0000;
            for (ii = 1; ii < N; ii = ii + 1) begin
                tx_real[ii] = 16'h0000; tx_imag[ii] = 16'h0000;
            end
        end
    endtask

    task fill_dc_imag;
        begin
            for (ii = 0; ii < N; ii = ii + 1) begin
                tx_real[ii] = 16'h0000;
                tx_imag[ii] = 16'h7FFF;
            end
        end
    endtask

    task fill_dc_complex;
        begin
            for (ii = 0; ii < N; ii = ii + 1) begin
                tx_real[ii] = 16'h4000;   // 0.5
                tx_imag[ii] = 16'h4000;   // 0.5
            end
        end
    endtask

    task fill_nyquist;
        begin
            for (ii = 0; ii < N; ii = ii + 1) begin
                tx_real[ii] = (ii % 2 == 0) ? 16'h7FFF : 16'h8001; // +32767/-32767
                tx_imag[ii] = 16'h0000;
            end
        end
    endtask

    task fill_complex_impulse;
        begin
            tx_real[0] = 16'h4000; tx_imag[0] = 16'h4000;
            for (ii = 1; ii < N; ii = ii + 1) begin
                tx_real[ii] = 16'h0000; tx_imag[ii] = 16'h0000;
            end
        end
    endtask

    // Test 7: cosine at bin 1. x[n] = round(16384 * cos(2*pi*n/1024)).
    // Pre-computed with the formula below.  We store the first 16 values
    // explicitly; the rest follow the same cosine pattern.
    // Exact computation: use fixed-point approximation table.
    // Since we cannot call math functions in Verilog, we generate it
    // using a parameter-driven always block trick with $cos (not supported
    // in XSim), so instead we pre-compute and hardcode the 1024 values
    // using an $fopen/$fscan approach or a simpler approximation.
    //
    // Simpler approach that is XSim-compatible:
    //   cos(2*pi*n/N) sampled at N=1024 points, amplitude 16384 (0x4000).
    //   x[n].real = round(16384 * cos(2*pi*n/1024))
    //   This equals the twiddle factor real part scaled to amplitude 16384.
    //   We can derive it from the twiddle ROM values:
    //     cos(2*pi*k/1024) = w_real[k] (Q1.15 → divide by 32768 to get fraction)
    //   But we don't have twiddle values here.
    //
    // Instead: use a Verilog integer cosine approximation valid for N=1024:
    //   angle_i = (n * 65536) / 1024 = n * 64   (in 1/65536 radians units)
    //   This approach is non-trivial without $cos.
    //
    // Cleanest XSim solution: pre-compute the first period and store it.
    // cos(2*pi*n/1024) for n=0..1023, amplitude 16384:
    //   n=0:   16384, n=1: 16383, n=2: 16381, n=4: 16375, n=8: 16352, n=16: 16257
    //   n=256: 0,     n=512: -16384, n=768: 0, n=1023: 16383
    //
    // We generate these in the $initial block using a Verilog real variable.
    integer        cos_n;
    real           angle_rad;
    real           cos_val;
    reg signed [DWIDTH-1:0] cosine_lut [0:N-1];

    initial begin
        // Pre-compute cosine LUT - real variables are supported in XSim
        for (cos_n = 0; cos_n < N; cos_n = cos_n + 1) begin
            angle_rad = 2.0 * 3.14159265358979 * cos_n / N;
            cos_val   = $cos(angle_rad) * 16384.0;
            // round toward nearest integer
            if (cos_val >= 0)
                cosine_lut[cos_n] = $rtoi(cos_val + 0.5);
            else
                cosine_lut[cos_n] = $rtoi(cos_val - 0.5);
        end
    end

    task fill_cosine_bin1;
        begin
            for (ii = 0; ii < N; ii = ii + 1) begin
                tx_real[ii] = cosine_lut[ii];
                tx_imag[ii] = 16'h0000;
            end
        end
    endtask

    // ═════════════════════════════════════════════════════════════════
    // MAIN STIMULUS
    // ═════════════════════════════════════════════════════════════════
    initial begin
        rst_n         = 0;
        s_axis_tvalid = 0;
        s_axis_tdata  = 0;
        s_axis_tlast  = 0;
        m_axis_tready = 1;
        total_pass    = 0;
        total_fail    = 0;
        out_idx       = 0;

        repeat(4) @(posedge clk);
        #1 rst_n = 1;
        repeat(2) @(posedge clk);

        // =============================================================
        // TEST 1 - Real DC
        // =============================================================
        $display("");
        $display("================================================================");
        $display(" TEST 1: Real DC   x[n] = 0x7FFF+j*0  for all n");
        $display(" Why: DC preserved under normalised DFT. All other bins cancel.");
        $display(" Expected: X[0].real=32767  X[0].imag=0  X[k!=0]=0");
        $display("================================================================");
        fill_dc_real;
        send_and_wait;
        print_header;
        pass_cnt = 0; fail_cnt = 0; x_cnt = 0;
        check_all_bins(0, 0, 32767, 0, 2000, 5);
        $display(" Test 1: %0d PASS, %0d FAIL (%0d X)", pass_cnt, fail_cnt, x_cnt);
        total_pass = total_pass + pass_cnt; total_fail = total_fail + fail_cnt;
        repeat(4) @(posedge clk);

        // =============================================================
        // TEST 2 - Real impulse
        // =============================================================
        $display("");
        $display("================================================================");
        $display(" TEST 2: Real impulse   x[0]=0x7FFF, x[1..1023]=0");
        $display(" Why: bit_rev(0)=0 so impulse lands at mem[0].");
        $display("      Every butterfly B=0 → X=Y=A>>1 whatever twiddle is.");
        $display("      After 10x>>1: 32767>>10 = 31 for ALL 1024 bins.");
        $display(" Expected: X[k].real=31  X[k].imag=0  for all k");
        $display("================================================================");
        fill_impulse_real;
        send_and_wait;
        print_header;
        pass_cnt = 0; fail_cnt = 0; x_cnt = 0;
        check_all_same(31, 0, 3);   // tol=3: covers truncation scatter over 10 stages
        $display(" Test 2: %0d PASS, %0d FAIL (%0d X)", pass_cnt, fail_cnt, x_cnt);
        total_pass = total_pass + pass_cnt; total_fail = total_fail + fail_cnt;
        repeat(4) @(posedge clk);

        // =============================================================
        // TEST 3 - Imaginary DC
        // =============================================================
        $display("");
        $display("================================================================");
        $display(" TEST 3: Imaginary DC   x[n] = 0 + j*0x7FFF  for all n");
        $display(" Why: Tests imaginary datapath in isolation.");
        $display("      Normalised DFT of DC: X[0] = the DC value itself.");
        $display("      Imaginary part passes through; real part = 0 throughout.");
        $display(" Expected: X[0].real=0  X[0].imag=32767  X[k!=0]=0");
        $display("================================================================");
        fill_dc_imag;
        send_and_wait;
        print_header;
        pass_cnt = 0; fail_cnt = 0; x_cnt = 0;
        // X[0]: real=0 (tol 50), imag=32767 (tol 2000)
        check_bin(0, 0, 32767, 50, 2000, 1);
        // X[k≠0]: all zero
        for (ci = 1; ci < N; ci = ci + 1) begin
            if (ci <= 3 || ci == 512 || ci >= N-3)
                check_bin(ci, 0, 0, 5, 5, 1);
            else
                check_bin(ci, 0, 0, 5, 5, 0);
        end
        $display(" Test 3: %0d PASS, %0d FAIL (%0d X)", pass_cnt, fail_cnt, x_cnt);
        total_pass = total_pass + pass_cnt; total_fail = total_fail + fail_cnt;
        repeat(4) @(posedge clk);

        // =============================================================
        // TEST 4 - Complex DC
        // =============================================================
        $display("");
        $display("================================================================");
        $display(" TEST 4: Complex DC   x[n] = 0x4000+j*0x4000 = 0.5+j*0.5 for all n");
        $display(" Why: Both real and imaginary datapaths active simultaneously.");
        $display("      Normalised DFT of DC: X[0] = input value = 0.5+j*0.5.");
        $display("      Magnitude = 0.7071 < 1.0 so no overflow risk at input.");
        $display(" Expected: X[0].real=16384  X[0].imag=16384  X[k!=0]=0");
        $display("================================================================");
        fill_dc_complex;
        send_and_wait;
        print_header;
        pass_cnt = 0; fail_cnt = 0; x_cnt = 0;
        check_bin(0, 16384, 16384, 1000, 1000, 1);
        for (ci = 1; ci < N; ci = ci + 1) begin
            if (ci <= 3 || ci == 512 || ci >= N-3)
                check_bin(ci, 0, 0, 5, 5, 1);
            else
                check_bin(ci, 0, 0, 5, 5, 0);
        end
        $display(" Test 4: %0d PASS, %0d FAIL (%0d X)", pass_cnt, fail_cnt, x_cnt);
        total_pass = total_pass + pass_cnt; total_fail = total_fail + fail_cnt;
        repeat(4) @(posedge clk);

        // =============================================================
        // TEST 5 - Nyquist alternating tone
        // =============================================================
        $display("");
        $display("================================================================");
        $display(" TEST 5: Nyquist tone   x[n]=+32767 (n even), -32767 (n odd)");
        $display(" Why: x[n]=cos(pi*n) is a real cosine at exactly bin N/2=512.");
        $display("      It is the highest representable frequency in the DFT.");
        $display("      Normalised DFT: X[512]=32767, all others=0.");
        $display("      Imaginary = 0 because x[n] is real AND bin 512 is");
        $display("      self-conjugate (N-512=512), so no imaginary component.");
        $display(" Expected: X[512].real=32767  X[512].imag=0  X[k!=512]=0");
        $display("================================================================");
        fill_nyquist;
        send_and_wait;
        print_header;
        pass_cnt = 0; fail_cnt = 0; x_cnt = 0;
        check_all_bins(512, 512, 32767, 0, 2000, 5);
        $display(" Test 5: %0d PASS, %0d FAIL (%0d X)", pass_cnt, fail_cnt, x_cnt);
        total_pass = total_pass + pass_cnt; total_fail = total_fail + fail_cnt;
        repeat(4) @(posedge clk);

        // =============================================================
        // TEST 6 - Complex impulse
        // =============================================================
        $display("");
        $display("================================================================");
        $display(" TEST 6: Complex impulse   x[0]=0x4000+j*0x4000, x[1..1023]=0");
        $display(" Why: Tests imaginary path through all 10 butterfly stages.");
        $display("      bit_rev(0)=0 → impulse at mem[0]; all other mem=0.");
        $display("      Every B=0 butterfly: X=Y=(A+0)>>1 for BOTH real and imag.");
        $display("      After 10x>>1: 16384>>10=16 for ALL bins, both components.");
        $display(" Expected: X[k].real=16  X[k].imag=16  for all k");
        $display("================================================================");
        fill_complex_impulse;
        send_and_wait;
        print_header;
        pass_cnt = 0; fail_cnt = 0; x_cnt = 0;
        check_all_same(16, 16, 3);
        $display(" Test 6: %0d PASS, %0d FAIL (%0d X)", pass_cnt, fail_cnt, x_cnt);
        total_pass = total_pass + pass_cnt; total_fail = total_fail + fail_cnt;
        repeat(4) @(posedge clk);

        // =============================================================
        // TEST 7 - Conjugate symmetry: real cosine at bin 1
        // =============================================================
        $display("");
        $display("================================================================");
        $display(" TEST 7: Real cosine at bin 1   x[n]=round(16384*cos(2*pi*n/1024))");
        $display(" Why: For any real input, X[N-k] = conj(X[k]) must hold.");
        $display("      A cosine at bin 1 splits its energy: X[1] and X[1023]");
        $display("      each receive half the amplitude (= 16384/2 = 8192).");
        $display("      Conjugate symmetry requires:");
        $display("        X[1].real  = X[1023].real  = 8192");
        $display("        X[1].imag  = -X[1023].imag  (both ~0 for bin 1)");
        $display("      All other bins = 0.");
        $display(" Expected: X[1].real=8192  X[1].imag=0");
        $display("           X[1023].real=8192  X[1023].imag=0");
        $display("           X[k!=1,1023] = 0");
        $display("================================================================");
        fill_cosine_bin1;
        send_and_wait;
        print_header;
        pass_cnt = 0; fail_cnt = 0; x_cnt = 0;

        // Check hot bins 1 and 1023
        check_bin(1,    8192, 0, 500, 100, 1);
        check_bin(1023, 8192, 0, 500, 100, 1);

        // Explicitly verify conjugate symmetry
        if ((^out_real[1]    !== 1'bx) && (^out_real[1023] !== 1'bx) &&
            (^out_imag[1]    !== 1'bx) && (^out_imag[1023] !== 1'bx)) begin
            $display(" Conj symmetry check:");
            $display("   X[1].real(%0d) vs X[1023].real(%0d) - %s",
                     $signed(out_real[1]), $signed(out_real[1023]),
                     ($signed(out_real[1]) == $signed(out_real[1023])) ? "MATCH" : "MISMATCH");
            $display("   X[1].imag(%0d) vs -X[1023].imag(%0d) - %s",
                     $signed(out_imag[1]), -$signed(out_imag[1023]),
                     ($signed(out_imag[1]) == -$signed(out_imag[1023])) ? "MATCH" : "MISMATCH");
        end

        // Check zero bins (skip 1 and 1023)
        for (ci = 0; ci < N; ci = ci + 1) begin
            if (ci == 1 || ci == 1023) begin
                // already checked above
            end else begin
                if (ci <= 3 || ci == 512 || ci >= N-3)
                    check_bin(ci, 0, 0, 5, 5, 1);
                else
                    check_bin(ci, 0, 0, 5, 5, 0);
            end
        end

        $display(" Test 7: %0d PASS, %0d FAIL (%0d X)", pass_cnt, fail_cnt, x_cnt);
        total_pass = total_pass + pass_cnt; total_fail = total_fail + fail_cnt;

        // =============================================================
        // OVERALL SUMMARY
        // =============================================================
        $display("");
        $display("================================================================");
        $display(" OVERALL: %0d PASS,  %0d FAIL  across all 7 tests",
                 total_pass, total_fail);
        if (total_fail == 0)
            $display(" *** ALL TESTS PASSED - FFT datapath verified ***");
        else
            $display(" *** %0d FAILURE(S) - debug before hardware deployment ***",
                     total_fail);
        $display("================================================================");
        $finish;
    end

    // ── Watchdog: 2200 µs covers 7 full FFT runs with margin ─────────
    initial begin
        #2200000;
        $display("ERROR: Simulation timed out at %0t ns.", $time);
        $display("  out_idx=%0d (expected %0d), done=%0b", out_idx, N, done);
        $finish;
    end

endmodule
