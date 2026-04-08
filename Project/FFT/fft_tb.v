// ─────────────────────────────────────────────────────────────────────────
// fft_tb.v  (rev 3 – diagnostic, DC-input, step-by-step)
//
// Test: DC input  x[n] = 0x7fff + j*0  for all n = 0..1023
// Expected FFT:   X[0] ≈ 32767,  X[k≠0] = 0   (exact for DC)
//
// After 10 stages of ÷2 scaling, the DC bin retains amplitude:
//   stage-0: x=(a+b)/2=(32767+32767)/2≈32767, y=(a-b)/2=0
//   Subsequent stages preserve bin-0 at ~32767; all others stay 0.
//
// Prints:
//   • First 4 and last 4 TX handshakes (with tready / sample_cnt info)
//   • Every RX sample: bin index, actual real, actual imag, expected,
//     and PASS / FAIL / X status
//   • Final summary: #PASS  #FAIL  #X_BINS
// ─────────────────────────────────────────────────────────────────────────
`timescale 1ns/1ps

module fft_tb;

    localparam N          = 1024;
    localparam LOG2N      = 10;
    localparam DWIDTH     = 16;
    localparam CLK_PERIOD = 10;   // 100 MHz → 10 ns period

    // ── DUT ports ───────────────────────────────────────────────────────
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

    // ── DUT instantiation ────────────────────────────────────────────────
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

    // ── Clock ────────────────────────────────────────────────────────────
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ── Output capture (combinational with output valid) ─────────────────
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

    // ── Stimulus and checking ─────────────────────────────────────────────
    integer i;
    integer pass_cnt, fail_cnt, x_cnt;
    integer exp_r, exp_i;
    integer actual_r, actual_i;

    initial begin
        rst_n         = 0;
        s_axis_tvalid = 0;
        s_axis_tdata  = 0;
        s_axis_tlast  = 0;
        m_axis_tready = 1;
        out_idx       = 0;
        pass_cnt      = 0;
        fail_cnt      = 0;
        x_cnt         = 0;

        repeat(2) @(posedge clk);
        #1; rst_n = 1;

        // ── SECTION 1: TRANSMIT DC INPUT ─────────────────────────────────
        $display("");
        $display("=================================================================");
        $display(" FFT Diagnostic Testbench  –  DC Input Test");
        $display("=================================================================");
        $display(" Input  : x[n] = 0x7fff + j*0x0000  for all n (DC, full-scale)");
        $display(" Expect : X[0] ~ 32767,  X[k!=0] = 0  (exact for DC input)");
        $display("-----------------------------------------------------------------");
        $display(" TX: sending %0d samples ...", N);

        for (i = 0; i < N; i = i + 1) begin
            s_axis_tvalid = 1;
            s_axis_tdata  = {16'h7fff, 16'h0000};   // real=1.0, imag=0
            s_axis_tlast  = (i == N - 1);

            // Correct handshake: sample tready at negedge (stable since last
            // posedge).  When high at negedge it stays high to next posedge.
            @(negedge clk);
            while (!s_axis_tready) @(negedge clk);
            @(posedge clk);   // ← handshake fires here (tvalid=1, tready=1)
            #1;

            // Print first 4 and last 4 handshakes for sanity check
            if (i < 4 || i >= N-4)
                $display("  TX[%4d]: tdata=0x%08h  tlast=%0b  (handshake OK)",
                         i, {16'h7fff, 16'h0000}, s_axis_tlast);
            if (i == 4)
                $display("  TX[   4..%0d]: ... (remaining samples omitted) ...", N-5);
        end
        s_axis_tvalid = 0;
        s_axis_tlast  = 0;
        $display(" TX: all %0d samples sent.", N);

        // ── SECTION 2: WAIT FOR FFT DONE ─────────────────────────────────
        $display("-----------------------------------------------------------------");
        $display(" Waiting for FFT computation to complete ...");
        @(posedge done);
        @(posedge clk);
        $display(" FFT done!  %0d output samples captured.", out_idx);

        // ── SECTION 3: PRINT AND VERIFY EVERY OUTPUT BIN ─────────────────
        $display("-----------------------------------------------------------------");
        $display(" RX Results (all %0d bins):", N);
        $display(" %4s | %9s | %9s | %9s | %9s | Status",
                 "Bin", "Act_Real", "Act_Imag", "Exp_Real", "Exp_Imag");
        $display(" -----|-----------|-----------|-----------|-----------|-------");

        for (i = 0; i < N; i = i + 1) begin
            exp_r = (i == 0) ? 32767 : 0;
            exp_i = 0;

            // ^sig === 1'bx is true when any bit of sig is X or Z
            if ((^out_real[i] === 1'bx) || (^out_imag[i] === 1'bx)) begin
                // Any X/Z bit in output
                $display(" %4d | X(unknown) | X(unknown) | %9d | %9d | FAIL(X)",
                         i, exp_r, exp_i);
                x_cnt    = x_cnt + 1;
                fail_cnt = fail_cnt + 1;

            end else begin
                actual_r = $signed(out_real[i]);
                actual_i = $signed(out_imag[i]);

                if (i == 0) begin
                    // Bin 0: accept 30000..32767 for real, ±50 for imag
                    if (actual_r >= 30000 && actual_i >= -50 && actual_i <= 50) begin
                        $display(" %4d | %9d | %9d | %9d | %9d | PASS",
                                 i, actual_r, actual_i, exp_r, exp_i);
                        pass_cnt = pass_cnt + 1;
                    end else begin
                        $display(" %4d | %9d | %9d | %9d | %9d | FAIL",
                                 i, actual_r, actual_i, exp_r, exp_i);
                        fail_cnt = fail_cnt + 1;
                    end

                end else begin
                    // Bins 1-1023: should be exactly 0; allow ±5 for rounding
                    if (actual_r >= -5 && actual_r <= 5 &&
                        actual_i >= -5 && actual_i <= 5) begin
                        // Only print a few representative passing bins
                        if (i <= 3 || i == 512 || i >= N-3)
                            $display(" %4d | %9d | %9d | %9d | %9d | PASS",
                                     i, actual_r, actual_i, exp_r, exp_i);
                        pass_cnt = pass_cnt + 1;
                    end else begin
                        $display(" %4d | %9d | %9d | %9d | %9d | FAIL",
                                 i, actual_r, actual_i, exp_r, exp_i);
                        fail_cnt = fail_cnt + 1;
                    end
                end
            end
        end

        // ── SECTION 4: SUMMARY ───────────────────────────────────────────
        $display("-----------------------------------------------------------------");
        $display(" SUMMARY: %0d PASS,  %0d FAIL  (%0d bins had X/unknown values)",
                 pass_cnt, fail_cnt, x_cnt);
        if (x_cnt > 0)
            $display(" NOTE: X values mean the data path (RAM/butterfly) has");
            $display("       uninitialized or propagated-X signals. Check that");
            $display("       all 1024 samples were written to RAM before FFT ran.");
        if (fail_cnt == 0)
            $display(" *** ALL %0d BINS PASSED ***", N);
        else
            $display(" *** %0d BIN(S) FAILED ***", fail_cnt);
        $display("=================================================================");
        $finish;
    end

    // ── Timeout watchdog (300 µs > expected ~277 µs) ─────────────────────
    initial begin
        #300000;
        $display("ERROR: Simulation timed out at %0t ns.", $time);
        $display("  out_idx=%0d (expected %0d samples)", out_idx, N);
        $finish;
    end

endmodule
