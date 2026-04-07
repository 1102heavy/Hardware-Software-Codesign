`timescale 1ns/1ps

module fft_8_tb;

    localparam N          = 8;
    localparam LOG2N      = 3;
    localparam DWIDTH     = 16;
    localparam CLK_PERIOD = 10;  // 100 MHz

    // ── DUT ports ──────────────────────────────────────────────────────
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

    // ── DUT instantiation ──────────────────────────────────────────────
    fft_top_8 #(
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

    // ── Clock ──────────────────────────────────────────────────────────
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ── Output capture ─────────────────────────────────────────────────
    integer out_idx;
    reg signed [DWIDTH-1:0] out_real [0:N-1];
    reg signed [DWIDTH-1:0] out_imag [0:N-1];

    always @(posedge clk) begin
        if (m_axis_tvalid && m_axis_tready) begin
            out_real[out_idx] <= $signed(m_axis_tdata[2*DWIDTH-1:DWIDTH]);
            out_imag[out_idx] <= $signed(m_axis_tdata[DWIDTH-1:0]);
            out_idx           <= out_idx + 1;
        end
    end

    // ── Shared check variables ────────────────────────────────────────
    integer ci, pass_cnt, fail_cnt, x_cnt;
    integer exp_r, exp_i, act_r, act_i;
    integer total_pass, total_fail;

    // ── Test vector arrays ─────────────────────────────────────────────
    reg [2*DWIDTH-1:0] dc_samples  [0:N-1];
    reg [2*DWIDTH-1:0] imp_samples [0:N-1];

    // ── Initialise test vectors ────────────────────────────────────────
    integer vi;
    initial begin
        for (vi = 0; vi < N; vi = vi + 1)
            dc_samples[vi] = {16'h7FFF, 16'h0000};   // DC: real=1.0, imag=0

        imp_samples[0] = {16'h7FFF, 16'h0000};       // Impulse: x[0]=1.0
        for (vi = 1; vi < N; vi = vi + 1)
            imp_samples[vi] = {16'h0000, 16'h0000};  // x[1..7]=0
    end

    // ── Helper task: send N samples and wait for done ─────────────────
    // use_dc = 1 => send dc_samples
    // use_dc = 0 => send imp_samples
    integer tx_i;
    task send_and_wait;
        input use_dc;
        begin
            out_idx = 0;

            for (tx_i = 0; tx_i < N; tx_i = tx_i + 1) begin
                s_axis_tvalid = 1'b1;
                s_axis_tdata  = (use_dc) ? dc_samples[tx_i] : imp_samples[tx_i];
                s_axis_tlast  = (tx_i == N - 1);

                @(negedge clk);
                while (!s_axis_tready)
                    @(negedge clk);

                @(posedge clk);
                #1;
                $display("  TX[%0d]: real=0x%04h  imag=0x%04h  tlast=%0b",
                         tx_i,
                         s_axis_tdata[2*DWIDTH-1:DWIDTH],
                         s_axis_tdata[DWIDTH-1:0],
                         s_axis_tlast);
            end

            s_axis_tvalid = 1'b0;
            s_axis_tlast  = 1'b0;
            s_axis_tdata  = {2*DWIDTH{1'b0}};

            @(posedge done);
            @(posedge clk);
            $display("  FFT done - %0d bins captured.", out_idx);
        end
    endtask

    // ═══════════════════════════════════════════════════════════════════
    // MAIN STIMULUS
    // ═══════════════════════════════════════════════════════════════════
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

        // ==============================================================
        // TEST 1 - DC input
        // ==============================================================
        $display("");
        $display("================================================================");
        $display(" TEST 1: DC input  x[n] = 0x7FFF + j*0  for all n = 0..7");
        $display(" Expected: X[0].real ~ 32767,  X[k!=0] = 0 + j*0  (exact)");
        $display("================================================================");

        send_and_wait(1'b1);

        $display(" Bin | Act_Real | Act_Imag | Exp_Real | Exp_Imag | Status");
        $display(" ----|----------|----------|----------|----------|-------");

        pass_cnt = 0;
        fail_cnt = 0;
        x_cnt    = 0;

        for (ci = 0; ci < N; ci = ci + 1) begin
            exp_r = (ci == 0) ? 32767 : 0;
            exp_i = 0;

            if ((^out_real[ci] === 1'bx) || (^out_imag[ci] === 1'bx)) begin
                $display(" %3d | X        | X        | %8d | %8d | FAIL(X)",
                         ci, exp_r, exp_i);
                x_cnt    = x_cnt + 1;
                fail_cnt = fail_cnt + 1;
            end
            else begin
                act_r = $signed(out_real[ci]);
                act_i = $signed(out_imag[ci]);

                if (ci == 0) begin
                    if (act_r >= 30000 && act_i >= -50 && act_i <= 50) begin
                        $display(" %3d | %8d | %8d | %8d | %8d | PASS",
                                 ci, act_r, act_i, exp_r, exp_i);
                        pass_cnt = pass_cnt + 1;
                    end
                    else begin
                        $display(" %3d | %8d | %8d | %8d | %8d | FAIL",
                                 ci, act_r, act_i, exp_r, exp_i);
                        fail_cnt = fail_cnt + 1;
                    end
                end
                else begin
                    if (act_r >= -5 && act_r <= 5 && act_i >= -5 && act_i <= 5) begin
                        $display(" %3d | %8d | %8d | %8d | %8d | PASS",
                                 ci, act_r, act_i, exp_r, exp_i);
                        pass_cnt = pass_cnt + 1;
                    end
                    else begin
                        $display(" %3d | %8d | %8d | %8d | %8d | FAIL",
                                 ci, act_r, act_i, exp_r, exp_i);
                        fail_cnt = fail_cnt + 1;
                    end
                end
            end
        end

        $display(" Test 1 summary: %0d PASS, %0d FAIL (%0d X-bins)",
                 pass_cnt, fail_cnt, x_cnt);
        total_pass = total_pass + pass_cnt;
        total_fail = total_fail + fail_cnt;

        // Let DUT return to idle
        repeat(4) @(posedge clk);

        // ==============================================================
        // TEST 2 - Impulse input
        // ==============================================================
        $display("");
        $display("================================================================");
        $display(" TEST 2: Impulse  x[0]=0x7FFF,  x[1..7]=0");
        $display(" Hand trace (3 stages of >>1 scaling):");
        $display("   Stage 0: mem[0]=0x3FFF, mem[1]=0x3FFF, rest=0");
        $display("   Stage 1: mem[0..3]=0x1FFF, rest=0");
        $display("   Stage 2: mem[0..7]=0x0FFF=4095  (all 8 bins equal)");
        $display(" Expected: X[k].real = 4095 +/-2,  X[k].imag = 0");
        $display("================================================================");

        send_and_wait(1'b0);

        $display(" Bin | Act_Real | Act_Imag | Exp_Real | Exp_Imag | Status");
        $display(" ----|----------|----------|----------|----------|-------");

        pass_cnt = 0;
        fail_cnt = 0;
        x_cnt    = 0;

        for (ci = 0; ci < N; ci = ci + 1) begin
            exp_r = 4095;
            exp_i = 0;

            if ((^out_real[ci] === 1'bx) || (^out_imag[ci] === 1'bx)) begin
                $display(" %3d | X        | X        | %8d | %8d | FAIL(X)",
                         ci, exp_r, exp_i);
                x_cnt    = x_cnt + 1;
                fail_cnt = fail_cnt + 1;
            end
            else begin
                act_r = $signed(out_real[ci]);
                act_i = $signed(out_imag[ci]);

                if (act_r >= 4090 && act_r <= 4097 &&
                    act_i >= -5   && act_i <= 5) begin
                    $display(" %3d | %8d | %8d | %8d | %8d | PASS",
                             ci, act_r, act_i, exp_r, exp_i);
                    pass_cnt = pass_cnt + 1;
                end
                else begin
                    $display(" %3d | %8d | %8d | %8d | %8d | FAIL",
                             ci, act_r, act_i, exp_r, exp_i);
                    fail_cnt = fail_cnt + 1;
                end
            end
        end

        $display(" Test 2 summary: %0d PASS, %0d FAIL (%0d X-bins)",
                 pass_cnt, fail_cnt, x_cnt);
        total_pass = total_pass + pass_cnt;
        total_fail = total_fail + fail_cnt;

        // ==============================================================
        // OVERALL SUMMARY
        // ==============================================================
        $display("");
        $display("================================================================");
        $display(" OVERALL: %0d PASS,  %0d FAIL  across both tests (%0d bins each)",
                 total_pass, total_fail, N);
        if (total_fail == 0)
            $display(" *** ALL TESTS PASSED - safe to scale to N=1024 ***");
        else
            $display(" *** %0d FAILURE(S) - debug before scaling up ***", total_fail);
        $display("================================================================");

        $finish;
    end

    // ── Watchdog ───────────────────────────────────────────────────────
    initial begin
        #50000;
        $display("ERROR: Simulation timed out at %0t ns.", $time);
        $display("  out_idx=%0d (expected %0d), done=%0b, state debug needed.",
                 out_idx, N, done);
        $finish;
    end

endmodule
