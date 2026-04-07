// ─────────────────────────────────────────────────────────────────────────
// fft_controller.v  (rev 6 - BRAM-aware Mealy FSM, 3 cycles per butterfly)
//
// This revision retimes the FFT controller around synchronous block memories.
// A naive "ram_style=block" swap would break the old async-read schedule, so
// the controller is updated together with the memory architecture:
//
//   - fft_data_ram becomes true dual-port block RAM
//   - fft_twiddle_rom becomes synchronous block ROM
//   - RAM/ROM addresses and write controls are driven combinationally
//     from the current state and counters so the memories see valid control
//     signals before the active clock edge
//
// The result is a shorter butterfly schedule:
//   S_READ   : present addresses, BRAM/ROM capture A/B/W at this edge
//   S_LATCH  : register A/B/W into the butterfly input registers
//   S_WRITE  : write both butterfly outputs back through the dual ports
//
// Performance @ N=1024:
//   Rev 5 distributed-RAM design : 27,648 cycles ≈ 276.5 us @ 100 MHz
//   Rev 6 BRAM-backed design     : 17,409 cycles ≈ 174.1 us @ 100 MHz
//   Saving                       : 10,239 cycles ≈ 102.4 us (37.0%)
// ─────────────────────────────────────────────────────────────────────────
`timescale 1ns/1ps

module fft_controller #(
    parameter N      = 1024,
    parameter LOG2N  = 10,
    parameter DWIDTH = 16
)(
    input  wire clk,
    input  wire rst_n,

    // ── AXI4-Stream Slave ─────────────────────────────────────────────
    input  wire                       s_axis_tvalid,
    output reg                        s_axis_tready,
    input  wire [2*DWIDTH-1:0]        s_axis_tdata,
    input  wire                       s_axis_tlast,

    // ── AXI4-Stream Master ────────────────────────────────────────────
    output reg                        m_axis_tvalid,
    input  wire                       m_axis_tready,
    output reg  [2*DWIDTH-1:0]        m_axis_tdata,
    output reg                        m_axis_tlast,

    // ── Data RAM - true dual-port BRAM ────────────────────────────────
    output reg  [LOG2N-1:0]           ram_addr_a,
    output reg  [LOG2N-1:0]           ram_addr_b,
    output reg  [2*DWIDTH-1:0]        ram_din_a,
    output reg  [2*DWIDTH-1:0]        ram_din_b,
    output reg                        ram_we_a,
    output reg                        ram_we_b,
    input  wire [2*DWIDTH-1:0]        ram_dout_a,
    input  wire [2*DWIDTH-1:0]        ram_dout_b,

    // ── Twiddle ROM ───────────────────────────────────────────────────
    output reg  [LOG2N-2:0]           rom_addr,
    input  wire signed [DWIDTH-1:0]   rom_w_real,
    input  wire signed [DWIDTH-1:0]   rom_w_imag,

    // ── Butterfly ─────────────────────────────────────────────────────
    output reg  signed [DWIDTH-1:0]   bfly_a_r, bfly_a_i,
    output reg  signed [DWIDTH-1:0]   bfly_b_r, bfly_b_i,
    output reg  signed [DWIDTH-1:0]   bfly_w_r, bfly_w_i,
    input  wire signed [DWIDTH:0]     bfly_x_r, bfly_x_i,
    input  wire signed [DWIDTH:0]     bfly_y_r, bfly_y_i,

    // ── Status ────────────────────────────────────────────────────────
    output reg                        done
);

    // ── State encoding (8 states → 3 bits) ───────────────────────────
    localparam [2:0]
        S_IDLE       = 3'd0,
        S_LOAD       = 3'd1,
        S_READ       = 3'd2,
        S_LATCH      = 3'd3,
        S_WRITE      = 3'd4,
        S_PRE_OUTPUT = 3'd5,
        S_OUTPUT     = 3'd6;

    reg [2:0] state, next_state;

    // ── Control counters ──────────────────────────────────────────────
    reg [LOG2N-1:0] sample_cnt;
    reg [3:0]        stage_reg;
    reg [LOG2N-1:0] group_start;
    reg [LOG2N-1:0] pair_idx;

    // ── Address generation (combinational) ────────────────────────────
    wire [LOG2N-1:0] span         = {{(LOG2N-1){1'b0}}, 1'b1} << stage_reg;
    wire [LOG2N:0]   group_size   = {1'b0, span} << 1;
    wire [LOG2N-1:0] addr_A       = group_start + pair_idx;
    wire [LOG2N-1:0] addr_B       = addr_A + span;
    wire [LOG2N-2:0] tw_addr_comb = pair_idx[LOG2N-2:0] << ((LOG2N-1) - stage_reg);

    wire last_butterfly = ((pair_idx + 1 >= span) &&
                           (({1'b0, group_start} + group_size) >= N) &&
                           (stage_reg >= LOG2N - 1));

    wire [2*DWIDTH-1:0] scaled_x = {bfly_x_r[DWIDTH:1], bfly_x_i[DWIDTH:1]};
    wire [2*DWIDTH-1:0] scaled_y = {bfly_y_r[DWIDTH:1], bfly_y_i[DWIDTH:1]};

    function automatic [LOG2N-1:0] bit_rev;
        input [LOG2N-1:0] in_val;
        integer bit_idx;
        begin
            bit_rev = {LOG2N{1'b0}};
            for (bit_idx = 0; bit_idx < LOG2N; bit_idx = bit_idx + 1)
                bit_rev[LOG2N-1-bit_idx] = in_val[bit_idx];
        end
    endfunction

    wire [LOG2N-1:0] sample_cnt_bitrev;
    assign sample_cnt_bitrev = bit_rev(sample_cnt);

    // ══════════════════════════════════════════════════════════════════
    // STATE REGISTER
    // ══════════════════════════════════════════════════════════════════
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= next_state;
    end

    // ══════════════════════════════════════════════════════════════════
    // NEXT-STATE LOGIC
    // ══════════════════════════════════════════════════════════════════
    always @(*) begin
        next_state = state;

        case (state)
            S_IDLE: begin
                if (s_axis_tvalid)
                    next_state = S_LOAD;
            end

            S_LOAD: begin
                if (s_axis_tvalid && s_axis_tlast)
                    next_state = S_READ;
            end

            S_READ:       next_state = S_LATCH;
            S_LATCH:      next_state = S_WRITE;
            S_WRITE:      next_state = last_butterfly ? S_PRE_OUTPUT : S_READ;
            S_PRE_OUTPUT: next_state = S_OUTPUT;

            S_OUTPUT: begin
                if (m_axis_tready && sample_cnt == N - 1)
                    next_state = S_IDLE;
            end

            default: next_state = S_IDLE;
        endcase
    end

    // ══════════════════════════════════════════════════════════════════
    // REGISTERED STATE/CHECKPOINT DATA
    // ══════════════════════════════════════════════════════════════════
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bfly_a_r    <= 0; bfly_a_i    <= 0;
            bfly_b_r    <= 0; bfly_b_i    <= 0;
            bfly_w_r    <= 0; bfly_w_i    <= 0;
            done        <= 0;
            sample_cnt  <= 0;
            stage_reg   <= 0;
            group_start <= 0;
            pair_idx    <= 0;
        end else begin
            done <= 1'b0;

            case (state)
                S_LOAD: begin
                    if (s_axis_tvalid) begin
                        if (s_axis_tlast) begin
                            sample_cnt  <= 0;
                            stage_reg   <= 0;
                            group_start <= 0;
                            pair_idx    <= 0;
                        end else begin
                            sample_cnt <= sample_cnt + 1'b1;
                        end
                    end
                end

                S_LATCH: begin
                    // RAM/ROM outputs were captured at the S_READ edge and are
                    // stable throughout this cycle.
                    bfly_a_r <= $signed(ram_dout_a[2*DWIDTH-1 : DWIDTH]);
                    bfly_a_i <= $signed(ram_dout_a[DWIDTH-1   : 0]);
                    bfly_b_r <= $signed(ram_dout_b[2*DWIDTH-1 : DWIDTH]);
                    bfly_b_i <= $signed(ram_dout_b[DWIDTH-1   : 0]);
                    bfly_w_r <= rom_w_real;
                    bfly_w_i <= rom_w_imag;
                end

                S_WRITE: begin
                    if (pair_idx + 1 < span) begin
                        pair_idx <= pair_idx + 1;
                    end else if (({1'b0, group_start} + group_size) < N) begin
                        pair_idx    <= 0;
                        group_start <= {1'b0, group_start} + group_size;
                    end else if (stage_reg < LOG2N - 1) begin
                        stage_reg   <= stage_reg + 1;
                        pair_idx    <= 0;
                        group_start <= 0;
                    end else begin
                        sample_cnt <= 0;
                    end
                end

                S_OUTPUT: begin
                    if (m_axis_tready) begin
                        if (sample_cnt < N - 1) begin
                            sample_cnt <= sample_cnt + 1'b1;
                        end else begin
                            sample_cnt <= 0;
                            done       <= 1'b1;
                        end
                    end
                end

                default: ;
            endcase
        end
    end

    // ══════════════════════════════════════════════════════════════════
    // COMBINATIONAL CONTROL OUTPUTS
    // ══════════════════════════════════════════════════════════════════
    always @(*) begin
        s_axis_tready = 1'b0;
        m_axis_tvalid = 1'b0;
        m_axis_tdata  = {2*DWIDTH{1'b0}};
        m_axis_tlast  = 1'b0;

        ram_addr_a = {LOG2N{1'b0}};
        ram_addr_b = {LOG2N{1'b0}};
        ram_din_a  = {2*DWIDTH{1'b0}};
        ram_din_b  = {2*DWIDTH{1'b0}};
        ram_we_a   = 1'b0;
        ram_we_b   = 1'b0;
        rom_addr   = {(LOG2N-1){1'b0}};

        case (state)
            S_LOAD: begin
                s_axis_tready = 1'b1;
                ram_addr_a    = sample_cnt_bitrev;
                ram_din_a     = s_axis_tdata;
                ram_we_a      = s_axis_tvalid;
            end

            S_READ: begin
                ram_addr_a = addr_A;
                ram_addr_b = addr_B;
                rom_addr   = tw_addr_comb;
            end

            S_LATCH: begin
                // Hold the same addresses for clean timing/debug visibility.
                ram_addr_a = addr_A;
                ram_addr_b = addr_B;
                rom_addr   = tw_addr_comb;
            end

            S_WRITE: begin
                ram_addr_a = addr_A;
                ram_addr_b = addr_B;
                ram_din_a  = scaled_x;
                ram_din_b  = scaled_y;
                ram_we_a   = 1'b1;
                ram_we_b   = 1'b1;
            end

            S_PRE_OUTPUT: begin
                ram_addr_a = {LOG2N{1'b0}};
            end

            S_OUTPUT: begin
                m_axis_tvalid = 1'b1;
                m_axis_tdata  = ram_dout_a;
                m_axis_tlast  = (sample_cnt == N - 1);

                // Current output beat is ram_dout_a; request the next beat only
                // when the current beat is accepted so the stream remains stable
                // under back-pressure.
                if (m_axis_tready && sample_cnt < N - 1)
                    ram_addr_a = sample_cnt + 1'b1;
                else
                    ram_addr_a = sample_cnt;
            end

            default: ;
        endcase
    end

endmodule
