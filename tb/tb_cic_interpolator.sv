// ========================================================================================== //
//                             ___                        ___                                 //
//                            /  /\           ___        /  /\                                //
//                           /  /::\         /__/\      /  /::\                               //
//                          /  /:/\:\        \__\:\    /  /:/\:\                              //
//                         /  /:/  \:\       /  /::\  /  /:/  \:\                             //
//                        /__/:/ \  \:\   __/  /:/\/ /__/:/ \  \:\                            //
//                        \  \:\  \__\/  /__/\/:/~~  \  \:\  \__\/                            //
//                         \  \:\        \  \::/      \  \:\                                  //
//                          \  \:\        \  \:\       \  \:\                                 //
//                           \  \:\        \__\/        \  \:\                                //
//                            \__\/                      \__\/                                //
//                                                                                            //
// ========================================================================================== //

`timescale 1ns/1ps

module tb_cic_interpolator;

    // --------------------------------------------------------------------
    // Parameters (must match DUT)
    // --------------------------------------------------------------------
    localparam int STAGES    = 3;
    localparam int R         = 8;
    localparam int M         = 1;
    localparam int IN_WIDTH  = 16;
    localparam int OUT_WIDTH = 24;
    localparam bit USE_SAT   = 1;
    localparam bit USE_ROUND = 1;

    localparam int Growth    = STAGES * $clog2(R*M);
    localparam int IntWidth  = IN_WIDTH + Growth;
    localparam int CombWidth = IntWidth;
    localparam int FullWidth = CombWidth;

    localparam int RCntWidth = (R <= 1) ? 1 : $clog2(R);

    // --------------------------------------------------------------------
    // DUT I/O
    // --------------------------------------------------------------------
    logic                        in_clock;
    logic                        in_reset_n;

    logic                        in_valid;
    logic                        in_ready;
    logic signed [IN_WIDTH-1:0]  in_data;

    logic                        out_valid;
    logic                        out_ready;
    logic signed [OUT_WIDTH-1:0] out_data;

    // --------------------------------------------------------------------
    // DUT instance
    // --------------------------------------------------------------------
    cic_interpolator #(
        .STAGES         (STAGES),
        .R              (R),
        .M              (M),
        .IN_WIDTH       (IN_WIDTH),
        .OUT_WIDTH      (OUT_WIDTH),
        .USE_SAT        (USE_SAT),
        .USE_ROUND      (USE_ROUND)
    ) dut (
        .in_clock       (in_clock),
        .in_reset_n     (in_reset_n),
        .in_valid       (in_valid),
        .in_ready       (in_ready),
        .in_data        (in_data),
        .out_valid      (out_valid),
        .out_ready      (out_ready),
        .out_data       (out_data)
    );

    // --------------------------------------------------------------------
    // Clock / reset
    // --------------------------------------------------------------------
    initial begin
        in_clock = 1'b0;
        forever #5 in_clock = ~in_clock;
    end

    initial begin
        in_reset_n = 1'b0;
        in_valid   = 1'b0;
        in_data    = '0;
        out_ready  = 1'b0;
        repeat (8) @(posedge in_clock);
        in_reset_n = 1'b1;
    end

    // --------------------------------------------------------------------
    // Reference model state
    // --------------------------------------------------------------------
    // Comb (low-rate)
    logic signed [CombWidth-1:0] comb_delay_mod [STAGES][M];
    logic signed [FullWidth-1:0] comb_out_model;

    // Integrator (high-rate)
    logic signed [IntWidth-1:0]  int_model [STAGES];

    // Interpolation control state
    logic [RCntWidth-1:0]        phase_cnt_model;
    logic                        have_pending_model;

    // Reference output
    logic signed [OUT_WIDTH-1:0] reference_out_data;
    logic                        reference_out_valid;

    // Intermediate model variables
    logic signed [FullWidth-1:0] feed_model;
    logic signed [FullWidth-1:0] tmp_model;

    longint unsigned             sample_cnt;
    longint unsigned             error_cnt;

    // --------------------------------------------------------------------
    // Reference rounding / saturation
    // --------------------------------------------------------------------
    function automatic signed [FullWidth-1:0]
        do_round_model(input logic signed [FullWidth-1:0] in_val);
        if (USE_ROUND && FullWidth > OUT_WIDTH) begin
            const int ROUND_SHIFT = FullWidth - OUT_WIDTH - 1;
            return in_val + signed'(FullWidth'(1) << ROUND_SHIFT);
        end else begin
            return in_val;
        end
    endfunction

    function automatic signed [OUT_WIDTH-1:0]
        do_saturate_model(input logic signed [FullWidth-1:0] in_val);
        if (!USE_SAT) begin
            return in_val[FullWidth-1 -: OUT_WIDTH];
        end else begin
            localparam signed [OUT_WIDTH-1:0] MAXV = {1'b0, {(OUT_WIDTH-1){1'b1}}};
            localparam signed [OUT_WIDTH-1:0] MINV = {1'b1, {(OUT_WIDTH-1){1'b0}}};

            logic signed [FullWidth-1:0] max_ext;
            logic signed [FullWidth-1:0] min_ext;

            max_ext = {{(FullWidth-OUT_WIDTH){MAXV[OUT_WIDTH-1]}}, MAXV};
            min_ext = {{(FullWidth-OUT_WIDTH){MINV[OUT_WIDTH-1]}}, MINV};

            if (in_val > max_ext)
                return MAXV;
            else if (in_val < min_ext)
                return MINV;
            else
                return in_val[FullWidth-1 -: OUT_WIDTH];
        end
    endfunction

    // --------------------------------------------------------------------
    // Reference model main logic
    // --------------------------------------------------------------------
    wire sample_accepted = in_valid && in_ready;
    wire out_step_model  = have_pending_model && (!reference_out_valid || out_ready);

    // Comb chain (low-rate)
    always_ff @(posedge in_clock or negedge in_reset_n) begin
        if (!in_reset_n) begin
            for (int s = 0; s < STAGES; s++)
                for (int d = 0; d < M; d++)
                    comb_delay_mod[s][d] <= '0;
            comb_out_model <= '0;
        end else if (sample_accepted) begin
            logic signed [FullWidth-1:0] stage;
            stage = {{(FullWidth-IN_WIDTH){in_data[IN_WIDTH-1]}}, in_data};

            for (int s = 0; s < STAGES; s++) begin
                logic signed [FullWidth-1:0] old;
                old = comb_delay_mod[s][M-1];

                for (int d = M-1; d > 0; d--)
                    comb_delay_mod[s][d] <= comb_delay_mod[s][d-1];

                comb_delay_mod[s][0] <= stage;
                stage                = stage - old;
            end

            comb_out_model <= stage;   // final comb output for this input sample
        end
    end

    // Integrator chain + output (high-rate)
    always_ff @(posedge in_clock or negedge in_reset_n) begin
        if (!in_reset_n) begin
            for (int i = 0; i < STAGES; i++)
                int_model[i] <= '0;

            reference_out_data      <= '0;
            reference_out_valid     <= 1'b0;

            have_pending_model <= 1'b0;
            phase_cnt_model    <= '0;

            sample_cnt         <= 0;
            error_cnt          <= 0;

        end else begin
            // Manage interpolation block state
            if (sample_accepted) begin
                have_pending_model <= 1'b1;
                phase_cnt_model    <= '0;
            end else if (out_step_model && (phase_cnt_model == R-1)) begin
                have_pending_model <= 1'b0;
            end

            // Reference out_valid behavior
            if (out_step_model) begin
                reference_out_valid <= 1'b1;
            end else if (reference_out_valid && !out_ready) begin
                reference_out_valid <= 1'b1;
            end else begin
                reference_out_valid <= 1'b0;
            end

            // Integrator & output when we emit a new high-rate sample
            if (out_step_model) begin
                // First phase uses comb_out_model, remaining R-1 phases use zero
                if (phase_cnt_model == '0)
                    feed_model = comb_out_model;
                else
                    feed_model = '0;

                // Integrator chain (mirrors DUT)
                int_model[0] <= int_model[0] + feed_model;
                for (int i = 1; i < STAGES; i++)
                    int_model[i] <= int_model[i] + int_model[i-1];

                // Output from last integrator (before update) -> round -> saturate
                tmp_model     = do_round_model(int_model[STAGES-1]);
                reference_out_data <= do_saturate_model(tmp_model);

                // Phase counter within this interpolation block
                if (phase_cnt_model != R-1)
                    phase_cnt_model <= phase_cnt_model + 1'b1;
                else
                    phase_cnt_model <= '0;
            end

            // Scoreboard: compare Reference vs DUT when both valid
            if (reference_out_valid && out_valid) begin
                sample_cnt <= sample_cnt + 1;
                if (reference_out_data !== out_data) begin
                    error_cnt <= error_cnt + 1;
                    $display("[%0t] ERROR: out_data mismatch. reference=%0d, dut=%0d",
                             $time, reference_out_data, out_data);
                end
            end
        end
    end

    // --------------------------------------------------------------------
    // Random back-pressure on out_ready
    // --------------------------------------------------------------------
    always @(posedge in_clock or negedge in_reset_n) begin
        if (!in_reset_n) begin
            out_ready <= 1'b0;
        end else begin
            // About 75% chance of ready = 1
            out_ready <= ($urandom_range(0,3) != 0);
        end
    end

    // --------------------------------------------------------------------
    // Driver task (respects ready/valid)
    // --------------------------------------------------------------------
    task automatic send_sample(input logic signed [IN_WIDTH-1:0] val);
        bit sent;
        sent = 0;
        while (!sent) begin
            @(posedge in_clock);
            if (!in_reset_n) begin
                in_valid <= 1'b0;
            end else if (in_ready) begin
                in_valid <= 1'b1;
                in_data  <= val;
                sent     = 1;   // blocking assignment
            end else begin
                in_valid <= 1'b0;
            end
        end
        @(posedge in_clock);
        in_valid <= 1'b0;
    endtask

    // --------------------------------------------------------------------
    // Stimulus: impulse / step / random
    // --------------------------------------------------------------------
    task automatic run_impulse_test();
        $display("[%0t] ==== INTERPOLATOR: Impulse test ====", $time);
        send_sample( 16'sd1 );
        // Send zeros to see the full impulse response
        for (int i = 0; i < 64; i++)
            send_sample( 16'sd0 );
    endtask

    task automatic run_step_test();
        $display("[%0t] ==== INTERPOLATOR: Step test ====", $time);
        for (int i = 0; i < 128; i++)
            send_sample( 16'sd500 );
    endtask

    task automatic run_random_test();
        $display("[%0t] ==== INTERPOLATOR: Random test ====", $time);
        for (int i = 0; i < 512; i++)
            send_sample( $urandom_range(-(1<<(IN_WIDTH-1)), (1<<(IN_WIDTH-1))-1) );
    endtask

    // --------------------------------------------------------------------
    // Main sequence
    // --------------------------------------------------------------------
    initial begin
        @(posedge in_reset_n);
        repeat (5) @(posedge in_clock);

        run_impulse_test();
        run_step_test();
        run_random_test();

        // Let remaining outputs flush
        repeat (R * 32) @(posedge in_clock);

        $display("==================================================");
        $display("CIC INTERPOLATOR TB DONE. samples checked = %0d, errors = %0d",
                 sample_cnt, error_cnt);
        if (error_cnt == 0)
            $display("STATUS: PASS");
        else
            $display("STATUS: FAIL");
        $display("==================================================");
        $finish;
    end

endmodule
