// ========================================================================================== //
//                             ___                        ___                                 //
//                            /  /\           ___        /  /\                                //
//                           /  /::\         /__/\      /  /::\                               //
//                          /  /:/\:\        \__\:\    /  /:/\:\                              //
//                         /  /:/  \:\       /  /::\  /  /:/  \:\                             //
//                        /__/:/ \  \:\   __/  /:/\/ /__/:/ \  \:\                            //
//                        \  \:\  \__\/  /__/\/:/    \  \:\  \__\/                            //
//                         \  \:\        \  \::/      \  \:\                                  //
//                          \  \:\        \  \:\       \  \:\                                 //
//                           \  \:\        \__\/        \  \:\                                //
//                            \__\/                      \__\/                                //
//                                                                                            //
// ========================================================================================== //

`timescale 1ns/1ps

module tb_cic_decimator;

    // ----------------------------------------------------------------
    // Parameters
    // ----------------------------------------------------------------
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

    // ----------------------------------------------------------------
    // DUT I/O
    // ----------------------------------------------------------------
    logic                        in_clock;
    logic                        in_reset_n;

    logic                        in_valid;
    logic                        in_ready;
    logic signed [IN_WIDTH-1:0]  in_data;

    logic                        out_valid;
    logic                        out_ready;
    logic signed [OUT_WIDTH-1:0] out_data;

    // ----------------------------------------------------------------
    // DUT Instance
    // ----------------------------------------------------------------
    cic_decimator #(
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

    // ----------------------------------------------------------------
    // Clock / Reset
    // ----------------------------------------------------------------
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

    // ----------------------------------------------------------------
    // Reference model state
    // ----------------------------------------------------------------
    logic signed [IntWidth-1:0]  int_model [STAGES];
    logic signed [CombWidth-1:0] comb_delay_model [STAGES][M];

    localparam int RCntWidth = (R <= 1) ? 1 : $clog2(R);
    logic [RCntWidth-1:0]        r_cnt_model;
    logic                        dec_fire_model;

    logic signed [OUT_WIDTH-1:0] reference_out_data;
    logic                        reference_out_valid;

    longint unsigned             sample_cnt;
    longint unsigned             error_cnt;

    // ----------------------------------------------------------------
    // Reference model rounding / saturation
    // ----------------------------------------------------------------
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

            logic signed [FullWidth-1:0] max_ext =
                {{(FullWidth-OUT_WIDTH){MAXV[OUT_WIDTH-1]}}, MAXV};
            logic signed [FullWidth-1:0] min_ext =
                {{(FullWidth-OUT_WIDTH){MINV[OUT_WIDTH-1]}}, MINV};

            if (in_val > max_ext)
                return MAXV;
            else if (in_val < min_ext)
                return MINV;
            else
                return in_val[FullWidth-1 -: OUT_WIDTH];
        end
    endfunction

    // ----------------------------------------------------------------
    // Reference model main logic
    // ----------------------------------------------------------------
    wire sample_accepted = in_valid && in_ready;

    always_ff @(posedge in_clock or negedge in_reset_n) begin
        if (!in_reset_n) begin
            for (int i = 0; i < STAGES; i++)
                int_model[i] <= '0;

            for (int s = 0; s < STAGES; s++)
                for (int d = 0; d < M; d++)
                    comb_delay_model[s][d] <= '0;

            r_cnt_model    <= '0;
            dec_fire_model <= 1'b0;

            reference_out_data  <= '0;
            reference_out_valid <= 1'b0;

            sample_cnt     <= 0;
            error_cnt      <= 0;
        end else begin
            // Integrator chain
            if (sample_accepted) begin
                logic signed [IntWidth-1:0] in_ext;
                in_ext = {{(IntWidth-IN_WIDTH){in_data[IN_WIDTH-1]}}, in_data};

                int_model[0] <= int_model[0] + in_ext;
                for (int i = 1; i < STAGES; i++)
                    int_model[i] <= int_model[i] + int_model[i-1];
            end

            // Decimation counter
            if (sample_accepted) begin
                if (r_cnt_model == R-1) begin
                    r_cnt_model    <= '0;
                    dec_fire_model <= 1'b1;
                end else begin
                    r_cnt_model    <= r_cnt_model + 1'b1;
                    dec_fire_model <= 1'b0;
                end
            end else begin
                dec_fire_model <= 1'b0;
            end

            // out_valid behavior
            if (dec_fire_model) begin
                reference_out_valid <= 1'b1;
            end else if (reference_out_valid && !out_ready) begin
                reference_out_valid <= 1'b1;
            end else begin
                reference_out_valid <= 1'b0;
            end

            // Comb chain + output
            if (dec_fire_model) begin
                logic signed [FullWidth-1:0] stage;
                stage = int_model[STAGES-1];

                for (int s = 0; s < STAGES; s++) begin
                    logic signed [FullWidth-1:0] old;
                    old = comb_delay_model[s][M-1];

                    for (int d = M-1; d > 0; d--)
                        comb_delay_model[s][d] <= comb_delay_model[s][d-1];

                    comb_delay_model[s][0] <= stage;
                    stage = stage - old;
                end

                stage          = do_round_model(stage);
                reference_out_data <= do_saturate_model(stage);
            end

            // Scoreboard
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

    // ----------------------------------------------------------------
    // Random back-pressure: use always here, not always_ff
    // ----------------------------------------------------------------
    always @(posedge in_clock or negedge in_reset_n) begin
        if (!in_reset_n) begin
            out_ready <= 1'b0;
        end else begin
            // Approximately 75% chance ready=1
            out_ready <= ($urandom_range(0,3) != 0);
        end
    end

    // ----------------------------------------------------------------
    // Driver task: automatic variable uses blocking assignment
    // ----------------------------------------------------------------
    task automatic send_sample(input logic signed [IN_WIDTH-1:0] val);
        bit sent;
        sent = 0;
        while (!sent) begin
            @(posedge in_clock);
            if (!in_reset_n) begin
                in_valid <= 1'b0;
                continue;
            end
            if (in_ready) begin
                in_valid <= 1'b1;
                in_data  <= val;
                sent     = 1;     // ★ Use blocking assignment
            end else begin
                in_valid <= 1'b0;
            end
        end
        @(posedge in_clock);
        in_valid <= 1'b0;
    endtask

    // ----------------------------------------------------------------
    // Stimulus
    // ----------------------------------------------------------------
    task automatic run_impulse_test();
        $display("[%0t] ==== Impulse test ====", $time);
        send_sample( 16'sd1 );
        for (int i = 0; i < R * STAGES * 8; i++)
            send_sample( 16'sd0 );
    endtask

    task automatic run_step_test();
        $display("[%0t] ==== Step test ====", $time);
        for (int i = 0; i < 256; i++)
            send_sample( 16'sd1000 );
    endtask

    task automatic run_random_test();
        $display("[%0t] ==== Random test ====", $time);
        for (int i = 0; i < 2000; i++)
            send_sample( $urandom_range(-(1<<(IN_WIDTH-1)), (1<<(IN_WIDTH-1))-1) );
    endtask

    // ----------------------------------------------------------------
    // Main sequence
    // ----------------------------------------------------------------
    initial begin
        @(posedge in_reset_n);
        repeat (5) @(posedge in_clock);

        run_impulse_test();
        run_step_test();
        run_random_test();

        repeat (R * STAGES * 8) @(posedge in_clock);

        $display("==================================================");
        $display("CIC DECIMATOR TB DONE. samples checked = %0d, errors = %0d",
                 sample_cnt, error_cnt);
        if (error_cnt == 0)
            $display("STATUS: PASS");
        else
            $display("STATUS: FAIL");
        $display("==================================================");
        $finish;
    end

endmodule
