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
// CIC DECIMATOR with ready/valid streaming interface                                         //
//  - Integrator stages optionally mapped to DSP48                                            //
//  - Fully parameterized (R, M, STAGES, I/O width)                                           //
//  - Internal full-precision arithmetic                                                      //
//  - Optional rounding and saturation                                                        //
//                                                                                            //
//  IMPORTANT USAGE NOTE:                                                                     //
//    This module assumes the upstream interface (in_valid/in_ready) supports back-pressure.  //
//    If the source cannot be stalled (e.g. real-time ADC, SERDES, LVDS sampler),             //
//    an input FIFO MUST be inserted before this block.                                       //
//                                                                                            //
//  DESIGN QUALITY NOTES:                                                                     //
//    • Supports arbitrary STAGES ≥ 1                                                         //
//    • Supports arbitrary R ≥ 1 (R=1 degenerates to non-decimating CIC)                      //
//    • M is restricted to 1 or 2 (larger M increases register footprint significantly)       //
//    • All parameters validated at elaboration                                               //
//    • All internal register widths derived analytically                                     //
//    • No vendor-specific primitives used                                                    //
// ========================================================================================== //

module cic_decimator #(
    parameter int STAGES    = 3,                            // Stages of cascaded integrator/comb
    parameter int R         = 8,                            // Decimation factor (R >= 1)
    parameter int M         = 1,                            // Comb differential delay, 1 or 2
    parameter int IN_WIDTH  = 16,                           // Input sample width (signed)
    parameter int OUT_WIDTH = 24,                           // Output sample width (signed)
    parameter bit USE_SAT   = 1,                            // Enable saturation (1=enabled)
    parameter bit USE_ROUND = 1                             // Enable rounding (1=enabled)
)(
    input  logic                        in_clock,           // System clock
    input  logic                        in_reset_n,         // Asynchronous reset, active low

    // Input streaming interface
    input  logic                        in_valid,           // Input sample valid
    output logic                        in_ready,           // CIC can accept new sample
    input  logic signed [IN_WIDTH-1:0]  in_data,            // Input sample

    // Output streaming interface
    output logic                        out_valid,          // Decimated output valid
    input  logic                        out_ready,          // Downstream ready
    output logic signed [OUT_WIDTH-1:0] out_data            // Output sample
);

    // ----------------- Derived Internal Bit Widths -----------------
    localparam int Growth    = STAGES * $clog2(R*M);        // CIC theoretical bit growth
    localparam int IntWidth  = IN_WIDTH + Growth;           // Integrator accumulator width
    localparam int CombWidth = IntWidth;                    // Comb datapath width
    localparam int FullWidth = CombWidth;                   // Internal full precision width

    // ----------------- Parameter Validation -----------------
    generate
        // STAGES must be at least 1
        if (STAGES < 1) begin : gen_check_stages
            initial $error("Parameter Error: STAGES (%0d) must be >= 1.", STAGES);
        end

        // R must be >= 1 (R=1 is allowed)
        if (R < 1) begin : gen_check_r
            initial $error("Parameter Error: R (%0d) must be >= 1.", R);
        end

        // M must be 1 or 2
        if (!(M == 1 || M == 2)) begin : gen_check_m
            initial $error("Parameter Error: M (%0d) must be 1 or 2.", M);
        end

        // Widths must be positive
        if (IN_WIDTH < 1) begin : gen_check_in_width
            initial $error("Parameter Error: IN_WIDTH (%0d) must be >= 1.", IN_WIDTH);
        end

        if (OUT_WIDTH < 2) begin : gen_check_out_width
            initial $error("Parameter Error: OUT_WIDTH (%0d) must be >= 2.", OUT_WIDTH);
        end

        // Output width cannot exceed internal precision
        if (OUT_WIDTH > FullWidth) begin : gen_check_FullWidth
            initial $error("OUT_WIDTH (%0d) cannot exceed FullWidth (%0d).",
                            OUT_WIDTH, FullWidth);
        end
    endgenerate

    // ----------------- Handshake Logic -----------------
    // Back-pressure passes upstream: stall integrators when output stalls.
    assign in_ready = !(out_valid && !out_ready);
    wire sample_accepted = in_valid && in_ready;

    // ==========================================================================================================================
    // 1. Integrator Chain (High-Rate Domain) — DSP-Friendly Accumulators
    // ==========================================================================================================================
    // Array has STAGES elements: integrator[0] .. integrator[STAGES-1]
    (* use_dsp = "yes" *)
    logic signed [IntWidth-1:0] integrator [STAGES];

    always_ff @(posedge in_clock or negedge in_reset_n) begin : integrator_chain
        if (!in_reset_n) begin
            for (int i = 0; i < STAGES; i++)
                integrator[i] <= '0;
        end else if (sample_accepted) begin
            // First stage accumulates sign-extended input
            integrator[0] <= integrator[0] +
                {{(IntWidth-IN_WIDTH){in_data[IN_WIDTH-1]}}, in_data};

            // Remaining stages accumulate outputs of previous stage
            for (int i = 1; i < STAGES; i++)
                integrator[i] <= integrator[i] + integrator[i-1];
        end
    end

    // ==========================================================================================================================
    // 2. Decimation Counter (Ensures 1-out-of-R Samples Enter Comb)
    // ==========================================================================================================================
    // Handle R=1 case: counter width minimum is 1 bit.
    localparam int RCntWidth = (R <= 1) ? 1 : $clog2(R);
    logic [RCntWidth-1:0]   r_cnt;
    logic                   dec_fire;   // Asserted exactly when a decimated sample is available

    always_ff @(posedge in_clock or negedge in_reset_n) begin : decimation_counter
        if (!in_reset_n) begin
            r_cnt    <= '0;
            dec_fire <= 1'b0;
        end else if (sample_accepted) begin
            if (r_cnt == R-1) begin
                r_cnt    <= '0;
                dec_fire <= 1'b1;      // Trigger comb processing for decimated sample
            end else begin
                r_cnt    <= r_cnt + 1'b1;
                dec_fire <= 1'b0;
            end
        end else begin
            // No new sample accepted, so decimation event cannot occur
            dec_fire <= 1'b0;
        end
    end

    // ==========================================================================================================================
    // 3. Comb Chain (Low-Rate Domain) — Differential Delay Stages
    // ==========================================================================================================================
    // comb_delay[s][d]:
    //   s = 0 .. STAGES-1: comb stage index
    //   d = 0 .. M-1      : differential delay index for each stage
    logic signed [CombWidth-1:0] comb_delay   [STAGES][M];
    logic signed [OUT_WIDTH-1:0] out_data_reg;

    // -----------------------------------
    // Rounding Helper Function
    // -----------------------------------
    function automatic signed [FullWidth-1:0]
        do_round(input logic signed [FullWidth-1:0] in_val);
        if (USE_ROUND && FullWidth > OUT_WIDTH) begin
            // Add 0.5 LSB (of the target OUT_WIDTH) before truncation for round-to-nearest
            const int ROUND_SHIFT = FullWidth - OUT_WIDTH - 1;
            return in_val + signed'(FullWidth'(1) << ROUND_SHIFT);
        end else begin
            return in_val;
        end
    endfunction

    // -----------------------------------
    // Saturation Helper Function
    // -----------------------------------
    function automatic signed [OUT_WIDTH-1:0]
        do_saturate(input logic signed [FullWidth-1:0] in_val);
        if (!USE_SAT) begin
            // Simple truncation (MSB-aligned)
            return in_val[FullWidth-1 -: OUT_WIDTH];
        end else begin
            localparam signed [OUT_WIDTH-1:0] MAXV = {1'b0, {(OUT_WIDTH-1){1'b1}}};
            localparam signed [OUT_WIDTH-1:0] MINV = {1'b1, {(OUT_WIDTH-1){1'b0}}};

            // Extend bounds to internal precision for comparison
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

    // -----------------------------------
    // Comb Processing and Output Control
    // -----------------------------------
    always_ff @(posedge in_clock or negedge in_reset_n) begin : comb_chain
        if (!in_reset_n) begin
            // Clear all comb delay registers and output state
            for (int s = 0; s < STAGES; s++)
                for (int d = 0; d < M; d++)
                    comb_delay[s][d] <= '0;

            out_data_reg <= '0;
            out_valid    <= 1'b0;

        end else begin
            // out_valid control:
            //  - Assert when a new decimated sample is produced (dec_fire)
            //  - Hold high if downstream is not ready
            //  - Deassert when sample has been accepted
            if (dec_fire) begin
                out_valid <= 1'b1;
            end else if (out_valid && !out_ready) begin
                out_valid <= 1'b1;
            end else begin
                out_valid <= 1'b0;
            end

            // Comb chain runs only when a new decimated input is available
            if (dec_fire) begin
                // 'stage' carries the signal through the cascaded comb stages:
                //  stage_0 input  = integrator[STAGES-1] (decimated integrator output)
                //  stage_s input  = output of stage_(s-1)
                //  stage_s output = stage_s_input - stage_s_input delayed by M samples
                logic signed [FullWidth-1:0] stage;
                stage = integrator[STAGES-1];

                for (int s = 0; s < STAGES; s++) begin
                    // Old value at the end of this stage's delay line = M-sample-old input
                    logic signed [FullWidth-1:0] old;
                    old = comb_delay[s][M-1];

                    // Shift this stage's delay line towards higher indices
                    for (int d = M-1; d > 0; d--)
                        comb_delay[s][d] <= comb_delay[s][d-1];

                    // Current input to this stage is stored at delay[0]
                    comb_delay[s][0] <= stage;

                    // Comb difference: current stage output
                    stage = stage - old;
                end

                // Final 'stage' value is the output of the last comb stage
                out_data_reg <= do_saturate(do_round(stage));
            end
        end
    end

    assign out_data = out_data_reg;

endmodule
