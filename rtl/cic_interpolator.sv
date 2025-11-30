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
// CIC INTERPOLATOR with ready/valid streaming interface                                      //
//  - Comb stages at low-rate input side                                                      //
//  - Integrator stages at high-rate output side                                              //
//  - Fully parameterized (R, M, STAGES, I/O width)                                           //
//  - Internal full-precision arithmetic                                                      //
//  - Optional rounding and saturation                                                        //
//                                                                                            //
//  IMPORTANT USAGE NOTE:                                                                     //
//    This module assumes the downstream interface (out_valid/out_ready) supports             //
//    back-pressure. If the sink cannot be stalled (e.g. real-time DAC),                      //
//    an output FIFO MUST be inserted after this block.                                       //
//                                                                                            //
//  DESIGN QUALITY NOTES:                                                                     //
//    • Parameter set mirrors cic_decimator (STAGES/R/M/width/round/sat)                      //
//    • Supports arbitrary STAGES ≥ 1                                                         //
//    • Supports arbitrary R ≥ 1 (R=1 degenerates to non-interpolating CIC)                   //
//    • M is restricted to 1 or 2                                                             //
//    • All parameters validated at elaboration                                               //
//    • All internal register widths derived analytically                                     //
//    • No vendor-specific primitives used                                                    //
// ========================================================================================== //

module cic_interpolator #(
    parameter int STAGES    = 3,                            // Stages of cascaded comb/integrator
    parameter int R         = 8,                            // Interpolation factor (R >= 1)
    parameter int M         = 1,                            // Comb differential delay, 1 or 2
    parameter int IN_WIDTH  = 16,                           // Input sample width (signed)
    parameter int OUT_WIDTH = 24,                           // Output sample width (signed)
    parameter bit USE_SAT   = 1,                            // Enable saturation (1=enabled)
    parameter bit USE_ROUND = 1                             // Enable rounding (1=enabled)
)(
    input  logic                        in_clock,           // System clock
    input  logic                        in_reset_n,         // Asynchronous reset, active low

    // Input streaming interface (low-rate)
    input  logic                        in_valid,           // Input sample valid
    output logic                        in_ready,           // Interpolator can accept new sample
    input  logic signed [IN_WIDTH-1:0]  in_data,            // Input sample

    // Output streaming interface (high-rate, xR)
    output logic                        out_valid,          // Interpolated output valid
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
        if (STAGES < 1) begin : gen_check_stages
            initial $error("Parameter Error: STAGES (%0d) must be >= 1.", STAGES);
        end

        if (R < 1) begin : gen_check_r
            initial $error("Parameter Error: R (%0d) must be >= 1.", R);
        end

        if (!(M == 1 || M == 2)) begin : gen_check_m
            initial $error("Parameter Error: M (%0d) must be 1 or 2.", M);
        end

        if (IN_WIDTH < 1) begin : gen_check_in_width
            initial $error("Parameter Error: IN_WIDTH (%0d) must be >= 1.", IN_WIDTH);
        end

        if (OUT_WIDTH < 2) begin : gen_check_out_width
            initial $error("Parameter Error: OUT_WIDTH (%0d) must be >= 2.", OUT_WIDTH);
        end

        if (OUT_WIDTH > FullWidth) begin : gen_check_FullWidth
            initial $error("OUT_WIDTH (%0d) cannot exceed FullWidth (%0d).",
                           OUT_WIDTH, FullWidth);
        end
    endgenerate

    // ----------------- Handshake / Interpolation State -----------------
    // For each accepted input sample, we will output R high-rate samples.
    // We do NOT accept a new sample while we are still expanding the previous one,
    // and we also propagate back-pressure from downstream.
    localparam int RCntWidth = (R <= 1) ? 1 : $clog2(R);

    logic [RCntWidth-1:0] phase_cnt;    // 0 .. R-1 within one interpolation block
    logic                 have_pending; // Currently expanding an input sample

    wire sample_accepted = in_valid && in_ready;

    // Back-pressure:
    //  - If there is a pending interpolation block (have_pending=1), we block new inputs
    //  - If output is stalled (out_valid && !out_ready), we also block new inputs
    assign in_ready = !have_pending && !(out_valid && !out_ready);

    // When can we advance one high-rate output step?
    // Exactly when:
    //   - we have a pending input sample to expand
    //   - and either there is no outstanding valid output, or the previous one is accepted
    wire out_step = have_pending && (!out_valid || out_ready);

    // ==========================================================================================================================
    // 1. Comb Chain (Low-Rate Domain) — Differential Delay Stages
    //    Runs once per accepted input sample, produces one "baseband" value for interpolation.
    // ==========================================================================================================================

    logic signed [CombWidth-1:0] comb_delay [STAGES][M];
    logic signed [FullWidth-1:0] comb_out_reg;

    always_ff @(posedge in_clock or negedge in_reset_n) begin : comb_chain
        if (!in_reset_n) begin
            for (int s = 0; s < STAGES; s++)
                for (int d = 0; d < M; d++)
                    comb_delay[s][d] <= '0;

            comb_out_reg <= '0;

        end else if (sample_accepted) begin
            // Sign-extend input to internal width
            logic signed [FullWidth-1:0] stage;
            stage = {{(FullWidth-IN_WIDTH){in_data[IN_WIDTH-1]}}, in_data};

            for (int s = 0; s < STAGES; s++) begin
                logic signed [FullWidth-1:0] old;
                old = comb_delay[s][M-1];

                // Shift delay line
                for (int d = M-1; d > 0; d--)
                    comb_delay[s][d] <= comb_delay[s][d-1];

                // Current input stored at delay[0]
                comb_delay[s][0] <= stage;

                // Comb difference: current stage output
                stage = stage - old;
            end

            // Final comb output, used as first high-rate input sample of this block
            comb_out_reg <= stage;
        end
    end

    // ==========================================================================================================================
    // 2. Integrator Chain (High-Rate Domain) — DSP-Friendly Accumulators
    //    For each input sample, outputs R samples:
    //      phase_cnt == 0    : integrator fed with comb_out_reg
    //      phase_cnt == 1..R-1 : integrator fed with 0 (zero-stuffing)
    // ==========================================================================================================================

    (* use_dsp = "yes" *)
    logic signed [IntWidth-1:0] integrator [STAGES];
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
    // Integrator Processing and Output Control
    // -----------------------------------
    always_ff @(posedge in_clock or negedge in_reset_n) begin : integrator_and_output
        if (!in_reset_n) begin
            for (int i = 0; i < STAGES; i++)
                integrator[i] <= '0;

            out_data_reg <= '0;
            out_valid    <= 1'b0;

            have_pending <= 1'b0;
            phase_cnt    <= '0;

        end else begin
            // out_valid control (mirrors cic_decimator pattern, but driven by out_step)
            if (out_step) begin
                out_valid <= 1'b1;
            end else if (out_valid && !out_ready) begin
                out_valid <= 1'b1;
            end else begin
                out_valid <= 1'b0;
            end

            // Manage interpolation block state
            if (sample_accepted) begin
                // Start a new interpolation block for this input sample
                have_pending <= 1'b1;
                phase_cnt    <= '0;
            end else if (out_step && (phase_cnt == R-1)) begin
                // Finished last (R-1)th output sample of this block
                have_pending <= 1'b0;
            end

            // Integrator chain runs only when producing a new output sample
            if (out_step) begin
                logic signed [FullWidth-1:0] feed;

                // First high-rate sample uses comb_out_reg; remaining R-1 samples use zero
                if (phase_cnt == '0)
                    feed = comb_out_reg;
                else
                    feed = '0;

                // First stage accumulates feed
                integrator[0] <= integrator[0] + feed;

                // Remaining stages accumulate outputs of previous stage (from previous cycle)
                for (int i = 1; i < STAGES; i++)
                    integrator[i] <= integrator[i] + integrator[i-1];

                // Final integrator output -> rounding -> saturation -> OUT_WIDTH
                out_data_reg <= do_saturate(do_round(integrator[STAGES-1]));

                // Advance phase counter
                if (phase_cnt != R-1)
                    phase_cnt <= phase_cnt + 1'b1;
                else
                    phase_cnt <= '0;   // don't care; will be cleared by have_pending
            end
        end
    end

    assign out_data = out_data_reg;

endmodule
