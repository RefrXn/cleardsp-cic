# CIC Decimator & Interpolator (SystemVerilog)

![ClearDSP Banner]([./assets/cleardsp-banner.png](https://github.com/RefrXn/cleardsp-cic/blob/main/assets/cleardsp-banner.png))

This repo contains a pair of **streaming CIC filters**:

- `cic_decimator.sv`          – CIC decimation filter with ready/valid interface  
- `cic_interpolator.sv`       – CIC interpolation filter with ready/valid interface  

Both cores are fully parameterized and use internal full-precision arithmetic with optional rounding and saturation.

## Features

- Parameterized:
  - `STAGES`                  – number of cascaded integrator/comb stages
  - `R`                       – decimation / interpolation factor (R ≥ 1)
  - `M`                       – comb differential delay (1 or 2)
  - `IN_WIDTH` / `OUT_WIDTH`  – signed fixed-point data widths
  - `USE_SAT`                 – optional output saturation
  - `USE_ROUND`               – optional round-to-nearest before truncation
- Streaming handshake:
  - Simple ready/valid protocol (`in_valid/in_ready`, `out_valid/out_ready`)
  - Full back-pressure support
- Internal bit-growth:
  - Bit growth = `STAGES * $clog2(R*M)`
  - Internal width = `IN_WIDTH + growth`

## Files

clear-dsp/
  ├─ rtl/
  │  ├─ cic_decimator.sv
  │  └─ cic_interpolator.sv
  ├─ tb/
  │  ├─ tb_cic_decimator.sv
  │  └─ tb_cic_interpolator.sv
  └─ README.md
