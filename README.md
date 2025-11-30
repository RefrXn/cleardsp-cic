# CIC Decimator & Interpolator

<img src="./assets/cleardsp-banner.png" alt="ClearDSP Banner" width="100%">

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

## Usage

### Decimation

```
cic_decimator #(
    .STAGES           (4),
    .R                (4),
    .M                (1),
    .IN_WIDTH         (16),
    .OUT_WIDTH        (16),
    .USE_SAT          (0),
    .USE_ROUND        (0)
) u_cic_decimator (
    .in_clock         (in_clock),
    .in_reset_n       (in_reset_n),
    .in_valid         (in_valid),
    .in_data          (in_data),
    .in_ready         (in_ready),
    .out_valid        (out_valid),
    .out_data         (out_data),
    .out_ready        (out_ready)
);
```

### Interpolation

```
cic_interpolator #(
    .STAGES           (4),
    .R                (4),
    .M                (1),
    .IN_WIDTH         (16),
    .OUT_WIDTH        (16),
    .USE_SAT          (0),
    .USE_ROUND        (0)
) u_cic_interpolator (
    .in_clock         (in_clock),
    .in_reset_n       (in_reset_n),
    .in_valid         (in_valid),
    .in_data          (in_data),
    .in_ready         (in_ready),
    .out_valid        (out_valid),
    .out_data         (out_data),
    .out_ready        (out_ready)
);
```

## About CIC Filter

A CIC (Cascaded Integrator–Comb) filter is a **multiplier-free multi-rate filter** commonly used for large integer decimation and interpolation factors.  
It is built only from adders, subtractors, and delay elements, which makes it very hardware-friendly.

### Basic Structure

A CIC filter has two main parts:

- **Integrator stages** – cascaded accumulators
- **Comb stages** – cascaded differentiators with delay `M`

For a decimator:

```
Input → Integrator[0..N-1] → ↓R → Comb[0..N-1] → Output
```

For an interpolator:

```
Input → Comb[0..N-1] → ↑R (zero-stuff) → Integrator[0..N-1] → Output
```

Where:

- `N` = `STAGES` = number of integrator/comb stages  
- `R` = decimation/interpolation factor  
- `M` = comb differential delay (usually 1)

### Integrator Section

Each integrator implements:

```
y[n] = y[n-1] + x[n]
```

- In the **decimator**, integrators run at the **high input rate**
- In the **interpolator**, integrators run at the **high output rate**

Because values accumulate, internal bit-width must be extended to avoid overflow.

### Comb Section

Each comb stage implements a simple difference:

```
y[n] = x[n] − x[n-M]
```

- In the **decimator**, combs run at the **low output rate** (after ↓R)
- In the **interpolator**, combs run at the **low input rate** (before ↑R)

The number of comb stages equals the number of integrator stages (`STAGES`).

### Transfer Function

The ideal CIC transfer function is:

```
H(z) = ( (1 − z⁻ᴿᴹ) / (1 − z⁻¹) )ᴺ
```

This gives:

- A `sincᴺ`-shaped magnitude response
- Zeros at multiples of the new sampling rate
- Noticeable passband droop (often corrected with a small FIR compensation filter)

### Bit Growth & Internal Width

The worst-case gain of a CIC filter is:

```
Gain = (R × M)ᴺ
```

This leads to bit growth:

```
growth_bits    = N × log₂(R × M)
internal_width = IN_WIDTH + growth_bits
```

This core computes the growth as:

```
Growth        = STAGES * $clog2(R*M);
InternalWidth = IN_WIDTH + Growth;
```

All integrator/comb arithmetic is performed at this internal width to keep full precision.

### Output Formatting (Rounding & Saturation)

The internal full-precision result is finally mapped to `OUT_WIDTH`:

- **Truncation**: take the most significant `OUT_WIDTH` bits  
- **Rounding (optional)**: add 0.5 LSB before truncation for round-to-nearest  
- **Saturation (optional)**: clamp to the signed `OUT_WIDTH` range to avoid wrap-around

These options are controlled by:

- `USE_ROUND` – enable/disable round-to-nearest before truncation  
- `USE_SAT`   – enable/disable saturation instead of simple wrap

### When to Use CIC Filters

- Large integer **decimation/interpolation factors**
- **Very high throughput** (simple add/sub + registers)
- **Very low resource usage** (no multipliers, no coefficient memory)

Typical applications include:

- Sigma–delta ADC decimation chains
- Digital down/up conversion
- Multi-rate DSP pipelines where a simple, low-cost filter is enough
- PDM to PCM conversion (for audio)