# CMSIS-STFT Library
CMSIS-STFT is a lightweight, memory-efficient Short-Time Fourier Transform (STFT) library optimized for ARM Cortex-M microcontrollers using the CMSIS-DSP (arm_math.h) framework.

## Getting Started & Resources

**See Implementation Examples**: Check out the [cmsis-stft-examples](https://github.com/cmsis-stft/cmsis-stft_examples) repository for complete STM32F407 example firmware and STM32CubeIDE projects.

**Visualize Real-Time Data**: Use the [pc_datalogger](https://github.com/cmsis-stft/pc_datalogger) desktop application to receive, plot, and log your STFT output over USB.

**Verify and Reproduce Results**: Reference the [cmsis-stft_matlab](https://github.com/cmsis-stft/cmsis-stft_matlab) repository for test-signal generators and verification scripts matching MATLAB's spectrogram function.

| File | Purpose |
|---|---|
| [inc/cmsis_stft.h](https://github.com/cmsis-stft/cmsis_stft/blob/main/inc/cmsis_stft.h) | Public API, types and sizing macros |
| [inc/stft_port.h](https://github.com/cmsis-stft/cmsis_stft/blob/main/inc/stft_port.h) | Core and FPU detection, alignment macro, FFT length limits (32–4096) |
| [src/cmsis_stft.c](https://github.com/cmsis-stft/cmsis_stft/blob/main/src/cmsis_stft.c) | Implementation |

**Features**
- F32, Q15 and Q31 processing paths.
- Windows: Hanning, Hamming, Blackman, Bartlett, none.
- Streaming: the window overlaps correctly across consecutive input buffers.
- The caller supplies one static memory arena; the library uses no `malloc`.

**Dependency:** CMSIS-DSP (`arm_math.h`).

## Usage (F32; Q15/Q31 are the same with `_q15` / `_q31`)

```c
#define HOP_LEN STFT_HOP_LEN(WIN_LEN, OVERLAP_PERCENT)
static float32_t arena[STFT_ARENA_SIZE_F32(BUF_LEN, WIN_LEN, FFT_LEN, HOP_LEN)];
static stft_instance_f32 stft;

void on_stft(float32_t *mag, uint32_t slices) { /* slices x (FFT_LEN/2 + 1) magnitudes */ }

stft_instance_init_f32(&stft, BUF_LEN, WIN_LEN, FFT_LEN, OVERLAP_PERCENT, HANNING,
                       arena, STFT_ARENA_SIZE_F32(BUF_LEN, WIN_LEN, FFT_LEN, HOP_LEN), 0);
stft_f32_set_callback(&stft, on_stft);

stft_adc_to_f32(adc_samples, input, BUF_LEN, ADC_OFFSET, ADC_BITS);  /* optional ADC conversion */
stft_f32(&stft, input);                                              /* call once per buffer */
```

