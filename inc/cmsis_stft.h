/* =========================================================================
   cmsis_stft.h - Macros, Enums, and Structs
   ========================================================================= */

#ifndef CUSTOM_STFT_H //Header Guard to see if the header file has been called already
#define CUSTOM_STFT_H

#include <stdlib.h>

#ifndef PI
    #define PI 3.14159265358979323846f // The standard value for floating point calculations
#endif

/**
 * @brief Calculates the hop length in samples based on window length and overlap percentage.
 * @param win_len Length of the window in samples.
 * @param overlap_pct Overlap percentage (0 to 99).
 * @return Hop length in samples.
 */
#define STFT_HOP_LEN(win_len, overlap_pct) \
    ((uint32_t)((((win_len) * (uint32_t)(100U - (overlap_pct)) + 99U)) / 100U))

/**
 * @brief Calculates the maximum number of FFT slices generated from a single buffer.
 * @param buf_len Total number of samples in the input buffer.
 * @param hop_len Hop length in samples.
 * @return Maximum number of slices.
 */    
#define STFT_MAX_SLICES(buf_len, hop_len) \
    (((buf_len) / hop_len + 1U))


/**
 * @brief Calculates the total number of float32_t elements required for the F32 memory arena.
 * @param buf_len Total number of samples in the input buffer.
 * @param win_len Length of the window in samples.
 * @param fft_len Size of the FFT (must be a power of 2).
 * @param hop_len Hop length in samples.
 * @return Total number of elements to allocate.
 */    
/* Total float32_t elements required for the static arena */
#define STFT_ARENA_SIZE_F32(buf_len, win_len, fft_len, hop_len) ( \
    (win_len) +                                                       /* 1. history_buffer */ \
    (win_len) +                                                       /* 2. window_coefficients */ \
    ((fft_len) * 2U) +                                                /* 3. fft_input + fft_output */ \
    (STFT_MAX_SLICES(buf_len, hop_len) * (1+((fft_len) / 2U))) /* 4. output_mag_array */ \
)

#define STFT_ARENA_SIZE_Q31(buf_len, win_len, fft_len, hop_len) ( \
    (win_len) +                                                       /* 1. history_buffer */ \
    (win_len) +                                                       /* 2. window_coefficients */ \
    ((fft_len) * 3U) +                                                /* 3. fft_input + fft_output */ \
    (STFT_MAX_SLICES(buf_len, hop_len) * (1+((fft_len) / 2U))) /* 4. output_mag_array */ \
)

#define STFT_ARENA_SIZE_Q15(buf_len, win_len, fft_len, hop_len) ( \
    (win_len) +                                                       /* 1. history_buffer */ \
    (win_len) +                                                       /* 2. window_coefficients */ \
    ((fft_len) * 3U) +                                                /* 3. fft_input + fft_output */ \
    (STFT_MAX_SLICES(buf_len, hop_len) * (1+((fft_len) / 2U))) /* 4. output_mag_array */ \
)


#include "stft_port.h" // Configuration file that defines the board architecture and global definitions
#include "arm_math.h"

/**
 * @brief Windowing function types supported by the STFT library.
 */
//
typedef enum{
    HAMMING,
    HANNING,
    BLACKMAN,
    BARTLETT,
    NO_WINDOW
}WINDOW_TYPE;

/**
 * @brief Return status codes for STFT library operations.
 */
//
typedef enum {
    STFT_OK              = 0,
    STFT_WARNING_OVERLAP = 1,
    STFT_ERROR_NULL_PTR  = 2,
    STFT_ERROR_FFT_SIZE  = 3,
    STFT_ERROR_OVERLAP   = 4,
    STFT_ERROR_BUSY      = 5,
	STFT_ERROR			 = 6,
	STFT_ERROR_ALLOC	 = 7
} stft_status;
// The STFT status acts similar to CMSIS return status to indicate the status of the executed function

// The user callback is a function that is assigned by the user so that they can use the STFT output
/**
 * @brief Callback function pointer type for F32 STFT output.
 * @param stft_output Pointer to the computed F32 magnitude array.
 * @param slices Number of valid slices processed in this callback.
 */
typedef void (*stft_callback_f32)(float32_t *stft_output, uint32_t slices);

/**
 * @brief Callback function pointer type for Q15 STFT output.
 * @param stft_output Pointer to the computed Q15 magnitude array.
 * @param slices Number of valid slices processed in this callback.
 */
typedef void (*stft_callback_q15)(q15_t *stft_output, uint16_t slices);
/**
 * @brief Callback function pointer type for Q31 STFT output.
 * @param stft_output Pointer to the computed Q31 magnitude array.
 * @param slices Number of valid slices processed in this callback.
 */
typedef void (*stft_callback_q31)(q31_t *stft_output, uint32_t slices);

// The STFT instance is defined for each data type float32, q15 and q31
/**
 * @brief F32 floating-point STFT instance structure.
 * 
 * Contains buffer pointers, window parameters, loop iterators, and the CMSIS-DSP 
 * FFT handler required to compute the Short-Time Fourier Transform.
 */
typedef struct{
    uint32_t buffer_length; // the length of the argument being passed
    int8_t overlap_percent; // determines the overlap between consecutive FFT's
    WINDOW_TYPE window_type; // specified by the user to use the correct windowing function
    uint32_t fft_length; // next power of 2 of buffer length to make computation easier using radix-2
    uint32_t window_length; // the length of the sliding window
    uint32_t hop_length; // calculated from overlap percent or user defined
    uint32_t slices; // number of output window slices for a given input
    uint32_t prev_buffer_remainder; // data to be carried/overlapped from the previous buffer
    uint8_t history_loop_iterator;
    arm_rfft_fast_instance_f32 ffthandler; // FFT instance created to be used by CMSIS ARM RFFT
    float32_t *stft_input_buffer;
    float32_t* stft_output;
    float32_t *history_buffer;
    float32_t *window_coeffecients;
    float32_t* stft_output_mag_array;
    stft_callback_f32 callback;
    uint8_t first_chunk;


    uint32_t start_cycles;
    uint32_t end_cycles;
    uint32_t total_cycles;
    float execution_time_us;

} stft_instance_f32;

/**
 * @brief Q15 fixed-point STFT instance structure.
 */
//
typedef struct{
    uint16_t buffer_length;
    int8_t overlap_percent;
    WINDOW_TYPE window_type;
    uint16_t fft_length;
    uint16_t window_length;
    uint16_t hop_length;
    uint16_t slices;
    uint16_t prev_buffer_remainder;
    uint8_t history_loop_iterator;
    arm_rfft_instance_q15 ffthandler;
    q15_t *stft_input_buffer;
    q15_t *stft_output;
    q15_t *history_buffer;
    q15_t *window_coeffecients;
    q15_t* stft_output_mag_array;
    stft_callback_q15 callback;
    uint8_t first_chunk;

    uint32_t start_cycles;
    uint32_t end_cycles;
    uint32_t total_cycles;
    float execution_time_us;
} stft_instance_q15;

/**
 * @brief Q31 fixed-point STFT instance structure.
 */
//
typedef struct{
    uint32_t buffer_length;
    int8_t overlap_percent;
    WINDOW_TYPE window_type;
    uint32_t fft_length;
    uint32_t window_length;
    uint32_t hop_length;
    uint32_t slices;
    uint32_t prev_buffer_remainder;
    uint8_t history_loop_iterator;
    arm_rfft_instance_q31 ffthandler;
    q31_t *stft_input_buffer;
    q31_t *stft_output;
    q31_t *history_buffer;
    q31_t *window_coeffecients;
    q31_t* stft_output_mag_array;
    stft_callback_q31 callback;
    uint8_t first_chunk;

    uint32_t start_cycles;
    uint32_t end_cycles;
    uint32_t total_cycles;
    float execution_time_us;
} stft_instance_q31;

stft_status stft_instance_init_f32(stft_instance_f32 *inst, uint32_t buffer_length, uint32_t window_length, uint32_t fft_length, int8_t overlap_percent, WINDOW_TYPE window_type, float32_t* work_arena, uint32_t arena_size_elems, uint32_t hop_length);
stft_status stft_instance_init_q15(stft_instance_q15 *inst, uint16_t buffer_length, uint16_t window_length, uint16_t fft_length, int8_t overlap_percent, WINDOW_TYPE window_type, q15_t* work_arena, uint32_t arena_size_elems, uint16_t hop_length);
stft_status stft_instance_init_q31(stft_instance_q31 *inst, uint32_t buffer_length, uint32_t window_length, uint32_t fft_length, int8_t overlap_percent, WINDOW_TYPE window_type, q31_t* work_arena, uint32_t arena_size_elems, uint32_t hop_length);

void stft_adc_to_f32(const uint16_t *adc_raw, float32_t *dst, uint32_t len, uint16_t offset, uint8_t bits);
void stft_adc_to_q15(const uint16_t *adc_raw, q15_t *dst, uint32_t len, uint16_t offset, uint8_t bits);
void stft_adc_to_q31(const uint16_t *adc_raw, q31_t *dst, uint32_t len, uint16_t offset, uint8_t bits);

stft_status stft_f32_set_callback(stft_instance_f32 *inst, stft_callback_f32 cb);
stft_status stft_q15_set_callback(stft_instance_q15 *inst, stft_callback_q15 cb);
stft_status stft_q31_set_callback(stft_instance_q31 *inst, stft_callback_q31 cb);

stft_status stft_f32(stft_instance_f32 *inst, float32_t* user_input);
stft_status stft_q15(stft_instance_q15 *inst, q15_t* user_input);
stft_status stft_q31(stft_instance_q31 *inst, q31_t* user_input);

void window_calculator_f32(float32_t *w, WINDOW_TYPE type, uint32_t N);
void window_calculator_q15(q15_t *w, WINDOW_TYPE type, uint32_t N);
void window_calculator_q31(q31_t *w, WINDOW_TYPE type, uint32_t N);

#endif


