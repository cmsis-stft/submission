/* =========================================================================
   stft_port.h - Macro & Portability Documentation
   ========================================================================= */



#ifndef STFT_PORT_H
#define STFT_PORT_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* =========================================================================
   1. CONFIGURATION LIMITS
   ========================================================================= */

#define MAX_FFT_LENGTH 4096
#define MIN_FFT_LENGTH 32

/* =========================================================================
   2. COMPILER-AGNOSTIC CORE DETECTION
   ========================================================================= */
#if defined(__CORTEX_M)
    #if (__CORTEX_M == 0)
        #define ARM_MATH_CM0
    #elif (__CORTEX_M == 3)
        #define ARM_MATH_CM3
    #elif (__CORTEX_M == 4)
        #define ARM_MATH_CM4
    #elif (__CORTEX_M == 7)
        #define ARM_MATH_CM7
    #elif (__CORTEX_M == 33)
        #define ARM_MATH_CM33
    #endif
#elif defined(__ARM_ARCH_7EM__)
    #define ARM_MATH_CM4
#elif defined(__ARM_ARCH_7M__)
    #define ARM_MATH_CM3
#elif defined(__ARM_ARCH_6M__)
    #define ARM_MATH_CM0
#else
    /* Fallback to Vendor Macros (Legacy Support) */
    #if defined(STM32F0xx) || defined(STM32L0xx)
        #define ARM_MATH_CM0
    #elif defined(STM32F1xx) || defined(STM32F2xx) || defined(STM32L1xx)
        #define ARM_MATH_CM3
    #elif defined(STM32F3xx) || defined(STM32F4xx) || defined(STM32L4xx) || defined(STM32G4xx) || defined(STM32WBxx)
        #define ARM_MATH_CM4
    #elif defined(STM32F7xx) || defined(STM32H7xx)
        #define ARM_MATH_CM7
    #elif defined(STM32U5xx) || defined(STM32H5xx)
        #define ARM_MATH_CM33
    #else
        #warning "Unknown target architecture — ARM_MATH core not auto-detected. Define manually."
    #endif
#endif

/* =========================================================================
   3. FPU DETECTION (CMSIS Standard)
   ========================================================================= */
#if (defined(__FPU_PRESENT) && (__FPU_PRESENT == 1U)) && (defined(__FPU_USED) && (__FPU_USED == 1U))
    #define DSP_HAS_FPU 1
#elif defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_8M_MAIN__)
    #define DSP_HAS_FPU 1
#else
    #define DSP_HAS_FPU 0
#endif

/* =========================================================================
   4. MEMORY ALIGNMENT MACROS
   ========================================================================= */
// Required for optimal SIMD/DMA performance in DSP applications
/**
 * @brief Ensures a variable or array is aligned in memory to 'x' bytes.
 * 
 * Required for optimal SIMD and DMA performance in DSP applications.
 * @param x The byte alignment boundary (e.g., 4).
 */
#if defined(__GNUC__) || defined(__clang__)
    #define STFT_ALIGN(x) __attribute__((aligned(x)))
#elif defined(__ICCARM__)
    #define STFT_ALIGN(x) _Pragma("data_alignment=" #x)
#else
    #define STFT_ALIGN(x)
#endif

#endif

