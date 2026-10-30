#define MIN(a, b) ((a) < (b) ? (a) : (b))

#include "cmsis_stft.h"

/**
 * @brief Initializes the F32 floating-point STFT instance and allocates internal buffers.
 *
 * @param[in,out] inst            Pointer to the STFT F32 instance structure.
 * @param[in]     buffer_length   Number of samples in the incoming data chunk.
 * @param[in]     window_length   Number of samples in the window.
 * @param[in]     fft_length      Size of the FFT (must be a power of 2).
 * @param[in]     overlap_percent Percentage of window overlap (0 to 99). Set to -1 to use manual hop_length.
 * @param[in]     window_type     Enum specifying the windowing function (e.g., HANNING).
 * @param[in]     work_arena      Pointer to the statically allocated float32_t memory arena.
 * @param[in]     arena_size_elems Total size of the arena in elements (not bytes).
 * @param[in]     hop_length      Manual hop length (used only if overlap_percent is -1).
 *
 * @retval STFT_OK                Initialization successful.
 * @retval STFT_WARNING_OVERLAP   Overlap calculation truncated, but initialization proceeded.
 * @retval STFT_ERROR_FFT_SIZE    FFT size is invalid or initialization failed in CMSIS-DSP.
 * @retval STFT_ERROR_ALLOC       Arena size is too small for the requested parameters.
 * @retval STFT_ERROR_NULL_PTR    Null pointer provided for instance or arena.
 */
stft_status stft_instance_init_f32(stft_instance_f32 *inst, uint32_t buffer_length, uint32_t window_length, uint32_t fft_length, int8_t overlap_percent, WINDOW_TYPE window_type, float32_t* work_arena, uint32_t arena_size_elems, uint32_t hop_length){
	uint8_t return_status = 0;
	if(!DSP_HAS_FPU){
		return STFT_ERROR;
	}
	if(inst == NULL || work_arena == NULL){
		return STFT_ERROR_NULL_PTR;
	}

	inst->buffer_length = buffer_length;
    inst->window_length = window_length;
	if(inst->window_length > inst->buffer_length){
		return STFT_ERROR;
	}
    if(overlap_percent != -1){
		if(overlap_percent >= 100 || overlap_percent < 0){
			return STFT_ERROR_OVERLAP;
		}
		inst->overlap_percent = overlap_percent;
		if((inst->window_length*(100-inst->overlap_percent))%100 != 0){
			return_status = 1;
		}
		inst->hop_length = (uint32_t) (inst->window_length*(100-inst->overlap_percent)+99)/100;
    }
    else{
    	if(hop_length != 0){
    		inst->hop_length = hop_length;
    	}
    	else{
    		return STFT_ERROR_OVERLAP;
    	}
    }

    inst->window_type = window_type;
    if(fft_length > MAX_FFT_LENGTH || fft_length < MIN_FFT_LENGTH || fft_length < inst->window_length){
        return STFT_ERROR_FFT_SIZE;
    }
    inst->fft_length = fft_length;
    inst->slices = (inst->buffer_length/inst->hop_length) + 1U;
    uint32_t required_elems = window_length + window_length + (2U * fft_length) + (inst->slices * (1+(fft_length / 2U)));
    if(arena_size_elems < required_elems){
    	return STFT_ERROR_ALLOC;
    }
    inst->prev_buffer_remainder = inst->buffer_length%inst->hop_length;
	inst->history_loop_iterator = (uint8_t) ((inst->window_length -inst->prev_buffer_remainder + inst->hop_length - 1U) / inst->hop_length);
	inst->callback = NULL;
    float32_t *ptr = work_arena;
    inst->history_buffer = ptr; ptr+=inst->window_length;
    inst->window_coeffecients = ptr; ptr+= inst->window_length;
    inst->stft_input_buffer = ptr; ptr+=inst->fft_length;
    inst->stft_output = ptr; ptr+= inst->fft_length;
    inst->stft_output_mag_array = ptr;
    memset(work_arena, 0, required_elems*sizeof(float32_t));
    window_calculator_f32(inst->window_coeffecients, inst->window_type, inst->window_length);
    if (arm_rfft_fast_init_f32(&inst->ffthandler, fft_length) != ARM_MATH_SUCCESS) {
		return STFT_ERROR_FFT_SIZE;
	}
    inst->first_chunk = 1;
    inst->start_cycles = 0;
    inst->end_cycles = 0;
    inst->total_cycles = 0;
    inst->execution_time_us = 0.0f;
    if(return_status == 1){
    	return STFT_WARNING_OVERLAP;
    }
    return STFT_OK;

}

/**
 * @brief Initializes the Q15 fixed-point STFT instance and allocates internal buffers.
 *
 * @param[in,out] inst            Pointer to the STFT Q15 instance structure.
 * @param[in]     buffer_length   Number of samples in the incoming data chunk.
 * @param[in]     window_length   Number of samples in the window.
 * @param[in]     fft_length      Size of the FFT (must be a power of 2).
 * @param[in]     overlap_percent Percentage of window overlap (0 to 99). Set to -1 to use manual hop_length.
 * @param[in]     window_type     Enum specifying the windowing function.
 * @param[in]     work_arena      Pointer to the statically allocated q15_t memory arena.
 * @param[in]     arena_size_elems Total size of the arena in elements.
 * @param[in]     hop_length      Manual hop length (used only if overlap_percent is -1).
 *
 * @retval STFT_OK                Initialization successful.
 * @retval STFT_WARNING_OVERLAP   Overlap calculation truncated.
 * @retval STFT_ERROR_ALLOC       Arena size insufficient.
 */
stft_status stft_instance_init_q15(stft_instance_q15 *inst, uint16_t buffer_length, uint16_t window_length, uint16_t fft_length, int8_t overlap_percent, WINDOW_TYPE window_type, q15_t* work_arena, uint32_t arena_size_elems, uint16_t hop_length){
	uint8_t return_status = 0;
	if(inst == NULL || work_arena == NULL){
		return STFT_ERROR_NULL_PTR;
	}

	inst->buffer_length = buffer_length;
	inst->window_length = window_length;
	if(inst->window_length > inst->buffer_length){
		return STFT_ERROR;
	}
	if(overlap_percent != -1){
		if(overlap_percent >= 100 || overlap_percent < 0){
			return STFT_ERROR_OVERLAP;
		}
		inst->overlap_percent = overlap_percent;
		if((inst->window_length*(100-inst->overlap_percent))%100 != 0){
			return_status = 1;
		}
		inst->hop_length = (uint16_t) ((inst->window_length*(100-inst->overlap_percent) + 99)/100);
	}
	else{
		if(hop_length != 0){
			inst->hop_length = hop_length;
		}
		else{
			return STFT_ERROR_OVERLAP;
		}
	}

	inst->window_type = window_type;
	if(fft_length > MAX_FFT_LENGTH || fft_length < MIN_FFT_LENGTH || fft_length < inst->window_length){
		return STFT_ERROR_FFT_SIZE;
	}
	inst->fft_length = fft_length;
	inst->slices = (inst->buffer_length/inst->hop_length) + 1U;
	uint32_t required_elems = window_length + window_length + (3U * fft_length) + (inst->slices * (1+(fft_length / 2U)));
	if(arena_size_elems < required_elems){
		return STFT_ERROR_ALLOC;
	}
	inst->prev_buffer_remainder = inst->buffer_length%inst->hop_length;
	inst->history_loop_iterator = (uint8_t) ((inst->window_length -inst->prev_buffer_remainder + inst->hop_length - 1U) / inst->hop_length);
	inst->callback = NULL;
	q15_t *ptr = work_arena;
	inst->history_buffer = ptr; ptr+=inst->window_length;
	inst->window_coeffecients = ptr; ptr+= inst->window_length;
	inst->stft_input_buffer = ptr; ptr+=inst->fft_length;
	inst->stft_output = ptr; ptr+= 2*inst->fft_length;
	inst->stft_output_mag_array = ptr;
	memset(work_arena, 0, required_elems*sizeof(q15_t));
	window_calculator_q15(inst->window_coeffecients, inst->window_type, inst->window_length);
	if (arm_rfft_init_q15(&inst->ffthandler, fft_length,0 ,1) != ARM_MATH_SUCCESS) {
			return STFT_ERROR_FFT_SIZE;
		}
    inst->first_chunk = 1;
	inst->start_cycles = 0;
    inst->end_cycles = 0;
    inst->total_cycles = 0;
    inst->execution_time_us = 0.0f;
	if(return_status == 1){
		return STFT_WARNING_OVERLAP;
	}
	return STFT_OK;
}

/**
 * @brief Initializes the Q31 fixed-point STFT instance and allocates internal buffers.
 *
 * @param[in,out] inst            Pointer to the STFT Q31 instance structure.
 * @param[in]     buffer_length   Number of samples in the incoming data chunk.
 * @param[in]     window_length   Number of samples in the window.
 * @param[in]     fft_length      Size of the FFT (must be a power of 2).
 * @param[in]     overlap_percent Percentage of window overlap (0 to 99). Set to -1 to use manual hop_length.
 * @param[in]     window_type     Enum specifying the windowing function.
 * @param[in]     work_arena      Pointer to the statically allocated q31_t memory arena.
 * @param[in]     arena_size_elems Total size of the arena in elements.
 * @param[in]     hop_length      Manual hop length (used only if overlap_percent is -1).
 *
 * @retval STFT_OK                Initialization successful.
 * @retval STFT_WARNING_OVERLAP   Overlap calculation truncated.
 * @retval STFT_ERROR_ALLOC       Arena size insufficient.
 */
stft_status stft_instance_init_q31(stft_instance_q31 *inst, uint32_t buffer_length, uint32_t window_length, uint32_t fft_length, int8_t overlap_percent, WINDOW_TYPE window_type, q31_t* work_arena, uint32_t arena_size_elems, uint32_t hop_length){
	uint8_t return_status = 0;
	if(inst == NULL || work_arena == NULL){
		return STFT_ERROR_NULL_PTR;
	}

	inst->buffer_length = buffer_length;
	inst->window_length = window_length;
	if(inst->window_length > inst->buffer_length){
		return STFT_ERROR;
	}
	if(overlap_percent != -1){
		if(overlap_percent >= 100 || overlap_percent < 0){
			return STFT_ERROR_OVERLAP;
		}
		inst->overlap_percent = overlap_percent;
		if((inst->window_length*(100-inst->overlap_percent))%100 != 0){
			return_status = 1;
		}
		inst->hop_length = (uint32_t) ((inst->window_length*(100-inst->overlap_percent) + 99)/100);
	}
	else{
		if(hop_length != 0){
			inst->hop_length = hop_length;
		}
		else{
			return STFT_ERROR_OVERLAP;
		}
	}

	inst->window_type = window_type;
	if(fft_length > MAX_FFT_LENGTH || fft_length < MIN_FFT_LENGTH || fft_length < inst->window_length){
		return STFT_ERROR_FFT_SIZE;
	}
	inst->fft_length = fft_length;
	inst->slices = (inst->buffer_length/inst->hop_length) + 1U;
	uint32_t required_elems = window_length + window_length + (3U * fft_length) + (inst->slices * (1+(fft_length / 2U)));
	if(arena_size_elems < required_elems){
		return STFT_ERROR_ALLOC;
	}
	inst->prev_buffer_remainder = inst->buffer_length%inst->hop_length;
	inst->history_loop_iterator = (uint8_t) ((inst->window_length -inst->prev_buffer_remainder + inst->hop_length - 1U) / inst->hop_length);
	inst->callback = NULL;
	q31_t *ptr = work_arena;
	inst->history_buffer = ptr; ptr+=inst->window_length;
	inst->window_coeffecients = ptr; ptr+= inst->window_length;
	inst->stft_input_buffer = ptr; ptr+=inst->fft_length;
	inst->stft_output = ptr; ptr+= 2*inst->fft_length;
	inst->stft_output_mag_array = ptr;
	memset(work_arena, 0, required_elems*sizeof(q31_t));
	window_calculator_q31(inst->window_coeffecients, inst->window_type, inst->window_length);
	if (arm_rfft_init_q31(&inst->ffthandler, fft_length, 0, 1) != ARM_MATH_SUCCESS) {
			return STFT_ERROR_FFT_SIZE;
		}
    inst->first_chunk = 1;
	inst->start_cycles = 0;
    inst->end_cycles = 0;
    inst->total_cycles = 0;
    inst->execution_time_us = 0.0f;
	if(return_status == 1){
		return STFT_WARNING_OVERLAP;
	}
	return STFT_OK;
}

/**
 * @brief Assigns a user callback function to the F32 STFT instance.
 * @param[in,out] inst Pointer to the initialized F32 instance.
 * @param[in]     cb   Function pointer to the user callback.
 * @retval STFT_OK Successfully assigned.
 * @retval STFT_ERROR_NULL_PTR Null instance pointer.
 */
stft_status stft_f32_set_callback(stft_instance_f32 *inst, stft_callback_f32 cb){
	if(inst == NULL){
		return STFT_ERROR_NULL_PTR;
	}
    inst->callback = cb;
    return STFT_OK;
}

/**
 * @brief Assigns a user callback function to the Q15 STFT instance.
 * @param[in,out] inst Pointer to the initialized Q15 instance.
 * @param[in]     cb   Function pointer to the user callback.
 * @retval STFT_OK Successfully assigned.
 */
stft_status stft_q15_set_callback(stft_instance_q15 *inst, stft_callback_q15 cb){
	if(inst == NULL){
		return STFT_ERROR_NULL_PTR;
	}
    inst->callback = cb;
    return STFT_OK;
}

/**
 * @brief Assigns a user callback function to the Q31 STFT instance.
 * @param[in,out] inst Pointer to the initialized Q31 instance.
 * @param[in]     cb   Function pointer to the user callback.
 * @retval STFT_OK Successfully assigned.
 */
stft_status stft_q31_set_callback(stft_instance_q31 *inst, stft_callback_q31 cb){
	if(inst == NULL){
		return STFT_ERROR_NULL_PTR;
	}
    inst->callback = cb;
    return STFT_OK;
}

/**
 * @brief Converts raw unsigned 16-bit ADC integers into centered float32_t values.
 * @param[in]  adc_raw Array of raw unsigned ADC values.
 * @param[out] dst     Array to store the centered F32 results.
 * @param[in]  len     Number of elements to convert.
 * @param[in]  offset  DC offset to subtract from the raw ADC value.
 * @param[in]  bits    Bit resolution of the ADC (unused in F32, included for API consistency).
 */
void stft_adc_to_f32(const uint16_t *adc_raw, float32_t *dst, uint32_t len, uint16_t offset, uint8_t bits){

	for(int i=0; i<len; i++){
		dst[i] = (float32_t) ((int32_t) adc_raw[i] - (int32_t) offset);
	}
}

/**
 * @brief Converts raw unsigned 16-bit ADC integers into centered, properly shifted Q15 values.
 * @param[in]  adc_raw Array of raw unsigned ADC values.
 * @param[out] dst     Array to store the saturated Q15 results.
 * @param[in]  len     Number of elements to convert.
 * @param[in]  offset  DC offset to subtract from the raw ADC value.
 * @param[in]  bits    Bit resolution of the ADC (determines left-shift magnitude).
 */
void stft_adc_to_q15(const uint16_t *adc_raw, q15_t *dst, uint32_t len, uint16_t offset, uint8_t bits){
	for(int i=0; i<len; i++){
		int32_t centered = (int32_t)adc_raw[i] - (int32_t)offset;
		dst[i] = (q15_t) __SSAT(centered<<((15-MIN(bits,15))),16);
	}
}

/**
 * @brief Converts raw unsigned 16-bit ADC integers into centered, properly shifted Q31 values.
 * @param[in]  adc_raw Array of raw unsigned ADC values.
 * @param[out] dst     Array to store the saturated Q31 results.
 * @param[in]  len     Number of elements to convert.
 * @param[in]  offset  DC offset to subtract from the raw ADC value.
 * @param[in]  bits    Bit resolution of the ADC (determines left-shift magnitude).
 */
void stft_adc_to_q31(const uint16_t *adc_raw, q31_t *dst, uint32_t len, uint16_t offset, uint8_t bits){
	for(int i=0; i<len; i++){
		int32_t centered = (int32_t)adc_raw[i] - (int32_t)offset;
		dst[i] = (q31_t) __SSAT((centered<<(31-MIN(bits,31))),32);
	}
}

/**
 * @brief Executes the Short-Time Fourier Transform using the F32 format.
 *
 * Handles memory chunking, window application, FFT calculation, and complex magnitude extraction.
 * Triggers the user callback with the magnitude array once processing is complete.
 *
 * @param[in,out] inst       Pointer to the initialized F32 instance.
 * @param[in]     user_input Pointer to the current floating-point input chunk.
 * @retval STFT_OK           Execution successful.
 * @retval STFT_ERROR_NULL_PTR Invalid pointer provided.
 */
stft_status stft_f32(stft_instance_f32 *inst, float32_t* user_input){
	if (inst == NULL || user_input == NULL) return STFT_ERROR_NULL_PTR;
	inst->history_loop_iterator = (uint8_t) ((inst->window_length -inst->prev_buffer_remainder + inst->hop_length - 1U) / inst->hop_length);
	if(inst->first_chunk){
		inst->history_loop_iterator = 1;
		inst->first_chunk = 0;
		inst->prev_buffer_remainder = 0;
	}
	uint32_t current_loop_iterator = (inst->buffer_length + inst->prev_buffer_remainder - inst->window_length) / inst->hop_length;
	inst->slices = inst->history_loop_iterator + current_loop_iterator;
	for(int i=0; i<inst->history_loop_iterator; i++){
		uint32_t segment1 = (inst->history_loop_iterator - 1 - i)*inst->hop_length + inst->prev_buffer_remainder;
		uint32_t segment2 = inst->window_length - segment1;
		memcpy(inst->stft_input_buffer, inst->history_buffer + inst->window_length - segment1, segment1*(sizeof(float32_t)));
		memcpy(inst->stft_input_buffer + segment1, user_input, segment2*(sizeof(float32_t)));
		memset(inst->stft_input_buffer + inst->window_length, 0, sizeof(float32_t)*(inst->fft_length - inst->window_length));
		if(inst->window_type != NO_WINDOW){
			arm_mult_f32(inst->stft_input_buffer, inst->window_coeffecients, inst->stft_input_buffer, inst->window_length);
		}
		arm_rfft_fast_f32(&inst->ffthandler, inst->stft_input_buffer, inst->stft_output, 0);
		arm_cmplx_mag_f32(inst->stft_output + 2, inst->stft_output_mag_array + i*(1+inst->fft_length/2) + 1, inst->fft_length/2 - 1);
		inst->stft_output_mag_array[i*(inst->fft_length/2 + 1)] = fabsf(inst->stft_output[0]);
		inst->stft_output_mag_array[i*(inst->fft_length/2 + 1) + inst->fft_length/2] = fabsf(inst->stft_output[1]);
	}

	current_loop_iterator = inst->slices - inst->history_loop_iterator;
	for(int i=0; i<current_loop_iterator; i++){
		memcpy(inst->stft_input_buffer, user_input + i*inst->hop_length + inst->hop_length - inst->prev_buffer_remainder, sizeof(float32_t)*inst->window_length);
		memset(inst->stft_input_buffer + inst->window_length, 0, sizeof(float32_t)*(inst->fft_length - inst->window_length));
		if(inst->window_type != NO_WINDOW){
			arm_mult_f32(inst->stft_input_buffer, inst->window_coeffecients, inst->stft_input_buffer, inst->window_length);
		}
		arm_rfft_fast_f32(&inst->ffthandler, inst->stft_input_buffer, inst->stft_output, 0);
		arm_cmplx_mag_f32(inst->stft_output + 2, inst->stft_output_mag_array + (inst->history_loop_iterator+i)*(1+inst->fft_length/2) + 1, inst->fft_length/2 - 1);
		inst->stft_output_mag_array[(inst->history_loop_iterator+i)*(inst->fft_length/2 + 1)] = fabsf(inst->stft_output[0]);
		inst->stft_output_mag_array[(inst->history_loop_iterator+i)*(inst->fft_length/2+1) + inst->fft_length/2] = fabsf(inst->stft_output[1]);
	}

	memcpy(inst->history_buffer, user_input + inst->buffer_length - inst->window_length, sizeof(float32_t)*inst->window_length);
	inst->prev_buffer_remainder = (inst->prev_buffer_remainder + inst->buffer_length) % inst->hop_length;
	if(inst->callback != NULL){
		inst->callback(inst->stft_output_mag_array, inst->slices);
	}
	return STFT_OK;
}

/**
 * @brief Executes the Short-Time Fourier Transform using the Q31 fixed-point format.
 * @param[in,out] inst       Pointer to the initialized Q31 instance.
 * @param[in]     user_input Pointer to the current Q31 input chunk.
 * @retval STFT_OK           Execution successful.
 */
stft_status stft_q31(stft_instance_q31 *inst, q31_t* user_input){

	// Total scaling factor here is N
	if (inst == NULL || user_input == NULL) return STFT_ERROR_NULL_PTR;
	inst->history_loop_iterator = (uint8_t) ((inst->window_length -inst->prev_buffer_remainder + inst->hop_length - 1U) / inst->hop_length);
	if(inst->first_chunk){
		inst->history_loop_iterator = 1;
		inst->first_chunk = 0;
		inst->prev_buffer_remainder = 0;
	}
	uint32_t current_loop_iterator = (inst->buffer_length + inst->prev_buffer_remainder - inst->window_length) / inst->hop_length;
	inst->slices = inst->history_loop_iterator + current_loop_iterator;
	for(int i=0; i<inst->history_loop_iterator; i++){
		uint32_t segment1 = (inst->history_loop_iterator - 1 - i)*inst->hop_length + inst->prev_buffer_remainder;
		uint32_t segment2 = inst->window_length - segment1;
		memcpy(inst->stft_input_buffer, inst->history_buffer + inst->window_length - segment1, segment1*(sizeof(q31_t)));
		memcpy(inst->stft_input_buffer + segment1, user_input, segment2*(sizeof(q31_t)));
		memset(inst->stft_input_buffer + inst->window_length, 0, sizeof(q31_t)*(inst->fft_length - inst->window_length));
		if(inst->window_type != NO_WINDOW){
			arm_mult_q31(inst->stft_input_buffer, inst->window_coeffecients, inst->stft_input_buffer, inst->window_length);
		}
		arm_rfft_q31(&inst->ffthandler, inst->stft_input_buffer, inst->stft_output);
		arm_cmplx_mag_q31(inst->stft_output + 2, inst->stft_output_mag_array + i*(1+inst->fft_length/2) + 1, inst->fft_length/2 - 1);
		inst->stft_output_mag_array[i*(1+inst->fft_length/2)] = abs(inst->stft_output[0] >> 1);
		inst->stft_output_mag_array[i*(1+inst->fft_length/2) + inst->fft_length/2] = abs(inst->stft_output[inst->fft_length] >> 1);
	}

	current_loop_iterator = inst->slices - inst->history_loop_iterator;
	for(int i=0; i<current_loop_iterator; i++){
		memcpy(inst->stft_input_buffer, user_input + i*inst->hop_length + inst->hop_length - inst->prev_buffer_remainder, sizeof(q31_t)*inst->window_length);
		memset(inst->stft_input_buffer + inst->window_length, 0, sizeof(q31_t)*(inst->fft_length - inst->window_length));
		if(inst->window_type != NO_WINDOW){
			arm_mult_q31(inst->stft_input_buffer, inst->window_coeffecients, inst->stft_input_buffer, inst->window_length);
		}
		arm_rfft_q31(&inst->ffthandler, inst->stft_input_buffer, inst->stft_output);
		arm_cmplx_mag_q31(inst->stft_output + 2, inst->stft_output_mag_array + (inst->history_loop_iterator+i)*(1+inst->fft_length/2) + 1, inst->fft_length/2 - 1);
		inst->stft_output_mag_array[(inst->history_loop_iterator+i)*(1+inst->fft_length/2)] = abs(inst->stft_output[0] >> 1);
		inst->stft_output_mag_array[(inst->history_loop_iterator+i)*(1+inst->fft_length/2) + inst->fft_length/2] = abs(inst->stft_output[inst->fft_length] >> 1);
	}

	memcpy(inst->history_buffer, user_input + inst->buffer_length - inst->window_length, sizeof(q31_t)*inst->window_length);
	inst->prev_buffer_remainder = (inst->prev_buffer_remainder + inst->buffer_length) % inst->hop_length;
	if(inst->callback != NULL){
		inst->callback(inst->stft_output_mag_array, inst->slices);
	}
	return STFT_OK;
}


/**
 * @brief Executes the Short-Time Fourier Transform using the Q15 fixed-point format.
 * @param[in,out] inst       Pointer to the initialized Q15 instance.
 * @param[in]     user_input Pointer to the current Q15 input chunk.
 * @retval STFT_OK           Execution successful.
 */
stft_status stft_q15(stft_instance_q15 *inst, q15_t* user_input){


	// Total scaling factor here is N
	if (inst == NULL || user_input == NULL) return STFT_ERROR_NULL_PTR;
	inst->history_loop_iterator = (uint8_t) ((inst->window_length -inst->prev_buffer_remainder + inst->hop_length - 1U) / inst->hop_length);
	if(inst->first_chunk){
		inst->history_loop_iterator = 1;
		inst->first_chunk = 0;
		inst->prev_buffer_remainder = 0;
	}
	uint32_t current_loop_iterator = (inst->buffer_length + inst->prev_buffer_remainder - inst->window_length) / inst->hop_length;
	inst->slices = inst->history_loop_iterator + current_loop_iterator;
	for(int i=0; i<inst->history_loop_iterator; i++){
		uint16_t segment1 = (inst->history_loop_iterator - 1 - i)*inst->hop_length + inst->prev_buffer_remainder;
		uint16_t segment2 = inst->window_length - segment1;
		memcpy(inst->stft_input_buffer, inst->history_buffer + inst->window_length - segment1, segment1*(sizeof(q15_t)));
		memcpy(inst->stft_input_buffer + segment1, user_input, segment2*(sizeof(q15_t)));
		memset(inst->stft_input_buffer + inst->window_length, 0, sizeof(q15_t)*(inst->fft_length - inst->window_length));
		if(inst->window_type != NO_WINDOW){
			arm_mult_q15(inst->stft_input_buffer, inst->window_coeffecients, inst->stft_input_buffer, inst->window_length);
		}
		arm_rfft_q15(&inst->ffthandler, inst->stft_input_buffer, inst->stft_output);
		arm_cmplx_mag_q15(inst->stft_output + 2, inst->stft_output_mag_array + i*(1+inst->fft_length/2) + 1, inst->fft_length/2 - 1);
		inst->stft_output_mag_array[i*(1+inst->fft_length/2)] = abs(inst->stft_output[0] >> 1);
		inst->stft_output_mag_array[i*(1+inst->fft_length/2) + inst->fft_length/2] = abs(inst->stft_output[inst->fft_length] >> 1);
	}

	current_loop_iterator = inst->slices - inst->history_loop_iterator;
	for(int i=0; i<current_loop_iterator; i++){
		memcpy(inst->stft_input_buffer, user_input + i*inst->hop_length + inst->hop_length - inst->prev_buffer_remainder, sizeof(q15_t)*inst->window_length);
		memset(inst->stft_input_buffer + inst->window_length, 0, sizeof(q15_t)*(inst->fft_length - inst->window_length));
		if(inst->window_type != NO_WINDOW){
			arm_mult_q15(inst->stft_input_buffer, inst->window_coeffecients, inst->stft_input_buffer, inst->window_length);
		}
		arm_rfft_q15(&inst->ffthandler, inst->stft_input_buffer, inst->stft_output);
		arm_cmplx_mag_q15(inst->stft_output + 2, inst->stft_output_mag_array + (inst->history_loop_iterator+i)*(1+inst->fft_length/2) + 1,inst->fft_length/2 - 1);
		inst->stft_output_mag_array[(inst->history_loop_iterator+i)*(1+inst->fft_length/2)] = abs(inst->stft_output[0] >> 1);
		inst->stft_output_mag_array[(inst->history_loop_iterator+i)*(1+inst->fft_length/2) + inst->fft_length/2] = abs(inst->stft_output[inst->fft_length] >> 1);
	}

	memcpy(inst->history_buffer, user_input + inst->buffer_length - inst->window_length, sizeof(q15_t)*inst->window_length);
	inst->prev_buffer_remainder = (inst->prev_buffer_remainder + inst->buffer_length) % inst->hop_length;
	if(inst->callback != NULL){
		inst->callback(inst->stft_output_mag_array, inst->slices);
	}
	return STFT_OK;
}

/**
 * @brief Flushes any remaining samples in the F32 history buffer by zero-padding the final slice.
 * @param[in,out] inst Pointer to the initialized F32 instance.
 * @retval STFT_OK     Flush completed successfully.
 */
stft_status flush_f32(stft_instance_f32 *inst){
	if(inst->prev_buffer_remainder != 0){
		memcpy(inst->stft_input_buffer, inst->history_buffer + inst->window_length - inst->prev_buffer_remainder, inst->prev_buffer_remainder*(sizeof(float32_t)));
		memset(inst->stft_input_buffer + inst->prev_buffer_remainder, 0, sizeof(float32_t)*(inst->fft_length - inst->prev_buffer_remainder));
		if(inst->window_type != NO_WINDOW){
			arm_mult_f32(inst->stft_input_buffer, inst->window_coeffecients, inst->stft_input_buffer, inst->window_length);
		}
		arm_rfft_fast_f32(&inst->ffthandler, inst->stft_input_buffer, inst->stft_output, 0);
		arm_cmplx_mag_f32(inst->stft_output + 2, inst->stft_output_mag_array + 1, inst->fft_length/2 - 1);
		inst->stft_output_mag_array[0] = fabsf(inst->stft_output[0]);
		inst->stft_output_mag_array[inst->fft_length/2] = fabsf(inst->stft_output[1]);
		if(inst->callback != NULL){
			inst->callback(inst->stft_output_mag_array, 1);
		}
	}
	else{

	}
	inst->first_chunk = 1;
	return STFT_OK;
}

/**
 * @brief Flushes any remaining samples in the Q31 history buffer by zero-padding the final slice.
 * @param[in,out] inst Pointer to the initialized Q31 instance.
 * @retval STFT_OK     Flush completed successfully.
 */
stft_status flush_q31(stft_instance_q31 *inst){
	if(inst->prev_buffer_remainder != 0){
		memcpy(inst->stft_input_buffer,inst->history_buffer + inst->window_length - inst->prev_buffer_remainder, inst->prev_buffer_remainder*(sizeof(q31_t)));
		memset(inst->stft_input_buffer + inst->prev_buffer_remainder, 0, sizeof(q31_t)*(inst->fft_length - inst->prev_buffer_remainder));
		if(inst->window_type != NO_WINDOW){
			arm_mult_q31(inst->stft_input_buffer, inst->window_coeffecients, inst->stft_input_buffer, inst->window_length);
		}
		arm_rfft_q31(&inst->ffthandler, inst->stft_input_buffer, inst->stft_output);
		arm_cmplx_mag_q31(inst->stft_output + 2, inst->stft_output_mag_array + 1, inst->fft_length/2 - 1);
		inst->stft_output_mag_array[0] = abs(inst->stft_output[0] >> 1);
		inst->stft_output_mag_array[inst->fft_length/2] = abs(inst->stft_output[inst->fft_length] >> 1);
		if(inst->callback != NULL){
			inst->callback(inst->stft_output_mag_array, 1);
		}
	}
	else{

	}
	inst->first_chunk = 1;
	return STFT_OK;
}

/**
 * @brief Flushes any remaining samples in the Q15 history buffer by zero-padding the final slice.
 * @param[in,out] inst Pointer to the initialized Q15 instance.
 * @retval STFT_OK     Flush completed successfully.
 */
stft_status flush_q15(stft_instance_q15 *inst){
	if(inst->prev_buffer_remainder != 0){
		memcpy(inst->stft_input_buffer,inst->history_buffer + inst->window_length - inst->prev_buffer_remainder, inst->prev_buffer_remainder*(sizeof(q15_t)));
		memset(inst->stft_input_buffer + inst->prev_buffer_remainder, 0, sizeof(q15_t)*(inst->fft_length - inst->prev_buffer_remainder));
		if(inst->window_type != NO_WINDOW){
			arm_mult_q15(inst->stft_input_buffer, inst->window_coeffecients, inst->stft_input_buffer, inst->window_length);
		}
		arm_rfft_q15(&inst->ffthandler, inst->stft_input_buffer, inst->stft_output);
		arm_cmplx_mag_q15(inst->stft_output + 2, inst->stft_output_mag_array + 1, inst->fft_length/2 - 1);
		inst->stft_output_mag_array[0] = abs(inst->stft_output[0] >> 1);
		inst->stft_output_mag_array[inst->fft_length/2] = abs(inst->stft_output[inst->fft_length] >> 1);
		if(inst->callback != NULL){
			inst->callback(inst->stft_output_mag_array, 1);
		}
	}
	else{

	}
	inst->first_chunk = 1;
	return STFT_OK;
}

/**
 * @brief Generates symmetric windowing coefficients in F32 format.
 * @param[out] w    Pointer to the array where coefficients will be stored.
 * @param[in]  type The specific window function to apply.
 * @param[in]  N    Length of the window.
 */
void window_calculator_f32(float32_t *w, WINDOW_TYPE type, uint32_t N) {
    if (w == NULL || N == 0U) return;

    switch (type) {
        case HANNING:
            for (uint32_t i = 0; i < N; i++) {
                w[i] = 0.5f * (1.0f - arm_cos_f32((2.0f * PI * (float32_t)i) / (float32_t)N));
            }
            break;

        case HAMMING:
            for (uint32_t i = 0; i < N; i++) {
                w[i] = 0.54f - (0.46f * arm_cos_f32((2.0f * PI * (float32_t)i) / (float32_t)N));
            }
            break;

        case BLACKMAN:
            for (uint32_t i = 0; i < N; i++) {
                float32_t angle = (2.0f * PI * (float32_t)i) / (float32_t)N;
                w[i] = 0.42f - (0.5f * arm_cos_f32(angle)) + (0.08f * arm_cos_f32(2.0f * angle));
            }
            break;

        case BARTLETT:
            if (N == 1U) {
                w[0] = 1.0f;
            } else {
                for (uint32_t i = 0; i < N; i++) {
                    int32_t num = (int32_t)(N - 1U) - abs((int32_t)(2U * i) - (int32_t)(N - 1U));
                    w[i] = (float32_t)num / (float32_t)(N - 1U);
                }
            }
            break;

        case NO_WINDOW:
        default:
            for (uint32_t i = 0; i < N; i++) {
                w[i] = 1.0f;
            }
            break;
    }
}
/**
 * @brief Generates symmetric windowing coefficients in Q15 format.
 * @param[out] w    Pointer to the array where coefficients will be stored.
 * @param[in]  type The specific window function to apply.
 * @param[in]  N    Length of the window.
 */
void window_calculator_q15(q15_t *w, WINDOW_TYPE type, uint32_t N) {
    if (w == NULL || N == 0U) return;

    switch (type) {
        case HANNING:
            for (uint32_t i = 0; i < N; i++) {
                q15_t angle = (q15_t)(((i << 15U) / N) & 0x7FFFU);
                int32_t c1  = (int32_t)arm_cos_q15(angle);
                /* 0.5 * (1.0 - cos) in Q15: (32768 - c1) >> 1 */
                int32_t val = (32768 - c1) >> 1;
                w[i] = (q15_t)__SSAT(val, 16);
            }
            break;

        case HAMMING:
            for (uint32_t i = 0; i < N; i++) {
                q15_t angle = (q15_t)(((i << 15U) / N) & 0x7FFFU);
                int32_t c1  = (int32_t)arm_cos_q15(angle);
                /* 0.54 = 17695/32768, 0.46 = 15073/32768 */
                int32_t val = ((17695 * 32768) - (15073 * c1) + 16384) >> 15;
                w[i] = (q15_t)__SSAT(val, 16);
            }
            break;

        case BLACKMAN:
            for (uint32_t i = 0; i < N; i++) {
                uint32_t phase = (i << 15U) / N;
                int32_t c1 = (int32_t)arm_cos_q15((q15_t)(phase & 0x7FFFU));
                int32_t c2 = (int32_t)arm_cos_q15((q15_t)((2U * phase) & 0x7FFFU));
                /* 0.42 = 13763/32768, 0.50 = 16384/32768, 0.08 = 2621/32768 */
                int32_t val = ((13763 * 32768) - (16384 * c1) + (2621 * c2) + 16384) >> 15;
                w[i] = (q15_t)__SSAT(val, 16);
            }
            break;

        case BARTLETT:
            if (N == 1U) {
                w[0] = 0x7FFF;
            } else {
                for (uint32_t i = 0; i < N; i++) {
                    int32_t num = (int32_t)(N - 1U) - abs((int32_t)(2U * i) - (int32_t)(N - 1U));
                    int32_t val = (num * 32767) / (int32_t)(N - 1U);
                    w[i] = (q15_t)__SSAT(val, 16);
                }
            }
            break;

        case NO_WINDOW:
        default:
            for (uint32_t i = 0; i < N; i++) {
                w[i] = 0x7FFF;
            }
            break;
    }
}

/**
 * @brief Generates symmetric windowing coefficients in Q31 format.
 * @param[out] w    Pointer to the array where coefficients will be stored.
 * @param[in]  type The specific window function to apply.
 * @param[in]  N    Length of the window.
 */
void window_calculator_q31(q31_t *w, WINDOW_TYPE type, uint32_t N) {
    if (w == NULL || N == 0U) return;

    switch (type) {
        case HANNING:
            for (uint32_t i = 0; i < N; i++) {
                q31_t angle = (q31_t)((((uint64_t)i << 31U) / N) & 0x7FFFFFFFULL);
                int64_t c1  = (int64_t)arm_cos_q31(angle);
                /* 0.5 * (1.0 - cos) in Q31 */
                int64_t val = ((1LL << 31U) - c1) >> 1;
                w[i] = (val > 0x7FFFFFFFLL) ? 0x7FFFFFFF : (q31_t)val;
            }
            break;

        case HAMMING:
            for (uint32_t i = 0; i < N; i++) {
                q31_t angle = (q31_t)((((uint64_t)i << 31U) / N) & 0x7FFFFFFFULL);
                int64_t c1  = (int64_t)arm_cos_q31(angle);
                /* 0.54 in Q31 = 1159641170, 0.46 in Q31 = 987842478 */
                int64_t val = ((1159641170LL << 31U) - (987842478LL * c1) + (1LL << 30U)) >> 31;
                w[i] = (val > 0x7FFFFFFFLL) ? 0x7FFFFFFF : (q31_t)val;
            }
            break;

        case BLACKMAN:
            for (uint32_t i = 0; i < N; i++) {
                uint64_t phase = ((uint64_t)i << 31U) / N;
                int64_t c1 = (int64_t)arm_cos_q31((q31_t)(phase & 0x7FFFFFFFULL));
                int64_t c2 = (int64_t)arm_cos_q31((q31_t)((2ULL * phase) & 0x7FFFFFFFULL));
                /* 0.42 = 901943132, 0.50 = 1073741824, 0.08 = 171798692 in Q31 */
                int64_t val = ((901943132LL << 31U) - (1073741824LL * c1) + (171798692LL * c2) + (1LL << 30U)) >> 31;
                w[i] = (val > 0x7FFFFFFFLL) ? 0x7FFFFFFF : (q31_t)val;
            }
            break;

        case BARTLETT:
            if (N == 1U) {
                w[0] = 0x7FFFFFFF;
            } else {
                for (uint32_t i = 0; i < N; i++) {
                    int64_t num = (int64_t)(N - 1U) - (int64_t)abs((int32_t)(2U * i) - (int32_t)(N - 1U));
                    int64_t val = (num * 0x7FFFFFFFLL) / (int64_t)(N - 1U);
                    w[i] = (q31_t)val;
                }
            }
            break;

        case NO_WINDOW:
        default:
            for (uint32_t i = 0; i < N; i++) {
                w[i] = 0x7FFFFFFF;
            }
            break;
    }
}
