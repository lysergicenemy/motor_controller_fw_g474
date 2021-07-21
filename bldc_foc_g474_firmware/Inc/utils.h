/*
	Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef UTILS_H_
#define UTILS_H_

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

float utSin(float angle); // tested - OK, ~111 cpuCycles
float utCos(float angle); // tested - OK, ~120 cpuCycles

void utils_step_towards(float *value, float goal, float step);
float utils_calc_ratio(float low, float high, float val);
void utils_norm_angle(float *angle);
void utils_norm_angle_rad(float *angle);
int utils_truncate_number(float *number, float min, float max);
int utils_truncate_number_int(int *number, int min, int max);
int utils_truncate_number_abs(float *number, float max);
float utils_map(float x, float in_min, float in_max, float out_min, float out_max);
int utils_map_int(int x, int in_min, int in_max, int out_min, int out_max);
void utils_deadband(float *value, float tres, float max);
float utils_angle_difference(float angle1, float angle2);
float utils_angle_difference_rad(float angle1, float angle2);
float utils_avg_angles_rad_fast(float *angles, float *weights, int angles_num);
float utils_avg_angle_rad_fast(float angle1, float angle2);
float utils_middle_of_3(float a, float b, float c);
int utils_middle_of_3_int(int a, int b, int c);
float utils_fast_inv_sqrt(float x);
float utils_fast_atan2(float y, float x);
bool utils_saturate_vector_2d(float *x, float *y, float max);
void utils_fast_sincos(float angle, float *sin, float *cos);
float utils_min_abs(float va, float vb);
float utils_max_abs(float va, float vb);
void utils_byte_to_binary(int x, char *b);
float utils_throttle_curve(float val, float curve_acc, float curve_brake, int mode);
uint32_t utils_crc32c(uint8_t *data, uint32_t len);
void utils_fft32_bin2(float *real_in, float *real, float *imag);
void utils_fft16_bin0(float *real_in, float *real, float *imag);
void utils_fft8_bin0(float *real_in, float *real, float *imag);
uint8_t utils_second_motor_id(void);
float utils_batt_liion_norm_v_to_capacity(float norm_v);
uint16_t utils_median_filter_uint16_run(uint16_t *buffer,
										unsigned int *buffer_index, unsigned int filter_len, uint16_t sample);

// Return the sign of the argument. -1 if negative, 1 if zero or positive.
#define SIGN(x) ((x < 0) ? -1 : 1)

#define SAT(A, Pos, Neg) (((A) > (Pos)) ? (Pos) : (((A) < (Neg)) ? (Neg) : (A)))
#define ABS(x) ((x < 0.0f) ? x = -x : x)

// Squared
#define SQ(x) ((x) * (x))
#define MAGF(x,y) ( sqrtf(SQ(x) + SQ(y)) )

// nan and infinity check for floats
#define UTILS_IS_INF(x) ((x) == (1.0f / 0.0f) || (x) == (-1.0f / 0.0f))
#define UTILS_IS_NAN(x) ((x) != (x))
#define UTILS_NAN_ZERO(x) (x = UTILS_IS_NAN(x) ? 0.0f : x)

/**
 * A simple low pass filter.
 *
 * @param value
 * The filtered value.
 *
 * @param sample
 * Next sample.
 *
 * @param filter_constant
 * Filter constant. Range 0.0 to 1.0, where 1.0 gives the unfiltered value.
 */
#define UTILS_LP_FAST(value, sample, filter_constant) (value -= (filter_constant) * ((value) - (sample)))
/* Saturation */
#define SAT(A, Pos, Neg) (((A) > (Pos)) ? (Pos) : (((A) < (Neg)) ? (Neg) : (A)))

// Constants
#define ONE_BY_SQRT3 (0.57735026919f)			 // 1 / sqrt(3)
#define ONE_BY_SQRT2 (0.707106781186f)			 // 1 / sqrt(2)
#define TWO_BY_SQRT3 (2.0f * 0.57735026919f)	 // (1 / sqrt(3)) * 2
#define SQRT3_BY_2 (0.86602540378f)				 // sqrt(3) / 2
#define SQRT2 (1.414213562f)					 // sqrt(2)
#define COS_30_DEG (0.86602540378f)
#define SIN_30_DEG (0.5f)
#define COS_MINUS_30_DEG (0.86602540378f)
#define SIN_MINUS_30_DEG (-0.5f)
#define RADS2RPM 9.54929658f
#define RPM2RADS (1.f / RADS2RPM)

#define MF_PI 3.14159265358979f
#define M_2PI (MF_PI * 2.0f)
#define M_PI2 (MF_PI * 0.5f)
#define M_PI4 (MF_PI * 0.25f)
#define M_PI3 (MF_PI / 3.0f)
#define ONE_BY_PI (0.318309886f)

#define _IQ(A) (int32_t)((A)*16777216.0f)
#define _IQ31(A) (int32_t)((A)*2147483648.f)
#define _IQmpy(A, B) (int32_t)(((int64_t)(A) * (B)) >> 24)
#define _IQtoF(A) ((float)(A) * (1.0f / 16777216.0f))
#define _IQ31toF(A) ((float)(A) * (1.0f / 2147483648.f))
#define _IQ31toF_FAST(A) ((float)(A) * (0.0000000004656612873f))

/** Fast fixed point sin()
 * ~12-bit, ~30 CPU cycles
 * A sine approximation via  a fourth-order cosine approx.
 * floatingPoint cast example: msin = _IQtoF(utSinAbs( _IQ(angle) ));
 * http://www.coranac.com/2009/07/sines/
 * */

static inline int32_t utSinPU(int32_t x)
{
	int32_t c, y;
	static const int32_t qN = 13, qA = 12, B = 19900, C = 3516;
	x = x >> 9;			//from 8.24
	c = x << (30 - qN); // Semi-circle info into carry.
	x -= 1 << qN;		// sine -> cosine calc

	x = x << (31 - qN);			// Mask with PI
	x = x >> (31 - qN);			// Note: SIGNED shift! (to qN)
	x = x * x >> (2 * qN - 14); // x=x^2 To Q14

	y = B - (x * C >> 14);		   // B - x^2*C
	y = (1 << qA) - (x * y >> 16); // A - x^2*(B-x^2*C)
	y = y << 12;				   // to 8.24
	return c >= 0 ? y : -y;
}
static inline int32_t utCosPU(int32_t inAngle)
{
	return utSinPU(inAngle + _IQ(0.25));
}

static inline int32_t utSinAbs(int32_t x)
{
	x = _IQmpy(x, _IQ(0.15915494309189533576f));
	return utSinPU(x);
}

static inline int32_t utCosAbs(int32_t x)
{
	x = _IQmpy(x, _IQ(0.15915494309189533576f));
	return utSinPU(x + _IQ(0.25f));
}
static inline float utArctan(float x)
{
	float xx = x * x;
	return ((0.077650957092f * xx + -0.287434475393f) * xx + (M_PI4 - 0.077650957092f - -0.287434475393f)) * x;
}

#endif /* UTILS_H_ */
