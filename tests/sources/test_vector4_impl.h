#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2018 Nicholas Frechette & Animation Compression Library contributors
// Copyright (c) 2018 Nicholas Frechette & Realtime Math contributors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
////////////////////////////////////////////////////////////////////////////////

#include <catch.hpp>

#include <rtm/type_traits.h>
#include <rtm/scalarf.h>
#include <rtm/scalard.h>
#include <rtm/vector4f.h>
#include <rtm/vector4d.h>
#include <rtm/quatf.h>
#include <rtm/quatd.h>

#include <cstring>
#include <limits>

using namespace rtm;

template<typename Vector4Type>
inline Vector4Type vector_unaligned_load_raw(const uint8_t* input);

template<>
inline vector4f vector_unaligned_load_raw<vector4f>(const uint8_t* input)
{
	return vector_unaligned_load(input);
}

template<>
inline vector4d vector_unaligned_load_raw<vector4d>(const uint8_t* input)
{
	return vector_unaligned_load(input);
}

template<typename Vector4Type>
inline Vector4Type vector_unaligned_load3_raw(const uint8_t* input);

template<>
inline vector4f vector_unaligned_load3_raw<vector4f>(const uint8_t* input)
{
	return vector_unaligned_load3(input);
}

template<>
inline vector4d vector_unaligned_load3_raw<vector4d>(const uint8_t* input)
{
	return vector_unaligned_load3(input);
}

template<typename Vector4Type, typename FloatType>
inline const FloatType* vector_as_float_ptr_raw(const Vector4Type& input);

template<>
inline const float* vector_as_float_ptr_raw<vector4f, float>(const vector4f& input)
{
	return vector_to_pointer(input);
}

template<>
inline const double* vector_as_float_ptr_raw<vector4d, double>(const vector4d& input)
{
	return vector_to_pointer(input);
}

template<typename Vector4Type>
inline Vector4Type scalar_cross3(const Vector4Type& lhs, const Vector4Type& rhs)
{
	return vector_set(vector_get_y(lhs) * vector_get_z(rhs) - vector_get_z(lhs) * vector_get_y(rhs),
		vector_get_z(lhs) * vector_get_x(rhs) - vector_get_x(lhs) * vector_get_z(rhs),
		vector_get_x(lhs) * vector_get_y(rhs) - vector_get_y(lhs) * vector_get_x(rhs));
}

template<typename Vector4Type, typename FloatType>
inline FloatType scalar_dot(const Vector4Type& lhs, const Vector4Type& rhs)
{
	return (vector_get_x(lhs) * vector_get_x(rhs)) + (vector_get_y(lhs) * vector_get_y(rhs)) + (vector_get_z(lhs) * vector_get_z(rhs)) + (vector_get_w(lhs) * vector_get_w(rhs));
}

template<typename Vector4Type, typename FloatType>
inline FloatType scalar_dot3(const Vector4Type& lhs, const Vector4Type& rhs)
{
	return (vector_get_x(lhs) * vector_get_x(rhs)) + (vector_get_y(lhs) * vector_get_y(rhs)) + (vector_get_z(lhs) * vector_get_z(rhs));
}

template<typename Vector4Type, typename FloatType>
inline Vector4Type scalar_normalize3(const Vector4Type& input, const Vector4Type& fallback, FloatType threshold)
{
	FloatType len_sq = scalar_dot3<Vector4Type, FloatType>(input, input);
	if (len_sq >= threshold)
	{
		FloatType inv_len = rtm::scalar_sqrt_reciprocal(len_sq);
		return vector_set(vector_get_x(input) * inv_len, vector_get_y(input) * inv_len, vector_get_z(input) * inv_len);
	}
	else
		return fallback;
}

template<typename Vector4Type, mix4 comp0, mix4 comp1, mix4 comp2, mix4 comp3>
inline Vector4Type scalar_mix(const Vector4Type& input0, const Vector4Type& input1)
{
	const auto x = rtm_impl::is_mix_xyzw(comp0) ? vector_get_component<comp0>(input0) : vector_get_component<comp0>(input1);
	const auto y = rtm_impl::is_mix_xyzw(comp1) ? vector_get_component<comp1>(input0) : vector_get_component<comp1>(input1);
	const auto z = rtm_impl::is_mix_xyzw(comp2) ? vector_get_component<comp2>(input0) : vector_get_component<comp2>(input1);
	const auto w = rtm_impl::is_mix_xyzw(comp3) ? vector_get_component<comp3>(input0) : vector_get_component<comp3>(input1);
	return vector_set(x, y, z, w);
}

template<typename FloatType>
void test_vector4_impl(const FloatType threshold)
{
	using QuatType = typename float_traits<FloatType>::quat;
	using Vector4Type = typename float_traits<FloatType>::vector4;
	using ScalarType = typename float_traits<FloatType>::scalar;

	const Vector4Type zero = vector_zero();
	const QuatType identity = quat_identity();

	struct alignas(16) Tmp
	{
		uint8_t padding0[8];	//  8 |  8
		FloatType values[4];	// 24 | 40
		uint8_t padding1[8];	// 32 | 48
	};

	Tmp tmp = { { 0 }, { FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0) }, {} };
	alignas(16) uint8_t buffer[64];

	const FloatType test_value0_flt[4] = { FloatType(2.0), FloatType(9.34), FloatType(-54.12), FloatType(6000.0) };
	const FloatType test_value1_flt[4] = { FloatType(0.75), FloatType(-4.52), FloatType(44.68), FloatType(-54225.0) };
	const FloatType test_value2_flt[4] = { FloatType(-2.65), FloatType(2.996113), FloatType(0.68123521), FloatType(-5.9182) };
	const FloatType test_value3_flt[4] = { FloatType(2.0), FloatType(-9.34), FloatType(54.12), FloatType(6000.1) };
	const Vector4Type test_value0 = vector_set(test_value0_flt[0], test_value0_flt[1], test_value0_flt[2], test_value0_flt[3]);
	const Vector4Type test_value1 = vector_set(test_value1_flt[0], test_value1_flt[1], test_value1_flt[2], test_value1_flt[3]);
	const Vector4Type test_value2 = vector_set(test_value2_flt[0], test_value2_flt[1], test_value2_flt[2], test_value2_flt[3]);
	const Vector4Type test_value3 = vector_set(test_value3_flt[0], test_value3_flt[1], test_value3_flt[2], test_value3_flt[3]);

	//////////////////////////////////////////////////////////////////////////
	// Setters, getters, and casts

	REQUIRE(vector_get_x(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0))) == FloatType(0.0));
	REQUIRE(vector_get_y(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0))) == FloatType(2.34));
	REQUIRE(vector_get_z(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0))) == FloatType(-3.12));
	REQUIRE(vector_get_w(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0))) == FloatType(10000.0));

	REQUIRE(vector_get_x(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12))) == FloatType(0.0));
	REQUIRE(vector_get_y(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12))) == FloatType(2.34));
	REQUIRE(vector_get_z(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12))) == FloatType(-3.12));

	REQUIRE(vector_get_x(vector_set(FloatType(-3.12))) == FloatType(-3.12));
	REQUIRE(vector_get_y(vector_set(FloatType(-3.12))) == FloatType(-3.12));
	REQUIRE(vector_get_z(vector_set(FloatType(-3.12))) == FloatType(-3.12));
	REQUIRE(vector_get_w(vector_set(FloatType(-3.12))) == FloatType(-3.12));

	REQUIRE(vector_get_x(vector_set(scalar_set(FloatType(-3.12)))) == FloatType(-3.12));
	REQUIRE(vector_get_y(vector_set(scalar_set(FloatType(-3.12)))) == FloatType(-3.12));
	REQUIRE(vector_get_z(vector_set(scalar_set(FloatType(-3.12)))) == FloatType(-3.12));
	REQUIRE(vector_get_w(vector_set(scalar_set(FloatType(-3.12)))) == FloatType(-3.12));

	REQUIRE(vector_get_x(zero) == FloatType(0.0));
	REQUIRE(vector_get_y(zero) == FloatType(0.0));
	REQUIRE(vector_get_z(zero) == FloatType(0.0));
	REQUIRE(vector_get_w(zero) == FloatType(0.0));

	REQUIRE(vector_get_x(vector_unaligned_load(&tmp.values[0])) == tmp.values[0]);
	REQUIRE(vector_get_y(vector_unaligned_load(&tmp.values[0])) == tmp.values[1]);
	REQUIRE(vector_get_z(vector_unaligned_load(&tmp.values[0])) == tmp.values[2]);
	REQUIRE(vector_get_w(vector_unaligned_load(&tmp.values[0])) == tmp.values[3]);

	REQUIRE(vector_get_x(vector_unaligned_load3(&tmp.values[0])) == tmp.values[0]);
	REQUIRE(vector_get_y(vector_unaligned_load3(&tmp.values[0])) == tmp.values[1]);
	REQUIRE(vector_get_z(vector_unaligned_load3(&tmp.values[0])) == tmp.values[2]);

	std::memcpy(&buffer[1], &tmp.values[0], sizeof(tmp.values));
	REQUIRE(vector_get_x(vector_unaligned_load_raw<Vector4Type>(&buffer[1])) == tmp.values[0]);
	REQUIRE(vector_get_y(vector_unaligned_load_raw<Vector4Type>(&buffer[1])) == tmp.values[1]);
	REQUIRE(vector_get_z(vector_unaligned_load_raw<Vector4Type>(&buffer[1])) == tmp.values[2]);
	REQUIRE(vector_get_w(vector_unaligned_load_raw<Vector4Type>(&buffer[1])) == tmp.values[3]);

	REQUIRE(vector_get_x(vector_unaligned_load3_raw<Vector4Type>(&buffer[1])) == tmp.values[0]);
	REQUIRE(vector_get_y(vector_unaligned_load3_raw<Vector4Type>(&buffer[1])) == tmp.values[1]);
	REQUIRE(vector_get_z(vector_unaligned_load3_raw<Vector4Type>(&buffer[1])) == tmp.values[2]);

	REQUIRE(vector_get_x(quat_to_vector(identity)) == quat_get_x(identity));
	REQUIRE(vector_get_y(quat_to_vector(identity)) == quat_get_y(identity));
	REQUIRE(vector_get_z(quat_to_vector(identity)) == quat_get_z(identity));
	REQUIRE(vector_get_w(quat_to_vector(identity)) == quat_get_w(identity));

	REQUIRE(vector_get_component<mix4::x>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0))) == FloatType(0.0));
	REQUIRE(vector_get_component<mix4::y>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0))) == FloatType(2.34));
	REQUIRE(vector_get_component<mix4::z>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0))) == FloatType(-3.12));
	REQUIRE(vector_get_component<mix4::w>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0))) == FloatType(10000.0));

	REQUIRE(vector_get_component<mix4::a>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0))) == FloatType(0.0));
	REQUIRE(vector_get_component<mix4::b>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0))) == FloatType(2.34));
	REQUIRE(vector_get_component<mix4::c>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0))) == FloatType(-3.12));
	REQUIRE(vector_get_component<mix4::d>(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0))) == FloatType(10000.0));

	REQUIRE(vector_get_component(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)), mix4::x) == FloatType(0.0));
	REQUIRE(vector_get_component(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)), mix4::y) == FloatType(2.34));
	REQUIRE(vector_get_component(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)), mix4::z) == FloatType(-3.12));
	REQUIRE(vector_get_component(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)), mix4::w) == FloatType(10000.0));

	REQUIRE(vector_get_component(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)), mix4::a) == FloatType(0.0));
	REQUIRE(vector_get_component(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)), mix4::b) == FloatType(2.34));
	REQUIRE(vector_get_component(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)), mix4::c) == FloatType(-3.12));
	REQUIRE(vector_get_component(vector_set(FloatType(0.0), FloatType(2.34), FloatType(-3.12), FloatType(10000.0)), mix4::d) == FloatType(10000.0));

	REQUIRE((vector_as_float_ptr_raw<Vector4Type, FloatType>(vector_unaligned_load(&tmp.values[0]))[0] == tmp.values[0]));
	REQUIRE((vector_as_float_ptr_raw<Vector4Type, FloatType>(vector_unaligned_load(&tmp.values[0]))[1] == tmp.values[1]));
	REQUIRE((vector_as_float_ptr_raw<Vector4Type, FloatType>(vector_unaligned_load(&tmp.values[0]))[2] == tmp.values[2]));
	REQUIRE((vector_as_float_ptr_raw<Vector4Type, FloatType>(vector_unaligned_load(&tmp.values[0]))[3] == tmp.values[3]));

	vector_unaligned_write(test_value0, &tmp.values[0]);
	REQUIRE(vector_get_x(test_value0) == tmp.values[0]);
	REQUIRE(vector_get_y(test_value0) == tmp.values[1]);
	REQUIRE(vector_get_z(test_value0) == tmp.values[2]);
	REQUIRE(vector_get_w(test_value0) == tmp.values[3]);

	vector_unaligned_write3(test_value1, &tmp.values[0]);
	REQUIRE(vector_get_x(test_value1) == tmp.values[0]);
	REQUIRE(vector_get_y(test_value1) == tmp.values[1]);
	REQUIRE(vector_get_z(test_value1) == tmp.values[2]);
	REQUIRE(vector_get_w(test_value0) == tmp.values[3]);

	vector_unaligned_write3(test_value1, &buffer[1]);
	REQUIRE(vector_get_x(test_value1) == vector_get_x(vector_unaligned_load3_raw<Vector4Type>(&buffer[1])));
	REQUIRE(vector_get_y(test_value1) == vector_get_y(vector_unaligned_load3_raw<Vector4Type>(&buffer[1])));
	REQUIRE(vector_get_z(test_value1) == vector_get_z(vector_unaligned_load3_raw<Vector4Type>(&buffer[1])));

	//////////////////////////////////////////////////////////////////////////
	// Arithmetic

	REQUIRE(scalar_near_equal(vector_get_x(vector_add(test_value0, test_value1)), test_value0_flt[0] + test_value1_flt[0], threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_add(test_value0, test_value1)), test_value0_flt[1] + test_value1_flt[1], threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_add(test_value0, test_value1)), test_value0_flt[2] + test_value1_flt[2], threshold));
	REQUIRE(scalar_near_equal(vector_get_w(vector_add(test_value0, test_value1)), test_value0_flt[3] + test_value1_flt[3], threshold));

	REQUIRE(scalar_near_equal(vector_get_x(vector_sub(test_value0, test_value1)), test_value0_flt[0] - test_value1_flt[0], threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_sub(test_value0, test_value1)), test_value0_flt[1] - test_value1_flt[1], threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_sub(test_value0, test_value1)), test_value0_flt[2] - test_value1_flt[2], threshold));
	REQUIRE(scalar_near_equal(vector_get_w(vector_sub(test_value0, test_value1)), test_value0_flt[3] - test_value1_flt[3], threshold));

	REQUIRE(scalar_near_equal(vector_get_x(vector_mul(test_value0, test_value1)), test_value0_flt[0] * test_value1_flt[0], threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_mul(test_value0, test_value1)), test_value0_flt[1] * test_value1_flt[1], threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_mul(test_value0, test_value1)), test_value0_flt[2] * test_value1_flt[2], threshold));
	// We have a strange codegen bug with gcc5, use the Catch near equal impl instead
	REQUIRE(vector_get_w(vector_mul(test_value0, test_value1)) == Approx(test_value0_flt[3] * test_value1_flt[3]).margin(threshold));

	REQUIRE(scalar_near_equal(vector_get_x(vector_mul(test_value0, FloatType(2.34))), test_value0_flt[0] * FloatType(2.34), threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_mul(test_value0, FloatType(2.34))), test_value0_flt[1] * FloatType(2.34), threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_mul(test_value0, FloatType(2.34))), test_value0_flt[2] * FloatType(2.34), threshold));
	REQUIRE(scalar_near_equal(vector_get_w(vector_mul(test_value0, FloatType(2.34))), test_value0_flt[3] * FloatType(2.34), threshold));

	REQUIRE(scalar_near_equal(vector_get_x(vector_div(test_value0, test_value1)), test_value0_flt[0] / test_value1_flt[0], threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_div(test_value0, test_value1)), test_value0_flt[1] / test_value1_flt[1], threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_div(test_value0, test_value1)), test_value0_flt[2] / test_value1_flt[2], threshold));
	REQUIRE(scalar_near_equal(vector_get_w(vector_div(test_value0, test_value1)), test_value0_flt[3] / test_value1_flt[3], threshold));

	REQUIRE(scalar_near_equal(vector_get_x(vector_max(test_value0, test_value1)), scalar_max(test_value0_flt[0], test_value1_flt[0]), threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_max(test_value0, test_value1)), scalar_max(test_value0_flt[1], test_value1_flt[1]), threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_max(test_value0, test_value1)), scalar_max(test_value0_flt[2], test_value1_flt[2]), threshold));
	REQUIRE(scalar_near_equal(vector_get_w(vector_max(test_value0, test_value1)), scalar_max(test_value0_flt[3], test_value1_flt[3]), threshold));

	REQUIRE(scalar_near_equal(vector_get_x(vector_min(test_value0, test_value1)), scalar_min(test_value0_flt[0], test_value1_flt[0]), threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_min(test_value0, test_value1)), scalar_min(test_value0_flt[1], test_value1_flt[1]), threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_min(test_value0, test_value1)), scalar_min(test_value0_flt[2], test_value1_flt[2]), threshold));
	REQUIRE(scalar_near_equal(vector_get_w(vector_min(test_value0, test_value1)), scalar_min(test_value0_flt[3], test_value1_flt[3]), threshold));

	REQUIRE(scalar_near_equal(vector_get_x(vector_clamp(test_value0, test_value1, test_value2)), scalar_clamp(test_value0_flt[0], test_value1_flt[0], test_value2_flt[0]), threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_clamp(test_value0, test_value1, test_value2)), scalar_clamp(test_value0_flt[1], test_value1_flt[1], test_value2_flt[1]), threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_clamp(test_value0, test_value1, test_value2)), scalar_clamp(test_value0_flt[2], test_value1_flt[2], test_value2_flt[2]), threshold));
	REQUIRE(scalar_near_equal(vector_get_w(vector_clamp(test_value0, test_value1, test_value2)), scalar_clamp(test_value0_flt[3], test_value1_flt[3], test_value2_flt[3]), threshold));

	REQUIRE(scalar_near_equal(vector_get_x(vector_abs(test_value0)), scalar_abs(test_value0_flt[0]), threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_abs(test_value0)), scalar_abs(test_value0_flt[1]), threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_abs(test_value0)), scalar_abs(test_value0_flt[2]), threshold));
	REQUIRE(scalar_near_equal(vector_get_w(vector_abs(test_value0)), scalar_abs(test_value0_flt[3]), threshold));

	REQUIRE(scalar_near_equal(vector_get_x(vector_neg(test_value0)), -test_value0_flt[0], threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_neg(test_value0)), -test_value0_flt[1], threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_neg(test_value0)), -test_value0_flt[2], threshold));
	REQUIRE(scalar_near_equal(vector_get_w(vector_neg(test_value0)), -test_value0_flt[3], threshold));

	REQUIRE(scalar_near_equal(vector_get_x(vector_reciprocal(test_value0)), scalar_reciprocal(test_value0_flt[0]), threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_reciprocal(test_value0)), scalar_reciprocal(test_value0_flt[1]), threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_reciprocal(test_value0)), scalar_reciprocal(test_value0_flt[2]), threshold));
	REQUIRE(scalar_near_equal(vector_get_w(vector_reciprocal(test_value0)), scalar_reciprocal(test_value0_flt[3]), threshold));

	REQUIRE(scalar_near_equal(vector_get_x(vector_floor(test_value0)), scalar_floor(test_value0_flt[0]), threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_floor(test_value0)), scalar_floor(test_value0_flt[1]), threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_floor(test_value0)), scalar_floor(test_value0_flt[2]), threshold));
	REQUIRE(scalar_near_equal(vector_get_w(vector_floor(test_value0)), scalar_floor(test_value0_flt[3]), threshold));

	REQUIRE(scalar_near_equal(vector_get_x(vector_ceil(test_value0)), scalar_ceil(test_value0_flt[0]), threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_ceil(test_value0)), scalar_ceil(test_value0_flt[1]), threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_ceil(test_value0)), scalar_ceil(test_value0_flt[2]), threshold));
	REQUIRE(scalar_near_equal(vector_get_w(vector_ceil(test_value0)), scalar_ceil(test_value0_flt[3]), threshold));

	const Vector4Type scalar_cross3_result = scalar_cross3<Vector4Type>(test_value0, test_value1);
	const Vector4Type vector_cross3_result = vector_cross3(test_value0, test_value1);
	REQUIRE(scalar_near_equal(vector_get_x(vector_cross3_result), vector_get_x(scalar_cross3_result), threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_cross3_result), vector_get_y(scalar_cross3_result), threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_cross3_result), vector_get_z(scalar_cross3_result), threshold));

	const FloatType test_value10_flt[4] = { FloatType(-0.001138), FloatType(0.91623), FloatType(-1.624598), FloatType(0.715671) };
	const FloatType test_value11_flt[4] = { FloatType(0.1138), FloatType(-0.623), FloatType(1.4598), FloatType(-0.5671) };
	const Vector4Type test_value10 = vector_set(test_value10_flt[0], test_value10_flt[1], test_value10_flt[2], test_value10_flt[3]);
	const Vector4Type test_value11 = vector_set(test_value11_flt[0], test_value11_flt[1], test_value11_flt[2], test_value11_flt[3]);
	const FloatType scalar_dot_result = scalar_dot<Vector4Type, FloatType>(test_value10, test_value11);
	const FloatType vector_dot_result = vector_dot(test_value10, test_value11);
	REQUIRE(scalar_near_equal(vector_dot_result, scalar_dot_result, threshold));

	const FloatType scalar_dot3_result = scalar_dot3<Vector4Type, FloatType>(test_value10, test_value11);
	const FloatType vector_dot3_result = vector_dot3(test_value10, test_value11);
	REQUIRE(scalar_near_equal(vector_dot3_result, scalar_dot3_result, threshold));

	const ScalarType vector_sdot_result = vector_dot_as_scalar(test_value10, test_value11);
	REQUIRE(scalar_near_equal(scalar_cast(vector_sdot_result), scalar_dot_result, threshold));

	const Vector4Type vector_vdot_result = vector_dot_as_vector(test_value10, test_value11);
	REQUIRE(scalar_near_equal(vector_get_x(vector_vdot_result), scalar_dot_result, threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_vdot_result), scalar_dot_result, threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_vdot_result), scalar_dot_result, threshold));
	REQUIRE(scalar_near_equal(vector_get_w(vector_vdot_result), scalar_dot_result, threshold));

	REQUIRE(scalar_near_equal(scalar_dot<Vector4Type, FloatType>(test_value0, test_value0), vector_length_squared(test_value0), threshold));
	REQUIRE(scalar_near_equal(scalar_dot3<Vector4Type, FloatType>(test_value0, test_value0), vector_length_squared3(test_value0), threshold));

	REQUIRE(scalar_near_equal(rtm::scalar_sqrt(scalar_dot<Vector4Type, FloatType>(test_value0, test_value0)), vector_length(test_value0), threshold));
	REQUIRE(scalar_near_equal(rtm::scalar_sqrt(scalar_dot3<Vector4Type, FloatType>(test_value0, test_value0)), vector_length3(test_value0), threshold));

	REQUIRE(scalar_near_equal(rtm::scalar_sqrt_reciprocal(scalar_dot<Vector4Type, FloatType>(test_value0, test_value0)), vector_length_reciprocal(test_value0), threshold));
	REQUIRE(scalar_near_equal(rtm::scalar_sqrt_reciprocal(scalar_dot3<Vector4Type, FloatType>(test_value0, test_value0)), vector_length_reciprocal3(test_value0), threshold));

	const Vector4Type test_value_diff = vector_sub(test_value0, test_value1);
	REQUIRE(scalar_near_equal(rtm::scalar_sqrt(scalar_dot3<Vector4Type, FloatType>(test_value_diff, test_value_diff)), vector_distance3(test_value0, test_value1), threshold));

	const Vector4Type scalar_normalize3_result = scalar_normalize3<Vector4Type, FloatType>(test_value0, zero, threshold);
	const Vector4Type vector_normalize3_result = vector_normalize3(test_value0, zero, threshold);
	REQUIRE(scalar_near_equal(vector_get_x(vector_normalize3_result), vector_get_x(scalar_normalize3_result), threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_normalize3_result), vector_get_y(scalar_normalize3_result), threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_normalize3_result), vector_get_z(scalar_normalize3_result), threshold));

	const Vector4Type scalar_normalize3_result0 = scalar_normalize3<Vector4Type, FloatType>(zero, zero, threshold);
	const Vector4Type vector_normalize3_result0 = vector_normalize3(zero, zero, threshold);
	REQUIRE(scalar_near_equal(vector_get_x(vector_normalize3_result0), vector_get_x(scalar_normalize3_result0), threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_normalize3_result0), vector_get_y(scalar_normalize3_result0), threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_normalize3_result0), vector_get_z(scalar_normalize3_result0), threshold));

	REQUIRE(scalar_near_equal(vector_get_x(vector_lerp(test_value10, test_value11, FloatType(0.33))), ((test_value11_flt[0] - test_value10_flt[0]) * FloatType(0.33)) + test_value10_flt[0], threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_lerp(test_value10, test_value11, FloatType(0.33))), ((test_value11_flt[1] - test_value10_flt[1]) * FloatType(0.33)) + test_value10_flt[1], threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_lerp(test_value10, test_value11, FloatType(0.33))), ((test_value11_flt[2] - test_value10_flt[2]) * FloatType(0.33)) + test_value10_flt[2], threshold));
	REQUIRE(scalar_near_equal(vector_get_w(vector_lerp(test_value10, test_value11, FloatType(0.33))), ((test_value11_flt[3] - test_value10_flt[3]) * FloatType(0.33)) + test_value10_flt[3], threshold));

	REQUIRE(scalar_near_equal(vector_get_x(vector_fraction(test_value0)), scalar_fraction(test_value0_flt[0]), threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_fraction(test_value0)), scalar_fraction(test_value0_flt[1]), threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_fraction(test_value0)), scalar_fraction(test_value0_flt[2]), threshold));
	REQUIRE(scalar_near_equal(vector_get_w(vector_fraction(test_value0)), scalar_fraction(test_value0_flt[3]), threshold));

	REQUIRE(scalar_near_equal(vector_get_x(vector_mul_add(test_value10, test_value11, test_value2)), (test_value10_flt[0] * test_value11_flt[0]) + test_value2_flt[0], threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_mul_add(test_value10, test_value11, test_value2)), (test_value10_flt[1] * test_value11_flt[1]) + test_value2_flt[1], threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_mul_add(test_value10, test_value11, test_value2)), (test_value10_flt[2] * test_value11_flt[2]) + test_value2_flt[2], threshold));
	REQUIRE(scalar_near_equal(vector_get_w(vector_mul_add(test_value10, test_value11, test_value2)), (test_value10_flt[3] * test_value11_flt[3]) + test_value2_flt[3], threshold));

	REQUIRE(scalar_near_equal(vector_get_x(vector_mul_add(test_value10, test_value11_flt[0], test_value2)), (test_value10_flt[0] * test_value11_flt[0]) + test_value2_flt[0], threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_mul_add(test_value10, test_value11_flt[1], test_value2)), (test_value10_flt[1] * test_value11_flt[1]) + test_value2_flt[1], threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_mul_add(test_value10, test_value11_flt[2], test_value2)), (test_value10_flt[2] * test_value11_flt[2]) + test_value2_flt[2], threshold));
	REQUIRE(scalar_near_equal(vector_get_w(vector_mul_add(test_value10, test_value11_flt[3], test_value2)), (test_value10_flt[3] * test_value11_flt[3]) + test_value2_flt[3], threshold));

	REQUIRE(scalar_near_equal(vector_get_x(vector_neg_mul_sub(test_value10, test_value11, test_value2)), (test_value10_flt[0] * -test_value11_flt[0]) + test_value2_flt[0], threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_neg_mul_sub(test_value10, test_value11, test_value2)), (test_value10_flt[1] * -test_value11_flt[1]) + test_value2_flt[1], threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_neg_mul_sub(test_value10, test_value11, test_value2)), (test_value10_flt[2] * -test_value11_flt[2]) + test_value2_flt[2], threshold));
	REQUIRE(scalar_near_equal(vector_get_w(vector_neg_mul_sub(test_value10, test_value11, test_value2)), (test_value10_flt[3] * -test_value11_flt[3]) + test_value2_flt[3], threshold));

	//////////////////////////////////////////////////////////////////////////
	// Comparisons and masking

	REQUIRE((vector_get_x(vector_less_than(test_value0, test_value1)) != FloatType(0.0)) == (test_value0_flt[0] < test_value1_flt[0]));
	REQUIRE((vector_get_y(vector_less_than(test_value0, test_value1)) != FloatType(0.0)) == (test_value0_flt[1] < test_value1_flt[1]));
	REQUIRE((vector_get_z(vector_less_than(test_value0, test_value1)) != FloatType(0.0)) == (test_value0_flt[2] < test_value1_flt[2]));
	REQUIRE((vector_get_w(vector_less_than(test_value0, test_value1)) != FloatType(0.0)) == (test_value0_flt[3] < test_value1_flt[3]));

	REQUIRE((vector_get_x(vector_less_equal(test_value0, test_value3)) != FloatType(0.0)) == (test_value0_flt[0] <= test_value3_flt[0]));
	REQUIRE((vector_get_y(vector_less_equal(test_value0, test_value3)) != FloatType(0.0)) == (test_value0_flt[1] <= test_value3_flt[1]));
	REQUIRE((vector_get_z(vector_less_equal(test_value0, test_value3)) != FloatType(0.0)) == (test_value0_flt[2] <= test_value3_flt[2]));
	REQUIRE((vector_get_w(vector_less_equal(test_value0, test_value3)) != FloatType(0.0)) == (test_value0_flt[3] <= test_value3_flt[3]));

	REQUIRE((vector_get_x(vector_greater_equal(test_value0, test_value1)) != FloatType(0.0)) == (test_value0_flt[0] >= test_value1_flt[0]));
	REQUIRE((vector_get_y(vector_greater_equal(test_value0, test_value1)) != FloatType(0.0)) == (test_value0_flt[1] >= test_value1_flt[1]));
	REQUIRE((vector_get_z(vector_greater_equal(test_value0, test_value1)) != FloatType(0.0)) == (test_value0_flt[2] >= test_value1_flt[2]));
	REQUIRE((vector_get_w(vector_greater_equal(test_value0, test_value1)) != FloatType(0.0)) == (test_value0_flt[3] >= test_value1_flt[3]));

	REQUIRE(vector_all_less_than(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(1.0))) == true);
	REQUIRE(vector_all_less_than(zero, vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0))) == false);
	REQUIRE(vector_all_less_than(zero, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0))) == false);
	REQUIRE(vector_all_less_than(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0))) == false);
	REQUIRE(vector_all_less_than(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0))) == false);
	REQUIRE(vector_all_less_than(zero, zero) == false);

	REQUIRE(vector_all_less_than3(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(0.0))) == true);
	REQUIRE(vector_all_less_than3(zero, vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0))) == false);
	REQUIRE(vector_all_less_than3(zero, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0))) == false);
	REQUIRE(vector_all_less_than3(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0))) == false);
	REQUIRE(vector_all_less_than3(zero, zero) == false);

	REQUIRE(vector_any_less_than(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(1.0))) == true);
	REQUIRE(vector_any_less_than(zero, vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0))) == true);
	REQUIRE(vector_any_less_than(zero, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0))) == true);
	REQUIRE(vector_any_less_than(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0))) == true);
	REQUIRE(vector_any_less_than(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0))) == true);
	REQUIRE(vector_any_less_than(zero, zero) == false);

	REQUIRE(vector_any_less_than3(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(0.0))) == true);
	REQUIRE(vector_any_less_than3(zero, vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0))) == true);
	REQUIRE(vector_any_less_than3(zero, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0))) == true);
	REQUIRE(vector_any_less_than3(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0))) == true);
	REQUIRE(vector_any_less_than3(zero, zero) == false);

	REQUIRE(vector_all_less_equal(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(1.0))) == true);
	REQUIRE(vector_all_less_equal(zero, vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0))) == true);
	REQUIRE(vector_all_less_equal(zero, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0))) == true);
	REQUIRE(vector_all_less_equal(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0))) == true);
	REQUIRE(vector_all_less_equal(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(1.0))) == true);
	REQUIRE(vector_all_less_equal(zero, vector_set(FloatType(-1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0))) == false);
	REQUIRE(vector_all_less_equal(zero, vector_set(FloatType(0.0), FloatType(-1.0), FloatType(0.0), FloatType(0.0))) == false);
	REQUIRE(vector_all_less_equal(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0), FloatType(0.0))) == false);
	REQUIRE(vector_all_less_equal(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(0.0), FloatType(-1.0))) == false);
	REQUIRE(vector_all_less_equal(zero, zero) == true);

	REQUIRE(vector_all_less_equal3(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(0.0))) == true);
	REQUIRE(vector_all_less_equal3(zero, vector_set(FloatType(1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0))) == true);
	REQUIRE(vector_all_less_equal3(zero, vector_set(FloatType(0.0), FloatType(1.0), FloatType(0.0), FloatType(0.0))) == true);
	REQUIRE(vector_all_less_equal3(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(1.0), FloatType(0.0))) == true);
	REQUIRE(vector_all_less_equal3(zero, vector_set(FloatType(-1.0), FloatType(0.0), FloatType(0.0), FloatType(0.0))) == false);
	REQUIRE(vector_all_less_equal3(zero, vector_set(FloatType(0.0), FloatType(-1.0), FloatType(0.0), FloatType(0.0))) == false);
	REQUIRE(vector_all_less_equal3(zero, vector_set(FloatType(0.0), FloatType(0.0), FloatType(-1.0), FloatType(0.0))) == false);
	REQUIRE(vector_all_less_equal3(zero, zero) == true);

	REQUIRE(vector_any_less_equal(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(1.0))) == true);
	REQUIRE(vector_any_less_equal(zero, vector_set(FloatType(1.0), FloatType(-1.0), FloatType(-1.0), FloatType(-1.0))) == true);
	REQUIRE(vector_any_less_equal(zero, vector_set(FloatType(-1.0), FloatType(1.0), FloatType(-1.0), FloatType(-1.0))) == true);
	REQUIRE(vector_any_less_equal(zero, vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(1.0), FloatType(-1.0))) == true);
	REQUIRE(vector_any_less_equal(zero, vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(1.0))) == true);
	REQUIRE(vector_any_less_equal(zero, vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(-1.0))) == false);
	REQUIRE(vector_any_less_equal(zero, zero) == true);

	REQUIRE(vector_any_less_equal3(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(0.0))) == true);
	REQUIRE(vector_any_less_equal3(zero, vector_set(FloatType(1.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0))) == true);
	REQUIRE(vector_any_less_equal3(zero, vector_set(FloatType(-1.0), FloatType(1.0), FloatType(-1.0), FloatType(0.0))) == true);
	REQUIRE(vector_any_less_equal3(zero, vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(1.0), FloatType(0.0))) == true);
	REQUIRE(vector_any_less_equal3(zero, vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0))) == false);
	REQUIRE(vector_any_less_equal3(zero, zero) == true);

	REQUIRE(vector_all_greater_equal(vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(1.0)), zero) == true);
	REQUIRE(vector_all_greater_equal(vector_set(FloatType(1.0), FloatType(-1.0), FloatType(-1.0), FloatType(-1.0)), zero) == false);
	REQUIRE(vector_all_greater_equal(vector_set(FloatType(-1.0), FloatType(1.0), FloatType(-1.0), FloatType(-1.0)), zero) == false);
	REQUIRE(vector_all_greater_equal(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(1.0), FloatType(-1.0)), zero) == false);
	REQUIRE(vector_all_greater_equal(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(1.0)), zero) == false);
	REQUIRE(vector_all_greater_equal(vector_set(FloatType(0.0), FloatType(-1.0), FloatType(-1.0), FloatType(-1.0)), zero) == false);
	REQUIRE(vector_all_greater_equal(vector_set(FloatType(-1.0), FloatType(0.0), FloatType(-1.0), FloatType(-1.0)), zero) == false);
	REQUIRE(vector_all_greater_equal(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(0.0), FloatType(-1.0)), zero) == false);
	REQUIRE(vector_all_greater_equal(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0)), zero) == false);
	REQUIRE(vector_all_greater_equal(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(-1.0)), zero) == false);
	REQUIRE(vector_all_greater_equal(zero, zero) == true);

	REQUIRE(vector_all_greater_equal3(vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(0.0)), zero) == true);
	REQUIRE(vector_all_greater_equal3(vector_set(FloatType(1.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0)), zero) == false);
	REQUIRE(vector_all_greater_equal3(vector_set(FloatType(-1.0), FloatType(1.0), FloatType(-1.0), FloatType(0.0)), zero) == false);
	REQUIRE(vector_all_greater_equal3(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(1.0), FloatType(0.0)), zero) == false);
	REQUIRE(vector_all_greater_equal3(vector_set(FloatType(0.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0)), zero) == false);
	REQUIRE(vector_all_greater_equal3(vector_set(FloatType(-1.0), FloatType(0.0), FloatType(-1.0), FloatType(0.0)), zero) == false);
	REQUIRE(vector_all_greater_equal3(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(0.0), FloatType(0.0)), zero) == false);
	REQUIRE(vector_all_greater_equal3(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0)), zero) == false);
	REQUIRE(vector_all_greater_equal3(zero, zero) == true);

	REQUIRE(vector_any_greater_equal(vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(1.0)), zero) == true);
	REQUIRE(vector_any_greater_equal(vector_set(FloatType(1.0), FloatType(-1.0), FloatType(-1.0), FloatType(-1.0)), zero) == true);
	REQUIRE(vector_any_greater_equal(vector_set(FloatType(-1.0), FloatType(1.0), FloatType(-1.0), FloatType(-1.0)), zero) == true);
	REQUIRE(vector_any_greater_equal(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(1.0), FloatType(-1.0)), zero) == true);
	REQUIRE(vector_any_greater_equal(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(1.0)), zero) == true);
	REQUIRE(vector_any_greater_equal(vector_set(FloatType(0.0), FloatType(-1.0), FloatType(-1.0), FloatType(-1.0)), zero) == true);
	REQUIRE(vector_any_greater_equal(vector_set(FloatType(-1.0), FloatType(0.0), FloatType(-1.0), FloatType(-1.0)), zero) == true);
	REQUIRE(vector_any_greater_equal(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(0.0), FloatType(-1.0)), zero) == true);
	REQUIRE(vector_any_greater_equal(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0)), zero) == true);
	REQUIRE(vector_any_greater_equal(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(-1.0)), zero) == false);
	REQUIRE(vector_any_greater_equal(zero, zero) == true);

	REQUIRE(vector_any_greater_equal3(vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(0.0)), zero) == true);
	REQUIRE(vector_any_greater_equal3(vector_set(FloatType(1.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0)), zero) == true);
	REQUIRE(vector_any_greater_equal3(vector_set(FloatType(-1.0), FloatType(1.0), FloatType(-1.0), FloatType(0.0)), zero) == true);
	REQUIRE(vector_any_greater_equal3(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(1.0), FloatType(0.0)), zero) == true);
	REQUIRE(vector_any_greater_equal3(vector_set(FloatType(0.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0)), zero) == true);
	REQUIRE(vector_any_greater_equal3(vector_set(FloatType(-1.0), FloatType(0.0), FloatType(-1.0), FloatType(0.0)), zero) == true);
	REQUIRE(vector_any_greater_equal3(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(0.0), FloatType(0.0)), zero) == true);
	REQUIRE(vector_any_greater_equal3(vector_set(FloatType(-1.0), FloatType(-1.0), FloatType(-1.0), FloatType(0.0)), zero) == false);
	REQUIRE(vector_any_greater_equal3(zero, zero) == true);

	REQUIRE(vector_all_near_equal(zero, zero, threshold) == true);
	REQUIRE(vector_all_near_equal(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(1.0)), FloatType(1.0001)) == true);
	REQUIRE(vector_all_near_equal(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(1.0)), FloatType(1.0)) == true);
	REQUIRE(vector_all_near_equal(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(1.0)), FloatType(0.9999)) == false);

	REQUIRE(vector_all_near_equal3(zero, zero, threshold) == true);
	REQUIRE(vector_all_near_equal3(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(2.0)), FloatType(1.0001)) == true);
	REQUIRE(vector_all_near_equal3(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(2.0)), FloatType(1.0)) == true);
	REQUIRE(vector_all_near_equal3(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(2.0)), FloatType(0.9999)) == false);

	REQUIRE(vector_any_near_equal(zero, zero, threshold) == true);
	REQUIRE(vector_any_near_equal(zero, vector_set(FloatType(1.0), FloatType(2.0), FloatType(2.0), FloatType(2.0)), FloatType(1.0001)) == true);
	REQUIRE(vector_any_near_equal(zero, vector_set(FloatType(2.0), FloatType(1.0), FloatType(2.0), FloatType(2.0)), FloatType(1.0001)) == true);
	REQUIRE(vector_any_near_equal(zero, vector_set(FloatType(2.0), FloatType(2.0), FloatType(1.0), FloatType(2.0)), FloatType(1.0001)) == true);
	REQUIRE(vector_any_near_equal(zero, vector_set(FloatType(2.0), FloatType(2.0), FloatType(2.0), FloatType(1.0)), FloatType(1.0001)) == true);
	REQUIRE(vector_any_near_equal(zero, vector_set(FloatType(1.0), FloatType(2.0), FloatType(2.0), FloatType(2.0)), FloatType(1.0)) == true);
	REQUIRE(vector_any_near_equal(zero, vector_set(FloatType(2.0), FloatType(1.0), FloatType(2.0), FloatType(2.0)), FloatType(1.0)) == true);
	REQUIRE(vector_any_near_equal(zero, vector_set(FloatType(2.0), FloatType(2.0), FloatType(1.0), FloatType(2.0)), FloatType(1.0)) == true);
	REQUIRE(vector_any_near_equal(zero, vector_set(FloatType(2.0), FloatType(2.0), FloatType(2.0), FloatType(1.0)), FloatType(1.0)) == true);
	REQUIRE(vector_any_near_equal(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(1.0)), FloatType(0.9999)) == false);

	REQUIRE(vector_any_near_equal3(zero, zero, threshold) == true);
	REQUIRE(vector_any_near_equal3(zero, vector_set(FloatType(1.0), FloatType(2.0), FloatType(2.0), FloatType(2.0)), FloatType(1.0001)) == true);
	REQUIRE(vector_any_near_equal3(zero, vector_set(FloatType(2.0), FloatType(1.0), FloatType(2.0), FloatType(2.0)), FloatType(1.0001)) == true);
	REQUIRE(vector_any_near_equal3(zero, vector_set(FloatType(2.0), FloatType(2.0), FloatType(1.0), FloatType(2.0)), FloatType(1.0001)) == true);
	REQUIRE(vector_any_near_equal3(zero, vector_set(FloatType(1.0), FloatType(2.0), FloatType(2.0), FloatType(2.0)), FloatType(1.0)) == true);
	REQUIRE(vector_any_near_equal3(zero, vector_set(FloatType(2.0), FloatType(1.0), FloatType(2.0), FloatType(2.0)), FloatType(1.0)) == true);
	REQUIRE(vector_any_near_equal3(zero, vector_set(FloatType(2.0), FloatType(2.0), FloatType(1.0), FloatType(2.0)), FloatType(1.0)) == true);
	REQUIRE(vector_any_near_equal3(zero, vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(2.0)), FloatType(0.9999)) == false);

	const FloatType inf = std::numeric_limits<FloatType>::infinity();
	const FloatType nan = std::numeric_limits<FloatType>::quiet_NaN();
	REQUIRE(vector_is_finite(zero) == true);
	REQUIRE(vector_is_finite(vector_set(inf, inf, inf, inf)) == false);
	REQUIRE(vector_is_finite(vector_set(inf, FloatType(1.0), FloatType(1.0), FloatType(1.0))) == false);
	REQUIRE(vector_is_finite(vector_set(FloatType(1.0), FloatType(inf), FloatType(1.0), FloatType(1.0))) == false);
	REQUIRE(vector_is_finite(vector_set(FloatType(1.0), FloatType(1.0), FloatType(inf), FloatType(1.0))) == false);
	REQUIRE(vector_is_finite(vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(inf))) == false);
	REQUIRE(vector_is_finite(vector_set(nan, nan, nan, nan)) == false);
	REQUIRE(vector_is_finite(vector_set(nan, FloatType(1.0), FloatType(1.0), FloatType(1.0))) == false);
	REQUIRE(vector_is_finite(vector_set(FloatType(1.0), FloatType(nan), FloatType(1.0), FloatType(1.0))) == false);
	REQUIRE(vector_is_finite(vector_set(FloatType(1.0), FloatType(1.0), FloatType(nan), FloatType(1.0))) == false);
	REQUIRE(vector_is_finite(vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(nan))) == false);

	REQUIRE(vector_is_finite3(zero) == true);
	REQUIRE(vector_is_finite3(vector_set(inf, inf, inf, inf)) == false);
	REQUIRE(vector_is_finite3(vector_set(inf, FloatType(1.0), FloatType(1.0), FloatType(1.0))) == false);
	REQUIRE(vector_is_finite3(vector_set(FloatType(1.0), FloatType(inf), FloatType(1.0), FloatType(1.0))) == false);
	REQUIRE(vector_is_finite3(vector_set(FloatType(1.0), FloatType(1.0), FloatType(inf), FloatType(1.0))) == false);
	REQUIRE(vector_is_finite3(vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(inf))) == true);
	REQUIRE(vector_is_finite3(vector_set(nan, nan, nan, nan)) == false);
	REQUIRE(vector_is_finite3(vector_set(nan, FloatType(1.0), FloatType(1.0), FloatType(1.0))) == false);
	REQUIRE(vector_is_finite3(vector_set(FloatType(1.0), FloatType(nan), FloatType(1.0), FloatType(1.0))) == false);
	REQUIRE(vector_is_finite3(vector_set(FloatType(1.0), FloatType(1.0), FloatType(nan), FloatType(1.0))) == false);
	REQUIRE(vector_is_finite3(vector_set(FloatType(1.0), FloatType(1.0), FloatType(1.0), FloatType(nan))) == true);

	//////////////////////////////////////////////////////////////////////////
	// Swizzling, permutations, and mixing

	REQUIRE(scalar_near_equal(vector_get_x(vector_select(vector_less_than(zero, vector_set(FloatType(1.0))), test_value0, test_value1)), test_value0_flt[0], threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_select(vector_less_than(zero, vector_set(FloatType(1.0))), test_value0, test_value1)), test_value0_flt[1], threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_select(vector_less_than(zero, vector_set(FloatType(1.0))), test_value0, test_value1)), test_value0_flt[2], threshold));
	REQUIRE(scalar_near_equal(vector_get_w(vector_select(vector_less_than(zero, vector_set(FloatType(1.0))), test_value0, test_value1)), test_value0_flt[3], threshold));
	REQUIRE(scalar_near_equal(vector_get_x(vector_select(vector_less_than(vector_set(FloatType(1.0)), zero), test_value0, test_value1)), test_value1_flt[0], threshold));
	REQUIRE(scalar_near_equal(vector_get_y(vector_select(vector_less_than(vector_set(FloatType(1.0)), zero), test_value0, test_value1)), test_value1_flt[1], threshold));
	REQUIRE(scalar_near_equal(vector_get_z(vector_select(vector_less_than(vector_set(FloatType(1.0)), zero), test_value0, test_value1)), test_value1_flt[2], threshold));
	REQUIRE(scalar_near_equal(vector_get_w(vector_select(vector_less_than(vector_set(FloatType(1.0)), zero), test_value0, test_value1)), test_value1_flt[3], threshold));

	//////////////////////////////////////////////////////////////////////////
	// Misc

	auto scalar_sign = [](FloatType value) { return value >= FloatType(0.0) ? FloatType(1.0) : FloatType(-1.0); };
	REQUIRE(vector_get_x(vector_sign(test_value0)) == scalar_sign(test_value0_flt[0]));
	REQUIRE(vector_get_y(vector_sign(test_value0)) == scalar_sign(test_value0_flt[1]));
	REQUIRE(vector_get_z(vector_sign(test_value0)) == scalar_sign(test_value0_flt[2]));
	REQUIRE(vector_get_w(vector_sign(test_value0)) == scalar_sign(test_value0_flt[3]));
}

template<typename Vector4Type, typename FloatType, mix4 XArg>
void test_vector_mix_impl(const FloatType threshold)
{
#define RTM_TEST_MIX_XYZ(results, input0, input1, comp0, comp1, comp2) \
	results[(int)comp0][(int)comp1][(int)comp2][(int)mix4::x] = vector_all_near_equal(vector_mix<comp0, comp1, comp2, mix4::x>(input0, input1), scalar_mix<Vector4Type, comp0, comp1, comp2, mix4::x>(input0, input1), threshold); \
	results[(int)comp0][(int)comp1][(int)comp2][(int)mix4::y] = vector_all_near_equal(vector_mix<comp0, comp1, comp2, mix4::y>(input0, input1), scalar_mix<Vector4Type, comp0, comp1, comp2, mix4::y>(input0, input1), threshold); \
	results[(int)comp0][(int)comp1][(int)comp2][(int)mix4::z] = vector_all_near_equal(vector_mix<comp0, comp1, comp2, mix4::z>(input0, input1), scalar_mix<Vector4Type, comp0, comp1, comp2, mix4::z>(input0, input1), threshold); \
	results[(int)comp0][(int)comp1][(int)comp2][(int)mix4::w] = vector_all_near_equal(vector_mix<comp0, comp1, comp2, mix4::w>(input0, input1), scalar_mix<Vector4Type, comp0, comp1, comp2, mix4::w>(input0, input1), threshold); \
	results[(int)comp0][(int)comp1][(int)comp2][(int)mix4::a] = vector_all_near_equal(vector_mix<comp0, comp1, comp2, mix4::a>(input0, input1), scalar_mix<Vector4Type, comp0, comp1, comp2, mix4::a>(input0, input1), threshold); \
	results[(int)comp0][(int)comp1][(int)comp2][(int)mix4::b] = vector_all_near_equal(vector_mix<comp0, comp1, comp2, mix4::b>(input0, input1), scalar_mix<Vector4Type, comp0, comp1, comp2, mix4::b>(input0, input1), threshold); \
	results[(int)comp0][(int)comp1][(int)comp2][(int)mix4::c] = vector_all_near_equal(vector_mix<comp0, comp1, comp2, mix4::c>(input0, input1), scalar_mix<Vector4Type, comp0, comp1, comp2, mix4::c>(input0, input1), threshold); \
	results[(int)comp0][(int)comp1][(int)comp2][(int)mix4::d] = vector_all_near_equal(vector_mix<comp0, comp1, comp2, mix4::d>(input0, input1), scalar_mix<Vector4Type, comp0, comp1, comp2, mix4::d>(input0, input1), threshold)

#define RTM_TEST_MIX_XY(results, input0, input1, comp0, comp1) \
	RTM_TEST_MIX_XYZ(results, input0, input1, comp0, comp1, mix4::x); \
	RTM_TEST_MIX_XYZ(results, input0, input1, comp0, comp1, mix4::y); \
	RTM_TEST_MIX_XYZ(results, input0, input1, comp0, comp1, mix4::z); \
	RTM_TEST_MIX_XYZ(results, input0, input1, comp0, comp1, mix4::w); \
	RTM_TEST_MIX_XYZ(results, input0, input1, comp0, comp1, mix4::a); \
	RTM_TEST_MIX_XYZ(results, input0, input1, comp0, comp1, mix4::b); \
	RTM_TEST_MIX_XYZ(results, input0, input1, comp0, comp1, mix4::c); \
	RTM_TEST_MIX_XYZ(results, input0, input1, comp0, comp1, mix4::d)

#define RTM_TEST_MIX_X(results, input0, input1, comp0) \
	RTM_TEST_MIX_XY(results, input0, input1, comp0, mix4::x); \
	RTM_TEST_MIX_XY(results, input0, input1, comp0, mix4::y); \
	RTM_TEST_MIX_XY(results, input0, input1, comp0, mix4::z); \
	RTM_TEST_MIX_XY(results, input0, input1, comp0, mix4::w); \
	RTM_TEST_MIX_XY(results, input0, input1, comp0, mix4::a); \
	RTM_TEST_MIX_XY(results, input0, input1, comp0, mix4::b); \
	RTM_TEST_MIX_XY(results, input0, input1, comp0, mix4::c); \
	RTM_TEST_MIX_XY(results, input0, input1, comp0, mix4::d)

	// This generates 8*8*8*8 = 4096 unit tests... it takes a while to compile and uses a lot of stack space
	// Disabled by default to reduce the build time
	bool vector_mix_results[8][8][8][8];

	const FloatType test_value0_flt[4] = { FloatType(2.0), FloatType(9.34), FloatType(-54.12), FloatType(6000.0) };
	const FloatType test_value1_flt[4] = { FloatType(0.75), FloatType(-4.52), FloatType(44.68), FloatType(-54225.0) };
	const Vector4Type test_value0 = vector_set(test_value0_flt[0], test_value0_flt[1], test_value0_flt[2], test_value0_flt[3]);
	const Vector4Type test_value1 = vector_set(test_value1_flt[0], test_value1_flt[1], test_value1_flt[2], test_value1_flt[3]);

	RTM_TEST_MIX_X(vector_mix_results, test_value0, test_value1, XArg);

	const int comp0 = (int)XArg;
	for (int comp1 = 0; comp1 < 8; ++comp1)
	{
		for (int comp2 = 0; comp2 < 8; ++comp2)
		{
			for (int comp3 = 0; comp3 < 8; ++comp3)
			{
				INFO("vector_mix<" << comp0 << ", " << comp1 << ", " << comp2 << ", " << comp3 << ">");
				REQUIRE(vector_mix_results[comp0][comp1][comp2][comp3] == true);
			}
		}
	}

#undef RTM_TEST_MIX_X
#undef RTM_TEST_MIX_XY
#undef RTM_TEST_MIX_XYZ
}
