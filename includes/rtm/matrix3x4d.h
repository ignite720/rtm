#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2017 Nicholas Frechette & Animation Compression Library contributors
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

#include "rtm/math.h"
#include "rtm/vector4d.h"
#include "rtm/quatd.h"
#include "rtm/impl/matrix_common.h"
#include "rtm/impl/matrix_affine_common.h"

namespace rtm
{
	//////////////////////////////////////////////////////////////////////////
	// Converts a translation vector into a 3x4 affine matrix.
	//////////////////////////////////////////////////////////////////////////
	inline matrix3x4d matrix_from_translation(const vector4d& translation) RTM_NO_EXCEPT
	{
		return matrix_set(vector_set(1.0, 0.0, 0.0, 0.0), vector_set(0.0, 1.0, 0.0, 0.0), vector_set(0.0, 0.0, 1.0, 0.0), vector_set(vector_get_x(translation), vector_get_y(translation), vector_get_z(translation), 1.0));
	}

	//////////////////////////////////////////////////////////////////////////
	// Sets a 3x4 affine matrix from a rotation quaternion, translation, and 3D scale.
	//////////////////////////////////////////////////////////////////////////
	inline matrix3x4d matrix_from_qvv(const quatd& quat, const vector4d& translation, const vector4d& scale) RTM_NO_EXCEPT
	{
		RTM_ASSERT(quat_is_normalized(quat), "Quaternion is not normalized");

		const double x2 = quat_get_x(quat) + quat_get_x(quat);
		const double y2 = quat_get_y(quat) + quat_get_y(quat);
		const double z2 = quat_get_z(quat) + quat_get_z(quat);
		const double xx = quat_get_x(quat) * x2;
		const double xy = quat_get_x(quat) * y2;
		const double xz = quat_get_x(quat) * z2;
		const double yy = quat_get_y(quat) * y2;
		const double yz = quat_get_y(quat) * z2;
		const double zz = quat_get_z(quat) * z2;
		const double wx = quat_get_w(quat) * x2;
		const double wy = quat_get_w(quat) * y2;
		const double wz = quat_get_w(quat) * z2;

		vector4d x_axis = vector_mul(vector_set(1.0 - (yy + zz), xy + wz, xz - wy, 0.0), vector_get_x(scale));
		vector4d y_axis = vector_mul(vector_set(xy - wz, 1.0 - (xx + zz), yz + wx, 0.0), vector_get_y(scale));
		vector4d z_axis = vector_mul(vector_set(xz + wy, yz - wx, 1.0 - (xx + yy), 0.0), vector_get_z(scale));
		vector4d w_axis = vector_set(vector_get_x(translation), vector_get_y(translation), vector_get_z(translation), 1.0);
		return matrix_set(x_axis, y_axis, z_axis, w_axis);
	}

	//////////////////////////////////////////////////////////////////////////
	// Converts a QVV transform into a 3x4 affine matrix.
	//////////////////////////////////////////////////////////////////////////
	inline matrix3x4d matrix_from_qvv(const qvvd& transform) RTM_NO_EXCEPT
	{
		return matrix_from_qvv(transform.rotation, transform.translation, transform.scale);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the desired 3x4 affine matrix axis.
	//////////////////////////////////////////////////////////////////////////
	inline const vector4d& matrix_get_axis(const matrix3x4d& input, axis4 axis) RTM_NO_EXCEPT
	{
		switch (axis)
		{
		case axis4::x: return input.x_axis;
		case axis4::y: return input.y_axis;
		case axis4::z: return input.z_axis;
		case axis4::w: return input.w_axis;
		default:
			RTM_ASSERT(false, "Invalid matrix axis");
			return input.x_axis;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Converts a 3x4 affine matrix into a rotation quaternion.
	//////////////////////////////////////////////////////////////////////////
	inline quatd quat_from_matrix(const matrix3x4d& input) RTM_NO_EXCEPT
	{
		return rtm_impl::quat_from_matrix(input.x_axis, input.y_axis, input.z_axis);
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies two 3x4 affine matrices.
	// Multiplication order is as follow: local_to_world = matrix_mul(local_to_object, object_to_world)
	//////////////////////////////////////////////////////////////////////////
	inline matrix3x4d matrix_mul(const matrix3x4d& lhs, const matrix3x4d& rhs) RTM_NO_EXCEPT
	{
		vector4d tmp = vector_mul(vector_dup_x(lhs.x_axis), rhs.x_axis);
		tmp = vector_mul_add(vector_dup_y(lhs.x_axis), rhs.y_axis, tmp);
		tmp = vector_mul_add(vector_dup_z(lhs.x_axis), rhs.z_axis, tmp);
		vector4d x_axis = tmp;

		tmp = vector_mul(vector_dup_x(lhs.y_axis), rhs.x_axis);
		tmp = vector_mul_add(vector_dup_y(lhs.y_axis), rhs.y_axis, tmp);
		tmp = vector_mul_add(vector_dup_z(lhs.y_axis), rhs.z_axis, tmp);
		vector4d y_axis = tmp;

		tmp = vector_mul(vector_dup_x(lhs.z_axis), rhs.x_axis);
		tmp = vector_mul_add(vector_dup_y(lhs.z_axis), rhs.y_axis, tmp);
		tmp = vector_mul_add(vector_dup_z(lhs.z_axis), rhs.z_axis, tmp);
		vector4d z_axis = tmp;

		tmp = vector_mul(vector_dup_x(lhs.w_axis), rhs.x_axis);
		tmp = vector_mul_add(vector_dup_y(lhs.w_axis), rhs.y_axis, tmp);
		tmp = vector_mul_add(vector_dup_z(lhs.w_axis), rhs.z_axis, tmp);
		vector4d w_axis = vector_add(rhs.w_axis, tmp);
		return matrix_set(x_axis, y_axis, z_axis, w_axis);
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies a 3x4 affine matrix and a 3D point.
	// Multiplication order is as follow: world_position = matrix_mul(local_position, local_to_world)
	//////////////////////////////////////////////////////////////////////////
	inline vector4d matrix_mul_point3(const vector4d& point, const matrix3x4d& mtx) RTM_NO_EXCEPT
	{
		vector4d tmp0;
		vector4d tmp1;

		tmp0 = vector_mul(vector_dup_x(point), mtx.x_axis);
		tmp0 = vector_mul_add(vector_dup_y(point), mtx.y_axis, tmp0);
		tmp1 = vector_mul_add(vector_dup_z(point), mtx.z_axis, mtx.w_axis);

		return vector_add(tmp0, tmp1);
	}

	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x4 affine matrix.
	// Note: This is a generic matrix 4x4 transpose, the resulting matrix is no longer
	// affine because the last row is no longer [0,0,0,1]
	//////////////////////////////////////////////////////////////////////////
	inline matrix4x4d matrix_transpose(const matrix3x4d& input) RTM_NO_EXCEPT
	{
		const vector4d v00_v01_v10_v11 = vector_mix<mix4::x, mix4::y, mix4::a, mix4::b>(input.x_axis, input.y_axis);
		const vector4d v02_v03_v12_v13 = vector_mix<mix4::z, mix4::w, mix4::c, mix4::d>(input.x_axis, input.y_axis);
		const vector4d v20_v21_v30_v31 = vector_mix<mix4::x, mix4::y, mix4::a, mix4::b>(input.z_axis, input.w_axis);
		const vector4d v22_v23_v32_v33 = vector_mix<mix4::z, mix4::w, mix4::c, mix4::d>(input.z_axis, input.w_axis);

		const vector4d x_axis = vector_mix<mix4::x, mix4::z, mix4::a, mix4::c>(v00_v01_v10_v11, v20_v21_v30_v31);
		const vector4d y_axis = vector_mix<mix4::y, mix4::w, mix4::b, mix4::d>(v00_v01_v10_v11, v20_v21_v30_v31);
		const vector4d z_axis = vector_mix<mix4::x, mix4::z, mix4::a, mix4::c>(v02_v03_v12_v13, v22_v23_v32_v33);
		const vector4d w_axis = vector_mix<mix4::y, mix4::w, mix4::b, mix4::d>(v02_v03_v12_v13, v22_v23_v32_v33);
		return matrix4x4d{ x_axis, y_axis, z_axis, w_axis };
	}

	//////////////////////////////////////////////////////////////////////////
	// Inverses a 3x4 affine matrix.
	// TODO: This is a generic matrix inverse function, can we do better?
	//////////////////////////////////////////////////////////////////////////
	inline matrix3x4d matrix_inverse(const matrix3x4d& input) RTM_NO_EXCEPT
	{
		matrix4x4d input_transposed = matrix_transpose(input);

		vector4d v00 = vector_mix<mix4::x, mix4::x, mix4::y, mix4::y>(input_transposed.z_axis, input_transposed.z_axis);
		vector4d v01 = vector_mix<mix4::x, mix4::x, mix4::y, mix4::y>(input_transposed.x_axis, input_transposed.x_axis);
		vector4d v02 = vector_mix<mix4::x, mix4::z, mix4::a, mix4::c>(input_transposed.z_axis, input_transposed.x_axis);
		vector4d v10 = vector_mix<mix4::z, mix4::w, mix4::z, mix4::w>(input_transposed.w_axis, input_transposed.w_axis);
		vector4d v11 = vector_mix<mix4::z, mix4::w, mix4::z, mix4::w>(input_transposed.y_axis, input_transposed.y_axis);
		vector4d v12 = vector_mix<mix4::y, mix4::w, mix4::b, mix4::d>(input_transposed.w_axis, input_transposed.y_axis);

		vector4d d0 = vector_mul(v00, v10);
		vector4d d1 = vector_mul(v01, v11);
		vector4d d2 = vector_mul(v02, v12);

		v00 = vector_mix<mix4::z, mix4::w, mix4::z, mix4::w>(input_transposed.z_axis, input_transposed.z_axis);
		v01 = vector_mix<mix4::z, mix4::w, mix4::z, mix4::w>(input_transposed.x_axis, input_transposed.x_axis);
		v02 = vector_mix<mix4::y, mix4::w, mix4::b, mix4::d>(input_transposed.z_axis, input_transposed.x_axis);
		v10 = vector_mix<mix4::x, mix4::x, mix4::y, mix4::y>(input_transposed.w_axis, input_transposed.w_axis);
		v11 = vector_mix<mix4::x, mix4::x, mix4::y, mix4::y>(input_transposed.y_axis, input_transposed.y_axis);
		v12 = vector_mix<mix4::x, mix4::z, mix4::a, mix4::c>(input_transposed.w_axis, input_transposed.y_axis);

		d0 = vector_neg_mul_sub(v00, v10, d0);
		d1 = vector_neg_mul_sub(v01, v11, d1);
		d2 = vector_neg_mul_sub(v02, v12, d2);

		v00 = vector_mix<mix4::y, mix4::z, mix4::x, mix4::y>(input_transposed.y_axis, input_transposed.y_axis);
		v01 = vector_mix<mix4::z, mix4::x, mix4::y, mix4::x>(input_transposed.x_axis, input_transposed.x_axis);
		v02 = vector_mix<mix4::y, mix4::z, mix4::x, mix4::y>(input_transposed.w_axis, input_transposed.w_axis);
		vector4d v03 = vector_mix<mix4::z, mix4::x, mix4::y, mix4::x>(input_transposed.z_axis, input_transposed.z_axis);
		v10 = vector_mix<mix4::b, mix4::y, mix4::w, mix4::x>(d0, d2);
		v11 = vector_mix<mix4::w, mix4::b, mix4::y, mix4::z>(d0, d2);
		v12 = vector_mix<mix4::d, mix4::y, mix4::w, mix4::x>(d1, d2);
		vector4d v13 = vector_mix<mix4::w, mix4::d, mix4::y, mix4::z>(d1, d2);

		vector4d c0 = vector_mul(v00, v10);
		vector4d c2 = vector_mul(v01, v11);
		vector4d c4 = vector_mul(v02, v12);
		vector4d c6 = vector_mul(v03, v13);

		v00 = vector_mix<mix4::z, mix4::w, mix4::y, mix4::z>(input_transposed.y_axis, input_transposed.y_axis);
		v01 = vector_mix<mix4::w, mix4::z, mix4::w, mix4::y>(input_transposed.x_axis, input_transposed.x_axis);
		v02 = vector_mix<mix4::z, mix4::w, mix4::y, mix4::z>(input_transposed.w_axis, input_transposed.w_axis);
		v03 = vector_mix<mix4::w, mix4::z, mix4::w, mix4::y>(input_transposed.z_axis, input_transposed.z_axis);
		v10 = vector_mix<mix4::w, mix4::x, mix4::y, mix4::a>(d0, d2);
		v11 = vector_mix<mix4::z, mix4::y, mix4::a, mix4::x>(d0, d2);
		v12 = vector_mix<mix4::w, mix4::x, mix4::y, mix4::c>(d1, d2);
		v13 = vector_mix<mix4::z, mix4::y, mix4::c, mix4::x>(d1, d2);

		c0 = vector_neg_mul_sub(v00, v10, c0);
		c2 = vector_neg_mul_sub(v01, v11, c2);
		c4 = vector_neg_mul_sub(v02, v12, c4);
		c6 = vector_neg_mul_sub(v03, v13, c6);

		v00 = vector_mix<mix4::w, mix4::x, mix4::w, mix4::x>(input_transposed.y_axis, input_transposed.y_axis);
		v01 = vector_mix<mix4::y, mix4::w, mix4::x, mix4::z>(input_transposed.x_axis, input_transposed.x_axis);
		v02 = vector_mix<mix4::w, mix4::x, mix4::w, mix4::x>(input_transposed.w_axis, input_transposed.w_axis);
		v03 = vector_mix<mix4::y, mix4::w, mix4::x, mix4::z>(input_transposed.z_axis, input_transposed.z_axis);
		v10 = vector_mix<mix4::z, mix4::b, mix4::a, mix4::z>(d0, d2);
		v11 = vector_mix<mix4::b, mix4::x, mix4::w, mix4::a>(d0, d2);
		v12 = vector_mix<mix4::z, mix4::d, mix4::c, mix4::z>(d1, d2);
		v13 = vector_mix<mix4::d, mix4::x, mix4::w, mix4::c>(d1, d2);

		vector4d c1 = vector_neg_mul_sub(v00, v10, c0);
		c0 = vector_mul_add(v00, v10, c0);
		vector4d c3 = vector_mul_add(v01, v11, c2);
		c2 = vector_neg_mul_sub(v01, v11, c2);
		vector4d c5 = vector_neg_mul_sub(v02, v12, c4);
		c4 = vector_mul_add(v02, v12, c4);
		vector4d c7 = vector_mul_add(v03, v13, c6);
		c6 = vector_neg_mul_sub(v03, v13, c6);

		vector4d x_axis = vector_mix<mix4::x, mix4::b, mix4::z, mix4::d>(c0, c1);
		vector4d y_axis = vector_mix<mix4::x, mix4::b, mix4::z, mix4::d>(c2, c3);
		vector4d z_axis = vector_mix<mix4::x, mix4::b, mix4::z, mix4::d>(c4, c5);
		vector4d w_axis = vector_mix<mix4::x, mix4::b, mix4::z, mix4::d>(c6, c7);

		double det = vector_dot(x_axis, input_transposed.x_axis);
		vector4d inv_det = vector_set(scalar_reciprocal(det));

		x_axis = vector_mul(x_axis, inv_det);
		y_axis = vector_mul(y_axis, inv_det);
		z_axis = vector_mul(z_axis, inv_det);
		w_axis = vector_mul(w_axis, inv_det);

#if defined(RTM_HAS_ASSERT_CHECKS)
		// Due to floating point noise we might have 0,0,0,~1 but not quite 0,0,0,1
		// We only really care if we assert
		w_axis = vector_set(vector_get_x(w_axis), vector_get_y(w_axis), vector_get_z(w_axis), 1.0);
#endif

		return matrix_set(x_axis, y_axis, z_axis, w_axis);
	}

	//////////////////////////////////////////////////////////////////////////
	// Removes the 3D scale from a 3x4 affine matrix.
	// Note that if the scaling is 0.0 for a particular axis, the original rotation axis cannot
	// be recovered trivially and no attempt is done to do so. In theory, we could use the other axes
	// to try and recover it.
	// TODO: Implement rotation recovering, perhaps in a separate function and rename this
	// one to matrix_remove_non_zero_scale(..)
	//////////////////////////////////////////////////////////////////////////
	inline matrix3x4d matrix_remove_scale(const matrix3x4d& input) RTM_NO_EXCEPT
	{
		matrix3x4d result;
		result.x_axis = vector_normalize3(input.x_axis, input.x_axis);
		result.y_axis = vector_normalize3(input.y_axis, input.y_axis);
		result.z_axis = vector_normalize3(input.z_axis, input.z_axis);
		result.w_axis = input.w_axis;
		return result;
	}
}
