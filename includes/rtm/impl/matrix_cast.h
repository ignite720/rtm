#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
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
#include "rtm/vector4f.h"
#include "rtm/vector4d.h"

namespace rtm
{
	namespace rtm_impl
	{
		//////////////////////////////////////////////////////////////////////////
		// A helper struct to cast matrices with similar width.
		//////////////////////////////////////////////////////////////////////////
		template<typename src_matrix_type>
		struct matrix_caster {};

		template<>
		struct matrix_caster<matrix3x3f>
		{
			constexpr explicit matrix_caster(const matrix3x3f& mtx_) RTM_NO_EXCEPT : mtx(mtx_) {}

			constexpr RTM_SIMD_CALL operator matrix3x3f() const RTM_NO_EXCEPT
			{
				return mtx;
			}

			inline RTM_SIMD_CALL operator matrix3x3d() const RTM_NO_EXCEPT
			{
				return matrix3x3d{ vector_cast(mtx.x_axis), vector_cast(mtx.y_axis), vector_cast(mtx.z_axis) };
			}

			inline RTM_SIMD_CALL operator matrix3x4f() const RTM_NO_EXCEPT
			{
				RTM_ASSERT(vector_get_w(mtx.x_axis) == 0.0f, "X axis does not have a W component == 0.0");
				RTM_ASSERT(vector_get_w(mtx.y_axis) == 0.0f, "Y axis does not have a W component == 0.0");
				RTM_ASSERT(vector_get_w(mtx.z_axis) == 0.0f, "Z axis does not have a W component == 0.0");
				return matrix3x4f{ mtx.x_axis, mtx.y_axis, mtx.z_axis, vector_set(0.0f, 0.0f, 0.0f, 1.0f) };
			}

			inline RTM_SIMD_CALL operator matrix3x4d() const RTM_NO_EXCEPT
			{
				RTM_ASSERT(vector_get_w(mtx.x_axis) == 0.0f, "X axis does not have a W component == 0.0");
				RTM_ASSERT(vector_get_w(mtx.y_axis) == 0.0f, "Y axis does not have a W component == 0.0");
				RTM_ASSERT(vector_get_w(mtx.z_axis) == 0.0f, "Z axis does not have a W component == 0.0");
				return matrix3x4d{ vector_cast(mtx.x_axis), vector_cast(mtx.y_axis), vector_cast(mtx.z_axis), vector_set(0.0, 0.0, 0.0, 1.0) };
			}

			constexpr RTM_SIMD_CALL operator matrix4x4f() const RTM_NO_EXCEPT
			{
				return matrix4x4f{ mtx.x_axis, mtx.y_axis, mtx.z_axis, mtx.z_axis };
			}

			inline RTM_SIMD_CALL operator matrix4x4d() const RTM_NO_EXCEPT
			{
				const vector4d z_axis = vector_cast(mtx.z_axis);
				return matrix4x4d{ vector_cast(mtx.x_axis), vector_cast(mtx.y_axis), z_axis, z_axis };
			}

			const matrix3x3f& mtx;
		};

		template<>
		struct matrix_caster<matrix3x3d>
		{
			constexpr explicit matrix_caster(const matrix3x3d& mtx_) RTM_NO_EXCEPT : mtx(mtx_) {}

			inline RTM_SIMD_CALL operator matrix3x3f() const RTM_NO_EXCEPT
			{
				return matrix3x3f{ vector_cast(mtx.x_axis), vector_cast(mtx.y_axis), vector_cast(mtx.z_axis) };
			}

			constexpr RTM_SIMD_CALL operator matrix3x3d() const RTM_NO_EXCEPT
			{
				return mtx;
			}

			inline RTM_SIMD_CALL operator matrix3x4f() const RTM_NO_EXCEPT
			{
				RTM_ASSERT(vector_get_w(mtx.x_axis) == 0.0, "X axis does not have a W component == 0.0");
				RTM_ASSERT(vector_get_w(mtx.y_axis) == 0.0, "Y axis does not have a W component == 0.0");
				RTM_ASSERT(vector_get_w(mtx.z_axis) == 0.0, "Z axis does not have a W component == 0.0");
				return matrix3x4f{ vector_cast(mtx.x_axis), vector_cast(mtx.y_axis), vector_cast(mtx.z_axis), vector_set(0.0f, 0.0f, 0.0f, 1.0f) };
			}

			inline RTM_SIMD_CALL operator matrix3x4d() const RTM_NO_EXCEPT
			{
				RTM_ASSERT(vector_get_w(mtx.x_axis) == 0.0f, "X axis does not have a W component == 0.0");
				RTM_ASSERT(vector_get_w(mtx.y_axis) == 0.0f, "Y axis does not have a W component == 0.0");
				RTM_ASSERT(vector_get_w(mtx.z_axis) == 0.0f, "Z axis does not have a W component == 0.0");
				return matrix3x4d{ mtx.x_axis, mtx.y_axis, mtx.z_axis, vector_set(0.0, 0.0, 0.0, 1.0) };
			}

			inline RTM_SIMD_CALL operator matrix4x4f() const RTM_NO_EXCEPT
			{
				const vector4f z_axis = vector_cast(mtx.z_axis);
				return matrix4x4f{ vector_cast(mtx.x_axis), vector_cast(mtx.y_axis), z_axis, z_axis };
			}

			constexpr RTM_SIMD_CALL operator matrix4x4d() const RTM_NO_EXCEPT
			{
				return matrix4x4d{ mtx.x_axis, mtx.y_axis, mtx.z_axis, mtx.z_axis };
			}

			const matrix3x3d& mtx;
		};

		template<>
		struct matrix_caster<matrix3x4f>
		{
			constexpr explicit matrix_caster(const matrix3x4f& mtx_) RTM_NO_EXCEPT : mtx(mtx_) {}

			constexpr RTM_SIMD_CALL operator matrix3x3f() const RTM_NO_EXCEPT
			{
				return matrix3x3f{ mtx.x_axis, mtx.y_axis, mtx.z_axis };
			}

			inline RTM_SIMD_CALL operator matrix3x3d() const RTM_NO_EXCEPT
			{
				return matrix3x3d{ vector_cast(mtx.x_axis), vector_cast(mtx.y_axis), vector_cast(mtx.z_axis) };
			}

			constexpr RTM_SIMD_CALL operator matrix3x4f() const RTM_NO_EXCEPT
			{
				return mtx;
			}

			inline RTM_SIMD_CALL operator matrix3x4d() const RTM_NO_EXCEPT
			{
				return matrix3x4d{ vector_cast(mtx.x_axis), vector_cast(mtx.y_axis), vector_cast(mtx.z_axis), vector_cast(mtx.w_axis) };
			}

			constexpr RTM_SIMD_CALL operator matrix4x4f() const RTM_NO_EXCEPT
			{
				return matrix4x4f{ mtx.x_axis, mtx.y_axis, mtx.z_axis, mtx.w_axis };
			}

			inline RTM_SIMD_CALL operator matrix4x4d() const RTM_NO_EXCEPT
			{
				return matrix4x4d{ vector_cast(mtx.x_axis), vector_cast(mtx.y_axis), vector_cast(mtx.z_axis), vector_cast(mtx.w_axis) };
			}

			const matrix3x4f& mtx;
		};

		template<>
		struct matrix_caster<matrix3x4d>
		{
			constexpr explicit matrix_caster(const matrix3x4d& mtx_) RTM_NO_EXCEPT : mtx(mtx_) {}

			inline RTM_SIMD_CALL operator matrix3x3f() const RTM_NO_EXCEPT
			{
				return matrix3x3f{ vector_cast(mtx.x_axis), vector_cast(mtx.y_axis), vector_cast(mtx.z_axis) };
			}

			constexpr RTM_SIMD_CALL operator matrix3x3d() const RTM_NO_EXCEPT
			{
				return matrix3x3d{ mtx.x_axis, mtx.y_axis, mtx.z_axis };
			}

			inline RTM_SIMD_CALL operator matrix3x4f() const RTM_NO_EXCEPT
			{
				return matrix3x4f{ vector_cast(mtx.x_axis), vector_cast(mtx.y_axis), vector_cast(mtx.z_axis), vector_cast(mtx.w_axis) };
			}

			constexpr RTM_SIMD_CALL operator matrix3x4d() const RTM_NO_EXCEPT
			{
				return mtx;
			}

			inline RTM_SIMD_CALL operator matrix4x4f() const RTM_NO_EXCEPT
			{
				return matrix4x4f{ vector_cast(mtx.x_axis), vector_cast(mtx.y_axis), vector_cast(mtx.z_axis), vector_cast(mtx.w_axis) };
			}

			constexpr RTM_SIMD_CALL operator matrix4x4d() const RTM_NO_EXCEPT
			{
				return matrix4x4d{ mtx.x_axis, mtx.y_axis, mtx.z_axis, mtx.w_axis };
			}

			const matrix3x4d& mtx;
		};

		template<>
		struct matrix_caster<matrix4x4f>
		{
			constexpr explicit matrix_caster(const matrix4x4f& mtx_) RTM_NO_EXCEPT : mtx(mtx_) {}

			constexpr RTM_SIMD_CALL operator matrix3x3f() const RTM_NO_EXCEPT
			{
				return matrix3x3f{ mtx.x_axis, mtx.y_axis, mtx.z_axis };
			}

			inline RTM_SIMD_CALL operator matrix3x3d() const RTM_NO_EXCEPT
			{
				return matrix3x3d{ vector_cast(mtx.x_axis), vector_cast(mtx.y_axis), vector_cast(mtx.z_axis) };
			}

			inline RTM_SIMD_CALL operator matrix3x4f() const RTM_NO_EXCEPT
			{
				RTM_ASSERT(vector_get_w(mtx.x_axis) == 0.0f, "X axis does not have a W component == 0.0");
				RTM_ASSERT(vector_get_w(mtx.y_axis) == 0.0f, "Y axis does not have a W component == 0.0");
				RTM_ASSERT(vector_get_w(mtx.z_axis) == 0.0f, "Z axis does not have a W component == 0.0");
				return matrix3x4f{ mtx.x_axis, mtx.y_axis, mtx.z_axis, mtx.w_axis };
			}

			inline RTM_SIMD_CALL operator matrix3x4d() const RTM_NO_EXCEPT
			{
				RTM_ASSERT(vector_get_w(mtx.x_axis) == 0.0f, "X axis does not have a W component == 0.0");
				RTM_ASSERT(vector_get_w(mtx.y_axis) == 0.0f, "Y axis does not have a W component == 0.0");
				RTM_ASSERT(vector_get_w(mtx.z_axis) == 0.0f, "Z axis does not have a W component == 0.0");
				return matrix3x4d{ vector_cast(mtx.x_axis), vector_cast(mtx.y_axis), vector_cast(mtx.z_axis), vector_cast(mtx.w_axis) };
			}

			constexpr RTM_SIMD_CALL operator matrix4x4f() const RTM_NO_EXCEPT
			{
				return mtx;
			}

			inline RTM_SIMD_CALL operator matrix4x4d() const RTM_NO_EXCEPT
			{
				return matrix4x4d{ vector_cast(mtx.x_axis), vector_cast(mtx.y_axis), vector_cast(mtx.z_axis), vector_cast(mtx.w_axis) };
			}

			const matrix4x4f& mtx;
		};

		template<>
		struct matrix_caster<matrix4x4d>
		{
			constexpr explicit matrix_caster(const matrix4x4d& mtx_) RTM_NO_EXCEPT : mtx(mtx_) {}

			inline RTM_SIMD_CALL operator matrix3x3f() const RTM_NO_EXCEPT
			{
				return matrix3x3f{ vector_cast(mtx.x_axis), vector_cast(mtx.y_axis), vector_cast(mtx.z_axis) };
			}

			constexpr RTM_SIMD_CALL operator matrix3x3d() const RTM_NO_EXCEPT
			{
				return matrix3x3d{ mtx.x_axis, mtx.y_axis, mtx.z_axis };
			}

			inline RTM_SIMD_CALL operator matrix3x4f() const RTM_NO_EXCEPT
			{
				RTM_ASSERT(vector_get_w(mtx.x_axis) == 0.0, "X axis does not have a W component == 0.0");
				RTM_ASSERT(vector_get_w(mtx.y_axis) == 0.0, "Y axis does not have a W component == 0.0");
				RTM_ASSERT(vector_get_w(mtx.z_axis) == 0.0, "Z axis does not have a W component == 0.0");
				return matrix3x4f{ vector_cast(mtx.x_axis), vector_cast(mtx.y_axis), vector_cast(mtx.z_axis), vector_cast(mtx.w_axis) };
			}

			inline RTM_SIMD_CALL operator matrix3x4d() const RTM_NO_EXCEPT
			{
				RTM_ASSERT(vector_get_w(mtx.x_axis) == 0.0, "X axis does not have a W component == 0.0");
				RTM_ASSERT(vector_get_w(mtx.y_axis) == 0.0, "Y axis does not have a W component == 0.0");
				RTM_ASSERT(vector_get_w(mtx.z_axis) == 0.0, "Z axis does not have a W component == 0.0");
				return matrix3x4d{ mtx.x_axis, mtx.y_axis, mtx.z_axis, mtx.w_axis };
			}

			inline RTM_SIMD_CALL operator matrix4x4f() const RTM_NO_EXCEPT
			{
				return matrix4x4f{ vector_cast(mtx.x_axis), vector_cast(mtx.y_axis), vector_cast(mtx.z_axis), vector_cast(mtx.w_axis) };
			}

			constexpr RTM_SIMD_CALL operator matrix4x4d() const RTM_NO_EXCEPT
			{
				return mtx;
			}

			const matrix4x4d& mtx;
		};
	}

	//////////////////////////////////////////////////////////////////////////
	// Casts a matrix from one variant to another.
	//////////////////////////////////////////////////////////////////////////
	template<typename matrix_type>
	constexpr rtm_impl::matrix_caster<matrix_type> RTM_SIMD_CALL matrix_cast(const matrix_type& input) RTM_NO_EXCEPT
	{
		return rtm_impl::matrix_caster<matrix_type>(input);
	}
}
