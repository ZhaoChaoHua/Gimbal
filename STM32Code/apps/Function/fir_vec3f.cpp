#include "fir_vec3f.h"

FIR_VEC3F::FIR_VEC3F()
{
	x_fir = filter_fir_f32_init(FIR_COEF_LEN, fir_coef, NULL, 1);
	y_fir = filter_fir_f32_init(FIR_COEF_LEN, fir_coef, NULL, 1);
	z_fir = filter_fir_f32_init(FIR_COEF_LEN, fir_coef, NULL, 1);
}

FIR_VEC3F::FIR_VEC3F(size_t coeff_size, const float32_t* x_coeffs, const float32_t* y_coeffs, const float32_t* z_coeffs)
{
	x_fir = filter_fir_f32_init(coeff_size, x_coeffs, NULL, 1);
	y_fir = filter_fir_f32_init(coeff_size, y_coeffs, NULL, 1);
	z_fir = filter_fir_f32_init(coeff_size, z_coeffs, NULL, 1);
}

Vector3f FIR_VEC3F::update(Vector3f in)
{
	Vector3f out;
	filter_fir_f32_process(x_fir, &in.x, &out.x);
	filter_fir_f32_process(y_fir, &in.y, &out.y);
	filter_fir_f32_process(z_fir, &in.z, &out.z);
	return out;
}
	
void FIR_VEC3F::clear(void)
{
	filter_fir_f32_clear(x_fir);
	filter_fir_f32_clear(y_fir);
	filter_fir_f32_clear(z_fir);
	return;
}
	


