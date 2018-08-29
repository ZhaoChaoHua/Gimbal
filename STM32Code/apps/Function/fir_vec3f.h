#ifndef __FIR_VEC3F_H__
#define __FIR_VEC3F_H__

#include "fir.h"
#include <AP_Math.h>


// FIR Definetion
#define FIR_ORDER			50
#define FIR_COEF_LEN      FIR_ORDER+1
// ACC X Freq cut = 
static float fir_coef[FIR_COEF_LEN] = {
	     0,0.0005038403298566, 0.001245499730966, 0.002219339446247,
   0.003416058902617,  0.00482284237148, 0.006423561070459, 0.008199027519145,
    0.01012729813741,  0.01218401931392,  0.01434281148689,  0.01657568517939,
    0.01885348242777,  0.02114633664091,  0.02342414363751,  0.02565703643226,
    0.02781585628243,  0.02987261256562,  0.03180092423536,   0.0335764358918,
    0.03517720190529,  0.03658403253415,  0.03778079657784,  0.03875467579176,
    0.03949636705117,  0.04000022907548,  0.03949636705117,  0.03875467579176,
    0.03778079657784,  0.03658403253415,  0.03517720190529,   0.0335764358918,
    0.03180092423536,  0.02987261256562,  0.02781585628243,  0.02565703643226,
    0.02342414363751,  0.02114633664091,  0.01885348242777,  0.01657568517939,
    0.01434281148689,  0.01218401931392,  0.01012729813741, 0.008199027519145,
   0.006423561070459,  0.00482284237148, 0.003416058902617, 0.002219339446247,
   0.001245499730966,0.0005038403298566,                 0
};
	
class FIR_VEC3F
{
	private:
    filter_fir_f32_t* x_fir;
	  filter_fir_f32_t* y_fir;
	  filter_fir_f32_t* z_fir;
	  Vector3f  input;
	  Vector3f  output;
	public:
		FIR_VEC3F();
		FIR_VEC3F(size_t coeff_size, const float32_t* x_coeffs, const float32_t* y_coeffs, const float32_t* z_coeffs);
	  Vector3f update(Vector3f in);
	  void clear(void);
};

#endif


