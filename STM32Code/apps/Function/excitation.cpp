#include <math.h>
#include "excitation.h"

#define  PI  3.14159265359f
#define  TWO_PI  6.283185307179586f


static float sin_func(float w, float time)
{
	return sinf(w*time);
}

void Exciter::init(EXCITATION_FUNCTION function, float dt)
{
	this->dt = dt;
	switch(function)
	{
		case SINE:
			this->excitation_function = &sin_func;
		  break;
		default:
			break;
	}
	
	t = 0;
	freq_cnt = 0;
	exciting = false;
}


int scop_excitation_freq;
float Exciter::update(float amp)
{
	float y = 0;
	if(exciting)
	{
		if(freq_cnt < FREQ_TABLE_LEN)
		{
			y = amp * amp_coef_table[freq_cnt] * excitation_function(log_w_table[freq_cnt], t);
			scop_excitation_freq = 1000*log_w_table[freq_cnt]/TWO_PI;
	
			t += dt;
			if(t>log_t_table[freq_cnt])
			{
				t = 0;
				freq_cnt ++;
				if(freq_cnt == FREQ_TABLE_LEN) 
				{
					freq_cnt = 0;
					exciting = false;
				}
			}
		}
	}
	
	return y; 
}

