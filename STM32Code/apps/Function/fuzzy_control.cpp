#include "fuzzy_control.h"

#ifndef MAX
# define MAX(x,y) ( (x) > (y) ? (x) : (y) )
#endif
#ifndef MIN
# define MIN(x,y) ( (x) < (y) ? (x) : (y) )
#endif


float fuzzy_prod_and(float a, float b) { return a*b;};


float Gaussmf::cal(float x)
{
	float tmp;

	if (sigma==0)
		return -1.0f;
	tmp = (x-center)/sigma;
	return(exp(-tmp*tmp/2));
}


Gaussmf::Gaussmf(float sigma, float center)
{
	this->sigma = sigma;
	this->center = center;
}

Fuzzifier::Fuzzifier():
	e_neg_mf(7.0f, -10.0f),
	e_pos_mf(7.0f, 10.0f),
  ce_neg_mf(7.0f, -10.0f),
  ce_pos_mf(7.0f, 10.0f)
{

}

void Fuzzifier::set_e_domain(float min, float max)
{
	this->e_neg_mf.set_center(min);
	this->e_pos_mf.set_center(max);
}

void Fuzzifier::set_ce_domain(float min, float max)
{
	this->ce_neg_mf.set_center(min);
	this->ce_pos_mf.set_center(max);
}

int scop_fc_fuzzificat_i1;
int scop_fc_fuzzificat_i2;
int scop_fc_fuzzificat_i3;
int scop_fc_fuzzificat_i4;
int scop_fc_fuzzificat_o1;
int scop_fc_fuzzificat_o2;
int scop_fc_fuzzificat_o3;
int scop_fc_fuzzificat_o4;

void Fuzzifier::fuzzification(float *input, float *output)
{
	output[0] = e_neg_mf.cal(input[0]);
	output[1] = e_pos_mf.cal(input[0]);
	output[2] = ce_neg_mf.cal(input[1]);
	output[3] = ce_pos_mf.cal(input[1]);
	
	scop_fc_fuzzificat_i1 = 100*input[0];
	scop_fc_fuzzificat_i2 = 100*input[1];
	scop_fc_fuzzificat_i3 = 100*input[1];
	scop_fc_fuzzificat_i4 = 100*input[1];
	
	scop_fc_fuzzificat_o1 = 100*output[0];
	scop_fc_fuzzificat_o2 = 100*output[1];
	scop_fc_fuzzificat_o3 = 100*output[2];
	scop_fc_fuzzificat_o4 = 100*output[3];
}
	
SugenoFuzzyLogic::SugenoFuzzyLogic()
{
	input_domain[0] = -10.0f;
	input_domain[1] = 10.0f;
	output_domain[0] = -40.0f;
	output_domain[1] = 40.0f;
	
	min_value = output_domain[0];
	zero_value = 0.0f;
	max_value = output_domain[1];
	
}


SugenoFuzzyLogic::SugenoFuzzyLogic(float input_min, float input_max, float output_min, float output_max)
{
	input_domain[0] = input_min;
	input_domain[1] = input_max;
	output_domain[0] = output_min;
	output_domain[1] = output_max;
	
	min_value = output_min;
	zero_value = 0;
	max_value = output_max;
	
	fuzzifier.set_e_domain(input_domain[0], input_domain[1]);
	fuzzifier.set_ce_domain(input_domain[0], input_domain[1]);
}


void SugenoFuzzyLogic::get_input(float e, float ce)
{
	input[0] = MAX(MIN(e, input_domain[1]), input_domain[0]);
	input[1] = MAX(MIN(ce, input_domain[1]), input_domain[0]);
}

int scop_fc_firing1, scop_fc_firing2, scop_fc_firing3, scop_fc_firing4;
int scop_fc_num, scop_fc_den;

float SugenoFuzzyLogic::cal_output(void)
{
	float num, den;
	fuzzifier.fuzzification(input, mf_value);
	firing_strength[0] = fuzzy_prod_and(mf_value[0], mf_value[2]);
	firing_strength[1] = fuzzy_prod_and(mf_value[0], mf_value[3]);
	firing_strength[2] = fuzzy_prod_and(mf_value[1], mf_value[2]);
	firing_strength[3] = fuzzy_prod_and(mf_value[1], mf_value[3]);
	output_level[0] = min_value;
	output_level[1] = zero_value;
	output_level[2] = zero_value;
	output_level[3] = max_value;
	num = den = 0.0f;
	for(int i = 0; i < 4; i++)
	{
		num += firing_strength[i]*output_level[i];
		den += firing_strength[i];
	}
	if(den == 0) 
	{
		rt_kprintf("fuzzy logic err: firing strength is 0.");
//		return 0;
	}
	
	scop_fc_firing1 = firing_strength[0] * 100;
	scop_fc_firing2 = firing_strength[1] * 100;
	scop_fc_firing3 = firing_strength[2] * 100;
	scop_fc_firing4 = firing_strength[3] * 100;
	scop_fc_den = den * 100;
	scop_fc_num = num * 100;
	return num/den;
}


FuzzyController::FuzzyController(float kp, float ke, float kce, float kout)
{
	this->kp = kp;
	this->ke = ke;
	this->kce = kce;
	this->kout = kout;
}




float FuzzyController::run(float err)
{
	this->last_e = this->e;
	this->e = err;
	this->ce = this->e - this->last_e;
	fuzzy_logic.get_input(ke * this->e, kce * this->ce);
	this->output = this->kout * fuzzy_logic.cal_output();
	this->output += this->kp * e;
	return this->output;
}

	
	
	
	

