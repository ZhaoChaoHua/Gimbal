#include <rtthread.h>
#include "stm32f4xx.h"
#include <AP_Math.h>
#include "AP_Progmem.h"

// Membership Function
class Gaussmf
{
	private:
		float sigma;
		float center;
	
	public:
		Gaussmf(float sigma, float center);
	  float cal(float x);
	  void set_sigma(float s) { sigma = s;};
		void set_center(float c) { center = c;};
};

// Fuzzification
class Fuzzifier
{
	private:
		Gaussmf e_neg_mf;
	  Gaussmf e_pos_mf;
	  Gaussmf ce_neg_mf;
	  Gaussmf ce_pos_mf;
	public:
		Fuzzifier();
		void fuzzification(float *input, float *output);
	  void set_e_domain(float min, float max);
	  void set_ce_domain(float min, float max);
};

// Logic operation and defuzzification
class SugenoFuzzyLogic
{
	private:
		Fuzzifier  fuzzifier;
	  float input[2];
	
		float min_value;
	  float zero_value;
	  float max_value;
	  float mf_value[4];
	  float firing_strength[4];
	  float output_level[4];
	  
	
	  float input_domain[2];
	  float output_domain[2];
	
	
	public:
		SugenoFuzzyLogic();
		SugenoFuzzyLogic(float input_min, float input_max, float output_min,  float output_max);
	  void get_input(float e, float ce);
	
		float cal_output(void);
};


class FuzzyController
{
	private:
		SugenoFuzzyLogic fuzzy_logic;
		float e;
	  float last_e;
		float ce;
		float output;
	
	  float kp;
	  float ke;
	  float kce;
	  float kout;
	public:
		
		FuzzyController(float kp, float ke, float kce, float kout);
	  float run(float e);
	  void set_kp(float kp) { this->kp = kp;};
	  void set_ke(float ke) {this->ke = ke;};
		void set_kce(float kce) {this->kce = kce;};
		void set_kout(float kout) {this->kout = kout;}
		float get_kp(void) {return this->kp;}
		float get_ke(void) {return this->ke;}
		float get_kce(void) {return this->kce;}
		float get_kout(void) {return this->kout;}		
		
};
