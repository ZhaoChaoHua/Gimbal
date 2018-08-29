#ifndef __FUZZYLOGIC_H__
#define __FUZZYLOGIC_H__
 
#include <rtthread.h>
#include "stm32f4xx.h"
#include <AP_Math.h>
#include "AP_Progmem.h"

#ifdef __cplusplus

#define MF_PARA_N  4
#define MAXNAME    10

// 定义自然语言变量
#define NB         1 // 负大
#define NM         2 // 负中
#define NS         3 // 负小
#define ZO         4 // 零
#define PS         5 // 正小
#define PM         6 // 正中
#define PB         7 // 正大

#define GET_PARAMN(name) ((int)(((const FuzzyLogic *) 1)->fisGetMfParaN(name)))

#define MF_GAUSSIAN(name, mftype, param1, param2, fcn) { name, mftype, GET_PARAMN(mftype), {param1, param2}, fcn, 0.0f, NULL }
#define MF_TRIANGLE(name, mftype, param1, param2, param3, fcn) { name, mftype, GET_PARAMN(mftype), {param1, param2,param3}, fcn, 0.0f, NULL }
#define MF_LINEAR(name, mftype, param1, param2, param3) { name, mftype, GET_PARAMN(mftype), {param1, param2,param3}, NULL, 0.0f, NULL }

// 隶属函数节点
struct mf_node {
    int8_t name;             /* MF name */
    char mftype[MAXNAME+1];
    int nparams;			  /* length of params field */
    float params[4];	      /* MF parameters */
    float (*mfFcn)(float, float *); /* pointer to a mem. fcn */
    float value;		      /* for Sugeno only */
    struct mf_node *next;
};

// IO结构体
struct io_node {
    char name[MAXNAME+1];
    float scale;
    uint8_t mf_n;
    float value;
    struct mf_node *mf;
    struct io_node *next;
};

// 规则元素
struct rule_element{
    float out;
    float *value;
    struct rule_element *next;
};

struct rule_node {
    struct rule_element *if_side;
    struct rule_element *then_side;
    float firing_strength;
    float rule_output;
    struct rule_node *next;
};

struct fuzzy_config {
    uint8_t rule_n;
    uint8_t input_n;
    uint8_t output_n;
    float (*andFcn)(float, float);
    float (*orFcn)(float, float);
    float (*impFcn)(float, float);
    float (*aggFcn)(float, float);
};

class FuzzyLogic
{
private:
	static float fisMin(float x, float y);
	static float fisMax(float x, float y);
	static float fisProduct(float x, float y);
	static float fisProbOr(float x, float y);
	static float fisSum(float x, float y);
	
    
	struct io_node *Fuzzy_inputs;
	struct io_node *Fuzzy_outputs;
	struct rule_node *Rule_Base;
    struct fuzzy_config flccon;

    float fuzzy_out;
    
    // 控制规则
    const uint8_t (&rule)[49][3];
    // E输入隶属函数
    const struct mf_node (&e_mf)[7];
    // Ec输入隶属函数
    const struct mf_node (&ec_mf)[7];
    // 输出隶属函数
    const struct mf_node (&out_mf)[7];
    
public:
	FuzzyLogic(const uint8_t (&inrule)[49][3], const struct mf_node (&ine_mf)[7], const struct mf_node (&inec_mf)[7], const struct mf_node (&inkp_mf)[7]);

    static float fisTriangleMf(float x, float *params);
	/* Trapezpoidal membership function */
	static float fisTrapezoidMf(float x, float *params);
	/* Gaussian membership function */
	static float fisGaussianMf(float x, float *params);
	/* Extended Gaussian membership function */
	static float fisGaussian2Mf(float x, float *params);
    static float linearMf(float x, float y, float *params);
    static float constantMf(float *params);
    
    /* returns the number of parameters of MF */
	static int fisGetMfParaN(const char *mfType);

    void start_init(float rang_ine, float rang_inec, float rang_out);
    void update(float ine, float inec);
	// Initialize Fuzzy system
	void initialize_fuzzy(float rang_ine, float rang_inec, float rang_out);
	// Get Fuzzy inputs
	void get_fuzzy_inputs(float ina, float inb);
	// fuzzification mothod
	void fuzzification();
	// rule_evaluation
	void rule_evaluation();
	// put fuzzy outputs
	float fuzzy_outputs();
	
};

#endif
#endif
