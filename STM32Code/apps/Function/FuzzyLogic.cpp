#include "FuzzyLogic.h"
#include <stdlib.h>

#define FuzzyDebug(fmt, args ...)  do {rt_kprintf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)

#ifndef ABS
# define ABS(x)   ( (x) > (0) ? (x): (-(x)) )
#endif
#ifndef MAX
# define MAX(x,y) ( (x) > (y) ? (x) : (y) )
#endif
#ifndef MIN
# define MIN(x,y) ( (x) < (y) ? (x) : (y) )
#endif

#define RULE_NUM             49
#define DOMAIN               3.0f

FuzzyLogic::FuzzyLogic(const uint8_t (&inrule)[49][3], const struct mf_node (&ine_mf)[7], const struct mf_node (&inec_mf)[7], const struct mf_node (&inkp_mf)[7]) :
    rule(inrule),
    e_mf(ine_mf),
    ec_mf(inec_mf),
    out_mf(inkp_mf)
{
    // 模糊控制
    flccon.rule_n = RULE_NUM;
    flccon.aggFcn = fisMax;
    flccon.andFcn = fisProduct;
    flccon.impFcn = fisMin;
    flccon.orFcn = fisProbOr;
    flccon.input_n = 2;
    flccon.output_n = 1;
}

void FuzzyLogic::start_init(float rang_ine, float rang_inec, float rang_out)
{
    // Fuzzy Logic Conreoller initialize
    initialize_fuzzy(rang_ine,rang_inec,rang_out);
}

void FuzzyLogic::update(float ine, float inec)
{
    // get the fuzzy system inputs
    get_fuzzy_inputs(ine, inec);
    // inputs fuzzification
    fuzzification();
    // rule evaluation
    rule_evaluation();
    // finally get the outputs
}

/***********************************************************************
 Fuzzy Logic Public functions
 **********************************************************************/
void FuzzyLogic::initialize_fuzzy(float limit_ine, float limit_inec, float limit_out)
{
    struct io_node *outptr;
    struct mf_node *top_mf;
    struct mf_node *mfptr;
    struct io_node *ioptr;
    struct rule_node *ruleptr;
    struct rule_element *ifptr;
    struct rule_element *thenptr;

    ioptr = NULL;
    ruleptr = NULL;
    ifptr = NULL;
    thenptr = NULL;
    // error input init
    ioptr = (struct io_node *)rt_malloc(sizeof(struct io_node));
    rt_memset(ioptr,0,sizeof(struct io_node));
    Fuzzy_inputs = ioptr;
    // input parameter config
    rt_sprintf(ioptr->name,"%s","ine");
    // range of error input
    ioptr->scale = DOMAIN/limit_ine;
    ioptr->mf_n = 7;
    // 隶属函数指针
    mfptr = NULL;
    
    // 依次循环7个隶属函数
    for(int i=0; i<ioptr->mf_n; i++) {
        if(mfptr == NULL) {                    /* first time thru only */
            mfptr = (struct mf_node *)rt_malloc(sizeof(struct mf_node));
            rt_memset(mfptr,0,sizeof(struct mf_node));
            top_mf = mfptr;
            ioptr->mf = mfptr;
        }
        else{
            for(mfptr = top_mf; mfptr->next != NULL; mfptr = mfptr->next); /* spin to last */
            mfptr->next = (struct mf_node *)rt_malloc(sizeof(struct mf_node));
            rt_memset(mfptr->next,0,sizeof(struct mf_node));
            mfptr = mfptr->next;
        }
        // mfFcn parameter
        rt_sprintf(mfptr->mftype,"%s",e_mf[i].mftype);
        mfptr->name = e_mf[i].name;
        mfptr->nparams = e_mf[i].nparams;
        mfptr->params[0] = e_mf[i].params[0];
        mfptr->params[1] = e_mf[i].params[1];
        mfptr->params[2] = e_mf[i].params[2];
        mfptr->params[3] = e_mf[i].params[3];
        mfptr->mfFcn = e_mf[i].mfFcn;
    }
    // ec input parameter init
    ioptr->next=(struct io_node *)calloc(1,sizeof(struct io_node));
    rt_memset(ioptr->next,0,sizeof(struct io_node));
    ioptr=ioptr->next;
    rt_sprintf(ioptr->name,"%s","inec");
    // rang of ec
    ioptr->scale = DOMAIN/limit_inec;
    ioptr->mf_n = 7;
    mfptr=NULL;
    
    for(int i=0;i<ioptr->mf_n; i++) {
        if(mfptr == NULL) {                    /* first time thru only */
            mfptr = (struct mf_node *)rt_malloc(sizeof(struct mf_node));
            rt_memset(mfptr,0,sizeof(struct mf_node));
            top_mf = mfptr;
            ioptr->mf = mfptr;
        }
        else{  
            for(mfptr = top_mf; mfptr->next != NULL; mfptr = mfptr->next); /* spin to last */
            mfptr->next=(struct mf_node *)rt_malloc(sizeof(struct mf_node));
            rt_memset(mfptr->next,0,sizeof(struct mf_node));
            mfptr=mfptr->next;
        }
        rt_sprintf(mfptr->mftype,"%s",ec_mf[i].mftype);
        mfptr->name = ec_mf[i].name;
        mfptr->nparams = ec_mf[i].nparams;
        mfptr->params[0] = ec_mf[i].params[0];
        mfptr->params[1] = ec_mf[i].params[1];
        mfptr->params[2] = ec_mf[i].params[2];
        mfptr->params[3] = ec_mf[i].params[3];
        mfptr->mfFcn = ec_mf[i].mfFcn;
    }
    // output fcn
    outptr=(struct io_node *)rt_malloc(sizeof(struct io_node));
    rt_memset(outptr,0,sizeof(struct io_node));
    Fuzzy_outputs=outptr;
    rt_sprintf(outptr->name,"%s","out");
    // range of out
    outptr->scale = DOMAIN/limit_out;
    outptr->mf_n = 7;
    mfptr=NULL;
    for(int i=0;i<outptr->mf_n; i++) {
        if(mfptr == NULL) {                    /* first time thru only */  
            mfptr = (struct mf_node *)rt_malloc(sizeof(struct mf_node));
            rt_memset(mfptr,0,sizeof(struct mf_node));
            top_mf = mfptr;
            outptr->mf = mfptr;
        }
        else{
            for(mfptr = top_mf; mfptr->next != NULL; mfptr = mfptr->next); /* spin to last */
            mfptr->next=(struct mf_node *)rt_malloc(sizeof(struct mf_node));
            rt_memset(mfptr->next,0,sizeof(struct mf_node));
            mfptr=mfptr->next;
        }
        rt_sprintf(mfptr->mftype,"%s",out_mf[i].mftype);
        mfptr->name = out_mf[i].name;
        mfptr->nparams = out_mf[i].nparams;
        mfptr->params[0] = out_mf[i].params[0];
        mfptr->params[1] = out_mf[i].params[1];
        mfptr->params[2] = out_mf[i].params[2];
        mfptr->params[3] = out_mf[i].params[3];
        mfptr->mfFcn = out_mf[i].mfFcn;
    }
    // Fuzzy rules
    ioptr=NULL;
    outptr=NULL;
    ruleptr=(struct rule_node *)rt_malloc(sizeof(struct rule_node));
    rt_memset(ruleptr,0,sizeof(struct rule_node));
    Rule_Base=ruleptr;
    for(int i=0;i<flccon.rule_n; i++) 
    {
        ioptr=Fuzzy_inputs;               /* points to e */
        for(mfptr = ioptr->mf; mfptr!=NULL; mfptr=mfptr->next) {
            // 匹配规则
            if(mfptr->name == rule[i][0]) {
                ifptr=(struct rule_element *)rt_malloc(sizeof(struct rule_element));
                rt_memset(ifptr,0,sizeof(struct rule_element));
                ruleptr->if_side=ifptr;
                ifptr->value = &mfptr->value;
                ifptr->next = (struct rule_element *)rt_malloc(sizeof(struct rule_element));
                rt_memset(ifptr->next,0,sizeof(struct rule_element));
                ifptr=ifptr->next;
                break;
            }
        }
        
        ioptr=ioptr->next;                 /* points to ec */
        for(mfptr=ioptr->mf;mfptr!=NULL;mfptr=mfptr->next) {
            if(mfptr->name == rule[i][1]) {
                ifptr->value = &mfptr->value;
                break;
            }
        }

        outptr=Fuzzy_outputs;/* point then stuff to output */
        for(mfptr=outptr->mf; mfptr!=NULL; mfptr=mfptr->next) {
            if(mfptr->name == rule[i][2]) {
                thenptr = (struct rule_element *)rt_malloc(sizeof(struct rule_element));
                rt_memset(thenptr,0,sizeof(struct rule_element));
                ruleptr->then_side = thenptr;
                thenptr->value = &mfptr->value;
                break;
            }
        }
        ruleptr->next=(struct rule_node *)rt_malloc(sizeof(struct rule_node));
        rt_memset(ruleptr->next,0,sizeof(struct rule_node));
        ruleptr=ruleptr->next;
    }
}

void FuzzyLogic::get_fuzzy_inputs(float ine, float inec)
{
    struct io_node *ioptr;
    ioptr = Fuzzy_inputs;
    ioptr->value = ine * ioptr->scale;
    ioptr = ioptr->next;
    ioptr->value = inec * ioptr->scale;
}

void FuzzyLogic::fuzzification()
{
    struct io_node *si;
    struct mf_node *mf;
    for(si=Fuzzy_inputs; si!=NULL; si=si->next)
        for(mf = si->mf; mf!=NULL; mf=mf->next)
            mf->value = mf->mfFcn(si->value, mf->params);
}

// rule_evaluation
void FuzzyLogic::rule_evaluation()
{
    struct rule_node *rule;
    struct rule_element *ip;    /* if ptr */
    struct io_node *inptr;
    struct mf_node *inmf;
    struct io_node *outptr;
    struct mf_node *outmf;
    float strength;
    float total_w;
    float total_wf;
    int i;
    float out;
    
    // 计算出规则强度
    for(rule=Rule_Base; rule!=NULL; rule=rule->next) {
        strength=1.0f;
        for(ip=rule->if_side;ip!=NULL;ip=ip->next) {
            strength = flccon.andFcn(strength,*(ip->value));
        }
        rule->firing_strength = strength;
        // 规则强度求和
        total_w += rule->firing_strength;
    }
    if(total_w == 0) {
        rt_kprintf("NO MATCHING RULES FOUND!\n");
        return;
    }
    
    // 计算出输出
    outptr = Fuzzy_outputs;
    inptr  = Fuzzy_inputs;
    for(outmf=outptr->mf; outmf!=NULL; outmf=outmf->next)
    {
        out = 0.0f;
        for(inmf = inptr->mf, i=0; inmf!=NULL; inmf=inmf->next, i++)
        {
            out += inmf->value * outmf->params[i];
        }
        out = out + outmf->params[i+1];
        outmf->value = out;
    }
    
    total_wf = 0.0f;
    for(rule=Rule_Base; rule!=NULL; rule=rule->next)
    {
        rule->rule_output = *(rule->then_side->value);
        total_wf += rule->firing_strength * rule->rule_output;
    }

    fuzzy_out = total_wf/total_w;
}

// put fuzzy outputs
float FuzzyLogic::fuzzy_outputs()
{
    return fuzzy_out;
}

/***********************************************************************
 Parameterized membership functions
 **********************************************************************/
/* Triangular membership function */
float FuzzyLogic::fisTriangleMf(float x, float *params)
{
	float a = params[0], b = params[1], c = params[2];

	if (a>b)
		FuzzyDebug("Illegal parameters in fisTriangleMf() --> a > b");
	if (b>c)
		FuzzyDebug("Illegal parameters in fisTriangleMf() --> b > c");

	if (a == b && b == c)
		return(x == a);
	if (a == b)
		return((c-x)/(c-b)*(b<=x)*(x<=c));
	if (b == c)
		return((x-a)/(b-a)*(a<=x)*(x<=b));
	return(MAX(MIN((x-a)/(b-a), (c-x)/(c-b)), 0));
}

/* Trapezpoidal membership function */
float FuzzyLogic::fisTrapezoidMf(float x, float *params)
{
	float a = params[0], b = params[1], c = params[2], d = params[3];
	float y1 = 0, y2 = 0;

	if (a>b) {
		FuzzyDebug("Illegal parameters in fisTrapezoidMf() --> a > b");
	}
	if (b>c) {
		FuzzyDebug("Illegal parameters in fisTrapezoidMf() --> b > c");
	}
	if (c>d) {
		FuzzyDebug("Illegal parameters in fisTrapezoidMf() --> c > d");
	}

	if (b <= x)
		y1 = 1;
	else if (x < a)
		y1 = 0;
	else if (a != b)
		y1 = (x-a)/(b-a);

	if (x <= c)
		y2 = 1;
	else if (d < x)
		y2 = 0;
	else if (c != d)
		y2 = (d-x)/(d-c);

	return(MIN(y1, y2));
}

/* Gaussian membership function */
float FuzzyLogic::fisGaussianMf(float x, float *params)
{
	float sigma = params[0], c = params[1];
	float tmp;

	if (sigma==0)
		FuzzyDebug("Illegal parameters in fisGaussianMF() --> sigma = 0");
	tmp = (x-c)/sigma;
	return(exp(-tmp*tmp/2));
}

/* Extended Gaussian membership function */
float FuzzyLogic::fisGaussian2Mf(float x, float *params)
{
	float sigma1 = params[0], c1 = params[1];
	float sigma2 = params[2], c2 = params[3];
	float tmp1, tmp2;

	if ((sigma1 == 0) || (sigma2 == 0))
		FuzzyDebug("Illegal parameters in fisGaussian2MF() --> sigma1 or sigma2 is zero");

	tmp1 = x >= c1? 1:exp(-powf((x-c1)/sigma1, 2.0)/2);
	tmp2 = x <= c2? 1:exp(-powf((x-c2)/sigma2, 2.0)/2);
	return(tmp1*tmp2);
}

/* returns the number of parameters of MF */
int FuzzyLogic::fisGetMfParaN(const char *mfType)
{
	if (strcmp(mfType, "trimf") == 0)
		return(3);
	if (strcmp(mfType, "trapmf") == 0)
		return(4);
	if (strcmp(mfType, "gaussmf") == 0)
		return(2);
	if (strcmp(mfType, "gauss2mf") == 0)
		return(4);
	FuzzyDebug("Given MF type (%s) is unknown.\n", mfType);
	return(0);	/* get rid of compiler warning */
}
/***********************************************************************
 T-norm and T-conorm operators
 **********************************************************************/
/* Copyright 1994-2002 The MathWorks, Inc.  */
/* $Revision: $  $Date: $  */

float FuzzyLogic::fisMin(float x, float y)
{ return((x) < (y) ? (x) : (y)); }

float FuzzyLogic::fisMax(float x, float y)
{ return((x) > (y) ? (x) : (y)); }

float FuzzyLogic::fisProduct(float x, float y)
{ return(x*y); } 

float FuzzyLogic::fisProbOr(float x, float y)
{ return(x + y - x*y); } 

float FuzzyLogic::fisSum(float x, float y)
{ return(x + y); } 

