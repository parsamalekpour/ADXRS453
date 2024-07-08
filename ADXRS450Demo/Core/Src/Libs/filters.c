/*
 * filters.c
 *
 *  Created on: Jun 24, 2024
 *      Author: ParsaMalekpour
 */
#include "filters.h"

float Filter_EMA_Init(Filter_EMA *filter, float alpha){
	Filter_EMA_SetAlpha(filter, alpha);
	filter->out = 0.0f;
}

void Filter_EMA_SetAlpha(Filter_EMA *filter, float alpha){
	filter->alpha = constrain(alpha, 0.0f, 1.0f);
}
float Filter_EMA_Updata(Filter_EMA *filter, float input){
	filter->out = filter->alpha * input + (1.0f - filter->alpha) * filter->out;
	return filter->out;
}
