/*
 * filters.h
 *
 *  Created on: Jun 24, 2024
 *      Author: ParsaMalekpour
 */

#ifndef SRC_LIBS_FILTERS_H_
#define SRC_LIBS_FILTERS_H_

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

typedef struct{
	float alpha;
	float out;
}Filter_EMA;

float Filter_EMA_Init(Filter_EMA *filter, float alpha);
void Filter_EMA_SetAlpha(Filter_EMA *filter, float alpha);
float Filter_EMA_Updata(Filter_EMA *filter, float input);


#endif /* SRC_LIBS_FILTERS_H_ */
