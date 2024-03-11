/*
 * snake_utils.h
 *
 *  Created on: 13 de mai de 2023
 *      Author: Paulo
 */

#include "NOKIA5110_fb.h"

#ifndef INC_SNAKE_UTILS_H_
#define INC_SNAKE_UTILS_H_

#define MAX_SIZE 100
#define INITIAL_SIZE 2

typedef struct cobra_t {
	int points;
	uint32_t size;
	uint32_t node_x[MAX_SIZE];
	uint32_t node_y[MAX_SIZE];
	uint32_t direction;
	struct pontos_t snake_coordinates; // basicamente um buffer
} cobra_t;

typedef enum difficulty{easy, med, hard} diff_t;

#endif /* INC_SNAKE_UTILS_H_ */
