/*
 * regio.h
 *
 *  Created on: Jun 19, 2015
 *      Author: rsalvi
 */

#ifndef REGIO_H_
#define REGIO_H_

#include <stdio.h>

int regio_wr32(unsigned int addr, unsigned int  data, int verbose);
int regio_rd32(unsigned int addr, unsigned int *data, int verbose);

#endif /* REGIO_H_ */
