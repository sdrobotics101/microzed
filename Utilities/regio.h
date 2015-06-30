/*
 * regio.h
 *
 *  Created on: Jun 19, 2015
 *      Author: rsalvi
 */

#ifndef REGIO_H_
#define REGIO_H_

#include <stdint.h>

int regio_wr32(uint32_t addr, uint32_t  data, int verbose);
int regio_rd32(uint32_t addr, uint32_t *data, int verbose);

#endif /* REGIO_H_ */
