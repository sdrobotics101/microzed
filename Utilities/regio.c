/*
 * regio.c
 *
 *  Created on: Jun 19, 2015
 *      Author: rsalvi
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>

#include "regio.h"

#define REGIO_FPGA_BASE       0x43c00000
#define REGIO_MAP_PAGESIZE    (16*64*1024)
#define REGIO_MAP_MASK        ((off_t) (REGIO_MAP_PAGESIZE-1))

int regio_wr32(uint32_t addr, uint32_t  data, int verbose) {

    int fd;
    int page_size, mapped_pages;
    void *pmem;
    void *vaddr;
    volatile uint32_t *reg_addr;
    off_t phy_addr;
    size_t phy_length;

    if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        printf("ERROR : failed to open /dev/mem\n");
        return (-1);
    }

    phy_addr     = REGIO_FPGA_BASE;
    phy_length   = REGIO_MAP_PAGESIZE;
    page_size 	 = getpagesize();
    mapped_pages = phy_length/page_size;

    if (verbose > 1) {
        printf("off_t size            : %lu\n", (long unsigned int)sizeof(off_t));
        printf("phy_addr & map_mask   : 0x%llx\n", (unsigned long long)(phy_addr & ~REGIO_MAP_MASK));
        printf("page size             : 0x%x (%d)\n", page_size, page_size);
        printf("num pages             : %d\n", mapped_pages);
    }

    pmem = mmap(0,                              /* start            */
                phy_length,                     /* length           */
                (PROT_READ | PROT_WRITE),       /* protection       */
                MAP_SHARED,                     /* flags            */
                fd,                             /* file descriptor  */
                phy_addr & ~REGIO_MAP_MASK);    /* offset           */

    if ((pmem == MAP_FAILED) || (pmem == NULL)) {
        printf("ERROR: failed to mmap /dev/mem\n");
        return (-1);
    }
    else {
        if (verbose > 1) {
            printf("pmem                  : 0x%lx\n", (long unsigned int)pmem);
            printf("pmem (ptr)            : %p\n", pmem);
        }
    }

    if (close(fd)) {
        printf("ERROR: failed to close /dev/mem\n");
        return (-1);
    }

    vaddr       = (void *)((char *) pmem + (phy_addr & REGIO_MAP_MASK));
    reg_addr    = ((volatile uint32_t *)((char *) vaddr + addr));
    *reg_addr   = data;

    if (verbose > 1) {
            printf("reg_addr              : 0x%08x\n", (uint32_t)reg_addr);
            printf("reg_data              : 0x%08x\n", data);
    }

    munmap(pmem, phy_length);

    if (verbose > 0)
        printf("HREG W 0x%08x 0x%08x\n", addr, data);

    return (0);
}

int regio_rd32(uint32_t addr, uint32_t *data, int verbose) {

    int fd;
    int page_size, mapped_pages;
    void *pmem;
    void *vaddr;
    volatile uint32_t *reg_addr;            
    uint32_t  reg_data;
    off_t phy_addr;
    size_t phy_length;

    if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        printf("ERROR : failed to open /dev/mem\n");
        return (-1);
    }

    phy_addr     = REGIO_FPGA_BASE;
    phy_length   = REGIO_MAP_PAGESIZE;
    page_size 	 = getpagesize();
    mapped_pages = phy_length/page_size;

    if (verbose > 1) {
        printf("off_t size            : %lu\n", (long unsigned int)sizeof(off_t));
        printf("phy_addr & map_mask   : 0x%llx\n", (unsigned long long)(phy_addr & ~REGIO_MAP_MASK));
        printf("page size             : 0x%x (%d)\n", page_size, page_size);
        printf("num pages             : %d\n", mapped_pages);
    }

    pmem = mmap(0,                                  /* start            */
                phy_length,                         /* length           */
                (PROT_READ | PROT_WRITE),           /* protection       */
                MAP_SHARED,                         /* flags            */
                fd,                                 /* file descriptor  */
                phy_addr & ~REGIO_MAP_MASK);        /* offset           */

    if ((pmem == MAP_FAILED) || (pmem == NULL)) {
        printf("ERROR: failed to mmap /dev/mem\n");
        return (-1);
    }
    else {
        if (verbose > 1) {
            printf("pmem                  : 0x%lx\n", (long unsigned int)pmem);
            printf("pmem (ptr)            : %p\n", pmem);
        }
    }

    if (close(fd)) {
        printf("ERROR: failed to close /dev/mem\n");
        return (-1);
    }

    vaddr       = (char *) pmem + (phy_addr & REGIO_MAP_MASK);
    reg_addr    = ((volatile uint32_t *)((char *) vaddr +  addr));

    reg_data    = *reg_addr;
    *data       = reg_data;

    if (verbose > 1) {
            printf("reg_addr              : 0x%08x\n", (uint32_t)reg_addr);
            printf("reg_data              : 0x%08x\n", reg_data);
    }

    munmap(pmem, phy_length);

    if (verbose > 0)
        printf("HREG R 0x%08x 0x%08x\n", addr, *data);

    return (0);
}


