#ifndef __FILTERS_TYPE_HPP__
#define __FILTERS_TYPE_HPP__

#define IIR_SHIFT 8

typedef struct {
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;
    float delay_element_1;
    float delay_element_2;
} lpf2pData;

#endif