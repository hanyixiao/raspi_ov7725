#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "defines.h"
#include "tables.h"
#include "savePng.h"
#include "bilinear.h"
#include "pixels.h"
#include "adaptive.h"
void img_init(int height,int width);
int bytesPerPixel(uint8_t alg);
void MalvarDemosaic(float *Output, 
                    const float *Input, 
                    int Width, int Height,
                    int RedX, int RedY);
void deBayerHQl(uint8_t *in,uint8_t * out);
void deBayerSSDD(uint8_t * in,uint8_t * out);
void deBayerQ(uint8_t * in,uint8_t * out);
void deBayerN(uint8_t * in,uint8_t * out);
uint8_t readImg(uint32_t numf,uint16_t offset,uint8_t * dat,uint8_t alg,char * fileName);
void rgb565torgb888(uint8_t * in,uint8_t * out);
uint8_t processImg(uint8_t * in,uint8_t * out,uint32_t numf,uint8_t alg,uint16_t offset,int sqrtUse,int sineUse,
char * fileName,int inDat);
void avgF(uint_fast32_t numf,uint8_t * inout);
