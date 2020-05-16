/*
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
//to compile just run make
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "defines.h"
#include "tables.h"
#include "savePng.h"
#include "bilinear.h"
#include "pixels.h"
#include "adaptive.h"
#include "convert.h"
#include "yuv.h"
uint32_t img_w=640;
uint32_t img_w_2=1280;
uint32_t img_w_3=1920;
uint32_t img_h=480;
uint32_t img_wo=640;
uint32_t img_ho=480;
char * buf;
int bytesPerPixel(uint8_t alg)
{
	return (alg<=BPP2AMT)?2:1;
}

void MalvarDemosaic(float *Output, const float *Input, int Width, int Height,int RedX, int RedY){//I do not take credit for this function the code is from http://www.ipol.im/pub/art/2011/g_mhcd/
	const int BlueX = 1 - RedX;
	const int BlueY = 1 - RedY;
	float *OutputRed = Output;
	float *OutputGreen = Output + Width*Height;
	float *OutputBlue = Output + 2*Width*Height;
	/* Neigh holds a copy of the 5x5 neighborhood around the current point */
	float Neigh[5][5];
	/* NeighPresence is used for boundary handling.  It is set to 0 if the 
	   neighbor is beyond the boundaries of the image and 1 otherwise. */
	int NeighPresence[5][5];
	int i, j, x, y, nx, ny;


	for(y = 0, i = 0; y < Height; y++)
	{
		for(x = 0; x < Width; x++, i++)
		{
			/* 5x5 neighborhood around the point (x,y) is copied into Neigh */
			for(ny = -2, j = x + Width*(y - 2); ny <= 2; ny++, j += Width)
			{
				for(nx = -2; nx <= 2; nx++)
				{
					if(0 <= x + nx && x + nx < Width 
							&& 0 <= y + ny && y + ny < Height)
					{
						Neigh[2 + nx][2 + ny] = Input[j + nx];
						NeighPresence[2 + nx][2 + ny] = 1;
					}
					else
					{
						Neigh[2 + nx][2 + ny] = 0;
						NeighPresence[2 + nx][2 + ny] = 0;
					}
				}
			}

			if((x & 1) == RedX && (y & 1) == RedY)
			{
				/* Center pixel is red */
				OutputRed[i] = Input[i];
				OutputGreen[i] = (2*(Neigh[2][1] + Neigh[1][2]
							+ Neigh[3][2] + Neigh[2][3])
						+ (NeighPresence[0][2] + NeighPresence[4][2]
							+ NeighPresence[2][0] + NeighPresence[2][4])*Neigh[2][2] 
						- Neigh[0][2] - Neigh[4][2]
						- Neigh[2][0] - Neigh[2][4])
					/ (2*(NeighPresence[2][1] + NeighPresence[1][2]
								+ NeighPresence[3][2] + NeighPresence[2][3]));
				OutputBlue[i] = (4*(Neigh[1][1] + Neigh[3][1]
							+ Neigh[1][3] + Neigh[3][3]) +
						3*((NeighPresence[0][2] + NeighPresence[4][2]
								+ NeighPresence[2][0] + NeighPresence[2][4])*Neigh[2][2] 
							- Neigh[0][2] - Neigh[4][2]
							- Neigh[2][0] - Neigh[2][4])) 
					/ (4*(NeighPresence[1][1] + NeighPresence[3][1]
								+ NeighPresence[1][3] + NeighPresence[3][3]));
			}
			else if((x & 1) == BlueX && (y & 1) == BlueY)
			{
				/* Center pixel is blue */
				OutputBlue[i] = Input[i];
				OutputGreen[i] = (2*(Neigh[2][1] + Neigh[1][2]
							+ Neigh[3][2] + Neigh[2][3])
						+ (NeighPresence[0][2] + NeighPresence[4][2]
							+ NeighPresence[2][0] + NeighPresence[2][4])*Neigh[2][2] 
						- Neigh[0][2] - Neigh[4][2]
						- Neigh[2][0] - Neigh[2][4])
					/ (2*(NeighPresence[2][1] + NeighPresence[1][2]
								+ NeighPresence[3][2] + NeighPresence[2][3]));
				OutputRed[i] = (4*(Neigh[1][1] + Neigh[3][1]
							+ Neigh[1][3] + Neigh[3][3]) +
						3*((NeighPresence[0][2] + NeighPresence[4][2]
								+ NeighPresence[2][0] + NeighPresence[2][4])*Neigh[2][2] 
							- Neigh[0][2] - Neigh[4][2]
							- Neigh[2][0] - Neigh[2][4])) 
					/ (4*(NeighPresence[1][1] + NeighPresence[3][1]
								+ NeighPresence[1][3] + NeighPresence[3][3]));
			}
			else
			{
				/* Center pixel is green */
				OutputGreen[i] = Input[i];

				if((y & 1) == RedY)
				{
					/* Left and right neighbors are red */
					OutputRed[i] = (8*(Neigh[1][2] + Neigh[3][2])
							+ (2*(NeighPresence[1][1] + NeighPresence[3][1]
									+ NeighPresence[0][2] + NeighPresence[4][2]
									+ NeighPresence[1][3] + NeighPresence[3][3])
								- NeighPresence[2][0] - NeighPresence[2][4])*Neigh[2][2]
							- 2*(Neigh[1][1] + Neigh[3][1]
								+ Neigh[0][2] + Neigh[4][2]
								+ Neigh[1][3] + Neigh[3][3])
							+ Neigh[2][0] + Neigh[2][4]) 
						/ (8*(NeighPresence[1][2] + NeighPresence[3][2]));
					OutputBlue[i] = (8*(Neigh[2][1] + Neigh[2][3])
							+ (2*(NeighPresence[1][1] + NeighPresence[3][1]
									+ NeighPresence[2][0] + NeighPresence[2][4]
									+ NeighPresence[1][3] + NeighPresence[3][3])
								- NeighPresence[0][2] - NeighPresence[4][2])*Neigh[2][2]
							- 2*(Neigh[1][1] + Neigh[3][1]
								+ Neigh[2][0] + Neigh[2][4]
								+ Neigh[1][3] + Neigh[3][3])
							+ Neigh[0][2] + Neigh[4][2]) 
						/ (8*(NeighPresence[2][1] + NeighPresence[2][3]));
				}
				else
				{
					/* Left and right neighbors are blue */
					OutputRed[i] = (8*(Neigh[2][1] + Neigh[2][3])
							+ (2*(NeighPresence[1][1] + NeighPresence[3][1]
									+ NeighPresence[2][0] + NeighPresence[2][4]
									+ NeighPresence[1][3] + NeighPresence[3][3])
								- NeighPresence[0][2] - NeighPresence[4][2])*Neigh[2][2]
							- 2*(Neigh[1][1] + Neigh[3][1]
								+ Neigh[2][0] + Neigh[2][4]
								+ Neigh[1][3] + Neigh[3][3])
							+ Neigh[0][2] + Neigh[4][2]) 
						/ (8*(NeighPresence[2][1] + NeighPresence[2][3]));
					OutputBlue[i] = (8*(Neigh[1][2] + Neigh[3][2])
							+ (2*(NeighPresence[1][1] + NeighPresence[3][1]
									+ NeighPresence[0][2] + NeighPresence[4][2]
									+ NeighPresence[1][3] + NeighPresence[3][3])
								- NeighPresence[2][0] - NeighPresence[2][4])*Neigh[2][2]
							- 2*(Neigh[1][1] + Neigh[3][1]
								+ Neigh[0][2] + Neigh[4][2]
								+ Neigh[1][3] + Neigh[3][3])
							+ Neigh[2][0] + Neigh[2][4]) 
						/ (8*(NeighPresence[1][2] + NeighPresence[3][2]));
				}
			}
		}
	}
}

void deBayerHQl(uint8_t *in,uint8_t * out){
	//from https://research.microsoft.com/en-us/um/people/lhe/papers/icassp04.demosaicing.pdf
	float * inf=malloc(img_w*img_h*sizeof(float));
	float * outf=malloc(img_w_3*img_h*sizeof(float));
	uint32_t z;
	for(z=0;z<img_w*img_h;++z)
		inf[z]=in[z];
	MalvarDemosaic(outf,inf,img_w,img_h,1,1);
	for(z=0;z<img_w_3*img_h;z+=3){
		out[z]=CLIP(outf[z/3]);
		out[z+1]=CLIP(outf[(z/3)+(img_w*img_h)]);
		out[z+2]=CLIP(outf[(z/3)+(img_w_2*img_h)]);
		
	}
	free(inf);
	free(outf);
}
void deBayerV(uint8_t * in,uint8_t * out){
//from http://www.eie.polyu.edu.hk/~enyhchan/J-TIP-CDemosaicking_using_VarCD.pdf
	
}

void deBayerSSDD(uint8_t * in,uint8_t * out){
//from http://www.ipol.im/pub/art/2011/bcms-ssdd/
	//This is a two pass method it first calls Adaptive_Color_Plane_Interpolation then improves the results
	uint32_t x,y;
	for (y=0;y<img_h;y+=2){
		for (x=0;x<img_w;x+=2){
			/*B Gb
			  Gr R*/
			out[x*3]=in[x];
		}
	}
}
void deBayerQ(uint8_t * in,uint8_t * out){
//generates quater resolution but pixel has real RGB value at each location
	uint32_t x,y;
	for (y=0;y<img_ho;++y){
		for (x=0;x<img_w;x+=2){
			out[(x/2*3)]=in[x+1+img_w];//red
			out[(x/2*3)+1]=(in[x+1]+in[x+img_w])/2;//green
			out[(x/2*3)+2]=in[x];//blue
		}
		out+=img_wo*3;
		in+=img_w*2;
	}
}
void deBayerN(uint8_t * in,uint8_t * out){
	uint32_t x,y;
	for (y=0;y<img_h;y+=2){
		for (x=0;x<img_w;x+=2){
			/* Correct table: (It is slightly different than the most common R G G B tables)
			 B Gb
			 Gr R
			*/ 
			out[(x*3)]=in[x+1+img_w];//red 
			out[(x*3)+1]=in[x+1];//green
			out[(x*3)+2]=in[x];//blue
			
			out[(x*3)+3]=in[x+1+img_w];//red 
			out[(x*3)+4]=in[x+1];//green		
			out[(x*3)+5]=in[x];//blue

			out[((x+img_w)*3)]=in[x+1+img_w];//red 
			out[((x+img_w)*3)+1]=in[x+img_w];//green
			out[((x+img_w)*3)+2]=in[x];//blue

			out[((x+img_w)*3)+3]=in[x+1+img_w];//red 
			out[((x+img_w)*3)+4]=in[x+img_w];//green
			out[((x+img_w)*3)+5]=in[x];//blue
		}
		out+=img_w*6;
		in+=img_w*2;
	}
}
uint8_t readImg(uint32_t numf,uint16_t offset,uint8_t * dat,uint8_t alg,char * fileName){
	FILE * myfile;
	if(fileName==0){
		sprintf(buf,"%d.RAW",numf);
		myfile = fopen(buf,"rb");
	}else
		myfile = fopen(fileName,"rb");
	if (myfile==0){
		if(fileName==0)
			printf("Cannot open file %s\n",buf);
		else
			printf("Cannot open file %s\n",fileName);
		return 1;
	}
	if (offset!=0)
		fseek(myfile,offset,SEEK_SET);
	int error=0;
	error=fread(dat,1,(img_w*img_h*bytesPerPixel(alg))-offset,myfile);
	fclose(myfile);
	if(error==0){
		puts("Error read 0 bytes");
		exit(1);
	}
	return 0;
}
void rgb565torgb888(uint8_t * in,uint8_t * out){
	uint32_t xy=img_w*img_h;
	uint16_t * ins=(uint16_t *)in;
	while(xy--){
		// R R R R R G G G   G G G B B B B B
		*out++=((*in++)>>3)*255/31;
		*out++=(((*ins++)>>5)&63)*255/63;
		*out++=((*in++)&31)*255/31;
	}
}
/* This function handles the processing of images
 * Paramaters:
 * in - must point to vaild memory enough to store img_w*img_h or img_w*img_h*2 depening on algorithm
 * out - must point to vaild memory enough to store img_wo*img_ho*3
 * numf - if fineName is set to 0 then this number will be used to generate a filename
 * alg - the algorithm being used to convert the image data
 * offset - how many bytes to skip when reading the file
 * sqrtUse do you want to use sqrt based brightness increase table
 * sineUse do you want to use sine based brightness increase table
 * filename if specified the file will be loaded instead of using numf
 * inDat do you want to skip reading a file and instead use data already present in *in */
uint8_t processImg(uint8_t * in,uint8_t * out,uint32_t numf,uint8_t alg,uint16_t offset,int sqrtUse,int sineUse,char * fileName,int inDat){
	if(!inDat){
		if(readImg(numf,offset,in,alg,fileName))
			return 1;
	}
	switch (alg){
	case ALG_YUV_0:
	case ALG_YUV_1:
	case ALG_YUV_2:
	case ALG_YUV_3:
		yuv2rgb(in,out,alg,img_w,img_h);
	break;
	case ALG_DEBAYER_HQ:
		deBayerHQl(in,out);
	break;
	case ALG_RGB565:
		rgb565torgb888(in,out);
	break;
	case ALG_DEBAYER_BL:
		deBayerBL(in,out,img_w,img_h);
	break;
	case ALG_DEBAYER_Q:
		deBayerQ(in,out);//causes low resolution but it's like have a 3cmos sensor or foveon sensor
	break;
	case ALG_DEBAYER_N:
		deBayerN(in,out);//nearest neighboor low quality but fast
	break;
	case ALG_ADAPTIVE:
		Adaptive_Color_Plane_Interpolation(in,out,img_w,img_h);
	break;
	default:
		puts("You must pick a valid algorithm to save the image as");
		return 1;
	}
	if(sineUse){
		uint32_t l=img_wo*img_ho*3;
		while(l--){
			*out=sine_tab[*out];
			++out;
		}
		out-=img_wo*img_ho*3;
	}
	if(sqrtUse){
		uint32_t l=img_wo*img_ho*3;
		while(l--){
			*out=sqrt_tab[*out];
			++out;
		}
	}
	return 0;
}
void avgF(uint_fast32_t numf,uint8_t * inout){
	uint32_t * temp=malloc(img_w*img_h*sizeof(uint32_t));
	uint_fast32_t xy;
	uint_fast32_t nl;
	for(xy=0;xy<img_w*img_h;++xy)
		temp[xy]=inout[xy];
	for(nl=img_w*img_h;nl<numf*img_w*img_h;nl+=img_w*img_h){
		//printf("Adding %d\n",nl/img_w/img_h/3);
		for (xy=0;xy<=img_w*img_h*3;xy++)
			temp[xy]+=inout[xy+nl];
	}
	for(xy=0;xy<img_w*img_h;++xy)
		temp[xy]/=numf;
	for(xy=0;xy<img_w*img_h;++xy)
		inout[xy]=temp[xy];
	free(temp);
}
/****this must be use the first******/
void img_init(int height,int width)
{
	img_h=height;
	img_w=width;	
}