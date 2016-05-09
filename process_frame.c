/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define IMG_SIZE NUM_COLORS*OSC_CAM_MAX_IMAGE_WIDTH*OSC_CAM_MAX_IMAGE_HEIGHT

const int nc = OSC_CAM_MAX_IMAGE_WIDTH;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT;

int16 imgDx[IMG_SIZE];
int16 imgDy[IMG_SIZE];

int TextColor;

float bgrImg[IMG_SIZE];

const float avgFac = 0.95;

/* skip pixel at border */
const int Border = 2;

/* after this number of steps object is set to background */
const int frgLimit = 100;

/* minimum size of objects (sum of all pixels) */
const int MinArea = 500;

struct OSC_VIS_REGIONS ImgRegions;/* these contain the foreground objects */

void ChangeDetection();
void CalculateAngles();
void Erode_3x3(int InIndex, int OutIndex);
void Dilate_3x3(int InIndex, int OutIndex);
void DetectRegions();
void DrawBoundingBoxes();

void ResetProcess() {
}

void ProcessFrame() {
	//initialize counters
	if (data.ipc.state.nStepCounter == 1) {

	} else {

		ChangeDetection();

		CalculateAngles();

		Erode_3x3(THRESHOLD, INDEX0);
		Dilate_3x3(INDEX0, THRESHOLD);

		DetectRegions();

		DrawBoundingBoxes();
	}
}

void ChangeDetection() {
	int r, c;
	//set result buffer to zero
	memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);
	//loop over the rows
	for (r = Border * nc; r < (nr - Border) * nc; r += nc) {
		//loop over the columns
		for (c = Border; c < (nc - Border); c++) {
			unsigned char* p = &data.u8TempImage[SENSORIMG][r + c];

			/* implement Sobel filter in x-direction */
			int32 dx = -(int32) *(p - nc - 1) + (int32) *(p - nc + 1)
				       - 2 * (int32) *(p - 1) + 2 * (int32) *(p + 1)
					   - (int32) *(p + nc - 1) + (int32) *(p + nc + 1);

			int32 dy = -(int32) *(p-nc-1) - 2* (int32) *(p-nc) - (int32) *(p-nc+1)
					   +(int32) *(p+nc-1) + 2* (int32) *(p+nc) + (int32) *(p+nc+1);

			/* check if norm is larger than threshold */
			int32 df2 = dx * dx + dy * dy;
			int32 thr2 = data.ipc.state.nThreshold * data.ipc.state.nThreshold;
			if (df2 > thr2) { //avoid square root
				//set pixel value to 255 in THRESHOLD image for gui
				data.u8TempImage[THRESHOLD][r + c] = 255;
			}
			//store derivatives (int16 is enough)
			imgDx[r + c] = (int16) dx;
			imgDy[r + c] = (int16) dy;
			//possibility to visualize data
			data.u8TempImage[BACKGROUND][r + c] = (uint8) MAX(0,
					MIN(255, 128+dx));
		}
	}
}

void CalculateAngles(void) {
	int o, c, i;
	int bins[] = {0, 45, 90, 135};
	int binCnt[sizeof(bins) / sizeof(int)];
	int binCntMax, iBinMax;

	//loop over objects
	for(o = 0; o < ImgRegions.noOfObjects; o++) {

		//get pointer to root run of current object
		struct OSC_VIS_REGIONS_RUN* currentRun = ImgRegions.objects[o].root;

		// reset bin counters
		for(i = 0; i < (sizeof(bins) / sizeof(int)); i++) {
			binCnt[i] = 0;
		}

		//loop over runs of current object
		do {

			//loop over pixel of current run
			for(c = currentRun->startColumn; c <= currentRun->endColumn; c++) {
				int r = currentRun->row;

				//processing angle for individual pixel at row r and column c
				double a = atan2(imgDy[r*nc+c], imgDx[r*nc+c]) * 180 / M_PI;

				// make clipping
				if(a < 0) a += 180;

				// do binning
				if((a <= 22.5) | (a > 157.5)) 	binCnt[0]++;
				if((a > 22.5)  & (a <= 67.5)) 	binCnt[1]++;
				if((a > 67.5)  & (a <= 112.5)) 	binCnt[2]++;
				if((a > 112.5) & (a <= 157.5)) 	binCnt[3]++;
			}

			currentRun = currentRun->next; //get net run of current object
		}  while(currentRun != NULL); //end of current object }

		// find index of most frequented bin
		binCntMax = 0; iBinMax = 0;
		for(i = 0; i < (sizeof(bins) / sizeof(int)); i++) {

			// find maximum
			if(binCnt[i] > binCntMax) {
				binCntMax = binCnt[i];	// set current bin as maximum
				iBinMax = i;			// save index of biggest bin
			}
		}
		printf("Region %i: %i\n", o, bins[iBinMax]);

		// find center
		char buffer [8];
		uint len = sprintf(buffer, "%d deg", bins[iBinMax]);
		uint16 cX = ImgRegions.objects[o].centroidX - 4*strlen(buffer);
		uint16 cY = ImgRegions.objects[o].centroidY - 6;
		DrawString(cX, cY, len, LARGE, BLUE, buffer);

	}
}

void Erode_3x3(int InIndex, int OutIndex) {
	int c, r;

	for (r = Border * nc; r < (nr - Border) * nc; r += nc) {
		for (c = Border; c < (nc - Border); c++) {
			unsigned char* p = &data.u8TempImage[InIndex][r + c];
			data.u8TempImage[OutIndex][r + c] = *(p - nc - 1) & *(p - nc)
					& *(p - nc + 1) & *(p - 1) & *p & *(p + 1) & *(p + nc - 1)
					& *(p + nc) & *(p + nc + 1);
		}
	}
}

void Dilate_3x3(int InIndex, int OutIndex) {
	int c, r;

	for (r = Border * nc; r < (nr - Border) * nc; r += nc) {
		for (c = Border; c < (nc - Border); c++) {
			unsigned char* p = &data.u8TempImage[InIndex][r + c];
			data.u8TempImage[OutIndex][r + c] = *(p - nc - 1) | *(p - nc)
					| *(p - nc + 1) | *(p - 1) | *p | *(p + 1) | *(p + nc - 1)
					| *(p + nc) | *(p + nc + 1);
		}
	}
}

void DetectRegions() {
	struct OSC_PICTURE Pic;
	int i;

	//set pixel value to 1 in INDEX0 because the image MUST be binary (i.e. values of 0 and 1)
	for (i = 0; i < IMG_SIZE; i++) {
		data.u8TempImage[INDEX0][i] = data.u8TempImage[THRESHOLD][i] ? 1 : 0;
	}

	//wrap image INDEX0 in picture struct
	Pic.data = data.u8TempImage[INDEX0];
	Pic.width = nc;
	Pic.height = nr;
	Pic.type = OSC_PICTURE_BINARY;

	//now do region labeling and feature extraction
	OscVisLabelBinary(&Pic, &ImgRegions);
	OscVisGetRegionProperties(&ImgRegions);
}

void DrawBoundingBoxes() {
	uint16 o;
	for (o = 0; o < ImgRegions.noOfObjects; o++) {
		if (ImgRegions.objects[o].area > MinArea) {
			DrawBoundingBox(ImgRegions.objects[o].bboxLeft,
					ImgRegions.objects[o].bboxTop,
					ImgRegions.objects[o].bboxRight,
					ImgRegions.objects[o].bboxBottom, false, BLUE);
		}
	}
}
