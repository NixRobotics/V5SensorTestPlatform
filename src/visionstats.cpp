#include "vex.h"
#include "visionstats.h"

using namespace vex;

tVisionSample visionSamples[2][SAMPLE_WINDOW];
int curVisionSample[2] = {0, 0};
tVisionStats visionStats[2];

void vsAddSample(int idx, int count)
{
    visionSamples[idx][curVisionSample[idx]].count = count;
}

void vsAddSample(int idx, int count, int cx, int cy, int size, int d1, int d2, int d3)
{
    visionSamples[idx][curVisionSample[idx]].count = count;
    visionSamples[idx][curVisionSample[idx]].x = cx;
    visionSamples[idx][curVisionSample[idx]].y = cy;           
    visionSamples[idx][curVisionSample[idx]].size = size;           
    visionSamples[idx][curVisionSample[idx]].dist1 = d1;
    visionSamples[idx][curVisionSample[idx]].dist2 = d2;
    visionSamples[idx][curVisionSample[idx]].dist3 = d3;    
}

bool vsUpdateSample(int idx)
{
    curVisionSample[idx] = curVisionSample[idx] + 1;
    if (curVisionSample[idx] >= SAMPLE_WINDOW) {
      curVisionSample[idx] = 0;
      return true;
    }
    return false;
}

void calcVisionStats(int idx, int iColRed, int iColGreen, int iColBlue, int iColHue, float fColSat)
{
  int validSamples = 0;
  int32_t xTotal = 0;
  int32_t yTotal = 0;
  int32_t sizeTotal = 0;
  int32_t countTotal = 0;
  int32_t d1Total = 0;
  int32_t d2Total = 0;
  int32_t d3Total = 0;
  int maxSize = 0;

  for (int i = 0; i < SAMPLE_WINDOW; i++) {
    countTotal += visionSamples[idx][i].count;
    if (visionSamples[idx][i].count > 0) {
      validSamples++;
      xTotal += visionSamples[idx][i].x;
      yTotal += visionSamples[idx][i].y;
      sizeTotal += visionSamples[idx][i].size;
      d1Total += visionSamples[idx][i].dist1;
      d2Total += visionSamples[idx][i].dist2;
      d3Total += visionSamples[idx][i].dist3;
      if (visionSamples[idx][i].size > maxSize) maxSize = visionSamples[idx][i].size;
    } else {
    }
  }

  float xMean = (float) xTotal / (float) validSamples;
  float yMean = (float) yTotal / (float) validSamples;
  double sizeMean = (double) sizeTotal / (double) validSamples;
  float d1Mean = (float) d1Total / (float) validSamples;
  float d2Mean = (float) d2Total / (float) validSamples;
  float d3Mean = (float) d3Total / (float) validSamples;
  float countMean = (float) countTotal / (float) SAMPLE_WINDOW;

  float xVar = 0.0;
  float yVar = 0.0;
  double sizeVar = 0.0;
  float countVar = 0.0;

  for (int i = 0; i < SAMPLE_WINDOW; i++) {
    countVar += ((float) visionSamples[idx][i].count - countMean) * ((float) visionSamples[idx][i].count - countMean);
    if (visionSamples[idx][i].count > 0) {
      xVar += ((float) visionSamples[idx][i].x - xMean) * ((float) visionSamples[idx][i].x - xMean);
      yVar += ((float) visionSamples[idx][i].y - yMean) * ((float) visionSamples[idx][i].y - yMean);
      sizeVar += ((double) visionSamples[idx][i].size - sizeMean) * ((double) visionSamples[idx][i].size - sizeMean);
    } else {
    }
  }

  xVar = fabsf(xVar / (float) validSamples);
  yVar = fabsf(yVar / (float) validSamples);
  sizeVar = fabs(sizeVar / (double) validSamples);
  countVar = fabsf(countVar / (float) SAMPLE_WINDOW);

  visionStats[idx].xMean = xMean;
  visionStats[idx].xVar = xVar;
  visionStats[idx].yMean = yMean;
  visionStats[idx].yVar = yVar;
  visionStats[idx].sizeMean = (float) sizeMean;
  visionStats[idx].sizeVar = (float) sizeVar;
  visionStats[idx].maxSize = maxSize;
  visionStats[idx].countMean = countMean;
  visionStats[idx].countVar = countVar;
  visionStats[idx].totalValid = validSamples;
  visionStats[idx].red = iColRed;
  visionStats[idx].green = iColGreen;
  visionStats[idx].blue = iColBlue;
  visionStats[idx].hue = iColHue;
  visionStats[idx].sat = fColSat;
  visionStats[idx].dist1 = (int) d1Mean;
  visionStats[idx].dist2 = (int) d2Mean;
  visionStats[idx].dist3 = (int) d3Mean;
}
