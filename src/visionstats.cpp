#include "vex.h"
#include "visionstats.h"

using namespace vex;

tVisionSample visionSamples[SAMPLE_WINDOW];
int curVisionSample = 0;
tVisionStats visionStats;

void vsAddSample(int count)
{
    visionSamples[curVisionSample].count = count;
}

void vsAddSample(int count, int cx, int cy, int size, int d1, int d2, int d3)
{
    visionSamples[curVisionSample].count = count;
    visionSamples[curVisionSample].x = cx;
    visionSamples[curVisionSample].y = cy;           
    visionSamples[curVisionSample].size = size;           
    visionSamples[curVisionSample].dist1 = d1;
    visionSamples[curVisionSample].dist2 = d2;
    visionSamples[curVisionSample].dist3 = d3;    
}

bool vsUpdateSample()
{
    curVisionSample++;
    if (curVisionSample >= SAMPLE_WINDOW) {
      curVisionSample = 0;
      return true;
    }
    return false;
}

void calcVisionStats(int iColRed, int iColGreen, int iColBlue, int iColHue, float fColSat)
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
    countTotal += visionSamples[i].count;
    if (visionSamples[i].count > 0) {
      validSamples++;
      xTotal += visionSamples[i].x;
      yTotal += visionSamples[i].y;
      sizeTotal += visionSamples[i].size;
      d1Total += visionSamples[i].dist1;
      d2Total += visionSamples[i].dist2;
      d3Total += visionSamples[i].dist3;
      if (visionSamples[i].size > maxSize) maxSize = visionSamples[i].size;
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
    countVar += ((float) visionSamples[i].count - countMean) * ((float) visionSamples[i].count - countMean);
    if (visionSamples[i].count > 0) {
      xVar += ((float) visionSamples[i].x - xMean) * ((float) visionSamples[i].x - xMean);
      yVar += ((float) visionSamples[i].y - yMean) * ((float) visionSamples[i].y - yMean);
      sizeVar += ((double) visionSamples[i].size - sizeMean) * ((double) visionSamples[i].size - sizeMean);
    } else {
    }
  }

  xVar = fabsf(xVar / (float) validSamples);
  yVar = fabsf(yVar / (float) validSamples);
  sizeVar = fabs(sizeVar / (double) validSamples);
  countVar = fabsf(countVar / (float) SAMPLE_WINDOW);

  visionStats.xMean = xMean;
  visionStats.xVar = xVar;
  visionStats.yMean = yMean;
  visionStats.yVar = yVar;
  visionStats.sizeMean = (float) sizeMean;
  visionStats.sizeVar = (float) sizeVar;
  visionStats.maxSize = maxSize;
  visionStats.countMean = countMean;
  visionStats.countVar = countVar;
  visionStats.totalValid = validSamples;
  visionStats.red = iColRed;
  visionStats.green = iColGreen;
  visionStats.blue = iColBlue;
  visionStats.hue = iColHue;
  visionStats.sat = fColSat;
  visionStats.dist1 = (int) d1Mean;
  visionStats.dist2 = (int) d2Mean;
  visionStats.dist3 = (int) d3Mean;
}
