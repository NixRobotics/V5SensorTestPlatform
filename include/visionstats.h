#ifndef   VISIONSTATS_H
#define   VISIONSTATS_H

using namespace vex;

struct tVisionStats {
  int red, green, blue, hue;
  float sat;
  float xMean, xVar;
  float yMean, yVar;
  float sizeMean, sizeVar;
  int maxSize;
  float countMean, countVar;
  int totalValid;
  int dist1, dist2, dist3;
};

struct tVisionSample {
  int x, y;
  int size;
  int count;
  int dist1, dist2, dist3;
};

#define SAMPLE_WINDOW 100

void vsAddSample(int count, int cx, int cy, int size, int d1, int d2, int d3);
void vsAddSample(int count);
bool vsUpdateSample();
void calcVisionStats(int iColRed, int iColGreen, int iColBlue, int iColHue, float fCotSat);

extern tVisionStats visionStats;

#endif
