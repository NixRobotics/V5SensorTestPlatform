#include "vex.h"
#include "visionopt.h"

using namespace vex;

bool bIsVisionOpt = false;
int voRedMin = 100, voRedMax = 200, voRedStep = 25;
int voGreenMin = 100, voGreenMax = 200, voGreenStep = 25;
int voBlueMin = 0, voBlueMax = 100, voBlueStep = 25;
int voHueMin = 10, voHueMax = 20, voHueStep = 2;
float voSatMin = 0.1, voSatMax = 0.5, voSatStep = 0.1;
int voRedCur = -1, voGreenCur = -1, voBlueCur = -1; // 100, 150, 200, 100, 16, 0.2
int voHueCur = -1;
float voSatCur = -1.0;

bool voRun()
{
    if(!bIsVisionOpt) return false;

    if (voRedCur < 0) {
        voRedCur = voRedMin;
        voGreenCur = voGreenMin;
        voBlueCur = voBlueMin;
        voHueCur = voHueMin;
        voSatCur = voSatMin;
    } else {
        voRedCur += voRedStep;
        if (voRedCur > voRedMax) {
            voRedCur = voRedMin;
            voGreenCur += voGreenStep;
            if (voGreenCur > voGreenMax) {
                voGreenCur = voGreenMin;
                voBlueCur += voBlueStep;
                if (voBlueCur > voBlueMax) {
                    voBlueCur = voBlueMin;
                    voHueCur += voHueStep;
                    if (voHueCur > voHueMax) {
                        voHueCur = voHueMin;
                        voSatCur += voSatStep;
                        if (voSatCur > voSatMax) {
                            voRedCur = voRedMin;
                            voGreenCur = voRedMin;
                            voBlueCur = voBlueMin;
                            voHueCur = voHueMin;
                            voSatCur = voSatMin;
                        }
                    }
                }
            }
        }
    }

    return true;

}

bool voPrint(tVisionStats *visionStats)
{
    if (!bIsVisionOpt) return false;

    static bool first = true;
    if (first) {
        printf("valid, red, green, blue, hue, sat, x_mean, x_var, y_mean, y_var, size_mean, size_var, size_max, d1, d2, d3, count_mean, count_var\n");
        first = false;
    }
    printf("%d, %d, %d, %d, %d, %0.1f, %.2f, %0.2f, %.2f, %0.2f, %.2f, %0.2f, %d, %d,%d,%d, %.2f, %0.2f\n",
        visionStats->totalValid,
        visionStats->red, visionStats->green, visionStats->blue,
        visionStats->hue,
        visionStats->sat,
        visionStats->xMean, visionStats->xVar,
        visionStats->yMean, visionStats->yVar,
        visionStats->sizeMean, visionStats->sizeVar, visionStats->maxSize,
        visionStats->dist1, visionStats->dist2, visionStats->dist3,
        visionStats->countMean, visionStats->countVar);

    return true;

}
