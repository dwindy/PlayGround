//
// Created by xin on 18/08/22.
//

#ifndef LIDARVIEWER_LSD_MERGE_H
#define LIDARVIEWER_LSD_MERGE_H
#include "opencv2/line_descriptor/descriptor.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;
using namespace cv::line_descriptor;
using namespace std;
/* merging 360/3.14 = 0.008722222 rad per degree*/
const float degreeThres = 0.008722222 * 5;
const float distanceThres = 25;
const float lineLength = 30;
/*
 * dealing with the case that two keylines overlap.
 * since those two keylines are close,
 * the merged line should be those two endpoints that far-est to eachother.
 * Still, merge line2 into line1
 */
void mergeKeyLineCaseSSEE(KeyLine &line1, const KeyLine &line2);

/*
 * dealing with the case that two keylines overlap.
 * since those two keylines are close,
 * the merged line should be those two endpoints that far-est to each other.
 * Still, merge line2 into line1
 */
void mergeKeyLine(KeyLine &line1, const KeyLine &line2);

void mergeKeyLine2(KeyLine &line1, const KeyLine &line2);

void mergeKeyLine1(vector<KeyLine> &keylines, vector<bool> &keylineMergeFlags);

#endif //LIDARVIEWER_LSD_MERGE_H
