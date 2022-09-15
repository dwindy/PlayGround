//
// Created by xin on 18/08/22.
//
/*
 * dealing with the case that two keylines overlap.
 * since those two keylines are close,
 * the merged line should be those two endpoints that far-est to eachother.
 * Still, merge line2 into line1
 */

#include "LSD_merge.h"

using namespace cv;
using namespace cv::line_descriptor;
using namespace std;
void mergeKeyLineCaseSSEE(KeyLine &line1, const KeyLine &line2) {
    float SX1 = line1.startPointX, SY1 = line1.startPointY, SX2 = line2.startPointX, SY2 = line2.startPointY;
    float EX1 = line1.endPointX, EY1 = line1.endPointY, EX2 = line2.endPointX, EY2 = line2.endPointY;
    float distanceS1E1 = (SX1 - EX1) * (SX1 - EX1) + (SY1 - EY1) * (SY1 - EY1);
    float distanceS1S2 = (SX1 - SX2) * (SX1 - SX2) + (SY1 - SY2) * (SY1 - SY2);
    float distanceS1E2 = (SX1 - EX2) * (SX1 - EX2) + (SY1 - EY2) * (SY1 - EY2);
    float distanceE1S2 = (EX1 - SX2) * (EX1 - SX2) + (EY1 - SY2) * (EY1 - SY2);
    float distanceE1E2 = (EX1 - EX2) * (EX1 - EX2) + (EY1 - EY2) * (EY1 - EY2);
    float distanceS2E2 = (SX2 - EX2) * (SX2 - EX2) + (SY2 - EY2) * (SY2 - EY2);
    float maxNumber = distanceS1E1;
    if (maxNumber < distanceS1S2)
        maxNumber = distanceS1S2;
    if (maxNumber < distanceS1E2)
        maxNumber = distanceS1E2;
    if (maxNumber < distanceE1S2)
        maxNumber = distanceE1S2;
    if (maxNumber < distanceE1E2)
        maxNumber = distanceE1E2;
    if (maxNumber < distanceS2E2)
        maxNumber = distanceS2E2;
    if (maxNumber == distanceS1E1) {
        line1.startPointX = SX1;
        line1.startPointY = SY1;
        line1.endPointX = EX1;
        line1.endPointY = EY1;
    }
    if (maxNumber == distanceS1S2) {
        line1.startPointX = SX1;
        line1.startPointY = SY1;
        line1.endPointX = SX2;
        line1.endPointY = SY2;
    }
    if (maxNumber == distanceS1E2) {
        line1.startPointX = SX1;
        line1.startPointY = SY1;
        line1.endPointX = EX2;
        line1.endPointY = EY2;
    }
    if (maxNumber == distanceE1S2) {
        line1.startPointX = SX2;//EX1;
        line1.startPointY = SY2;//EY1;
        line1.endPointX = EX1;//SX2;
        line1.endPointY = EY1;//SY2;
    }
    if (maxNumber == distanceE1E2) {
        line1.startPointX = EX1;
        line1.startPointY = EY1;
        line1.endPointX = EX2;
        line1.endPointY = EY2;
    }
    if (maxNumber == distanceS2E2) {
        line1.startPointX = SX2;
        line1.startPointY = SY2;
        line1.endPointX = EX2;
        line1.endPointY = EY2;
    }
    //merge others
    line1.numOfPixels += line2.numOfPixels;
    line1.angle = (line1.angle + line2.angle) / 2;
    line1.lineLength = sqrt((line1.endPointX - line1.startPointX) * (line1.endPointX - line1.startPointX) +
                            (line1.endPointY - line1.startPointY) * (line1.endPointY - line1.startPointY));
    line1.response += line2.response;
    line1.size = (line1.startPointX - line1.endPointX) * (line1.startPointY - line1.endPointY);
}

/*
 * dealing with the case that two keylines overlap.
 * since those two keylines are close,
 * the merged line should be those two endpoints that far-est to each other.
 * Still, merge line2 into line1
 */
void mergeKeyLine2(KeyLine &line1, const KeyLine &line2) {

    float SX1 = line1.startPointX, SY1 = line1.startPointY, SX2 = line2.startPointX, SY2 = line2.startPointY;
    float EX1 = line1.endPointX, EY1 = line1.endPointY, EX2 = line2.endPointX, EY2 = line2.endPointY;
    //to find out the farest points
    float distanceS1E1 = (SX1 - EX1) * (SX1 - EX1) + (SY1 - EY1) * (SY1 - EY1);
    float distanceS1S2 = (SX1 - SX2) * (SX1 - SX2) + (SY1 - SY2) * (SY1 - SY2);
    float distanceS1E2 = (SX1 - EX2) * (SX1 - EX2) + (SY1 - EY2) * (SY1 - EY2);
    float distanceE1S2 = (EX1 - SX2) * (EX1 - SX2) + (EY1 - SY2) * (EY1 - SY2);
    float distanceE1E2 = (EX1 - EX2) * (EX1 - EX2) + (EY1 - EY2) * (EY1 - EY2);
    float distanceS2E2 = (SX2 - EX2) * (SX2 - EX2) + (SY2 - EY2) * (SY2 - EY2);
    float maxNumber = distanceS1E1;
    if (maxNumber < distanceS1S2)
        maxNumber = distanceS1S2;
    if (maxNumber < distanceS1E2)
        maxNumber = distanceS1E2;
    if (maxNumber < distanceE1S2)
        maxNumber = distanceE1S2;
    if (maxNumber < distanceE1E2)
        maxNumber = distanceE1E2;
    if (maxNumber < distanceS2E2)
        maxNumber = distanceS2E2;
    if (maxNumber == distanceS1E1) {
        line1.startPointX = SX1;
        line1.startPointY = SY1;
        line1.endPointX = EX1;
        line1.endPointY = EY1;
    }
    if (maxNumber == distanceS1S2) {
        line1.startPointX = SX1;
        line1.startPointY = SY1;
        line1.endPointX = SX2;
        line1.endPointY = SY2;
    }
    if (maxNumber == distanceS1E2) {
        line1.startPointX = SX1;
        line1.startPointY = SY1;
        line1.endPointX = EX2;
        line1.endPointY = EY2;
    }
    if (maxNumber == distanceE1S2) {
        line1.startPointX = SX2;//EX1;
        line1.startPointY = SY2;//EY1;
        line1.endPointX = EX1;//SX2;
        line1.endPointY = EY1;//SY2;
    }
    if (maxNumber == distanceE1E2) {
        line1.startPointX = EX1;
        line1.startPointY = EY1;
        line1.endPointX = EX2;
        line1.endPointY = EY2;
    }
    if (maxNumber == distanceS2E2) {
        line1.startPointX = SX2;
        line1.startPointY = SY2;
        line1.endPointX = EX2;
        line1.endPointY = EY2;
    }
    //merge others
    line1.numOfPixels += line2.numOfPixels; //not relyable because overlap
    line1.angle = (line1.angle + line2.angle) / 2;
    line1.lineLength = sqrt((line1.endPointX - line1.startPointX) * (line1.endPointX - line1.startPointX) +
                            (line1.endPointY - line1.startPointY) * (line1.endPointY - line1.startPointY));
    line1.response += line2.response;
    line1.size = (line1.startPointX - line1.endPointX) * (line1.startPointY - line1.endPointY);
}

/*
 * merge the primary keylines
 */
void mergeKeyLine1(vector<KeyLine> &keylines, vector<bool> &keylineMergeFlags){
    for (int i = 0; i < keylines.size(); i++) {
        if (keylineMergeFlags[i])//If this keyline has been merge into another
            continue;
        //looking for pairs
        for (int j = 0; j < keylines.size(); j++) {
            if (!keylineMergeFlags[j] && j != i) { //This keyline shouldn't be merged into another | and j!=i
                float degree1 = abs(keylines[i].angle) + abs(keylines[j].angle);
                float degree2 = abs(keylines[j].angle - keylines[i].angle);
                if ((abs(degree1 - 3.14) < degreeThres) || degree2 < degreeThres) {//angle < 5 degrees
                    float distanceSS = (keylines[i].startPointX - keylines[j].startPointX) *
                                       (keylines[i].startPointX - keylines[j].startPointX) +
                                       (keylines[i].startPointY - keylines[j].startPointY) *
                                       (keylines[i].startPointY - keylines[j].startPointY);
                    float distanceSE = (keylines[i].startPointX - keylines[j].endPointX) *
                                       (keylines[i].startPointX - keylines[j].endPointX) +
                                       (keylines[i].startPointY - keylines[j].endPointY) *
                                       (keylines[i].startPointY - keylines[j].endPointY);
                    float distanceES = (keylines[i].endPointX - keylines[j].startPointX) *
                                       (keylines[i].endPointX - keylines[j].startPointX) +
                                       (keylines[i].endPointY - keylines[j].startPointY) *
                                       (keylines[i].endPointY - keylines[j].startPointY);
                    float distanceEE = (keylines[i].endPointX - keylines[j].endPointX) *
                                       (keylines[i].endPointX - keylines[j].endPointX) +
                                       (keylines[i].endPointY - keylines[j].endPointY) *
                                       (keylines[i].endPointY - keylines[j].endPointY);
                    float midXi = (keylines[i].startPointX + keylines[i].endPointX) / 2;
                    float midYi = (keylines[i].startPointY + keylines[i].endPointY) / 2;
                    float midXj = (keylines[j].startPointX + keylines[j].endPointX) / 2;
                    float midYj = (keylines[j].startPointY + keylines[j].endPointY) / 2;
                    float distanceMid = (midXi - midXj) * (midXi - midXj) + (midYi - midYj) * (midYi - midYj);
                    bool SS = false, SE = false, ES = false, EE = false, Mid = false;
                    if (distanceSS < distanceThres)SS = true;
                    if (distanceSE < distanceThres)SE = true;
                    if (distanceEE < distanceThres)EE = true;
                    if (distanceES < distanceThres)ES = true;
                    if (distanceMid < distanceThres)Mid = true;
                    if (SS || SE || EE || ES || Mid) {//start or end points is close
                        cout << "keyline1 No." << keylines[i].class_id << " with angle " << keylines[i].angle
                             << " response " << keylines[i].response << " length :" << keylines[i].lineLength
                             << " start point " << keylines[i].startPointX << " " << keylines[i].startPointY
                             << " end point " << keylines[i].endPointX << " " << keylines[i].endPointY << endl;
                        cout << "merge with" << endl;
                        cout <<" degree1 "<<degree1<<" degree2 "<<degree2<<endl;
                        cout << "keyline2 No." << keylines[j].class_id << " with angle " << keylines[j].angle
                             << " response " << keylines[j].response << " length :" << keylines[j].lineLength
                             << " start point " << keylines[j].startPointX << " " << keylines[j].startPointY
                             << " end point " << keylines[j].endPointX << " " << keylines[j].endPointY << endl;
                        /* draw it on the image */
//                        if (keylines[j].octave == 0) {
//                            /* get extremes of line */
//                            Point pt1 = Point2f(keylines[j].startPointX, keylines[j].startPointY);
//                            Point pt2 = Point2f(keylines[j].endPointX, keylines[j].endPointY);
//                            /* draw line */
//                            line(output_bd_merge, pt1, pt2, Scalar(0, 255, 0), 3);
//                            imshow("BD lines merged", output_bd_merge);
//                            waitKey();
//                        }
                        mergeKeyLine2(keylines[i], keylines[j]);
                        keylineMergeFlags[j] = true;
                        cout << "result : " << endl;
                        cout << "keyline new1 No." << keylines[i].class_id << " with angle " << keylines[i].angle
                             << " response " << keylines[i].response << " length :" << keylines[i].lineLength
                             << " start point " << keylines[i].startPointX << " " << keylines[i].startPointY
                             << " end point " << keylines[i].endPointX << " " << keylines[i].endPointY << endl;
                        //j = 0; //reset j. maybe do not do this.

                        /* draw it on the image */
//                        if (keylines[i].octave == 0) {
//                            /* get extremes of line */
//                            Point pt1 = Point2f(keylines[i].startPointX, keylines[i].startPointY);
//                            Point pt2 = Point2f(keylines[i].endPointX, keylines[i].endPointY);
//                            /* draw line */
//                            line(output_bd_merge, pt1, pt2, Scalar(255, 255, 0), 3);
//                        }
//                        imshow("BD lines merged", output_bd_merge);
//                        waitKey();
                    }
                }
            }
        }
    }
}
