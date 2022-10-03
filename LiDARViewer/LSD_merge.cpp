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

/*
 * pair up the LSD lines, keypoints and 2d lidar points
 */
void connectPointsLines(cv::Mat &im, vector<KeyLine> selectedKeyLSDLines, vector<point2d> &imgKeyPoint, vector<point2d> &LiDAR2d) {
    //todo this should be in a line class ---(y2-y1)x+(x1-x2)y+(x2y1-x1y2)=0
    vector<vector<float>> keyLineABCs;//store keylines in terms of Ax+By+C = 0;
    for (int i = 0; i < selectedKeyLSDLines.size(); i++) {
        float x1 = selectedKeyLSDLines[i].startPointX, y1 = selectedKeyLSDLines[i].startPointY;
        float x2 = selectedKeyLSDLines[i].endPointX, y2 = selectedKeyLSDLines[i].endPointY;
        float A = y2 - y1, B = x1 - x2, C = x2 * y1 - x1 * y2;
        vector<float> thisLine;
        thisLine.push_back(A);
        thisLine.push_back(B);
        thisLine.push_back(C);
        keyLineABCs.push_back(thisLine);
    }
    ///Step 1 pair up LSD lines and ORB keypoints
    vector<int> keyPt2LSD(imgKeyPoint.size(), -1);
    for (int i = 0; i < imgKeyPoint.size(); i++) {
        float x0 = imgKeyPoint[i].x, y0 = imgKeyPoint[i].y;
        float minDistance1 = 5, minDistance2 = 5, minDistance3 = 5;
        imgKeyPoint[i].index2line = -1;
        int index1 = -1, index2 = -1, index3 = -1;
        for (int j = 0; j < selectedKeyLSDLines.size(); j++) {
            ///Step 1.1 search for 3 lines
            ///point to line --- d = abs(Ax0+By0+C) / abs(sqrt(A^2+B^2))
            float A = keyLineABCs[j][0], B = keyLineABCs[j][1], C = keyLineABCs[j][2];
            float dis = abs(A * x0 + B * y0 + C) /
                        sqrt(A * A + B * B);
            if (dis < minDistance1) {
                minDistance3 = minDistance2;
                index3 = index2;
                minDistance2 = minDistance1;
                index2 = index1;
                minDistance1 = dis;
                index1 = j;
            } else {
                if (dis >= minDistance1 && dis < minDistance2) {
                    minDistance3 = minDistance2;
                    index3 = index2;
                    minDistance2 = dis;
                    index2 = j;
                } else {
                    if (dis >= minDistance2 && dis < minDistance3) {
                        minDistance3 = dis;
                        index3 = j;
                    }
                }
            }
        }
        ///Step 1.2 select nearest line from above 3
        float disToLineThres = 3; //? not in use current because above 3 line is qualified already
        float disTo2EndsThres = 1;
        if (index1 > -1) {
            //cout<<"point "<<x0<<","<<y0<<" close to "<<keyLineABCs[index][0]<<","<<keyLineABCs[index][1]<<","<<keyLineABCs[index][2]<<endl;
            ///Candidate 1 --- distance to each endpoints and sum up
            float x1 = selectedKeyLSDLines[index1].startPointX, y1 = selectedKeyLSDLines[index1].startPointY;
            float x2 = selectedKeyLSDLines[index1].endPointX, y2 = selectedKeyLSDLines[index1].endPointY;
            float disToStart = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
            float disToEnd = sqrt((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2));
            float disTo2EndPoints = disToStart + disToEnd;
            if (disToStart <= disToLineThres || disToEnd <= disToLineThres || disTo2EndPoints <= disTo2EndsThres) {
                imgKeyPoint[i].index2line = index1;
            } else {
                if (index2 > -1) {
                    ///Candidate 2 --- distance to each endpoints and sum up
                    x1 = selectedKeyLSDLines[index2].startPointX, y1 = selectedKeyLSDLines[index2].startPointY;
                    x2 = selectedKeyLSDLines[index2].endPointX, y2 = selectedKeyLSDLines[index2].endPointY;
                    disToStart = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
                    disToEnd = sqrt((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2));
                    disTo2EndPoints = disToStart + disToEnd;
                    if (disToStart <= disToLineThres || disToEnd <= disToLineThres ||
                        disTo2EndPoints <= disTo2EndsThres) {
                        imgKeyPoint[i].index2line = index2;
                    }
                } else {
                    if (index3 > -1) {
                        ///Candidate 2 --- distance to each endpoints and sum up
                        x1 = selectedKeyLSDLines[index3].startPointX, y1 = selectedKeyLSDLines[index3].startPointY;
                        x2 = selectedKeyLSDLines[index3].endPointX, y2 = selectedKeyLSDLines[index3].endPointY;
                        disToStart = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
                        disToEnd = sqrt((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2));
                        disTo2EndPoints = disToStart + disToEnd;
                        if (disToStart <= disToLineThres || disToEnd <= disToLineThres ||
                            disTo2EndPoints <= disTo2EndsThres) {
                            imgKeyPoint[i].index2line = index3;
                        }
                    }
                }
            }
        }
    }
    ///Step 2 pair up LSD lines and lidar points
    //vector<int> lidarPt2LSD(LiDAR2d.size(), -1);
    for (int i = 0; i < LiDAR2d.size(); i++) {
        float x0 = LiDAR2d[i].x, y0 = LiDAR2d[i].y;
        ///Step 2.1 search for 3 nearest lines first
        float minDistance1 = 5, minDistance2 = 5, minDistance3 = 5;
        LiDAR2d[i].index2line = -1;
        int index1 = -1, index2 = -1, index3 = -1;
        for (int j = 0; j < selectedKeyLSDLines.size(); j++) {
            ///Step 1.1 point to line --- d = abs(Ax0+By0+C) / abs(sqrt(A^2+B^2))
            float A = keyLineABCs[j][0], B = keyLineABCs[j][1], C = keyLineABCs[j][2];
            float dis = abs(A * x0 + B * y0 + C) /
                        sqrt(A * A + B * B);
            if (dis < minDistance1) {
                minDistance3 = minDistance2;
                index3 = index2;
                minDistance2 = minDistance1;
                index2 = index1;
                minDistance1 = dis;
                index1 = j;
            } else {
                if (dis >= minDistance1 && dis < minDistance2) {
                    minDistance3 = minDistance2;
                    index3 = index2;
                    minDistance2 = dis;
                    index2 = j;
                } else {
                    if (dis >= minDistance2 && dis < minDistance3) {
                        minDistance3 = dis;
                        index3 = j;
                    }
                }
            }
        }
        ///Step 2.2 LiDAR point distance to each LSD endpoints
        float disToLineThres = 3; //? not in use current because above 3 line is qualified already
        float disTo2EndsThres = 1;
        ///Candidate 1 --- distance to each endpoints and sum up
        float x1 = selectedKeyLSDLines[index1].startPointX, y1 = selectedKeyLSDLines[index1].startPointY;
        float x2 = selectedKeyLSDLines[index1].endPointX, y2 = selectedKeyLSDLines[index1].endPointY;
        float disToStart = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
        float disToEnd = sqrt((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2));
        float disTo2EndPoints = disToStart + disToEnd;
        if (disToStart <= disToLineThres || disToEnd <= disToLineThres || disTo2EndPoints <= disTo2EndsThres) {
            LiDAR2d[i].index2line = index1;
        } else {
            if (index2 > -1) {
                ///Candidate 2 --- distance to each endpoints and sum up
                x1 = selectedKeyLSDLines[index2].startPointX, y1 = selectedKeyLSDLines[index2].startPointY;
                x2 = selectedKeyLSDLines[index2].endPointX, y2 = selectedKeyLSDLines[index2].endPointY;
                disToStart = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
                disToEnd = sqrt((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2));
                disTo2EndPoints = disToStart + disToEnd;
                if (disToStart <= disToLineThres || disToEnd <= disToLineThres ||
                    disTo2EndPoints <= disTo2EndsThres) {
                    LiDAR2d[i].index2line = index2;
                }
            } else {
                if (index3 > -1) {
                    ///Candidate 2 --- distance to each endpoints and sum up
                    x1 = selectedKeyLSDLines[index3].startPointX, y1 = selectedKeyLSDLines[index3].startPointY;
                    x2 = selectedKeyLSDLines[index3].endPointX, y2 = selectedKeyLSDLines[index3].endPointY;
                    disToStart = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
                    disToEnd = sqrt((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2));
                    disTo2EndPoints = disToStart + disToEnd;
                    if (disToStart <= disToLineThres || disToEnd <= disToLineThres ||
                        disTo2EndPoints <= disTo2EndsThres) {
                        LiDAR2d[i].index2line = index3;
                    }
                }
            }
        }
    }
    for(int i = 0; i < keyLineABCs.size();i++){
        keyLineABCs[i].clear();
    }
    keyLineABCs.clear();
}
