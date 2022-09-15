//
// Created by xin on 09/08/22.
/*
 * this funciont is going to test if the LSD(BinaryDescriptor) could be merged
 * todo: merge with point feature and LiDAR image
 */
//

//#include "opencv2/line_descriptor/descriptor.hpp"
//#include <opencv2/core/utility.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/highgui.hpp>
#include "LSD_merge.h"

using namespace cv;
using namespace cv::line_descriptor;
using namespace std;

static const char *keys = {"{@image_path | | Image path }" "{@image_path2 | | Image path 2 }"};


///*
// * dealing with the case that two keylines overlap.
// * since those two keylines are close,
// * the merged line should be those two endpoints that far-est to eachother.
// * Still, merge line2 into line1
// */
//void mergeKeyLineCaseSSEE(KeyLine &line1, const KeyLine &line2) {
//    float SX1 = line1.startPointX, SY1 = line1.startPointY, SX2 = line2.startPointX, SY2 = line2.startPointY;
//    float EX1 = line1.endPointX, EY1 = line1.endPointY, EX2 = line2.endPointX, EY2 = line2.endPointY;
//    float distanceS1E1 = (SX1 - EX1) * (SX1 - EX1) + (SY1 - EY1) * (SY1 - EY1);
//    float distanceS1S2 = (SX1 - SX2) * (SX1 - SX2) + (SY1 - SY2) * (SY1 - SY2);
//    float distanceS1E2 = (SX1 - EX2) * (SX1 - EX2) + (SY1 - EY2) * (SY1 - EY2);
//    float distanceE1S2 = (EX1 - SX2) * (EX1 - SX2) + (EY1 - SY2) * (EY1 - SY2);
//    float distanceE1E2 = (EX1 - EX2) * (EX1 - EX2) + (EY1 - EY2) * (EY1 - EY2);
//    float distanceS2E2 = (SX2 - EX2) * (SX2 - EX2) + (SY2 - EY2) * (SY2 - EY2);
//    float maxNumber = distanceS1E1;
//    if (maxNumber < distanceS1S2)
//        maxNumber = distanceS1S2;
//    if (maxNumber < distanceS1E2)
//        maxNumber = distanceS1E2;
//    if (maxNumber < distanceE1S2)
//        maxNumber = distanceE1S2;
//    if (maxNumber < distanceE1E2)
//        maxNumber = distanceE1E2;
//    if (maxNumber < distanceS2E2)
//        maxNumber = distanceS2E2;
//    if (maxNumber == distanceS1E1) {
//        line1.startPointX = SX1;
//        line1.startPointY = SY1;
//        line1.endPointX = EX1;
//        line1.endPointY = EY1;
//    }
//    if (maxNumber == distanceS1S2) {
//        line1.startPointX = SX1;
//        line1.startPointY = SY1;
//        line1.endPointX = SX2;
//        line1.endPointY = SY2;
//    }
//    if (maxNumber == distanceS1E2) {
//        line1.startPointX = SX1;
//        line1.startPointY = SY1;
//        line1.endPointX = EX2;
//        line1.endPointY = EY2;
//    }
//    if (maxNumber == distanceE1S2) {
//        line1.startPointX = SX2;//EX1;
//        line1.startPointY = SY2;//EY1;
//        line1.endPointX = EX1;//SX2;
//        line1.endPointY = EY1;//SY2;
//    }
//    if (maxNumber == distanceE1E2) {
//        line1.startPointX = EX1;
//        line1.startPointY = EY1;
//        line1.endPointX = EX2;
//        line1.endPointY = EY2;
//    }
//    if (maxNumber == distanceS2E2) {
//        line1.startPointX = SX2;
//        line1.startPointY = SY2;
//        line1.endPointX = EX2;
//        line1.endPointY = EY2;
//    }
//    //merge others
//    line1.numOfPixels += line2.numOfPixels;
//    line1.angle = (line1.angle + line2.angle) / 2;
//    line1.lineLength = sqrt((line1.endPointX - line1.startPointX) * (line1.endPointX - line1.startPointX) +
//                            (line1.endPointY - line1.startPointY) * (line1.endPointY - line1.startPointY));
//    line1.response += line2.response;
//    line1.size = (line1.startPointX - line1.endPointX) * (line1.startPointY - line1.endPointY);
//}
//
///*
// * dealing with the case that two keylines overlap.
// * since those two keylines are close,
// * the merged line should be those two endpoints that far-est to each other.
// * Still, merge line2 into line1
// */
//void mergeKeyLine2(KeyLine &line1, const KeyLine &line2) {
//
//    float SX1 = line1.startPointX, SY1 = line1.startPointY, SX2 = line2.startPointX, SY2 = line2.startPointY;
//    float EX1 = line1.endPointX, EY1 = line1.endPointY, EX2 = line2.endPointX, EY2 = line2.endPointY;
//    //to find out the farest points
//    float distanceS1E1 = (SX1 - EX1) * (SX1 - EX1) + (SY1 - EY1) * (SY1 - EY1);
//    float distanceS1S2 = (SX1 - SX2) * (SX1 - SX2) + (SY1 - SY2) * (SY1 - SY2);
//    float distanceS1E2 = (SX1 - EX2) * (SX1 - EX2) + (SY1 - EY2) * (SY1 - EY2);
//    float distanceE1S2 = (EX1 - SX2) * (EX1 - SX2) + (EY1 - SY2) * (EY1 - SY2);
//    float distanceE1E2 = (EX1 - EX2) * (EX1 - EX2) + (EY1 - EY2) * (EY1 - EY2);
//    float distanceS2E2 = (SX2 - EX2) * (SX2 - EX2) + (SY2 - EY2) * (SY2 - EY2);
//    float maxNumber = distanceS1E1;
//    if (maxNumber < distanceS1S2)
//        maxNumber = distanceS1S2;
//    if (maxNumber < distanceS1E2)
//        maxNumber = distanceS1E2;
//    if (maxNumber < distanceE1S2)
//        maxNumber = distanceE1S2;
//    if (maxNumber < distanceE1E2)
//        maxNumber = distanceE1E2;
//    if (maxNumber < distanceS2E2)
//        maxNumber = distanceS2E2;
//    if (maxNumber == distanceS1E1) {
//        line1.startPointX = SX1;
//        line1.startPointY = SY1;
//        line1.endPointX = EX1;
//        line1.endPointY = EY1;
//    }
//    if (maxNumber == distanceS1S2) {
//        line1.startPointX = SX1;
//        line1.startPointY = SY1;
//        line1.endPointX = SX2;
//        line1.endPointY = SY2;
//    }
//    if (maxNumber == distanceS1E2) {
//        line1.startPointX = SX1;
//        line1.startPointY = SY1;
//        line1.endPointX = EX2;
//        line1.endPointY = EY2;
//    }
//    if (maxNumber == distanceE1S2) {
//        line1.startPointX = SX2;//EX1;
//        line1.startPointY = SY2;//EY1;
//        line1.endPointX = EX1;//SX2;
//        line1.endPointY = EY1;//SY2;
//    }
//    if (maxNumber == distanceE1E2) {
//        line1.startPointX = EX1;
//        line1.startPointY = EY1;
//        line1.endPointX = EX2;
//        line1.endPointY = EY2;
//    }
//    if (maxNumber == distanceS2E2) {
//        line1.startPointX = SX2;
//        line1.startPointY = SY2;
//        line1.endPointX = EX2;
//        line1.endPointY = EY2;
//    }
//    //merge others
//    line1.numOfPixels += line2.numOfPixels; //not relyable because overlap
//    line1.angle = (line1.angle + line2.angle) / 2;
//    line1.lineLength = sqrt((line1.endPointX - line1.startPointX) * (line1.endPointX - line1.startPointX) +
//                            (line1.endPointY - line1.startPointY) * (line1.endPointY - line1.startPointY));
//    line1.response += line2.response;
//    line1.size = (line1.startPointX - line1.endPointX) * (line1.startPointY - line1.endPointY);
//}
//
///*
// * merge the primary keylines
// */
//void mergeKeyLine1(vector<KeyLine> &keylines, vector<bool> &keylineMergeFlags){
//    for (int i = 0; i < keylines.size(); i++) {
//        if (keylineMergeFlags[i])//If this keyline has been merge into another
//            continue;
//        cout << "keyline1 No." << keylines[i].class_id << " with angle " << keylines[i].angle
//             << " response " << keylines[i].response << " length :" << keylines[i].lineLength
//             << " start point " << keylines[i].startPointX << " " << keylines[i].startPointY
//             << " end point " << keylines[i].endPointX << " " << keylines[i].endPointY << endl;
//        /* draw it on the image */
////        if (keylines[i].octave == 0) {
////            /* get extremes of line */
////            Point pt1 = Point2f(keylines[i].startPointX, keylines[i].startPointY);
////            Point pt2 = Point2f(keylines[i].endPointX, keylines[i].endPointY);
////            /* draw line */
////            line(output_bd_merge, pt1, pt2, Scalar(255, 0, 0), 3);
////            //imshow("BD lines merged", output_bd_merge);
////            //waitKey();
////        }
//
//        //looking for pairs
//        for (int j = 0; j < keylines.size(); j++) {
//            if (!keylineMergeFlags[j] && j != i) { //This keyline shouldn't be merged into another | and j!=i
//                float degree1 = abs(keylines[i].angle) + abs(keylines[j].angle);
//                float degree2 = abs(keylines[j].angle - keylines[i].angle);
//                if ((abs(degree1 - 3.14) < degreeThres) || degree2 < degreeThres) {//angle < 5 degrees
//                    float distanceSS = (keylines[i].startPointX - keylines[j].startPointX) *
//                                       (keylines[i].startPointX - keylines[j].startPointX) +
//                                       (keylines[i].startPointY - keylines[j].startPointY) *
//                                       (keylines[i].startPointY - keylines[j].startPointY);
//                    float distanceSE = (keylines[i].startPointX - keylines[j].endPointX) *
//                                       (keylines[i].startPointX - keylines[j].endPointX) +
//                                       (keylines[i].startPointY - keylines[j].endPointY) *
//                                       (keylines[i].startPointY - keylines[j].endPointY);
//                    float distanceES = (keylines[i].endPointX - keylines[j].startPointX) *
//                                       (keylines[i].endPointX - keylines[j].startPointX) +
//                                       (keylines[i].endPointY - keylines[j].startPointY) *
//                                       (keylines[i].endPointY - keylines[j].startPointY);
//                    float distanceEE = (keylines[i].endPointX - keylines[j].endPointX) *
//                                       (keylines[i].endPointX - keylines[j].endPointX) +
//                                       (keylines[i].endPointY - keylines[j].endPointY) *
//                                       (keylines[i].endPointY - keylines[j].endPointY);
//                    float midXi = (keylines[i].startPointX + keylines[i].endPointX) / 2;
//                    float midYi = (keylines[i].startPointY + keylines[i].endPointY) / 2;
//                    float midXj = (keylines[j].startPointX + keylines[j].endPointX) / 2;
//                    float midYj = (keylines[j].startPointY + keylines[j].endPointY) / 2;
//                    float distanceMid = (midXi - midXj) * (midXi - midXj) + (midYi - midYj) * (midYi - midYj);
//                    bool SS = false, SE = false, ES = false, EE = false, Mid = false;
//                    if (distanceSS < distanceThres)SS = true;
//                    if (distanceSE < distanceThres)SE = true;
//                    if (distanceEE < distanceThres)EE = true;
//                    if (distanceES < distanceThres)ES = true;
//                    if (distanceMid < distanceThres)Mid = true;
//                    if (SS || SE || EE || ES || Mid) {//start or end points is close
//                        cout << "merge with" << endl;
//                        cout <<" degree1 "<<degree1<<" degree2 "<<degree2<<endl;
//                        cout << "keyline2 No." << keylines[j].class_id << " with angle " << keylines[j].angle
//                             << " response " << keylines[j].response << " length :" << keylines[j].lineLength
//                             << " start point " << keylines[j].startPointX << " " << keylines[j].startPointY
//                             << " end point " << keylines[j].endPointX << " " << keylines[j].endPointY << endl;
//                        /* draw it on the image */
////                        if (keylines[j].octave == 0) {
////                            /* get extremes of line */
////                            Point pt1 = Point2f(keylines[j].startPointX, keylines[j].startPointY);
////                            Point pt2 = Point2f(keylines[j].endPointX, keylines[j].endPointY);
////                            /* draw line */
////                            line(output_bd_merge, pt1, pt2, Scalar(0, 255, 0), 3);
////                            imshow("BD lines merged", output_bd_merge);
////                            waitKey();
////                        }
//                        mergeKeyLine2(keylines[i], keylines[j]);
//                        keylineMergeFlags[j] = true;
//                        cout << "result : " << endl;
//                        cout << "keyline new1 No." << keylines[i].class_id << " with angle " << keylines[i].angle
//                             << " response " << keylines[i].response << " length :" << keylines[i].lineLength
//                             << " start point " << keylines[i].startPointX << " " << keylines[i].startPointY
//                             << " end point " << keylines[i].endPointX << " " << keylines[i].endPointY << endl;
//                        //j = 0; //reset j. maybe do not do this.
//
//                        /* draw it on the image */
////                        if (keylines[i].octave == 0) {
////                            /* get extremes of line */
////                            Point pt1 = Point2f(keylines[i].startPointX, keylines[i].startPointY);
////                            Point pt2 = Point2f(keylines[i].endPointX, keylines[i].endPointY);
////                            /* draw line */
////                            line(output_bd_merge, pt1, pt2, Scalar(255, 255, 0), 3);
////                        }
////                        imshow("BD lines merged", output_bd_merge);
////                        waitKey();
//                    }
//                }
//            }
//        }
//    }
//}

int main(int argc, char **argv) {
    cout << "argc num " << argc << " , " << argv[1] << " " << argv[2] << endl;
    /* get parameters from comand line */
    CommandLineParser parser(argc, argv, keys);
    String image_path = parser.get<String>(0);
    if (image_path.empty())
        return -1;
    /* load image */
    cv::Mat imageMat = imread(image_path, 1);
    if (imageMat.data == nullptr) {
        std::cout << "Error, image could not be loaded. Please, check its path" << std::endl;
        return -1;
    }
    /* create a random binary mask */   //what is the mask for?
    cv::Mat mask = Mat::ones(imageMat.size(), CV_8UC1);
    /* create a pointer to a BinaryDescriptor object with deafult parameters */
    Ptr<BinaryDescriptor> bd = BinaryDescriptor::createBinaryDescriptor();
    /* create a structure to store extracted lines */
    vector<KeyLine> keylines;
    cv::Mat output_bd = imageMat.clone();
    cv::Mat output_bd_merge = imageMat.clone();
    bd->detect(imageMat, keylines, mask);
    /* compute descriptors */
    cv::Mat descriptors;
    bd->compute(imageMat, keylines, descriptors);
    /* draw lines extracted from octave 0 */
    if (output_bd.channels() == 1)
        cvtColor(output_bd, output_bd, COLOR_GRAY2BGR);

    /* print all keylines attribution */
//    for (int i = 0; i < keylines.size(); i++) {
//        cout << "keyline No." << keylines[i].class_id << " with angle " << keylines[i].angle
//             << " response " << keylines[i].response << " length :" << keylines[i].lineLength
//             << " start point " << keylines[i].startPointX << " " << keylines[i].startPointY
//             << " end point " << keylines[i].endPointX << " " << keylines[i].endPointY
//             <<endl;
//    }

    /* show */
    for (size_t i = 0; i < keylines.size(); i++) {
        KeyLine kl = keylines[i];
        if (kl.octave == 0) {
            /* get a random color */
            int R = (rand() % (int) (255 + 1));
            int G = (rand() % (int) (255 + 1));
            int B = (rand() % (int) (255 + 1));
            /* get extremes of line */
            Point pt1 = Point2f(kl.startPointX, kl.startPointY);
            Point pt2 = Point2f(kl.endPointX, kl.endPointY);
            /* draw line */
            line(output_bd, pt1, pt2, Scalar(B, G, R), 3);
        }
    }
    imshow("BD keylines original", output_bd);
    waitKey();

    std::vector<bool> keylineMergeFlags(keylines.size(), false);
    mergeKeyLine1(keylines,keylineMergeFlags);

    /* select the lines with enough length */
    vector<KeyLine> selectedKeyLines;
    for (int i = 0; i < keylines.size(); i++) {
        if (keylineMergeFlags[i])
            continue;
        if(keylines[i].lineLength>lineLength){
            selectedKeyLines.push_back(keylines[i]);
        }
    }
    /* show */
    for (size_t i = 0; i < selectedKeyLines.size(); i++) {
        KeyLine kl = selectedKeyLines[i];
        cout << "selected keyline No." << keylines[i].class_id << " with angle " << keylines[i].angle
             << " response " << keylines[i].response << " length :" << keylines[i].lineLength
             << " start point " << keylines[i].startPointX << " " << keylines[i].startPointY
             << " end point " << keylines[i].endPointX << " " << keylines[i].endPointY
             << endl;
        if (kl.octave == 0) {
            /* get a random color */
            int R = (rand() % (int) (255 + 1));
            int G = (rand() % (int) (255 + 1));
            int B = (rand() % (int) (255 + 1));
            /* get extremes of line */
            Point pt1 = Point2f(kl.startPointX, kl.startPointY);
            Point pt2 = Point2f(kl.endPointX, kl.endPointY);
            /* draw line */
            line(output_bd_merge, pt1, pt2, Scalar(B, G, R), 3);
        }
    }
    imshow("BD lines merged", output_bd_merge);
    waitKey();

    cout << "system ended" << endl;
}