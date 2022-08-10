//
// Created by xin on 09/08/22.
/*
 * this funciont is going to test if the LSD(BinaryDescriptor) could be merged
 * todo: 重叠位置的keylines should be merged.
 */
//

#include "opencv2/line_descriptor/descriptor.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace cv::line_descriptor;
using namespace std;

static const char *keys = {"{@image_path | | Image path }" "{@image_path2 | | Image path 2 }"};

/*
 * dealing with the case that two keylines overlap.
 * since those two keylines are close,
 * the merged line should be those two endpoints that far-est to eachother.
 * Still, merge line2 into line1
 */
void mergeKeyLineCaseSSEE(KeyLine &line1, KeyLine line2) {
    double SX1 = line1.startPointX, SY1 = line1.startPointY, SX2 = line2.startPointX, SY2 = line2.startPointY;
    double EX1 = line1.endPointX, EY1 = line1.endPointY, EX2 = line2.endPointX, EY2 = line2.endPointY;
    double distanceS1E1 = (SX1 - EX1) * (SX1 - EX1) + (SY1 - EY1) * (SY1 - EY1);
    double distanceS1S2 = (SX1 - SX2) * (SX1 - SX2) + (SY1 - SY2) * (SY1 - SY2);
    double distanceS1E2 = (SX1 - EX2) * (SX1 - EX2) + (SY1 - EY2) * (SY1 - EY2);
    double distanceE1S2 = (EX1 - SX2) * (EX1 - SX2) + (EY1 - SY2) * (EY1 - SY2);
    double distanceE1E2 = (EX1 - EX2) * (EX1 - EX2) + (EY1 - EY2) * (EY1 - EY2);
    double distanceS2E2 = (SX2 - EX2) * (SX2 - EX2) + (SY2 - EY2) * (SY2 - EY2);
    double maxNumber = distanceS1E1;
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

/*merge line2 INTO line1*/
void mergeKeyLine(KeyLine &line1, KeyLine line2) {
    //1st Merge start & end points
    double distanceThres = 100;
    double distanceSS = (line1.startPointX - line2.startPointX) * (line1.startPointX - line2.startPointX)
                        + (line1.startPointY - line2.startPointY) * (line1.startPointY - line2.startPointY);
    double distanceSE = (line1.startPointX - line2.endPointX) * (line1.startPointX - line2.endPointX)
                        + (line1.startPointY - line2.endPointY) * (line1.startPointY - line2.endPointY);
    double distanceES = (line1.endPointX - line2.startPointX) * (line1.endPointX - line2.startPointX)
                        + (line1.endPointY - line2.startPointY) * (line1.endPointY - line2.startPointY);
    double distanceEE = (line1.endPointX - line2.endPointX) * (line1.endPointX - line2.endPointX)
                        + (line1.endPointY - line2.endPointY) * (line1.endPointY - line2.endPointY);
    double distanceSSEE = distanceSS + distanceEE;
    //Case: two keylines overlapped
    if (distanceSS < distanceThres && distanceEE < distanceThres && distanceSSEE < (2 * distanceThres)) {
        mergeKeyLineCaseSSEE(line1, line2);
    } else {//Other cases
        if (distanceSS < distanceThres) {//remove Starts and Starts | end1-----start1-start2-----end2
//            line1.startPointX = line1.endPointX;
//            line1.startPointY = line1.endPointY;
//            line1.endPointX = line2.endPointX;
//            line1.endPointY = line2.endPointY;
            mergeKeyLineCaseSSEE(line1,line2);
        }
        if (distanceSE < distanceThres) {//remove start and end | end1-----start1-end2-----start2
//            line1.startPointX = line1.endPointX;
//            line1.startPointY = line1.endPointY;
//            line1.endPointX = line2.startPointX;
//            line1.endPointY = line2.startPointY;
            mergeKeyLineCaseSSEE(line1,line2);
        }
        if (distanceES < distanceThres) {//remove ends and Starts | start1-----end1-start2-----end2
//            line1.endPointX = line2.endPointX;
//            line1.endPointY = line2.endPointY;
            mergeKeyLineCaseSSEE(line1,line2);
        }
        if (distanceEE < distanceThres) {//remove end and end | start1-----end1-end2-----start2
//            line1.endPointX = line2.startPointX;
//            line1.endPointY = line2.startPointY;
            mergeKeyLineCaseSSEE(line1,line2);
        }
        //merge others
//        line1.numOfPixels += line2.numOfPixels;
//        line1.angle = (line1.angle + line2.angle) / 2;
//        line1.lineLength += line2.lineLength;
//        line1.response += line2.response;
//        line1.size = (line1.endPointX - line1.startPointX) * (line1.endPointX - line1.startPointX);
    }
}

int main(int argc, char **argv) {
    bool compare = false;
    if (argc == 3)
        compare = true;
    cout << "argc num " << argc << " , " << argv[1] << " " << argv[2] << endl;
    /* get parameters from comand line */
    CommandLineParser parser(argc, argv, keys);
    String image_path = parser.get<String>(0);
    if (image_path.empty())
        return -1;
    /* load image */
    cv::Mat imageMat = imread(image_path, 1);
    if (imageMat.data == NULL) {
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
    /* print keylines */
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
    imshow("BD lines", output_bd);
    waitKey();

//    /*
//     * TEST CASE
//     */
//    keylines[0].startPointX = 5;
//    keylines[0].startPointY = 5;
//    keylines[0].endPointX = 10;
//    keylines[0].endPointY = 10;
//    keylines[1].startPointX = 6;
//    keylines[1].startPointY = 6;
//    keylines[1].endPointX = 9;
//    keylines[1].endPointY = 9;
//    cout<<"input keyline0 "<<keylines[0].startPointX<<" , "<<keylines[0].startPointY<<" | "<<keylines[0].endPointX<<" , "<<keylines[0].endPointY<<endl;
//    cout<<"input keyline1 "<<keylines[1].startPointX<<" , "<<keylines[1].startPointY<<" | "<<keylines[1].endPointX<<" , "<<keylines[1].endPointY<<endl;
//    mergeKeyLine(keylines[0],keylines[1]);
//    cout<<"------result keyline "<<keylines[0].startPointX<<" , "<<keylines[0].startPointY<<" | "<<keylines[0].endPointX<<" , "<<keylines[0].endPointY<<endl;
//    keylines[1].startPointX = 4;
//    keylines[1].startPointY = 4;
//    keylines[1].endPointX = 9;
//    keylines[1].endPointY = 9;
//    cout<<"input keyline0 "<<keylines[0].startPointX<<" , "<<keylines[0].startPointY<<" | "<<keylines[0].endPointX<<" , "<<keylines[0].endPointY<<endl;
//    cout<<"input keyline1 "<<keylines[1].startPointX<<" , "<<keylines[1].startPointY<<" | "<<keylines[1].endPointX<<" , "<<keylines[1].endPointY<<endl;
//    mergeKeyLine(keylines[0],keylines[1]);
//    cout<<"------result keyline "<<keylines[0].startPointX<<" , "<<keylines[0].startPointY<<" | "<<keylines[0].endPointX<<" , "<<keylines[0].endPointY<<endl;
//    keylines[1].startPointX = 6;
//    keylines[1].startPointY = 6;
//    keylines[1].endPointX = 11;
//    keylines[1].endPointY =11;
//    cout<<"input keyline0 "<<keylines[0].startPointX<<" , "<<keylines[0].startPointY<<" | "<<keylines[0].endPointX<<" , "<<keylines[0].endPointY<<endl;
//    cout<<"input keyline1 "<<keylines[1].startPointX<<" , "<<keylines[1].startPointY<<" | "<<keylines[1].endPointX<<" , "<<keylines[1].endPointY<<endl;
//    mergeKeyLine(keylines[0],keylines[1]);
//    cout<<"------result keyline "<<keylines[0].startPointX<<" , "<<keylines[0].startPointY<<" | "<<keylines[0].endPointX<<" , "<<keylines[0].endPointY<<endl;

//    keylines[0].startPointX = 848;
//    keylines[0].startPointY = 0;
//    keylines[0].endPointX = 854;
//    keylines[0].endPointY = 183;
//    keylines[1].startPointX = 854;
//    keylines[1].startPointY = 183;
//    keylines[1].endPointX = 851;
//    keylines[1].endPointY = 45;
//    cout<<"input keyline0 "<<keylines[0].startPointX<<" , "<<keylines[0].startPointY<<" | "<<keylines[0].endPointX<<" , "<<keylines[0].endPointY<<endl;
//    cout<<"input keyline1 "<<keylines[1].startPointX<<" , "<<keylines[1].startPointY<<" | "<<keylines[1].endPointX<<" , "<<keylines[1].endPointY<<endl;
//    mergeKeyLine(keylines[0],keylines[1]);
//    cout<<"------result keyline "<<keylines[0].startPointX<<" , "<<keylines[0].startPointY<<" | "<<keylines[0].endPointX<<" , "<<keylines[0].endPointY<<endl;


    /* merging 360/3.14 = 0.008722222 per degree*/
    std::vector<bool> keylineMergeFlags(keylines.size(), false);
    for (int i = 0; i < keylines.size(); i++) {
        if(keylineMergeFlags[i]==true)
            continue;
        cout << "keyline1 No." << keylines[i].class_id << " with angle " << keylines[i].angle
             << " response " << keylines[i].response << " length :" << keylines[i].lineLength
             << " start point " << keylines[i].startPointX << " " << keylines[i].startPointY
             << " end point " << keylines[i].endPointX << " " << keylines[i].endPointY
             << endl;
        for (int j = 0; j < keylines.size(); j++) {
            if (keylineMergeFlags[j] == false&&j!=i) {
                if (abs(keylines[j].angle - keylines[i].angle) < (0.008722222 * 10)) {//angle < 5 degrees
                    double distanceSS = (keylines[i].startPointX - keylines[j].startPointX) *
                                        (keylines[i].startPointX - keylines[j].startPointX) +
                                        (keylines[i].startPointY - keylines[j].startPointY) *
                                        (keylines[i].startPointY - keylines[j].startPointY);
                    double distanceSE = (keylines[i].startPointX - keylines[j].endPointX) *
                                        (keylines[i].startPointX - keylines[j].endPointX) +
                                        (keylines[i].startPointY - keylines[j].endPointY) *
                                        (keylines[i].startPointY - keylines[j].endPointY);
                    double distanceES = (keylines[i].endPointX - keylines[j].startPointX) *
                                        (keylines[i].endPointX - keylines[j].startPointX) +
                                        (keylines[i].endPointY - keylines[j].startPointY) *
                                        (keylines[i].endPointY - keylines[j].startPointY);
                    double distanceEE = (keylines[i].endPointX - keylines[j].endPointX) *
                                        (keylines[i].endPointX - keylines[j].endPointX) +
                                        (keylines[i].endPointY - keylines[j].endPointY) *
                                        (keylines[i].endPointY - keylines[j].endPointY);
                    bool SS = false, SE = false, ES = false, EE = false;
                    if (distanceSS < 100)SS = true;
                    if (distanceSE < 100)SE = true;
                    if (distanceEE < 100)EE = true;
                    if (distanceES < 100)ES = true;
                    if (SS || SE || EE || ES) {//start or end points is close
                        cout << "merge with" << endl;
                        cout << "keyline2 No." << keylines[j].class_id << " with angle " << keylines[j].angle
                             << " response " << keylines[j].response << " length :" << keylines[j].lineLength
                             << " start point " << keylines[j].startPointX << " " << keylines[j].startPointY
                             << " end point " << keylines[j].endPointX << " " << keylines[j].endPointY
                             << endl;
                        mergeKeyLine(keylines[i], keylines[j]);
                        keylineMergeFlags[j] = true;
                        cout << "result : " << endl;
                        cout << "keyline new1 No." << keylines[i].class_id << " with angle " << keylines[i].angle
                             << " response " << keylines[i].response << " length :" << keylines[i].lineLength
                             << " start point " << keylines[i].startPointX << " " << keylines[i].startPointY
                             << " end point " << keylines[i].endPointX << " " << keylines[i].endPointY
                             << endl;
                        j = 0;//reset j
                    }
                }
            }
        }
    }
//    cout<<"¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬"<<endl;
//    for (int i = 0; i < keylines.size(); i++) {
//        if(keylineMergeFlags[i]== true)
//            continue;
//        cout << "keyline No." << keylines[i].class_id << " with angle " << keylines[i].angle
//             << " response " << keylines[i].response << " length :" << keylines[i].lineLength
//             << " start point " << keylines[i].startPointX << " " << keylines[i].startPointY
//             << " end point " << keylines[i].endPointX << " " << keylines[i].endPointY
//             <<endl;
//    }


    /* show */
    for (size_t i = 0; i < keylines.size(); i++) {
        KeyLine kl = keylines[i];
        if(keylineMergeFlags[i]== true)
            continue;
        cout << "keyline No." << keylines[i].class_id << " with angle " << keylines[i].angle
             << " response " << keylines[i].response << " length :" << keylines[i].lineLength
             << " start point " << keylines[i].startPointX << " " << keylines[i].startPointY
             << " end point " << keylines[i].endPointX << " " << keylines[i].endPointY
             <<endl;
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
        imshow("BD lines merged", output_bd_merge);
        waitKey();
    }
    cv::circle(output_bd_merge,cv::Point (848,0),5, cv::Scalar(0,0,255),1);
    cv::circle(output_bd_merge,cv::Point (848,183),5, cv::Scalar(0,0,255),1);
    cv::circle(output_bd_merge,cv::Point (848,183),7, cv::Scalar(255,0,0),1);
    cv::circle(output_bd_merge,cv::Point (851,45),5, cv::Scalar(255,0,0),1);
    imshow("BD lines merged", output_bd_merge);
    waitKey();

    cout<<"system ended"<<endl;
}