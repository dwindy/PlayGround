//
// Created by xin on 20/07/22.
//
/*M///////////////////////////////////////////////////////////////////////////////////////
 //
 //  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 //
 //  By downloading, copying, installing or using the software you agree to this license.
 //  If you do not agree to this license, do not download, install,
 //  copy or use the software.
 //
 //
 //                           License Agreement
 //                For Open Source Computer Vision Library
 //
 // Copyright (C) 2014, Biagio Montesano, all rights reserved.
 // Third party copyrights are property of their respective owners.
 //
 // Redistribution and use in source and binary forms, with or without modification,
 // are permitted provided that the following conditions are met:
 //
 //   * Redistribution's of source code must retain the above copyright notice,
 //     this list of conditions and the following disclaimer.
 //
 //   * Redistribution's in binary form must reproduce the above copyright notice,
 //     this list of conditions and the following disclaimer in the documentation
 //     and/or other materials provided with the distribution.
 //
 //   * The name of the copyright holders may not be used to endorse or promote products
 //     derived from this software without specific prior written permission.
 //
 // This software is provided by the copyright holders and contributors "as is" and
 // any express or implied warranties, including, but not limited to, the implied
 // warranties of merchantability and fitness for a particular purpose are disclaimed.
 // In no event shall the Intel Corporation or contributors be liable for any direct,
 // indirect, incidental, special, exemplary, or consequential damages
 // (including, but not limited to, procurement of substitute goods or services;
 // loss of use, data, or profits; or business interruption) however caused
 // and on any theory of liability, whether in contract, strict liability,
 // or tort (including negligence or otherwise) arising in any way out of
 // the use of this software, even if advised of the possibility of such damage.
 //
 //M*/

#include <iostream>
#include <opencv2/opencv_modules.hpp>

#ifdef HAVE_OPENCV_FEATURES2D

//#include <opencv2/line_descriptor.hpp>
//#include "opencv4/opencv2/line_descriptor/descriptor.hpp"
#include "opencv2/line_descriptor/descriptor.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace cv::line_descriptor;
using namespace std;

#define MATCHES_DIST_THRESHOLD 25

static const char *keys = {"{@image_path | | Image path }" "{@image_path2 | | Image path 2 }"};

static void help() {
    cout << "\nThis example shows the functionalities of lines extraction " << "furnished by BinaryDescriptor class\n"
         << "Please, run this sample using a command in the form\n"
         << "./example_line_descriptor_lines_extraction <path_to_input_image>" << endl;
}

int main(int argc, char **argv) {
    bool compare = false;
    if (argc == 3)
        compare = true;
    cout << "argc num " << argc << " " << argv[1] << " " << argv[2] << endl;
    /* get parameters from comand line */
    CommandLineParser parser(argc, argv, keys);
    String image_path = parser.get<String>(0);
    cout << " image_path " << image_path << endl;
    String image_path2;
    if (compare)
        image_path2 = parser.get<String>(1);
    cout << " image_path2 " << image_path2 << endl;
    if (image_path.empty()) {
        help();
        return -1;
    }

    /* load image */
    cv::Mat imageMat = imread(image_path, 1);
    if (imageMat.data == NULL) {
        std::cout << "Error, image could not be loaded. Please, check its path" << std::endl;
        return -1;
    }
    cv::Mat imageMat2;
    if (compare) {
        imageMat2 = imread(image_path2, 1);
        if (imageMat2.data == NULL) {
            std::cout << "Error, image 2 could not be loaded. Please, check its path" << std::endl;
            return -1;
        }
    }

    /* create a random binary mask */   //what is the mask for?
    cv::Mat mask = Mat::ones(imageMat.size(), CV_8UC1);
    cv::Mat mask2;
    if (compare)
        mask2 = Mat::ones(imageMat2.size(), CV_8UC1);

    /* create a pointer to a BinaryDescriptor object with deafult parameters */
    Ptr<LSDDetector> lsd = LSDDetector::createLSDDetector();
    Ptr<BinaryDescriptor> bd = BinaryDescriptor::createBinaryDescriptor();
    Ptr<BinaryDescriptor> bd2 = BinaryDescriptor::createBinaryDescriptor();

    /* create a structure to store extracted lines */
    vector<KeyLine> lines;
    vector<KeyLine> keylines;
    vector<KeyLine> keylines2;
    /* extract lines */
    cv::Mat output_lsd = imageMat.clone();
    cv::Mat output_bd = imageMat.clone();
    cv::Mat output_bd2 = imageMat2.clone();
    //bd->detect( imageMat, lines, 2, 1, mask );
    lsd->detect(imageMat, lines, 2, 1, mask);
    bd->detect(imageMat, keylines, mask);
    if (compare)
        bd2->detect(imageMat2, keylines2, mask2);

    /* compute descriptors */
    cv::Mat descriptors;
    bd->compute(imageMat, keylines, descriptors);
    cv::Mat descriptors2;
    if (compare)
        bd2->compute(imageMat2, keylines2, descriptors2);

    /* draw lines extracted from octave 0 */
    if (output_lsd.channels() == 1)
        cvtColor(output_lsd, output_lsd, COLOR_GRAY2BGR);
    for (size_t i = 0; i < lines.size(); i++) {
        KeyLine kl = lines[i];
        if (kl.octave == 0) {
            /* get a random color */
            int R = (rand() % (int) (255 + 1));
            int G = (rand() % (int) (255 + 1));
            int B = (rand() % (int) (255 + 1));
            /* get extremes of line */
            Point pt1 = Point2f(kl.startPointX, kl.startPointY);
            Point pt2 = Point2f(kl.endPointX, kl.endPointY);
            /* draw line */
            line(output_lsd, pt1, pt2, Scalar(B, G, R), 3);
        }
    }
    if (output_bd.channels() == 1)
        cvtColor(output_bd, output_bd, COLOR_GRAY2BGR);
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
    if (compare) {
        if (output_bd2.channels() == 1)
            cvtColor(output_bd2, output_bd2, COLOR_GRAY2BGR);
        for (size_t i = 0; i < keylines2.size(); i++) {
            KeyLine kl = keylines2[i];
            if (kl.octave == 0) {
                /* get a random color */
                int R = (rand() % (int) (255 + 1));
                int G = (rand() % (int) (255 + 1));
                int B = (rand() % (int) (255 + 1));
                /* get extremes of line */
                Point pt1 = Point2f(kl.startPointX, kl.startPointY);
                Point pt2 = Point2f(kl.endPointX, kl.endPointY);
                /* draw line */
                line(output_bd2, pt1, pt2, Scalar(B, G, R), 3);
            }
        }
    }
    /* show lines on image */
    imshow("LSD lines", output_lsd);
    imshow("BD lines", output_bd);
    imshow("BD lines2", output_bd2);
    waitKey();

    cout << "keyline 1 number " << keylines.size() << " with descriptor matrix " << descriptors.rows << " "
         << descriptors.cols << endl;
    cout << "keyline 2 number " << keylines2.size() << " with descriptor2 matrix " << descriptors2.rows << " "
         << descriptors2.cols << endl;

    /* select keylines from first octave and their descriptors */
    std::vector<KeyLine> lbd_octave1, lbd_octave2;
    Mat left_lbd, right_lbd;
    for (int i = 0; i < (int) keylines.size(); i++) {
        if (keylines[i].octave == 0) {
            lbd_octave1.push_back(keylines[i]);
            left_lbd.push_back(descriptors.row(i));
        }
    }
    for (int j = 0; j < (int) keylines2.size(); j++) {
        if (keylines2[j].octave == 0) {
            lbd_octave2.push_back(keylines2[j]);
            right_lbd.push_back(descriptors2.row(j));
        }
    }
    /* create a BinaryDescriptorMatcher object */
    Ptr<BinaryDescriptorMatcher> bdm = BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
    /* require match */
    std::vector<DMatch> matches;
    bdm->match(left_lbd, right_lbd, matches);
    cout << "Dmatch vector size " << matches.size() << endl;
//    /* select best matches */
//    std::vector<DMatch> good_matches;
//    for (int i = 0; i < (int) matches.size(); i++) {
//        cout<<matches[i].distance<<endl;
//        if (matches[i].distance < MATCHES_DIST_THRESHOLD)//25
//            good_matches.push_back(matches[i]);//push back the index?
//    }
//    /* plot matches */
//    cv::Mat outImg;
//    cv::Mat scaled1, scaled2;
//    std::vector<char> matchesMask(matches.size(), 1);
//    drawLineMatches(imageMat, lbd_octave1, imageMat2, lbd_octave2, good_matches, outImg, Scalar::all(-1),
//                    Scalar::all(-1), matchesMask,
//                    DrawLinesMatchesFlags::DEFAULT);
//
//    imshow("Matches", outImg);
//    waitKey();
    /* select best matches */
    std::vector<DMatch> good_matches1;//>100
    std::vector<DMatch> good_matches2;//75-100
    std::vector<DMatch> good_matches3;//50-74
    std::vector<DMatch> good_matches4;//25-49
    std::vector<DMatch> good_matches5;//0-24
    for (int i = 0; i < (int) matches.size(); i++) {
        if (matches[i].distance > 60)//25
            good_matches1.push_back(matches[i]);//push back the index?
        if (55 < matches[i].distance &&matches[i].distance <= 60)//25
            good_matches2.push_back(matches[i]);//push back the index?
        if (25 < matches[i].distance &&matches[i].distance<= 40)//25
            good_matches3.push_back(matches[i]);//push back the index?
        if (10 < matches[i].distance &&matches[i].distance<= 25)//25
            good_matches4.push_back(matches[i]);//push back the index?
        if (matches[i].distance < 10)//25
            good_matches5.push_back(matches[i]);//push back the index?
    }
    /* plot matches */
    cv::Mat outImg1,outImg2,outImg3,outImg4,outImg5;
    std::vector<char> matchesMask(matches.size(), 1);
    drawLineMatches(imageMat, lbd_octave1, imageMat2, lbd_octave2, good_matches1, outImg1, Scalar::all(-1),
                    Scalar::all(-1), matchesMask,
                    DrawLinesMatchesFlags::DEFAULT);
    drawLineMatches(imageMat, lbd_octave1, imageMat2, lbd_octave2, good_matches2, outImg2, Scalar::all(-1),
                    Scalar::all(-1), matchesMask,
                    DrawLinesMatchesFlags::DEFAULT);
    drawLineMatches(imageMat, lbd_octave1, imageMat2, lbd_octave2, good_matches3, outImg3, Scalar::all(-1),
                    Scalar::all(-1), matchesMask,
                    DrawLinesMatchesFlags::DEFAULT);
    drawLineMatches(imageMat, lbd_octave1, imageMat2, lbd_octave2, good_matches4, outImg4, Scalar::all(-1),
                    Scalar::all(-1), matchesMask,
                    DrawLinesMatchesFlags::DEFAULT);
    drawLineMatches(imageMat, lbd_octave1, imageMat2, lbd_octave2, good_matches5, outImg5, Scalar::all(-1),
                    Scalar::all(-1), matchesMask,
                    DrawLinesMatchesFlags::DEFAULT);
    imshow("Matches1", outImg1);
    imshow("Matches2", outImg2);
    imshow("Matches3", outImg3);
    imshow("Matches4", outImg4);
    imshow("Matches5", outImg5);

    waitKey();
}

#else

int main()
 {
     std::cerr << "OpenCV was built without features2d module" << std::endl;
     return 0;
 }

#endif // HAVE_OPENCV_FEATURES2D