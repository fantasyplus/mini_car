/** MIT License
Copyright (c) 2017 Miguel Maestre Trueba
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *
 *@copyright Copyright 2017 Miguel Maestre Trueba
 *@file classifier.cpp
 *@author Miguel Maestre Trueba
 *@brief Implementation of the methods of class classifier.
 *@brief These methods are in charge of receiving images and detecting and classifying signs in the images.
 */

#include <cv_bridge/cv_bridge.h>
#include <cstdlib>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "ros/console.h"
#include "classifier.hpp"

void classifier::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    // Convert from ROS Image msg to OpenCV image
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        imagen = cv_ptr->image;
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

cv::Mat classifier::deNoise(cv::Mat inputImage)
{
    cv::Mat output;
    // Apply gaussian filter to denoise image
    cv::GaussianBlur(inputImage, output, cv::Size(3, 3), 0, 0);

    return output;
}

std::vector<cv::Mat> classifier::MSER_Features(cv::Mat img, double &area)
{
    cv::Mat bgr[3];
    cv::Mat red_blue;
    cv::Mat rb_binary;
    cv::Mat detection;
    cv::Size size(64, 64);
    std::vector<cv::Mat> detections;

    // Normalize images with respect to Red and Blue and binarize
    split(img, bgr);

    cv::Mat red_norm = 255 * (bgr[2] / (bgr[0] + bgr[1] + bgr[2]));
    cv::Mat red;
    red_norm.convertTo(red, CV_8UC1);
    cv::Mat blue_norm = 255 * (bgr[0] / (bgr[0] + bgr[1] + bgr[2]));
    cv::Mat blue;
    blue_norm.convertTo(blue, CV_8UC1);

    max(red, blue, red_blue);

    threshold(red_blue, rb_binary, 200, 255, cv::THRESH_BINARY);

    // MSER regions detection
    cv::Ptr<cv::MSER> ms = cv::MSER::create(50, 1000, 14400, 0.9, 0.1, 200, 1.01, 0.1, 1);
    std::vector<std::vector<cv::Point>> regions;
    std::vector<cv::Rect> mser_bbox;
    ms->detectRegions(rb_binary, regions, mser_bbox);

    // For every bounding box in the image
    for (cv::Rect i : mser_bbox)
    {
        // Ratio filter of detected regions
        double ratio = (static_cast<double>(i.height) / static_cast<double>(i.width));

        if (ratio > 0.2 && ratio < 2.2)
        {
            // Crop bounding boxes to get new images
            detection = img(i);

            // if (!detection.empty())
            // {
            //     cv::imshow("Image", detection);
            //     cv::waitKey();
            // }

            area = static_cast<double>(i.height) * static_cast<double>(i.width);

            // Resize images  to fit the trained data
            cv::resize(detection, detection, size);

            // Output the vector of images
            detections.push_back(detection);
            this->boxes.push_back(i);
        }
    }
    return detections;
}

cv::Mat classifier::HOG_Features(cv::HOGDescriptor hog, std::vector<cv::Mat> imgs)
{
    std::vector<std::vector<float>> HOG;

    // For all of the images of the vector, compute HOG features
    for (cv::Mat i : imgs)
    {
        std::vector<float> descriptor;
        hog.compute(i, descriptor);
        HOG.push_back(descriptor);
    }

    // Convert HOG features vector to Matrix for the SVM
    cv::Mat signMat(HOG.size(), HOG[0].size(), CV_32FC1);
    auto i = 0;
    while (i < HOG.size())
    {
        auto j = 0;
        while (j < HOG[0].size())
        {
            signMat.at<float>(i, j) = HOG[i][j];
            j++;
        }
        i++;
    }
    return signMat;
}

void classifier::loadTrainingImgs(std::vector<cv::Mat> &trainImgs, std::vector<int> &trainLabels)
{
    // Load all the forward signs images from dataset and label them
    cv::String pathname = "/home/xt/mini_car/src/vision_detection/Training_Images/forward";
    std::vector<cv::String> filenames;
    cv::glob(pathname, filenames);
    cv::Size size(64, 64);

    for (cv::String i : filenames)
    {
        cv::Mat src = imread(i);

        cv::resize(src, src, size);
        trainImgs.push_back(src);
        trainLabels.push_back(1);
    }

    // // Load all the turn signs images from dataset and label them
    cv::String pathname2 = "/home/xt/mini_car/src/vision_detection/Training_Images/turn_left";
    std::vector<cv::String> filenames2;
    cv::glob(pathname2, filenames2);
    cv::Size size2(64, 64);

    for (cv::String i : filenames2)
    {
        cv::Mat src2 = imread(i);

        cv::resize(src2, src2, size2);
        trainImgs.push_back(src2);
        trainLabels.push_back(2);
    }

    // // Load all the stop signs images from dataset and label them
    cv::String pathname3 = "/home/xt/mini_car/src/vision_detection/Training_Images/stop";
    std::vector<cv::String> filenames3;
    cv::glob(pathname3, filenames3);
    cv::Size size3(64, 64);

    for (cv::String i : filenames3)
    {
        cv::Mat src3 = imread(i);

        cv::resize(src3, src3, size3);
        trainImgs.push_back(src3);
        trainLabels.push_back(3);
    }

    // Load all the dead end signs images from dataset and label them
    // cv::String pathname4 = "/home/xt/mini_car/src/vision_detection/Training_Images/dead_end";
    // std::vector<cv::String> filenames4;
    // cv::glob(pathname4, filenames4);
    // cv::Size size4(64, 64);

    // for (cv::String i : filenames4)
    // {
    //     cv::Mat src4 = imread(i);

    //     cv::resize(src4, src4, size4);
    //     trainImgs.push_back(src4);
    //     trainLabels.push_back(4);
    // }

    // Load all the no left turn signs images from dataset and label them
    // cv::String pathname5 = "/home/xt/mini_car/src/vision_detection/Training_Images/no_left";
    // std::vector<cv::String> filenames5;
    // cv::glob(pathname5, filenames5);
    // cv::Size size5(64, 64);

    // for (cv::String i : filenames5)
    // {
    //     cv::Mat src5 = imread(i);

    //     cv::resize(src5, src5, size5);
    //     trainImgs.push_back(src5);
    //     trainLabels.push_back(5);
    // }

    // Load all the no right signs images from dataset and label them
    // cv::String pathname6 = "/home/xt/mini_car/src/vision_detection/Training_Images/no_right";
    // std::vector<cv::String> filenames6;
    // cv::glob(pathname6, filenames6);
    // cv::Size size6(64, 64);

    // for (cv::String i : filenames6)
    // {
    //     cv::Mat src6 = imread(i);

    //     cv::resize(src6, src6, size6);

    //     trainImgs.push_back(src6);
    //     trainLabels.push_back(6);
    // }

    // Load all the parking signs images from dataset and label them
    cv::String pathname7 = "/home/xt/mini_car/src/vision_detection/Training_Images/parking";
    std::vector<cv::String> filenames7;
    cv::glob(pathname7, filenames7);
    cv::Size size7(64, 64);

    for (cv::String i : filenames7)
    {
        cv::Mat src7 = imread(i);

        cv::resize(src7, src7, size7);
        trainImgs.push_back(src7);
        trainLabels.push_back(7);
    }

    // Load all the round about signs images from dataset and label them
    cv::String pathname8 = "/home/xt/mini_car/src/vision_detection/Training_Images/round_about";
    std::vector<cv::String> filenames8;
    cv::glob(pathname8, filenames8);
    cv::Size size8(64, 64);

    for (cv::String i : filenames8)
    {
        cv::Mat src8 = imread(i);

        cv::resize(src8, src8, size8);
        trainImgs.push_back(src8);
        trainLabels.push_back(8);
    }
}

void classifier::SVMTraining(cv::Ptr<cv::ml::SVM> &svm, cv::Mat trainHOG, std::vector<int> trainLabels)
{
    // Set parameters of the SVM
    svm->setGamma(0.50625);
    svm->setC(12.5);
    svm->setKernel(cv::ml::SVM::RBF);
    svm->setType(cv::ml::SVM::C_SVC);

    // Feed SVM with all the labeled data and train it
    cv::Ptr<cv::ml::TrainData> td = cv::ml::TrainData::create(trainHOG, cv::ml::ROW_SAMPLE, trainLabels);
    svm->train(td);
}

int classifier::trainStage(cv::HOGDescriptor &hog, cv::Ptr<cv::ml::SVM> &svm, std::vector<cv::Mat> &trainImgs,
                           std::vector<int> &trainLabels)
{
    ROS_INFO_STREAM("SVM Training Stage started...");
    // Load training data and resize
    this->loadTrainingImgs(trainImgs, trainLabels);

    // HOG features of training images
    cv::Mat trainHOG = this->HOG_Features(hog, trainImgs);

    // Train SVM and save model
    this->SVMTraining(svm, trainHOG, trainLabels);
    ROS_INFO_STREAM("SVM Training Stage completed");
    ros::Duration(2).sleep();

    // Return 1 as success
    return 1;
}

std::vector<float> classifier::SVMTesting(cv::Ptr<cv::ml::SVM> &svm, cv::Mat testHOG)
{
    cv::Mat answer;

    // Feed SVM with HOG features from detections and label it
    svm->predict(testHOG, answer);

    // Return the label of the detection
    auto i = 0;
    ROS_WARN("answer.rows: %d", answer.rows);
    while (i < answer.rows)
    {
        this->traffic_signs.push_back(answer.at<float>(i, 0));
        ROS_WARN("traffic_sign: %f\n", this->traffic_signs.front());
        i++;
    }
    return this->traffic_signs;
}

int classifier::visualization()
{
    cv::Mat viz;
    this->imagen.copyTo(viz);
    cv::putText(viz, "Robot View", cv::Point(10, 30), cv::FONT_HERSHEY_DUPLEX, 1, CV_RGB(0, 0, 255));

    // For all the signs in the image, print bounding box and type of sign
    for (cv::Rect i : this->boxes)
    {
        if (!this->traffic_signs.empty())
        {
            float traffic_sign = this->traffic_signs.front();
            cv::rectangle(viz, i, CV_RGB(50, 200, 0), 2);
            if (traffic_sign == 1)
            {
                cv::Point org(i.x, i.y - 5);
                cv::putText(viz, "Forward", org, cv::FONT_HERSHEY_DUPLEX, 1, CV_RGB(0, 0, 255));
            }
            if (traffic_sign == 2)
            {
                cv::Point org(i.x, i.y - 5);
                cv::putText(viz, "Left", org, cv::FONT_HERSHEY_DUPLEX, 1, CV_RGB(0, 0, 255));
            }
            if (traffic_sign == 3)
            {
                cv::Point org(i.x, i.y - 5);
                cv::putText(viz, "Stop", org, cv::FONT_HERSHEY_DUPLEX, 1, CV_RGB(0, 0, 255));
            }
            // if (traffic_sign == 4)
            // {
            //     cv::Point org(i.x, i.y - 5);
            //     cv::putText(viz, "Dead End", org, cv::FONT_HERSHEY_DUPLEX, 1, CV_RGB(0, 0, 255));
            // }
            // if (traffic_sign == 5)
            // {
            //     cv::Point org(i.x, i.y - 5);
            //     cv::putText(viz, "No Left", org, cv::FONT_HERSHEY_DUPLEX, 1, CV_RGB(0, 0, 255));
            // }
            // if (traffic_sign == 6)
            // {
            //     cv::Point org(i.x, i.y - 5);
            //     cv::putText(viz, "No Right", org, cv::FONT_HERSHEY_DUPLEX, 1, CV_RGB(0, 0, 255));
            // }
            if (traffic_sign == 7)
            {
                cv::Point org(i.x, i.y - 5);
                cv::putText(viz, "Parking", org, cv::FONT_HERSHEY_DUPLEX, 1, CV_RGB(0, 0, 255));
            }
            if (traffic_sign == 8)
            {
                cv::Point org(i.x, i.y - 5);
                cv::putText(viz, "Round About", org, cv::FONT_HERSHEY_DUPLEX, 1, CV_RGB(0, 0, 255));
            }
            this->traffic_signs.pop_back();
        }
    }

    this->boxes.clear();
    cv::namedWindow("view");
    imshow("view", viz);
    return 1;
}
