/**
 * @file opencv_image_test.cpp
 * @brief sample for each module with C++
 */

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
//#include <opencv2/xfeatures2d.hpp>

void orb_test(const cv::Mat& img)
{
  cv::Ptr<cv::ORB> detector = cv::ORB::create();
  std::vector<cv::KeyPoint> keypoints;
  detector->detect(img, keypoints);

  cv::Mat img_keypoints;
  cv::drawKeypoints(img, keypoints, img_keypoints);

  cv::imshow("ORB keypoints", img_keypoints);
  cv::waitKey();
}

int main(int argc, char **argv)
{
  cv::CommandLineParser parser(argc, argv, "{@input | box.png | input image}");
  cv::Mat src = imread(cv::samples::findFile(parser.get<cv::String>("@input")), cv::IMREAD_GRAYSCALE);
  if (src.empty())
  {
    std::cout << "Could not open or find the image" << std::endl;
    std::cout << "Usage: " << argv[0] << " <input image>" << std::endl;
    return -1;
  }
  //cv::imshow("", src);
  //cv::waitKey();

  orb_test(src);
  return 0;
}
