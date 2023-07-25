/*!
  Esto es un esqueleto de programa para usar en las prácticas
  de Visión Artificial.

  Se supone que se utilizará OpenCV.

  Para compilar, puedes ejecutar:
    g++ -Wall -o esqueleto esqueleto.cc `pkg-config opencv --cflags --libs`

*/

#include <iostream>
#include <exception>

//Includes para OpenCV, Descomentar según los módulo utilizados.
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/calib3d/calib3d.hpp>

#include "handle.h"

const cv::String keys =
"{help h usage ? |      | print this message.}"
"{v video        |      | the input is a video file.}"
"{fourcc         |      | output video codec used, for example \"MJPG\". Default same as input.}"
"{@intrinsics    |<none>| intrinsics parameters file.}"
"{@input         |<none>| input image|video.}"
"{@output        |<none>| output image|video.}"
;

using namespace cv;

int main(int argc, char* const* argv) {

    int retCode = EXIT_SUCCESS;
    cv::CommandLineParser parser(argc, argv, keys);

    parser.about("Undistort an image or video file.");
    if (parser.has("help")) {
        parser.printMessage();
        return EXIT_SUCCESS;
    }

    auto is_video = parser.has("v");

    auto calib_fname = parser.get<std::string>("@intrinsics");
    auto input_fname = parser.get<std::string>("@input");
    auto output_fname = parser.get<std::string>("@output");

    if (!parser.check())
    {
        parser.printErrors();
        return EXIT_FAILURE;
    }

    try {

        float error;
        cv::Size camera_size(8,6);
        cv::Mat img, K, dist_coeffs, rvec, tvec;
        std::vector<cv::Point2f> corner_points;

        K = cv::imread(input_fname);
        dist_coeffs = cv::imread(output_fname);
        img = K.clone();

        
        std::vector<cv::Point2f> point2f;
        std::vector<cv::Point3f> point3f;

        cv::namedWindow("INPUT", cv::WINDOW_GUI_EXPANDED + cv::WINDOW_AUTOSIZE);
        cv::namedWindow("OUTPUT", cv::WINDOW_GUI_EXPANDED + cv::WINDOW_AUTOSIZE);
        cv::namedWindow("OTHER", cv::WINDOW_GUI_EXPANDED + cv::WINDOW_AUTOSIZE);

        std::vector<std::vector<Point3f> > objectPoints;
        std::vector<std::vector<Point2f> > imagePoints;
        cv::Mat camera_matrix;
        cv::Size img_size_;

        if (is_video) { }
        else
        {

           std::vector<cv::Point3f> ret_v = fsiv_generate_3d_calibration_points(camera_size, 4);
           fsiv_find_chessboard_corners(dist_coeffs, camera_size, corner_points);

          // cv::calibrateCamera(objectPoints, imagePoints, img_size_, camera_matrix, dist_coeffs, rvec, tvec, 0);

           imshow("INPUT", K);
           imshow("OUTPUT", dist_coeffs);
           imshow("OTHER", img);
           cv::waitKey();
        }
    }
    catch (std::exception& e)
    {
        std::cerr << "caught exception: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}
