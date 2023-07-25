
#include "handle.h"

std::vector<cv::Point3f>
fsiv_generate_3d_calibration_points(const cv::Size& board_size,
    float square_size)
{
    std::vector<cv::Point3f> ret_v;
    //TODO
    //Remenber: the first inner point has (1,1) in board coordinates.

    for (int i = 0; i < board_size.height; i++)
    {
        for (int j = 0; j < board_size.width; j++)
        {
            ret_v.push_back(cv::Point3f(float(j * square_size), float(i * square_size), 0.0f));
        }
    }

    CV_Assert(ret_v.size() == board_size.width * board_size.height);
    return ret_v;
}


bool
fsiv_find_chessboard_corners(const cv::Mat& img, const cv::Size& board_size,
    std::vector<cv::Point2f>& corner_points,
    const char* wname)
{ 
   //  CV_Assert(img.type() == CV_8UC3);

    bool was_found = false;
    cv::Mat mtGray;
    cv::cvtColor(img, mtGray, cv::COLOR_BGR2GRAY);
    
    was_found = cv::findChessboardCorners(img, board_size, corner_points, cv::CALIB_CB_FAST_CHECK);

    if (was_found) {
        std::cout << "Chess boards was found" << std::endl;
        cv::cornerSubPix(mtGray, corner_points, cv::Size(8, 6), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));


        cv::drawChessboardCorners(img, board_size, cv::Mat(corner_points), was_found);
        std::cout << "Drawing...." << std::endl;
    }
    else {
        std::cout << "Chess boards isn't found" << std::endl;
        return 0;
    }

  
    return was_found;
}


void fsiv_compute_camera_pose(const std::vector<cv::Point3f>& _3dpoints,
    const std::vector<cv::Point2f>& _2dpoints,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    cv::Mat& rvec,
    cv::Mat& tvec)
{
     CV_Assert(_3dpoints.size() >= 4 && _3dpoints.size() == _2dpoints.size());

    cv::solvePnP(_3dpoints, _2dpoints, camera_matrix, dist_coeffs, rvec, tvec);

    CV_Assert(rvec.rows == 3 && rvec.cols == 1 && rvec.type() == CV_64FC1);
    CV_Assert(tvec.rows == 3 && tvec.cols == 1 && tvec.type() == CV_64FC1);
}

void
fsiv_draw_axes(cv::Mat& img,
    const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,
    const cv::Mat& rvec, const cv::Mat& tvec,
    const float size, const int line_width)
{

    std::vector<cv::Point3f> points;
    points.push_back(cv::Point3f(0, 0, 0));
    points.push_back(cv::Point3f(size, 0, 0));
    points.push_back(cv::Point3f(0, size, 0));
    points.push_back(cv::Point3f(0, 0, size));

    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(points, rvec, tvec, camera_matrix, dist_coeffs, projected_points);

    cv::line(img, projected_points[0], projected_points[1], cv::Scalar(255, 0, 0), line_width);
    cv::line(img, projected_points[0], projected_points[2], cv::Scalar(0, 255, 0), line_width);
    cv::line(img, projected_points[0], projected_points[3], cv::Scalar(0, 0, 255), line_width);
}

void
fsiv_load_calibration_parameters(cv::FileStorage& fs,
    cv::Size& camera_size,
    float& error,
    cv::Mat& camera_matrix,
    cv::Mat& dist_coeffs,
    cv::Mat& rvec,
    cv::Mat& tvec)
{
    CV_Assert(fs.isOpened());

    fs["Camera_Size"] >> camera_size;
    fs["Error"] >> error;
    fs["Camera_Matrix"] >> camera_matrix;
    fs["Distortion_Coefficients"] >> dist_coeffs;
    fs["Rotation_Vector"] >> rvec;
    fs["Translation_Vector"] >> tvec;

   
    CV_Assert(fs.isOpened());
    CV_Assert(camera_matrix.type() == CV_64FC1 && camera_matrix.rows == 3 && camera_matrix.cols == 3);
    CV_Assert(dist_coeffs.type() == CV_64FC1 && dist_coeffs.rows == 1 && dist_coeffs.cols == 5);
    CV_Assert(rvec.type() == CV_64FC1 && rvec.rows == 3 && rvec.cols == 1);
    CV_Assert(tvec.type() == CV_64FC1 && tvec.rows == 3 && tvec.cols == 1);
    return;
}

void
fsiv_draw_3d_model(cv::Mat& img, const cv::Mat& M, const cv::Mat& dist_coeffs,
    const cv::Mat& rvec, const cv::Mat& tvec,
    const float size)
{
    CV_Assert(img.type() == CV_8UC3);


    cv::Mat R;
    cv::Rodrigues(rvec, R); // 3x3
    
    cv::Mat A = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(A.rowRange(0, 3).colRange(0, 3));
    tvec.copyTo(A.rowRange(0, 3).col(3));
    
    cv::Mat model_pts_3d;
    cv::perspectiveTransform(M, model_pts_3d, A);
    
    std::vector<cv::Point2f> model_pts_2d;
    cv::projectPoints(model_pts_3d, rvec, tvec, M, dist_coeffs, model_pts_2d);
    
    for (int i = 0; i < model_pts_2d.size(); i++)
    {
        cv::circle(img, model_pts_2d[i], size, cv::Scalar(0, 0, 255), -1);
    }



}

void
fsiv_project_image(const cv::Mat& input, cv::Mat& output,
    const cv::Size& board_size,
    const std::vector<cv::Point2f>& _2dpoints)
{
    CV_Assert(!input.empty() && input.type() == CV_8UC3);
    CV_Assert(!output.empty() && output.type() == CV_8UC3);
    CV_Assert(board_size.area() == _2dpoints.size());

    std::vector<cv::Point2f> board_corners;
    board_corners.push_back(cv::Point2f(0, 0));
    board_corners.push_back(cv::Point2f(board_size.width, 0));
    board_corners.push_back(cv::Point2f(board_size.width, board_size.height));
    board_corners.push_back(cv::Point2f(0, board_size.height));

    cv::Mat H = cv::findHomography(board_corners, _2dpoints);
    cv::warpPerspective(input, output, H, board_size);

}