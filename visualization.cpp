/**
 * @file visualization.cpp
 * @author: Kuang Fangjun <csukuangfj at gmail dot com>
 * @date August 13, 2018
 */

#include "lane_line/visualization.h"
#include "lane_line/Line.h"
#include "lane_line/Drawing.h"
#include "lane_line/Transformation.h"

#include <vector>
#include <iostream>


namespace tt
{


void drawYolodetectionCallback(const perception_msgs::Objects::ConstPtr &msg, cv::Mat tmp_raw_image)
{
    std::vector<perception_msgs::Object> v = msg->objects;
    for(const auto & itor : v)
    {
        int box_x = (int)itor.bounding_box_size.x/2;
        int box_y = (int)itor.bounding_box_size.y/2;
        
        cv::Rect rect((int)itor.bounding_box_center.x - box_x, (int)itor.bounding_box_center.y - box_y, (int)itor.bounding_box_size.x, (int)itor.bounding_box_size.y);

        cv::rectangle(tmp_raw_image, rect, cv::Scalar(255,0,0), 2);
    }

}
cv::Mat visualizeResult(std::shared_ptr<tt::LaneLineDetector> detector,
                        const cv::Mat &raw_image,
                        bool draw_left_right_boundary,
                        bool only_show_original)
{
    /*
     *  ----------------------------------------------------
     *  |Segmentation_result || Narrowing_result || Curves |
     *  ----------------------------------------------------
     *  |              original image                      |
     *  ----------------------------------------------------
     */
    cv::Mat segmentation_result = detector->preprocessing()->getResult();
    cv::Mat narrowing_result = detector->narrowing()->getResult();

    cv::Mat ipm_image = detector->getIpmImage();

    cv::Mat tmp_raw_image = raw_image.clone();
    
    /*--------------Z.H-----start------------*/
    // draw yolo detection
    /*
    */
    auto dst = detector->ipm_tf()->computeRawImage(ipm_image, tmp_raw_image.size());
    cv::addWeighted(tmp_raw_image, 0.7, dst, 0.3, 0, tmp_raw_image);
    // draw Line
    std::vector<tt::Line> line_tmp;
    std::vector<tt::Line> line_left_tmp;
    std::vector<tt::Line> line_right_tmp;
    std::vector<cv::Point2f> points_tmp;
    
    bool clip_line = true;
    auto& curves = detector->getKalmanFilteredCenterCurves();
    for (const auto &c : curves)
    {
        const auto line = dynamic_cast<const tt::Line *>(c.get());
        CHECK_NOTNULL(line);
        tt::Drawing::lineInPlace(ipm_image,
                                 tmp_raw_image,
                                 detector->ipm_tf()->getIPMToImage(),
                                 *line,
                                 tt::Drawing::green(), 3, clip_line);
    }

    auto& left_curves = detector->getKalmanFilteredLeftCurves();
    for (const auto &c : left_curves)
    {
        const auto line = dynamic_cast<const tt::Line *>(c.get());
        CHECK_NOTNULL(line);
        line_left_tmp.push_back(*line);
        if (draw_left_right_boundary)
        {
            tt::Drawing::lineInPlace(ipm_image,
                                 tmp_raw_image,
                                 detector->ipm_tf()->getIPMToImage(),
                                 *line,
                                 tt::Drawing::red(), 3, clip_line);
        }
        
    }

    auto& right_curves = detector->getKalmanFilteredRightCurves();
    for (const auto &c : right_curves)
    {
        const auto line = dynamic_cast<const tt::Line *>(c.get());
        CHECK_NOTNULL(line);
        line_right_tmp.push_back(*line);
        if (draw_left_right_boundary)
        {
            tt::Drawing::lineInPlace(ipm_image,
                                 tmp_raw_image,
                                 detector->ipm_tf()->getIPMToImage(),
                                 *line,
                                 tt::Drawing::blue(), 3, clip_line);
        }  
    }

    cv::Point2f truck_pos_ = cv::Point2f(960,1200);

    static cv::Point2f truck_pos = tt::Transformation::transformPoint(
                detector->ipm_tf()->getImageToIPM(),
                truck_pos_);
    for(int i = 1; i < line_right_tmp.size(); i++)
    {
        std::vector<Point> points;
        line_right_tmp[i].computeStartEndPoints(ipm_image, points);
        cv::Point2f p2 = cv::Point2f(points[1]);
        if((p2.x - truck_pos.x)>0)
        {
            line_tmp.push_back(line_right_tmp[i-1]);
            break;
        }
    }
    for(int i = 0; i < line_left_tmp.size(); i++)
    {
        std::vector<Point> points;
        line_left_tmp[i].computeStartEndPoints(ipm_image, points);
        cv::Point2f p2 = cv::Point2f(points[1]);
        if((p2.x - truck_pos.x)>0)
        {
            line_tmp.push_back(line_left_tmp[i]);
            break;
        }
    }


    for(const auto &l : line_tmp)
    {
        std::vector<Point> points;
        l.computeStartEndPoints(ipm_image, points);
        std::vector<cv::Point2f> cv_points {cv::Point2f(points[0]),
                                            cv::Point2f(points[1])};

        cv_points = tt::Transformation::transformPoints(detector->ipm_tf()->getIPMToImage(), cv_points);
        tt::Line line;
        line.initWithPoints(tt::Point(cv_points[0]), tt::Point(cv_points[1]));
        line.computeStartEndPoints(tmp_raw_image, points);

        cv::Point p1 = cv::Point2f(points[0]);
        cv::Point p2 = cv::Point2f(points[1]);

        int start_x = 0;
        int start_y = cvRound(tmp_raw_image.rows/3.0f);
        cv::clipLine(cv::Rect(start_x, start_y, 1920, 1200), p1, p2);

        points_tmp.push_back(p1);
        points_tmp.push_back(p2);
        //cv::circle(tmp_raw_image, p1, 10, cv::Scalar(0,0,255), -1, 8);
        //cv::circle(tmp_raw_image, p1, 10, cv::Scalar(255,0,0), -1, 8);

    }
    std::cout << "HHH: " << points_tmp.size() << std::endl;
    for(auto &i: points_tmp)
    {
        std::cout << "LL:" << i.x <<"; "<< i.y << std::endl;
    }
    
    if(points_tmp.size() == 4 )
    {
        cv::Point pt[1][4];
        pt[0][0] = points_tmp[0];
        pt[0][1] = points_tmp[2];
        pt[0][2] = points_tmp[3];
        pt[0][3] = points_tmp[1];
        const cv::Point* ppt[1]={pt[0]};
        int npt[1] = {4};
        cv::fillPoly(tmp_raw_image,ppt,npt,1,cv::Scalar(0,255,0)); 
    }
    /*
    if(points_tmp.size() > 8)
    {
        for(int i = 2; i < points_tmp.size()-4;)
        {
            
            bool flag = false;
            if(points_tmp[5].x > 1 && i < 5)
            {
                cv::Point pt[1][5];
                pt[0][0] = points_tmp[i];
                pt[0][1] = points_tmp[i+2];
                pt[0][2] = points_tmp[i+3];
                pt[0][3] = cv::Point(0,1199);
                pt[0][2] = points_tmp[i+1];
                const cv::Point* ppt[1]={pt[0]};
                int npt[1] = {5};
                cv::fillPoly(tmp_raw_image,ppt,npt,1,cv::Scalar(0,255,0));
                flag = true; 
            }
            if(flag)
            {
                i+=4;
            }
            cv::Point pt[1][4];
            pt[0][0] = points_tmp[i];
            pt[0][1] = points_tmp[i+2];
            pt[0][2] = points_tmp[i+3];
            pt[0][3] = points_tmp[i+1];
            const cv::Point* ppt[1]={pt[0]};
            int npt[1] = {4};
            cv::fillPoly(tmp_raw_image,ppt,npt,1,cv::Scalar(0,255,0)); 
            cv::addWeighted(dst,0.6,src,0.4,0,dst);
            i+=4;
        }
    }*/

    line_tmp.clear();
    line_left_tmp.clear();
    line_right_tmp.clear();
    points_tmp.clear();

    /*--------------Z.H----end-------------*/

   
    /*
    auto& curves = detector->getKalmanFilteredCenterCurves();
    // std::cout << "visualization step - center line size:" << curves.size() << std::endl;
    for (const auto &c : curves)
    {
        const auto line = dynamic_cast<const tt::Line *>(c.get());
        CHECK_NOTNULL(line);
        // std::cout << "visualization step - center line para:" << line->getThetaInDegree() << "---" << line->getD() << std::endl;
        tt::Drawing::lineInPlace(ipm_image,
                                 tmp_raw_image,
                                 detector->ipm_tf()->getIPMToImage(),
                                 *line,
                                 tt::Drawing::green(), 3, true);
    }

    if (draw_left_right_boundary)
    {
        bool clip_line = true;
        auto& left_curves = detector->getKalmanFilteredLeftCurves();
        // std::cout << "visualization step - left line size:" << left_curves.size() << std::endl;
        for (const auto &c : left_curves)
        {
            const auto line = dynamic_cast<const tt::Line *>(c.get());
            CHECK_NOTNULL(line);
            // std::cout << "visualization step - left line para:" << line->getThetaInDegree() << "---" << line->getD() << std::endl;
            tt::Drawing::lineInPlace(ipm_image,
                                     tmp_raw_image,
                                     detector->ipm_tf()->getIPMToImage(),
                                     *line,
                                     tt::Drawing::red(), 3, clip_line);
        }

        auto& right_curves = detector->getKalmanFilteredRightCurves();
        // std::cout << "visualization step - right line size:" << right_curves.size() << std::endl;
        for (const auto &c : right_curves)
        {
            const auto line = dynamic_cast<const tt::Line *>(c.get());
            CHECK_NOTNULL(line);
            // std::cout << "visualization step - right line para:" << line->getThetaInDegree() << "---" << line->getD() << std::endl;
            tt::Drawing::lineInPlace(ipm_image,
                                     tmp_raw_image,
                                     detector->ipm_tf()->getIPMToImage(),
                                     *line,
                                     tt::Drawing::blue(), 3, clip_line);
        }
    }*/
    

    if (only_show_original)
    {
        cv::resize(tmp_raw_image, tmp_raw_image, cv::Size(960, 600));
        return tmp_raw_image;
    }

    cv::Mat tmp;
    int cols = ipm_image.cols * 3;
    cv::resize(tmp_raw_image, tmp, cv::Size(cols,
                                            static_cast<int>(1.0f*raw_image.rows*raw_image.cols/cols)));  // NOLINT

    int rows = segmentation_result.rows + tmp.rows;

    cv::Mat res = cv::Mat::zeros(rows, cols, CV_8UC3);
    tmp.copyTo(res(cv::Rect(0, ipm_image.rows, tmp.cols, tmp.rows)));

    cv::cvtColor(segmentation_result, tmp, cv::COLOR_GRAY2BGR);
    tmp.copyTo(res(cv::Rect(0, 0, segmentation_result.cols, segmentation_result.rows)));        // NOLINT

    std::vector<cv::Mat> mn(3);
    cv::split(narrowing_result, mn);
    mn[1].convertTo(mn[1], CV_8UC1);
    cv::cvtColor(mn[1], tmp, cv::COLOR_GRAY2BGR);
    tmp.copyTo(res(cv::Rect(segmentation_result.cols, 0, tmp.cols, tmp.rows)));

    tmp = ipm_image.clone();
    tmp.copyTo(res(cv::Rect(segmentation_result.cols + narrowing_result.cols, 0, tmp.cols, tmp.rows)));     // NOLINT

    cv::resize(res, res, cv::Size(960, 600));
    return res;
}

}  // namespace tt

