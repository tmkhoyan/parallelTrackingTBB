/*
 * Description:    computes blob in parallel 
 *
 * Author:          Tigran Mkhoyan
 * Email :          t.mkhoyan@tudelft.nl
 */
#include "blob_conf.h"  // parallel dedector is defined in parallel_detector

class Par_Detector : public cv::ParallelLoopBody
{

private:
    cv::Mat Img_in;
    cv::Mat& Img_out; //changed inside func
    int subMatYlen;
    cv::Ptr<cv::SimpleBlobDetector> detector;
    std::vector<cv::KeyPoint> &keypoints; //changed inside func

public:
    Par_Detector(cv::Mat Img_in_, cv::Mat& Img_out_, std::vector<cv::KeyPoint> &keypoints_,  int subMatYlen_) : //constructor
        Img_in(Img_in_), 
        Img_out(Img_out_),
        subMatYlen(subMatYlen_), 
        detector(cv::SimpleBlobDetector::create(setupBlobDetector_())),
        keypoints(keypoints_) {}

    virtual void operator()(const cv::Range& range) const
    {
        //reserve max 100 keypoints
        // std::vector<cv::KeyPoint> keypoints; 
        keypoints.reserve(100);

        for(int i = range.start; i < range.end; i++)
        {
           // work on chunks of matrices
            cv::Mat subImg_in(Img_in  , cv::Rect(0, (Img_in.rows/subMatYlen)*i  , Img_in.cols, Img_in.rows/subMatYlen));
            cv::Mat subImg_out(Img_out, cv::Rect(0, (Img_out.rows/subMatYlen)*i ,Img_out.cols, Img_out.rows/subMatYlen));
            //detect 
            detector->detect(subImg_in, keypoints);
        }
         //draw
         cv::drawKeypoints(Img_out, keypoints, Img_out, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
         //draw circle in the middle
         // cv::circle(Img_out,cv::Point2d(keypoints[0].pt), BLOB_CENT_SIZE, BLOB_CENT_COL, -1);//, 8, 0 );
    }
    //function to setup parameters
    cv::SimpleBlobDetector::Params setupBlobDetector_(){
    // Setup SimpleBlobDetector parameters.
    cv::SimpleBlobDetector::Params params;

    // Change thresholds
    params.minThreshold         = BLOB_minThr;
    params.maxThreshold         = BLOB_maxThr;

    // Filter by Area.
    params.filterByArea         = BLOB_filtA;
    params.minArea              = BLOB_minA;

    // Filter by Circularity
    params.filterByCircularity  = BLOB_filtCirc;
    params.minCircularity       = BLOB_minCirc;

    // Filter by Convexity
    params.filterByConvexity    = BLOB_filConv;
    params.minConvexity         = BLOB_minConv;

    // Filter by Inertia
    params.filterByInertia      = BLOB_filtIn;
    params.minInertiaRatio      = BLOB_mintInR;
    return params;

    }
};