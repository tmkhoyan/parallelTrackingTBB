
#include "blob_conf.h"  // parallel dedector is defined in parallel_detector

template <typename T>
    inline void offsetKeyPointsY(std::vector<cv::KeyPoint> &kp, const T &offsetY){
    for(auto &p: kp )
        (p.pt).y = (p.pt).y + offsetY; //or only y 
    }

template <typename T>
    inline void offsetKeyPoints(std::vector<cv::KeyPoint> &kp, const T &offsetP){
    for(auto &p: kp )
        p.pt = p.pt+offsetP;
        // (p.pt).y = (p.pt).y+y; //or only y 
    }

class Par_Detector : public cv::ParallelLoopBody
{

private:
    cv::Mat &Img_in;
    int subMatYlen;
    cv::Ptr<cv::SimpleBlobDetector> detector;
    std::vector<cv::KeyPoint> &keypoints; //changed inside func

public:
    Par_Detector(cv::Mat Img_in_, std::vector<cv::KeyPoint> &keypoints_, int subMatYlen_) : //constructor
        Img_in(Img_in_), 
        subMatYlen(subMatYlen_), 
        detector(cv::SimpleBlobDetector::create(setupBlobDetector_())),
        keypoints(keypoints_) {}

    virtual void operator()(const cv::Range& range) const
    {
        // cv::imshow("aa",Img_in);
        // cv::waitKey(10000);
        // std::vector<cv::Point2d> foundPoints; foundPoints.reserve(1000);
        for(int i = range.start; i < range.end; i++)
        {
           // split matrices in long rows
            std::vector<cv::KeyPoint> keypoints1;
            cv::Rect subRect(0, (Img_in.rows/subMatYlen)*i  , Img_in.cols, Img_in.rows/subMatYlen);
            cv::Mat subImg_in(Img_in , subRect);
            //detect 
            detector->detect(subImg_in, keypoints1);
            //first draw in submatrix
            cv::drawKeypoints(subImg_in, keypoints1, subImg_in, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

            //ofsett to correct positio
            offsetKeyPoints<cv::Point2f>(keypoints1,subRect.tl());

            //insert in output vector
            keypoints.insert(keypoints.end(),std::make_move_iterator(keypoints1.begin()),std::make_move_iterator(keypoints1.end()));
            
            //if also want to keep keypoints vector
            // for(auto kp: keypoints1)
                // foundPoints.push_back(cv::Point2d((kp.pt).x,(kp.pt).y+(subRect.tl()).y));

            // if((keypoints.size())){
            //     cv::drawKeypoints(subImg_in, keypoints1, subImg_in, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            //     cout << "size is::::" << (keypoints1.size()) << endl;
            //     offsetKeyPoints<Point2f>(keypoints,subRect.tl());
            // }
           // cv::rectangle(Img_out,subRect, BLOB_CENT_COL); // visualize pattern of submatrices
           cv::rectangle(Img_in,subRect, BLOB_CENT_COL);
           cv::putText(Img_in, "Thread " + std::to_string(i), subRect.tl() + cv::Point(10,20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,255,0), 1, 1);
        }
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