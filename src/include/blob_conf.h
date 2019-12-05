/*
 * Description:    Parallel tracker and feature extraction method initial release
 *
 * Author:          Tigran Mkhoyan
 * Email :          t.mkhoyan@tudelft.nl
 */



   // thresholds
  #define BLOB_minThr 10;
  #define BLOB_maxThr 300;
  #define BLOB_maxThr 300;

  // #define //Area.
  #define BLOB_filtA true;
  #define BLOB_minA 2000;


  // #define // Circularity
  #define BLOB_filtCirc true;
  #define BLOB_minCirc 0.1;

  // #define //Convexity
  #define BLOB_filConv true;
  #define BLOB_minConv 0.87;

  // #define //Inertia
  #define BLOB_filtIn true;
  #define BLOB_mintInR 0.7;

  //drawing
  cv::Scalar BLOB_CENT_COL(0.0,255.0,0.0);
  size_t  BLOB_CENT_SIZE = 2;
