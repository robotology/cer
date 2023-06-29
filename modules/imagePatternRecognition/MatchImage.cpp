#include <opencv2/opencv.hpp>
#include "imagePatternRecognition.h"

cv::Point ImagePatternRecognition::MatchImage( cv::Mat img, cv::Mat templ, cv::Mat result, int match_method )
{

  /// Create the result matrix
  int result_cols =  img.cols - templ.cols + 1;
  int result_rows = img.rows - templ.rows + 1;

  result.create( result_rows, result_cols, CV_32FC1 );

  /// Do the Matching and Normalize
  matchTemplate( img, templ, result, match_method );
  normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

  /// Localizing the best match with minMaxLoc
  double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
  cv::Point matchLoc;

  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

  /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
  if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
    { matchLoc = minLoc; }
  else
    { matchLoc = maxLoc; }

  return cv::Point( matchLoc.x + templ.cols/2 , matchLoc.y + templ.rows/2 );
}