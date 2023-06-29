#include "utils/draw_matches.h"

static void drawKeypoint(cv::Mat& img, const cv::Point2f& p, const cv::Scalar& color, int flags) {
  CV_Assert(!img.empty());
  Point center(cvRound(p.x * kDrawMultiplier), cvRound(p.y * kDrawMultiplier) );

  if(flags & DrawMatchesFlags::DRAW_RICH_KEYPOINTS ) {
    int radius = cvRound(2/2 * kDrawMultiplier); // KeyPoint::size is a diameter

    // draw the circles around keypoints with the keypoints size
    circle(img, center, radius, color, 1, CV_AA, kDrawShiftBits );
  }
  else {
    // draw center with R=3
    int radius = 3 * kDrawMultiplier;
    circle(img, center, radius, color, 1, CV_AA, kDrawShiftBits );
  }
}

void drawKeypoints(const cv::Mat& image, const std::vector<cv::Point2f>& points, 
                   cv::Mat& out_image, const cv::Scalar& sc_color, int flags ) {
  if(!(flags & DrawMatchesFlags::DRAW_OVER_OUTIMG)) {
    if(image.type() == CV_8UC3) {
      image.copyTo(out_image);
    }
    else if(image.type() == CV_8UC1) {
      cvtColor(image, out_image, CV_GRAY2BGR);
    }
    else {
      CV_Error(CV_StsBadArg, "Incorrect type of input image.\n" );
    }
  }

  RNG& rng=theRNG();
  bool is_rand_color = sc_color == Scalar::all(-1);

  CV_Assert(!out_image.empty() );
  for(int i=0; i<points.size(); ++i ) {
    cv::Scalar color = is_rand_color ? Scalar(rng(256), rng(256), rng(256)) : sc_color;
    drawKeypoint(out_image, points[i], color, flags );
  }
}

static void prepareImgAndDrawKeypoints(const cv::Mat& img1, const std::vector<cv::Point2f>& keypoints1,
                                       const cv::Mat& img2, const std::vector<cv::Point2f>& keypoints2,
                                       cv::Mat& out_img, cv::Mat& out_img1, cv::Mat& out_img2,
                                       const cv::Scalar& single_point_color, int flags) {
  Size size(img1.cols + img2.cols, MAX(img1.rows, img2.rows) );
  if( flags & DrawMatchesFlags::DRAW_OVER_OUTIMG ) {
    if( size.width > out_img.cols || size.height > out_img.rows )
        CV_Error( CV_StsBadSize, "out_img has size less than need to draw img1 and img2 together" );
    out_img1 = out_img(Rect(0, 0, img1.cols, img1.rows));
    out_img2 = out_img(Rect(img1.cols, 0, img2.cols, img2.rows));
  }
  else {
    out_img.create(size, CV_MAKETYPE(img1.depth(), 3));
    out_img = Scalar::all(0);
    out_img1 = out_img(Rect(0, 0, img1.cols, img1.rows));
    out_img2 = out_img(Rect(img1.cols, 0, img2.cols, img2.rows));

    if(img1.type() == CV_8U)
      cvtColor(img1, out_img1, CV_GRAY2BGR);
    else
      img1.copyTo(out_img1);

    if(img2.type() == CV_8U)
      cvtColor(img2, out_img2, CV_GRAY2BGR);
    else
      img2.copyTo(out_img2);
  }

  // draw keypoints
  if(!(flags & DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS) ) {
    cv::Mat draw_out_img1 = out_img(Rect(0, 0, img1.cols, img1.rows) );
    drawKeypoints( draw_out_img1, keypoints1, draw_out_img1, single_point_color, flags + DrawMatchesFlags::DRAW_OVER_OUTIMG );

    cv::Mat draw_out_img2 = outImg( Rect(img1.cols, 0, img2.cols, img2.rows) );
    drawKeypoints( draw_out_img2, keypoints2, draw_out_img2, single_point_color, flags + DrawMatchesFlags::DRAW_OVER_OUTIMG );
  }
}

static void drawMatch(cv::Mat& out_img, cv::Mat& out_img1, cv::Mat& out_img2, const cv::Point2f& kp1, 
                      const cv::Point2f& kp2, const cv::Scalar& match_color, int flags ) {
  RNG& rng = theRNG();
  bool is_rand_match_color = match_color == Scalar::all(-1);
  cv::Scalar color = is_rand_match_color ? Scalar( rng(256), rng(256), rng(256) ) : match_color;

  drawKeypoint(out_img1, kp1, color, flags );
  drawKeypoint(out_img2, kp2, color, flags );

  cv::Point2f dpt2 = Point2f(std::min(kp2.x+out_img1.cols, float(out_img.cols-1)), kp2.y );

  line(out_img, Point(cvRound(kp1.x*kDrawMultiplier), cvRound(kp1.y*kDrawMultiplier)),
       Point(cvRound(dpt2.x*kDrawMultiplier), cvRound(dpt2.y*kDrawMultiplier)), color, 
       1, CV_AA, kDrawShiftBits);
}

void drawMatches(const cv::Mat& img1, const std::vector<cv::Point2f>& keypoints1,
                 const cv::Mat& img2, const std::vector<cv::Point2f>& keypoints2, cv::Mat& out_img, 
                 const cv::Scalar& match_color, const cv::Scalar& single_point_color, int flags ) {
  cv::Mat out_img1, out_img2;
  prepareImgAndDrawKeypoints(img1, keypoints1, img2, keypoints2,
                             out_img, out_img1, out_img2, single_point_color, flags);
  // draw matches
  for(size_t m = 0; m < keypoints1.size(); m++ ) {
    drawMatch(out_img, out_img1, out_img2, keypoints1[m], keypoints2[m], match_color, flags);
  }
}
