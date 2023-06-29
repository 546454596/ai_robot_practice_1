// cvCorrectMatches function is Copyright (C) 2009, Jostein Austvik Jacobsen.
// cvTriangulatePoints function is derived from icvReconstructPointsFor3View, originally by Valery Mosyagin.
// HZ, R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision, Cambridge Univ. Press, 2003.

#include "utils/triangulate.h"

// This method is the same as icvReconstructPointsFor3View, with only a few numbers adjusted for two-view geometry
void cvTriangulatePoints(cv::CvMat *proj_matr1, cv::CvMat *proj_matr2, cv::CvMat *proj_points1,
                         cv::CvMat *proj_points2, cv::CvMat *points_4d) {
  if (proj_matr1 == 0 || proj_matr2 == 0 || proj_points1 == 0 || proj_points2 == 0 || points_4d == 0) {
    CV_Error(CV_StsNullPtr, "Some of parameters is a NULL pointer");
  }

  if (!CV_IS_MAT(proj_matr1) || !CV_IS_MAT(proj_matr2) || !CV_IS_MAT(proj_points1) ||
      !CV_IS_MAT(proj_points2) || !CV_IS_MAT(points_4d)) {
    CV_Error(CV_StsUnsupportedFormat, "Input parameters must be matrices");
  }

  int num_points;
  num_points = proj_points1->cols;

  if (num_points < 1) {
    CV_Error(CV_StsOutOfRange, "Number of points must be more than zero");
  }

  if (proj_points2->cols != num_points || points_4d->cols != num_points) {
    CV_Error(CV_StsUnmatchedSizes, "Number of points must be the same");
  }

  if (proj_points1->rows != 2 || proj_points2->rows != 2) {
    CV_Error(CV_StsUnmatchedSizes, "Number of proj points coordinates must be == 2");
  }

  if (points_4d->rows != 4) {
    CV_Error(CV_StsUnmatchedSizes, "Number of world points coordinates must be == 4");
  }

  if (proj_matr1->cols != 4 || proj_matr1->rows != 3 || proj_matr2->cols != 4 || proj_matr2->rows != 3) {
    CV_Error(CV_StsUnmatchedSizes, "Size of projection matrices must be 3x4");
  }

  cv::CvMat matra;
  double matra_dat[12];
  matra = cvMat(4, 3, CV_64F, matra_dat);

  cv::CvMat *proj_points[2];
  cv::CvMat *proj_matrs[2];

  proj_points[0] = proj_points1;
  proj_points[1] = proj_points2;

  proj_matrs[0] = proj_matr1;
  proj_matrs[1] = proj_matr2;

  /* Solve system for each point */
  int i, j;
  double matrb_dat[4], matr_pos_dat[4];
  cv::CvMat matrb = cvMat(4, 1, CV_64F, matrb_dat);
  cv::CvMat matr_pos = cvMat(3, 1, CV_64F, matr_pos_dat);

  for (i = 0; i < num_points; i++) {
    // Fill matrix for current point
    for (j = 0; j < 2; j++) {
      double x, y;
      x = cvmGet(proj_points[j], 0, i);
      y = cvmGet(proj_points[j], 1, i);
      for (int k = 0; k < 3; k++) {
        cvmSet(&matra, j * 2 + 0, k, x * cvmGet(proj_matrs[j], 2, k) - cvmGet(proj_matrs[j], 0, k));
        cvmSet(&matra, j * 2 + 1, k, y * cvmGet(proj_matrs[j], 2, k) - cvmGet(proj_matrs[j], 1, k));
      }
      int k = 3;
      cvmSet(&matrb, j * 2, 0, x * cvmGet(proj_matrs[j], 2, k) - cvmGet(proj_matrs[j], 0, k));
      cvmSet(&matrb, j * 2 + 1, 0, y * cvmGet(proj_matrs[j], 2, k) - cvmGet(proj_matrs[j], 1, k));
    }
    /* Solve system for current point */
    {
      cvSolve(&matra, &matrb, &matr_pos, CV_SVD);
      /* Copy computed point */
      cvmSet(points_4d, 0, i, cvmGet(&matr_pos, 0, 0)); // X
      cvmSet(points_4d, 1, i, cvmGet(&matr_pos, 1, 0)); // Y
      cvmSet(points_4d, 2, i, cvmGet(&matr_pos, 2, 0)); // Z
      cvmSet(points_4d, 3, i, -1);                      // W
    }
  }
}

void triangulatePoints(cv::InputArray proj_matr1, cv::InputArray proj_matr2, cv::InputArray proj_points1,
                       cv::InputArray proj_points2, cv::OutputArray points_4d) {
  cv::Mat matr1 = proj_matr1.getMat(), matr2 = proj_matr2.getMat();
  cv::Mat points1 = proj_points1.getMat(), points2 = proj_points2.getMat();

  if ((points1.rows == 1 || points1.cols == 1) && points1.channels() == 2) {
    points1 = points1.reshape(1, static_cast<int>(points1.total())).t();
  }
  if ((points2.rows == 1 || points2.cols == 1) && points2.channels() == 2) {
    points2 = points2.reshape(1, static_cast<int>(points2.total())).t();
  }

  cv::CvMat cv_matr1 = matr1, cv_matr2 = matr2;
  cv::CvMat cv_points1 = points1, cv_points2 = points2;

  points_4d.create(4, points1.cols, points1.type());
  cv::CvMat cv_points_4d = points_4d.getMat();

  cvTriangulatePts(&cv_matr1, &cv_matr2, &cv_points1, &cv_points2, &cv_points_4d);
}
