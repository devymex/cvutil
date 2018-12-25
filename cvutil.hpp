#ifndef __CVCAM_HPP
#define __CVCAM_HPP

#include <opencv2/opencv.hpp> //for cv::VideoCapture
#include <string> // for std::string

namespace cvu {

extern cv::Scalar BGR_GREEN;
extern cv::Scalar BGR_RED;
extern cv::Scalar BGR_BLUE;
extern cv::Scalar BGR_BLACK;
extern cv::Scalar BGR_WHITE;
extern cv::Scalar BGR_YELLOW;
extern cv::Scalar BGR_PURPLE;
extern cv::Scalar BGR_CYAN;

cv::VideoCapture OpenCamera(std::string strCamInfo);

class StereoParams {
private:
	cv::Mat rmap[2][2];
public:
	StereoParams() = default;
	StereoParams(std::string strParamFn, cv::Size imgSize);
	void Initialize(std::string strParamFn, cv::Size imgSize);
	void RectifyStereoPair(cv::Mat &leftImg, cv::Mat rightImg);
};

void StereoSplit(cv::Mat combined, cv::Mat &img1, cv::Mat &img2,
		bool bVertical = false);

cv::Mat StereoCombine(cv::Mat img1, cv::Mat img2, bool bVertical = false);

enum TEXT_ALIGN {TA_CENTERMIDDLE = 0,
				 TA_CENTERLEFT = 1, TA_CENTERRIGHT = 2,
				 TA_TOPMIDDEL = 4, TA_BOTTOMMIDDLE = 8,
				 TA_TOPLEFT = (TA_CENTERLEFT | TA_TOPMIDDEL),
				 TA_TOPRIGHT = (TA_CENTERRIGHT | TA_TOPMIDDEL),
				 TA_BOTTOMLEFT = (TA_CENTERLEFT | TA_BOTTOMMIDDLE),
				 TA_BOTTOMRIGHT = (TA_CENTERRIGHT | TA_BOTTOMMIDDLE)
				 };

cv::Rect DrawAlignedText(cv::Mat img, const std::string &strText, cv::Point org,
		TEXT_ALIGN alignment = TA_CENTERMIDDLE, double dFontScale = 1.,
		cv::Scalar color = cv::Scalar::all(255), int nWeight = 1,
		int nFontFace = cv::FONT_HERSHEY_COMPLEX, int nLineType = cv::LINE_AA);

}
#endif
