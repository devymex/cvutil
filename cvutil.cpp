#include "cvutil.hpp"

#include <glog/logging.h>
#include <experimental/filesystem>

#include <fcntl.h> // for ::open
#include <unistd.h> // for ::close
#include <sys/ioctl.h> //for ::ioctl
#include <linux/videodev2.h> //for ::v4l2_capability, ::VIDIOC_QUERYCAP
namespace cvu {

cv::Scalar BGR_BLACK	= cv::Scalar(0, 0, 0);
cv::Scalar BGR_GREEN	= cv::Scalar(0, 255, 0);
cv::Scalar BGR_RED		= cv::Scalar(0, 0, 255);
cv::Scalar BGR_BLUE		= cv::Scalar(255, 0, 0);
cv::Scalar BGR_YELLOW	= cv::Scalar(0, 255, 255);
cv::Scalar BGR_PURPLE	= cv::Scalar(255, 0, 255);
cv::Scalar BGR_CYAN		= cv::Scalar(255, 255, 0);
cv::Scalar BGR_WHITE	= cv::Scalar(255, 255, 255);

template<typename _CB>
std::vector<std::string> FindFiles(std::string strPath, _CB callback) {
	namespace stdfs = std::experimental::filesystem;
	std::vector<std::string> results;
	for (auto &p : stdfs::directory_iterator(strPath)) {
		std::string strPath = p.path();
		if (callback(strPath)) {
			results.push_back(strPath);
		}
	}
	return results;
}

std::vector<std::string> SplitText(std::string strText, char cSep) {
	std::istringstream iss(strText);
	std::vector<std::string> results;
	for (std::string strSub; std::getline(iss, strSub, cSep); ) {
		results.emplace_back(std::move(strSub));
	}
	return std::move(results);
}

void ParseKeyValue(std::string strText, std::string &strKey,
		std::string &strValue, char cSep = '=') {
	auto subStrs = SplitText(strText, cSep);
	strKey.clear();
	strValue.clear();
	if (subStrs.size() > 0) {
		strKey = std::move(subStrs[0]);
		if (subStrs.size() > 1) {
			strValue = std::move(subStrs[1]);
		}
	}
}

int ParseCameraNameIndex(std::string &strCamName) {
	std::string strIndex;
	ParseKeyValue(strCamName, strCamName, strIndex, '@');
	int nIndex = -1;
	if (!strIndex.empty()) {
		nIndex = std::atoi(strIndex.c_str());
	}
	return nIndex;
}

std::string GetCameraName(std::string strDevPath) {
	std::string strResult;
	int fd = open(strDevPath.c_str(), O_RDONLY);
	if (fd >= 0) {
		v4l2_capability video_cap;
		if (ioctl(fd, VIDIOC_QUERYCAP, &video_cap) == 0) {
			strResult = (char*)video_cap.card;
		}
		close(fd);
	}
	return strResult;
}

std::string FindMatchedDevPath(std::string strCamName) {
	int nIndex = ParseCameraNameIndex(strCamName);
	auto allDevPathAry = FindFiles("/dev", [&](std::string strDevPath) {
			const std::string strDevVideo = "/dev/video";
			if (!strDevPath.compare(0, strDevVideo.size(), strDevVideo)) {
				auto strName = GetCameraName(strDevPath);
				return strName.find(strCamName) != std::string::npos;
			}
			return false;
		});
	CHECK(!allDevPathAry.empty()) << "Can't found camera " << strCamName;
	if (nIndex < 0) {
		nIndex = 0;
	} else {
		CHECK_LT(nIndex, allDevPathAry.size()) << "Exceeded maximum device ID";
		std::sort(allDevPathAry.begin(), allDevPathAry.end());
	}
	return allDevPathAry[nIndex];
}

cv::VideoCapture OpenCamera(std::string strCamInfo) {
	std::vector<std::string> settings = SplitText(strCamInfo, ',');
	CHECK(!settings.empty());
	std::string strCamName = settings.front();
	CHECK(!strCamName.empty()) << strCamInfo;
	settings.erase(settings.begin());

	if (strCamName.front() != '/') {
		strCamName = FindMatchedDevPath(strCamName);
	}
	cv::VideoCapture cam(strCamName, cv::CAP_V4L);
	CHECK(cam.isOpened()) << "Can't open device " << strCamName;

	for (auto &strSet : settings) {
		std::string strKey, strVal;
		ParseKeyValue(strSet, strKey, strVal);
		CHECK(!strKey.empty() && !strVal.empty()) << strSet;
		if (strKey == "FC") {
			cam.set(cv::CAP_PROP_FOURCC, *(int*)strVal.c_str());
		} else if (strKey == "FW") {
			cam.set(cv::CAP_PROP_FRAME_WIDTH, std::atoi(strVal.c_str()));
		} else if (strKey == "FH") {
			cam.set(cv::CAP_PROP_FRAME_HEIGHT, std::atoi(strVal.c_str()));
		} else if (strKey == "AE") {
			cam.set(cv::CAP_PROP_AUTO_EXPOSURE, std::atof(strVal.c_str()));			
		} else if (strKey == "EP") {
			cam.set(cv::CAP_PROP_EXPOSURE, std::atof(strVal.c_str()));
		}else {
			LOG(WARNING) << "Unkown property " << strSet;
		}
	}
	return cam;
}

StereoParams::StereoParams(std::string strParamFn, cv::Size imgSize) {
	Initialize(strParamFn, imgSize);
}

void StereoParams::Initialize(std::string strParamFn, cv::Size imgSize) {
	cv::FileStorage loader(strParamFn, cv::FileStorage::READ);
	CHECK(loader.isOpened()) << strParamFn;
	cv::Mat R1, R2, P1, P2, Q, K1, K2, T1, T2;
	loader["K1"] >> K1;
	loader["K2"] >> K2;
	loader["T1"] >> T1;
	loader["T2"] >> T2;
	loader["R1"] >> R1;
	loader["R2"] >> R2;
	loader["P1"] >> P1;
	loader["P2"] >> P2;
	//cv::Mat roi1, roi2;
	//loader["ROI1"] >> roi1;
	//loader["ROI2"] >> roi2;
	//cv::Rect roiRect1{*(cv::Rect*)roi1.data};
	//cv::Rect roiRect2{*(cv::Rect*)roi2.data};
	loader.release();
	cv::initUndistortRectifyMap(K1, T1, R1, P1, imgSize,
			CV_16SC2, rmap[0][0], rmap[0][1]);
	cv::initUndistortRectifyMap(K2, T2, R2, P2, imgSize,
			CV_16SC2, rmap[1][0], rmap[1][1]);
}

void StereoParams::RectifyStereoPair(cv::Mat &leftImg, cv::Mat rightImg) {
	CHECK(!rmap[0][0].empty() && !rmap[0][0].empty() &&
		!rmap[0][0].empty() && !rmap[0][0].empty());
	cv::remap(leftImg, leftImg, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
	cv::remap(rightImg, rightImg, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
}


void StereoSplit(cv::Mat combined, cv::Mat &img1, cv::Mat &img2, bool bVertical) {
	CHECK(!combined.empty());
	if (bVertical) {
		CHECK_EQ(combined.rows % 2, 0);
		int nHalf = combined.rows / 2;
		img1 = combined.rowRange(0, nHalf).clone();
		img2 = combined.rowRange(nHalf, combined.rows).clone();
	} else {
		CHECK_EQ(combined.cols % 2, 0);
		int nHalf = combined.cols / 2;
		img1 = combined.colRange(0, nHalf).clone();
		img2 = combined.colRange(nHalf, combined.cols).clone();
	}
}

cv::Mat StereoCombine(cv::Mat img1, cv::Mat img2, bool bVertical) {
	CHECK(!img1.empty());
	CHECK(img1.size() == img2.size());
	CHECK_EQ(img1.type(), img2.type());
	cv::Mat combined;
	if (bVertical) {
		combined = cv::Mat(img1.rows * 2, img1.cols, img1.type());
		img1.copyTo(combined.rowRange(0, img1.rows));
		img2.copyTo(combined.rowRange(img1.rows, combined.rows));
	} else {
		combined = cv::Mat(img1.rows, img1.cols * 2, img1.type());
		img1.copyTo(combined.colRange(0, img1.cols));
		img2.copyTo(combined.colRange(img1.cols, combined.cols));
	}
	return combined;
}

cv::Rect DrawAlignedText(cv::Mat img, const std::string &strText, cv::Point org,
		TEXT_ALIGN alignment, double dFontScale, cv::Scalar color,
		int nWeight, int nFontFace, int nLineType) {
	int nBaseLine = 0;
	cv::Size predSize = cv::getTextSize(strText,
			nFontFace, dFontScale, nWeight, &nBaseLine);
	cv::Rect predBox(org.x, org.y, predSize.width,
			predSize.height + nBaseLine - int(dFontScale * 2.0 - 0.75));

	switch (alignment & 3) {
	case 0: predBox.x -= (int)(predBox.width / 2. - 0.5); break;
	case 1: break;
	case 2: predBox.x -= predBox.width - 1; break;
	default: LOG(FATAL) << "Unknow alignment: " << (int)alignment;
	}
	switch (alignment >> 2) {
	case 0: predBox.y -= (int)(predBox.height / 2. - 0.5); break;
	case 1: break;
	case 2: predBox.y -= predBox.height - 1; break;
	default: LOG(FATAL) << "Unknow alignment: " << (int)alignment;
	}
	cv::putText(img, strText, predBox.tl() + cv::Point(0, predSize.height),
			nFontFace, dFontScale, color, nWeight, cv::LINE_AA);
	return predBox;
}

void DrawRotatedRectangle(cv::Mat& img, const cv::RotatedRect &rotBox,
		cv::Scalar &color, int nLineWidth, int nLineType) {
	cv::Point2f vertices[4];
	rotBox.points(vertices);
	for (int i = 0; i < 4; i++) {
		cv::line(img, vertices[i], vertices[(i + 1) % 4], color,
				nLineWidth, nLineType);
	}
}

} //namespace cvu

