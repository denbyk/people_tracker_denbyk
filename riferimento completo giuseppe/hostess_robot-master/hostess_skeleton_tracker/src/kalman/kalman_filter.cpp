#include <iostream>
#include <deque>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#define PI 3.14159265358979323846
#define C 1.1547005383792515291871035014902

using namespace cv;
using namespace std;


struct mouse_info_struct { int x,y; };
struct mouse_info_struct mouse_info = {-1,-1}, last_mouse;

deque<Point> mousev,kalmanv,secondkalmanv;

void on_mouse(int event, int x, int y, int flags, void* param) {
	//if (event == CV_EVENT_LBUTTONUP)
	{
		last_mouse = mouse_info;
		mouse_info.x = x;
		mouse_info.y = y;

//		cout << "got mouse " << x <<","<< y <<endl;
	}
}

int main (int argc, char * const argv[]) {
    Mat img(800, 800, CV_8UC3);
    KalmanFilter KF(4, 2, 0);
    KalmanFilter KF2(4, 2, 0);
    Mat_<float> state(4, 1); /* (x, y, Vx, Vy) */
    Mat processNoise(4, 1, CV_32F);
    Mat_<float> measurement(2,1); measurement.setTo(Scalar(0));
    char code = (char)-1;

	namedWindow("mouse kalman");
	setMouseCallback("mouse kalman", on_mouse, 0);

        KF.statePre.at<float>(0) = mouse_info.x;
		KF.statePre.at<float>(1) = mouse_info.y;
		KF.statePre.at<float>(2) = 0;
		KF.statePre.at<float>(3) = 0;
		KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);

		KF2.statePre.at<float>(0) = mouse_info.x;
		KF2.statePre.at<float>(1) = mouse_info.y;
		KF2.statePre.at<float>(2) = 0;
		KF2.statePre.at<float>(3) = 0;
		KF2.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);

        setIdentity(KF.measurementMatrix);
        setIdentity(KF.processNoiseCov, Scalar::all(1e1));
        setIdentity(KF.measurementNoiseCov, Scalar::all(1e2));
        setIdentity(KF.errorCovPost, Scalar::all(1e-1));

        setIdentity(KF2.measurementMatrix);
		setIdentity(KF2.processNoiseCov, Scalar::all(1e1));
		setIdentity(KF2.measurementNoiseCov, Scalar::all(1e2));
		setIdentity(KF2.errorCovPost, Scalar::all(1e-1));

		mousev.clear();
		kalmanv.clear();
		secondkalmanv.clear();
        //randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));

		double ticks = (double)cv::getTickCount();

		Point oldpoint;

		int counter = 0;

        for(;;)
        {
        	double precTick = ticks;

			ticks = (double)cv::getTickCount();

			double dT = (ticks - precTick) / cv::getTickFrequency();

//            Point statePt(state(0),state(1));

            Mat prediction = KF.predict();
            Mat prediction2 = KF2.predict();

            code = (char)waitKey(30);

            cv::Mat meas(2, 1, CV_32F);

            Point measPt, statePt, statePt2;

            statePt = cv::Point(prediction.at<float>(0),prediction.at<float>(1));
            statePt2 = cv::Point(prediction2.at<float>(0),prediction2.at<float>(1));

			if( code != 'p' )
			{

				measurement(0) = meas.at<float>(0) = mouse_info.x;
				measurement(1) = meas.at<float>(1) = mouse_info.y;

				KF.transitionMatrix.at<float>(2) = dT;
				KF.transitionMatrix.at<float>(7) = dT;
				KF2.transitionMatrix.at<float>(2) = dT;
				KF2.transitionMatrix.at<float>(7) = dT;

				measPt = cv::Point(measurement(0),measurement(1));
				if(mousev.size() >= 90)
				{
					mousev.pop_front();
				}
				mousev.push_back(measPt);
				// generate measurement
				//measurement += KF.measurementMatrix*state;

				Mat estimated = KF.correct(measurement);
				statePt = cv::Point(estimated.at<float>(0),estimated.at<float>(1));
				if(kalmanv.size() >= 90)
				{
					kalmanv.pop_front();
				}
				kalmanv.push_back(statePt);

				measurement(0) = estimated.at<float>(0);
				measurement(1) = estimated.at<float>(1);

				Mat estimated2 = KF2.correct(measurement);
				statePt2 = cv::Point(estimated2.at<float>(0),estimated2.at<float>(1));
				if(secondkalmanv.size() >= 90)
				{
					secondkalmanv.pop_front();
				}
				secondkalmanv.push_back(statePt2);

			}

			cv::Mat error(2, 2, CV_32F, cv::Scalar::all(0));

			cv::Mat temp = (KF2.measurementMatrix * KF2.errorCovPre * KF2.measurementMatrix.t()) + KF2.measurementNoiseCov;

			error.at<float>(0, 0) = 1 / temp.at<float>(0, 0);
			error.at<float>(1, 1) = 1 / temp.at<float>(1, 1);

			cv::Mat mu(2, 1, CV_32F);
			mu.at<float>(0) = fabs(meas.at<float>(0) - statePt2.x);
			mu.at<float>(1) = fabs(meas.at<float>(1) - statePt2.y);

			cv::Mat eigenvalues, eigenvectors;

			cv::Mat result = ((mu.t() * error) * mu);

			cv::eigen(temp, eigenvalues, eigenvectors);

			//std::cout << eigenvalues << " " << eigenvectors << std::endl;
			//std::cout << eigenvalues.at<float>(0)<< " " << eigenvalues.at<float>(1) << std::endl;

			float a, b;
			float c;
			a = 3 * std::sqrt(eigenvalues.at<float>(0));
			b = 3 * std::sqrt(eigenvalues.at<float>(1));
			c = (180 / 3.14) * std::atan2(eigenvectors.at<float>(1, 0), eigenvectors.at<float>(0, 0));

			img = Scalar::all(0);

			//std::cout << a << " " << b << std::endl;
			ellipse(img, statePt2, cv::Size(a, b), c, 0, 360, cv::Scalar(255, 0, 0), 1);

//			if(result.operator cv::Mat().at<float>(0) <= 9)
//			{
//				std::cout << "ci sono" << std::endl;
//			}
//			else
//			{
//				std::cout << "non ci sono" << std::endl;
//			}

            // plot points
#define drawCross( center, color, d )                                 \
line( img, Point( center.x - d, center.y - d ),                \
Point( center.x + d, center.y + d ), color, 2, CV_AA, 0); \
line( img, Point( center.x + d, center.y - d ),                \
Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 )

            drawCross( statePt2, Scalar(255, 255, 255), 5);
            drawCross( statePt, Scalar(255,255,255), 5 );
            drawCross( measPt, Scalar(255,255,255), 5 );
//            drawCross( predictPt, Scalar(0,255,0), 3 );
//			line( img, statePt, measPt, Scalar(0,0,255), 3, CV_AA, 0 );
//			line( img, statePt, predictPt, Scalar(0,255,255), 3, CV_AA, 0 );

			for (int i = 0; i < mousev.size()-1; i++) {
				line(img, mousev[i], mousev[i+1], Scalar(255,0,0), 1);
			}
			for (int i = 0; i < kalmanv.size()-1; i++) {
				line(img, kalmanv[i], kalmanv[i+1], Scalar(0,255,0), 1);
			}
			for (int i = 0; i < secondkalmanv.size()-1; i++) {
				line(img, secondkalmanv[i], secondkalmanv[i+1], Scalar(0,0,255), 1);
			}

			if(secondkalmanv.size() > 1)
			{
				double dX = 5 * (secondkalmanv[secondkalmanv.size() - 1].x - secondkalmanv[secondkalmanv.size() - 2].x);
				double dY = 5 * (secondkalmanv[secondkalmanv.size() - 1].y - secondkalmanv[secondkalmanv.size() - 2].y);

				double main_radius = sqrt(pow(dX, 2) + pow(dY, 2));

				Size s(main_radius, main_radius / 2);
				double alphaRAD = acos(dX / main_radius);
				double alphaDEG = alphaRAD / PI * 180;

				if(dY < 0)
				{
					alphaDEG = 180-alphaDEG;
				}

				ellipse(img, statePt2, s, alphaDEG, 0, 360, Scalar(0, 0, 255), 1);

				Point F1(statePt2.x + dX / C, statePt2.y + dY / C);
				Point F2(statePt2.x - dX / C, statePt2.y - dY / C);

				double distance = sqrt(pow(statePt.x - F1.x, 2) + pow(statePt.y - F1.y, 2)) + sqrt(pow(statePt.x - F2.x, 2) + pow(statePt.y - F2.y, 2));
			}


//            randn( processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
//            state = KF.transitionMatrix*state + processNoise;

            imshow( "mouse kalman", img );
        }

    return 0;
}

