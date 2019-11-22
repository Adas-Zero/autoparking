#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

class Foo{
    public:
    cv::Mat frame;
    //--- INITIALIZE VIDEOCAPTURE
    cv::VideoCapture cap;
    int init(){
        std::cout << "init" << std::endl;
        // open the default camera using default API
        // cap.open(0);
        // OR advance usage: select any API backend
        int deviceID = 0;             // 0 = open default camera
        int apiID = cv::CAP_ANY;      // 0 = autodetect default API
        // open selected camera using selected API
        cap.open(deviceID + apiID);
        // check if we succeeded
        if (!cap.isOpened()) {
            std::cerr << "ERROR! Unable to open camera\n";
            return -1;
        }
        //--- GRAB AND WRITE LOOP
        std::cout << "Start grabbing" << std::endl
            << "Press any key to terminate" << std::endl;
    }

    void show(){
        std::cout << "show" << std::endl;;
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            std::cerr << "ERROR! blank frame grabbed\n";
            return;
        }
        // show live and wait for a key with timeout long enough to show images
        cv::imshow("Live", frame);
        cv::waitKey(1);
    }
};

extern "C" {
    Foo* Foo_new(){ return new Foo(); }
    void Foo_bar(Foo* foo){ foo->init(); }
    void Foo_end(Foo* foo){ foo->show(); }
}



