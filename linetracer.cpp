#include <opencv2/opencv.hpp>
#include "dxl.hpp"
#include <iostream>
#include <cstdlib>
using namespace cv;
using namespace std;
string src = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)320, height=(int)240, format=(string)NV12, framerate=(fraction)40/1 ! \
     nvvidconv flip-method=0 ! video/x-raw, width=(int)320, height=(int)240, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
int main()
{
    
    Dxl bugger;
    bugger.dxl_open();

    VideoCapture cap1(src, CAP_GSTREAMER);
    if (!cap1.isOpened()) { 
        cerr << "Camera open failed!" << endl; 
        return -1;
    }

    Mat frame;
    int ch,max=0,max_n,error,prv_error=0,Lspeed=150,Rspeed=150;
    int*p;
    Mat labels,stats,centroids;

    Point p_pt;
    Point o_pt(160,120);
    

    while (true) {
        // int64 t1=getTickCount();
        cap1 >> frame; //frame 받아오기
        if (frame.empty()){  //받아온 frame이 제대로 들어오는지 확인
            cerr << "frame empty!" << endl;
            break;
        }
        
        cvtColor(frame,frame,COLOR_BGR2GRAY); //color를 gray로 변환
        Mat dst=frame(Rect(0,frame.rows/3*2,frame.cols,frame.rows/3)); //영상 부분추출 
        dst= dst + 100 - mean(dst)[0];
        threshold(dst,dst,130,255,THRESH_BINARY); //영상 이진화
        int cnt=connectedComponentsWithStats(dst,labels,stats,centroids);  //영상 레이블링
        cvtColor(dst,dst,COLOR_GRAY2BGR);

        if(cnt>1){
            int*p=stats.ptr<int>(1);
            max=p[4];
            max_n=1;
            for(int i=2;i<cnt;i++){
                int*p=stats.ptr<int>(i);
                if(p[4]>max){
                    max=p[4];
                    max_n=i;
                }  
            }
            double*p_center=centroids.ptr<double>(max_n);
            p_pt=Point2d(p_center[0],p_center[1]);

            if(abs(p_pt.x-o_pt.x)>50){
                p_pt=o_pt;
            }
            cout<<"norm : "<<abs(p_pt.x-o_pt.x);
        }
        else{
            p_pt=o_pt;
        }
        o_pt=p_pt;

        circle(dst,p_pt,3,Scalar(0,0,255),-1);
        error=(dst.cols/2-p_pt.x)/2;
        cout<<"현제 error : "<<error<<'\t'<<endl;
        int ls=Lspeed-error;
        int rs=-(Rspeed+error);
        bugger.dxl_set_velocity(ls, rs);
        
        imshow("frame",dst);
        waitKey(5);

        if (bugger.kbhit()){
            if(bugger.getch()=='q')break;
        }
        // int64 t2=getTickCount();
        // double ms=(t2-t1)*1000/getTickFrequency();
        // cout<<"측정시간 : "<<ms<<"ms."<<endl;
    }
    
    bugger.dxl_close();
    return 0;
}