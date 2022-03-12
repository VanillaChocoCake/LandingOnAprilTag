
/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.
This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include "iostream"
#include "unistd.h"
#include "apriltag/apriltag_pose.h"
#include "opencv2/opencv.hpp"
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/tag25h9.h"
#include "apriltag/tag16h5.h"
#include "apriltag/tagCircle21h7.h"
#include "apriltag/tagCircle49h12.h"
#include "apriltag/tagCustom48h12.h"
#include "apriltag/tagStandard41h12.h"
#include "apriltag/tagStandard52h13.h"
#include "apriltag/common/getopt.h"
#include "cmath"
#include "ctime"

using namespace std;
using namespace cv;

int Desired_Num = 0;
int fontface = FONT_HERSHEY_PLAIN;
double fontscale = 1.8;
int Emergency_Message_Sent = 0;
int write_time = 0;
int Emergency = 0;
int Brake = 0;
clock_t start, now;
int dark_value = 0;
double contrast_value = 1.0;
int i = 0;

apriltag_family_t * create_tag(const char*famname)
{
    apriltag_family_t *tf = nullptr;
    if (!strcmp(famname, "tag36h11"))
    {
        tf = tag36h11_create();
    }
    else if (!strcmp(famname, "tag25h9"))
    {
        tf = tag25h9_create();
    }
    else if (!strcmp(famname, "tag16h5"))
    {
        tf = tag16h5_create();
    }
    else if (!strcmp(famname, "tagCircle21h7"))
    {
        tf = tagCircle21h7_create();
    }
    else if (!strcmp(famname, "tagCircle49h12"))
    {
        tf = tagCircle49h12_create();
    }
    else if (!strcmp(famname, "tagStandard41h12"))
    {
        tf = tagStandard41h12_create();
    }
    else if (!strcmp(famname, "tagStandard52h13"))
    {
        tf = tagStandard52h13_create();
    }
    else if (!strcmp(famname, "tagCustom48h12"))
    {
        tf = tagCustom48h12_create();
    }
    else
    {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }
    return (apriltag_family_t *) tf;
}

void destroy_tag(apriltag_family_t *tf, const char*famname)
{
    if (!strcmp(famname, "tag36h11"))
    {
        tag36h11_destroy(tf);
    }
    else if (!strcmp(famname, "tag25h9"))
    {
        tag25h9_destroy(tf);
    }
    else if (!strcmp(famname, "tag16h5"))
    {
        tag16h5_destroy(tf);
    }
    else if (!strcmp(famname, "tagCircle21h7"))
    {
        tagCircle21h7_destroy(tf);
    }
    else if (!strcmp(famname, "tagCircle49h12"))
    {
        tagCircle49h12_destroy(tf);
    }
    else if (!strcmp(famname, "tagStandard41h12"))
    {
        tagStandard41h12_destroy(tf);
    }
    else if (!strcmp(famname, "tagStandard52h13"))
    {
        tagStandard52h13_destroy(tf);
    }
    else if (!strcmp(famname, "tagCustom48h12"))
    {
        tagCustom48h12_destroy(tf);
    }
}

int calculate_degree(double x, double y, int middle_x, int middle_y)
{
    double relative_x = x-middle_x;
    double relative_y = y-middle_y;
    double Cos_degree = -relative_y/sqrt(pow(relative_x,2)+pow(relative_y,2));
    double degree = acos(Cos_degree) * 180.0 / 3.14159;
    if(x > middle_x)
    {
        degree = -degree;
    }
    return degree;
}

Mat Handle_Emergency(Mat frame, int width, int height)
{
    if(Emergency == 0)
    {
        return frame;
    }
    else if(Emergency == 1)
    {
        putText(frame,
                "Emergency situation!Switched to Manual!",
                Point(50, height / 4),
                fontface,
                fontscale,
                Scalar(0, 0, 0xff),
                1);
        if(Emergency_Message_Sent == 0)
        {
            if(access("/path/DataTransmission/data.txt", F_OK) != -1)
            {
                remove("/path/DataTransmission/data.txt");
            }
            FILE *fp=fopen("/path/DataTransmission/data.txt", "w+");
            fprintf(fp, "-666 -666 -666\n");
            fclose(fp);
            Emergency_Message_Sent = 1;
        }
        return frame;
    }
}

void remove_data_txt()
{
    if(access("/path/DataTransmission/data.txt", F_OK) != -1)
    {
        remove("/path/DataTransmission/data.txt");
    }
}

void write_data(int degree, int Horizontal_Distance, int Delta_Height)
{
    if(Brake == 1)
    {
        remove_data_txt();
        return;
    }
    if(write_time >= 30
    && access("/path/DataTransmission/data.txt", F_OK) != -1)
    {
        remove_data_txt();
        write_time = 0;
    }
    if(access("/path/DataTransmission/data.txt", F_OK) == -1)
    {
        write_time += 1;
        FILE *fp=fopen("/path/DataTransmission/data.txt", "w+");
        fprintf(fp, "%d %d %d\n",degree, Horizontal_Distance, Delta_Height);
        fclose(fp);
    }
    else
    {
        write_time += 1;
        FILE *fp=fopen("/path/DataTransmission/data.txt", "at+");
        fprintf(fp, "%d %d %d\n",degree, Horizontal_Distance, Delta_Height);
        fclose(fp);
    }
    start = clock();
}

int main(int argc, char *argv[])
{
    remove_data_txt();
    int width = 0;
    int height = 0;
    cout << "目标Apriltag序号：" << endl;
    cin >> Desired_Num;
    apriltag_detection_info_t info;
    //info.tagsize = 14.4;//cm
    info.tagsize = 19.2;
    info.fx = 931.6;
    info.fy = 1356.6;
    info.cx = 615.5;
    info.cy = 530.8;
    getopt_t *getopt = getopt_create();
    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 1, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, 't', "threads", "1", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "2.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    // Initialize camera
    int camera = 0;
    while(true)
    {
        cout << "摄像头序号：" << endl;
        cin >> camera;
        VideoCapture test(camera);
        if (!test.isOpened())
        {
            cerr << "摄像头未打开！" << endl;
            continue;
        }
        break;
    }
    VideoCapture cap(camera);
    if(!cap.isOpened())
    {
        cerr << "摄像头未打开！正在退出..." << endl;
        exit(0);
    }
    cout << "摄像头图像已捕获！" << endl;
    cout << cap.get(CAP_PROP_FPS) << endl;
    width = 1280;
    height = 1024;
    //cout << cap.get(CAP_PROP_FRAME_WIDTH) << " " << cap.get(CAP_PROP_FRAME_HEIGHT) << endl;
    int middle_x = width / 2;
    int middle_y = height / 2;
    // Initialize tag detector with options
    apriltag_family_t *tf = nullptr;
    const char *famname = getopt_get_string(getopt, "family");
    tf = create_tag(famname);
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    Mat frame, gray;
    start = clock();
    while (true)
    {
        cap.read(frame);
        //Mat ori_frame;
        try
        {
            //ori_frame = frame.clone();
            cvtColor(frame, gray, COLOR_BGR2GRAY);
            gray.convertTo(gray, -1, contrast_value, dark_value);
            resize(frame, frame, Size(1280, 1024), 0, 0, INTER_CUBIC);
            //resize(ori_frame, ori_frame, Size(1280, 1024), 0, 0, INTER_CUBIC);
            resize(gray, gray, Size(1280, 1024), 0, 0, INTER_CUBIC);
            GaussianBlur(gray, gray, Size(5, 5), 0);
            medianBlur(gray, gray, 5);
        }
        catch(...)
        {
            continue;
        }
        image_u8_t im =
                {
                        .width = gray.cols,
                        .height = gray.rows,
                        .stride = gray.cols,
                        .buf = gray.data
                };
        zarray_t *detections = apriltag_detector_detect(td, &im);
        if(Emergency == 0)
        {
            for (int i = 0; i < zarray_size(detections); i++)
            {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);
                info.det = det;
                if (det->id != Desired_Num)
                {
                    continue;
                }
                line(frame,
                     Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[1][0], det->p[1][1]),
                     Scalar(0, 0xff, 0),
                     2);
                line(frame,
                     Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0, 0, 0xff),
                     2);
                line(frame,
                     Point(det->p[1][0], det->p[1][1]),
                     Point(det->p[2][0], det->p[2][1]),
                     Scalar(0xff, 0, 0),
                     2);
                line(frame,
                     Point(det->p[2][0], det->p[2][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0xff, 0, 0),
                     2);

                apriltag_pose_t pose;
                double err = estimate_tag_pose(&info, &pose);
                int degree = calculate_degree(det->c[0],
                                              det->c[1],
                                              middle_x,
                                              middle_y);
                line(frame,
                     Point(middle_x, middle_y),
                     Point(middle_x, 0),
                     Scalar(0xff, 0, 0),
                     2);
                line(frame,
                     Point(middle_x, middle_y),
                     Point(det->c[0], det->c[1]),
                     Scalar(0xff, 0, 0),
                     2);
                String deg = to_string(degree) + "deg";
                double horizontal_distance_double =
                        sqrt(pose.t->data[0] * pose.t->data[0] + pose.t->data[1] * pose.t->data[1]);
                int Horizontal_Distance = horizontal_distance_double;
                double delta_height_double = pose.t->data[2];
                int Delta_Height = delta_height_double;
                //Not accurate, but I think it's fine.
                string Horizontal_Distance_s = "Horizontal Distance:" + to_string(Horizontal_Distance) + "cm";
                string Delta_Height_s = "Delta Height:" + to_string(Delta_Height) + "cm";
                stringstream ss;
                ss << det->id;
                //The id of the Apriltag.
                String text = ss.str();
                putText(frame,
                        text,
                        Point(det->c[0], det->c[1]),
                        fontface,
                        fontscale,
                        Scalar(0xff, 0x99, 0),
                        2);
                putText(frame,
                        Horizontal_Distance_s + "    " + Delta_Height_s,
                        Point(50, height * 3 / 4),
                        fontface,
                        fontscale,
                        Scalar(0xff, 0x99, 0),
                        2);
                putText(frame,
                        deg,
                        Point(middle_x, middle_y),
                        fontface,
                        fontscale,
                        Scalar(0xff, 0, 0x11),
                        2);
                write_data(degree, Horizontal_Distance, Delta_Height);
                cout << pose.t->data[0] << " " << pose.t->data[1] << " " << pose.t->data[2] << endl;
                cout << "对比度系数：" << contrast_value << endl;
                cout << "dark_value:" << dark_value << endl;
                cout << "degree:" << degree << endl;
            }
        }
        apriltag_detections_destroy(detections);
        now = clock();
        auto time_spent = (double)(now - start) / CLOCKS_PER_SEC;
        if(time_spent >= 0.8&& Emergency == 0)
        {
            remove_data_txt();
        }
        char cmd = waitKey(5);
        frame = Handle_Emergency(frame, width, height);
        if (cmd == int('b') && Emergency == 0)
        {
            Emergency = 1;
            imwrite("/home/sunpeihan/Desktop/ori" + to_string(i) + ".png",frame);
            imwrite("/home/sunpeihan/Desktop/frame" + to_string(i) + ".png", frame);
            imwrite("/home/sunpeihan/Desktop/gray" + to_string(i) + ".png", gray);
            i += 1;
        }
        else if(cmd == int('w'))
        {
            dark_value += 10;
        }
        else if(cmd == int('s'))
        {
            dark_value -= 10;
        }
        else if(cmd == int('e'))
        {
            contrast_value += 0.1;
            if(contrast_value > 5.0)
            {
                contrast_value = 5.0;
            }
        }
        else if(cmd == int('d'))
        {
            contrast_value -= 0.1;
            if(contrast_value < 0.1)
            {
                contrast_value = 0.1;
            }
        }
        else if(cmd >= 0 && Emergency == 1)
        {
            break;
        }
        //imshow("原图", frame);
        imshow("修改过的视频流", frame);
        imshow("灰度图", gray);
    }
    apriltag_detector_destroy(td);
    destroy_tag(tf,famname);
    getopt_destroy(getopt);
    return 0;
}
