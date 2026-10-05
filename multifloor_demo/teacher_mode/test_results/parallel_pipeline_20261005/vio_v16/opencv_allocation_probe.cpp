#include <opencv2/core.hpp>
#include <iostream>
int main(){cv::Mat a{512,512,CV_8UC1};cv::Mat b=cv::Mat(512,512,CV_8UC1);std::cout<<"braces_rows="<<a.rows<<" braces_cols="<<a.cols<<" braces_type="<<a.type()<<" explicit_rows="<<b.rows<<" explicit_cols="<<b.cols<<" explicit_type="<<b.type()<<'\n';}
