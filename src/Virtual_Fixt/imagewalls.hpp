#ifndef IMAGEWALLS_HPP
#define IMAGEWALLS_HPP
#include<string>
#include<math.h>
#include<armadillo>
#include <opencv2/opencv.hpp>//namespace cv
using namespace std;
const int THRESHOLD = 130;

struct neighbor{
    double dist;
    int x;
    int y;
  };

class imagewalls {
  
  cv::Mat image;//The grayscale image used as reference
  double boundary;//this is the largest allowable distance (in pixels) from the nearest black pixel
  double Kp, Kd;
  
  inline double euclidist (int x, int y, int xt, int yt){
    double d= sqrt(pow(x-xt,2.)+pow(y-yt,2.));
    return d;}
    
  public:
    int width,height;
    imagewalls(string imageName, double _boundary, double _Kp){
      image = cv::imread(imageName.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
      boundary = _boundary;
      Kp=_Kp;
      width = image.cols;
      height = image.rows;
    };
    arma::vec wallforce (int x, int y);
    neighbor findnearest(int x, int y);
    bool pixelcheck (int x, int y);
};//end main class definition

arma::vec imagewalls::wallforce(int x, int y){
  neighbor nearest = findnearest(x,y); 
  double Fmag = Kp*(boundary - nearest.dist);
  
  arma::vec Fwall = arma::zeros<arma::vec>(2);
  Fwall(0) = Fmag*(nearest.x-x)/nearest.dist;
  Fwall(1) = Fmag*(nearest.y-y)/nearest.dist;
  return Fwall;};

//imagewalls::
neighbor imagewalls::findnearest(int x, int y){
  neighbor nearpixel;
  int j = 1; bool found = false;
  while(found==false and j<=width){
    arma::vec dist = width*arma::ones<arma::vec>(8*j);
    arma::mat pixels = arma::zeros<arma::mat>(8*j,2);
    for(int m=-j;m<=j;m++){
      if(pixelcheck(y+m,x-j)){found=true;
        dist = arma::shift(dist,1);dist(0) = euclidist(x,y,x-j,y+m);
        pixels = arma::shift(pixels,1,0); pixels(0,0)=x-j; pixels(0,1)=y+m;
                             };
      if(pixelcheck(y+m,x+j)){found = true;
        dist = arma::shift(dist,1);dist(0) = euclidist(x,y,x+j,y+m);
        pixels = arma::shift(pixels,1,0); pixels(0,0)=x+j; pixels(0,1)=y+m;};
    };
    for(int n=-j+1;n<j;n++){
      if(pixelcheck(y-j,x+n)){found=true;
        dist = arma::shift(dist,1);dist(0) = euclidist(x,y,x+n,y-j);
        pixels = arma::shift(pixels,1,0); pixels(0,0)=x+n; pixels(0,1)=y-j;};
      if(pixelcheck(y+j,x+n)){found = true;
        dist = arma::shift(dist,1);dist(0) = euclidist(x,y,x+n,y+j);
        pixels = arma::shift(pixels,1,0); pixels(0,0)=x+n; pixels(0,1)=y+j;};
    };
    nearpixel.x = pixels(dist.index_min(),0);
    nearpixel.y = pixels(dist.index_min(),1);
    nearpixel.dist = dist(0);
    j++;    
  };
return nearpixel;};

bool imagewalls::pixelcheck (int y, int x){
  bool black = false; 
  if(x<=0 or x>=width or y<=0 or y>=height){//donothing
  }else if((double)image.at<uchar>(y,x)<THRESHOLD){black = true;};
  return black;};

#endif