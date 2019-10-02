#ifndef DKLCOST_HPP
#define DKLCOST_HPP
#include<armadillo>
#include<math.h>
#include<omp.h>
#include <opencv2/opencv.hpp>//namespace cv
using namespace std;

template <class system>
class dklcost {
  system* sys;
  double L1,L2,T;
  int X1,X2;//index of relavant dimensions in xvector
  arma::uvec X_DKL;
  int K;//number of samples in the search domain
  arma::mat sigma; 
  int MAXINT=240;
  public:
    arma::mat xpast;
    arma::mat domainsamps;
    arma::vec qs_i,ps_i;
    double Q;
    arma::mat R;
    int T_index,t_now=0;
    cv::Mat img;
    dklcost(double _Q, arma::mat _R,int _K,arma::mat _sigma, int _X1,int _X2,cv::Mat _img,double _L1,double _L2,
        double _T,system *_sys){
      Q=_Q; R=_R; sys=_sys; K = _K; img = _img; T=_T; // initialize with Q, R, sys, img, and the domain
      X1 = _X1; X2=_X2; L1 = _L1; L2 = _L2; X_DKL<<X1<<X2; sigma=_sigma;
      T_index = T/sys->dt;
      omp_set_dynamic(0); // get rid of dynamic stuff
      omp_set_num_threads(16); // set the number of threads
      xpast.set_size(sys->Xcurr.n_rows,300/sys->dt);//initialize xpast to hold up to fiveminutes of data
      domainsamps.set_size(2,K);
      qs_i.zeros(K);ps_i.set_size(K);
      resample(); //initialize the uniform samples over the domain and the probability of each sample
    };
    double l (const arma::vec& x,const arma::vec& u,double ti);
    arma::vec dldx (const arma::vec&x, const arma::vec& u, double ti);
    double calc_cost (const arma::mat& x,const arma::mat& u);
    void xmemory (const arma::vec&);
    void resample ();
    void qs_disc(const arma::mat& x);
    double phid(const arma::vec& x);
};/////////end main class def

template<class system> double dklcost<system>::l (const arma::vec& x,const arma::vec& u,double ti){
      arma::vec xproj = sys->proj_func(x);
      arma::mat Qtemp = arma::zeros<arma::mat>(xproj.n_rows,xproj.n_rows);
      Qtemp(X1,X1)= pow(xproj(X1)/(L1+(0.1*L1)),8);
      Qtemp(X2,X2) = pow(xproj(X2)/(L2+(0.1*L2)),8);
      return arma::as_scalar((xproj.t()*Qtemp*xproj+u.t()*R*u)/2);
}

template<class system> arma::vec dklcost<system>::dldx (const arma::vec&x, const arma::vec& u, double ti){
  arma::vec xproj = sys->proj_func(x);
  arma::vec a; a.zeros(xproj.n_rows);
  arma::mat Qtemp = arma::zeros<arma::mat>(xproj.n_rows,xproj.n_rows);
  Qtemp(X1,X1)= pow(xproj(X1)/(L1+(0.1*L1)),8);
  Qtemp(X2,X2) = pow(xproj(X2)/(L2+(0.1*L2)),8);
  a=a+5*Qtemp*xproj;
  for(int n=0;n<ps_i.n_rows;n++){
    arma::vec s_x = domainsamps.col(n)-xproj.elem(X_DKL);
    a.elem(X_DKL)-= arma::as_scalar(ps_i(n)/qs_i(n))*exp(-0.5*arma::as_scalar(s_x.t()*sigma.i()*s_x))*sigma.i()*s_x;
  };
  return a;}

template<class system> double dklcost<system>::calc_cost (const arma::mat& x,const arma::mat& u){
  double J1 = 0.,Jtemp;
  arma::mat xjoined;
  
  if(t_now<=MAXINT){xjoined = arma::join_rows(xpast.cols(0,t_now),x);}
  else{xjoined = arma::join_rows(xpast.cols(t_now-MAXINT,t_now),x);};
  qs_disc(xjoined);
  J1 = -arma::as_scalar(arma::sum(ps_i%arma::log(qs_i)));
  J1 = Q*J1;
  for (int i = 0; i<x.n_cols; i++){
    arma::vec xproj = sys->proj_func(x.col(i));
    J1+=l(xproj,u.col(i),sys->tcurr+(double)i*sys->dt); 
  };
return J1;}

template<class system> void dklcost<system>::qs_disc(const arma::mat& x){//double start_time = omp_get_wtime();
  #pragma omp parallel for
  for(int n=0;n<qs_i.n_rows;n++){
    qs_i(n) = 0.;
    for(int j=0;j<x.n_cols;j++){
        arma::vec sj = domainsamps.col(n)-sys->proj_func(x.col(j)).elem(X_DKL);
        qs_i(n)+=sys->dt*exp(-0.5*arma::as_scalar(sj.t()*sigma.i()*sj));
      };
    };
  qs_i=arma::normalise(qs_i,1);//normalise the discrete pdf over the samples
  }
      
template<class system> void dklcost<system>::resample(){
  //Choose K samples over the domain [[-L1,L1],[-L2,L2]]
  //setup CDF of image for inverse transform sampling
  double epsilon = pow(10,-1);
  double* imgCDF = new double[img.size().width*img.size().height]; 
  double imgNorm = (cv::mean(img)[0]+epsilon)*img.size().width*img.size().height;
    imgCDF[0] = (img.at<uchar>(0,0)+epsilon)/imgNorm; 
    for(int m=0;m<img.size().height;m++){
      for(int n=0;n<img.size().width;n++){
        imgCDF[(m*img.size().height)+n+1] = imgCDF[(m*img.size().height)+n]+(img.at<uchar>(m,n)+epsilon)/imgNorm;
      };
    };//end CDF setup  
  //setup uniform real distribution for inverse transform sampling
  random_device rd; mt19937 eng(rd()); uniform_real_distribution<> distr(0,1);
  
  #pragma omp parallel for
  for(int n=0;n<ps_i.n_rows;n++){
    double k = distr(eng); 
    int i = 0;
    while (imgCDF[i+1]<=k){i++;};
    domainsamps(1,n) = round(i/img.size().width)/img.size().width-0.5;
    domainsamps(0,n) = fmod(i,img.size().width)/img.size().width-0.5;//(i-(i/2200)*2200)-0.5;
    ps_i(n) = phid(domainsamps.col(n));
  };
  ps_i=arma::normalise(ps_i,1);//normalise the discrete pdf over the samples
}

template<class system> void dklcost<system>::xmemory(const arma::vec& x){
  xpast.col(t_now)= x;
  t_now++;
  //resample();
}
template<class system> double dklcost<system>::phid(const arma::vec& x){
  double ind1 = (x(1)+L1)*img.size().width; double ind2 = (x(0)+L2)*img.size().width;
  double intensity = img.at<uchar>(round(ind1),round(ind2));
  double totalInt = cv::mean(img)[0]*(L1*2)*(L2*2);
  intensity = intensity/totalInt;
  return intensity;};
#endif