#ifndef MDA_HPP
#define MDA_HPP
#include<armadillo>
#include <iostream>


class mda{
    double gamma;
    bool replace;
    
    public:
    mda(double _gamma,bool _replace){gamma = _gamma;replace = _replace;};
    
    
    arma::vec filter(const arma::vec& uc,const arma::vec& user){
        double dotprod = arma::as_scalar(dot(uc,user));
        double phi=acos(dotprod/(arma::norm(uc)*arma::norm(user)));
        arma::vec ucurr;
        if(dotprod>0 && abs(phi)<=gamma){
            ucurr = user;
            }else if(replace==true) {ucurr = uc;
                }else {ucurr = arma::zeros(size(uc));}
     return ucurr;}
};

#endif