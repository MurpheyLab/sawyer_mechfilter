#ifndef DOUBLEINT_HPP
#define DOUBLEINT_HPP
#include<armadillo>
#include"rk4_int.hpp"

const double PI = 3.1415926535987;

class DoubleInt {
    public:
        double dt,B=0.01;
        double tcurr=0.0;
        arma::vec Xcurr, Ucurr;
        arma::mat xdlist;
        DoubleInt (double);
        arma::vec proj_func (const arma::vec& x);
        inline arma::vec f(const arma::vec& x, const arma::vec& u);
        inline arma::mat dfdx(const arma::vec& x, const arma::vec& u);
        inline arma::mat hx(const arma::vec& x);
        void step(void);
        
        
};

DoubleInt::DoubleInt (double _dt){
   dt = _dt;//step size
}

arma::vec DoubleInt::proj_func (const arma::vec& x){
    return x;
}
inline arma::vec DoubleInt::f(const arma::vec& x, const arma::vec& u){
    arma::vec xdot = {x(1),
                      u(0),
                      x(3),
                      u(1)};;
    return xdot;
}; 

inline arma::mat DoubleInt::dfdx(const arma::vec& x, const arma::vec& u){
    arma::mat A = {
        {0,1,0,0},
        {0,0,0,0},
        {0,0,0,1},
        {0,0,0,0}
    };
    return A;
}; 

inline arma::mat DoubleInt::hx(const arma::vec& x){
    arma::mat H = {
        {0,0},
        {1,0},
        {0,0},
        {0,1}
    };
    return H;
}; 

void DoubleInt::step(){
    Xcurr = RK4_step(this,Xcurr,Ucurr,dt);
    tcurr = tcurr+dt;
};


#endif