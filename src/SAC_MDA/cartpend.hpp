//an example class definition for a particular system
//To define a class that can be used in the template SAC and cost functions,
//it must include variables Xcurr, Ucurr. It must also have functions for the
//system dynamics and relevant derivatives named f, dfdx, and hx respectively.
//Finally, Xcurr should be managed only by this class through the use of a 
//function called step that evolves the system forward in time by dt seconds.

#ifndef CARTPEND_HPP
#define CARTPEND_HPP
#include<armadillo>
#include"rk4_int.hpp"
//define any relevant constants such as pi here
const double PI = 3.1415926535987;

class CartPend {
    double m, B, g, h;//system parameters: mass, damping, gravity, pendulum legth
    public:
        double dt;//time between system updates
        double tcurr=0.0;//initilaize system time
        arma::vec Xcurr, Ucurr;//current system state arranged as q1,dq1,q2,dq2
        //class member function prototypes
        CartPend (double, double,double,double,double);
        arma::vec proj_func (const arma::vec& x);
        inline arma::vec f(const arma::vec& x, const arma::vec& u);
        inline arma::mat dfdx(const arma::vec& x, const arma::vec& u);
        inline arma::vec hx(const arma::vec& x);
        void step(void);
        
        
};

CartPend::CartPend (double _m, double _B, double _g, double _h, double _dt){
    m = _m; B = -1*_B; g = _g; h=_h;//system parameters
    dt = _dt;//step size
}

arma::vec CartPend::proj_func (const arma::vec& x){//function for angle wrapping between -pi to pi
    arma::vec xwrap=x;
    xwrap(0) = fmod(x(0)+PI, 2*PI);
    if (xwrap(0) < 0.0) xwrap(0) = xwrap(0) + 2*PI;
    xwrap(0) = xwrap(0) - PI;
    return xwrap;
}
inline arma::vec CartPend::f(const arma::vec& x, const arma::vec& u){//system dynamics, xdot = f(x)
    arma::vec xdot = {x(1),
                      g/h*sin(x(0)) + B*x(1)/(m*h*h)-u(0)*cos(x(0))/h,
                      x(3),
                      u(0)};;
    return xdot;
}; 

inline arma::mat CartPend::dfdx(const arma::vec& x, const arma::vec& u){//derivative of f wrt x
    arma::mat A = {
        {0,1,0,0},
        {g/h*cos(x(0))+ u(0)*sin(x(0))/h, B/(m*h*h),0,0},
        {0,0,0,1},
        {0,0,0,0}
    };
    return A;
}; 

inline arma::vec CartPend::hx(const arma::vec& x){//derivative of f wrt the control
    arma::vec H = {
        {0},
        {-cos(x(0))/h},
        {0},
        {1}
    };
    return H;
}; 

void CartPend::step(){//step the system forward by dt seconds
    Xcurr = RK4_step(this,Xcurr,Ucurr,dt);
    tcurr = tcurr+dt;
};


#endif