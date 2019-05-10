//An example class for defining an objective function for SAC
//This example defines a quadratic error-based cost function
//All cost classes must include member functions for the derivative
//of the incremental cost dldx, and a function to evaluate the total
//cost over a trajectory and control pair named calc_cost.
//Member functions defining the desired trajectory xd and the 
//incremental cost l are recommended but not required for integration
//into the template function for SAC.

#ifndef ERRORCOST_HPP
#define ERRORCOST_HPP
#include<armadillo>

template <class system>
class errorcost {
    system* sys;
  public:
    arma::mat Q;//The weights on the state x
    arma::mat R;//the weights on the control u
    std::function<arma::vec(double)> xd;//a function defining the desired trajectory
    //the class constructor
    errorcost(arma::mat _Q, arma::mat _R,std::function<arma::vec(double)> _xd, system *_sys){
      Q=_Q; R=_R; sys=_sys; xd = _xd;// initialize with Q, R, sys, xd
      };
    inline double l (const arma::vec& x,const arma::vec& u,double ti){//incremental cost
      arma::vec xproj = sys->proj_func(x);
      return arma::as_scalar(((xproj.t()-xd(ti).t())*Q*(xproj-xd(ti))+u.t()*R*u)/2);
      }
    inline arma::vec dldx (const arma::vec& x,const arma::vec& u,double ti){//derivative of the l wrt x
      arma::vec xproj = sys->proj_func(x);
      return Q*(xproj-xd(ti));
      }
    double calc_cost(const arma::mat& x,const arma::mat& u){//a function for evaluating the cost of a particular trajectory/control pair
      arma::vec xproj;
      double J1 = 0.0;
      for (int i = 0; i<x.n_cols; i++){
        xproj = sys->proj_func(x.col(i));
        J1+=l(xproj,u.col(i),sys->tcurr+(double)i*sys->dt);
        }
      return J1;
      }
    
};

#endif