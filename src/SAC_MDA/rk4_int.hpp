#ifndef RK4_INT_HPP
#define RK4_INT_HPP
#include<armadillo>
//#include"cartpend.hpp"

template <class T, class input> arma::vec RK4_step(T *sys, const arma::vec& x, input u, double dt){
  arma::vec k1, k2, k3, k4;
  k1 = sys->f(x,u)*dt; 
  k2 = sys->f(x+k1/2, u)*dt; 
  k3 = sys->f(x+k2/2, u)*dt;
  k4 = sys->f(x+k3, u)*dt;
  return x + (k1/6)+(k2/3)+(k3/3)+(k4/6);
    
};

#endif