#ifndef ERRORCOST_HPP
#define ERRORCOST_HPP
#include<armadillo>

template <class system>
class errorcost {
        system* sys;
    public:
        arma::mat Q;
        arma::mat R;
        std::function<arma::vec(double)> xd;
        errorcost(arma::mat _Q, arma::mat _R,std::function<arma::vec(double)> _xd, system *_sys){
            Q=_Q; R=_R; sys=_sys; xd = _xd;// initialize with Q, R, sys, xd
            
            };
        inline double l (const arma::vec& x,const arma::vec& u,double ti){
            arma::vec xproj = sys->proj_func(x);
            return arma::as_scalar(((xproj.t()-xd(ti).t())*Q*(xproj-xd(ti))+u.t()*R*u)/2);
        }
        inline arma::vec dldx (const arma::vec& x,const arma::vec& u,double ti){
            arma::vec xproj = sys->proj_func(x);
            return Q*(xproj-xd(ti));
        }
        double calc_cost(const arma::mat& x,const arma::mat& u){
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