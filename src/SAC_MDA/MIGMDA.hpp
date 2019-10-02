#ifndef MIGMDA_HPP
#define MIGMDA_HPP
#include<armadillo>
#include <iostream>
#include"SAC.hpp"

template<class system, class objective>
class migmda{
    sac<system,objective>* sacsys;
    bool replace;
        
    public:
    migmda(sac<system,objective> *_sacsys,bool _replace){sacsys = _sacsys; replace = _replace;};
    
    
    bool filter(const arma::vec& user){
        arma::vec ucurr;bool accept = false;
        double dJdlam_int = 0.;
        arma::mat utemp = arma::zeros(user.n_rows, sacsys->T_index); utemp.col(0) = user;
        arma::mat unom = arma::zeros(user.n_rows,sacsys->T_index);
        arma::mat xsol = sacsys->xforward(utemp);
        double J1 = sacsys->cost->calc_cost(xsol,utemp);//must execute before rhoback for ergodic cost fxns
        arma::mat rhosol = sacsys->rhoback(xsol,utemp);
        for(int i = 0;i<utemp.n_cols;i++) dJdlam_int+=sacsys->dJdlam_t(xsol.col(i),rhosol.col(i),utemp.col(i),unom.col(i));
        //if(sacsys->dJdlam_t(xsol.col(0),rhosol.col(0),utemp.col(0),unom.col(0))>=0.){
        if(dJdlam_int<=0.){
            ucurr = user; accept = true;
            }else if(replace==true) {sacsys->SAC_calc(); ucurr = sacsys->ulist.col(0);
                }else {ucurr = arma::zeros(size(user));}
     return accept;}
};

#endif