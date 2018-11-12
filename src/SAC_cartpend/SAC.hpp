#ifndef SAC_HPP
#define SAC_HPP
#include<armadillo>
#include <iostream>

struct xupair{
  arma::vec x;
  arma::vec u;
  double t;
};



template <class system, class objective>
class sac {
  system* sys; //from sys use sys->f, sys->proj_func, sys->dfdx, sys->hx,sys->dt
  objective* cost; //from cost use cost->dldx, cost->calc_cost
  std::function<arma::vec(double)> unom;
  //algorithm parameters
  double gamma = -5; double delt_init = 0.5; double beta = 0.55;
  double tcalc; int kmax = 6; double T;  
  arma::vec umax;
  
  //saturation function for the control        
  arma::vec saturation(const arma::vec& u){
    arma::vec usat; usat.zeros(u.n_rows);
    for (int i = 0; i<u.n_rows; i++){
      if(u(i) > umax(i)) usat(i) = umax(i);
        else if(u(i) < -umax(i)){ usat(i) = -umax(i);}
          else usat(i) = u(i);};
    return usat;}
  //incorporating ustar into u matrix and restricting the application interval
  inline arma::mat uInc(arma::vec &ut, double tau[]){
    arma::mat usol = ulist;
    for(int i = 0; i<T_index;i++){
      if(sys->tcurr+(double)i*sys->dt > tau[0] && sys->tcurr+(double)i*sys->dt < tau[1] ){
        usol.col(i) = ut;} }
  return usol;}
    
  public:
  bool iterative=false;
  int T_index;
  arma::mat ulist;
    
  sac(system *_sys, objective *_cost, double _tcalc,double _T,const arma::vec& _umax,std::function<arma::vec(double)> _unom){
    sys = _sys; cost=_cost; tcalc=_tcalc; T=_T;umax = _umax; unom = _unom;
    T_index = T/sys->dt;
    ulist = arma::zeros(1,T_index); 
    for(int i = 0;i<ulist.n_cols-1;i++) ulist.col(i) = unom(sys->tcurr + (double)i*sys->dt);
  };
    
  void SAC_calc();//main function for calculating the current SAC action
  void unom_shift ();  
  double dJdlam_t(const arma::vec& xt,const arma::vec& rhot,const arma::vec& u2t,const arma::vec& u1t);//Mode insertion gradient at time t
  arma::mat xforward(const arma::mat& u);//forward simulation of x
  arma::mat rhoback(const arma::mat& xsol,const arma::mat& u); //backward simulation of the adjoint
  inline arma::vec f(const arma::vec& rho, xupair pair){
    return -cost->dldx(pair.x,pair.u,pair.t) - sys->dfdx(pair.x,pair.u).t()*rho;}//f for rho backwards sim
};

//main function for calculating a single SAC control vector
template <class system, class objective>
void sac<system,objective>::SAC_calc(){
  ulist.col(0) = sys->Ucurr;
  arma::vec ustar;
  ustar = arma::zeros<arma::vec>(size(sys->Ucurr));
  double tau[2] = {0,0};
  arma::uword tautemp;
  arma::mat xsol,rhosol;
  arma::mat utemp = ulist;
  arma::mat usched = arma::zeros<arma::mat>(1,T_index);
  arma::mat Jtau = arma::zeros<arma::mat>(1,T_index);
  double J1init,J1new,dJmin,alphad,lambda;
          
  xsol = xforward(ulist);
  J1init = cost->calc_cost(xsol,ulist);//must execute before rhoback for ergodic cost fxns
  rhosol = rhoback(xsol, ulist);
  dJmin = 0.;//gamma*J1init
  alphad = gamma*J1init;
  arma::vec Lam;
  double dJdlam;
  for(int i = 0; i<T_index;i++){
    Lam = sys->hx(xsol.col(i)).t()*rhosol.col(i)*rhosol.col(i).t()*sys->hx(xsol.col(i));
    usched.col(i) = (Lam +cost->R).i()*(Lam*ulist.col(i) + sys->hx(xsol.col(i)).t()*rhosol.col(i)*alphad);
    dJdlam = dJdlam_t(xsol.col(i),rhosol.col(i),usched.col(i),ulist.col(i));
    Jtau.col(i) =arma::norm(usched.col(i))+dJdlam+pow((double)i*sys->dt,beta);
    }
  tautemp = Jtau.index_min(); //cout<<Jtau;
  ustar=saturation(usched.col(tautemp));//ustar.u=usched.col(0);
  int k = 0; J1new = 1000*J1init;
  while(J1new-J1init>dJmin && k<= kmax){
    lambda = delt_init*pow(beta,k);
    tau[0] = sys->tcurr + (double)tautemp*sys->dt -(lambda/2);
    tau[1] = sys->tcurr + (double)tautemp*sys->dt+(lambda/2);
    utemp = uInc(ustar,tau);
    xsol = xforward(utemp);
    J1new = cost->calc_cost(xsol,utemp);
    k++;}
  ulist = utemp;
return;}
    
//forward simulation of x
template <class system, class objective>
arma::mat sac<system,objective>::xforward(const arma::mat& u){
  arma::mat xsol = arma::zeros<arma::mat>(sys->Xcurr.n_rows,T_index);
  arma::vec x0 = sys->Xcurr;
  for(int i = 0; i<T_index;i++){
    xsol.col(i)=x0;
    x0 = RK4_step<system,const arma::vec&>(sys,x0,u.col(i),sys->dt);
  }    
return xsol;}

//backward simulaiton of the adjoint
template <class system, class objective>
arma::mat sac<system,objective>::rhoback(const arma::mat& xsol,const arma::mat& u){
  arma::mat rhosol = arma::zeros<arma::mat>(sys->Xcurr.n_rows,T_index);
  arma::vec rho0 = sys->Xcurr;
  xupair current;
  rho0.zeros();
  for(int i = T_index-1; i>=0;i--){
    rhosol.col(i)=rho0;
    current.x =xsol.col(i);
    current.u = u.col(i);
    current.t = sys->tcurr+(double)i*sys->dt;
    rho0 = RK4_step<sac,xupair>(this,rho0,current,-1.0*sys->dt);
  } 
return rhosol;}

template <class system, class objective>
double sac<system,objective>::dJdlam_t(const arma::vec& xt, const arma::vec& rhot, const arma::vec& u2t, 
                                       const arma::vec& u1t){
    return arma::as_scalar(rhot.t()*(sys->f(xt,u2t)-sys->f(xt,u1t)));}

//shift the unom values with time
template <class system, class objective>
void sac<system,objective>::unom_shift(){
  for(int i = 0;i<ulist.n_cols-1;i++) ulist.col(i) = ulist.col(i+1);
    ulist.col(ulist.n_rows) = unom(sys->tcurr +T+sys->dt);
};

#endif