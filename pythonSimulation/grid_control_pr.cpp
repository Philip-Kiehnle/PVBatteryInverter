#include "grid_control_pr.hpp"

#include <complex>
#include <algorithm>

const double PI=std::acos(-1);
const double MAX_DELTA=0.1;

Grid_Control_PR::Grid_Control_PR(const double dt):i_des_last{0,0},dt_{dt},kp_{100,100},ki_{8000,8000},integral1{0.0,0.0},integral2{0.0,0.0}{

}

std::complex<double> Grid_Control_PR::step_ctrl(const std::complex<double> i_act_ab,const std::complex<double> i_des_dq, const double grid_phase, const double grid_f){
	auto i_des_delta=i_des_dq-i_des_last;
	i_des_delta.real(std::clamp(i_des_delta.real(),-MAX_DELTA,MAX_DELTA));
	i_des_delta.imag(std::clamp(i_des_delta.imag(),-MAX_DELTA,MAX_DELTA));
	i_des_last+=i_des_delta;
	auto i_des_ab=i_des_last*std::polar(1.0,grid_phase);
	auto i_grid_err=i_des_ab-i_act_ab;
	this->integral1[0]+=(i_grid_err.real()*this->ki_[0]-this->integral2[0])*this->dt_;
	this->integral1[1]+=(i_grid_err.imag()*this->ki_[1]-this->integral2[1])*this->dt_;
	this->integral2[0]+=(4*PI*PI*grid_f*grid_f)*this->integral1[0]*this->dt_;
	this->integral2[1]+=(4*PI*PI*grid_f*grid_f)*this->integral1[1]*this->dt_;
	auto v_grid_ctrl_ab=std::complex<double>{this->kp_[0]*i_grid_err.real()+this->integral1[0],this->kp_[1]*i_grid_err.imag()+this->integral1[1]};
	return v_grid_ctrl_ab;
}

void Grid_Control_PR::reset(){
	this->integral1[0]=0;
	this->integral1[1]=0;
	this->integral2[0]=0;
	this->integral2[1]=0;
	this->i_des_last=0;
}

void Grid_Control_PR::set_params(double kp, double ki){
	this->kp_[0]=kp;
	this->kp_[1]=kp;
	this->ki_[0]=ki;
	this->ki_[1]=ki;
	this->reset();
}

extern "C" 
{
    // include below each method you want to make visible outside
//    __declspec(dllexport) Test* init(int k) {return new Test(k);}
    Grid_Control_PR* init(const double dt) {return new Grid_Control_PR(dt);}
    void set_params(Grid_Control_PR *self, double kp, double ki) {self->set_params(kp, ki);}
//    std::complex<double>  step_ctrl(Test *self) {return self->getInt();}
    
    // Note: the '__declspec(dllexport)' is only necessary in Windows
}
