#include "grid_control_pr.hpp"

#include <complex>
#include <algorithm>

const double PI=std::acos(-1);
const double MAX_DELTA=0.1;



Grid_Control_PR::Grid_Control_PR(const double dt):i_des_last{0,0},integral{{dt},{dt}},dt_{dt},osf{1},kp_{100},ki_{8000}{
}
Grid_Control_PR::Grid_Control_PR(const double dt, const unsigned int oversample_factor):i_des_last{0,0},integral{{dt/oversample_factor},{dt/oversample_factor}},dt_{dt},osf{oversample_factor},kp_{100},ki_{8000}{

}

std::complex<double> Grid_Control_PR::step_ctrl(const std::complex<double> i_act_ab,const std::complex<double> i_des_dq, const double grid_phase, const double grid_f){
	auto i_des_delta=i_des_dq-i_des_last;
	i_des_delta.real(std::clamp(i_des_delta.real(),-MAX_DELTA,MAX_DELTA));
	i_des_delta.imag(std::clamp(i_des_delta.imag(),-MAX_DELTA,MAX_DELTA));
	i_des_last+=i_des_delta;
	auto i_des_ab=i_des_last*std::polar(1.0,grid_phase);
	auto i_grid_err=i_des_ab-i_act_ab;
	for (unsigned int n=0;n<this->osf;n++){
		this->integral[0].step(i_grid_err*this->ki_-this->integral[1].value);
		this->integral[1].step((4*PI*PI*grid_f*grid_f)*this->integral[0].value);
	}
	auto v_grid_ctrl_ab=this->kp_*i_grid_err+this->integral[0].value;
	return v_grid_ctrl_ab;
}

void Grid_Control_PR::reset(){
	this->integral[0].reset();
	this->integral[1].reset();
	this->i_des_last=0;
}

void Grid_Control_PR::set_params(double kp, double ki){
	this->kp_=kp;
	this->ki_=ki;
	this->reset();
}