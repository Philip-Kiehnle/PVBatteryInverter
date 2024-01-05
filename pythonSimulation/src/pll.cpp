#include "pll.hpp"

#include <algorithm>

using namespace std::complex_literals;

const double PI=std::acos(-1);
const double SQRT2=std::sqrt(2);
const double LOCK_THRESHOLD=0.001;
const double LOCK_DELAY=500;

double limit_angle(double angle,bool &has_wrapped){
	if(angle>PI){
		has_wrapped=true;
		return angle-2*PI;
	}
	if(angle<-PI){
		has_wrapped=true;
		return angle+2*PI;
	}
	has_wrapped=false;
	return angle;
}

double limit_angle(double angle){
	bool dummy;
	return limit_angle(angle,dummy);
}

void PLL::step(std::complex<double> in,double dt){
	this->out*=std::exp(1.0i*2.0*PI*this->f*dt);
	const double in_norm=std::norm(in);
	if (in_norm<10){
		return; // Do not try to track nonexisting grid voltage
	}
	const double phase_err=std::arg(in*std::conj(out));
	this->phase_err_filter=(15*this->phase_err_filter+phase_err)/16;
	const double ampl_quot=in_norm/std::norm(out);
	this->f+=this->phase_err_filter*0.1+(this->phase_err_filter-last_phase_err)*1;
	this->last_phase_err=phase_err_filter;
	this->out*=(ampl_quot-1)*0.01+1;
	this->out_mag=std::abs(this->out);
	this->out_phase=std::arg(this->out);
	// Maybe have out be actually in phase to in without delay:
	// real_out=out*std::exp(1i*2.0*PI*this->f*dt);
}

void PLL::step(double in_phase,double in_mag,double dt){
	this->out_phase=limit_angle(this->out_phase+2.0*PI*this->f*dt);
	if (in_mag<10){
		return;
	}
	double phase_err=limit_angle(in_phase-out_phase);	
	this->phase_err_filter=(15*this->phase_err_filter+phase_err)/16;
	if (std::abs(this->phase_err_filter)<LOCK_THRESHOLD){
		if (this->lock_counter<100000){
			this->lock_counter+=1;
		}
	}else{
		this->lock_counter=0;
	}
	this->locked=this->lock_counter>=LOCK_DELAY;
	this->f+=this->phase_err_filter*0.1+(this->phase_err_filter-last_phase_err)*5;
	this->last_phase_err=phase_err_filter;
	this->out_mag+=0.01*(in_mag-out_mag);
}

double pt1_factor(double dt){
	return exp(-50*2*PI/SQRT2*dt);
}

const int PLL_harmonic::h[]={1,-1,5,-5,7,-11,13};

PLL_harmonic::PLL_harmonic(double dt,double omega0):theta{0},dt_{dt},omega0_{omega0},lock_count{0},pt1{pt1_factor(dt),pt1_factor(dt),pt1_factor(dt),pt1_factor(dt),pt1_factor(dt),pt1_factor(dt),pt1_factor(dt)},omega_ctrl(184, 16928*dt){

}

void PLL_harmonic::step(std::complex<double> vab){
	std::complex<double> harmonics[N];
	std::complex<double> vab_n[N];
	std::complex<double> vab_s=0;
	for(uint32_t i=0;i<N;i++){
		harmonics[i]=std::polar(1.0,-this->theta*this->h[i]);
		vab_n[i]=std::conj(harmonics[i])*this->vdq[i];
		vab_s+=vab_n[i];
	}
	for(uint32_t i=0;i<N;i++){
		this->vdq[i]=this->pt1[i].step((vab-vab_s+vab_n[i])*harmonics[i]);
	}
	const double d=this->vdq[0].real();
	const double q=this->vdq[0].imag();
	const auto norm_factor=std::max(std::abs(d),10.0);
	this->omega=this->omega_ctrl.step(q/norm_factor)+this->omega0_;
	theta=limit_angle(this->theta+this->omega*this->dt_,this->wrap);
	double lock_delta=-this->dt_;
	if (abs(d)>200*abs(q) && std::abs(d)>50 && abs(this->omega)>40*2*PI){
		lock_delta=this->dt_;
	}
	this->lock_count=std::clamp(this->lock_count+lock_delta,0.0,1.3);
	this->locked=this->lock_count>=1.0;

	this->out_mag=d;
	if(this->wrap){
		this->out_mag_slow=this->out_mag;
	}
	this->out_phase=this->theta;
	this->f=this->omega/2/PI;
}

std::complex<double> PLL_harmonic::vab(double dt) const{
	const double angle=limit_angle(this->theta+dt*this->omega);
	std::complex<double> res=0;
	for(uint32_t i=0;i<N;i++){
		res+=std::polar(1.0,angle*this->h[i])*this->vdq[i];
	}
	return res;
}
std::complex<double> PLL_harmonic::vab0(double dt) const{
	const double angle=limit_angle(this->theta+dt*this->omega);
	return std::polar(1.0,angle*this->h[0])*this->vdq[0];
}