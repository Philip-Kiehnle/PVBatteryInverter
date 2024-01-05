#ifndef GRID_CONTROL_HPP
#define GRID_CONTROL_HPP

#include "pll.hpp"
#include <complex>

template<typename T>
struct Integrator{
	Integrator(double dt):dt_{dt},value{}{}
	T step(T u){
		value+=(last_u+u)/2.0*dt_;
		//value+=u*dt_;
		last_u=u;
		return value;
	}
	void reset(){
		value=T{};
	}
	double dt_;
	T value;
	T last_u;
};
template<typename T>
struct Integrator_euler{
	Integrator_euler(double dt):dt_{dt},value{}{}
	T step(T u){
		value+=u*dt_;
		return value;
	}
	void reset(){
		value=T{};
	}
	double dt_;
	T value;
};
class Grid_Control_PR{
	public:
		Grid_Control_PR(const double dt);
		Grid_Control_PR(const double dt, const unsigned int oversample_factor);
		std::complex<double> step_ctrl(const std::complex<double> i_act_ab, const std::complex<double> i_des_dq, const double grid_phase, const double grid_f);
		void reset();
		void set_params(double kp, double ki);
		std::complex<double> i_des_last;
		Integrator_euler<std::complex<double>> integral[2];
	private:
		const double dt_;
		const unsigned int osf;
		double kp_;
		double ki_;
};
#endif