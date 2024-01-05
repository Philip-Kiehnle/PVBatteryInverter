#ifndef BASICS_HPP
#define BASICS_HPP

#include <algorithm>

template<typename T>
class PT1{
	public:
		PT1(double factor):fac{factor}{}
		T step(T in){
			state=(state-in)*fac+in;
			return state;
		}
		operator T(){
			return state;
		}
	private:
		const double fac;
		T state;
};

template<typename T>
class PI_ctrl{
	public:
		PI_ctrl(double kp, double ki):value{0},kp_{kp},ki_{ki},state{0}{}
		T step(T in){
			auto res=kp_*in+ki_*state;
			state+=in;
			value=res;
			return res;
		}
		T value;
	private:
		const double kp_;
		const double ki_;
		T state;
};

template<typename T>
class PI_ctrl_limited{
	public:
		PI_ctrl_limited(double kp, double ki, T limit):value{0},kp_{kp},ki_{ki},limit_{limit},state{0}{}
		T step(T in,double dt){
			auto res=std::clamp(kp_*in+ki_*state,-limit_,limit_);
			state+=(res-ki_*state)/kp_*dt;
			value=res;
			return res;
		}
		T value;
	private:
		const double kp_;
		const double ki_;
		const T limit_;
		T state;
};
#endif