#ifndef PLL_HPP
#define PLL_HPP

#include "basics.hpp"

#include <complex>

class PLL{
	public:
		void step(std::complex<double> in,double dt);
		void step(double in_phase,double in_mag,double dt);
		double f=50;
		std::complex<double> out=10;
		double out_phase=0;
		double out_mag=0;
		bool locked=false;
		int lock_counter=0;
	private:
		double phase_err_filter=0;
		double last_phase_err=0;
};

class PLL_harmonic{
	public:
		const static unsigned int N=7; // number of harmonics to control
		PLL_harmonic(double dt,double omega0);
		void step(std::complex<double> vab);
		std::complex<double> vab(double dt) const;
		std::complex<double> vab0(double dt) const;
		double out_mag;
		double out_mag_slow;
		double out_phase;
		double f;
		std::complex<double> vdq[N];
		double theta;
		double omega;
		bool locked;
		bool wrap;
	private:
		const double dt_;
		const double omega0_;
		static const int h[];
		double lock_count;
		PT1<std::complex<double> > pt1[N];
		PI_ctrl<double> omega_ctrl;
};

#endif