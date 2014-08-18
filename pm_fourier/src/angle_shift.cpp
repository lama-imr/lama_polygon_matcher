
#include <iostream>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_fft_real.h>
#include <gsl/gsl_fft_halfcomplex.h>
#include <gsl/gsl_fft_complex.h>

#include "pm_fourier/angle_shift.h"

using std::vector;

vector<double> fft2(const vector<double> &data);

int getAngleShiftFFT(const vector<double> &fft1, const vector<double> &ftd2, double &maxVal) 
{
	gsl_complex_packed_array cpl = new double[fft1.size()];


	if (fft1.size() > ftd2.size())
  {
		std::cerr << __FUNCTION__ << "fft1 and fft(scan) have bad lengths!: fft1.size=" << fft1.size() << ", fft2.size=" << ftd2.size() << "!\n";
		return 0;
	}

	for (size_t i = 0; i < fft1.size() / 2; i++)
  {
		cpl[2 * i] = fft1[2 * i] * ftd2[2 * i] + fft1[2 * i + 1] * ftd2[2 * i + 1];
		cpl[2*i+1] = fft1[2 * i] * ftd2[2 * i + 1] - fft1[2 * i + 1] * ftd2[2 * i];
	}


	gsl_fft_complex_workspace *work = 
		gsl_fft_complex_workspace_alloc(fft1.size() / 2);

	gsl_fft_complex_wavetable *wave = 
		gsl_fft_complex_wavetable_alloc(fft1.size() / 2);
	gsl_fft_complex_inverse(cpl, 1, fft1.size() / 2, wave, work);

	gsl_fft_complex_wavetable_free(wave);
	gsl_fft_complex_workspace_free(work);

	double d;
	double maxd = -1;
	int maxdi = -1;
	for(size_t i = 0; i < fft1.size() / 2; i++)
  {
		d = cpl[2 * i] * cpl[2 * i] + cpl[2 * i + 1] * cpl[2 * i + 1];
		if (d > maxd || i == 0)
    {
			maxdi = i;
			maxd = d;
		}
	}
	delete [] cpl;
	maxVal = maxd;

	return maxdi;
}

int getAngleShiftFFT(const vector<double> &fft1, const vector<double> &ftd2) 
{
	double maxVal;
	int res = getAngleShiftFFT(fft1, ftd2, maxVal);
	return res;	
}


/** function was copied from CLaloc.cc */
vector<double> fft2(const vector<double> &data)
{
	double *d = new double[data.size()];

	gsl_fft_real_wavetable *real;
	gsl_fft_real_workspace *work;

	for(size_t i = 0; i < data.size(); i++)
  {
		d[i] = data[i];
	}

	work = gsl_fft_real_workspace_alloc(data.size());
	real = gsl_fft_real_wavetable_alloc(data.size());

	gsl_fft_real_transform(d, 1, data.size(), real, work);
	gsl_fft_real_wavetable_free(real);

	gsl_complex_packed_array cpl = new double[data.size() * 2];

	gsl_fft_halfcomplex_unpack(d, cpl, 1, data.size());

	// jen pro kresleni
	
	gsl_fft_halfcomplex_wavetable *hc = 
		gsl_fft_halfcomplex_wavetable_alloc(data.size());
	for(size_t i = 100; i < data.size(); i++)
  {
		d[i] = 0;
	}
	gsl_fft_halfcomplex_inverse(d, 1, data.size(), hc, work);
	gsl_fft_halfcomplex_wavetable_free(hc);

	
  //	std::ofstream ofs("scanifft.dat");
  //	ofs << "#ifft from fft2\n";
  //	for(int i=0;i<data.size();i++)
  //		ofs << d[i] << "\n";
  //	ofs.close();

	vector<double> tmpd;
	for (size_t i = 0; i < data.size(); i++)
  {
		tmpd.push_back(d[i]);
	}
  // saveScan("fft1.dat",tmpd,maxScanPhi);
	

	vector<double> result;
	result.reserve(data.size());
	
	for(size_t i = 0; i < data.size() * 2; i++)
  {
		result.push_back(cpl[i]);
	}

	delete [] cpl;
	delete [] d;

	gsl_fft_real_workspace_free(work);
  // gsl_fft_real_wavetable_free(real);
	return result;
}


