#ifndef _PM_FOURIER_FOURIER_H_
#define _PM_FOURIER_FOURIER_H_

#include <vector>
#include <pm_fourier/spoint.h>

double getSimpleSimilarityFft(const std::vector<double> &fft1, const std::vector<double> &fft2, const int size);

double getSimilarityNccFft(const std::vector<double> &fft1, const std::vector<double> &fft2, const int size);

double getSimilarityFourier(const std::vector<SPoint> &polygon1, const std::vector<SPoint> &polygon2, const int fftSize);

#endif // _PM_FOURIER_FOURIER_H_


