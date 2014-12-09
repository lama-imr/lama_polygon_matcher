#ifndef _PM_FOURIER_FOURIER_H_
#define _PM_FOURIER_FOURIER_H_

#include <vector>

#include <lama_common/point.h>

using std::vector;
using lama_common::Point2;

double getSimpleDissimilarityFft(const vector<double> &fft1, const vector<double> &fft2, const int size);

double getDissimilarityNccFft(const vector<double> &fft1, const vector<double> &fft2, const int size);

double getDissimilarityFourier(const vector<Point2> &polygon1, const vector<Point2> &polygon2, const int fftSize);

#endif // _PM_FOURIER_FOURIER_H_

