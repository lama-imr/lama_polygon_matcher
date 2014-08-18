#ifndef _PM_FOURIER_ANGLE_SHIFT_H_
#define _PM_FOURIER_ANGLE_SHIFT_H_

#include <vector>

int getAngleShiftFFT(const std::vector<double> &fft1, const std::vector<double> &ftd2, double &maxVal);
int getAngleShiftFFT(const std::vector<double> &fft1, const std::vector<double> &ftd2);
std::vector<double> fft2(const std::vector<double> &data);

#endif // _PM_FOURIER_ANGLE_SHIFT_H_

