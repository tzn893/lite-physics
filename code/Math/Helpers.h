//
#pragma once


#include <math.h>


template<typename T>
T Max(const T& x, const T& y)
{
	return x > y ? x : y;
}

template<typename T>
T Min(const T& x, const T& y)
{
	return x < y ? x : y;
}

template<typename T>
T Clamp(const T& x, const T& lower, const T& upper)
{
	return Max(Min(x, upper), lower);
}