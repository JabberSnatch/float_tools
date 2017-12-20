/*
Copyright (c) 2017 Samuel Bourasseau

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#pragma once
#ifndef __FLOAT_TOOLS_H_INCLUDED__
#define __FLOAT_TOOLS_H_INCLUDED__


/*
*	float_tools is a lightweight library that provides routines to split IEEE754 floating point
*	values into their binary representations.
*	It supports floats and double out of the box but can be extended to handle any IEEE754
*	compliant type by specialising FloatConverter<T> in a similar fashion to float and double.
*	
*	Macros are provided to tweak a little bit the behaviour of the library. #define them before
*	including the file.
*	FLOAT_TOOLS_DEBUG_MODE
*		Enables pre-conditions checking through assertions. It checks for instance that input
*		bitfields are constrained to their expected bounds.
*	FLOAT_TOOLS_SILENT_MODE
*		Enables silent invalid input correction, this sets the library to return coherent results
*		even for invalid inputs. See the code to find out the exact nature of the correction.
*/


#include <limits>
#include <cstdint>


#ifdef FLOAT_TOOLS_DEBUG_MODE
#include <cassert>
#define ft_assert(condition) assert(condition)
#else
#define ft_assert(condition) {}
#endif


namespace float_tools
{

template <typename T> union FloatConverter {};
template <> union FloatConverter<float>
{
	using utype = uint32_t;
	static constexpr utype kExplicitDigitsCount =
		static_cast<utype>(std::numeric_limits<float>::digits) - 1ull;
	float f; utype u;
};
template <> union FloatConverter<double>
{
	using utype = uint64_t;
	static constexpr utype kExplicitDigitsCount =
		static_cast<utype>(std::numeric_limits<double>::digits) - 1ull;
	double f; utype u;
};

template <typename T> using BitsType = typename FloatConverter<T>::utype;
template <typename T> using UintType = typename FloatConverter<T>::utype;


template <typename T> T				makeFloat(bool _is_negative,
											  int _exponent,
											  T _significand);
template <typename T> T				makeFloat(BitsType<T> _sign_bit,
											  BitsType<T> _exponent_bits,
											  BitsType<T> _significand_bits);

template <typename T> BitsType<T>	getSignBit(T _value);
template <typename T> bool			getSign(T _value);
template <typename T> BitsType<T>	makeSignBit(bool _is_negative);

template <typename T> BitsType<T>	getExponentBits(T _value);
template <typename T> int			getExponent(T _value);
template <typename T> BitsType<T>	makeExponentBits(int _exponent);
template <typename T> UintType<T>	encodeExponent(int _exponent);
template <typename T> BitsType<T>	makeExponentBits(UintType<T> _encoded_exponent);
template <typename T> UintType<T>	getEncodedExponent(BitsType<T> _exponent_bits);
template <typename T> int			decodeExponent(UintType<T> _encoded_exponent);

template <typename T> BitsType<T>	getSignificandBits(T _value);
template <typename T> T				getSignificand(T _value);
template <typename T> BitsType<T>	makeSignificandBits(T _significand);
template <typename T> T				makeSignificand(BitsType<T> _significand_bits);


template <typename T>
constexpr UintType<T> kExponentShift = FloatConverter<T>::kExplicitDigitsCount;
template <typename T>
constexpr UintType<T> kExponentSize = (sizeof(T) * 8u - FloatConverter<T>::kExplicitDigitsCount - 1u);
template <typename T>
constexpr UintType<T> kExponentBias = (1ull << (kExponentSize<T> - 1u)) - 1ull;
template <typename T>
constexpr int kMinExponent = -static_cast<int>(kExponentBias<T>);
template <typename T>
constexpr int kMaxExponent = static_cast<int>(kExponentBias<T>) + 1;

template <typename T>
constexpr BitsType<T> kSignBitMask = (1ull << (kExponentShift<T> + kExponentSize<T>));
template <typename T>
constexpr BitsType<T> kSignificandMask = (1ull << FloatConverter<T>::kExplicitDigitsCount) - 1ull;
template <typename T>
constexpr BitsType<T> kExponentMask = ~(kSignBitMask<T> | kSignificandMask<T>);


static_assert(sizeof(float) == sizeof(BitsType<float>), "");
static_assert(sizeof(double) == sizeof(BitsType<double>), "");
static_assert(FloatConverter<float>::kExplicitDigitsCount ==
			  static_cast<UintType<float>>(std::numeric_limits<float>::digits) - 1ull, "");
static_assert(FloatConverter<double>::kExplicitDigitsCount ==
			  static_cast<UintType<double>>(std::numeric_limits<double>::digits) - 1ull, "");
static_assert(kExponentShift<float> == 23u, "");
static_assert(kExponentShift<double> == 52u, "");
static_assert(kExponentShift<float> + kExponentSize<float> == 31u, "");
static_assert(kExponentShift<double> + kExponentSize<double> == 63u, "");
static_assert(kExponentBias<float> == 127u, "");
static_assert(kExponentBias<double> == 1023u, "");
static_assert(kMinExponent<float> == std::numeric_limits<float>::min_exponent - 2, "");
static_assert(kMinExponent<double> == std::numeric_limits<double>::min_exponent - 2, "");
static_assert(kMaxExponent<float> == std::numeric_limits<float>::max_exponent, "");
static_assert(kMaxExponent<double> == std::numeric_limits<double>::max_exponent, "");


template <typename T>
T makeFloat(bool _is_negative, int _exponent, T _significand)
{
	return makeFloat<T>(makeSignBit<T>(_is_negative), makeExponentBits<T>(_exponent), makeSignificandBits<T>(_significand));
}

template <typename T>
T makeFloat(BitsType<T> _sign_bit, BitsType<T> _exponent_bits, BitsType<T> _significand_bits)
{
	ft_assert((_sign_bit & ~kSignBitMask<T>) == 0ull);
	ft_assert((_exponent_bits & ~kExponentMask<T>) == 0ull);
	ft_assert((_significand_bits & ~kSignificandMask<T>) == 0ull);
#ifdef FLOAT_TOOLS_SILENT_MODE
	_sign_bit &= kSignBitMask<T>;
	_exponent_bits &= kExponentMask<T>;
	_significand_bits &= kSignificandMask<T>;
#endif
	FloatConverter<T> conv;
	conv.u = _sign_bit | _exponent_bits | _significand_bits;
	return conv.f;
}

template <typename T>
BitsType<T> getSignBit(T _value)
{
	FloatConverter<T> conv;
	conv.f = _value;
	return kSignBitMask<T> & conv.u;
}

template <typename T>
bool getSign(T _value)
{
	return getSignBit<T>(_value);
}

template <typename T>
BitsType<T> makeSignBit(bool _is_negative)
{
	return (_is_negative) ? kSignBitMask<T> : 0ull;
}

template <typename T>
BitsType<T> getExponentBits(T _value)
{
	FloatConverter<T> conv;
	conv.f = _value;
	return kExponentMask<T> & conv.u;
}

template <typename T>
int getExponent(T _value)
{
	return decodeExponent<T>(getEncodedExponent<T>(getExponentBits<T>(_value)));
}

template <typename T>
BitsType<T> makeExponentBits(int _exponent)
{
	return makeExponentBits<T>(encodeExponent<T>(_exponent));
}

template <typename T>
UintType<T> encodeExponent(int _exponent)
{
	ft_assert(_exponent >= kMinExponent<T>);
	ft_assert(_exponent <= kMaxExponent<T>);
#ifdef FLOAT_TOOLS_SILENT_MODE
	_exponent = std::max(kMinExponent<T>, _exponent);
	_exponent = std::min(kMaxExponent<T>, _exponent);
#endif
	ft_assert(_exponent + kExponentBias<T> >= 0);
	return static_cast<UintType<T>>(_exponent + kExponentBias<T>);
}

template <typename T>
BitsType<T> makeExponentBits(UintType<T> _encoded_exponent)
{
	constexpr UintType<T> kLowExponentMask = ((1ull << kExponentSize<T>) - 1ull);
	ft_assert((_encoded_exponent & ~kLowExponentMask) == 0ull);
#ifdef FLOAT_TOOLS_SILENT_MODE
	_encoded_exponent = _encoded_exponent & kLowExponentMask;
#endif
	return static_cast<BitsType<T>>(_encoded_exponent) << kExponentShift<T>;
}

template <typename T>
UintType<T> getEncodedExponent(BitsType<T> _exponent_bits)
{
	ft_assert((_exponent_bits & ~kExponentMask<T>) == 0ull);
#ifdef FLOAT_TOOLS_SILENT_MODE
	_exponent_bits = _exponent_bits & kExponentMask<T>;
#endif
	return static_cast<UintType<T>>(_exponent_bits >> kExponentShift<T>);
}

template <typename T>
int decodeExponent(UintType<T> _encoded_exponent)
{
	constexpr UintType<T> kLowExponentMask = ((1ull << kExponentSize<T>) - 1ull);
	ft_assert((_encoded_exponent & ~kLowExponentMask) == 0ull);
#ifdef FLOAT_TOOLS_SILENT_MODE
	_encoded_exponent = _encoded_exponent & kLowExponentMask;
#endif
	return static_cast<int>(_encoded_exponent) - kExponentBias<T>;
}

template <typename T>
BitsType<T> getSignificandBits(T _value)
{
	FloatConverter<T> conv;
	conv.f = _value;
	return kSignificandMask<T> & conv.u;
}

template <typename T>
T getSignificand(T _value)
{
	const BitsType<T> significand_bits = getSignificandBits<T>(_value);
	return makeSignificand<T>(significand_bits);
}

template <typename T>
BitsType<T> makeSignificandBits(T _significand)
{
	ft_assert(_significand < static_cast<T>(1));
	ft_assert(_significand >= static_cast<T>(0));
#ifdef FLOAT_TOOLS_SILENT_MODE
	_significand = std::abs(_significand);
	T integral_part = static_cast<T>(0);
	_significand = std::modf(_significand, &integral_part);
#endif
	return getSignificandBits<T>(_significand + static_cast<T>(1));
}

template <typename T>
T makeSignificand(BitsType<T> _significand_bits)
{
	ft_assert((_significand_bits & ~kSignificandMask<T>) == 0ull);
#ifdef FLOAT_TOOLS_SILENT_MODE
	_significand_bits = _significand_bits & kSignificandMask<T>;
#endif
	return makeFloat<T>(0ull, makeExponentBits<T>(0), _significand_bits) - static_cast<T>(1);
}


template <typename FloatType = float>
void demo(bool _is_negative, int _exponent, FloatType _significand)
{
	using UintType = UintType<FloatType>;
	//
	ft_assert(_exponent >= kMinExponent<FloatType>);
	ft_assert(_exponent <= kMaxExponent<FloatType>);
	ft_assert(_significand < static_cast<FloatType>(1));
	ft_assert(_significand >= static_cast<FloatType>(0));
	//
	const FloatType input_exponent_as_float = static_cast<FloatType>(_exponent);
	const FloatType ground_truth =
		std::pow(static_cast<FloatType>(2), input_exponent_as_float) *
		(static_cast<FloatType>(1) + _significand) *
		((_is_negative) ? static_cast<FloatType>(-1) : static_cast<FloatType>(1));
	FloatConverter<FloatType> ground_truth_converter; ground_truth_converter.f = ground_truth;
	//
	const FloatType validation_input_composition = makeFloat(_is_negative, _exponent, _significand);
	FloatConverter<FloatType> input_composition_converter; input_composition_converter.f = validation_input_composition;
	//
	const UintType sign_bit = getSignBit(ground_truth);
	const UintType exponent_bits = getExponentBits(ground_truth);
	const UintType significand_bits = getSignificandBits(ground_truth);
	const FloatType validation_bits_composition = makeFloat<FloatType>(sign_bit, exponent_bits, significand_bits);
	FloatConverter<FloatType> bits_composition_converter; bits_composition_converter.f = validation_bits_composition;
	//
	const bool sign = getSign(ground_truth);
	const int exponent = getExponent(ground_truth);
	const FloatType mantissa = getSignificand(ground_truth);
	const FloatType validation_elements_composition = makeFloat(sign, exponent, mantissa);
	FloatConverter<FloatType> elements_composition_converter; elements_composition_converter.f = validation_elements_composition;
	//
	// A fully set exponent (every bit is 1) means either inf or nan, but ground_truth will
	// evaluate to inf whether _significand is 0 or not, so we just bite the bullet and disable
	// post-condition validation here.
	if (_exponent < kMaxExponent<FloatType>)
	{
		ft_assert(ground_truth_converter.u == input_composition_converter.u);
		ft_assert(ground_truth_converter.u == bits_composition_converter.u);
		ft_assert(ground_truth_converter.u == elements_composition_converter.u);
	}
}


} // namespace float_tools


#undef ft_assert

#endif
