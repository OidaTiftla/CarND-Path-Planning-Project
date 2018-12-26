//
// Created by chmst on 10/11/2016.
// The idea came from Compiler-enforced semantic types (https://stackoverflow.com/questions/39412785/compiler-enforced-semantic-types)
//

#ifndef UNITS_HPP
#define UNITS_HPP

#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>

// metre, kilogram, second, ampere, kelvin, mole, candela
// https://en.wikipedia.org/wiki/International_System_of_Units
template <int Tm, int Tkg, int Ts, int TA, int TK, int Tmol, int Tcd>
struct SiUnit {
    enum {
        m = Tm,
        kg = Tkg,
        s = Ts,
        A = TA,
        K = TK,
        mol = Tmol,
        cd = Tcd,
    };
};

// Semantic Units for SI basic units
using NoUnit = SiUnit<0, 0, 0, 0, 0, 0, 0>;
using Meter = SiUnit<1, 0, 0, 0, 0, 0, 0>;
using Kilogram = SiUnit<0, 1, 0, 0, 0, 0, 0>;
using Second = SiUnit<0, 0, 1, 0, 0, 0, 0>;
using Ampere = SiUnit<0, 0, 0, 1, 0, 0, 0>;
using Kelvin = SiUnit<0, 0, 0, 0, 1, 0, 0>;
using Mole = SiUnit<0, 0, 0, 0, 0, 1, 0>;
using Candela = SiUnit<0, 0, 0, 0, 0, 0, 1>;

// Semantic Units for SI derived units
using Hertz = SiUnit<0, 0, -1, 0, 0, 0, 0>; // is 1/second (Hz)
using Newton = SiUnit<1, 1, -2, 0, 0, 0, 0>; // is kilogram*meters/second^2 (N)
using Pascal = SiUnit<-1, 1, -2, 0, 0, 0, 0>; // is kilogram/meters/second^2 (Pa)
using Joule = SiUnit<2, 1, -2, 0, 0, 0, 0>; // is kilogram*meters^2/second^2 (J)
using Watt = SiUnit<2, 1, -3, 0, 0, 0, 0>; // is kilogram*meters^2/second^3 (W)
using Coulomb = SiUnit<0, 0, 1, 1, 0, 0, 0>; // is second*ampere (C)
using Volt = SiUnit<1, 1, -3, -1, 0, 0, 0>; // is kilogram*meters^2/second^3/ampere (V)
using Farad = SiUnit<-2, -1, 4, 2, 0, 0, 0>; // is second^4*ampere^2/kilogram/meters^2 (F)
using Ohm = SiUnit<2, 1, -3, -2, 0, 0, 0>; // is kilogram*meters^2/second^3/ampere^2 (Ohm)
using Siemens = SiUnit<-2, -1, 3, 2, 0, 0, 0>; // is second^3*ampere^2/kilogram/meters^2 (S)
using Weber = SiUnit<2, 1, -2, -1, 0, 0, 0>; // is kilogram*meter^2/second^2/ampere (Wb)
using Tesla = SiUnit<0, 1, -2, -1, 0, 0, 0>; // is kilogram/second^2/ampere (T)
using Henry = SiUnit<2, 1, -2, -2, 0, 0, 0>; // is kilogram*meter^2/second^2/ampere^2 (H)
using Lux = SiUnit<-2, 0, 0, 0, 0, 0, 1>; // is candela/meter^2 (lx)

// radian, degree
template <bool Trad, bool Tdeg>
struct AngleUnit {
    enum {
        rad = Trad,
        deg = Tdeg,
    };
};

// Semantic Units for angle units
using Radian = AngleUnit<true, false>;
using Degree = AngleUnit<false, true>;

template<typename TUnit> // a magnitude with a unit
struct Value {
    double value; // the magnitude
    constexpr explicit Value(const double d) : value(d) {} // construct a Value from a double

    Value operator-() const {
        return Value(-this->value);
    }

    Value& operator+() const {
        return *this;
    }

    Value& operator+=(const Value& rhs) {
        this->value += rhs.value;
        return *this;
    }

    Value& operator-=(const Value& rhs) {
        this->value -= rhs.value;
        return *this;
    }

    Value& operator*=(const double rhs) {
        this->value *= rhs;
        return *this;
    }

    Value& operator/=(const double rhs) {
        this->value /= rhs;
        return *this;
    }

    template<typename TUnit_ = TUnit>
    operator typename std::enable_if<std::is_same<TUnit_, NoUnit>::value, double>::type() {
        return value;
    }
};

// Semantic Value Types for SI basic units
using Number = Value<NoUnit>;
using Distance = Value<Meter>;
using Mass = Value<Kilogram>;
using Time = Value<Second>;
using Current = Value<Ampere>;
using Temperature = Value<Kelvin>;
using AmountOfSubstance = Value<Mole>;
using LuminousIntensity = Value<Candela>;

// Semantic Value Types for SI derived units
using Speed = Value<SiUnit<1, 0, -1, 0, 0, 0, 0>>; // is meters/second
using Acceleration = Value<SiUnit<1, 0, -2, 0, 0, 0, 0>>; // is meters/second^2
using Jerk = Value<SiUnit<1, 0, -3, 0, 0, 0, 0>>; // is meters/second^3
using Frequency = Value<Hertz>; // is Hz
using Force = Value<Newton>; // is N
using Pressure = Value<Pascal>; // is Pa
using Energy = Value<Joule>; // is J
using Power = Value<Watt>; // is W
using ElectricCharge = Value<Coulomb>; // is C
using Voltage = Value<Volt>; // is V
using Capacitance = Value<Farad>; // is F
using Resistance = Value<Ohm>; // is Ohm
using ElectricalConductance = Value<Siemens>; // is S
using MagneticFlux = Value<Weber>; // is Wb
using MagneticFluxDensity = Value<Tesla>; // is T
using Inductance = Value<Henry>; // is H
using Illuminance = Value<Lux>; // is lx

// Semantic Value Types for angle units
using AngleRad = Value<Radian>; // is rad
using AngleDeg = Value<Degree>; // is °

// Operator overloads to properly calculate units for SI units
template <int Tm, int Tkg, int Ts, int TA, int TK, int Tmol, int Tcd>
inline constexpr Value<SiUnit<Tm, Tkg, Ts, TA, TK, Tmol, Tcd>> operator+(const Value<SiUnit<Tm, Tkg, Ts, TA, TK, Tmol, Tcd>>& lhs, const Value<SiUnit<Tm, Tkg, Ts, TA, TK, Tmol, Tcd>>& rhs) {
    return Value<SiUnit<Tm, Tkg, Ts, TA, TK, Tmol, Tcd>>(lhs.value + rhs.value);
}
template <int Tm, int Tkg, int Ts, int TA, int TK, int Tmol, int Tcd>
inline constexpr Value<SiUnit<Tm, Tkg, Ts, TA, TK, Tmol, Tcd>> operator-(const Value<SiUnit<Tm, Tkg, Ts, TA, TK, Tmol, Tcd>>& lhs, const Value<SiUnit<Tm, Tkg, Ts, TA, TK, Tmol, Tcd>>& rhs) {
    return Value<SiUnit<Tm, Tkg, Ts, TA, TK, Tmol, Tcd>>(lhs.value - rhs.value);
}
template <int Tm_lhs, int Tkg_lhs, int Ts_lhs, int TA_lhs, int TK_lhs, int Tmol_lhs, int Tcd_lhs, int Tm_rhs, int Tkg_rhs, int Ts_rhs, int TA_rhs, int TK_rhs, int Tmol_rhs, int Tcd_rhs>
inline constexpr Value<SiUnit<Tm_lhs + Tm_rhs, Tkg_lhs + Tkg_rhs, Ts_lhs + Ts_rhs, TA_lhs + TA_rhs, TK_lhs + TK_rhs, Tmol_lhs + Tmol_rhs, Tcd_lhs + Tcd_rhs>> operator*(const Value<SiUnit<Tm_lhs, Tkg_lhs, Ts_lhs, TA_lhs, TK_lhs, Tmol_lhs, Tcd_lhs>>& lhs, const Value<SiUnit<Tm_rhs, Tkg_rhs, Ts_rhs, TA_rhs, TK_rhs, Tmol_rhs, Tcd_rhs>>& rhs) {
    return Value<SiUnit<Tm_lhs + Tm_rhs, Tkg_lhs + Tkg_rhs, Ts_lhs + Ts_rhs, TA_lhs + TA_rhs, TK_lhs + TK_rhs, Tmol_lhs + Tmol_rhs, Tcd_lhs + Tcd_rhs>>(lhs.value * rhs.value);
}
template <int Tm_lhs, int Tkg_lhs, int Ts_lhs, int TA_lhs, int TK_lhs, int Tmol_lhs, int Tcd_lhs, int Tm_rhs, int Tkg_rhs, int Ts_rhs, int TA_rhs, int TK_rhs, int Tmol_rhs, int Tcd_rhs>
inline constexpr Value<SiUnit<Tm_lhs - Tm_rhs, Tkg_lhs - Tkg_rhs, Ts_lhs - Ts_rhs, TA_lhs - TA_rhs, TK_lhs - TK_rhs, Tmol_lhs - Tmol_rhs, Tcd_lhs - Tcd_rhs>> operator/(const Value<SiUnit<Tm_lhs, Tkg_lhs, Ts_lhs, TA_lhs, TK_lhs, Tmol_lhs, Tcd_lhs>>& lhs, const Value<SiUnit<Tm_rhs, Tkg_rhs, Ts_rhs, TA_rhs, TK_rhs, Tmol_rhs, Tcd_rhs>>& rhs) {
    return Value<SiUnit<Tm_lhs - Tm_rhs, Tkg_lhs - Tkg_rhs, Ts_lhs - Ts_rhs, TA_lhs - TA_rhs, TK_lhs - TK_rhs, Tmol_lhs - Tmol_rhs, Tcd_lhs - Tcd_rhs>>(lhs.value / rhs.value);
}

// Operator overloads to properly calculate units for angle units
template <typename TUnit>
inline constexpr Value<TUnit> operator+(const Value<TUnit>& lhs, const Value<TUnit>& rhs) {
    return Value<TUnit>(lhs.value + rhs.value);
}
template <typename TUnit>
inline constexpr Value<TUnit> operator-(const Value<TUnit>& lhs, const Value<TUnit>& rhs) {
    return Value<TUnit>(lhs.value - rhs.value);
}
template <typename TUnit>
inline constexpr Value<TUnit> operator*(const Value<TUnit>& lhs, const double rhs) {
    return Value<TUnit>(lhs.value * rhs);
}
template <typename TUnit>
inline constexpr Value<TUnit> operator*(const double lhs, const Value<TUnit>& rhs) {
    return Value<TUnit>(lhs * rhs.value);
}
template <typename TUnit>
inline constexpr Value<TUnit> operator/(const Value<TUnit>& lhs, const double rhs) {
    return Value<TUnit>(lhs.value / rhs);
}

// Operator overloads to properly compare values
template <typename TUnit>
inline constexpr bool operator<(const Value<TUnit>& lhs, const Value<TUnit>& rhs) {
    return lhs.value < rhs.value;
}
template <typename TUnit>
inline constexpr bool operator<=(const Value<TUnit>& lhs, const Value<TUnit>& rhs) {
    return lhs.value <= rhs.value;
}
template <typename TUnit>
inline constexpr bool operator>(const Value<TUnit>& lhs, const Value<TUnit>& rhs) {
    return lhs.value > rhs.value;
}
template <typename TUnit>
inline constexpr bool operator>=(const Value<TUnit>& lhs, const Value<TUnit>& rhs) {
    return lhs.value >= rhs.value;
}
template <typename TUnit>
inline constexpr bool operator==(const Value<TUnit>& lhs, const Value<TUnit>& rhs) {
    return lhs.value == rhs.value;
}
template <typename TUnit>
inline constexpr bool operator!=(const Value<TUnit>& lhs, const Value<TUnit>& rhs) {
    return lhs.value != rhs.value;
}

// write to stream with units for SI units
template <int Tm, int Tkg, int Ts, int TA, int TK, int Tmol, int Tcd>
inline std::ostream& operator<<(std::ostream& out, const Value<SiUnit<Tm, Tkg, Ts, TA, TK, Tmol, Tcd>>& o) {
    out << o.value;

    // check for SI derived units
    if (Tm == 0
        && Tkg == 0
        && Ts == -1
        && TA == 0
        && TK == 0
        && Tmol == 0
        && Tcd == 0) {
        return out << "Hz";
    } else if (Tm == 1
        && Tkg == 1
        && Ts == -2
        && TA == 0
        && TK == 0
        && Tmol == 0
        && Tcd == 0) {
        return out << "N";
    } else if (Tm == -1
        && Tkg == 1
        && Ts == -2
        && TA == 0
        && TK == 0
        && Tmol == 0
        && Tcd == 0) {
        return out << "Pa";
    } else if (Tm == 2
        && Tkg == 1
        && Ts == -2
        && TA == 0
        && TK == 0
        && Tmol == 0
        && Tcd == 0) {
        return out << "J";
    } else if (Tm == 2
        && Tkg == 1
        && Ts == -3
        && TA == 0
        && TK == 0
        && Tmol == 0
        && Tcd == 0) {
        return out << "W";
    } else if (Tm == 0
        && Tkg == 0
        && Ts == 1
        && TA == 1
        && TK == 0
        && Tmol == 0
        && Tcd == 0) {
        return out << "C";
    } else if (Tm == 1
        && Tkg == 1
        && Ts == -3
        && TA == -1
        && TK == 0
        && Tmol == 0
        && Tcd == 0) {
        return out << "V";
    } else if (Tm == -2
        && Tkg == -1
        && Ts == 4
        && TA == 2
        && TK == 0
        && Tmol == 0
        && Tcd == 0) {
        return out << "F";
    } else if (Tm == 2
        && Tkg == 1
        && Ts == -3
        && TA == -2
        && TK == 0
        && Tmol == 0
        && Tcd == 0) {
        return out << "Ohm";
    } else if (Tm == -2
        && Tkg == -1
        && Ts == 3
        && TA == 2
        && TK == 0
        && Tmol == 0
        && Tcd == 0) {
        return out << "S";
    } else if (Tm == 2
        && Tkg == 1
        && Ts == -2
        && TA == -1
        && TK == 0
        && Tmol == 0
        && Tcd == 0) {
        return out << "Wb";
    } else if (Tm == 0
        && Tkg == 1
        && Ts == -2
        && TA == -1
        && TK == 0
        && Tmol == 0
        && Tcd == 0) {
        return out << "T";
    } else if (Tm == 2
        && Tkg == 1
        && Ts == -2
        && TA == -2
        && TK == 0
        && Tmol == 0
        && Tcd == 0) {
        return out << "H";
    } else if (Tm == -2
        && Tkg == 0
        && Ts == 0
        && TA == 0
        && TK == 0
        && Tmol == 0
        && Tcd == 1) {
        return out << "lx";
    } else {
        // use SI basic units
        bool first = true;
        if (Tm != 0) {
            if (!first) {
                out << "*";
            }
            if (Tm == 1) {
                out << "m";
            } else {
                out << "m^" << Tm;
            }
            first = false;
        }
        if (Tkg != 0) {
            if (!first) {
                out << "*";
            }
            if (Tkg == 1) {
                out << "kg";
            } else {
                out << "kg^" << Tkg;
            }
            first = false;
        }
        if (Ts != 0) {
            if (!first) {
                out << "*";
            }
            if (Ts == 1) {
                out << "s";
            } else {
                out << "s^" << Ts;
            }
            first = false;
        }
        if (TA != 0) {
            if (!first) {
                out << "*";
            }
            if (TA == 1) {
                out << "A";
            } else {
                out << "A^" << TA;
            }
            first = false;
        }
        if (TK != 0) {
            if (!first) {
                out << "*";
            }
            if (TK == 1) {
                out << "K";
            } else {
                out << "K^" << TK;
            }
            first = false;
        }
        if (Tmol != 0) {
            if (!first) {
                out << "*";
            }
            if (Tmol == 1) {
                out << "mol";
            } else {
                out << "mol^" << Tmol;
            }
            first = false;
        }
        if (Tcd != 0) {
            if (!first) {
                out << "*";
            }
            if (Tcd == 1) {
                out << "cd";
            } else {
                out << "cd^" << Tcd;
            }
            first = false;
        }
        return out;
    }
}

// write to stream with units for angle units
inline std::ostream& operator<<(std::ostream& out, const AngleRad& o) {
    return out << o.value << "rad";
}
inline std::ostream& operator<<(std::ostream& out, const AngleDeg& o) {
    return out << o.value << "°";
}


// Define literals for SI basic units
inline constexpr Distance operator"" _m(long double ld) {
    return Distance(static_cast<double>(ld));
}
inline constexpr Distance operator"" _m(unsigned long long ull) {
    return Distance(static_cast<double>(ull));
}
inline constexpr Mass operator"" _kg(long double ld) {
    return Mass(static_cast<double>(ld));
}
inline constexpr Mass operator"" _kg(unsigned long long ull) {
    return Mass(static_cast<double>(ull));
}
inline constexpr Time operator"" _s(long double ld) {
    return Time(static_cast<double>(ld));
}
inline constexpr Time operator"" _s(unsigned long long ull) {
    return Time(static_cast<double>(ull));
}
inline constexpr Current operator"" _A(long double ld) {
    return Current(static_cast<double>(ld));
}
inline constexpr Current operator"" _A(unsigned long long ull) {
    return Current(static_cast<double>(ull));
}
inline constexpr Temperature operator"" _K(long double ld) {
    return Temperature(static_cast<double>(ld));
}
inline constexpr Temperature operator"" _K(unsigned long long ull) {
    return Temperature(static_cast<double>(ull));
}
inline constexpr AmountOfSubstance operator"" _mol(long double ld) {
    return AmountOfSubstance(static_cast<double>(ld));
}
inline constexpr AmountOfSubstance operator"" _mol(unsigned long long ull) {
    return AmountOfSubstance(static_cast<double>(ull));
}
inline constexpr LuminousIntensity operator"" _cd(long double ld) {
    return LuminousIntensity(static_cast<double>(ld));
}
inline constexpr LuminousIntensity operator"" _cd(unsigned long long ull) {
    return LuminousIntensity(static_cast<double>(ull));
}

// Define literals for SI derived units
inline constexpr Frequency operator"" _Hz(long double ld) {
    return Frequency(static_cast<double>(ld));
}
inline constexpr Frequency operator"" _Hz(unsigned long long ull) {
    return Frequency(static_cast<double>(ull));
}
inline constexpr Force operator"" _N(long double ld) {
    return Force(static_cast<double>(ld));
}
inline constexpr Force operator"" _N(unsigned long long ull) {
    return Force(static_cast<double>(ull));
}
inline constexpr Pressure operator"" _Pa(long double ld) {
    return Pressure(static_cast<double>(ld));
}
inline constexpr Pressure operator"" _Pa(unsigned long long ull) {
    return Pressure(static_cast<double>(ull));
}
inline constexpr Energy operator"" _J(long double ld) {
    return Energy(static_cast<double>(ld));
}
inline constexpr Energy operator"" _J(unsigned long long ull) {
    return Energy(static_cast<double>(ull));
}
inline constexpr Power operator"" _W(long double ld) {
    return Power(static_cast<double>(ld));
}
inline constexpr Power operator"" _W(unsigned long long ull) {
    return Power(static_cast<double>(ull));
}
inline constexpr ElectricCharge operator"" _C(long double ld) {
    return ElectricCharge(static_cast<double>(ld));
}
inline constexpr ElectricCharge operator"" _C(unsigned long long ull) {
    return ElectricCharge(static_cast<double>(ull));
}
inline constexpr Voltage operator"" _V(long double ld) {
    return Voltage(static_cast<double>(ld));
}
inline constexpr Voltage operator"" _V(unsigned long long ull) {
    return Voltage(static_cast<double>(ull));
}
inline constexpr Capacitance operator"" _F(long double ld) {
    return Capacitance(static_cast<double>(ld));
}
inline constexpr Capacitance operator"" _F(unsigned long long ull) {
    return Capacitance(static_cast<double>(ull));
}
inline constexpr Resistance operator"" _Ohm(long double ld) {
    return Resistance(static_cast<double>(ld));
}
inline constexpr Resistance operator"" _Ohm(unsigned long long ull) {
    return Resistance(static_cast<double>(ull));
}
inline constexpr ElectricalConductance operator"" _S(long double ld) {
    return ElectricalConductance(static_cast<double>(ld));
}
inline constexpr ElectricalConductance operator"" _S(unsigned long long ull) {
    return ElectricalConductance(static_cast<double>(ull));
}
inline constexpr MagneticFlux operator"" _Wb(long double ld) {
    return MagneticFlux(static_cast<double>(ld));
}
inline constexpr MagneticFlux operator"" _Wb(unsigned long long ull) {
    return MagneticFlux(static_cast<double>(ull));
}
inline constexpr MagneticFluxDensity operator"" _T(long double ld) {
    return MagneticFluxDensity(static_cast<double>(ld));
}
inline constexpr MagneticFluxDensity operator"" _T(unsigned long long ull) {
    return MagneticFluxDensity(static_cast<double>(ull));
}
inline constexpr Inductance operator"" _H(long double ld) {
    return Inductance(static_cast<double>(ld));
}
inline constexpr Inductance operator"" _H(unsigned long long ull) {
    return Inductance(static_cast<double>(ull));
}
inline constexpr Illuminance operator"" _lx(long double ld) {
    return Illuminance(static_cast<double>(ld));
}
inline constexpr Illuminance operator"" _lx(unsigned long long ull) {
    return Illuminance(static_cast<double>(ull));
}

// Define literals for angle units
inline constexpr AngleRad operator"" _rad(long double ld) {
    return AngleRad(static_cast<double>(ld));
}
inline constexpr AngleRad operator"" _rad(unsigned long long ull) {
    return AngleRad(static_cast<double>(ull));
}
inline constexpr AngleDeg operator"" _deg(long double ld) {
    return AngleDeg(static_cast<double>(ld));
}
inline constexpr AngleDeg operator"" _deg(unsigned long long ull) {
    return AngleDeg(static_cast<double>(ull));
}

// Define literals for imperial units
inline constexpr Speed operator"" _mph(long double ld) {
    return static_cast<double>(ld) * (1609.344_m / 3600_s);
}
inline constexpr Speed operator"" _mph(unsigned long long ull) {
    return static_cast<double>(ull) * (1609.344_m / 3600_s);
}

// Define some helper functions
template <typename TUnit>
inline constexpr Value<TUnit> abs(const Value<TUnit>& v) {
    return Value<TUnit>(abs(v.value));
}

template <typename TUnit>
inline constexpr Value<TUnit> max(const Value<TUnit>& v1, const Value<TUnit>& v2) {
    return Value<TUnit>(max(v1.value, v2.value));
}

template <typename TUnit>
inline constexpr Value<TUnit> min(const Value<TUnit>& v1, const Value<TUnit>& v2) {
    return Value<TUnit>(min(v1.value, v2.value));
}

inline constexpr AngleRad ToRadian(const AngleDeg& deg) {
    return AngleRad(deg.value * M_PI / 180);
}

inline constexpr AngleDeg ToDegree(const AngleRad& rad) {
    return AngleDeg(rad.value * 180 / M_PI);
}

inline AngleDeg NormalizeAroundZero(AngleDeg v) {
    while (v > 180_deg) {
        v -= 360_deg;
    }
    while (v < -180_deg) {
        v += 360_deg;
    }
    return v;
}

inline AngleRad NormalizeAroundZero(AngleRad v) {
    while (v > AngleRad(M_PI)) {
        v -= AngleRad(M_2_PI);
    }
    while (v < AngleRad(-M_PI)) {
        v += AngleRad(M_2_PI);
    }
    return v;
}

inline constexpr double sin(const AngleRad& rad) {
    return sin(rad.value);
}

inline constexpr double cos(const AngleRad& rad) {
    return cos(rad.value);
}

inline constexpr double tan(const AngleRad& rad) {
    return tan(rad.value);
}

inline constexpr double sin(const AngleDeg& deg) {
    return sin(ToRadian(deg));
}

inline constexpr double cos(const AngleDeg& deg) {
    return cos(ToRadian(deg));
}

inline constexpr double tan(const AngleDeg& deg) {
    return tan(ToRadian(deg));
}

template <typename TUnit>
inline constexpr AngleRad atan2(const Value<TUnit>& y, const Value<TUnit>& x) {
    return AngleRad(atan2(y.value, x.value));
}

template <typename TUnit>
inline constexpr AngleRad atan(const Value<TUnit>& y, const Value<TUnit>& x) {
    return AngleRad(atan(y.value, x.value));
}

template <int Tm, int Tkg, int Ts, int TA, int TK, int Tmol, int Tcd>
inline constexpr
typename std::enable_if<Tm % 2 == 0 && Tkg % 2 == 0 && Ts % 2 == 0 && TA % 2 == 0 && TK % 2 == 0 && Tmol % 2 == 0 && Tcd % 2 == 0,
    Value<SiUnit<Tm / 2, Tkg / 2, Ts / 2, TA / 2, TK / 2, Tmol / 2, Tcd / 2>>>::type sqrt(const Value<SiUnit<Tm, Tkg, Ts, TA, TK, Tmol, Tcd>>& v) {
    return Value<SiUnit<Tm / 2, Tkg / 2, Ts / 2, TA / 2, TK / 2, Tmol / 2, Tcd / 2>>(sqrt(v.value));
}

template <int exp, int Tm, int Tkg, int Ts, int TA, int TK, int Tmol, int Tcd>
inline Value<SiUnit<Tm * exp, Tkg * exp, Ts * exp, TA * exp, TK * exp, Tmol * exp, Tcd * exp>> pow(const Value<SiUnit<Tm, Tkg, Ts, TA, TK, Tmol, Tcd>>& v) {
    double value = 1;
    for (int i = 0; i < exp; ++i) {
        value *= v.value;
    }
    if (exp < 0) {
        value = 1 / value;
    }
    return Value<SiUnit<Tm * exp, Tkg * exp, Ts * exp, TA * exp, TK * exp, Tmol * exp, Tcd * exp>>(value);
}

#endif //UNITS_HPP
