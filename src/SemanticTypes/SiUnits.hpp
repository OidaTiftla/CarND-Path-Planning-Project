//
// Created by chmst on 10/11/2016.
//

#ifndef SIUNITS_HPP
#define SIUNITS_HPP

#include <iostream>

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

template<typename Unit> // a magnitude with a unit
struct Value
{
    const double value; // the magnitude
    constexpr explicit Value(double d) : value(d) {} // construct a Value from a double
};

// Semantic Units for SI basic units
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

// Semantic Value Types for SI basic units
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

// Operator overloads to properly calculate units
template <int Tm, int Tkg, int Ts, int TA, int TK, int Tmol, int Tcd>
Value<SiUnit<Tm, Tkg, Ts, TA, TK, Tmol, Tcd>> operator+(const Value<SiUnit<Tm, Tkg, Ts, TA, TK, Tmol, Tcd>>& lhs, const Value<SiUnit<Tm, Tkg, Ts, TA, TK, Tmol, Tcd>>& rhs) {
    return Value<SiUnit<Tm, Tkg, Ts, TA, TK, Tmol, Tcd>>(lhs.value + rhs.value);
}
template <int Tm, int Tkg, int Ts, int TA, int TK, int Tmol, int Tcd>
Value<SiUnit<Tm, Tkg, Ts, TA, TK, Tmol, Tcd>> operator-(const Value<SiUnit<Tm, Tkg, Ts, TA, TK, Tmol, Tcd>>& lhs, const Value<SiUnit<Tm, Tkg, Ts, TA, TK, Tmol, Tcd>>& rhs) {
    return Value<SiUnit<Tm, Tkg, Ts, TA, TK, Tmol, Tcd>>(lhs.value - rhs.value);
}
template <int Tm_lhs, int Tkg_lhs, int Ts_lhs, int TA_lhs, int TK_lhs, int Tmol_lhs, int Tcd_lhs, int Tm_rhs, int Tkg_rhs, int Ts_rhs, int TA_rhs, int TK_rhs, int Tmol_rhs, int Tcd_rhs>
Value<SiUnit<Tm_lhs + Tm_rhs, Tkg_lhs + Tkg_rhs, Ts_lhs + Ts_rhs, TA_lhs + TA_rhs, TK_lhs + TK_rhs, Tmol_lhs + Tmol_rhs, Tcd_lhs + Tcd_rhs>> operator*(const Value<SiUnit<Tm_lhs, Tkg_lhs, Ts_lhs, TA_lhs, TK_lhs, Tmol_lhs, Tcd_lhs>>& lhs, const Value<SiUnit<Tm_rhs, Tkg_rhs, Ts_rhs, TA_rhs, TK_rhs, Tmol_rhs, Tcd_rhs>>& rhs) {
    return Value<SiUnit<Tm_lhs + Tm_rhs, Tkg_lhs + Tkg_rhs, Ts_lhs + Ts_rhs, TA_lhs + TA_rhs, TK_lhs + TK_rhs, Tmol_lhs + Tmol_rhs, Tcd_lhs + Tcd_rhs>>(lhs.value * rhs.value);
}
template <int Tm_lhs, int Tkg_lhs, int Ts_lhs, int TA_lhs, int TK_lhs, int Tmol_lhs, int Tcd_lhs, int Tm_rhs, int Tkg_rhs, int Ts_rhs, int TA_rhs, int TK_rhs, int Tmol_rhs, int Tcd_rhs>
Value<SiUnit<Tm_lhs - Tm_rhs, Tkg_lhs - Tkg_rhs, Ts_lhs - Ts_rhs, TA_lhs - TA_rhs, TK_lhs - TK_rhs, Tmol_lhs - Tmol_rhs, Tcd_lhs - Tcd_rhs>> operator/(const Value<SiUnit<Tm_lhs, Tkg_lhs, Ts_lhs, TA_lhs, TK_lhs, Tmol_lhs, Tcd_lhs>>& lhs, const Value<SiUnit<Tm_rhs, Tkg_rhs, Ts_rhs, TA_rhs, TK_rhs, Tmol_rhs, Tcd_rhs>>& rhs) {
    return Value<SiUnit<Tm_lhs - Tm_rhs, Tkg_lhs - Tkg_rhs, Ts_lhs - Ts_rhs, TA_lhs - TA_rhs, TK_lhs - TK_rhs, Tmol_lhs - Tmol_rhs, Tcd_lhs - Tcd_rhs>>(lhs.value / rhs.value);
}

template <int Tm, int Tkg, int Ts, int TA, int TK, int Tmol, int Tcd>
std::ostream& operator<<(std::ostream& out, const Value<SiUnit<Tm, Tkg, Ts, TA, TK, Tmol, Tcd>>& o) {
    out << o.value;

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

// Define literals for SI basic units
constexpr Distance operator"" _m(long double ld)
{
    return Distance(static_cast<double>(ld));
}
constexpr Mass operator"" _kg(long double ld)
{
    return Mass(static_cast<double>(ld));
}
constexpr Time operator"" _s(long double ld)
{
    return Time(static_cast<double>(ld));
}
constexpr Current operator"" _A(long double ld)
{
    return Current(static_cast<double>(ld));
}
constexpr Temperature operator"" _K(long double ld)
{
    return Temperature(static_cast<double>(ld));
}
constexpr AmountOfSubstance operator"" _mol(long double ld)
{
    return AmountOfSubstance(static_cast<double>(ld));
}
constexpr LuminousIntensity operator"" _cd(long double ld)
{
    return LuminousIntensity(static_cast<double>(ld));
}

// Define literals for SI derived units
constexpr Frequency operator"" _Hz(long double ld)
{
    return Frequency(static_cast<double>(ld));
}
constexpr Force operator"" _N(long double ld)
{
    return Force(static_cast<double>(ld));
}
constexpr Pressure operator"" _Pa(long double ld)
{
    return Pressure(static_cast<double>(ld));
}
constexpr Energy operator"" _J(long double ld)
{
    return Energy(static_cast<double>(ld));
}
constexpr Power operator"" _W(long double ld)
{
    return Power(static_cast<double>(ld));
}
constexpr ElectricCharge operator"" _C(long double ld)
{
    return ElectricCharge(static_cast<double>(ld));
}
constexpr Voltage operator"" _V(long double ld)
{
    return Voltage(static_cast<double>(ld));
}
constexpr Capacitance operator"" _F(long double ld)
{
    return Capacitance(static_cast<double>(ld));
}
constexpr Resistance operator"" _Ohm(long double ld)
{
    return Resistance(static_cast<double>(ld));
}
constexpr ElectricalConductance operator"" _S(long double ld)
{
    return ElectricalConductance(static_cast<double>(ld));
}
constexpr MagneticFlux operator"" _Wb(long double ld)
{
    return MagneticFlux(static_cast<double>(ld));
}
constexpr MagneticFluxDensity operator"" _T(long double ld)
{
    return MagneticFluxDensity(static_cast<double>(ld));
}
constexpr Inductance operator"" _H(long double ld)
{
    return Inductance(static_cast<double>(ld));
}
constexpr Illuminance operator"" _lx(long double ld)
{
    return Illuminance(static_cast<double>(ld));
}

#endif //SIUNITS_HPP
