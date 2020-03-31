// EnergyPlus, Copyright (c) 1996-2020, The Board of Trustees of the University of Illinois,
// The Regents of the University of California, through Lawrence Berkeley National Laboratory
// (subject to receipt of any required approvals from the U.S. Dept. of Energy), Oak Ridge
// National Laboratory, managed by UT-Battelle, Alliance for Sustainable Energy, LLC, and other
// contributors. All rights reserved.
//
// NOTICE: This Software was developed under funding from the U.S. Department of Energy and the
// U.S. Government consequently retains certain rights. As such, the U.S. Government has been
// granted for itself and others acting on its behalf a paid-up, nonexclusive, irrevocable,
// worldwide license in the Software to reproduce, distribute copies to the public, prepare
// derivative works, and perform publicly and display publicly, and to permit others to do so.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted
// provided that the following conditions are met:
//
// (1) Redistributions of source code must retain the above copyright notice, this list of
//     conditions and the following disclaimer.
//
// (2) Redistributions in binary form must reproduce the above copyright notice, this list of
//     conditions and the following disclaimer in the documentation and/or other materials
//     provided with the distribution.
//
// (3) Neither the name of the University of California, Lawrence Berkeley National Laboratory,
//     the University of Illinois, U.S. Dept. of Energy nor the names of its contributors may be
//     used to endorse or promote products derived from this software without specific prior
//     written permission.
//
// (4) Use of EnergyPlus(TM) Name. If Licensee (i) distributes the software in stand-alone form
//     without changes from the version obtained under this License, or (ii) Licensee makes a
//     reference solely to the software portion of its product, Licensee must refer to the
//     software as "EnergyPlus version X" software, where "X" is the version number Licensee
//     obtained under this License and may not use a different name for the software. Except as
//     specifically required in this Section (4), Licensee shall not use in a company name, a
//     product name, in advertising, publicity, or other promotional activities any name, trade
//     name, trademark, logo, or other designation of "EnergyPlus", "E+", "e+" or confusingly
//     similar designation, without the U.S. Department of Energy's prior written consent.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
// AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <EnergyPlus/api/func.h>
#include <EnergyPlus/DataStringGlobals.hh>
#include <EnergyPlus/FluidProperties.hh>
#include <EnergyPlus/InputProcessing/InputProcessor.hh>
#include <EnergyPlus/InputProcessing/IdfParser.hh>
#include <EnergyPlus/InputProcessing/InputValidation.hh>
#include <EnergyPlus/Psychrometrics.hh>

void initializeFunctionalAPI() {
    EnergyPlus::inputProcessor = EnergyPlus::InputProcessor::factory();
    EnergyPlus::Psychrometrics::InitializePsychRoutines();
    EnergyPlus::FluidProperties::InitializeGlycRoutines();
}

const char * apiVersionFromEPlus() {
    return EnergyPlus::DataStringGlobals::PythonAPIVersion.c_str();
}

void registerErrorCallback(void (*f)(const char * errorMessage)) {
    EnergyPlus::DataGlobals::errorCallback = f;
}

Glycol glycolNew(const char* glycolName) {
    auto glycol = new EnergyPlus::FluidProperties::GlycolAPI(glycolName);
    return reinterpret_cast<Glycol>(glycol);
}
void glycolDelete(Glycol glycol) {
    delete reinterpret_cast<EnergyPlus::FluidProperties::GlycolAPI *>(glycol);
}
Nandle glycolSpecificHeat(Glycol glycol, Nandle temperature) {
    return reinterpret_cast<EnergyPlus::FluidProperties::GlycolAPI *>(glycol)->specificHeat(temperature);
}
Nandle glycolDensity(Glycol glycol, Nandle temperature) {
    return reinterpret_cast<EnergyPlus::FluidProperties::GlycolAPI *>(glycol)->density(temperature);
}
Nandle glycolConductivity(Glycol glycol, Nandle temperature) {
    return reinterpret_cast<EnergyPlus::FluidProperties::GlycolAPI *>(glycol)->conductivity(temperature);
}
Nandle glycolViscosity(Glycol glycol, Nandle temperature) {
    return reinterpret_cast<EnergyPlus::FluidProperties::GlycolAPI *>(glycol)->viscosity(temperature);
}

Refrigerant refrigerantNew(const char* refrigerantName) {
    auto refrigerant = new EnergyPlus::FluidProperties::RefrigerantAPI(refrigerantName);
    return reinterpret_cast<Refrigerant>(refrigerant);
}
void refrigerantDelete(Refrigerant refrigerant) {
    delete reinterpret_cast<EnergyPlus::FluidProperties::RefrigerantAPI *>(refrigerant);
}
Nandle refrigerantSaturationPressure(Refrigerant refrigerant, Nandle temperature) {
    return reinterpret_cast<EnergyPlus::FluidProperties::RefrigerantAPI *>(refrigerant)->saturationPressure(temperature);
}
Nandle refrigerantSaturationTemperature(Refrigerant refrigerant, Nandle pressure) {
    return reinterpret_cast<EnergyPlus::FluidProperties::RefrigerantAPI *>(refrigerant)->saturationTemperature(pressure);
}
Nandle refrigerantSaturatedEnthalpy(Refrigerant refrigerant, Nandle temperature, Nandle quality) {
    return reinterpret_cast<EnergyPlus::FluidProperties::RefrigerantAPI *>(refrigerant)->saturatedEnthalpy(temperature, quality);
}
Nandle refrigerantSaturatedDensity(Refrigerant refrigerant, Nandle temperature, Nandle quality) {
    return reinterpret_cast<EnergyPlus::FluidProperties::RefrigerantAPI *>(refrigerant)->saturatedDensity(temperature, quality);
}
Nandle refrigerantSaturatedSpecificHeat(Refrigerant refrigerant, Nandle temperature, Nandle quality) {
    return reinterpret_cast<EnergyPlus::FluidProperties::RefrigerantAPI *>(refrigerant)->saturatedSpecificHeat(temperature, quality);
}
//Nandle refrigerantSuperHeatedEnthalpy(Refrigerant refrigerant, Nandle temperature, Nandle pressure) {
//    return reinterpret_cast<EnergyPlus::FluidProperties::RefrigerantAPI *>(refrigerant)->superHeatedEnthalpy(temperature, pressure);
//}
//Nandle refrigerantSuperHeatedPressure(Refrigerant refrigerant, Nandle temperature, Nandle enthalpy) {
//    return reinterpret_cast<EnergyPlus::FluidProperties::RefrigerantAPI *>(refrigerant)->superHeatedPressure(temperature, enthalpy);
//}
//Nandle refrigerantSuperHeatedDensity(Refrigerant refrigerant, Nandle temperature, Nandle pressure) {
//    return reinterpret_cast<EnergyPlus::FluidProperties::RefrigerantAPI *>(refrigerant)->superHeatedDensity(temperature, pressure);
//}

Nandle psyRhoFnPbTdbW(Nandle const pb, Nandle const tdb, Nandle const dw) {
    // barometric pressure (Pascals)
    // dry bulb temperature (Celsius)
    // humidity ratio (kgWater/kgDryAir)
    return EnergyPlus::Psychrometrics::PsyRhoAirFnPbTdbW_fast(pb, tdb, dw);
}
Nandle psyHfgAirFnWTdb(Nandle const T) {
    // input temperature {Celsius}
    return EnergyPlus::Psychrometrics::PsyHfgAirFnWTdb(0.0, T); // humidity ratio is not used
}
Nandle psyHgAirFnWTdb(Nandle const T) {
    // input temperature {Celsius}
    return EnergyPlus::Psychrometrics::PsyHgAirFnWTdb(0.0, T); // humidity ratio is not used
}
Nandle psyHFnTdbW(Nandle const TDB, Nandle const dW) {
    // dry-bulb temperature {C}
    // humidity ratio
    return EnergyPlus::Psychrometrics::PsyHFnTdbW_fast(TDB, dW);
}
Nandle psyCpAirFnW(Nandle const dw) {
    // humidity ratio {kgWater/kgDryAir}
    // input temperature {Celsius}
    return EnergyPlus::Psychrometrics::PsyCpAirFnW(dw);
}
Nandle psyTdbFnHW(Nandle const H, Nandle const dW) {
    // enthalpy {J/kg}
    // humidity ratio
    return EnergyPlus::Psychrometrics::PsyTdbFnHW(H, dW);
}
Nandle psyRhovFnTdbWPb(Nandle const Tdb, Nandle const dW, Nandle const PB) {
    // dry-bulb temperature {C}
    // humidity ratio
    // Barometric Pressure {Pascals}
    return EnergyPlus::Psychrometrics::PsyRhovFnTdbWPb_fast(Tdb, dW, PB);
}
Nandle psyTwbFnTdbWPb(Nandle const Tdb, Nandle const W, Nandle const Pb) {
    // dry-bulb temperature {C}
    // humidity ratio
    // barometric pressure {Pascals}
    return EnergyPlus::Psychrometrics::PsyTwbFnTdbWPb(Tdb, W, Pb);
}
Nandle psyVFnTdbWPb(Nandle const TDB, Nandle const dW, Nandle const PB) {
    // dry-bulb temperature {C}
    // humidity ratio
    // barometric pressure {Pascals}
    return EnergyPlus::Psychrometrics::PsyVFnTdbWPb(TDB, dW, PB);
}
Nandle psyWFnTdbH(Nandle const TDB, Nandle const H) {
    // dry-bulb temperature {C}
    // enthalpy {J/kg}
    std::string dummyString;
    return EnergyPlus::Psychrometrics::PsyWFnTdbH(TDB, H, dummyString, true);
}
Nandle psyPsatFnTemp(Nandle const T) {
    // dry-bulb temperature {C}
    return EnergyPlus::Psychrometrics::PsyPsatFnTemp(T);
}
Nandle psyTsatFnHPb(Nandle const H, Nandle const Pb) {
    // enthalpy {J/kg}
    // barometric pressure {Pascals}
    return EnergyPlus::Psychrometrics::PsyTsatFnHPb(H, Pb);
}
Nandle psyRhovFnTdbRh(Nandle const Tdb, Nandle const RH) {
    // dry-bulb temperature {C}
    // relative humidity value (0.0-1.0)
    return EnergyPlus::Psychrometrics::PsyRhovFnTdbRh(Tdb, RH);
}
Nandle psyRhFnTdbRhov(Nandle const Tdb, Nandle const Rhovapor) {
    // dry-bulb temperature {C}
    // vapor density in air {kg/m3}
    return EnergyPlus::Psychrometrics::PsyRhFnTdbRhov(Tdb, Rhovapor);
}
Nandle psyRhFnTdbWPb(Nandle const TDB, Nandle const dW, Nandle const PB) {
    // dry-bulb temperature {C}
    // humidity ratio
    // barometric pressure {Pascals}
    return EnergyPlus::Psychrometrics::PsyRhFnTdbWPb(TDB, dW, PB);
}
Nandle psyWFnTdpPb(Nandle const TDP, Nandle const PB) {
    // dew-point temperature {C}
    // barometric pressure {Pascals}
    return EnergyPlus::Psychrometrics::PsyWFnTdpPb(TDP, PB);
}
Nandle psyWFnTdbRhPb(Nandle const TDB, Nandle const RH, Nandle const PB) {
    // dry-bulb temperature {C}
    // relative humidity value (0.0-1.0)
    // barometric pressure {Pascals}
    return EnergyPlus::Psychrometrics::PsyWFnTdbRhPb(TDB, RH, PB);
}
Nandle psyWFnTdbTwbPb(Nandle const TDB, Nandle const TWBin, Nandle const PB) {
    // dry-bulb temperature {C}
    // wet-bulb temperature {C}
    // barometric pressure {Pascals}
    return EnergyPlus::Psychrometrics::PsyWFnTdbTwbPb(TDB, TWBin, PB);
}
Nandle psyHFnTdbRhPb(Nandle const TDB, Nandle const RH, Nandle const PB) {
    // dry-bulb temperature {C}
    // relative humidity value (0.0 - 1.0)
    // barometric pressure (N/M**2) {Pascals}
    return EnergyPlus::Psychrometrics::PsyHFnTdbRhPb(TDB, RH, PB);
}
Nandle psyTdpFnWPb(Nandle const W, Nandle const PB) {
    // humidity ratio
    // barometric pressure (N/M**2) {Pascals}
    return EnergyPlus::Psychrometrics::PsyTdpFnWPb(W, PB);
}
Nandle psyTdpFnTdbTwbPb(Nandle const TDB, Nandle const TWB, Nandle const PB) {
    // dry-bulb temperature {C}
    // wet-bulb temperature {C}
    // barometric pressure (N/M**2) {Pascals}
    return EnergyPlus::Psychrometrics::PsyTdpFnTdbTwbPb(TDB, TWB, PB);
}
