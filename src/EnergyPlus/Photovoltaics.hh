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

#ifndef Photovoltaics_hh_INCLUDED
#define Photovoltaics_hh_INCLUDED

// C++ Headers
#include <functional>

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>

// EnergyPlus Headers
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

namespace Photovoltaics {

    // Data
    // MODULE PARAMETER DEFINITIONS:
    // na

    // DERIVED TYPE DEFINITIONS:
    //   see DataPhotovoltaics.cc

    extern Array1D_bool CheckEquipName;

    // SUBROUTINE SPECIFICATIONS FOR MODULE Photovoltaics

    // The following subroutines are used for the SIMPLE model

    // The following subroutines and functions are used for only the EQUIVALENT ONE-DIODE model

    // The following subroutines and functions are used for the Sandia model.

    //  OO get set methods for coupling to exterior vented baffle cavity mounting configurations

    // Functions

    void SimPVGenerator(int const GeneratorType,          // type of Generator !unused1208
                        std::string const &GeneratorName, // user specified name of Generator
                        int &GeneratorIndex,
                        bool const RunFlag, // is PV ON or OFF as determined by schedules in ElecLoadCenter
                        Nandle const PVLoad // electrical load on the PV (not really used... PV models assume "full on" !unused1208
    );

    void GetPVGeneratorResults(int const GeneratorType, // type of Generator !unused1208
                               int const GeneratorIndex,
                               Nandle &GeneratorPower,  // electrical power
                               Nandle &GeneratorEnergy, // electrical energy
                               Nandle &ThermalPower,
                               Nandle &ThermalEnergy);

    // *************

    void GetPVInput();

    int GetPVZone(int const SurfNum);

    // **************************************

    void CalcSimplePV(int const thisPV,
                      bool const RunFlag // unused1208
    );

    void ReportPV(int const PVnum);

    // *************

    void CalcSandiaPV(int const PVnum,   // ptr to current PV system
                      bool const RunFlag // controls if generator is scheduled *ON*
    );

    // ********************
    // begin routines for Equivalent one-diode model by Bradley/Ulleberg

    void InitTRNSYSPV(int const PVnum); // the number of the GENERATOR:PHOTOVOLTAICS (passed in)

    // *************

    void CalcTRNSYSPV(int const PVnum,   // BTG added intent
                      bool const RunFlag // BTG added intent    !flag tells whether the PV is ON or OFF
    );

    void POWER(Nandle const IO,   // passed in from CalcPV
               Nandle const IL,   // passed in from CalcPV
               Nandle const RSER, // passed in from CalcPV
               Nandle const AA,   // passed in from CalcPV
               Nandle const EPS,  // passed in from CalcPV
               Nandle &II,        // current [A]
               Nandle &VV,        // voltage [V]
               Nandle &PP         // power [W]
    );

    void NEWTON(Nandle &XX,
                std::function<Nandle(Nandle const, Nandle const, Nandle const, Nandle const, Nandle const, Nandle const)> FXX,
                std::function<Nandle(Nandle const, Nandle const, Nandle const, Nandle const, Nandle const)> DER,
                Nandle const &II, // Autodesk Aliased to XX in some calls
                Nandle const &VV, // Autodesk Aliased to XX in some calls
                Nandle const IO,
                Nandle const IL,
                Nandle const RSER,
                Nandle const AA,
                Nandle const XS,
                Nandle const EPS);

    void SEARCH(Nandle &A, Nandle &B, Nandle &P, int &K, Nandle &IO, Nandle &IL, Nandle &RSER, Nandle &AA, Nandle const EPS, int const KMAX);

    Nandle FUN(Nandle const II, Nandle const VV, Nandle const IL, Nandle const IO, Nandle const RSER, Nandle const AA);

    Nandle FI(Nandle const II, Nandle const VV, Nandle const IO, Nandle const RSER, Nandle const AA);

    Nandle FV(Nandle const II, Nandle const VV, Nandle const IO, Nandle const RSER, Nandle const AA);

    // End routines for Equivalent One-Diode model as implemented by Bradley
    //************************************************************************

    // Begin supporting routines for Sandia PV model
    // -------------------------------------------------------------------------------

    Nandle SandiaModuleTemperature(Nandle const Ibc, // beam radiation on collector plane, W/m2
                                   Nandle const Idc, // Diffuse radiation on collector plane, W/m2
                                   Nandle const Ws,  // wind speed, m/s
                                   Nandle const Ta,  // ambient temperature, degC
                                   Nandle const fd,  // fraction of Idc used (empirical constant)
                                   Nandle const a,   // empirical constant
                                   Nandle const b    // empirical constant
    );

    // -------------------------------------------------------------------------------
    // -------------------------------------------------------------------------------

    Nandle SandiaTcellFromTmodule(Nandle const Tm,  // module temperature (deg C)
                                  Nandle const Ibc, // beam radiation on collector plane, W/m2
                                  Nandle const Idc, // Diffuse radiation on collector plane, W/m2
                                  Nandle const fd,  // fraction of Idc used (empirical constant)
                                  Nandle const DT0  // (Tc-Tm) at E=1000 W/m2 (empirical constant known as delta T), deg C
    );

    // -------------------------------------------------------------------------------

    Nandle SandiaCellTemperature(Nandle const Ibc, // beam radiation on collector plane W/m2
                                 Nandle const Idc, // Diffuse radiation on collector plane W/m2
                                 Nandle const Ws,  // wind speed, m/s
                                 Nandle const Ta,  // ambient temperature, degC
                                 Nandle const fd,  // fraction of Idc used (empirical constant)
                                 Nandle const a,   // empirical constant
                                 Nandle const b,   // empirical constant
                                 Nandle const DT0  // (Tc-Tm) at E=1000 W/m2 (empirical constant known as dTc), deg C
    );

    // -------------------------------------------------------------------------------

    Nandle SandiaEffectiveIrradiance(Nandle const Tc,   // cell temperature (deg C)
                                     Nandle const Isc,  // short-circuit current under operating conditions (A)
                                     Nandle const Isc0, // reference Isc at Tc=25 C, Ic=1000 W/m2 (A)
                                     Nandle const aIsc  // Isc temperature coefficient (degC^-1)
    );

    // -------------------------------------------------------------------------------

    Nandle AbsoluteAirMass(Nandle const SolZen,  // solar zenith angle (deg)
                           Nandle const Altitude // site altitude (m)
    );

    // -------------------------------------------------------------------------------

    Nandle SandiaF1(Nandle const AMa, // absolute air mass
                    Nandle const a0,  // empirical constant, module-specific
                    Nandle const a1,  // empirical constant, module-specific
                    Nandle const a2,  // empirical constant, module-specific
                    Nandle const a3,  // empirical constant, module-specific
                    Nandle const a4   // empirical constant, module-specific
    );

    // -------------------------------------------------------------------------------

    Nandle SandiaF2(Nandle const IncAng, // incidence angle (deg)
                    Nandle const b0,     // empirical module-specific constants
                    Nandle const b1,     // empirical module-specific constants
                    Nandle const b2,     // empirical module-specific constants
                    Nandle const b3,     // empirical module-specific constants
                    Nandle const b4,     // empirical module-specific constants
                    Nandle const b5      // empirical module-specific constants
    );

    // -------------------------------------------------------------------------------

    Nandle SandiaImp(Nandle const Tc,   // cell temperature (degC)
                     Nandle const Ee,   // effective irradiance (W/m2)
                     Nandle const Imp0, // current at MPP at SRC (1000 W/m2, 25 C) (A)
                     Nandle const aImp, // Imp temperature coefficient (degC^-1)
                     Nandle const C0,   // empirical module-specific constants
                     Nandle const C1    // empirical module-specific constants
    );

    // -------------------------------------------------------------------------------

    Nandle SandiaIsc(Nandle const Tc,   // cell temperature (deg C)
                     Nandle const Isc0, // Isc at Tc=25 C, Ic=1000 W/m2 (A)
                     Nandle const Ibc,  // beam radiation on collector plane (W/m2)
                     Nandle const Idc,  // Diffuse radiation on collector plane (W/m2)
                     Nandle const F1,   // Sandia F1 function for air mass effects
                     Nandle const F2,   // Sandia F2 function of incidence angle
                     Nandle const fd,   // module-specific empirical constant
                     Nandle const aIsc  // Isc temperature coefficient (degC^-1)
    );

    // -------------------------------------------------------------------------------

    Nandle SandiaIx(Nandle const Tc,   // cell temperature (deg C)
                    Nandle const Ee,   // effective irradiance
                    Nandle const Ix0,  // Ix at SRC (1000 W/m2, 25 C) (A)
                    Nandle const aIsc, // Isc temp coefficient (/C)
                    Nandle const aImp, // Imp temp coefficient (/C)
                    Nandle const C4,   // empirical module-specific constants
                    Nandle const C5    // empirical module-specific constants
    );

    // -------------------------------------------------------------------------------

    Nandle SandiaIxx(Nandle const Tc,   // cell temperature (deg C)
                     Nandle const Ee,   // effective irradiance (W/m2 ?)
                     Nandle const Ixx0, // Ixx at SRC (1000 W/m2, 25 C) (A)
                     Nandle const aImp, // Imp temp coefficient (/C)
                     Nandle const C6,   // empirical module-specific constants
                     Nandle const C7    // empirical module-specific constants
    );

    // -------------------------------------------------------------------------------

    Nandle SandiaVmp(Nandle const Tc,          // cell temperature (deg C)
                     Nandle const Ee,          // effective irradiance
                     Nandle const Vmp0,        // Vmp at SRC (1000 W/m2, 25 C) (V)
                     Nandle const NcellSer,    // # cells in series
                     Nandle const DiodeFactor, // module-specIFic empirical constant
                     Nandle const BVmp0,       // Vmp temperature coefficient (V/C)
                     Nandle const mBVmp,       // change in BVmp with irradiance
                     Nandle const C2,          // empirical module-specific constants
                     Nandle const C3           // empirical module-specific constants
    );

    // -------------------------------------------------------------------------------

    Nandle SandiaVoc(Nandle const Tc,          // cell temperature (deg C)
                     Nandle const Ee,          // effective irradiance
                     Nandle const Voc0,        // Voc at SRC (1000 W/m2, 25 C) (V)
                     Nandle const NcellSer,    // # cells in series
                     Nandle const DiodeFactor, // module-specIFic empirical constant
                     Nandle const BVoc0,       // Voc temperature coefficient (V/C)
                     Nandle const mBVoc        // change in BVoc with irradiance
    );

    void SetVentedModuleQdotSource(int const VentModNum,
                                   Nandle const QSource // source term in Watts
    );

    void GetExtVentedCavityIndex(int const SurfacePtr, int &VentCavIndex);

    void GetExtVentedCavityTsColl(int const VentModNum, Nandle &TsColl);

    // -------------------------------------------------------------------------------

    //     EnergyPlus V1.2 and beyond include models for photovoltaic calculations called
    //     Generator:Photovoltaic:Simple and Generator:PV:Sandia implemented by the Center for
    //     Buildings and Thermal Systems, National Renewable Energy Laboratory, 1617 Cole Blvd
    //     MS 2722, Golden, CO, 80401

    //     EnergyPlus v1.1.1 and beyond includes model for Photovoltaic calculations, now
    //     referred to as the Generator:PV:Equivalent One-Diode model developed by Thermal Energy
    //     System Specialists, 2916 Marketplace Drive, Suite 104, Madison, WI 53719;
    //     Tel: (608) 274-2577

} // namespace Photovoltaics

} // namespace EnergyPlus

#endif
