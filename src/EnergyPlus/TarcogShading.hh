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

#ifndef TarcogShading_hh_INCLUDED
#define TarcogShading_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array2A.hh>

// EnergyPlus Headers
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

namespace TarcogShading {

    // Functions

    void shading(Array1D<Nandle> const &theta,
                 Array1D<Nandle> const &gap,
                 Array1D<Nandle> &hgas,
                 Array1D<Nandle> &hcgas,
                 Array1D<Nandle> &hrgas,
                 Array2<Nandle> const &frct,
                 Array2_int const &iprop,
                 Array1D<Nandle> const &pressure,
                 Array1D_int const &nmix,
                 const Array1D<Nandle> &xwght,
                 Array2<Nandle> const &xgcon,
                 Array2<Nandle> const &xgvis,
                 Array2<Nandle> const &xgcp,
                 int const nlayer,
                 Nandle const width,
                 Nandle const height,
                 Nandle const angle,
                 Nandle const Tout,
                 Nandle const Tin,
                 Array1D<Nandle> const &Atop,
                 Array1D<Nandle> const &Abot,
                 Array1D<Nandle> const &Al,
                 Array1D<Nandle> const &Ar,
                 Array1D<Nandle> const &Ah,
                 Array1D<Nandle> const &vvent,
                 Array1D<Nandle> const &tvent,
                 Array1D_int const &LayerType,
                 Array1D<Nandle> &Tgaps,
                 Array1D<Nandle> &qv,
                 Array1D<Nandle> &hcv,
                 int &nperr,
                 std::string &ErrorMessage,
                 Array1D<Nandle> &vfreevent);

    void forcedventilation(const Array1D_int &iprop,
                           const Array1D<Nandle> &frct,
                           Nandle const press,
                           int const nmix,
                           const Array1D<Nandle> &xwght,
                           Array2A<Nandle> const xgcon,
                           Array2A<Nandle> const xgvis,
                           Array2A<Nandle> const xgcp,
                           Nandle const s,
                           Nandle const H,
                           Nandle const hc,
                           Nandle const forcedspeed,
                           Nandle const Tinlet,
                           Nandle &Toutlet,
                           Nandle const Tav,
                           Nandle &hcv,
                           Nandle &qv,
                           int &nperr,
                           std::string &ErrorMessage);

    void shadingin(const Array1D_int &iprop1,
                   const Array1D<Nandle> &frct1,
                   Nandle const press1,
                   int const nmix1,
                   const Array1D_int &iprop2,
                   const Array1D<Nandle> &frct2,
                   Nandle const press2,
                   int const nmix2,
                   const Array1D<Nandle> &xwght,
                   Array2A<Nandle> const xgcon,
                   Array2A<Nandle> const xgvis,
                   Array2A<Nandle> const xgcp,
                   Nandle &Atop,
                   Nandle &Abot,
                   Nandle const Al,
                   Nandle const Ar,
                   Nandle const Ah,
                   Nandle const s1,
                   Nandle const s2,
                   Nandle const H,
                   Nandle const L,
                   Nandle const angle,
                   Nandle const hc1,
                   Nandle const hc2,
                   Nandle &speed1,
                   Nandle &speed2,
                   Nandle &Tgap1,
                   Nandle &Tgap2,
                   Nandle const Tav1,
                   Nandle const Tav2,
                   Nandle &hcv1,
                   Nandle &hcv2,
                   Nandle &qv1,
                   Nandle &qv2,
                   int &nperr,
                   std::string &ErrorMessage);

    void shadingedge(const Array1D_int &iprop1,
                     const Array1D<Nandle> &frct1,
                     Nandle const press1,
                     int const nmix1,
                     const Array1D_int &iprop2,
                     const Array1D<Nandle> &frct2,
                     Nandle const press2,
                     int const nmix2,
                     const Array1D<Nandle> &xwght,
                     Array2A<Nandle> const xgcon,
                     Array2A<Nandle> const xgvis,
                     Array2A<Nandle> const xgcp,
                     Nandle &Atop,
                     Nandle &Abot,
                     Nandle const Al,
                     Nandle const Ar,
                     Nandle &Ah,
                     Nandle const s,
                     Nandle const H,
                     Nandle const L,
                     Nandle const angle,
                     Nandle const forcedspeed,
                     Nandle const hc,
                     Nandle const Tenv,
                     Nandle const Tav,
                     Nandle &Tgap,
                     Nandle &hcv,
                     Nandle &qv,
                     int &nperr,
                     std::string &ErrorMessage,
                     Nandle &speed);

    void updateEffectiveMultipliers(int const nlayer,                // Number of layers
                                    Nandle const width,              // IGU width [m]
                                    Nandle const height,             // IGU height [m]
                                    const Array1D<Nandle> &Atop,     // Top openning area [m2]
                                    const Array1D<Nandle> &Abot,     // Bottom openning area [m2]
                                    const Array1D<Nandle> &Al,       // Left side openning area [m2]
                                    const Array1D<Nandle> &Ar,       // Right side openning area [m2]
                                    const Array1D<Nandle> &Ah,       // Front side openning area [m2]
                                    Array1D<Nandle> &Atop_eff,       // Output - Effective top openning area [m2]
                                    Array1D<Nandle> &Abot_eff,       // Output - Effective bottom openning area [m2]
                                    Array1D<Nandle> &Al_eff,         // Output - Effective left side openning area [m2]
                                    Array1D<Nandle> &Ar_eff,         // Output - Effective right side openning area [m2]
                                    Array1D<Nandle> &Ah_eff,         // Output - Effective front side openning area [m2]
                                    const Array1D_int &LayerType,    // Layer type
                                    const Array1D<Nandle> &SlatAngle // Venetian layer slat angle [deg]
    );

} // namespace TarcogShading

} // namespace EnergyPlus

#endif
