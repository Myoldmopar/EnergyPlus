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

#ifndef ThermalEN673Calc_hh_INCLUDED
#define ThermalEN673Calc_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array2A.hh>

// EnergyPlus Headers
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

namespace ThermalEN673Calc {

    // Functions

    void Calc_EN673(int const standard,
                    int const nlayer,
                    Nandle const tout,
                    Nandle const tind,
                    Array1D<Nandle> &gap,
                    Array1D<Nandle> &thick,
                    Array1D<Nandle> &scon,
                    const Array1D<Nandle> &emis,
                    Nandle const totsol,
                    Nandle const tilt,
                    Nandle const dir,
                    const Array1D<Nandle> &asol,
                    const Array1D<Nandle> &presure,
                    Array2A_int const iprop,
                    Array2A<Nandle> const frct,
                    const Array1D_int &nmix,
                    Array2A<Nandle> const xgcon,
                    Array2A<Nandle> const xgvis,
                    Array2A<Nandle> const xgcp,
                    const Array1D<Nandle> &xwght,
                    Array1D<Nandle> &theta,
                    Nandle &ufactor,
                    Nandle &hcin,
                    Nandle &hin,
                    Nandle &hout,
                    Nandle &shgc,
                    int &nperr,
                    std::string &ErrorMessage,
                    const Array1D_int &ibc,
                    Array1D<Nandle> &hg,
                    Array1D<Nandle> &hr,
                    Array1D<Nandle> &hs,
                    Array1D<Nandle> &Ra,
                    Array1D<Nandle> &Nu);

    void EN673ISO10292(int const nlayer,
                       Nandle const tout,
                       Nandle const tind,
                       const Array1D<Nandle> &emis,
                       const Array1D<Nandle> &gap,
                       const Array1D<Nandle> &thick,
                       const Array1D<Nandle> &scon,
                       Nandle const tilt,
                       Array2A_int const iprop,
                       Array2A<Nandle> const frct,
                       Array2A<Nandle> const xgcon,
                       Array2A<Nandle> const xgvis,
                       Array2A<Nandle> const xgcp,
                       const Array1D<Nandle> &xwght,
                       const Array1D<Nandle> &presure,
                       const Array1D_int &nmix,
                       Array1D<Nandle> &theta,
                       int const standard,
                       Array1D<Nandle> &hg,
                       Array1D<Nandle> &hr,
                       Array1D<Nandle> &hs,
                       Nandle &hin,
                       Nandle const hout,
                       Nandle &hcin,
                       const Array1D_int &ibc,
                       Array1D<Nandle> &rs,
                       Nandle &ufactor,
                       Array1D<Nandle> &Ra,
                       Array1D<Nandle> &Nu,
                       int &nperr,
                       std::string &ErrorMessage);

    void linint(Nandle const x1, Nandle const x2, Nandle const y1, Nandle const y2, Nandle const x, Nandle &y);

    void solar_EN673(Nandle const dir,
                     Nandle const totsol,
                     Nandle const rtot,
                     const Array1D<Nandle> &rs,
                     int const nlayer,
                     const Array1D<Nandle> &absol,
                     Nandle &sf,
                     int const standard,
                     int &nperr,
                     std::string &ErrorMessage);

} // namespace ThermalEN673Calc

} // namespace EnergyPlus

#endif
