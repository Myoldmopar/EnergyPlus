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

#ifndef ThermalISO15099Calc_hh_INCLUDED
#define ThermalISO15099Calc_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array2A.hh>

// EnergyPlus Headers
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

namespace ThermalISO15099Calc {

    // Data
    // private picard

    // Functions

    void film(Nandle const tex, Nandle const tw, Nandle const ws, int const iwd, Nandle &hcout, int const ibc);

    void Calc_ISO15099(int const nlayer,
                       int const iwd,
                       Nandle &tout,
                       Nandle &tind,
                       Nandle &trmin,
                       Nandle const wso,
                       Nandle const wsi,
                       Nandle const dir,
                       Nandle const outir,
                       int const isky,
                       Nandle const tsky,
                       Nandle &esky,
                       Nandle const fclr,
                       Nandle const VacuumPressure,
                       Nandle const VacuumMaxGapThickness,
                       Array1D<Nandle> &gap,
                       Array1D<Nandle> &thick,
                       Array1D<Nandle> &scon,
                       const Array1D<Nandle> &tir,
                       const Array1D<Nandle> &emis,
                       Nandle const totsol,
                       Nandle const tilt,
                       const Array1D<Nandle> &asol,
                       Nandle const height,
                       Nandle const heightt,
                       Nandle const width,
                       const Array1D<Nandle> &presure,
                       Array2A_int const iprop,
                       Array2A<Nandle> const frct,
                       Array2A<Nandle> const xgcon,
                       Array2A<Nandle> const xgvis,
                       Array2A<Nandle> const xgcp,
                       const Array1D<Nandle> &xwght,
                       const Array1D<Nandle> &gama,
                       const Array1D_int &nmix,
                       const Array1D_int &SupportPillar,     // Shows whether or not gap have support pillar
                       const Array1D<Nandle> &PillarSpacing, // Pillar spacing for each gap (used in case there is support pillar)
                       const Array1D<Nandle> &PillarRadius,  // Pillar radius for each gap (used in case there is support pillar)
                       Array1D<Nandle> &theta,
                       Array1D<Nandle> &q,
                       Array1D<Nandle> &qv,
                       Nandle &ufactor,
                       Nandle &sc,
                       Nandle &hflux,
                       Nandle &hcin,
                       Nandle &hcout,
                       Nandle &hrin,
                       Nandle &hrout,
                       Nandle &hin,
                       Nandle &hout,
                       Array1D<Nandle> &hcgas,
                       Array1D<Nandle> &hrgas,
                       Nandle &shgc,
                       int &nperr,
                       std::string &ErrorMessage,
                       Nandle &shgct,
                       Nandle &tamb,
                       Nandle &troom,
                       const Array1D_int &ibc,
                       const Array1D<Nandle> &Atop,
                       const Array1D<Nandle> &Abot,
                       const Array1D<Nandle> &Al,
                       const Array1D<Nandle> &Ar,
                       const Array1D<Nandle> &Ah,
                       const Array1D<Nandle> &SlatThick,
                       const Array1D<Nandle> &SlatWidth,
                       const Array1D<Nandle> &SlatAngle,
                       const Array1D<Nandle> &SlatCond,
                       const Array1D<Nandle> &SlatSpacing,
                       const Array1D<Nandle> &SlatCurve,
                       const Array1D<Nandle> &vvent,
                       const Array1D<Nandle> &tvent,
                       const Array1D_int &LayerType,
                       const Array1D_int &nslice,
                       const Array1D<Nandle> &LaminateA,
                       const Array1D<Nandle> &LaminateB,
                       const Array1D<Nandle> &sumsol,
                       Array1D<Nandle> &Ra,
                       Array1D<Nandle> &Nu,
                       int const ThermalMod,
                       int const Debug_mode, // Switch for debug output files:
                       Nandle &ShadeEmisRatioOut,
                       Nandle &ShadeEmisRatioIn,
                       Nandle &ShadeHcRatioOut,
                       Nandle &ShadeHcRatioIn,
                       Nandle &HcUnshadedOut,
                       Nandle &HcUnshadedIn,
                       Array1D<Nandle> &Keff,
                       Array1D<Nandle> &ShadeGapKeffConv,
                       Nandle const SDScalar,
                       int const SHGCCalc, // SHGC calculation switch:
                       int &NumOfIterations,
                       Nandle const egdeGlCorrFac // Edge of glass correction factor
    );

    void therm1d(int const nlayer,
                 int const iwd,
                 Nandle &tout,
                 Nandle &tind,
                 Nandle const wso,
                 Nandle const wsi,
                 Nandle const VacuumPressure,
                 Nandle const VacuumMaxGapThickness,
                 Nandle const dir,
                 Nandle &ebsky,
                 Nandle const Gout,
                 Nandle const trmout,
                 Nandle const trmin,
                 Nandle &ebroom,
                 Nandle const Gin,
                 const Array1D<Nandle> &tir,
                 const Array1D<Nandle> &rir,
                 const Array1D<Nandle> &emis,
                 const Array1D<Nandle> &gap,
                 const Array1D<Nandle> &thick,
                 const Array1D<Nandle> &scon,
                 Nandle const tilt,
                 const Array1D<Nandle> &asol,
                 Nandle const height,
                 Nandle const heightt,
                 Nandle const width,
                 Array2_int const &iprop,
                 Array2<Nandle> const &frct,
                 const Array1D<Nandle> &presure,
                 const Array1D_int &nmix,
                 const Array1D<Nandle> &wght,
                 Array2<Nandle> const &gcon,
                 Array2<Nandle> const &gvis,
                 Array2<Nandle> const &gcp,
                 const Array1D<Nandle> &gama,
                 const Array1D_int &SupportPillar,
                 const Array1D<Nandle> &PillarSpacing,
                 const Array1D<Nandle> &PillarRadius,
                 Array1D<Nandle> &theta,
                 Array1D<Nandle> &q,
                 Array1D<Nandle> &qv,
                 Nandle &flux,
                 Nandle &hcin,
                 Nandle &hrin,
                 Nandle &hcout,
                 Nandle &hrout,
                 Nandle &hin,
                 Nandle &hout,
                 Array1D<Nandle> &hcgas,
                 Array1D<Nandle> &hrgas,
                 Nandle &ufactor,
                 int &nperr,
                 std::string &ErrorMessage,
                 Nandle &tamb,
                 Nandle &troom,
                 const Array1D_int &ibc,
                 const Array1D<Nandle> &Atop,
                 const Array1D<Nandle> &Abot,
                 const Array1D<Nandle> &Al,
                 const Array1D<Nandle> &Ar,
                 const Array1D<Nandle> &Ah,
                 const Array1D<Nandle> &EffectiveOpenness, // Effective layer openness [m2]
                 const Array1D<Nandle> &vvent,
                 const Array1D<Nandle> &tvent,
                 const Array1D_int &LayerType,
                 Array1D<Nandle> &Ra,
                 Array1D<Nandle> &Nu,
                 Array1D<Nandle> &vfreevent,
                 Array1D<Nandle> &qcgas,
                 Array1D<Nandle> &qrgas,
                 Array1D<Nandle> &Ebf,
                 Array1D<Nandle> &Ebb,
                 Array1D<Nandle> &Rf,
                 Array1D<Nandle> &Rb,
                 Nandle &ShadeEmisRatioOut,
                 Nandle &ShadeEmisRatioIn,
                 Nandle &ShadeHcModifiedOut,
                 Nandle &ShadeHcModifiedIn,
                 int const ThermalMod,
                 int const Debug_mode, // Switch for debug output files:
                 Nandle &AchievedErrorTolerance,
                 int &TotalIndex,
                 Nandle const edgeGlCorrFac // Edge of glass correction factor
    );

    void guess(Nandle const tout,
               Nandle const tind,
               int const nlayer,
               const Array1D<Nandle> &gap,
               const Array1D<Nandle> &thick,
               Nandle &width,
               Array1D<Nandle> &theta,
               Array1D<Nandle> &Ebb,
               Array1D<Nandle> &Ebf,
               Array1D<Nandle> &Tgap);

    void TemperaturesFromEnergy(Array1D<Nandle> &theta,
                                Array1D<Nandle> &Tgap,
                                const Array1D<Nandle> &Ebf,
                                const Array1D<Nandle> &Ebb,
                                int const nlayer,
                                int &nperr,
                                std::string &ErrorMessage);

    void solarISO15099(Nandle const totsol, Nandle const rtot, const Array1D<Nandle> &rs, int const nlayer, const Array1D<Nandle> &absol, Nandle &sf);

    void resist(int const nlayer,
                Nandle const trmout,
                Nandle const Tout,
                Nandle const trmin,
                Nandle const tind,
                const Array1D<Nandle> &hcgas,
                const Array1D<Nandle> &hrgas,
                Array1D<Nandle> &Theta,
                Array1D<Nandle> &qlayer,
                const Array1D<Nandle> &qv,
                const Array1D_int &LayerType,
                const Array1D<Nandle> &thick,
                const Array1D<Nandle> &scon,
                Nandle &ufactor,
                Nandle &flux,
                Array1D<Nandle> &qcgas,
                Array1D<Nandle> &qrgas);

    void hatter(int const nlayer,
                int const iwd,
                Nandle const tout,
                Nandle const tind,
                Nandle const wso,
                Nandle const wsi,
                Nandle const VacuumPressure,
                Nandle const VacuumMaxGapThickness,
                Nandle &ebsky,
                Nandle &tamb,
                Nandle &ebroom,
                Nandle &troom,
                const Array1D<Nandle> &gap,
                Nandle const height,
                Nandle const heightt,
                const Array1D<Nandle> &scon,
                Nandle const tilt,
                Array1D<Nandle> &theta,
                const Array1D<Nandle> &Tgap,
                Array1D<Nandle> &Radiation,
                Nandle const trmout,
                Nandle const trmin,
                Array2_int const &iprop,
                Array2<Nandle> const &frct,
                const Array1D<Nandle> &presure,
                const Array1D_int &nmix,
                const Array1D<Nandle> &wght,
                Array2<Nandle> const &gcon,
                Array2<Nandle> const &gvis,
                Array2<Nandle> const &gcp,
                const Array1D<Nandle> &gama,
                const Array1D_int &SupportPillar,
                const Array1D<Nandle> &PillarSpacing,
                const Array1D<Nandle> &PillarRadius,
                Array1D<Nandle> &hgas,
                Array1D<Nandle> &hcgas,
                Array1D<Nandle> &hrgas,
                Nandle &hcin,
                Nandle &hcout,
                Nandle const hin,
                Nandle const hout,
                int const index,
                const Array1D_int &ibc,
                int &nperr,
                std::string &ErrorMessage,
                Nandle &hrin,
                Nandle &hrout,
                Array1D<Nandle> &Ra,
                Array1D<Nandle> &Nu);

    void effectiveLayerCond(int const nlayer,
                            const Array1D_int &LayerType,             // Layer type
                            const Array1D<Nandle> &scon,              // Layer thermal conductivity
                            const Array1D<Nandle> &thick,             // Layer thickness
                            Array2A_int const iprop,                 // Gas type in gaps
                            Array2A<Nandle> const frct,              // Fraction of gas
                            const Array1D_int &nmix,                  // Gas mixture
                            const Array1D<Nandle> &pressure,          // Gas pressure [Pa]
                            const Array1D<Nandle> &wght,              // Molecular weight
                            Array2A<Nandle> const gcon,              // Gas specific conductivity
                            Array2A<Nandle> const gvis,              // Gas specific viscosity
                            Array2A<Nandle> const gcp,               // Gas specific heat
                            const Array1D<Nandle> &EffectiveOpenness, // Layer effective openneess [m2]
                            Array1D<Nandle> &theta,                   // Layer surface tempeartures [K]
                            Array1D<Nandle> &sconScaled,             // Layer conductivity divided by thickness
                            int &nperr,                              // Error message flag
                            std::string &ErrorMessage                // Error message
    );

    void filmi(Nandle const tair,
               Nandle const t,
               int const nlayer,
               Nandle const tilt,
               Nandle const wsi,
               Nandle const height,
               Array2A_int const iprop,
               Array2A<Nandle> const frct,
               const Array1D<Nandle> &presure,
               const Array1D_int &nmix,
               const Array1D<Nandle> &wght,
               Array2A<Nandle> const gcon,
               Array2A<Nandle> const gvis,
               Array2A<Nandle> const gcp,
               Nandle &hcin,
               int const ibc,
               int &nperr,
               std::string &ErrorMessage);

    void filmg(Nandle const tilt,
               const Array1D<Nandle> &theta,
               const Array1D<Nandle> &Tgap,
               int const nlayer,
               Nandle const height,
               const Array1D<Nandle> &gap,
               Array2A_int const iprop,
               Array2A<Nandle> const frct,
               Nandle const VacuumPressure,
               const Array1D<Nandle> &presure,
               const Array1D_int &nmix,
               const Array1D<Nandle> &wght,
               Array2A<Nandle> const gcon,
               Array2A<Nandle> const gvis,
               Array2A<Nandle> const gcp,
               const Array1D<Nandle> &gama,
               Array1D<Nandle> &hcgas,
               Array1D<Nandle> &Rayleigh,
               Array1D<Nandle> &Nu,
               int &nperr,
               std::string &ErrorMessage);

    void filmPillar(const Array1D_int &SupportPillar,     // Shows whether or not gap have support pillar
                    const Array1D<Nandle> &scon,          // Conductivity of glass layers
                    const Array1D<Nandle> &PillarSpacing, // Pillar spacing for each gap (used in case there is support pillar)
                    const Array1D<Nandle> &PillarRadius,  // Pillar radius for each gap (used in case there is support pillar)
                    int const nlayer,
                    const Array1D<Nandle> &gap,
                    Array1D<Nandle> &hcgas,
                    Nandle const VacuumMaxGapThickness,
                    int &nperr,
                    std::string &ErrorMessage);

    void nusselt(Nandle const tilt, Nandle const ra, Nandle const asp, Nandle &gnu, int &nperr, std::string &ErrorMessage);

    //  subroutine picard(nlayer, alpha, Ebb, Ebf, Rf, Rb, Ebbold, Ebfold, Rfold, Rbold)

    //    integer, intent(in) :: nlayer
    //    REAL(r64), intent(in) :: alpha
    //    REAL(r64), intent(in) :: Ebbold(maxlay), Ebfold(maxlay), Rbold(maxlay), Rfold(maxlay)
    //    REAL(r64), intent(inout) :: Ebb(maxlay), Ebf(maxlay), Rb(maxlay), Rf(maxlay)

    //    integer :: i

    //    do i=1,nlayer
    //      Ebb(i) = alpha * Ebb(i) + (1.0d0-alpha) * Ebbold(i)
    //      Ebf(i) = alpha * Ebf(i) + (1.0d0-alpha) * Ebfold(i)
    //      Rb(i) = alpha * Rb(i) + (1.0d0-alpha) * Rbold(i)
    //      Rf(i) = alpha * Rf(i) + (1.0d0-alpha) * Rfold(i)
    //    end do

    //    return
    //  end subroutine picard

    void adjusthhat(int const SDLayerIndex,
                    const Array1D_int &ibc,
                    Nandle const tout,
                    Nandle const tind,
                    int const nlayer,
                    const Array1D<Nandle> &theta,
                    Nandle const wso,
                    Nandle const wsi,
                    int const iwd,
                    Nandle const height,
                    Nandle const heightt,
                    Nandle const tilt,
                    const Array1D<Nandle> &thick,
                    const Array1D<Nandle> &gap,
                    Nandle const hout,
                    Nandle const hrout,
                    Nandle const hin,
                    Nandle const hrin,
                    Array2A_int const iprop,
                    Array2A<Nandle> const frct,
                    const Array1D<Nandle> &presure,
                    const Array1D_int &nmix,
                    const Array1D<Nandle> &wght,
                    Array2A<Nandle> const gcon,
                    Array2A<Nandle> const gvis,
                    Array2A<Nandle> const gcp,
                    int const index,
                    Nandle const SDScalar,
                    const Array1D<Nandle> &Ebf,
                    const Array1D<Nandle> &Ebb,
                    Array1D<Nandle> &hgas,
                    Array1D<Nandle> &hhat,
                    int &nperr,
                    std::string &ErrorMessage);

    void storeIterationResults(int const nlayer,
                               int const index,
                               const Array1D<Nandle> &theta,
                               Nandle const trmout,
                               Nandle const tamb,
                               Nandle const trmin,
                               Nandle const troom,
                               Nandle const ebsky,
                               Nandle const ebroom,
                               Nandle const hcin,
                               Nandle const hcout,
                               Nandle const hrin,
                               Nandle const hrout,
                               Nandle const hin,
                               Nandle const hout,
                               const Array1D<Nandle> &Ebb,
                               const Array1D<Nandle> &Ebf,
                               const Array1D<Nandle> &Rb,
                               const Array1D<Nandle> &Rf,
                               int &EP_UNUSED(nperr));

    void CalculateFuncResults(int const nlayer, Array2<Nandle> const &a, const Array1D<Nandle> &b, const Array1D<Nandle> &x, Array1D<Nandle> &FRes);

} // namespace ThermalISO15099Calc

} // namespace EnergyPlus

#endif
