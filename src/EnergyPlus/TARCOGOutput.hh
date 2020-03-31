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

#ifndef TARCOGOutput_hh_INCLUDED
#define TARCOGOutput_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array2A.hh>

// EnergyPlus Headers
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

namespace TARCOGOutput {

    // Data
    // variables:
    // bi...Debug files handles:
    // character(len=1000) :: DebugDir
    extern std::string DBGD;
    extern std::string FileMode;
    extern std::string FilePosition;
    extern bool WriteDebugOutput;
    extern int DebugMode;
    extern int winID;
    extern int iguID;

    extern int InArgumentsFile;
    extern int OutArgumentsFile;
    extern int WINCogFile;

    // Intermediate debug files
    extern int IterationCSVFileNumber;
    extern int TarcogIterationsFileNumber;

    extern std::string IterationCSVName;

    // integer, parameter :: IterationHHAT = 102
    // character(len=1000)    :: IterationHHATName = 'IterationHHAT.csv'

    extern std::string WinCogFileName;
    // character(len=1000)    :: SHGCFileName = 'test.w7'
    extern std::string DebugOutputFileName;

    extern std::string const VersionNumber;
    extern std::string const VersionCompileDateCC;

    // Functions

    void WriteInputArguments(Nandle const tout,
                             Nandle const tind,
                             Nandle const trmin,
                             Nandle const wso,
                             int const iwd,
                             Nandle const wsi,
                             Nandle const dir,
                             Nandle const outir,
                             int const isky,
                             Nandle const tsky,
                             Nandle const esky,
                             Nandle const fclr,
                             Nandle const VacuumPressure,
                             Nandle const VacuumMaxGapThickness,
                             const Array1D_int &ibc,
                             Nandle const hout,
                             Nandle const hin,
                             int const standard,
                             int const ThermalMod,
                             Nandle const SDScalar,
                             Nandle const height,
                             Nandle const heightt,
                             Nandle const width,
                             Nandle const tilt,
                             Nandle const totsol,
                             int const nlayer,
                             const Array1D_int &LayerType,
                             const Array1D<Nandle> &thick,
                             const Array1D<Nandle> &scon,
                             const Array1D<Nandle> &asol,
                             const Array1D<Nandle> &tir,
                             const Array1D<Nandle> &emis,
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
                             const Array1D_int &nslice,
                             const Array1D<Nandle> &LaminateA,
                             const Array1D<Nandle> &LaminateB,
                             const Array1D<Nandle> &sumsol,
                             const Array1D<Nandle> &gap,
                             const Array1D<Nandle> &vvent,
                             const Array1D<Nandle> &tvent,
                             const Array1D<Nandle> &presure,
                             const Array1D_int &nmix,
                             Array2A_int const iprop,
                             Array2A<Nandle> const frct,
                             Array2A<Nandle> const xgcon,
                             Array2A<Nandle> const xgvis,
                             Array2A<Nandle> const xgcp,
                             const Array1D<Nandle> &xwght);

    void WriteModifiedArguments(int const InArgumentsFile,
                                std::string const &DBGD,
                                Nandle const esky,
                                Nandle const trmout,
                                Nandle const trmin,
                                Nandle const ebsky,
                                Nandle const ebroom,
                                Nandle const Gout,
                                Nandle const Gin,
                                int const nlayer,
                                const Array1D_int &LayerType,
                                const Array1D_int &nmix,
                                Array2A<Nandle> const frct,
                                const Array1D<Nandle> &thick,
                                const Array1D<Nandle> &scon,
                                const Array1D<Nandle> &gap,
                                Array2A<Nandle> const xgcon,
                                Array2A<Nandle> const xgvis,
                                Array2A<Nandle> const xgcp,
                                const Array1D<Nandle> &xwght);

    void WriteOutputArguments(int &OutArgumentsFile,
                              std::string const &DBGD,
                              int const nlayer,
                              Nandle const tamb,
                              const Array1D<Nandle> &q,
                              const Array1D<Nandle> &qv,
                              const Array1D<Nandle> &qcgas,
                              const Array1D<Nandle> &qrgas,
                              const Array1D<Nandle> &theta,
                              const Array1D<Nandle> &vfreevent,
                              const Array1D<Nandle> &vvent,
                              const Array1D<Nandle> &Keff,
                              const Array1D<Nandle> &ShadeGapKeffConv,
                              Nandle const troom,
                              Nandle const ufactor,
                              Nandle const shgc,
                              Nandle const sc,
                              Nandle const hflux,
                              Nandle const shgct,
                              Nandle const hcin,
                              Nandle const hrin,
                              Nandle const hcout,
                              Nandle const hrout,
                              const Array1D<Nandle> &Ra,
                              const Array1D<Nandle> &Nu,
                              const Array1D_int &LayerType,
                              const Array1D<Nandle> &Ebf,
                              const Array1D<Nandle> &Ebb,
                              const Array1D<Nandle> &Rf,
                              const Array1D<Nandle> &Rb,
                              Nandle const ebsky,
                              Nandle const Gout,
                              Nandle const ebroom,
                              Nandle const Gin,
                              Nandle const ShadeEmisRatioIn,
                              Nandle const ShadeEmisRatioOut,
                              Nandle const ShadeHcRatioIn,
                              Nandle const ShadeHcRatioOut,
                              Nandle const HcUnshadedIn,
                              Nandle const HcUnshadedOut,
                              const Array1D<Nandle> &hcgas,
                              const Array1D<Nandle> &hrgas,
                              Nandle const AchievedErrorTolerance,
                              int const NumOfIter);

    void WriteOutputEN673(int &OutArgumentsFile,
                          std::string const &DBGD,
                          int const nlayer,
                          Nandle const ufactor,
                          Nandle const hout,
                          Nandle const hin,
                          const Array1D<Nandle> &Ra,
                          const Array1D<Nandle> &Nu,
                          const Array1D<Nandle> &hg,
                          const Array1D<Nandle> &hr,
                          const Array1D<Nandle> &hs,
                          int &nperr);

    void WriteTARCOGInputFile(std::string const &VerNum,
                              Nandle const tout,
                              Nandle const tind,
                              Nandle const trmin,
                              Nandle const wso,
                              int const iwd,
                              Nandle const wsi,
                              Nandle const dir,
                              Nandle const outir,
                              int const isky,
                              Nandle const tsky,
                              Nandle const esky,
                              Nandle const fclr,
                              Nandle const VacuumPressure,
                              Nandle const VacuumMaxGapThickness,
                              int const CalcDeflection,
                              Nandle const Pa,
                              Nandle const Pini,
                              Nandle const Tini,
                              const Array1D_int &ibc,
                              Nandle const hout,
                              Nandle const hin,
                              int const standard,
                              int const ThermalMod,
                              Nandle const SDScalar,
                              Nandle const height,
                              Nandle const heightt,
                              Nandle const width,
                              Nandle const tilt,
                              Nandle const totsol,
                              int const nlayer,
                              const Array1D_int &LayerType,
                              const Array1D<Nandle> &thick,
                              const Array1D<Nandle> &scon,
                              const Array1D<Nandle> &YoungsMod,
                              const Array1D<Nandle> &PoissonsRat,
                              const Array1D<Nandle> &asol,
                              const Array1D<Nandle> &tir,
                              const Array1D<Nandle> &emis,
                              const Array1D<Nandle> &Atop,
                              const Array1D<Nandle> &Abot,
                              const Array1D<Nandle> &Al,
                              const Array1D<Nandle> &Ar,
                              const Array1D<Nandle> &Ah,
                              const Array1D_int &SupportPillar,     // Shows whether or not gap have support pillar
                              const Array1D<Nandle> &PillarSpacing, // Pillar spacing for each gap (used in case there is support pillar)
                              const Array1D<Nandle> &PillarRadius,  // Pillar radius for each gap (used in case there is support pillar)
                              const Array1D<Nandle> &SlatThick,
                              const Array1D<Nandle> &SlatWidth,
                              const Array1D<Nandle> &SlatAngle,
                              const Array1D<Nandle> &SlatCond,
                              const Array1D<Nandle> &SlatSpacing,
                              const Array1D<Nandle> &SlatCurve,
                              const Array1D_int &nslice,
                              const Array1D<Nandle> &gap,
                              const Array1D<Nandle> &GapDef,
                              const Array1D<Nandle> &vvent,
                              const Array1D<Nandle> &tvent,
                              const Array1D<Nandle> &presure,
                              const Array1D_int &nmix,
                              Array2A_int const iprop,
                              Array2A<Nandle> const frct,
                              Array2A<Nandle> const xgcon,
                              Array2A<Nandle> const xgvis,
                              Array2A<Nandle> const xgcp,
                              const Array1D<Nandle> &xwght,
                              const Array1D<Nandle> &gama);

    void FinishDebugOutputFiles(int const nperr);

    void PrepDebugFilesAndVariables(
        std::string const &Debug_dir, std::string const &Debug_file, int const Debug_mode, int const win_ID, int const igu_ID, int &nperr);

} // namespace TARCOGOutput

} // namespace EnergyPlus

#endif
