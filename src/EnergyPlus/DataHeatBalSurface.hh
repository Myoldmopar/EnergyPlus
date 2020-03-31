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

#ifndef DataHeatBalSurface_hh_INCLUDED
#define DataHeatBalSurface_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>
#include <ObjexxFCL/Array2D.hh>
#include <ObjexxFCL/Array3D.hh>

// EnergyPlus Headers
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

namespace DataHeatBalSurface {

    // Data
    // MODULE PARAMETER DEFINITIONS
    extern Nandle const MinSurfaceTempLimit;            // Lowest inside surface temperature allowed in Celsius
    extern Nandle const MinSurfaceTempLimitBeforeFatal; // 2.5 times MinSurfaceTempLimit
    extern Nandle const DefaultSurfaceTempLimit;        // Highest inside surface temperature allowed in Celsius
    extern std::vector<bool> Zone_has_mixed_HT_models;  // True if any surfaces in zone use CondFD, HAMT, or Kiva

    // DERIVED TYPE DEFINITIONS

    // MODULE VARIABLE DECLARATIONS:

    // SUBROUTINE SPECIFICATIONS FOR MODULE DataHeatBalSurface
    // Integer Variables for the Heat Balance Simulation
    extern Array1D_int SUMH; // From Old Bldctf.inc

    // Variables Dimensioned to Max Number of Heat Transfer Surfaces (maxhts)
    extern Nandle MaxSurfaceTempLimit;            // Highest inside surface temperature allowed in Celsius
    extern Nandle MaxSurfaceTempLimitBeforeFatal; // 2.5 times MaxSurfaceTempLimit
    extern Array1D<Nandle> CTFConstInPart;        // Constant Inside Portion of the CTF calculation
    extern Array1D<Nandle> CTFConstOutPart;       // Constant Outside Portion of the CTF calculation
    extern Array1D<Nandle> TempSurfIn;            // Temperature of the Inside Surface for each heat transfer surface
    extern Array1D<Nandle> TempSurfInTmp;         // Inside Surface Temperature Of Each Heat Transfer Surface
    extern Array1D<Nandle> HcExtSurf;             // Outside Convection Coefficient
    extern Array1D<Nandle> HAirExtSurf;           // Outside Convection Coefficient
    extern Array1D<Nandle> HSkyExtSurf;           // Outside Convection Coefficient
    extern Array1D<Nandle> HGrdExtSurf;           // Outside Convection Coefficient
    extern Array1D<Nandle> TempSource;            // Temperature at the source location for each heat transfer surface
    extern Array1D<Nandle> TempUserLoc;           // Temperature at the user specified location for each heat transfer surface
    extern Array1D<Nandle> TempSurfInRep;         // Temperature of the Inside Surface for each heat transfer surface
    extern Array1D<Nandle> TempSurfInMovInsRep;   // Temperature of interior movable insulation on the side facing the zone
    // (report)
    extern Array1D<Nandle> QConvInReport; // Surface convection heat gain at inside face [J]
    extern Array1D<Nandle> QdotConvInRep; // Surface convection heat transfer rate at inside face surface [W]
    // (report)
    extern Array1D<Nandle> QdotConvInRepPerArea; // Surface conv heat transfer rate per m2 at inside face surf
    //  (report){w/m2]

    // these next three all are for net IR thermal radiation exchange with other surfaces in the model.
    extern Array1D<Nandle> QRadNetSurfInReport;        // Surface thermal radiation heat gain at Inside face [J]
    extern Array1D<Nandle> QdotRadNetSurfInRep;        // Surface thermal radiation heat transfer inside face surface [W]
    extern Array1D<Nandle> QdotRadNetSurfInRepPerArea; // [W/m2]Surface thermal radiation heat transfer rate per m2 at
    //      Inside face surf
    // these next three all are for solar radiation gains on inside face
    extern Array1D<Nandle> QRadSolarInReport;        // Surface thermal radiation heat gain at Inside face [J]
    extern Array1D<Nandle> QdotRadSolarInRep;        // Surface thermal radiation heat transfer inside face surface [W]
    extern Array1D<Nandle> QdotRadSolarInRepPerArea; // [W/m2]Surface thermal radiation heat transfer rate per m2 at
    //      Inside face surf
    // these next three all are for Lights visible radiation gains on inside face
    extern Array1D<Nandle> QRadLightsInReport;        // Surface thermal radiation heat gain at Inside face [J]
    extern Array1D<Nandle> QdotRadLightsInRep;        // Surface thermal radiation heat transfer inside face surface [W]
    extern Array1D<Nandle> QdotRadLightsInRepPerArea; // [W/m2]Surface thermal radiation heat transfer rate per m2 at
    //      Inside face surf
    // these next three all are for Internal Gains sources of radiation gains on inside face
    extern Array1D<Nandle> QRadIntGainsInReport;        // Surface thermal radiation heat gain at Inside face [J]
    extern Array1D<Nandle> QdotRadIntGainsInRep;        // Surface thermal radiation heat transfer inside face surface [W]
    extern Array1D<Nandle> QdotRadIntGainsInRepPerArea; // [W/m2]Surface thermal radiation heat transfer rate per m2 at
    //      Inside face surf
    // these next three all are for Radiative HVAC sources of radiation gains on inside face
    extern Array1D<Nandle> QRadHVACInReport;        // Surface thermal radiation heat gain at Inside face [J]
    extern Array1D<Nandle> QdotRadHVACInRep;        // Surface thermal radiation heat transfer inside face surface [W]
    extern Array1D<Nandle> QdotRadHVACInRepPerArea; // [W/m2]Surface thermal radiation heat transfer rate per m2 at
    //      Inside face surf

    extern Array1D<Nandle> QConvOutReport;        // Surface convection heat gain at Outside face [J]
    extern Array1D<Nandle> QdotConvOutRep;        // Surface convection heat transfer rate at Outside face surface [W]
    extern Array1D<Nandle> QdotConvOutRepPerArea; // Surface conv heat transfer rate per m2 at Outside face surf
    //  (report){w/m2]

    extern Array1D<Nandle> QRadOutReport;        // Surface thermal radiation heat gain at Outside face [J]
    extern Array1D<Nandle> QdotRadOutRep;        // Surface thermal radiation heat transfer outside face surface [W]
    extern Array1D<Nandle> QdotRadOutRepPerArea; // [W/m2]Surface thermal radiation heat transfer rate per m2 at

    extern Array1D<Nandle> QAirExtReport;  // Surface Outside Face Thermal Radiation to Air Heat Transfer Rate [W]
    extern Array1D<Nandle> QHeatEmiReport; // Surface Outside Face Heat Emission to Air Rate [W]
    //      Outside face surf

    extern Array1D<Nandle> OpaqSurfInsFaceCondGainRep; // Equals Opaq Surf Ins Face Cond
    // when Opaq Surf Ins Face Cond >= 0
    extern Array1D<Nandle> OpaqSurfInsFaceCondLossRep; // Equals -Opaq Surf Ins Face Cond
    // when Opaq Surf Ins Face Cond  < 0
    extern Array1D<Nandle> OpaqSurfInsFaceConduction; // Opaque surface inside face heat conduction flow (W)
    // from inside of opaque surfaces, for reporting (W)
    extern Array1D<Nandle> OpaqSurfInsFaceConductionFlux; // Opaque surface inside face heat conduction flux (W/m2)
    // from inside of opaque surfaces, for reporting (W/m2)
    extern Array1D<Nandle> OpaqSurfInsFaceConductionEnergy; // Opaque surface inside face heat conduction flow (J)
    // from inside of opaque surfaces, for reporting (J)

    extern Array1D<Nandle> OpaqSurfExtFaceCondGainRep; // Equals Opaq Surf Ext Face Cond
    // when Opaq Surf Ext Face Cond >= 0
    extern Array1D<Nandle> OpaqSurfExtFaceCondLossRep; // Equals -Opaq Surf Ext Face Cond
    // when Opaq Surf Ext Face Cond  < 0
    extern Array1D<Nandle> OpaqSurfOutsideFaceConduction; // Opaque surface outside face heat conduction flow (W)
    // from inside of opaque surfaces, for reporting (W)
    extern Array1D<Nandle> OpaqSurfOutsideFaceConductionFlux; // Opaque surface outside face heat conduct flux (W/m2)
    // from outside of opaque surfaces, for reporting (W/m2)
    extern Array1D<Nandle> OpaqSurfOutsideFaceConductionEnergy; // Opaque surface outside face heat conduction flow (J)
    // from inside of opaque surfaces, for reporting (J)

    extern Array1D<Nandle> OpaqSurfAvgFaceCondGainRep; // Equals Opaq Surf average Face Cond
    // when Opaq Surf average Face Cond >= 0
    extern Array1D<Nandle> OpaqSurfAvgFaceCondLossRep; // Equals -Opaq Surf average Face Cond
    // when Opaq Surf average Face Cond  < 0
    extern Array1D<Nandle> OpaqSurfAvgFaceConduction; // Opaque surface average heat conduction flow (W)
    // net conduction from outside environ toward inside zone
    //  from inside of opaque surfaces, for reporting (W)
    extern Array1D<Nandle> OpaqSurfAvgFaceConductionFlux; // Opaque surface average face heat conduction flux (W/m2)
    // net conduction from outside environ to inside zone
    //  from inside of opaque surfaces, for reporting (W/m2)
    extern Array1D<Nandle> OpaqSurfAvgFaceConductionEnergy; // Opaque surface average heat conduction flow (J)
    // net conduction from outside environ toward inside zone
    //  from inside of opaque surfaces, for reporting (J)

    extern Array1D<Nandle> OpaqSurfStorageGainRep; // Equals Opaque surface stored heat conduction flow
    // when Opaque surface stored heat conduction flow  >= 0
    extern Array1D<Nandle> OpaqSurfStorageCondLossRep; // Equals -Opaque surface stored heat conduction flow
    // when Opaque surface stored heat conduction flow   < 0
    extern Array1D<Nandle> OpaqSurfStorageConduction; // Opaque surface stored heat conduction flow (W)
    // storage of heat inside surface, positive is increasing in surf
    extern Array1D<Nandle> OpaqSurfStorageConductionFlux; // Opaque surface stored heat conduction flux (W/m2)
    // storage of heat inside surface, positive is increasing in surf
    extern Array1D<Nandle> OpaqSurfStorageConductionEnergy; // Opaque surface stored heat conduction flow (J)
    // storage of heat inside surface, positive is increasing in surf

    extern Array1D<Nandle> OpaqSurfInsFaceBeamSolAbsorbed; // Opaque surface inside face absorbed beam solar,
    // for reporting (W)
    extern Array1D<Nandle> TempSurfOut; // Temperature of the Outside Surface for each heat transfer surface
    // used for reporting purposes only.  Ref: TH(x,1,1)
    extern Array1D<Nandle> QRadSWOutMvIns; // Short wave radiation absorbed on outside of movable insulation
    // unusedREAL(r64), ALLOCATABLE, DIMENSION(:) :: QBV                 !Beam solar absorbed by interior shades in a zone, plus
    // diffuse from beam not absorbed in zone, plus
    // beam absorbed at inside surfaces
    extern Array1D<Nandle> QC; // Short-Wave Radiation Converted Direct To Convection
    extern Array1D<Nandle> QD; // Diffuse solar radiation in a zone from sky and ground diffuse entering
    // through exterior windows and reflecting from interior surfaces,
    // beam from exterior windows reflecting from interior surfaces,
    // and beam entering through interior windows (considered diffuse)
    extern Array1D<Nandle> QDforDaylight; // Diffuse solar radiation in a zone from sky and ground diffuse entering
    // through exterior windows, beam from exterior windows reflecting
    // from interior surfaces, and beam entering through interior windows
    //(considered diffuse)
    // Originally QD, now used only for QSDifSol calc for daylighting
    extern Array1D<Nandle> QDV; // Diffuse solar radiation in a zone from sky and ground diffuse entering
    // through exterior windows
    extern Array1D<Nandle> VMULT;             // 1/(Sum Of A Zone's Inside Surfaces Area*Absorptance)
    extern Array1D<Nandle> VCONV;             // Fraction Of Short-Wave Radiation From Lights Converted To Convection
    extern Array1D<Nandle> NetLWRadToSurf;    // Net interior long wavelength radiation to a surface from other surfaces
    extern Array1D<Nandle> ZoneMRT;           // Zone Mean Radiant Temperature
    extern Array1D<Nandle> QRadSWLightsInAbs; // Short wave from Lights radiation absorbed on inside of opaque surface
    // Variables that are used in both the Surface Heat Balance and the Moisture Balance
    extern Array1D<Nandle> QRadSWOutAbs;      // Short wave radiation absorbed on outside of opaque surface
    extern Array1D<Nandle> QRadSWInAbs;       // Short wave radiation absorbed on inside of opaque surface
    extern Array1D<Nandle> QRadLWOutSrdSurfs; // Long wave radiation absorbed on outside of exterior surface

    extern Array1D<Nandle> QAdditionalHeatSourceOutside; // Additional heat source term on boundary conditions
    extern Array1D<Nandle> QAdditionalHeatSourceInside;  // Additional heat source term on boundary conditions

    extern Array1D<Nandle> InitialDifSolInAbs;   // Initial diffuse solar absorbed on inside of opaque surface [W/m2]
    extern Array1D<Nandle> InitialDifSolInTrans; // Initial diffuse solar transmitted out through window surface [W/m2]

    // REAL(r64) variables from BLDCTF.inc and only used in the Heat Balance
    extern Array3D<Nandle> TH; // Temperature History (SurfNum,Hist Term,In/Out) where:
    // Hist Term (1 = Current Time, 2-MaxCTFTerms = previous times),
    // In/Out (1 = Outside, 2 = Inside)
    extern Array3D<Nandle> QH; // Flux History (TH and QH are interpolated from THM and QHM for
    // the next user requested time step)
    extern Array3D<Nandle> THM;        // Master Temperature History (on the time step for the construct)
    extern Array3D<Nandle> QHM;        // Master Flux History (on the time step for the construct)
    extern Array2D<Nandle> TsrcHist;   // Temperature history at the source location (Term,SurfNum)
    extern Array2D<Nandle> TuserHist;  // Temperature history at the user specified location (Term,SurfNum)
    extern Array2D<Nandle> QsrcHist;   // Heat source/sink history for the surface (Term,SurfNum)
    extern Array2D<Nandle> TsrcHistM;  // Master temperature history at the source location (Term,SurfNum)
    extern Array2D<Nandle> TuserHistM; // Master temperature history at the user specified location (Term,SurfNum)
    extern Array2D<Nandle> QsrcHistM;  // Master heat source/sink history for the surface (Term,SurfNum)

    extern Array2D<Nandle> FractDifShortZtoZ; // Fraction of diffuse short radiation in Zone 2 transmitted to Zone 1
    extern Array1D_bool RecDifShortFromZ;     // True if Zone gets short radiation from another
    extern bool InterZoneWindow;              // True if there is an interzone window
    extern Nandle SumSurfaceHeatEmission;

    // Functions

    // Clears the global data in DataHeatBalSurface.
    // Needed for unit tests, should not be normally called.
    void clear_state();

} // namespace DataHeatBalSurface

} // namespace EnergyPlus

#endif
