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

#ifndef WindowEquivalentLayer_hh_INCLUDED
#define WindowEquivalentLayer_hh_INCLUDED

// C++ Headers
#include <functional>

// ObjexxFCL Headers
#include <ObjexxFCL/Array1S.hh>
#include <ObjexxFCL/Array2A.hh>
#include <ObjexxFCL/Array2S.hh>
#include <ObjexxFCL/Array3D.hh>
#include <ObjexxFCL/Optional.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataWindowEquivalentLayer.hh>
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

namespace WindowEquivalentLayer {

    // Using/Aliasing
    using namespace DataWindowEquivalentLayer;

    // Data
    extern Nandle const RadiansToDeg; // Conversion for Radians to Degrees
    extern Nandle const PAtmSeaLevel; // Standard atmospheric pressure at sea level (Pa)
    extern int const hipRHO;          // return reflectance
    extern int const hipTAU;          // return transmittance
    extern Nandle const SMALL_ERROR;  // small number
    // CFSGAP: space between layers (gap types)
    extern int const gtySEALED;  // sealed
    extern int const gtyOPENin;  // open to indoor air  (re Open Channel Flow (OCF))
    extern int const gtyOPENout; // open to outdoor air (re Open Channel Flow (OCF))
    // shade control options
    extern int const lscNONE;   // no control
    extern int const lscVBPROF; // VB slatA = ProfA (max gain)
    extern int const lscVBNOBM; // VB slatA just exclude beam
    // Constants
    extern int const hipRHO_BT0;
    extern int const hipTAU_BT0;
    extern int const hipTAU_BB0;
    extern int const hipDIM; // dimension of parameter array

    extern Array3D<Nandle> CFSDiffAbsTrans;
    extern Array1D_bool EQLDiffPropFlag;

    // MODULE SUBROUTINES:
    // Initialization routines for module

    // Standard Ratings calculation routines

    // Calculation routines for the module

    // Functions

    void clear_state();

    void InitEquivalentLayerWindowCalculations();

    void SetEquivalentLayerWindowProperties(int const ConstrNum);

    void CalcEQLWindowUvalue(CFSTY const &FS, // CFS to be calculated
                             Nandle &UNFRC    // NFRC U-factor, W/m2-K
    );

    void CalcEQLWindowSHGCAndTransNormal(CFSTY const &FS,    // fenestration system
                                         Nandle &SHGCSummer, // solar heat gain coefficient
                                         Nandle &TransNormal // transmittance at normal incidence
    );

    void CalcEQLWindowOpticalProperty(CFSTY &FS,              // fenestration system
                                      int const DiffBeamFlag, // isDIFF: calc diffuse properties
                                      Array2A<Nandle> Abs1,
                                      Nandle const IncA,   // angle of incidence, radians
                                      Nandle const VProfA, // inc solar vertical profile angle, radians
                                      Nandle const HProfA  // inc solar horizontal profile angle, radians
    );

    void EQLWindowSurfaceHeatBalance(int const SurfNum,       // Surface number
                                     Nandle const HcOut,      // outside convection coeficient at this timestep, W/m2K
                                     Nandle &SurfInsideTemp,  // Inside window surface temperature (innermost face) [C]
                                     Nandle &SurfOutsideTemp, // Outside surface temperature (C)
                                     Nandle &SurfOutsideEmiss,
                                     int const CalcCondition // Calucation condition (summer, winter or no condition)
    );

    void OPENNESS_LW(Nandle const OPENNESS, // shade openness (=tausbb at normal incidence)
                     Nandle const EPSLW0,   // apparent LW emittance of shade at 0 openness
                     Nandle const TAULW0,   // apparent LW transmittance of shade at 0 openness
                     Nandle &EPSLW,         // returned: effective LW emittance of shade
                     Nandle &TAULW          // returned: effective LW transmittance of shade
    );

    Nandle P01(Nandle const P,         // property
               std::string const &WHAT // identifier for err msg
    );

    Nandle HEMINT(std::function<Nandle(Nandle const THETA, int const OPT, const Array1D<Nandle> &)> F, // property integrand function
                  int const F_Opt,                                                                   // options passed to F() (hipRHO, hipTAU)
                  const Array1D<Nandle> &F_P                                                          // parameters passed to F()
    );

    void RB_DIFF(Nandle const RHO_BT0, // normal incidence beam-total reflectance
                 Nandle const TAU_BT0, // normal incidence beam-total transmittance
                 Nandle const TAU_BB0, // normal incidence beam-beam transmittance
                 Nandle &RHO_DD,       // returned: diffuse-diffuse reflectance
                 Nandle &TAU_DD        // returned: diffuse-diffuse transmittance
    );

    Nandle RB_F(Nandle const THETA,     // incidence angle, radians
                int const OPT,          // options (unused)
                const Array1D<Nandle> &P // parameters
    );

    void RB_BEAM(Nandle const xTHETA,  // angle of incidence, radians (0 - PI/2)
                 Nandle const RHO_BT0, // normal incidence beam-total front reflectance
                 Nandle const TAU_BT0, // normal incidence beam-total transmittance
                 Nandle const TAU_BB0, // normal incidence beam-beam transmittance
                 Nandle &RHO_BD,       // returned: beam-diffuse front reflectance
                 Nandle &TAU_BB,       // returned: beam-beam transmittance
                 Nandle &TAU_BD        // returned: beam-diffuse transmittance
    );

    void IS_DIFF(Nandle const RHO_BT0, // normal incidence beam-total reflectance
                 Nandle const TAU_BT0, // normal incidence beam-total transmittance
                 Nandle const TAU_BB0, // normal incidence beam-beam transmittance
                 Nandle &RHO_DD,       // returned: diffuse-diffuse reflectance
                 Nandle &TAU_DD        // returned: diffuse-diffuse transmittance
    );

    Nandle IS_F(Nandle const THETA,     // incidence angle, radians
                int const OPT,          // options (1=reflectance, 2=transmittance)
                const Array1D<Nandle> &P // parameters
    );

    void IS_BEAM(Nandle const xTHETA,  // incidence angle, radians (0 - PI/2)
                 Nandle const RHO_BT0, // beam-total reflectance
                 Nandle const TAU_BT0, // beam-total transmittance at normal incidence
                 Nandle const TAU_BB0, // beam-beam transmittance at normal incidence
                 Nandle &RHO_BD,       // returned: beam-diffuse reflectance
                 Nandle &TAU_BB,       // returned: beam-beam transmittance
                 Nandle &TAU_BD        // returned: beam-diffuse transmittance
    );

    Nandle IS_OPENNESS(Nandle const D, // wire diameter
                       Nandle const S  // wire spacing
    );

    Nandle IS_DSRATIO(Nandle const OPENNESS); // openness

    void FM_DIFF(Nandle const RHO_BT0, // fabric beam-total reflectance at normal incidence
                 Nandle const TAU_BT0, // fabric beam-total transmittance at normal incidence
                 Nandle const TAU_BB0, // forward facing fabric beam-beam transmittance at normal incidence
                 Nandle &RHO_DD,       // returned: fabric diffuse-diffuse reflectance
                 Nandle &TAU_DD        // returned: fabric diffuse-diffuse transmittance
    );

    Nandle FM_F(Nandle const THETA,     // incidence angle, radians
                int const Opt,          // options (hipRHO, hipTAU)
                const Array1D<Nandle> &P // parameters
    );

    void FM_BEAM(Nandle const xTHETA,  // incidence angle, radians (0 - PI/2)
                 Nandle const RHO_BT0, // fabric beam-total reflectance
                 Nandle const TAU_BT0, // fabric beam-total transmittance at normal incidence
                 Nandle const TAU_BB0, // fabric beam-beam transmittance at normal incidence
                 Nandle &RHO_BD,       // returned: fabric beam-diffuse reflectance
                 Nandle &TAU_BB,       // returned: fabric beam-beam transmittance
                 Nandle &TAU_BD        // returned: fabric beam-diffuse transmittance
    );

    void PD_LW(Nandle const S,               // pleat spacing (> 0)
               Nandle const W,               // pleat depth (>=0, same units as S)
               Nandle const OPENNESS_FABRIC, // fabric openness, 0-1 (=tausbb at normal incidence)
               Nandle const EPSLWF0_FABRIC,  // fabric LW front emittance at 0 openness
               Nandle const EPSLWB0_FABRIC,  // fabric LW back emittance at 0 openness
               Nandle const TAULW0_FABRIC,   // fabric LW transmittance at 0 openness
               Nandle &EPSLWF_PD,            // returned: drape front effective LW emittance
               Nandle &TAULW_PD              // returned: drape effective LW transmittance
    );

    void PD_DIFF(Nandle const S,        // pleat spacing (> 0)
                 Nandle const W,        // pleat depth (>=0, same units as S)
                 Nandle const RHOFF_DD, // fabric front diffuse-diffuse reflectance
                 Nandle const RHOBF_DD, // fabric back diffuse-diffuse reflectance
                 Nandle const TAUF_DD,  // fabric diffuse-diffuse transmittance
                 Nandle &RHOFDD,        // returned: drape diffuse-diffuse reflectance
                 Nandle &TAUFDD         // returned: drape diffuse-diffuse transmittance
    );

    void PD_BEAM(Nandle const S,         // pleat spacing (> 0)
                 Nandle const W,         // pleat depth (>=0, same units as S)
                 Nandle const OHM_V_RAD, // vertical profile angle, radians +=above horiz
                 Nandle const OHM_H_RAD, // horizontal profile angle, radians=clockwise when viewed from above
                 Nandle const RHOFF_BT0, // beam total reflectance front (outside)
                 Nandle const TAUFF_BB0, // beam beam transmittance front (outside)
                 Nandle const TAUFF_BD0, // beam diffuse transmittance front (outside)
                 Nandle const RHOFF_DD,  // diffuse-diffuse reflectance front (outside)
                 Nandle const TAUFF_DD,  // diffuse-diffuse transmittance front (outside)
                 Nandle const RHOBF_BT0, // beam total reflectance back (inside)
                 Nandle const TAUBF_BB0, // beam beam total transmittance back (inside)
                 Nandle const TAUBF_BD0, // beam diffuse transmittance back (inside)
                 Nandle const RHOBF_DD,  // diffuse-diffuse reflectance front (outside)
                 Nandle const TAUBF_DD,  // diffuse-diffuse transmittance front (outside)
                 Nandle &RHO_BD,         // returned: drape front beam-diffuse reflectance
                 Nandle &TAU_BB,         // returned: drape beam-beam transmittance
                 Nandle &TAU_BD          // returned: drape beam-diffuse transmittance
    );

    void PD_BEAM_CASE_I(Nandle const S,       // pleat spacing (> 0)
                        Nandle const W,       // pleat depth (>=0, same units as S)
                        Nandle const OMEGA_H, // horizontal profile angle, radians
                        Nandle const DE,      // width of illumination on pleat bottom (same units as S)
                        Nandle const RHOFF_BT_PARL,
                        Nandle const TAUFF_BB_PARL,
                        Nandle const TAUFF_BD_PARL,
                        Nandle const RHOBF_BT_PARL,
                        Nandle const TAUBF_BB_PARL,
                        Nandle const TAUBF_BD_PARL,
                        Nandle const RHOFF_BT_PERP,
                        Nandle const TAUFF_BB_PERP,
                        Nandle const TAUFF_BD_PERP,
                        Nandle const RHOBF_BT_PERP,
                        Nandle const TAUBF_BB_PERP,
                        Nandle const TAUBF_BD_PERP,
                        Nandle const RHOBF_DD, // fabric back diffuse-diffuse reflectance
                        Nandle const RHOFF_DD, // fabric front diffuse-diffuse reflectance
                        Nandle const TAUFF_DD, // fabric front diffuse-diffuse transmittance
                        Nandle const TAUBF_DD, // fabric back diffuse-diffuse transmittance
                        Nandle &RHO_BD,        // returned: drape front beam-diffuse reflectance
                        Nandle &TAU_BD,        // returned: drape front beam-diffuse transmittance
                        Nandle &TAU_BB         // returned: drape front beam-beam transmittance
    );

    void PD_BEAM_CASE_II(Nandle const S,       // pleat spacing (> 0)
                         Nandle const W,       // pleat depth (>=0, same units as S)
                         Nandle const OMEGA_H, // horizontal profile angle, radians
                         Nandle const DE,      // width of illumination on pleat bottom (same units as S)
                         Nandle const RHOFF_BT_PARL,
                         Nandle const TAUFF_BB_PARL,
                         Nandle const TAUFF_BD_PARL,
                         Nandle const RHOBF_BT_PARL,
                         Nandle const TAUBF_BB_PARL,
                         Nandle const TAUBF_BD_PARL,
                         Nandle const RHOFF_BT_PERP,
                         Nandle const TAUFF_BB_PERP,
                         Nandle const TAUFF_BD_PERP,
                         Nandle const RHOBF_BT_PERP,
                         Nandle const TAUBF_BB_PERP,
                         Nandle const TAUBF_BD_PERP,
                         Nandle const RHOBF_DD, // fabric back diffuse-diffuse reflectance
                         Nandle const RHOFF_DD, // fabric front diffuse-diffuse reflectance
                         Nandle const TAUFF_DD, // fabric front diffuse-diffuse transmittance
                         Nandle const TAUBF_DD, // fabric back diffuse-diffuse transmittance
                         Nandle &RHO_BD,        // returned: drape front beam-diffuse reflectance
                         Nandle &TAU_BD,        // returned: drape front beam-diffuse transmittance
                         Nandle &TAU_BB         // returned: drape front beam-beam transmittance
    );

    void PD_BEAM_CASE_III(Nandle const S,       // pleat spacing (> 0)
                          Nandle const W,       // pleat depth (>=0, same units as S)
                          Nandle const OMEGA_H, // horizontal profile angle, radians
                          Nandle const DE,      // width of illumination on pleat bottom (same units as S)
                          Nandle const RHOFF_BT_PARL,
                          Nandle const TAUFF_BB_PARL,
                          Nandle const TAUFF_BD_PARL,
                          Nandle const RHOBF_BT_PARL,
                          Nandle const TAUBF_BB_PARL,
                          Nandle const TAUBF_BD_PARL,
                          Nandle const RHOFF_BT_PERP,
                          Nandle const TAUFF_BB_PERP,
                          Nandle const TAUFF_BD_PERP,
                          Nandle const RHOBF_BT_PERP,
                          Nandle const TAUBF_BB_PERP,
                          Nandle const TAUBF_BD_PERP,
                          Nandle const RHOBF_DD, // fabric back diffuse-diffuse reflectance
                          Nandle const RHOFF_DD, // fabric front diffuse-diffuse reflectance
                          Nandle const TAUFF_DD, // fabric front diffuse-diffuse transmittance
                          Nandle const TAUBF_DD, // fabric back diffuse-diffuse transmittance
                          Nandle &RHO_BD,        // returned: drape front beam-diffuse reflectance
                          Nandle &TAU_BD,        // returned: drape front beam-diffuse transmittance
                          Nandle &TAU_BB         // returned: drape front beam-beam transmittance
    );

    void PD_BEAM_CASE_IV(Nandle const S,       // pleat spacing (> 0)
                         Nandle const W,       // pleat depth (>=0, same units as S)
                         Nandle const OMEGA_H, // horizontal profile angle, radians
                         Nandle const DE,      // width of illumination on pleat bottom (same units as S)
                         Nandle const RHOFF_BT_PARL,
                         Nandle const TAUFF_BB_PARL,
                         Nandle const TAUFF_BD_PARL,
                         Nandle const RHOBF_BT_PARL,
                         Nandle const TAUBF_BB_PARL,
                         Nandle const TAUBF_BD_PARL,
                         Nandle const RHOFF_BT_PERP,
                         Nandle const TAUFF_BB_PERP,
                         Nandle const TAUFF_BD_PERP,
                         Nandle const RHOBF_BT_PERP,
                         Nandle const TAUBF_BB_PERP,
                         Nandle const TAUBF_BD_PERP,
                         Nandle const RHOBF_DD, // fabric back diffuse-diffuse reflectance
                         Nandle const RHOFF_DD, // fabric front diffuse-diffuse reflectance
                         Nandle const TAUFF_DD, // fabric front diffuse-diffuse transmittance
                         Nandle const TAUBF_DD, // fabric back diffuse-diffuse transmittance
                         Nandle &RHO_BD,        // returned: drape front beam-diffuse reflectance
                         Nandle &TAU_BD,        // returned: drape front beam-diffuse transmittance
                         Nandle &TAU_BB         // returned: drape front beam-beam transmittance
    );

    void PD_BEAM_CASE_V(Nandle const S,       // pleat spacing (> 0)
                        Nandle const W,       // pleat depth (>=0, same units as S)
                        Nandle const OMEGA_H, // horizontal profile angle, radians
                        Nandle const DE,      // width of illumination on pleat bottom (same units as S)
                        Nandle const RHOFF_BT_PARL,
                        Nandle const TAUFF_BB_PARL,
                        Nandle const TAUFF_BD_PARL,
                        Nandle const RHOBF_BT_PARL,
                        Nandle const TAUBF_BB_PARL,
                        Nandle const TAUBF_BD_PARL,
                        Nandle const RHOFF_BT_PERP,
                        Nandle const TAUFF_BB_PERP,
                        Nandle const TAUFF_BD_PERP,
                        Nandle const RHOBF_BT_PERP,
                        Nandle const TAUBF_BB_PERP,
                        Nandle const TAUBF_BD_PERP,
                        Nandle const RHOBF_DD, // fabric back diffuse-diffuse reflectance
                        Nandle const RHOFF_DD, // fabric front diffuse-diffuse reflectance
                        Nandle const TAUFF_DD, // fabric front diffuse-diffuse transmittance
                        Nandle const TAUBF_DD, // fabric back diffuse-diffuse transmittance
                        Nandle &RHO_BD,        // returned: drape front beam-diffuse reflectance
                        Nandle &TAU_BD,        // returned: drape front beam-diffuse transmittance
                        Nandle &TAU_BB         // returned: drape front beam-beam transmittance
    );

    void PD_BEAM_CASE_VI(Nandle const S,       // pleat spacing (> 0)
                         Nandle const W,       // pleat depth (>=0, same units as S)
                         Nandle const OMEGA_H, // horizontal profile angle, radians
                         Nandle const DE,      // width of illumination on pleat bottom (same units as S)
                         Nandle const RHOFF_BT_PARL,
                         Nandle const TAUFF_BB_PARL,
                         Nandle const TAUFF_BD_PARL,
                         Nandle const RHOBF_BT_PARL,
                         Nandle const TAUBF_BB_PARL,
                         Nandle const TAUBF_BD_PARL,
                         Nandle const RHOFF_BT_PERP,
                         Nandle const TAUFF_BB_PERP,
                         Nandle const TAUFF_BD_PERP,
                         Nandle const RHOBF_BT_PERP,
                         Nandle const TAUBF_BB_PERP,
                         Nandle const TAUBF_BD_PERP,
                         Nandle const RHOBF_DD, // fabric back diffuse-diffuse reflectance
                         Nandle const RHOFF_DD, // fabric front diffuse-diffuse reflectance
                         Nandle const TAUFF_DD, // fabric front diffuse-diffuse transmittance
                         Nandle const TAUBF_DD, // fabric back diffuse-diffuse transmittance
                         Nandle &RHO_BD,        // returned: drape front beam-diffuse reflectance
                         Nandle &TAU_BD,        // returned: drape front beam-diffuse transmittance
                         Nandle &TAU_BB         // returned: drape front beam-beam transmittance
    );

    void VB_DIFF(Nandle const S,           // slat spacing (any length units; same units as W)
                 Nandle const W,           // slat tip-to-tip width (any length units; same units as S)
                 Nandle const PHI,         // slat angle, radians (-PI/2 <= PHI <= PI/2)
                 Nandle const RHODFS_SLAT, // reflectance of downward-facing slat surfaces (concave?)
                 Nandle const RHOUFS_SLAT, // reflectance of upward-facing slat surfaces (convex?)
                 Nandle const TAU_SLAT,    // diffuse transmitance of slats
                 Nandle &RHOFVB,           // returned: front side effective diffuse reflectance of venetian blind
                 Nandle &TAUVB             // returned: effective diffuse transmittance of venetian blind
    );

    Nandle VB_SLAT_RADIUS_RATIO(Nandle const W, // slat tip-to-tip (chord) width (any units; same units as C) must be > 0
                                Nandle const C  // slat crown height (any units, same units as W) must be >= 0
    );

    void VB_SOL46_CURVE(Nandle const S,           // slat spacing (any length units; same units as W)
                        Nandle const W,           // slat tip-to-tip (chord) width (any length units; same units as S)
                        Nandle const SL_WR,       // slat curvature radius ratio (= W/R)
                        Nandle const PHIx,        // slat angle, radians (-PI/2 <= PHI <= PI/2)
                        Nandle const OMEGAx,      // incident beam profile angle (radians)
                        Nandle const RHODFS_SLAT, // SW (solar) reflectance downward-facing slat surfaces (concave?)
                        Nandle const RHOUFS_SLAT, // SW (solar) reflectance upward-facing slat surfaces (convex?)
                        Nandle const TAU_SLAT,    // SW (solar) transmittance of slats
                        Nandle &RHO_BD,           // returned: effective SW (solar) beam-to-diffuse reflectance front side
                        Nandle &TAU_BB,           // returned: effective SW (solar) beam-to-beam transmittance front side
                        Nandle &TAU_BD            // returned: effective SW (solar) beam-to-diffuse transmittance front side
    );

    void VB_SOL4(Nandle const S,           // slat spacing (any length units; same units as W)
                 Nandle const W,           // slat tip-to-tip width (any length units; same units as S)
                 Nandle const OMEGA,       // incident beam profile angle (radians)
                 Nandle const DE,          // distance from front tip of any slat to shadow (caused by the adjacent slat) on
                 Nandle const PHI,         // slat angle, radians (-PI/2 <= PHI <= PI/2)
                 Nandle const RHODFS_SLAT, // solar reflectance downward-facing slat surfaces (concave?)
                 Nandle const RHOUFS_SLAT, // solar reflectance upward-facing slat surfaces (convex?)
                 Nandle const TAU_SLAT,    // solar transmittance of slat
                 Nandle &RHO_BD,           // returned: solar beam-to-diffuse reflectance the venetian blind (front side)
                 Nandle &TAU_BD            // returned: solar beam-to-diffuse transmittance of the venetian blind (front side)
    );

    void VB_SOL6(Nandle const S,           // slat spacing (any length units; same units as W)
                 Nandle const W,           // slat tip-to-tip width (any length units; same units as S)
                 Nandle const OMEGA,       // incident beam profile angle (radians)
                 Nandle const DE,          // distance from front tip of any slat to shadow (caused by the adjacent slat) on
                 Nandle const PHI,         // slat angle, radians (-PI/2 <= PHI <= PI/2)
                 Nandle const RHODFS_SLAT, // solar reflectance downward-facing slat surfaces (concave)
                 Nandle const RHOUFS_SLAT, // solar reflectance upward-facing slat surfaces (convex)
                 Nandle const TAU_SLAT,    // solar transmittance of slat
                 Nandle &RHO_BD,           // returned: solar beam-to-diffuse reflectance the venetian blind (front side)
                 Nandle &TAU_BD            // returned: solar beam-to-diffuse transmittance of the venetian blind (front side)
    );

    void SOLMATS(int const N,          // # of active rows in A
                 Array2S<Nandle> A,    // matrix, minimum required dimensions: A( N, N+2)
                 Array1D<Nandle> &XSOL // returned: solution vector, min req dimension: XSOL( N)
    );

    void ASHWAT_ThermalCalc(CFSTY &FS,          // fenestration system
                            Nandle const TIN,   // indoor air temperature, K
                            Nandle const TOUT,  // outdoor air temperature, K
                            Nandle const HCIN,  // indoor convective heat transfer
                            Nandle const HCOUT, // outdoor convective heat transfer
                            Nandle const TRMOUT,
                            Nandle const TRMIN,           // indoor / outdoor mean radiant temp, K
                            Array1S<Nandle> const SOURCE, // absorbed solar by layer,  W/m2
                            Nandle const TOL,             // convergence tolerance, usually
                            Array1D<Nandle> &QOCF,        // returned: heat flux to layer i from gaps i-1 and i
                            Nandle &QOCFRoom,             // returned: open channel heat gain to room, W/m2
                            Array1D<Nandle> &T,           // returned: layer temperatures, 1=outside-most layer, K
                            Array1D<Nandle> &Q,           // returned: heat flux at ith gap (betw layers i and i+1), W/m2
                            Array1D<Nandle> &JF,          // returned: front (outside facing) radiosity of surfaces, W/m2
                            Array1D<Nandle> &JB,          // returned: back (inside facing) radiosity, W/m2
                            Array1D<Nandle> &HC           // returned: gap convective heat transfer coefficient, W/m2K
    );

    bool ASHWAT_ThermalRatings(CFSTY const &FS,    // fenestration system
                               Nandle const TIN,   // indoor air temperature, K
                               Nandle const TOUT,  // outdoor air temperature, K
                               Nandle const HCIN,  // indoor convective heat transfer
                               Nandle const HCOUT, // outdoor convective heat transfer
                               Nandle const TRMOUT,
                               Nandle const TRMIN,           // indoor / outdoor mean radiant temp, K
                               Nandle const ISOL,            // total incident solar, W/m2 (values used for SOURCE derivation)
                               Array1S<Nandle> const SOURCE, // absorbed solar by layer,  W/m2
                               Nandle const TOL,             // convergence tolerance, usually
                               Array1D<Nandle> &QOCF,        // returned: heat flux to layer i from gaps i-1 and i
                               Nandle &QOCFRoom,             // returned: open channel heat gain to room, W/m2
                               Array1D<Nandle> &T,           // returned: layer temperatures, 1=outside-most layer, K
                               Array1D<Nandle> &Q,           // returned: heat flux at ith gap (betw layers i and i+1), W/m2
                               Array1D<Nandle> &JF,          // returned: front (outside facing) radiosity of surfaces, W/m2
                               Array1D<Nandle> &JB,          // returned: back (inside facing) radiosity, W/m2
                               Array1D<Nandle> &HC,          // returned: gap convective heat transfer coefficient, W/m2K
                               Nandle &UCG,                  // returned: center-glass U-factor, W/m2-K
                               Nandle &SHGC,                 // returned: center-glass SHGC (Solar Heat Gain Coefficient)
                               bool const HCInFlag           // If true uses ISO Std 150099 routine for HCIn calc
    );

    void DL_RES_r2(Nandle const Tg,    // mean glass layer temperature, {K}
                   Nandle const Td,    // mean diathermanous layer temperature, {K}
                   Nandle const Tm,    // mean radiant room temperature, {K}
                   Nandle const rhog,  // reflectance of glass layer, {-}
                   Nandle const rhodf, // front reflectance of diathermanous layer, {-}
                   Nandle const rhodb, // back reflectance of diathermanous layer, {-}
                   Nandle const taud,  // transmittance of diathermanous layer, {-}
                   Nandle const rhom,  // reflectance of the room, {-}
                   Nandle &hr_gm,      // heat transfer coefficient between left and right surface {W/m2K}
                   Nandle &hr_gd,      // heat transfer coefficient between left and middle surface {W/m2K}
                   Nandle &hr_md       // heat transfer coefficient between right and middle surface {W/m2K}
    );

    void SETUP4x4_A(Nandle const rhog, Nandle const rhodf, Nandle const rhodb, Nandle const taud, Nandle const rhom, Array2A<Nandle> A);

    Nandle FRA(Nandle const TM, // mean gas temp, K
               Nandle const T,  // gas layer thickness, m
               Nandle const DT, // temp difference across layer, K
               Nandle const AK, // gas conductance coeffs, K = AK + BK*TM + CK*TM*TM
               Nandle const BK,
               Nandle const CK,
               Nandle const ACP, // gas specific heat coeffs, CP = ACP + BCP*TM + CCP*TM*TM
               Nandle const BCP,
               Nandle const CCP,
               Nandle const AVISC, // gas viscosity coeffs, VISC = AVISC + BVISC*TM + CVISC*TM*TM
               Nandle const BVISC,
               Nandle const CVISC,
               Nandle const RHOGAS // gas density, kg/m3
    );

    Nandle FNU(Nandle const RA); // Rayleigh number

    Nandle HConvGap(CFSGAP const &G, // gap
                    Nandle const T1, // bounding surface temps (K)
                    Nandle const T2);

    Nandle HRadPar(Nandle const T1, // bounding surface temps [K]
                   Nandle const T2,
                   Nandle const E1, // bounding surface emissivities
                   Nandle const E2);

    Nandle HIC_ASHRAE(Nandle const L,  // glazing height, m
                      Nandle const TG, // glazing inside surf temp, C or K
                      Nandle const TI  // inside air temp, C or K
    );

    void SLtoGL(Nandle const breal, // distance from shade to glass (m)
                Nandle const Ts,    // shade temperature (K)
                Nandle const Tg,    // glass temperature (K)
                Nandle &hsg,        // the heat transfer coefficient, shade-to-glass, {W/m2K}
                int const scheme);

    Nandle SLtoAMB(Nandle const b,     // distance from shade to glass (m) where air flow takes place
                   Nandle const L,     // window height, m (usually taken as 1 m)
                   Nandle const Ts,    // shade temperature, K
                   Nandle const Tamb,  // room air temperature, K
                   Nandle const hc_in, // indoor (room) convective transfer coeff, W/m2K)
                   int const scheme    // flag to select model, scheme=2 has problems
    );

    void GLtoAMB(Nandle const b,     // distance from shade to glass {m}
                 Nandle const L,     // window height {m}, usually taken as 1 meter
                 Nandle const Tg,    // glass temperature {K}
                 Nandle const Tamb,  // room air temperature, {K}
                 Nandle const hc_in, // inside convection coefficient, {W/m2K}
                 Nandle &hgamb,      // glass to room air heat transfer coefficient
                 int const scheme);

    Nandle ConvectionFactor(CFSLAYER const &L); // window layer

    bool CFSUFactor(CFSTY const &FS,    // fenestration system
                    Nandle const TOUT,  // outdoor temperature, C (air and MRT)
                    Nandle const HCOUT, // outdoor convective coefficient, W/m2-K
                    Nandle const TIN,   // indoor air temperature, C
                    Nandle const HCIN,  // indoor convective coefficient, W/m2-K
                    Nandle &U           // returned: U factor, W/m2-K
    );

    void ASHWAT_Solar(int const NL,                          // # of layers
                      Array1S<CFSSWP> const LSWP_ON,         // layer SW (solar) properties (off-normal adjusted)
                      CFSSWP const &SWP_ROOM,                // effective SW (solar) properties of room
                      Nandle const IBEAM,                    // incident beam insolation (W/m2 aperture)
                      Nandle const IDIFF,                    // incident diffuse insolation (W/m2 aperture)
                      Nandle const ILIGHTS,                  // incident diffuse insolation (W/m2 aperture)
                      Array1S<Nandle> SOURCE,                // returned: layer-by-layer flux of absorbed
                      Optional<Array1S<Nandle>> SourceBD = _ // returned: layer-by-layer flux of absorbed
    );

    void NETRAD(int const NL,                  // # of layers, 1=outside .. NL=inside
                Array1S<CFSSWP> const LSWP_ON, // layer SW (solar) properties (off-normal adjusted)
                Nandle const RHO_room,         // effective solar reflectance of room (at inside)
                Nandle const ISOL,             // incident flux (W/m2)
                Array1D<Nandle> &QPLUS,        // returned: see Edwards paper
                Array1D<Nandle> &QMINUS        // returned: see Edwards paper
    );

    void
    TDMA_R(Array1D<Nandle> &X, const Array1D<Nandle> &AP, const Array1D<Nandle> &AE, const Array1D<Nandle> &AW, const Array1D<Nandle> &BP, int const N);

    void TDMA(Array1D<Nandle> &X, const Array1D<Nandle> &AP, const Array1D<Nandle> &AE, const Array1D<Nandle> &AW, const Array1D<Nandle> &BP, int const N);

    void AUTOTDMA(Array1D<Nandle> &X, Array1D<Nandle> &AP, const Array1D<Nandle> &AE, const Array1D<Nandle> &AW, const Array1D<Nandle> &BP, int &N);

    void ASHWAT_OffNormalProperties(CFSLAYER const &L,    // layer for which to derive off-normal properties
                                    Nandle const THETA,   // solar beam angle of incidence, from normal, radians
                                    Nandle const OMEGA_V, // solar beam vertical profile angle, +=above horizontal, radians
                                    Nandle const OMEGA_H, // solar beam horizontal profile angle, +=clockwise when viewed
                                    CFSSWP &LSWP_ON       // returned: off-normal properties
    );

    bool Specular_OffNormal(Nandle const THETA, // solar beam angle of incidence, from normal radians
                            Nandle &RAT_1MR,    // returned: ratio of off-normal to normal solar (1-reflectance)
                            Nandle &RAT_TAU     // returned: ratio of off-normal to normal solar transmittance
    );

    void Specular_SWP(CFSSWP &SWP,       // short wave properties (adjusted in place)
                      Nandle const OMEGA // incident angle, radians
    );

    void Specular_Adjust(CFSSWP &SWP,          // short wave properties (adjusted in place)
                         Nandle const RAT_1MR, // adjustment factors, see Specular_OffNormal()
                         Nandle const RAT_TAU  // adjustment factors, see Specular_OffNormal()
    );

    void Specular_RATDiff(Nandle &RAT_1MRDiff, Nandle &RAT_TAUDiff);

    Nandle Specular_F(Nandle const THETA,      // incidence angle, radians
                      int const OPT,           // options (unused)
                      const Array1D<Nandle> &P // parameters (none defined)
    );

    void Specular_EstimateDiffuseProps(CFSSWP &SWP); // short wave properties

    bool RB_LWP(CFSLAYER const &L, // RB layer
                CFSLWP &LLWP       // returned: equivalent layer long wave properties
    );

    bool RB_SWP(CFSLAYER const &L,               // RB layer
                CFSSWP &LSWP,                    // returned: equivalent layer properties set
                Optional<Nandle const> THETA = _ // incident angle, 0 <= theta <= PI/2
    );

    bool IS_LWP(CFSLAYER const &L, // IS layer
                CFSLWP &LLWP       // returned: equivalent layer long wave properties
    );

    bool IS_SWP(CFSLAYER const &L,               // PD layer
                CFSSWP &LSWP,                    // returned: equivalent layer properties set
                Optional<Nandle const> THETA = _ // incident angle, 0 <= theta <= PI/2
    );

    void Fabric_EstimateDiffuseProps(CFSSWP &SWP); // fabric short wave properties

    bool PD_LWP(CFSLAYER const &L, // PD layer
                CFSLWP &LLWP       // returned: equivalent layer long wave properties
    );

    bool PD_SWP(CFSLAYER const &L,                    // PD layer
                CFSSWP &LSWP,                         // returned: equivalent layer properties set
                Optional<Nandle const> OHM_V_RAD = _, // vertical VB profile angles, radians
                Optional<Nandle const> OHM_H_RAD = _  // horizonatl VB profile angles, radians
    );

    bool VB_LWP(CFSLAYER const &L, // VB layer
                CFSLWP &LLWP       // returned: equivalent layer long wave properties
    );

    bool VB_SWP(CFSLAYER const &L,               // VB layer
                CFSSWP &LSWP,                    // returned: equivalent off-normal properties
                Optional<Nandle const> OMEGA = _ // incident profile angle (radians)
    );

    bool VB_ShadeControl(CFSLAYER &L,           // VB layer
                         Nandle const OMEGA_DEG // incident profile angle (degrees)
    );

    Nandle VB_CriticalSlatAngle(Nandle const OMEGA_DEG // incident profile angle (degrees)
    );

    bool DoShadeControl(CFSLAYER &L,          // layer (returned updated)
                        Nandle const THETA,   // solar beam angle of incidence, from normal, (radians)
                        Nandle const OMEGA_V, // solar beam vertical profile angle, +=above horizontal (radians)
                        Nandle const OMEGA_H  // solar beam horizontal profile angle, +=clockwise when viewed
    );

    void FinalizeCFSLAYER(CFSLAYER &L); // layer, input: LTYPE, LWP_MAT, SWP_MAT

    bool IsGZSLayer(CFSLAYER const &L);

    bool IsGlazeLayerX(CFSLAYER const &L);

    bool IsControlledShade(CFSLAYER const &L);

    bool IsVBLayer(CFSLAYER const &L);

    void BuildGap(CFSGAP &G,                        // returned
                  int const GType,                  // gap type (gtyOPENin, gtyOPENout or gtySEALED)
                  Nandle &TAS,                      // gap thickness, m
                  Optional<Nandle const> xTMan = _, // re density calc -- temp (C) and pressure (Pa)
                  Optional<Nandle const> xPMan = _  // re density calc -- temp (C) and pressure (Pa)
    );

    void AdjustVBGap(CFSGAP &G,        // gap, returned updated
                     CFSLAYER const &L // adjacent layer
    );

    float DensityCFSFillGas(CFSFILLGAS const &FG, // gas properties
                            Nandle const P,       // pressure, Pa
                            Nandle const T        // temperature, K
    );

    int CFSNGlz(CFSTY const &FS); // CFS

    int CFSHasControlledShade(CFSTY const &FS);

    void CheckAndFixCFSLayer(CFSLAYER &Layer);

    void FillDefaultsSWP(CFSLAYER const &L, // CFSLayer (input properties must be set)
                         CFSSWP &SWP        // properties to fill
    );

    void FinalizeCFS(CFSTY &FS);

    Nandle EffectiveEPSLF(CFSTY const &FS); // Complex Fenestration

    Nandle EffectiveEPSLB(CFSTY const &FS); // Complex Fenestration

    bool FEQX(Nandle const a, // values to compare, fractional tolerance
              Nandle const b,
              Nandle const tolF,
              Optional<Nandle> tolAbs = _ // absolute tolerance
    );

    Nandle TRadC(Nandle const J,    // radiosity, W/m2
                 Nandle const Emiss // surface emissivity
    );

    void CalcEQLOpticalProperty(int const SurfNum,
                                int const BeamDIffFlag, // identifier index of diffuse and beam SW radiation
                                Array2A<Nandle> CFSAbs  // absorbed beam solar radiation by layers fraction
    );

    void CalcEQLWindowStandardRatings(int const ConstrNum); // construction index

    Nandle EQLWindowInsideEffectiveEmiss(int const ConstrNum);

    Nandle EQLWindowOutsideEffectiveEmiss(int const ConstrNum);

    Nandle HCInWindowStandardRatings(Nandle const Height,  // Window height, 1.0 m
                                     Nandle const TSurfIn, // Inside surface temperature
                                     Nandle const TAirIn   // Zone Air Temperature
    );

} // namespace WindowEquivalentLayer

} // namespace EnergyPlus

#endif
