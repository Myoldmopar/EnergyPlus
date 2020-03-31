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

#ifndef TARCOGMain_hh_INCLUDED
#define TARCOGMain_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array2A.hh>

// EnergyPlus Headers
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

namespace TARCOGMain {

    // Functions

    void TARCOG90(int const nlayer,                    // Number of layers (glass + SD)
                  int const iwd,                       // Wind direction:
                  Nandle &tout,                        // Outdoor temperature [K]
                  Nandle &tind,                        // Indoor temperature [K]
                  Nandle &trmin,                       // Indoor mean radiant temperature [K]
                  Nandle const wso,                    // Outdoor wind speed [m/s]
                  Nandle const wsi,                    // Inside forced air speed [m/s]
                  Nandle const dir,                    // Direct solar radiation [W/m2]
                  Nandle const outir,                  // IR radiance of window's exterior surround [W/m2]
                  int const isky,                      // Flag for sky temperature(Tsky) and sky emittance(esky)
                  Nandle const tsky,                   // Night sky temperature [K]
                  Nandle &esky,                        // Effective night sky emittance
                  Nandle const fclr,                   // Fraction of sky that is clear
                  Nandle const VacuumPressure,         // maximal pressure for gas to be considered as vacuum
                  Nandle &VacuumMaxGapThickness,       // maximum allowed thickness without producing warning message
                  int const CalcDeflection,            // Deflection calculation flag:
                  Nandle const Pa,                     // Atmospheric (outside/inside) pressure (used onlu if CalcDeflection = 1)
                  Nandle const Pini,                   // Initial presssure at time of fabrication (used only if CalcDeflection = 1)
                  Nandle const Tini,                   // Initial temperature at time of fabrication (used only if CalcDeflection = 1)
                  Array1D<Nandle> &gap,                 // Vector of gap widths [m]
                  Array1D<Nandle> &GapDefMax,           // Vector of gap widths in deflected state. It will be used as input
                  Array1D<Nandle> &thick,               // Vector of glazing layer thicknesses [m]
                  Array1D<Nandle> &scon,                // Vector of conductivities of each glazing layer  [W/mK]
                  const Array1D<Nandle> &YoungsMod,     // Youngs Modulus coefficients used in deflection calculations
                  const Array1D<Nandle> &PoissonsRat,   // Poissons Ratio coefficients used in deflection calculations
                  const Array1D<Nandle> &tir,           // Vector of IR transmittances of each surface
                  const Array1D<Nandle> &emis,          // Vector of IR emittances of each surface
                  Nandle const totsol,                 // Total solar transmittance of the IGU
                  Nandle const tilt,                   // Window tilt [degrees]
                  const Array1D<Nandle> &asol,          // Vector of Absorbed solar energy fractions for each layer
                  Nandle const height,                 // IGU cavity height
                  Nandle const heightt,                // Window height
                  Nandle const width,                  // Window width
                  const Array1D<Nandle> &presure,       // Vector of gas pressures in gaps [N/m2]
                  Array2A_int const iprop,             // Matrix of gas codes - see mgas definition
                  Array2A<Nandle> const frct,          // Matrix of mass percentages in gap mixtures
                  Array2A<Nandle> const xgcon,         // Matrix of constants for gas conductivity calc
                  Array2A<Nandle> const xgvis,         // Matrix of constants for gas dynamic viscosity calc
                  Array2A<Nandle> const xgcp,          // Matrix of constants for gas specific heat calc at constant pressure
                  const Array1D<Nandle> &xwght,         // Vector of Molecular weights for gasses
                  const Array1D<Nandle> &gama,          // Vector of spefic heat ration for low pressure calc
                  const Array1D_int &nmix,              // Vector of number of gasses in gas mixture of each gap
                  const Array1D_int &SupportPillar,     // Shows whether or not gap have support pillar
                  const Array1D<Nandle> &PillarSpacing, // Pillar spacing for each gap (used in case there is support pillar)
                  const Array1D<Nandle> &PillarRadius,  // Pillar radius for each gap (used in case there is support pillar)
                  Array1D<Nandle> &theta,               // Vector of average temperatures of glazing surfaces [K]
                  Array1D<Nandle> &LayerDef,            // Vector of layers deflection. [m]
                  Array1D<Nandle> &q,                   // Vector of various heat fluxes [W/m2]
                  Array1D<Nandle> &qv,                  // Vector of heat fluxes to each gap by ventillation [W/m2]
                  Nandle &ufactor,                     // Center of glass U-value [W/m2 K]
                  Nandle &sc,                          // Shading Coefficient
                  Nandle &hflux,                       // Net heat flux between room and window [W/m2]
                  Nandle &hcin,                        // Indoor convective surface heat transfer coefficient  [W/m2 K]
                  Nandle &hcout,                       // Outdoor convective surface heat transfer coefficient [W/m2 K]
                  Nandle &hrin,                        // Indoor radiative surface heat transfer coefficient [W/m2 K]
                  Nandle &hrout,                       // Outdoor radiative surface heat transfer coefficient [W/m2 K]
                  Nandle &hin,                         // Indoor combined film coefficient (if non-zero) [W/m2K]
                  Nandle &hout,                        // Outdoor combined film coefficient (if non-zero) [W/m2K]
                  Array1D<Nandle> &hcgas,               // Convective part of gap effective conductivity (including in and out)
                  Array1D<Nandle> &hrgas,               // Radiative part of gap effective conductivity (including in and out)
                  Nandle &shgc,                        // Solar heat gain coefficient - per ISO 15099
                  int &nperr,                          // Error code
                  std::string &ErrorMessage,           // To store error message from tarcog execution
                  Nandle &shgct,                       // Solar heat gain coefficient - per old procedure
                  Nandle &tamb,                        // Outdoor environmental temperature [K]
                  Nandle &troom,                       // Indoor environmental temperature [K]
                  const Array1D_int &ibc,               // Vector of boundary condition flags (ibc(1) - outdoor, ibc(2) - indoor
                  const Array1D<Nandle> &Atop,         // Vector with areas of top openings - between SD layers and top of
                  const Array1D<Nandle> &Abot,         // Vector with areas of bottom openings - between SD layers and
                  const Array1D<Nandle> &Al,           // Vector with areas of left-hand side openings - between SD layers and
                  const Array1D<Nandle> &Ar,           // Vector of areas of right-hand side openings - between SD layers and
                  const Array1D<Nandle> &Ah,           // Vector of total areas of holes for each SD [m2]
                  const Array1D<Nandle> &SlatThick,    // Thickness of the slat material [m]
                  const Array1D<Nandle> &SlatWidth,    // Slat width [m]
                  const Array1D<Nandle> &SlatAngle,    // Slat tilt angle [deg]
                  const Array1D<Nandle> &SlatCond,     // Conductivity of the slat material [W/m.K]
                  const Array1D<Nandle> &SlatSpacing,  // Distance between slats [m]
                  const Array1D<Nandle> &SlatCurve,    // Curvature radius of the slat [m]
                  const Array1D<Nandle> &vvent,        // Vector of velocities for forced ventilation, for each gap, and for
                  const Array1D<Nandle> &tvent,        // Vector of temperatures of ventilation gas for forced ventilation,
                  const Array1D_int &LayerType,         // Glazing layer type flag
                  const Array1D_int &nslice,            // Vector of numbers of slices in a laminated glazing layers
                  const Array1D<Nandle> &LaminateA,    // Left-hand side array for creating slice equations
                  const Array1D<Nandle> &LaminateB,    // Right-hand side array for creating slice equations
                  const Array1D<Nandle> &sumsol,       // Array of absorbed solar energy fractions for each laminated
                  Array1D<Nandle> &hg,                  // Gas conductance of the glazing cavity [W/m2 K]
                  Array1D<Nandle> &hr,                  // Radiation conductance of the glazing cavity [W/m2 K]
                  Array1D<Nandle> &hs,                  // Thermal conductance of the glazing cavity [W/m2 K]
                  Nandle &he,                          // External heat transfer coefficient [W/m2 K] - EN673 and ISO 10292 procedure
                  Nandle &hi,                          // Internal heat transfer coefficient [W/m2 K] - EN673 and ISO 10292 procedure
                  Array1D<Nandle> &Ra,                  // Vector of Rayleigh numbers, for each gap
                  Array1D<Nandle> &Nu,                  // Vector of Nusselt numbers, for each gap
                  int const standard,                  // Calculation standard switch:
                  int const ThermalMod,                // Thermal model:
                  int const Debug_mode,                // Switch for debug output files:
                  std::string const &Debug_dir,        // Target directory for debug files
                  std::string const &Debug_file,       // File name template for debug files
                  int const win_ID,                    // ID of window (passed by W6)
                  int const igu_ID,                    // ID of the IGU (passed by W6)
                  Nandle &ShadeEmisRatioOut,           // Ratio of modified to glass emissivity at the outermost glazing surface
                  Nandle &ShadeEmisRatioIn,            // Ratio of modified to glass emissivity at the innermost glazing surface
                  Nandle &ShadeHcRatioOut,             // Ratio of modified to unshaded Hc at the outermost glazing surface
                  Nandle &ShadeHcRatioIn,              // Ratio of modified to unshaded Hc at the innermost glazing surface
                  Nandle &HcUnshadedOut,               // Hc value at outermost glazing surface of an unshaded subsystem [W/m2 K]
                  Nandle &HcUnshadedIn,                // Hc value at innermost glazing surface of an unshaded subsystem [W/m2 K]
                  Array1D<Nandle> &Keff,                // Vector of keff values for gaps [W/m.K]
                  Array1D<Nandle> &ShadeGapKeffConv,    // Vector of convective keff values for areas above/below
                  Nandle const SDScalar,               // Factor of Venetian SD layer contribution to convection
                  int const SHGCCalc,                  // SHGC calculation switch:
                  int &NumOfIterations,                // Number of iterations for reacing solution
                  Nandle const edgeGlCorrFac           // Edge of glass correction factor
    );

} // namespace TARCOGMain

} // namespace EnergyPlus

#endif
