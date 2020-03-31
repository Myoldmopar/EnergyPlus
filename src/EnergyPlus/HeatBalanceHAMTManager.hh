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

#ifndef HeatBalanceHAMTManager_hh_INCLUDED
#define HeatBalanceHAMTManager_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Optional.hh>

// EnergyPlus Headers
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {
    class OutputFiles;

namespace HeatBalanceHAMTManager {

    // Data
    // MODULE PARAMETER DEFINITIONS:
    extern int const ittermax; // Maximum Number of itterations
    extern int const adjmax;   // Maximum Number of Adjacent Cells

    extern Nandle const wdensity; // Density of water kg.m-3
    extern Nandle const wspech;   // Specific Heat Capacity of Water J.kg-1.K-1 (at 20C)
    extern Nandle const whv;      // Evaporation enthalpy of water J.kg-1
    extern Nandle const convt;    // Temperature convergence limit
    extern Nandle const qvplim;   // Maximum latent heat W
    extern Nandle const rhmax;    // Maximum RH value

    // DERIVED TYPE DEFINITIONS:

    // MODULE VARIABLE DECLARATIONS:
    extern Array1D_int firstcell;
    extern Array1D_int lastcell;
    extern Array1D_int Extcell;
    extern Array1D_int ExtRadcell;
    extern Array1D_int ExtConcell;
    extern Array1D_int ExtSkycell;
    extern Array1D_int ExtGrncell;
    extern Array1D_int Intcell;
    extern Array1D_int IntConcell;

    extern Array1D<Nandle> watertot;
    extern Array1D<Nandle> surfrh;
    extern Array1D<Nandle> surfextrh;
    extern Array1D<Nandle> surftemp;
    extern Array1D<Nandle> surfexttemp;
    extern Array1D<Nandle> surfvp;

    extern Array1D<Nandle> extvtc;   // External Surface vapor transfer coefficient
    extern Array1D<Nandle> intvtc;   // Internal Surface Vapor Transfer Coefficient
    extern Array1D_bool extvtcflag;  // External Surface vapor transfer coefficient flag
    extern Array1D_bool intvtcflag;  // Internal Surface Vapor Transfer Coefficient flag
    extern Array1D_bool MyEnvrnFlag; // Flag to reset surface properties.

    extern Nandle deltat; // time step in seconds

    extern int TotCellsMax; // Maximum number of cells per material

    extern bool latswitch;  // latent heat switch,
    extern bool rainswitch; // rain switch,

    // SUBROUTINE SPECIFICATIONS FOR MODULE HeatBalanceHAMTManager:

    // Types

    struct subcell
    {
        // Members
        int matid;        // Material Id Number
        int sid;          // Surface Id Number
        Nandle Qadds;     // Additional sources of heat
        Nandle density;   // Density
        Nandle wthermalc; // Moisture Dependant Thermal Conductivity
        Nandle spech;     // Specific Heat capacity
        Nandle htc;       // Heat Transfer Coefficient
        Nandle vtc;       // Vapor Transfer Coefficient
        Nandle mu;        // Vapor Diffusion resistance Factor
        Nandle volume;    // Cell Volume
        Nandle temp;
        Nandle tempp1;
        Nandle tempp2;
        Nandle wreport; // Water content for reporting
        Nandle water;   // Water Content of cells
        Nandle vp;      // Vapor Pressure
        Nandle vpp1;    // Vapor Pressure
        Nandle vpsat;   // Saturation Vapor Pressure
        Nandle rh;
        Nandle rhp1;
        Nandle rhp2;             // Relative Humidity
        Nandle rhp;              // cell relative humidity (percent - reporting)
        Nandle dwdphi;           // Moisture storage capacity
        Nandle dw;               // Liquid transport Coefficient
        Array1D<Nandle> origin;  // Cell origin. The geometric centre of the cell.
        Array1D<Nandle> length;  // Cell lengths
        Array1D<Nandle> overlap; // Area of overlap
        Array1D<Nandle> dist;    // distance between cell origins
        Array1D_int adjs;
        Array1D_int adjsl;

        // Default Constructor
        subcell()
            : matid(-1), sid(-1), Qadds(0.0), density(-1.0), wthermalc(0.0), spech(0.0), htc(-1.0), vtc(-1.0), mu(-1.0), volume(0.0), temp(0.0),
              tempp1(0.0), tempp2(0.0), wreport(0.0), water(0.0), vp(0.0), vpp1(0.0), vpsat(0.0), rh(0.1), rhp1(0.1), rhp2(0.1), rhp(10.0),
              dwdphi(-1.0), dw(-1.0), origin(3, 0.0), length(3, 0.0), overlap(6, 0.0), dist(6, 0.0), adjs(6, 0), adjsl(6, 0)
        {
        }
    };

    // Object Data
    extern Array1D<subcell> cells;

    // Functions

    void ManageHeatBalHAMT(int const SurfNum, Nandle &TempSurfInTmp, Nandle &TempSurfOutTmp);

    void GetHeatBalHAMTInput();

    void InitHeatBalHAMT(EnergyPlus::OutputFiles &outputFiles);

    void CalcHeatBalHAMT(int const sid, Nandle &TempSurfInTmp, Nandle &TempSurfOutTmp);

    void UpdateHeatBalHAMT(int const sid);

    void
    interp(int const ndata, const Array1D<Nandle> &xx, const Array1D<Nandle> &yy, Nandle const invalue, Nandle &outvalue, Optional<Nandle> outgrad = _);

    Nandle RHtoVP(Nandle const RH, Nandle const Temperature);

    Nandle WVDC(Nandle const Temperature, Nandle const ambp);

    //                                 COPYRIGHT NOTICE

    //     Portions Copyright (c) University College London 2007.  All rights
    //     reserved.

    //     UCL LEGAL NOTICE
    //     Neither UCL, members of UCL nor any person or organisation acting on
    //     behalf of either:

    //     A. Makes any warranty of representation, express or implied with
    //        respect to the accuracy, completeness, or usefulness of the
    //        information contained in this program, including any warranty of
    //        merchantability or fitness of any purpose with respect to the
    //        program, or that the use of any information disclosed in this
    //        program may not infringe privately-owned rights, or

    //     B. Assumes any liability with respect to the use of, or for any and
    //        all damages resulting from the use of the program or any portion
    //        thereof or any information disclosed therein.

} // namespace HeatBalanceHAMTManager

} // namespace EnergyPlus

#endif
