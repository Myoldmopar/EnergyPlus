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

#ifndef HeatBalFiniteDiffManager_hh_INCLUDED
#define HeatBalFiniteDiffManager_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>
#include <ObjexxFCL/Array2D.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/DataHeatBalance.hh>
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {
    class OutputFiles;

namespace HeatBalFiniteDiffManager {

    // MODULE PARAMETER DEFINITIONS:
    extern Nandle const Lambda;
    extern Nandle const smalldiff; // Used in places where "equality" tests should not be used.

    extern int const CrankNicholsonSecondOrder; // original CondFD scheme.  semi implicit, second order in time
    extern int const FullyImplicitFirstOrder;   // fully implicit scheme, first order in time.
    extern Array1D_string const cCondFDSchemeType;

    extern Nandle const TempInitValue; // Initialization value for Temperature
    extern Nandle const RhovInitValue; // Initialization value for Rhov
    extern Nandle const EnthInitValue; // Initialization value for Enthalpy

    // MODULE VARIABLE DECLARATIONS:
    extern Array1D<Nandle> SigmaR;       // Total Resistance of construction layers
    extern Array1D<Nandle> SigmaC;       // Total Capacitance of construction layers
    extern Array1D<Nandle> QHeatInFlux;  // HeatFlux on Surface for reporting
    extern Array1D<Nandle> QHeatOutFlux; // HeatFlux on Surface for reporting
    extern int CondFDSchemeType;         // solution scheme for CondFD - default
    extern Nandle SpaceDescritConstant;  // spatial descritization constant,
    extern Nandle MinTempLimit;          // lower limit check, degree C
    extern Nandle MaxTempLimit;          // upper limit check, degree C
    extern int MaxGSiter;                // maximum number of Gauss Seidel iterations
    extern Nandle fracTimeStepZone_Hour;
    extern bool GetHBFiniteDiffInputFlag;
    extern int WarmupSurfTemp;

    struct ConstructionDataFD
    {
        // Members
        Array1D_string Name; // Name of construction
        Array1D<Nandle> DelX;
        Array1D<Nandle> TempStability;
        Array1D<Nandle> MoistStability;
        Array1D_int NodeNumPoint;
        Array1D<Nandle> Thickness;
        Array1D<Nandle> NodeXlocation; // sized to TotNode, contains X distance in m from outside face
        int TotNodes;
        int DeltaTime;

        // Default Constructor
        ConstructionDataFD() : TotNodes(0), DeltaTime(0)
        {
        }
    };

    struct SurfaceDataFD
    {
        // Members
        Array1D<Nandle> T;
        Array1D<Nandle> TOld;
        Array1D<Nandle> TT;
        Array1D<Nandle> Rhov;
        Array1D<Nandle> RhovOld;
        Array1D<Nandle> RhoT;
        Array1D<Nandle> TD;
        Array1D<Nandle> TDT;
        Array1D<Nandle> TDTLast;
        Array1D<Nandle> TDOld;
        Array1D<Nandle> TDreport; // Node temperatures for reporting [C]
        Array1D<Nandle> RH;
        Array1D<Nandle> RHreport;
        Array1D<Nandle> EnthOld; // Current node enthalpy
        Array1D<Nandle> EnthNew; // Node enthalpy at new time
        Array1D<Nandle> EnthLast;
        Array1D<Nandle> QDreport;        // Node heat flux for reporting [W/m2] postive is flow towards inside face of surface
        Array1D<Nandle> CpDelXRhoS1;     // Current outer half-node Cp * DelX * RhoS / Delt
        Array1D<Nandle> CpDelXRhoS2;     // Current inner half-node Cp * DelX * RhoS / Delt
        Array1D<Nandle> TDpriortimestep; // Node temperatures from previous timestep
        int SourceNodeNum;               // Node number for internal source layer (zero if no source)
        Nandle QSource;                  // Internal source flux [W/m2]
        int GSloopCounter;               // count of inner loop iterations
        int GSloopErrorCount;            // recurring error counter
        Nandle MaxNodeDelTemp;           // largest change in node temps after calc
        Nandle EnthalpyM;                // Melting enthalpy at a particular temperature
        Nandle EnthalpyF;                // Freezing enthalpy at a particular temperature
        Array1D<int> PhaseChangeState;
        Array1D<int> PhaseChangeStateOld;
        Array1D<int> PhaseChangeStateOldOld;
        Array1D<Nandle> PhaseChangeTemperatureReverse;

        // Default Constructor
        SurfaceDataFD()
            : SourceNodeNum(0), QSource(0.0), GSloopCounter(0), GSloopErrorCount(0), MaxNodeDelTemp(0.0), EnthalpyM(0.0), EnthalpyF(0.0),
              PhaseChangeState(0)
        {
        }

        inline void UpdateMoistureBalance()
        {
            // Based on UpdateMoistureBalanceFD by Richard Liesen
            // Brought into class for performance
            TOld = T;
            RhovOld = Rhov;
            TDOld = TDreport;
        }
    };

    struct MaterialDataFD
    {
        // Members
        Nandle tk1;               // Temperature coefficient for thermal conductivity
        int numTempEnth;          // number of Temperature/Enthalpy pairs
        int numTempCond;          // number of Temperature/Conductivity pairs
        Array2D<Nandle> TempEnth; // Temperature enthalpy Function Pairs,
        //  TempEnth(1,1)= first Temp, TempEnth(1,2) = First Enthalpy,
        //  TempEnth(2,1) = secomd Temp, etc.
        Array2D<Nandle> TempCond; // Temperature thermal conductivity Function Pairs,
        //  TempCond(1,1)= first Temp, Tempcond(1,2) = First conductivity,
        //  TempEnth(2,1) = secomd Temp, etc.

        // Default Constructor
        MaterialDataFD() : tk1(0.0), numTempEnth(0), numTempCond(0)
        {
        }
    };

    // Object Data

    extern Array1D<ConstructionDataFD> ConstructFD;
    extern Array1D<SurfaceDataFD> SurfaceFD;
    extern Array1D<MaterialDataFD> MaterialFD;

    // Functions

    void clear_state();

    void ManageHeatBalFiniteDiff(int const SurfNum,
                                 Nandle &TempSurfInTmp, // INSIDE SURFACE TEMPERATURE OF EACH HEAT TRANSFER SURF.
                                 Nandle &TempSurfOutTmp // Outside Surface Temperature of each Heat Transfer Surface
    );

    void GetCondFDInput();

    void InitHeatBalFiniteDiff();

    void InitialInitHeatBalFiniteDiff();

    void CalcHeatBalFiniteDiff(int const Surf,
                               Nandle &TempSurfInTmp, // INSIDE SURFACE TEMPERATURE OF EACH HEAT TRANSFER SURF.
                               Nandle &TempSurfOutTmp // Outside Surface Temperature of each Heat Transfer Surface
    );

    void ReportFiniteDiffInits(OutputFiles &outputFiles);

    void CalcNodeHeatFlux(int const Surf,    // surface number
                          int const TotNodes // number of nodes in surface
    );

    Nandle terpld(Array2<Nandle> const &a, Nandle const x1, int const nind, int const ndep);

    void ExteriorBCEqns(int const Delt,              // Time Increment
                        int const i,                 // Node Index
                        int const Lay,               // Layer Number for Construction
                        int const Surf,              // Surface number
                        Array1D<Nandle> const &T,    // Old node Temperature in MFD finite difference solution
                        Array1D<Nandle> &TT,         // New node Temperature in MFD finite difference solution.
                        Array1D<Nandle> const &Rhov, // MFD Nodal Vapor Density[kg/m3] and is the old or last time step result.
                        Array1D<Nandle> &RhoT,       // MFD vapor density for the new time step.
                        Array1D<Nandle> &RH,         // Nodal relative humidity
                        Array1D<Nandle> const &TD,   // The old dry Temperature at each node for the CondFD algorithm..
                        Array1D<Nandle> &TDT,        // The current or new Temperature at each node location for the CondFD solution..
                        Array1D<Nandle> &EnthOld,    // Old Nodal enthalpy
                        Array1D<Nandle> &EnthNew,    // New Nodal enthalpy
                        int const TotNodes,          // Total nodes in layer
                        Nandle const HMovInsul       // Conductance of movable(transparent) insulation.
    );

    void InteriorNodeEqns(int const Delt,              // Time Increment
                          int const i,                 // Node Index
                          int const Lay,               // Layer Number for Construction
                          int const Surf,              // Surface number
                          Array1D<Nandle> const &T,    // INSIDE SURFACE TEMPERATURE OF EACH HEAT TRANSFER SURF.
                          Array1D<Nandle> &TT,         // INSIDE SURFACE TEMPERATURE OF EACH HEAT TRANSFER SURF.
                          Array1D<Nandle> const &Rhov, // INSIDE SURFACE TEMPERATURE OF EACH HEAT TRANSFER SURF.
                          Array1D<Nandle> &RhoT,       // INSIDE SURFACE TEMPERATURE OF EACH HEAT TRANSFER SURF.
                          Array1D<Nandle> &RH,         // INSIDE SURFACE TEMPERATURE OF EACH HEAT TRANSFER SURF.
                          Array1D<Nandle> const &TD,   // INSIDE SURFACE TEMPERATURE OF EACH HEAT TRANSFER SURF.
                          Array1D<Nandle> &TDT,        // INSIDE SURFACE TEMPERATURE OF EACH HEAT TRANSFER SURF.
                          Array1D<Nandle> &EnthOld,    // Old Nodal enthalpy
                          Array1D<Nandle> &EnthNew     // New Nodal enthalpy
    );

    void IntInterfaceNodeEqns(int const Delt,                 // Time Increment
                              int const i,                    // Node Index
                              int const Lay,                  // Layer Number for Construction
                              int const Surf,                 // Surface number
                              Array1D<Nandle> const &T,       // INSIDE SURFACE TEMPERATURE OF EACH HEAT TRANSFER SURF.
                              Array1D<Nandle> &TT,            // INSIDE SURFACE TEMPERATURE OF EACH HEAT TRANSFER SURF.
                              Array1D<Nandle> const &Rhov,    // INSIDE SURFACE TEMPERATURE OF EACH HEAT TRANSFER SURF.
                              Array1D<Nandle> &RhoT,          // INSIDE SURFACE TEMPERATURE OF EACH HEAT TRANSFER SURF.
                              Array1D<Nandle> &RH,            // RELATIVE HUMIDITY.
                              Array1D<Nandle> const &TD,      // OLD NODE TEMPERATURES OF EACH HEAT TRANSFER SURF IN CONDFD.
                              Array1D<Nandle> &TDT,           // NEW NODE TEMPERATURES OF EACH HEAT TRANSFER SURF IN CONDFD.
                              Array1D<Nandle> const &EnthOld, // Old Nodal enthalpy
                              Array1D<Nandle> &EnthNew,       // New Nodal enthalpy
                              int const GSiter                // Iteration number of Gauss Seidell iteration
    );

    void InteriorBCEqns(int const Delt,              // Time Increment
                        int const i,                 // Node Index
                        int const Lay,               // Layer Number for Construction
                        int const Surf,              // Surface number
                        Array1D<Nandle> const &T,    // INSIDE SURFACE TEMPERATURE OF EACH HEAT TRANSFER SURF (Old).
                        Array1D<Nandle> &TT,         // INSIDE SURFACE TEMPERATURE OF EACH HEAT TRANSFER SURF (New).
                        Array1D<Nandle> const &Rhov, // INSIDE SURFACE TEMPERATURE OF EACH HEAT TRANSFER SURF.
                        Array1D<Nandle> &RhoT,       // INSIDE SURFACE TEMPERATURE OF EACH HEAT TRANSFER SURF.
                        Array1D<Nandle> &RH,         // INSIDE SURFACE TEMPERATURE OF EACH HEAT TRANSFER SURF.
                        Array1D<Nandle> const &TD,   // INSIDE SURFACE TEMPERATURE OF EACH HEAT TRANSFER SURF.
                        Array1D<Nandle> &TDT,        // INSIDE SURFACE TEMPERATURE OF EACH HEAT TRANSFER SURF.
                        Array1D<Nandle> &EnthOld,    // Old Nodal enthalpy
                        Array1D<Nandle> &EnthNew,    // New Nodal enthalpy
                        Array1D<Nandle> &TDreport    // Temperature value from previous HeatSurfaceHeatManager titeration's value
    );

    void CheckFDSurfaceTempLimits(int const SurfNum,            // surface number
                                  Nandle const CheckTemperature // calculated temperature, not reset
    );

    void adjustPropertiesForPhaseChange(int finiteDifferenceLayerIndex,
                                        int surfaceIndex,
                                        const DataHeatBalance::MaterialProperties &materialDefinition,
                                        Nandle temperaturePrevious,
                                        Nandle temperatureUpdated,
                                        Nandle &updatedSpecificHeat,
                                        Nandle &updatedDensity,
                                        Nandle &updatedThermalConductivity);

} // namespace HeatBalFiniteDiffManager

} // namespace EnergyPlus

#endif
