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

#ifndef PondGroundHeatExchanger_hh_INCLUDED
#define PondGroundHeatExchanger_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>
#include <EnergyPlus/Plant/PlantLocation.hh>
#include <EnergyPlus/PlantComponent.hh>

namespace EnergyPlus {

namespace PondGroundHeatExchanger {

    extern Nandle const SmallNum;      // Very small number to avoid div0 errors
    extern Nandle const StefBoltzmann; // Stefan-Boltzmann constant

    extern int NumOfPondGHEs; // Number of pond ground heat exchangers

    struct PondGroundHeatExchangerData : PlantComponent
    {
        // Members
        std::string Name;           // name of pond GHE
        std::string InletNode;      // pond inlet fluid node
        std::string OutletNode;     // pond outlet fluid node
        Nandle DesignMassFlowRate;  // design flow rate of circulating fluid
        Nandle DesignCapacity;      // design cooling capacity of pond at
        Nandle Depth;               // depth of pond
        Nandle Area;                // area of pond
        Nandle TubeInDiameter;      // hydronic tube inside diameter
        Nandle TubeOutDiameter;     // hydronic tube outside diameter
        Nandle TubeConductivity;    // hydronic tube thermal conductivity
        Nandle GrndConductivity;    // ground thermal conductivity
        Nandle CircuitLength;       // length of each circuit
        Nandle BulkTemperature;     // current pond bulk temperature
        Nandle PastBulkTemperature; // past pond bulk temperature
        int NumCircuits;            // number of circuits in total
        int InletNodeNum;           // inlet node number
        int OutletNodeNum;          // oulet node number
        int FrozenErrIndex;         // for recurring warnings
        int ConsecutiveFrozen;      // count of time steps consecutive frozen
        // loop topology variables
        int LoopNum;
        int LoopSideNum;
        int BranchNum;
        int CompNum;

        // Report data
        Nandle InletTemp;        // fluid inlet temperature
        Nandle OutletTemp;       // fluid outlet temperature
        Nandle MassFlowRate;     // fluid mass flow rate
        Nandle PondTemp;         // pond bulk temperature
        Nandle HeatTransferRate; // total fluid heat transfer rate, Watts
        Nandle Energy;           // cumulative energy, Joules

        bool OneTimeFlag;
        bool MyFlag;
        bool setupOutputVarsFlag;

        int WaterIndex;

        // Default Constructor
        PondGroundHeatExchangerData()
            : DesignMassFlowRate(0.0), DesignCapacity(0.0), Depth(0.0), Area(0.0), TubeInDiameter(0.0), TubeOutDiameter(0.0), TubeConductivity(0.0),
              GrndConductivity(0.0), CircuitLength(0.0), BulkTemperature(0.0), PastBulkTemperature(0.0), NumCircuits(0), InletNodeNum(0),
              OutletNodeNum(0), FrozenErrIndex(0), ConsecutiveFrozen(0), LoopNum(0), LoopSideNum(0), BranchNum(0), CompNum(0), InletTemp(0.0),
              OutletTemp(0.0), MassFlowRate(0.0), PondTemp(0.0), HeatTransferRate(0.0), Energy(0.0), OneTimeFlag(true), MyFlag(true),
              setupOutputVarsFlag(true), WaterIndex(0)
        {
        }

        void simulate(const PlantLocation &calledFromLocation, bool FirstHVACIteration, Nandle &CurLoad, bool RunFlag) override;

        static PlantComponent *factory(std::string const &objectName);

        void getDesignCapacities(const PlantLocation &calledFromLocation, Nandle &MaxLoad, Nandle &MinLoad, Nandle &OptLoad) override;

        void InitPondGroundHeatExchanger(bool FirstHVACIteration);

        void setupOutputVars();

        void CalcPondGroundHeatExchanger();

        Nandle CalcTotalFLux(Nandle PondBulkTemp // pond temp for this flux calculation
        );

        Nandle CalcEffectiveness(Nandle InsideTemperature, // Temperature of fluid in pipe circuit, in C
                                 Nandle PondTemperature,   // Temperature of pond water (i.e. outside the pipe), in C
                                 Nandle massFlowRate       // Mass flow rate, in kg/s
        );

        Nandle CalcSolarFlux();

        void UpdatePondGroundHeatExchanger();

        void ReportPondGroundHeatExchanger();
    };

    // Object Data
    extern Array1D<PondGroundHeatExchangerData> PondGHE;

    void GetPondGroundHeatExchanger();

} // namespace PondGroundHeatExchanger

} // namespace EnergyPlus

#endif
