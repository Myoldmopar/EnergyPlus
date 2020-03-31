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

#ifndef WaterUse_hh_INCLUDED
#define WaterUse_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>
#include <EnergyPlus/PlantComponent.hh>

namespace EnergyPlus {

namespace WaterUse {

    enum struct HeatRecoveryHXEnum
    {
        Ideal,
        CounterFlow,
        CrossFlow
    };

    enum struct HeatRecoveryConfigEnum
    {
        Plant,
        Equipment,
        PlantAndEquip
    };

    extern bool getWaterUseInputFlag;

    extern Array1D_bool CheckEquipName;

    struct WaterEquipmentType
    {
        std::string Name; // Name of DHW
        std::string EndUseSubcatName;
        int Connections;          // Index for WATER USE CONNECTIONS object
        Nandle PeakVolFlowRate;   // Peak volumetric flow rate, also water consumption rate (m3/s)
        int FlowRateFracSchedule; // Pointer to schedule object
        Nandle ColdVolFlowRate;
        Nandle HotVolFlowRate;
        Nandle TotalVolFlowRate; // Volumetric flow rate, also water consumption rate (m3/s)
        Nandle ColdMassFlowRate;
        Nandle HotMassFlowRate;
        Nandle TotalMassFlowRate; // Mass flow rate (kg/s)
        Nandle DrainMassFlowRate;
        int ColdTempSchedule;   // Index for schedule object
        int HotTempSchedule;    // Index for schedule object
        int TargetTempSchedule; // Index for schedule object
        Nandle ColdTemp;        // Cold supply water temperature (C)
        Nandle HotTemp;         // Hot supply water temperature (C)
        Nandle TargetTemp;      // Target (mixed) water temperature (C)
        Nandle MixedTemp;       // Actual outlet (mixed) water temperature (C)
        Nandle DrainTemp;
        int Zone;                 // Index for zone object
        int SensibleFracSchedule; // Pointer to schedule object
        Nandle SensibleRate;
        Nandle SensibleEnergy;
        Nandle SensibleRateNoMultiplier;
        int LatentFracSchedule; // Pointer to schedule object
        Nandle LatentRate;
        Nandle LatentEnergy;
        Nandle LatentRateNoMultiplier;
        Nandle MoistureRate;
        Nandle MoistureMass;
        Nandle ColdVolume;  // Water consumption (m3)
        Nandle HotVolume;   // Water consumption (m3)
        Nandle TotalVolume; // Water consumption (m3)
        Nandle Power;       // Heating rate required to meet the mixed water temperature (W)
        Nandle Energy;      // Heating energy required to meet the mixed water temperature (J)
        bool setupMyOutputVars;

        WaterEquipmentType()
            : Connections(0), PeakVolFlowRate(0.0), FlowRateFracSchedule(0), ColdVolFlowRate(0.0), HotVolFlowRate(0.0), TotalVolFlowRate(0.0),
              ColdMassFlowRate(0.0), HotMassFlowRate(0.0), TotalMassFlowRate(0.0), DrainMassFlowRate(0.0), ColdTempSchedule(0), HotTempSchedule(0),
              TargetTempSchedule(0), ColdTemp(0.0), HotTemp(0.0), TargetTemp(0.0), MixedTemp(0.0), DrainTemp(0.0), Zone(0), SensibleFracSchedule(0),
              SensibleRate(0.0), SensibleEnergy(0.0), SensibleRateNoMultiplier(0.0), LatentFracSchedule(0), LatentRate(0.0), LatentEnergy(0.0),
              LatentRateNoMultiplier(0.0), MoistureRate(0.0), MoistureMass(0.0), ColdVolume(0.0), HotVolume(0.0), TotalVolume(0.0), Power(0.0),
              Energy(0.0), setupMyOutputVars(true)
        {
        }

        // Reset Some Values to Zeros
        void reset()
        {
            SensibleRate = 0.0;
            SensibleEnergy = 0.0;
            LatentRate = 0.0;
            LatentEnergy = 0.0;
            MixedTemp = 0.0;
            TotalMassFlowRate = 0.0;
            DrainTemp = 0.0;
        }

        void CalcEquipmentFlowRates();

        void CalcEquipmentDrainTemp();

        void setupOutputVars();
    };

    struct WaterConnectionsType : PlantComponent
    {
        std::string Name; // Name of DHW
        bool Init;        // Flag for initialization:  TRUE means do the init
        bool InitSizing;  // Flag for initialization of plant sizing
        bool StandAlone;  // Flag for operation with no plant connections
        int InletNode;    // Hot water demand node
        int OutletNode;   // Cold water supply node
        int SupplyTankNum;
        int RecoveryTankNum;
        int TankDemandID; // array to request flow from supply tank
        int TankSupplyID; // array to send flow to recovery tank
        bool HeatRecovery;
        HeatRecoveryHXEnum HeatRecoveryHX;
        HeatRecoveryConfigEnum HeatRecoveryConfig;
        Nandle HXUA;
        Nandle Effectiveness;
        Nandle RecoveryRate;
        Nandle RecoveryEnergy;
        Nandle MainsMassFlowRate; // Mass flow rate (kg/s)
        Nandle TankMassFlowRate;  // Mass flow rate (kg/s)
        Nandle ColdMassFlowRate;  // Mass flow rate (kg/s)  cold = mains + tank
        Nandle HotMassFlowRate;   // Mass flow rate (kg/s)
        Nandle TotalMassFlowRate; // Mass flow rate (kg/s) total = cold + hot
        Nandle DrainMassFlowRate;
        Nandle RecoveryMassFlowRate;
        Nandle PeakVolFlowRate;  // Volumetric flow rate, also water consumption rate (m3/s)
        Nandle MainsVolFlowRate; // Volumetric flow rate, also water consumption rate (m3/s)
        Nandle TankVolFlowRate;  // Volumetric flow rate, also water consumption rate (m3/s)
        Nandle ColdVolFlowRate;  // Volumetric flow rate, also water consumption rate (m3/s)
        Nandle HotVolFlowRate;   // Volumetric flow rate, also water consumption rate (m3/s)
        Nandle TotalVolFlowRate; // Volumetric flow rate, also water consumption rate (m3/s)
        Nandle DrainVolFlowRate;
        Nandle PeakMassFlowRate; // Peak Mass flow rate for MassFlowRateMax
        int ColdTempSchedule;    // Index for schedule object
        int HotTempSchedule;     // Index for schedule object
        Nandle MainsTemp;        // Cold supply water temperature (C)
        Nandle TankTemp;         // Cold supply water temperature (C)
        Nandle ColdSupplyTemp;   // cold from mains, schedule, or tank, depending
        Nandle ColdTemp;         // Cold supply water temperature (C)  actual cold (could be reheated)
        Nandle HotTemp;          // Hot supply water temperature (C)
        Nandle DrainTemp;
        Nandle RecoveryTemp;
        Nandle ReturnTemp;
        Nandle WasteTemp;
        Nandle TempError;
        Nandle MainsVolume; // Water consumption (m3)
        Nandle TankVolume;  // Water consumption (m3)
        Nandle ColdVolume;  // Water consumption (m3)
        Nandle HotVolume;   // Water consumption (m3)
        Nandle TotalVolume; // Water consumption (m3)
        Nandle Power;       // Heating rate required to raise temperature from cold to hot (W)
        Nandle Energy;      // Heating energy required to raise temperature from cold to hot (J)
        int NumWaterEquipment;
        int MaxIterationsErrorIndex; // recurring error index
        Array1D_int myWaterEquipArr;
        int PlantLoopNum;
        int PlantLoopSide;
        int PlantLoopBranchNum;
        int PlantLoopCompNum;
        bool MyEnvrnFlag;
        bool setupMyOutputVars;

        WaterConnectionsType()
            : Init(true), InitSizing(true), StandAlone(false), InletNode(0), OutletNode(0), SupplyTankNum(0), RecoveryTankNum(0), TankDemandID(0),
              TankSupplyID(0), HeatRecovery(false), HeatRecoveryHX(HeatRecoveryHXEnum::Ideal), HeatRecoveryConfig(HeatRecoveryConfigEnum::Plant),
              HXUA(0.0), Effectiveness(0.0), RecoveryRate(0.0), RecoveryEnergy(0.0), MainsMassFlowRate(0.0), TankMassFlowRate(0.0),
              ColdMassFlowRate(0.0), HotMassFlowRate(0.0), TotalMassFlowRate(0.0), DrainMassFlowRate(0.0), RecoveryMassFlowRate(0.0),
              PeakVolFlowRate(0.0), MainsVolFlowRate(0.0), TankVolFlowRate(0.0), ColdVolFlowRate(0.0), HotVolFlowRate(0.0), TotalVolFlowRate(0.0),
              DrainVolFlowRate(0.0), PeakMassFlowRate(0.0), ColdTempSchedule(0), HotTempSchedule(0), MainsTemp(0.0), TankTemp(0.0),
              ColdSupplyTemp(0.0), ColdTemp(0.0), HotTemp(0.0), DrainTemp(0.0), RecoveryTemp(0.0), ReturnTemp(0.0), WasteTemp(0.0), TempError(0.0),
              MainsVolume(0.0), TankVolume(0.0), ColdVolume(0.0), HotVolume(0.0), TotalVolume(0.0), Power(0.0), Energy(0.0), NumWaterEquipment(0),
              MaxIterationsErrorIndex(0), PlantLoopNum(0), PlantLoopSide(0), PlantLoopBranchNum(0), PlantLoopCompNum(0), MyEnvrnFlag(true),
              setupMyOutputVars(true)
        {
        }

        static PlantComponent *factory(std::string const &objectName);

        void simulate(const PlantLocation &calledFromLocation, bool FirstHVACIteration, Nandle &CurLoad, bool RunFlag) override;

        void InitConnections();

        void CalcConnectionsFlowRates(bool FirstHVACIteration);

        void CalcConnectionsDrainTemp();

        void CalcConnectionsHeatRecovery();

        void UpdateWaterConnections();

        void ReportWaterUse();

        void setupOutputVars();
    };

    void clear_state();

    void SimulateWaterUse(bool FirstHVACIteration);

    void GetWaterUseInput();

    void ReportStandAloneWaterUse();

    void CalcWaterUseZoneGains();

    extern Array1D<WaterEquipmentType> WaterEquipment;

    extern Array1D<WaterConnectionsType> WaterConnections;

} // namespace WaterUse

} // namespace EnergyPlus

#endif
