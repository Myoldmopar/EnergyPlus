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

#ifndef ENERGYPLUS_UNITARYSYSTEM_HH
#define ENERGYPLUS_UNITARYSYSTEM_HH

#include <string>
#include <vector>
#include <EnergyPlus/DataHVACSystems.hh>

namespace EnergyPlus {

namespace UnitarySystems {

    void clear_state();

    extern int numUnitarySystems;
    extern bool economizerFlag;      // holds air loop economizer status
    extern bool SuppHeatingCoilFlag; // set to TRUE when simulating supplemental heating coil

    // why are these external?
    // Last mode of operation
    extern int const CoolingMode;  // last compressor operating mode was in cooling
    extern int const HeatingMode;  // last compressor operating mode was in heating
    extern bool HeatingLoad;       // True when zone needs heating
    extern bool CoolingLoad;       // True when zone needs cooling
    extern Nandle MoistureLoad;    // Dehumidification Load (W)
    extern Nandle CompOnMassFlow;  // Supply air mass flow rate w/ compressor ON [kg/s]
    extern Nandle CompOffMassFlow; // Supply air mass flow rate w/ compressor OFF [kg/s]

    // Compressor operation
    extern int const On;  // normal compressor operation
    extern int const Off; // signal DXCoil that compressor shouldn't run

    // Coil type for SimWater and SimSteamCoil
    extern int const CoolingCoil;
    extern int const HeatingCoil;
    extern int const SuppHeatCoil;

    // Supply Air Sizing Option
    extern int const None;
    extern int const SupplyAirFlowRate;
    extern int const FlowPerFloorArea;
    extern int const FractionOfAutoSizedCoolingValue;
    extern int const FractionOfAutoSizedHeatingValue;
    extern int const FlowPerCoolingCapacity;
    extern int const FlowPerHeatingCapacity;

    struct DesignSpecMSHP
    {
        // friend class UnitarySys;

    public:
        DesignSpecMSHP(); // constructor
        ~DesignSpecMSHP() // destructor
        {
        }

        std::string name;
        static DesignSpecMSHP *factory(int object_type_of_num, std::string const objectName);
        int numOfSpeedHeating;
        int numOfSpeedCooling;
        Nandle noLoadAirFlowRateRatio;
        std::vector<Nandle> coolingVolFlowRatio; // The ratio of flow to max for this speed
        std::vector<Nandle> heatingVolFlowRatio; // The ratio of flow to max for this speed

        //    private:
        int m_DesignSpecMSHPType_Num;
        bool m_SingleModeFlag;

        static void getDesignSpecMSHP();
        static void getDesignSpecMSHPdata(bool errorsFound);
    };

    struct UnitarySys : HVACSystemData
    {

        enum class ControlType : int
        {
            None,
            Load,
            Setpoint,
            CCMASHRAE
        };

        enum class DehumCtrlType : int
        {
            None,
            CoolReheat,
            Multimode
        };

        enum class FanPlace : int
        {
            NotYetSet,
            BlowThru,
            DrawThru
        };

        // Airflow control for contant fan mode
        enum class UseCompFlow : int
        {
            FlowNotYetSet,
            UseCompressorOnFlow, // set compressor OFF air flow rate equal to compressor ON air flow rate
            UseCompressorOffFlow // set compressor OFF air flow rate equal to user defined value
        };

        // friend class DesignSpecMSHP;

        int m_UnitarySysNum;
        int m_unitarySystemType_Num;
        bool m_ThisSysInputShouldBeGotten;
        int m_SysAvailSchedPtr; // Pointer to the availability schedule
        ControlType m_ControlType;
        DehumCtrlType m_DehumidControlType_Num;
        bool m_Humidistat;
        bool m_ValidASHRAECoolCoil;
        bool m_ValidASHRAEHeatCoil;
        bool m_SimASHRAEModel; // flag denoting that ASHRAE model (SZVAV) should be used
        bool m_setFaultModelInput;
        int m_FanIndex;
        FanPlace m_FanPlace;
        int m_FanOpModeSchedPtr;
        bool m_FanExists;
        int m_FanType_Num;
        bool m_RequestAutoSize;
        Nandle m_ActualFanVolFlowRate;
        Nandle m_DesignFanVolFlowRate;
        Nandle m_DesignMassFlowRate;
        int m_FanAvailSchedPtr;
        int m_FanOpMode;
        int m_ATMixerIndex;
        int m_ATMixerPriNode;
        int m_ATMixerSecNode;
        bool m_AirLoopEquipment;
        int m_ZoneInletNode;
        int m_ZoneSequenceCoolingNum;
        int m_ZoneSequenceHeatingNum;
        bool m_HeatCoilExists;
        Nandle m_HeatingSizingRatio;
        int m_HeatingCoilType_Num;
        bool m_DXHeatingCoil;
        int m_HeatingCoilIndex;
        int m_HeatingCoilAvailSchPtr;
        Nandle m_DesignHeatingCapacity;
        Nandle m_MaxHeatAirVolFlow;
        int m_NumOfSpeedHeating;
        bool m_MultiSpeedHeatingCoil;
        bool m_VarSpeedHeatingCoil;
        int m_SystemHeatControlNodeNum;
        bool m_CoolCoilExists;
        int m_CoolingCoilType_Num;
        int m_CoolingCoilSubType_Num;
        int m_NumOfSpeedCooling;
        int m_CoolingCoilAvailSchPtr;
        Nandle m_DesignCoolingCapacity;
        Nandle m_MaxCoolAirVolFlow;
        int m_CondenserNodeNum;
        int m_CondenserType;
        int m_CoolingCoilIndex;
        bool m_HeatPump;
        int m_ActualDXCoilIndexForHXAssisted;
        bool m_DiscreteSpeedCoolingCoil;
        bool m_ContSpeedCoolingCoil;
        int m_SystemCoolControlNodeNum;
        int m_WaterCyclingMode;
        bool m_ISHundredPercentDOASDXCoil;
        bool m_RunOnSensibleLoad;
        bool m_RunOnLatentLoad;
        bool m_RunOnLatentOnlyWithSensible;
        int m_DehumidificationMode;
        int m_SuppHeatCoilType_Num;
        bool m_SuppCoilExists;
        Nandle m_DesignSuppHeatingCapacity;
        int m_SuppCoilAirInletNode;
        int m_SuppCoilAirOutletNode;
        int m_SuppCoilFluidInletNode;
        Nandle m_MaxSuppCoilFluidFlow;
        int m_SuppHeatCoilIndex;
        int m_SuppHeatControlNodeNum;
        Nandle m_SupHeaterLoad;
        int m_CoolingSAFMethod;
        int m_HeatingSAFMethod;
        int m_NoCoolHeatSAFMethod;
        Nandle m_MaxNoCoolHeatAirVolFlow;
        UseCompFlow m_AirFlowControl;
        bool m_CoolingCoilUpstream;
        Nandle m_MaxOATSuppHeat;
        Nandle m_MinOATCompressorCooling;
        Nandle m_MinOATCompressorHeating;
        Nandle m_MaxONOFFCyclesperHour;
        Nandle m_HPTimeConstant;
        Nandle m_OnCyclePowerFraction;
        Nandle m_FanDelayTime;
        Nandle m_AncillaryOnPower;
        Nandle m_AncillaryOffPower;
        Nandle m_DesignHRWaterVolumeFlow;
        Nandle m_MaxHROutletWaterTemp;
        bool m_HeatRecActive;
        int m_HeatRecoveryInletNodeNum;
        int m_HeatRecoveryOutletNodeNum;
        int m_DesignSpecMSHPIndex;
        Nandle m_NoLoadAirFlowRateRatio;
        Nandle m_IdleMassFlowRate;
        Nandle m_IdleVolumeAirRate; // idle air flow rate [m3/s]
        Nandle m_IdleSpeedRatio;
        int m_SingleMode;
        bool m_MultiOrVarSpeedHeatCoil;
        bool m_MultiOrVarSpeedCoolCoil;
        Nandle m_PartLoadFrac;
        Nandle m_CoolingPartLoadFrac;
        Nandle m_HeatingPartLoadFrac;
        Nandle m_SuppHeatPartLoadFrac;
        Nandle m_HeatCompPartLoadRatio;
        Nandle m_CoolCompPartLoadRatio;
        Nandle m_SpeedRatio;
        Nandle m_CycRatio;

        bool m_MyEnvrnFlag;
        bool m_MyEnvrnFlag2;
        bool m_MyPlantScanFlag;
        bool m_MySuppCoilPlantScanFlag;
        bool m_MySetPointCheckFlag;
        bool m_MySizingCheckFlag;
        bool m_InitHeatPump; // Heat pump initialization flag (for error reporting)

        int m_HRLoopNum;
        int m_HRLoopSideNum;
        int m_HRBranchNum;
        int m_HRCompNum;
        int m_SuppCoilLoopNum;
        int m_SuppCoilLoopSide;
        int m_SuppCoilBranchNum;
        int m_SuppCoilCompNum;
        int m_SuppCoilFluidOutletNodeNum;

        Nandle m_WSHPRuntimeFrac;
        Nandle m_CompPartLoadRatio;
        Nandle m_CoolingCoilSensDemand;
        Nandle m_CoolingCoilLatentDemand;
        Nandle m_HeatingCoilSensDemand;
        Nandle m_SenLoadLoss;
        Nandle m_LatLoadLoss;
        Nandle m_DesignHeatRecMassFlowRate;
        Nandle m_HeatRecoveryMassFlowRate;
        Nandle m_HeatRecoveryRate;
        Nandle m_HeatRecoveryEnergy;
        Nandle m_HeatRecoveryInletTemp;
        Nandle m_HeatRecoveryOutletTemp;

        int m_IterationCounter;

        Nandle m_DesiredOutletTemp;
        Nandle m_DesiredOutletHumRat;
        int m_FrostControlStatus;

        Nandle m_CoolingCycRatio;
        Nandle m_CoolingSpeedRatio;
        int m_CoolingSpeedNum;
        Nandle m_HeatingCycRatio;
        Nandle m_HeatingSpeedRatio;
        int m_HeatingSpeedNum;
        int m_SpeedNum;

        Nandle m_DehumidInducedHeatingDemandRate;

        Nandle m_TotalAuxElecPower;
        Nandle m_HeatingAuxElecConsumption;
        Nandle m_CoolingAuxElecConsumption;
        Nandle m_ElecPower;
        Nandle m_ElecPowerConsumption;

        int m_LastMode;
        bool m_FirstPass;

        Nandle m_TotCoolEnergyRate;
        Nandle m_SensCoolEnergyRate;
        Nandle m_LatCoolEnergyRate;
        Nandle m_TotHeatEnergyRate;
        Nandle m_SensHeatEnergyRate;
        Nandle m_LatHeatEnergyRate;

        bool m_DesignFanVolFlowRateEMSOverrideOn;         // If true, then EMS is calling to override autosize fan flow
        bool m_MaxHeatAirVolFlowEMSOverrideOn;            // If true, then EMS is calling to override autosize fan flow
        bool m_MaxCoolAirVolFlowEMSOverrideOn;            // If true, then EMS is calling to override autosize fan flow
        bool m_MaxNoCoolHeatAirVolFlowEMSOverrideOn;      // If true, then EMS is calling to override autosize fan flow
        Nandle m_DesignFanVolFlowRateEMSOverrideValue;    // EMS value for override of fan flow rate autosize [m3/s]
        Nandle m_MaxHeatAirVolFlowEMSOverrideValue;       // EMS value for override of fan flow rate autosize [m3/s]
        Nandle m_MaxCoolAirVolFlowEMSOverrideValue;       // EMS value for override of fan flow rate autosize [m3/s]
        Nandle m_MaxNoCoolHeatAirVolFlowEMSOverrideValue; // EMS value for override of fan flow rate autosize [m3/s]
        bool m_EMSOverrideSensZoneLoadRequest;            // If true, then EMS is calling to override zone load
        bool m_EMSOverrideMoistZoneLoadRequest;           // If true, then EMS is calling to override zone load
        Nandle m_EMSSensibleZoneLoadValue;                // Value EMS is directing to use
        Nandle m_EMSMoistureZoneLoadValue;                // Value EMS is directing to use
        // Staged thermostat control
        int m_StageNum; // Stage number specified by staged thermostat
        bool m_Staged;  // Using Staged thermostat

        Nandle m_HeatingFanSpeedRatio;
        Nandle m_CoolingFanSpeedRatio;
        Nandle m_NoHeatCoolSpeedRatio;
        bool m_MyFanFlag;
        bool m_MyCheckFlag;
        Nandle m_SensibleLoadMet;
        Nandle m_LatentLoadMet;
        bool m_MyStagedFlag;
        Nandle m_SensibleLoadPredicted;
        Nandle m_MoistureLoadPredicted;

        // Fault model of coil SAT sensor
        bool m_FaultyCoilSATFlag;     // True if the coil has SAT sensor fault
        int m_FaultyCoilSATIndex;     // Index of the fault object corresponding to the coil
        Nandle m_FaultyCoilSATOffset; // Coil SAT sensor offset

        int m_TESOpMode; // operating mode of TES DX cooling coil
        bool m_initLoadBasedControlAirLoopPass;
        int m_airLoopPassCounter;
        int m_airLoopReturnCounter;
        bool m_FanCompNotSetYet;
        bool m_CoolCompNotSetYet;
        bool m_HeatCompNotSetYet;
        bool m_SuppCompNotSetYet;
        bool m_OKToPrintSizing;
        Nandle m_SmallLoadTolerance;

    public:
        // SZVAV variables
        int UnitarySystemType_Num;
        int MaxIterIndex;
        int RegulaFalsiFailedIndex;
        int NodeNumOfControlledZone;
        Nandle FanPartLoadRatio;
        Nandle CoolCoilWaterFlowRatio;
        Nandle HeatCoilWaterFlowRatio;
        int ControlZoneNum;              // index of unit in ZoneEquipConfig
        int AirInNode;                   // Parent inlet air node number
        int AirOutNode;                  // Parent outlet air node number
        Nandle MaxCoolAirMassFlow;       // Maximum coil air mass flow for cooling [kg/s]
        Nandle MaxHeatAirMassFlow;       // Maximum coil air mass flow for heating [kg/s]
        Nandle MaxNoCoolHeatAirMassFlow; // Maximum coil air mass flow for no cooling or heating [kg/s]
        Nandle DesignMinOutletTemp;      // DOAS DX Cooling or SZVAV coil outlet air minimum temperature [C]
        Nandle DesignMaxOutletTemp;      // Maximum supply air temperature from heating coil [C]
        Nandle LowSpeedCoolFanRatio;     // cooling mode ratio of low speed fan flow to full flow rate
        Nandle LowSpeedHeatFanRatio;     // heating mode ratio of low speed fan flow to full flow rate
        Nandle MaxCoolCoilFluidFlow;     // Maximum cooling coil fluid flow for chilled water coil
        Nandle MaxHeatCoilFluidFlow;     // Maximum heating coil fluid flow for hot water or steam coil
        int CoolCoilInletNodeNum;        // Cooling coil air inlet node number
        int CoolCoilOutletNodeNum;       // Cooling coil air outlet node number
        int CoolCoilFluidOutletNodeNum;  // Cooling coil fluid outlet node number (from Plant Loop data)
        int CoolCoilLoopNum;             // Plant loop num of chilled water coil
        int CoolCoilLoopSide;            // Supply side or demand side
        int CoolCoilBranchNum;           // Branch of number of the cooling coil in the plant loop
        int CoolCoilCompNum;             // Comp num of the cooling coil in the plant loop
        int CoolCoilFluidInletNode;      // Cooling coil fluid inlet node
        int HeatCoilLoopNum;             // Plant loop num of hot water or steam coil
        int HeatCoilLoopSide;            // Supply side or demand side
        int HeatCoilBranchNum;           // Branch of number of the heating coil in the plant loop
        int HeatCoilCompNum;             // Comp num of the heating coil in the plant loop
        int HeatCoilFluidInletNode;      // Heating coil fluid inlet node
        int HeatCoilFluidOutletNodeNum;  // Heating coil fluid outlet node number (from Plant Loop data)
        int HeatCoilInletNodeNum;        // Heating coil air inlet node number
        int HeatCoilOutletNodeNum;       // Heating coil air outlet node number
        bool ATMixerExists;              // true if AT mixer is connected to Unitary System
        int ATMixerType;                 // type of AT mixer, inlet-side or supply-side
        int ATMixerOutNode;              // AT mixer outlet node number
        Nandle ControlZoneMassFlowFrac;  // fraction of air flow to the control zone
        DesignSpecMSHP *m_CompPointerMSHP;
        std::string Name;
        std::string UnitType;
        Nandle LoadSHR;                  // Load sensible heat ratio with humidity control
        Nandle CoilSHR;    // Load sensible heat ratio with humidity control

        //    private:
        // private members not initialized in constructor
        std::string m_FanName;
        std::string m_ATMixerName;
        std::string m_HeatingCoilName;
        std::string m_HeatingCoilTypeName;
        std::string m_CoolingCoilName;
        std::string m_SuppHeatCoilName;
        std::string m_SuppHeatCoilTypeName;
        std::string m_DesignSpecMultispeedHPType;
        std::string m_DesignSpecMultispeedHPName;
        std::vector<Nandle> m_CoolVolumeFlowRate;
        std::vector<Nandle> m_CoolMassFlowRate;
        std::vector<Nandle> m_MSCoolingSpeedRatio;
        std::vector<Nandle> m_HeatVolumeFlowRate;
        std::vector<Nandle> m_HeatMassFlowRate;
        std::vector<Nandle> m_MSHeatingSpeedRatio;
        std::vector<Nandle> m_HeatingVolFlowRatio;
        std::vector<int> m_IterationMode; // array of operating mode each iteration
        std::vector<Nandle> FullOutput;   // Full output for different speed
        std::vector<Nandle> FullLatOutput;   // Full latent output for different speed
        std::vector<Nandle> SpeedSHR; // SHR at different speed

        struct WarnMessages
        {
            // Warning message variables
            int m_HXAssistedSensPLRIter = 0;        // used in HX Assisted calculations
            int m_HXAssistedSensPLRIterIndex = 0;   // used in HX Assisted calculations
            int m_HXAssistedSensPLRFail = 0;        // used in HX Assisted calculations
            int m_HXAssistedSensPLRFailIndex = 0;   // used in HX Assisted calculations
            int m_HXAssistedSensPLRFail2 = 0;       // used in HX Assisted calculations
            int m_HXAssistedSensPLRFailIndex2 = 0;  // used in HX Assisted calculations
            int m_HXAssistedLatPLRIter = 0;         // used in HX Assisted calculations
            int m_HXAssistedLatPLRIterIndex = 0;    // used in HX Assisted calculations
            int m_HXAssistedLatPLRFail = 0;         // used in HX Assisted calculations
            int m_HXAssistedLatPLRFailIndex = 0;    // used in HX Assisted calculations
            int m_HXAssistedCRLatPLRIter = 0;       // used in HX Assisted calculations
            int m_HXAssistedCRLatPLRIterIndex = 0;  // used in HX Assisted calculations
            int m_HXAssistedCRLatPLRFail = 0;       // used in HX Assisted calculations
            int m_HXAssistedCRLatPLRFailIndex = 0;  // used in HX Assisted calculations
            int m_HXAssistedCRLatPLRFail2 = 0;      // used in HX Assisted calculations
            int m_HXAssistedCRLatPLRFailIndex2 = 0; // used in HX Assisted calculations
            int m_SensPLRIter = 0;                  // used in cool coil calculations
            int m_SensPLRIterIndex = 0;             // used in cool coil calculations
            int m_SensPLRFail = 0;                  // used in cool coil calculations
            int m_SensPLRFailIndex = 0;             // used in cool coil calculations
            int m_LatPLRIter = 0;                   // used in cool coil calculations
            int m_LatPLRIterIndex = 0;              // used in cool coil calculations
            int m_LatPLRFail = 0;                   // used in cool coil calculations
            int m_LatPLRFailIndex = 0;              // used in cool coil calculations
            int m_HeatCoilSensPLRIter = 0;          // used in heat coil calculations
            int m_HeatCoilSensPLRIterIndex = 0;     // used in heat coil calculations
            int m_HeatCoilSensPLRFail = 0;          // used in heat coil calculations
            int m_HeatCoilSensPLRFailIndex = 0;     // used in heat coil calculations
            int m_SuppHeatCoilSensPLRIter = 0;      // used in supp heat coil calculations
            int m_SuppHeatCoilSensPLRIterIndex = 0; // used in supp heat coil calculations
            int m_SuppHeatCoilSensPLRFail = 0;      // used in supp heat coil calculations
            int m_SuppHeatCoilSensPLRFailIndex = 0; // used in supp heat coil calculations
            int m_LatMaxIterIndex = 0;              // used in PLR calculations for moisture load
            int m_LatRegulaFalsiFailedIndex = 0;    // used in PLR calculations for moisture load
        };
        WarnMessages warnIndex;

        static void getUnitarySystemInput(std::string const &Name, bool const ZoneEquipment, int const ZoneOAUnitNum);

        static Nandle DOE2DXCoilResidual(Nandle const PartLoadRatio,    // compressor cycling ratio (1.0 is continuous, 0.0 is off)
                                         std::vector<Nandle> const &Par // par(1) = DX coil number
        );

        static Nandle DOE2DXCoilHumRatResidual(Nandle const PartLoadRatio,    // compressor cycling ratio (1.0 is continuous, 0.0 is off)
                                               std::vector<Nandle> const &Par // par(1) = DX coil number
        );

        static Nandle calcUnitarySystemLoadResidual(Nandle const PartLoadRatio,    // DX cooling coil part load ratio
                                                    std::vector<Nandle> const &Par // Function parameters
        );

        static Nandle HXAssistedCoolCoilTempResidual(Nandle const PartLoadRatio,    // compressor cycling ratio (1.0 is continuous, 0.0 is off)
                                                     std::vector<Nandle> const &Par // par(1) = DX coil number
        );

        static Nandle hotWaterHeatingCoilResidual(Nandle const PartLoadFrac,     // Compressor cycling ratio (1.0 is continuous, 0.0 is off)
                                                  std::vector<Nandle> const &Par // par(1) = DX coil number
        );

        static Nandle HXAssistedCoolCoilHRResidual(Nandle const PartLoadRatio,    // compressor cycling ratio (1.0 is continuous, 0.0 is off)
                                                   std::vector<Nandle> const &Par // par(1) = DX coil number
        );

        static Nandle DXCoilVarSpeedResidual(Nandle const SpeedRatio,       // compressor speed ratio (1.0 is max, 0.0 is min)
                                             std::vector<Nandle> const &Par // par(1) = DX coil number
        );

        static Nandle heatingCoilVarSpeedResidual(Nandle const SpeedRatio,       // compressor speed ratio (1.0 is max, 0.0 is min)
                                                  std::vector<Nandle> const &Par // par(1) = DX coil number
        );

        static Nandle DXCoilVarSpeedHumRatResidual(Nandle const SpeedRatio,       // compressor speed ratio (1.0 is max, 0.0 is min)
                                                   std::vector<Nandle> const &Par // par(1) = DX coil number
        );

        static Nandle DXCoilCyclingResidual(Nandle const CycRatio,         // compressor cycling ratio (1.0 is continuous, 0.0 is off)
                                            std::vector<Nandle> const &Par // par(1) = DX coil number
        );

        static Nandle DXCoilCyclingHumRatResidual(Nandle const CycRatio,         // compressor cycling ratio (1.0 is continuous, 0.0 is off)
                                                  std::vector<Nandle> const &Par // par(1) = DX coil number
        );

        static Nandle heatingCoilVarSpeedCycResidual(Nandle const CycRatio,         // compressor cycling ratio (1.0 is continuous, 0.0 is off)
                                                     std::vector<Nandle> const &Par // par(1) = DX coil number
        );

        static Nandle TESIceStorageCoilOutletResidual(Nandle const PartLoadRatio,    // compressor cycling ratio (1.0 is continuous, 0.0 is off)
                                                      std::vector<Nandle> const &Par // par( 1 ) = double( UnitarySysNum );
        );

        static Nandle multiModeDXCoilResidual(Nandle const PartLoadRatio,    // compressor cycling ratio (1.0 is continuous, 0.0 is off)
                                              std::vector<Nandle> const &Par // par(1) = DX coil number
        );

        static Nandle multiModeDXCoilHumRatResidual(Nandle const PartLoadRatio,    // compressor cycling ratio (1.0 is continuous, 0.0 is off)
                                                    std::vector<Nandle> const &Par // par(1) = DX coil number
        );

        static Nandle coolWaterHumRatResidual(Nandle const PartLoadRatio,    // compressor cycling ratio (1.0 is continuous, 0.0 is off)
                                              std::vector<Nandle> const &Par // par(1) = CoolWater coil number
        );

        static Nandle coolWaterTempResidual(Nandle const PartLoadRatio,    // compressor cycling ratio (1.0 is continuous, 0.0 is off)
                                            std::vector<Nandle> const &Par // par(1) = CoolWater coil number
        );

        static Nandle gasElecHeatingCoilResidual(Nandle const PartLoadFrac,     // Compressor cycling ratio (1.0 is continuous, 0.0 is off)
                                                 std::vector<Nandle> const &Par // par(1) = DX coil number
        );

        static Nandle steamHeatingCoilResidual(Nandle const PartLoadFrac,     // Compressor cycling ratio (1.0 is continuous, 0.0 is off)
                                               std::vector<Nandle> const &Par // par(1) = DX coil number
        );

        static Nandle heatWatertoAirHPTempResidual(Nandle const PartLoadRatio,    // compressor cycling ratio (1.0 is continuous, 0.0 is off)
                                                   std::vector<Nandle> const &Par // par(1) = HeatWatertoAirHP coil number
        );

        static Nandle coolWatertoAirHPHumRatResidual(Nandle const PartLoadRatio,    // compressor cycling ratio (1.0 is continuous, 0.0 is off)
                                                     std::vector<Nandle> const &Par // par(1) = CoolWatertoAirHP coil number
        );

        static Nandle coolWatertoAirHPTempResidual(Nandle const PartLoadRatio,    // compressor cycling ratio (1.0 is continuous, 0.0 is off)
                                                   std::vector<Nandle> const &Par // par(1) = CoolWatertoAirHP coil number
        );

        static Nandle DXHeatingCoilResidual(Nandle const PartLoadFrac,     // Compressor cycling ratio (1.0 is continuous, 0.0 is off)
                                            std::vector<Nandle> const &Par // par(1) = DX coil number
        );

        void initUnitarySystems(int const &AirLoopNum, bool const &FirstHVACIteration, int const ZoneOAUnitNum, Nandle const OAUCoilOutTemp);

        void checkNodeSetPoint(int const AirLoopNum,       // number of the current air loop being simulated
                               int const ControlNode,      // Node to test for set point
                               int const CoilType,         // True if cooling coil, then test for HumRatMax set point
                               Nandle const OAUCoilOutTemp // the coil inlet temperature of OutdoorAirUnit
        );

        void frostControlSetPointLimit(Nandle &TempSetPoint,       // temperature setpoint of the sensor node
                                       Nandle &HumRatSetPoint,     // humidity ratio setpoint of the sensor node
                                       Nandle const BaroPress,     // baromtric pressure, Pa [N/m^2]
                                       Nandle const TfrostControl, // minimum temperature limit for forst control
                                       int const ControlMode       // temperature or humidity control mode
        );

        void reportUnitarySystem(int const AirLoopNum);

        void unitarySystemHeatRecovery();

        void controlUnitarySystemtoSP(int const AirLoopNum,          // Primary air loop number
                                      bool const FirstHVACIteration, // True when first HVAC iteration
                                      int &CompOn,                   // compressor on/off control
                                      Nandle const OAUCoilOutTemp,   // the coil inlet temperature of OutdoorAirUnit
                                      bool HXUnitOn,                 // Flag to control HX for HXAssisted Cooling Coil
                                      Nandle &sysOutputProvided,     // sensible output at supply air node
                                      Nandle &latOutputProvided      // latent output at supply air node
        );

        void controlUnitarySystemtoLoad(int const AirLoopNum,          // Primary air loop number
                                        bool const FirstHVACIteration, // True when first HVAC iteration
                                        int &CompOn,                   // Determines if compressor is on or off
                                        Nandle const OAUCoilOutTemp,   // the coil inlet temperature of OutdoorAirUnit
                                        bool HXUnitOn,                 // Flag to control HX for HXAssisted Cooling Coil
                                        Nandle &sysOutputProvied,      // system sensible output at supply air node
                                        Nandle &latOutputProvided      // system latent output at supply air node
        );

        void updateUnitarySystemControl(int const AirLoopNum,  // number of the current air loop being simulated
                                        int const OutNode,     // coil outlet node number
                                        int const ControlNode, // control node number
                                        Nandle &OnOffAirFlowRatio,
                                        bool const FirstHVACIteration,
                                        Nandle const OAUCoilOutletTemp, // "ONLY" for zoneHVAC:OutdoorAirUnit
                                        Nandle &ZoneLoad,
                                        Nandle const MaxOutletTemp // limits heating coil outlet temp [C]
        );

        void controlUnitarySystemOutput(int const AirLoopNum,          // Index to air loop
                                        bool const FirstHVACIteration, // True when first HVAC iteration
                                        Nandle &OnOffAirFlowRatio,     // ratio of heating PLR to cooling PLR (is this correct?)
                                        Nandle const ZoneLoad,
                                        Nandle &FullSensibleOutput,
                                        bool HXUnitOn, // Flag to control HX for HXAssisted Cooling Coil
                                        int CompOn);

        void initLoadBasedControl(int const AirLoopNum, // number of the current air loop being simulated
                                  bool const FirstHVACIteration,
                                  Nandle &OnOffAirFlowRatio,
                                  Nandle &ZoneLoad);

        void setOnOffMassFlowRate(Nandle &OnOffAirFlowRatio, // ratio of coil on to coil off air flow rate
                                  Nandle const PartLoadRatio // coil part-load ratio
        );

        void setAverageAirFlow(Nandle const PartLoadRatio, // unit part load ratio
                               Nandle &OnOffAirFlowRatio   // ratio of compressor ON airflow to AVERAGE airflow over timestep
        );

        void calculateCapacity(Nandle &SensOutput, // sensible output of AirloopHVAC:UnitarySystem
                               Nandle &LatOutput   // latent output of AirloopHVAC:UnitarySystem
        );

        void calcUnitaryCoolingSystem(int const AirLoopNum,          // index to air loop
                                      bool const FirstHVACIteration, // True when first HVAC iteration
                                      Nandle const PartLoadRatio,    // coil operating part-load ratio
                                      int const CompOn,              // compressor control (0=off, 1=on)
                                      Nandle const OnOffAirFlowRatio,
                                      Nandle const CoilCoolHeatRat, // ratio of cooling to heating PLR for cycling fan RH control
                                      bool const HXUnitOn           // Flag to control HX for HXAssisted Cooling Coil
        );

        void calcUnitaryHeatingSystem(int const AirLoopNum,           // index to air loop
                                      bool const FirstHVACIteration,  // True when first HVAC iteration
                                      Nandle const PartLoadRatio,     // coil operating part-load ratio
                                      int const CompOn,               // comrpressor control (0=off, 1=on)
                                      Nandle const OnOffAirFlowRatio, // ratio of on to off flow rate
                                      Nandle HeatCoilLoad             // adjusted heating coil load if outlet temp exceeds max (W)
        );

        void calcUnitarySuppHeatingSystem(bool const FirstHVACIteration, // True when first HVAC iteration
                                          Nandle const PartLoadRatio,    // coil operating part-load ratio
                                          Nandle const SuppCoilLoad      // adjusted supp coil load when outlet temp exceeds max (W)
        );

        void calcUnitarySuppSystemToSP(bool const FirstHVACIteration // True when first HVAC iteration
        );

        void controlCoolingSystemToSP(int const AirLoopNum,          // index to air loop
                                      bool const FirstHVACIteration, // First HVAC iteration flag
                                      bool &HXUnitOn,                // flag to enable heat exchanger heat recovery
                                      int &CompOp                    // compressor on/off control
        );

        void controlHeatingSystemToSP(int const AirLoopNum,          // index to air loop
                                      bool const FirstHVACIteration, // First HVAC iteration flag
                                      int &CompOn,                   // compressor on/off control
                                      Nandle &HeatCoilLoad           // load met by heating coil
        );

        void controlSuppHeatSystemToSP(int const AirLoopNum,         // index to air loop
                                       bool const FirstHVACIteration // First HVAC iteration flag
        );

        void simMultiSpeedCoils(int const AirLoopNum,          // Index to air loop
                                bool const FirstHVACIteration, // True when first HVAC iteration
                                int &CompOn,                   // compressor on/off control
                                bool const SensibleLoad,
                                bool const LatentLoad,
                                Nandle const PartLoadFrac,
                                int const CoilType,
                                int const SpeedNumber);

        void calcPassiveSystem(int const AirLoopNum,         // Index to air loop
                               bool const FirstHVACIteration // True when first HVAC iteration
        );

        void heatPumpRunFrac(Nandle const PLR,   // part load ratio
                             bool &errFlag,      // part load factor out of range flag
                             Nandle &RuntimeFrac // the required run time fraction to meet part load
        );

        void setSpeedVariables(bool const SensibleLoad,   // True when meeting a sensible load (not a moisture load)
                               Nandle const PartLoadRatio // operating PLR
        );

    public:
        UnitarySys(); // constructor

        ~UnitarySys() // destructor
        {
        }

        static void getUnitarySystemInputData(std::string const &Name, bool const ZoneEquipment, int const ZoneOAUnitNum, bool &errorsFound);

        static HVACSystemData *factory(int const object_type_of_num, std::string const objectName, bool const ZoneEquipment, int const ZoneOAUnitNum);

        void simulateSys(std::string const &Name,
                         bool const firstHVACIteration,
                         int const &AirLoopNum,
                         int &CompIndex,
                         bool &HeatActive,
                         bool &CoolActive,
                         int const OAUnitNum,         // If the system is an equipment of OutdoorAirUnit
                         Nandle const OAUCoilOutTemp, // the coil inlet temperature of OutdoorAirUnit
                         bool const ZoneEquipment,    // TRUE if called as zone equipment
                         Nandle &sysOutputProvided,   // sensible output at supply air node
                         Nandle &latOutputProvided    // latent output at supply air node
        );

        void calcUnitarySystemToLoad(int const AirLoopNum,          // index to air loop
                                     bool const FirstHVACIteration, // True when first HVAC iteration
                                     Nandle const CoolPLR,          // operating cooling part-load ratio []
                                     Nandle const HeatPLR,          // operating cooling part-load ratio []
                                     Nandle &OnOffAirFlowRatio,     // ratio of heating PLR to cooling PLR (is this correct?)
                                     Nandle &SensOutput,            // sensible capacity (W)
                                     Nandle &LatOutput,             // latent capacity (W)
                                     bool HXUnitOn,                 // Flag to control HX for HXAssisted Cooling Coil
                                     Nandle HeatCoilLoad,           // Adjusted load to heating coil when SAT exceeds max limit (W)
                                     Nandle SuppCoilLoad,           // Adjusted load to supp heating coil when SAT exceeds max limit (W)
                                     int const CompOn               // Determines if compressor is on or off
        );

        static void checkUnitarySysCoilInOASysExists(std::string const &UnitarySysName, int const ZoneOAUnitNum);

        static void getUnitarySysHeatCoolCoil(std::string const &UnitarySysName, // Name of Unitary System object
                                              bool &CoolingCoil,                 // Cooling coil exists
                                              bool &HeatingCoil,                 // Heating coil exists
                                              int const ZoneOAUnitNum            // index to zone OA unit
        );

        static Nandle calcUnitarySystemWaterFlowResidual(Nandle const PartLoadRatio,    // water mass flow rate [kg/s]
                                                         std::vector<Nandle> const &Par // Function parameters
        );

        void simulate(std::string const &Name,
            bool const firstHVACIteration,
            int const &AirLoopNum,
            int &CompIndex,
            bool &HeatActive,
            bool &CoolActive,
            int const OAUnitNum,         // If the system is an equipment of OutdoorAirUnit
            Nandle const OAUCoilOutTemp, // the coil inlet temperature of OutdoorAirUnit
            bool const ZoneEquipment,    // TRUE if called as zone equipment
            Nandle &sysOutputProvided,   // sensible output at supply air node
            Nandle &latOutputProvided    // latent output at supply air node
        ) override;

        void sizeSystem(bool const FirstHVACIteration, int const AirLoopNum) override;
        int getAirInNode(std::string const &UnitarySysName, int const ZoneOAUnitNum, bool &errFlag) override;
        int getAirOutNode(std::string const &UnitarySysName, int const ZoneOAUnitNum, bool &errFlag) override;
    };

    extern std::vector<UnitarySys> unitarySys;
    extern std::vector<DesignSpecMSHP> designSpecMSHP;
    int getDesignSpecMSHPIndex(std::string const &objectName);
    int getUnitarySystemIndex(std::string const &objectName);

} // namespace UnitarySystems
} // namespace EnergyPlus
#endif // ENERGYPLUS_UNITARYSYSTEM_HH
