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

#ifndef Furnaces_hh_INCLUDED
#define Furnaces_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>
#include <ObjexxFCL/Optional.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>
#include <EnergyPlus/VariableSpeedCoils.hh>

namespace EnergyPlus {

namespace Furnaces {

    // Using/Aliasing
    using VariableSpeedCoils::MaxSpedLevels;

    // Data
    // MODULE PARAMETER DEFINITIONS
    // na

    // Last mode of operation
    extern int const CoolingMode; // last compressor operating mode was in cooling
    extern int const HeatingMode; // last compressor operating mode was in heating
    // Airflow control for contant fan mode
    extern int const UseCompressorOnFlow;  // set compressor OFF air flow rate equal to compressor ON air flow rate
    extern int const UseCompressorOffFlow; // set compressor OFF air flow rate equal to user defined value
    // Compressor operation
    extern int const On;  // normal compressor operation
    extern int const Off; // signal DXCoil that compressor shouldn't run

    // Dehumidification control modes (DehumidControlMode)
    extern int const DehumidControl_None;
    extern int const DehumidControl_Multimode;
    extern int const DehumidControl_CoolReheat;

    // DERIVED TYPE DEFINITIONS

    // MODULE VARIABLE DECLARATIONS:
    extern int NumFurnaces; // The number of furnaces found in the input data file
    extern Array1D_bool MySizeFlag;
    extern Array1D_bool CheckEquipName;
    extern Nandle ModifiedHeatCoilLoad; // used to adjust heating coil capacity if outlet temp > DesignMaxOutletTemp,
    // used for Coil:Gas:Heating and Coil:Electric:Heating coils only.
    extern Nandle OnOffAirFlowRatioSave;        // Saves the OnOffAirFlowRatio calculated in RegulaFalsi CALLs.
    extern Nandle OnOffFanPartLoadFractionSave; // Global part-load fraction passed to fan object
    extern Nandle CompOnMassFlow;               // Supply air mass flow rate w/ compressor ON [kg/s]
    extern Nandle CompOffMassFlow;              // Supply air mass flow rate w/ compressor OFF [kg/s]
    extern Nandle CompOnFlowRatio;              // fan flow ratio when coil on
    extern Nandle CompOffFlowRatio;             // fan flow ratio when coil off
    extern Nandle FanSpeedRatio;                // ratio of air flow ratio passed to fan object
    extern Nandle CoolHeatPLRRat;               // ratio of cooling to heating PLR, used for cycling fan RH control
    extern bool HeatingLoad;
    extern bool CoolingLoad;
    extern bool EconomizerFlag;             // holds air loop economizer status
    extern int AirLoopPass;                 // Number of air loop pass
    extern bool HPDehumidificationLoadFlag; // true if there is dehumidification load (heat pumps only)
    extern Nandle TempSteamIn;              // steam coil steam inlet temperature
    // starting add variables for variable speed water source heat pump
    extern Nandle SaveCompressorPLR;        // holds compressor PLR from active DX coil
    extern std::string CurrentModuleObject; // Object type for getting and error messages
    // ending varibles for variable speed water source heat pump

    // Subroutine Specifications for the Module
    // Driver/Manager Routines

    // Get Input routines for module

    // Initialization routines for module

    // Calculate routines to check convergence

    // Supporting routines for module

    // modules for variable speed heat pump

    // Reporting routines for module

    // Types

    struct FurnaceEquipConditions
    {
        // Members
        std::string Name;                   // Name of the Furnace
        int FurnaceType_Num;                // Numeric Equivalent for Furnace Type
        int FurnaceIndex;                   // Index to furnace
        int SchedPtr;                       // Index to furnace operating schedule
        int FanSchedPtr;                    // Index to fan operating mode schedule
        int FanAvailSchedPtr;               // Index to fan availability schedule
        int ControlZoneNum;                 // Index to controlled zone
        int ZoneSequenceCoolingNum;         // Index to cooling sequence/priority for this zone
        int ZoneSequenceHeatingNum;         // Index to heating sequence/priority for this zone
        int CoolingCoilType_Num;            // Numeric Equivalent for Cooling Coil Type
        int CoolingCoilIndex;               // Index to cooling coil
        int ActualDXCoilIndexForHXAssisted; // Index to DX cooling coil when HX assisted
        bool CoolingCoilUpstream;           // Indicates if cooling coil is upstream of heating coil
        int HeatingCoilType_Num;            // Numeric Equivalent for Heating Coil Type
        int HeatingCoilIndex;               // Index to heating coil
        int ReheatingCoilType_Num;          // Numeric Equivalent for Reheat Coil Type
        int ReheatingCoilIndex;             // Index to reheat coil
        std::string HeatingCoilName;        // name of heating coil
        std::string HeatingCoilType;        // type of heating coil
        int CoilControlNode;                // control node for hot water and steam heating coils
        int HWCoilAirInletNode;             // air inlet node number of HW coil for PTAC, PTHP, HeatCool, HeatOnly
        int HWCoilAirOutletNode;            // air outlet node number of HW coil for PTAC, PTHP, HeatCool, HeatOnly
        int SuppCoilAirInletNode;           // air inlet node number of HW coil for HeatCool Reheat Coil
        int SuppCoilAirOutletNode;          // air outlet node number of HW coil for HeatCool Reheat Coil
        int SuppHeatCoilType_Num;           // Numeric Equivalent for Supplemental Heat Coil Type
        int SuppHeatCoilIndex;              // Index to supplemental heater
        int SuppCoilControlNode;            // control node for steam and hot water heating coil
        std::string SuppHeatCoilName;       // name of supplemental heating coil
        std::string SuppHeatCoilType;       // type of supplemental heating coil
        int FanType_Num;                    // Integer equivalent of fan type (1=OnOff, 2 = ConstVolume)
        int FanIndex;                       // Index to fan object
        int FurnaceInletNodeNum;            // Furnace inlet node number
        int FurnaceOutletNodeNum;           // Furnace inlet node number
        int OpMode;                         // operation mode: 1 = cycling fan, cycling coils
        //                 2 = continuous fan, cycling coils
        int LastMode;                       // last mode of operation, coolingmode or heatingmode
        int AirFlowControl;                 // fan control mode, UseCompressorOnFlow or UseCompressorOffFlow
        int FanPlace;                       // fan placement; 1=blow through, 2=draw through
        int NodeNumOfControlledZone;        // Node number of controlled zone air node
        int WatertoAirHPType;               // Type of water to air heat pump model used
        Nandle CoolingConvergenceTolerance; // Convergence tolerance for cooling,
        //   ratio (CoolingCoilLoad - FurnaceCoolingOutput)/CoolingCoilLoad
        Nandle HeatingConvergenceTolerance; // Convergence tolerance for heating,
        //   ratio (HeatingCoilLoad - HeatPumpheatingOutput)/HeatingCoilLoad
        Nandle DesignHeatingCapacity;                   // Nominal Capacity of Heating Coil [W]
        Nandle DesignCoolingCapacity;                   // Nominal Capacity of Cooling Coil [W]
        Nandle CoolingCoilSensDemand;                   // Sensible demand on Cooling Coil [W]
        Nandle HeatingCoilSensDemand;                   // Sensible demand on Heating Coil [W]
        Nandle CoolingCoilLatentDemand;                 // Latent demand on Cooling Coil [W]
        Nandle DesignSuppHeatingCapacity;               // Nominal Capacity of Supplemental Heating Coil [W]
        Nandle DesignFanVolFlowRate;                    // Vol Flow through the Furnace being Simulated [m**3/Sec]
        bool DesignFanVolFlowRateEMSOverrideOn;         // if true, then EMS is calling to override autosize fan flow
        Nandle DesignFanVolFlowRateEMSOverrideValue;    // EMS value for override of fan flow rate autosize [m3/s]
        Nandle DesignMassFlowRate;                      // Design mass flow rate through furnace [kg/s]
        Nandle MaxCoolAirVolFlow;                       // supply air volumetric flow rate during cooling operation [m3/s]
        bool MaxCoolAirVolFlowEMSOverrideOn;            // if true, EMS is calling to override autosize flow during cooling
        Nandle MaxCoolAirVolFlowEMSOverrideValue;       // EMS value for override of flow during cooling [m3/s]
        Nandle MaxHeatAirVolFlow;                       // supply air volumetric flow rate during cooling operation [m3/s]
        bool MaxHeatAirVolFlowEMSOverrideOn;            // if true, EMS is calling to override autosize flow during heating
        Nandle MaxHeatAirVolFlowEMSOverrideValue;       // EMS value for override of flow during heating operation [m3/s]
        Nandle MaxNoCoolHeatAirVolFlow;                 // supply air volumetric flow rate when no cooling or heating [m3/s]
        bool MaxNoCoolHeatAirVolFlowEMSOverrideOn;      // if true, EMS is calling to override autosize no heatcool rate
        Nandle MaxNoCoolHeatAirVolFlowEMSOverrideValue; // EMS value for override of flow during no heat cool [m3/s]
        Nandle MaxCoolAirMassFlow;                      // supply air mass flow rate during cooling operation [kg/s]
        Nandle MaxHeatAirMassFlow;                      // supply air mass flow rate during heating operation [kg/s]
        Nandle MaxNoCoolHeatAirMassFlow;                // supply air mass flow rate when no cooling or heating [kg/s]
        Nandle MaxHeatCoilFluidFlow;                    // water or steam mass flow rate for heating coil [kg/s]
        Nandle MaxSuppCoilFluidFlow;                    // water or steam mass flow rate for supplemental heating coil [kg/s]
        Nandle ControlZoneMassFlowFrac;                 // Fraction of furnace flow to control zone
        Nandle DesignMaxOutletTemp;                     // Maximum supply air temperature from furnace heater [C]
        Nandle MdotFurnace;                             // Mass flow rate through furnace [kg/s]
        Nandle FanPartLoadRatio;                        // Part load ratio of furnace fan (mdot actual/mdot design)
        Nandle CompPartLoadRatio;                       // Part load ratio of furnace compressor (load / steady-state output)
        Nandle WSHPRuntimeFrac;                         // Runtime fraction of water source heat pump
        Nandle CoolPartLoadRatio;                       // Cooling part load ratio
        Nandle HeatPartLoadRatio;                       // Heating part load ratio
        Nandle MinOATCompressorCooling;                 // Minimum outdoor operating temperature for heat pump compressor
        Nandle MinOATCompressorHeating;                 // Minimum outdoor operating temperature for heat pump compressor
        Nandle MaxOATSuppHeat;                          // Maximum outdoor dry-bulb temperature for
        int CondenserNodeNum;                           // Node number of outdoor condenser/compressor
        Nandle MaxONOFFCyclesperHour;                   // Maximum ON/OFF Cycling Rate [cycles/hr]
        Nandle HPTimeConstant;                          // Heat Pump Time Constant [s]
        Nandle OnCyclePowerFraction;                    // Fraction of on-cycle power use [~]
        // supplemental heating coil operation
        Nandle FanDelayTime; // Fan delay time, time delay for the HP's fan to
        // shut off after compressor cycle off  [s]
        bool Humidistat;                        // Humidistat control (heatcool units only and not heatpump)
        bool InitHeatPump;                      // Heat pump initialization flag (for error reporting)
        int DehumidControlType_Num;             // 0 = None, 1=MultiMode, 2=CoolReheat
        int LatentMaxIterIndex;                 // Index to recurring warning message
        int LatentRegulaFalsiFailedIndex;       // Index to recurring warning message
        int LatentRegulaFalsiFailedIndex2;      // Index to recurring warning message
        int SensibleMaxIterIndex;               // Index to recurring warning message
        int SensibleRegulaFalsiFailedIndex;     // Index to recurring warning message
        int WSHPHeatMaxIterIndex;               // Index to recurring warning message
        int WSHPHeatRegulaFalsiFailedIndex;     // Index to recurring warning message
        int DXHeatingMaxIterIndex;              // Index to recurring warning message
        int DXHeatingRegulaFalsiFailedIndex;    // Index to recurring warning messages
        int HeatingMaxIterIndex;                // Index to recurring warning message
        int HeatingMaxIterIndex2;               // Index to recurring warning message
        int HeatingRegulaFalsiFailedIndex;      // Index to recurring warning messages
        Nandle ActualFanVolFlowRate;            // Volumetric flow rate from fan object
        Nandle HeatingSpeedRatio;               // Fan speed ratio in heating mode
        Nandle CoolingSpeedRatio;               // Fan speed ratio in cooling mode
        Nandle NoHeatCoolSpeedRatio;            // Fan speed ratio when no cooling or heating
        int ZoneInletNode;                      // Zone inlet node number in the controlled zone
        Nandle SenLoadLoss;                     // Air distribution system sensible loss
        Nandle LatLoadLoss;                     // Air distribution system latent loss
        Nandle SensibleLoadMet;                 // System sensible load
        Nandle LatentLoadMet;                   // System latent load
        Nandle DehumidInducedHeatingDemandRate; // Additional heating demand on supplemental heater
        // when heat pumps operate on dehumidification mode
        int CoilOutletNode;                   // outlet node for hot water and steam heating coil
        int LoopNum;                          // plant loop index for water and steam heating coil
        int LoopSide;                         // plant loop side  index for water and steam heating coil
        int BranchNum;                        // plant loop branch index for water and steam heating coil
        int CompNum;                          // plant loop component index for water and steam heating coil
        int SuppCoilOutletNode;               // outlet node for hot water and steam supplemental heating coil
        int LoopNumSupp;                      // plant loop index for water and steam supplemental heating coil
        int LoopSideSupp;                     // plant loop side  index for  water and steam supplemental heating coil
        int BranchNumSupp;                    // plant loop branch index for water and steam supplemental heating coil
        int CompNumSupp;                      // plant loop component index for water and steam supplemental heating coil
        int HotWaterCoilMaxIterIndex;         // Index to recurring warning message
        int HotWaterCoilMaxIterIndex2;        // Index to recurring warning message
        bool EMSOverrideSensZoneLoadRequest;  // if true, then EMS is calling to override zone load
        Nandle EMSSensibleZoneLoadValue;      // Value EMS is directing to use
        bool EMSOverrideMoistZoneLoadRequest; // if true, then EMS is calling to override zone load
        Nandle EMSMoistureZoneLoadValue;      // Value EMS is directing to use
        // starting added varibles for variable speed water source heat pump, Bo Shen, ORNL, March 2012
        int HeatCoolMode;                    // System operating mode (0 = floating, 1 = cooling, 2 = heating)
        int NumOfSpeedCooling;               // The number of speeds for cooling
        int NumOfSpeedHeating;               // The number of speeds for heating
        Nandle IdleSpeedRatio;               // idle air fan ratio
        Nandle IdleVolumeAirRate;            // idle air flow rate
        Nandle IdleMassFlowRate;             // idle air flow rate
        Nandle FanVolFlow;                   // fan volumetric flow rate
        bool CheckFanFlow;                   // Supply airflow check
        Array1D<Nandle> HeatVolumeFlowRate;  // Supply air volume flow rate during heating operation
        Array1D<Nandle> HeatMassFlowRate;    // Supply air mass flow rate during heating operation
        Array1D<Nandle> CoolVolumeFlowRate;  // Supply air volume flow rate during cooling operation
        Array1D<Nandle> CoolMassFlowRate;    // Supply air mass flow rate during cooling operation
        Array1D<Nandle> MSHeatingSpeedRatio; // Fan speed ratio in heating mode
        Array1D<Nandle> MSCoolingSpeedRatio; // Fan speed ratio in cooling mode
        bool bIsIHP;
        int CompSpeedNum;
        Nandle CompSpeedRatio;
        int ErrIndexCyc;
        int ErrIndexVar;
        // end of the additional variables for variable speed water source heat pump
        int WaterCyclingMode; // Heat Pump Coil water flow mode; See definitions in DataHVACGlobals,
        // 1=water cycling, 2=water constant, 3=water constant on demand (old mode)
        int iterationCounter;       // track time step iterations
        Array1D<int> iterationMode; // keep track of previous iteration mode (i.e., cooling or heating)
        bool FirstPass;             // used to determine when first call is made

        // Default Constructor
        FurnaceEquipConditions()
            : FurnaceType_Num(0), FurnaceIndex(0), SchedPtr(0), FanSchedPtr(0), FanAvailSchedPtr(0), ControlZoneNum(0), ZoneSequenceCoolingNum(0),
              ZoneSequenceHeatingNum(0), CoolingCoilType_Num(0), CoolingCoilIndex(0), ActualDXCoilIndexForHXAssisted(0), CoolingCoilUpstream(true),
              HeatingCoilType_Num(0), HeatingCoilIndex(0), ReheatingCoilType_Num(0), ReheatingCoilIndex(0), CoilControlNode(0), HWCoilAirInletNode(0),
              HWCoilAirOutletNode(0), SuppCoilAirInletNode(0), SuppCoilAirOutletNode(0), SuppHeatCoilType_Num(0), SuppHeatCoilIndex(0),
              SuppCoilControlNode(0), FanType_Num(0), FanIndex(0), FurnaceInletNodeNum(0), FurnaceOutletNodeNum(0), OpMode(0), LastMode(0),
              AirFlowControl(0), FanPlace(0), NodeNumOfControlledZone(0), WatertoAirHPType(0), CoolingConvergenceTolerance(0.0),
              HeatingConvergenceTolerance(0.0), DesignHeatingCapacity(0.0), DesignCoolingCapacity(0.0), CoolingCoilSensDemand(0.0),
              HeatingCoilSensDemand(0.0), CoolingCoilLatentDemand(0.0), DesignSuppHeatingCapacity(0.0), DesignFanVolFlowRate(0.0),
              DesignFanVolFlowRateEMSOverrideOn(false), DesignFanVolFlowRateEMSOverrideValue(0.0), DesignMassFlowRate(0.0), MaxCoolAirVolFlow(0.0),
              MaxCoolAirVolFlowEMSOverrideOn(false), MaxCoolAirVolFlowEMSOverrideValue(0.0), MaxHeatAirVolFlow(0.0),
              MaxHeatAirVolFlowEMSOverrideOn(false), MaxHeatAirVolFlowEMSOverrideValue(0.0), MaxNoCoolHeatAirVolFlow(0.0),
              MaxNoCoolHeatAirVolFlowEMSOverrideOn(false), MaxNoCoolHeatAirVolFlowEMSOverrideValue(0.0), MaxCoolAirMassFlow(0.0),
              MaxHeatAirMassFlow(0.0), MaxNoCoolHeatAirMassFlow(0.0), MaxHeatCoilFluidFlow(0.0), MaxSuppCoilFluidFlow(0.0),
              ControlZoneMassFlowFrac(0.0), DesignMaxOutletTemp(9999.0), MdotFurnace(0.0), FanPartLoadRatio(0.0), CompPartLoadRatio(0.0),
              WSHPRuntimeFrac(0.0), CoolPartLoadRatio(0.0), HeatPartLoadRatio(0.0), MinOATCompressorCooling(0.0), MinOATCompressorHeating(0.0),
              MaxOATSuppHeat(0.0), CondenserNodeNum(0), MaxONOFFCyclesperHour(0.0), HPTimeConstant(0.0), OnCyclePowerFraction(0.0), FanDelayTime(0.0),
              Humidistat(false), InitHeatPump(false), DehumidControlType_Num(0), LatentMaxIterIndex(0), LatentRegulaFalsiFailedIndex(0),
              LatentRegulaFalsiFailedIndex2(0), SensibleMaxIterIndex(0), SensibleRegulaFalsiFailedIndex(0), WSHPHeatMaxIterIndex(0),
              WSHPHeatRegulaFalsiFailedIndex(0), DXHeatingMaxIterIndex(0), DXHeatingRegulaFalsiFailedIndex(0), HeatingMaxIterIndex(0),
              HeatingMaxIterIndex2(0), HeatingRegulaFalsiFailedIndex(0), ActualFanVolFlowRate(0.0), HeatingSpeedRatio(1.0), CoolingSpeedRatio(1.0),
              NoHeatCoolSpeedRatio(1.0), ZoneInletNode(0), SenLoadLoss(0.0), LatLoadLoss(0.0), SensibleLoadMet(0.0), LatentLoadMet(0.0),
              DehumidInducedHeatingDemandRate(0.0), CoilOutletNode(0), LoopNum(0), LoopSide(0), BranchNum(0), CompNum(0), SuppCoilOutletNode(0),
              LoopNumSupp(0), LoopSideSupp(0), BranchNumSupp(0), CompNumSupp(0), HotWaterCoilMaxIterIndex(0), HotWaterCoilMaxIterIndex2(0),
              EMSOverrideSensZoneLoadRequest(false), EMSSensibleZoneLoadValue(0.0), EMSOverrideMoistZoneLoadRequest(false),
              EMSMoistureZoneLoadValue(0.0), HeatCoolMode(0), NumOfSpeedCooling(0), NumOfSpeedHeating(0), IdleSpeedRatio(0.0), IdleVolumeAirRate(0.0),
              IdleMassFlowRate(0.0), FanVolFlow(0.0), CheckFanFlow(true), HeatVolumeFlowRate(MaxSpedLevels, 0.0),
              HeatMassFlowRate(MaxSpedLevels, 0.0), CoolVolumeFlowRate(MaxSpedLevels, 0.0), CoolMassFlowRate(MaxSpedLevels, 0.0),
              MSHeatingSpeedRatio(MaxSpedLevels, 0.0), MSCoolingSpeedRatio(MaxSpedLevels, 0.0), bIsIHP(false), CompSpeedNum(0), CompSpeedRatio(0.0),
              ErrIndexCyc(0), ErrIndexVar(0), WaterCyclingMode(0), iterationCounter(0), iterationMode(0), FirstPass(true)
        {
        }
    };

    // Object Data
    extern Array1D<FurnaceEquipConditions> Furnace;

    // Functions

    void clear_state();

    void SimFurnace(std::string const &FurnaceName,
                    bool const FirstHVACIteration,
                    int const AirLoopNum, // Primary air loop number
                    int &CompIndex        // Pointer to which furnace
    );

    // Get Input Section of the Module
    //******************************************************************************

    void GetFurnaceInput();

    // End of Get Input subroutines for this Module
    //******************************************************************************

    // Beginning Initialization Section of the Module
    //******************************************************************************

    void InitFurnace(int const FurnaceNum,         // index to Furnace
                     int const AirLoopNum,         // index to air loop
                     Nandle &OnOffAirFlowRatio,    // ratio of on to off air mass flow rate
                     int &OpMode,                  // fan operating mode
                     Nandle &ZoneLoad,             // zone sensible load to be met (modified here as needed) (W)
                     Nandle &MoistureLoad,         // zone moisture load (W)
                     bool const FirstHVACIteration // TRUE if first HVAC iteration
    );

    void SetOnOffMassFlowRate(int const FurnaceNum,      // index to furnace
                              int const AirLoopNum,      // index to air loop !unused1208
                              Nandle &OnOffAirFlowRatio, // ratio of coil on to coil off air flow rate
                              int const OpMode,          // fan operating mode
                              Nandle const ZoneLoad,     // sensible load to be met (W) !unused1208
                              Nandle const MoistureLoad, // moisture load to be met (W)
                              Nandle const PartLoadRatio // coil part-load ratio
    );

    void SizeFurnace(int const FurnaceNum, bool const FirstHVACIteration);

    // End Initialization Section of the Module
    //******************************************************************************

    // Beginning of Update subroutines for the Furnace Module
    // *****************************************************************************

    void CalcNewZoneHeatOnlyFlowRates(int const FurnaceNum,          // Index to furnace
                                      bool const FirstHVACIteration, // Iteration flag
                                      Nandle const ZoneLoad,         // load to be met by furnace (W)
                                      Nandle &HeatCoilLoad,          // actual load passed to heating coil (W)
                                      Nandle &OnOffAirFlowRatio      // ratio of coil on to coil off air flow rate
    );

    void CalcNewZoneHeatCoolFlowRates(int const FurnaceNum,
                                      bool const FirstHVACIteration,
                                      int const CompOp,          // compressor operation flag (1=On, 0=Off)
                                      Nandle const ZoneLoad,     // the control zone load (watts)
                                      Nandle const MoistureLoad, // the control zone latent load (watts)
                                      Nandle &HeatCoilLoad,      // Heating load to be met by heating coil ( excluding heat pump DX coil)
                                      Nandle &ReheatCoilLoad,    // Heating load to be met by reheat coil using hstat (excluding HP DX coil)
                                      Nandle &OnOffAirFlowRatio, // Ratio of compressor ON air flow to AVERAGE air flow over time step
                                      bool &HXUnitOn             // flag to control HX based on zone moisture load
    );

    void CalcWaterToAirHeatPump(int const AirLoopNum,          // index to air loop
                                int const FurnaceNum,          // index to Furnace
                                bool const FirstHVACIteration, // TRUE on first HVAC iteration
                                int const CompOp,              // compressor operation flag (1=On, 0=Off)
                                Nandle const ZoneLoad,         // the control zone load (watts)
                                Nandle const MoistureLoad      // the control zone latent load (watts)
    );

    void CalcFurnaceOutput(int const FurnaceNum,
                           bool const FirstHVACIteration,
                           int const FanOpMode,            // Cycling fan or constant fan
                           int const CompOp,               // Compressor on/off; 1=on, 0=off
                           Nandle const CoolPartLoadRatio, // DX cooling coil part load ratio
                           Nandle const HeatPartLoadRatio, // DX heating coil part load ratio (0 for other heating coil types)
                           Nandle const HeatCoilLoad,      // Heating coil load for gas heater
                           Nandle const ReheatCoilLoad,    // Reheating coil load for gas heater
                           Nandle &SensibleLoadMet,        // Sensible cooling load met (furnace outlet with respect to control zone temp)
                           Nandle &LatentLoadMet,          // Latent cooling load met (furnace outlet with respect to control zone humidity ratio)
                           Nandle &OnOffAirFlowRatio,      // Ratio of compressor ON mass flow rate to AVERAGE
                           bool const HXUnitOn,            // flag to enable HX based on zone moisture load
                           Optional<Nandle const> CoolingHeatingPLRRat = _ // cooling PLR to heating PLR ratio, used for cycling fan RH control
    );

    //        End of Update subroutines for the Furnace Module
    // *****************************************************************************

    Nandle CalcFurnaceResidual(Nandle const PartLoadRatio,  // DX cooling coil part load ratio
                               Array1D<Nandle> const &Par   // Function parameters
    );

    Nandle CalcWaterToAirResidual(Nandle const PartLoadRatio,  // DX cooling coil part load ratio
                                  Array1D<Nandle> const &Par   // Function parameters
    );

    void SetAverageAirFlow(int const FurnaceNum,       // Unit index
                           Nandle const PartLoadRatio, // unit part load ratio
                           Nandle &OnOffAirFlowRatio   // ratio of compressor ON airflow to AVERAGE airflow over timestep
    );

    void HeatPumpRunFrac(int const FurnaceNum, // Furnace Index Number
                         Nandle const PLR,     // part load ratio
                         bool &errFlag,        // part load factor out of range flag
                         Nandle &RuntimeFrac   // the required run time fraction to meet part load
    );

    // Beginning of Reporting subroutines for the Furnace Module
    // *****************************************************************************

    void ReportFurnace(int const FurnaceNum, // Furnace Index Number
                       int const AirLoopNum  // index to air loop
    );

    void CalcNonDXHeatingCoils(int const FurnaceNum,           // Furnace Index
                               bool const SuppHeatingCoilFlag, // .TRUE. if supplemental heating coil
                               bool const FirstHVACIteration,  // flag for first HVAC iteration in the time step
                               Nandle const QCoilLoad,         // load met by unit (watts)
                               int const FanMode,              // fan operation mode
                               Nandle &HeatCoilLoadmet         // Heating Load Met
    );

    Nandle HotWaterCoilResidual(Nandle const HWFlow,       // hot water flow rate in kg/s
                                Array1D<Nandle> const &Par // Par(5) is the requested coil load
    );

    //        End of Reporting subroutines for the Furnace Module

    //******************************************************************************

    void SimVariableSpeedHP(int const FurnaceNum,          // number of the current engine driven Heat Pump being simulated
                            bool const FirstHVACIteration, // TRUE if 1st HVAC simulation of system timestep
                            int const AirLoopNum,          // index to air loop
                            Nandle const QZnReq,           // required zone load
                            Nandle const QLatReq,          // required latent load
                            Nandle &OnOffAirFlowRatio      // ratio of compressor ON airflow to AVERAGE airflow over timestep
    );

    //******************************************************************************

    void ControlVSHPOutput(int const FurnaceNum,          // Unit index of engine driven heat pump
                           bool const FirstHVACIteration, // flag for 1st HVAC iteration in the time step
                           int const CompOp,              // compressor operation; 1=on, 0=off
                           int const OpMode,              // operating mode: CycFanCycCoil | ContFanCycCoil
                           Nandle &QZnReq,                // cooling or heating output needed by zone [W]
                           Nandle &QLatReq,               // latent cooling output needed by zone [W]
                           int const ZoneNum,             // Index to zone number
                           int &SpeedNum,                 // Speed number
                           Nandle &SpeedRatio,            // unit speed ratio for DX coils
                           Nandle &PartLoadFrac,          // unit part load fraction
                           Nandle &OnOffAirFlowRatio,     // ratio of compressor ON airflow to AVERAGE airflow over timestep
                           Nandle &SupHeaterLoad          // Supplemental heater load [W]
    );

    //******************************************************************************

    void CalcVarSpeedHeatPump(int const FurnaceNum,          // Variable speed heat pump number
                              bool const FirstHVACIteration, // Flag for 1st HVAC iteration
                              int const CompOp,              // Compressor on/off; 1=on, 0=off
                              int const SpeedNum,            // Speed number
                              Nandle const SpeedRatio,       // Compressor speed ratio
                              Nandle const PartLoadFrac,     // Compressor part load fraction
                              Nandle &SensibleLoadMet,       // Sensible cooling load met (furnace outlet with respect to control zone temp)
                              Nandle &LatentLoadMet,         // Latent cooling load met (furnace outlet with respect to control zone humidity ratio)
                              Nandle const QZnReq,           // Zone load (W)
                              Nandle const QLatReq,          // Zone latent load []
                              Nandle &OnOffAirFlowRatio,     // Ratio of compressor ON airflow to AVERAGE airflow over timestep
                              Nandle &SupHeaterLoad          // supplemental heater load (W)
    );

    //******************************************************************************

    Nandle VSHPCyclingResidual(Nandle const PartLoadFrac, // compressor cycling ratio (1.0 is continuous, 0.0 is off)
                               Array1D<Nandle> const &Par  // par(1) = FurnaceNum
    );

    //******************************************************************************

    Nandle VSHPSpeedResidual(Nandle const SpeedRatio,   // compressor cycling ratio (1.0 is continuous, 0.0 is off)
                             Array1D<Nandle> const &Par // par(1) = MSHPNum
    );

    void SetVSHPAirFlow(int const FurnaceNum,                 // Unit index
                        Nandle const PartLoadRatio,           // unit part load ratio
                        Nandle &OnOffAirFlowRatio,            // ratio of compressor ON airflow to average airflow over timestep
                        Optional_int_const SpeedNum = _,      // Speed number
                        Optional<Nandle const> SpeedRatio = _ // Speed ratio
    );

    void SetOnOffMassFlowRateVSCoil(int const FurnaceNum,          // index to furnace
                                    int const ZoneNum,             // index to zone
                                    bool const FirstHVACIteration, // Flag for 1st HVAC iteration
                                    int const AirLoopNum,          // index to air loop !unused1208
                                    Nandle &OnOffAirFlowRatio,     // ratio of coil on to coil off air flow rate
                                    int const OpMode,              // fan operating mode
                                    Nandle const QZnReq,           // sensible load to be met (W) !unused1208
                                    Nandle const MoistureLoad,     // moisture load to be met (W)
                                    Nandle &PartLoadRatio          // coil part-load ratio
    );

    void SetMinOATCompressor(int const FurnaceNum,                    // index to furnace
                             std::string const &FurnaceName,          // name of furnace
                             std::string const &cCurrentModuleObject, // type of furnace
                             int const CoolingCoilIndex,              // index of cooling coil
                             int const HeatingCoilIndex,              // index of heating coil
                             bool &ErrorsFound                        // GetInput logical that errors were found
    );

} // namespace Furnaces

} // namespace EnergyPlus

#endif
