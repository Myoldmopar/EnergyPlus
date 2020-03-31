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

#ifndef PackagedTerminalHeatPump_hh_INCLUDED
#define PackagedTerminalHeatPump_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>
#include <ObjexxFCL/Optional.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>
#include <EnergyPlus/VariableSpeedCoils.hh>

namespace EnergyPlus {

namespace PackagedTerminalHeatPump {

    // Using/Aliasing
    using VariableSpeedCoils::MaxSpedLevels;

    // Data
    // MODULE PARAMETER DEFINITIONS
    // Compressor operation
    extern int const On;  // normal compressor operation
    extern int const Off; // signal DXCoil that compressor shouldn't run

    // Last mode of operation
    extern int const CoolingMode; // last compressor operating mode was in cooling
    extern int const HeatingMode; // last compressor operating mode was in heating

    // Airflow control for contant fan mode
    extern int const UseCompressorOnFlow;  // set compressor OFF air flow rate equal to compressor ON air flow rate
    extern int const UseCompressorOffFlow; // set compressor OFF air flow rate equal to user defined value

    // Unit type
    extern int const PTHPUnit;   // equivalent to PackagedTerminal:HeatPump:AirToAir
    extern int const PTACUnit;   // equivalent to PackagedTerminal:AirConditioner
    extern int const PTWSHPUnit; // equivalent to WaterToAirHeatPump

    // DERIVED TYPE DEFINITIONS

    // MODULE VARIABLE DECLARATIONS:
    extern Array1D_bool CheckEquipName;

    extern Nandle SupHeaterLoad;     // load to be met by supplemental heater [W]
    extern int NumPTHP;              // total number of PTHP's
    extern int NumPTAC;              // total number of PTAC's
    extern int NumPTWSHP;            // total number of PTWSHP's
    extern int NumPTUs;              // total number of PTHP and PTAC units
    extern Nandle CompOnMassFlow;    // Supply air mass flow rate w/ compressor ON
    extern Nandle OACompOnMassFlow;  // OA mass flow rate w/ compressor ON
    extern Nandle CompOffMassFlow;   // Supply air mass flow rate w/ compressor OFF
    extern Nandle OACompOffMassFlow; // OA mass flow rate w/ compressor OFF
    extern Nandle CompOnFlowRatio;   // fan flow ratio when coil on
    extern Nandle CompOffFlowRatio;  // fan flow ratio when coil off
    extern Nandle FanSpeedRatio;     // ratio of air flow ratio passed to fan object
    extern bool GetPTUnitInputFlag;  // First time, input is "gotten"
    extern Nandle SaveCompressorPLR; // holds compressor PLR from active DX coil
    extern Nandle SteamDensity;      // density of steam at 100C, used for steam heating coils
    extern bool HeatingLoad;         // defines a heating load on PTUnit
    extern bool CoolingLoad;         // defines a cooling load on PTUnit
    extern Nandle MinWaterFlow;      // minimum water flow for heating [kg/s]
    extern Nandle TempSteamIn;       // steam coil steam inlet temperature

    // SUBROUTINE SPECIFICATIONS FOR MODULE

    // modules for variable speed heat pump

    // Types

    struct PTUnitData
    {
        // Members
        // input data
        int UnitType_Num;                // paramter equivalent to type of unit
        int ZoneEquipType;               // Type of PT unit
        bool useVSCoilModel;             // does PT use VS coil models
        int SchedPtr;                    // index number to availability schedule
        Nandle MaxCoolAirVolFlow;        // supply air volumetric flow rate during cooling operation [m3/s]
        Nandle MaxHeatAirVolFlow;        // supply air volumetric flow rate during heating operation [m3/s]
        Nandle MaxNoCoolHeatAirVolFlow;  // supply air volumetric flow rate when no cooling or heating [m3/s]
        Nandle CoolOutAirVolFlow;        // OA volumetric flow rate during cooling operation [m3/s]
        Nandle HeatOutAirVolFlow;        // OA volumetric flow rate during heating operation [m3/s]
        Nandle NoCoolHeatOutAirVolFlow;  // OA volumetric flow rate when no cooling or heating [m3/s]
        Nandle CoolOutAirMassFlow;       // OA mass flow rate during cooling operation [kg/s]
        Nandle HeatOutAirMassFlow;       // OA mass flow rate during heating operation [kg/s]
        Nandle NoCoolHeatOutAirMassFlow; // OA mass flow rate when no cooling or heating [kg/s]
        int OutsideAirNode;              // OAmixer outside air node number
        int AirReliefNode;               // OAmixer relief air node number
        std::string OAMixType;           // type of outside air mixer
        std::string OAMixName;           // name of OAmixer
        int OAMixIndex;
        std::string FanName;        // name of fan
        std::string FanType;        // type of fan
        int FanType_Num;            // fan type number (see DataHVACGlobals)
        int FanIndex;               // index number to fan
        int FanSchedPtr;            // index number to fan operating mode schedule
        int FanAvailSchedPtr;       // index to fan availability schedule
        std::string DXCoolCoilName; // name of DX cooling coil
        std::string DXCoolCoilType; // type of DX cooling coil,Coil:DX:CoolingBypassFactorEmpirical or
        //                        'CoilSystem:Cooling:DX:HeatExchangerAssisted'
        int DXCoolCoilType_Num;           // numeric equivalent for DX cooling coil type
        int CoolCoilCompIndex;            // cooling coil index number (index for DX coil or HX Assisted object)
        int DXCoolCoilIndexNum;           // actual DX cooling coil index number
        int CondenserNodeNum;             // DX cooling coil condenser node number
        int DXHeatCoilIndexNum;           // actual DX heating coil index number
        std::string DXHeatCoilName;       // name of DX heating coil
        std::string DXHeatCoilType;       // type of DX heating coil,Coil:DX:HeatingEmpirical
        int DXHeatCoilType_Num;           // numeric equivalent for DX heating coil type
        std::string ACHeatCoilName;       // name of heating coil for PTAC
        std::string ACHeatCoilType;       // type of heating coil for PTAC
        Nandle ACHeatCoilCap;             // heating coil capacity for PTAC
        int ACHeatCoilIndex;              // heating coil index number for PTAC
        int SuppCoilFluidInletNode;       // steam inlet node number of HW coil for PTAC and HP
        int HWCoilSteamOutletNode;        // steam inlet node number of HW coil for PTAC and HP
        std::string SuppHeatCoilName;     // name of supplemental heating coil
        int SuppHeatCoilType_Num;         // numeric equivalent for supplemental heating coil type
        int ACHeatCoilType_Num;           // numeric equivalent for PTAC heating coil type
        int SuppHeatCoilIndex;            // supplemental heater index number
        int SupHeatCoilCap;               // supplemental heater coil capacity [W]
        int SupCoilAirInletNode;          // air inlet node for supplemental coil for HP
        std::string SuppHeatCoilType;     // supplemental heater coil type
        Nandle MaxSATSupHeat;             // maximum supply air temperature from supplemental heater [C]
        Nandle MaxOATSupHeat;             // maximum outdoor air temp for supplemental heater operation [C]
        int OpMode;                       // mode of operation; 1=cycling fan, cycling compressor, 2=continuous fan, cycling compresor
        int FanPlace;                     // fan placement;     1=blow through, 2=draw through
        Nandle CoolConvergenceTol;        // Convergence tolerance, fraction (ZoneLoad - Equip Output)/ZoneLoad
        Nandle HeatConvergenceTol;        // Convergence tolerance, fraction (ZoneLoad - Equip Output)/ZoneLoad
        Nandle MinOATCompressorCooling;   // Minimum OAT for compressor operation in cooling mode [C]
        Nandle MinOATCompressorHeating;   // Minimum OAT for compressor operation in heating mode [C]
        int IterErrIndex;                 // index for recurring warnings
        std::string AvailManagerListName; // Name of an availability manager list object
        int WaterCyclingMode;             // Heat Pump Coil water flow mode; See definitions in DataHVACGlobals,
        // 1=water cycling, 2=water constant, 3=water constant on demand (old mode)
        int PTObjectIndex; // index for PT unit
        // Water source HP specific variables
        Nandle MaxONOFFCyclesperHour; // Maximum ON/OFF Cycling Rate [cycles/hr]
        Nandle HPTimeConstant;        // Heat Pump Time Constant [s]
        Nandle OnCyclePowerFraction;  // Fraction of on-cycle power use [~]
        // supplemental heating coil operation
        Nandle FanDelayTime; // Fan delay time, time delay for the HP's fan to
        // shut off after compressor cycle off  [s]
        Nandle DesignHeatingCapacity;     // Nominal Capacity of Heating Coil [W]
        Nandle DesignCoolingCapacity;     // Nominal Capacity of Cooling Coil [W]
        Nandle DesignSuppHeatingCapacity; // Nominal Capacity of Supplemental Heating Coil [W]
        // addition for OA to Zone Units
        bool ATMixerExists;      // True if there is an ATMixer
        std::string ATMixerName; // name of air terminal mixer
        int ATMixerIndex;        // index to the air terminal mixer
        int ATMixerType;         // 1 = inlet side mixer, 2 = supply side mixer
        int ATMixerPriNode;      // primary inlet air node number for the air terminal mixer
        int ATMixerSecNode;      // secondary air inlet node number for the air terminal mixer
        int ATMixerOutNode;      // outlet air node number for the air terminal mixer
        // Report data
        Nandle TotHeatEnergyRate;      // total heating output [W]
        Nandle TotHeatEnergy;          // total heating output [J]
        Nandle TotCoolEnergyRate;      // total cooling output [W]
        Nandle TotCoolEnergy;          // total cooling output [J]
        Nandle SensHeatEnergyRate;     // sensible heating output [W]
        Nandle SensHeatEnergy;         // sensible heating output [J]
        Nandle SensCoolEnergyRate;     // sensible cooling output [W]
        Nandle SensCoolEnergy;         // sensible cooling output [J]
        Nandle LatHeatEnergyRate;      // latent heating output [W]
        Nandle LatHeatEnergy;          // latent heating output [J]
        Nandle LatCoolEnergyRate;      // latent cooling output [W]
        Nandle LatCoolEnergy;          // latent cooling output [J]
        Nandle ElecPower;              // electricity consumed [W]
        Nandle ElecConsumption;        // electricity consumed [J]
        Nandle CompPartLoadRatio;      // compressor part-load ratio for time step
        int LastMode;                  // last mode of operation, coolingmode or heatingmode
        int AirFlowControl;            // fan control mode, UseCompressorOnFlow or UseCompressorOffFlow
        int ControlType;               // Setpoint, Load based or ASHRAE (SZVAV) control
        bool validASHRAECoolCoil;      // cooling coil model that conforms to ASHRAE 90.1 requirements and methodology
        bool validASHRAEHeatCoil;      // heating coil model that conforms to ASHRAE 90.1 requirements and methodology
        bool simASHRAEModel;           // flag denoting that ASHRAE model (SZVAV) should be used
        Nandle CompPartLoadFrac;       // compressor part load ratio
        int PlantCoilOutletNode;       // outlet node for water coil
        int SuppCoilLoopNum;           // plant loop index for water heating coil
        int SuppCoilLoopSide;          // plant loop side  index for water heating coil
        int SuppCoilBranchNum;         // plant loop branch index for water heating coil
        int SuppCoilCompNum;           // plant loop component index for water heating coil
        Nandle MaxSuppCoilFluidFlow;   // water or steam mass flow rate supp. heating coil [kg/s]
        int HotWaterCoilMaxIterIndex;  // Index to recurring warning message
        int HotWaterCoilMaxIterIndex2; // Index to recurring warning message
        Nandle ActualFanVolFlowRate;   // Volumetric flow rate from fan object
        Nandle HeatingSpeedRatio;      // Fan speed ratio in heating mode
        Nandle CoolingSpeedRatio;      // Fan speed ratio in cooling mode
        Nandle NoHeatCoolSpeedRatio;   // Fan speed ratio when no cooling or heating
        int AvailStatus;
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
        int CompSpeedNum;
        Nandle CompSpeedRatio;
        int ErrIndexCyc;
        int ErrIndexVar;
        int ZonePtr;                    // pointer to a zone served by a fancoil unit
        int HVACSizingIndex;            // index of a HVACSizing object for a fancoil unit
        bool FirstPass;                 // used to reset sizing flags
        Nandle HeatCoilWaterFlowRate;   //
        Nandle ControlZoneMassFlowFrac; //

        // variables used in SZVAV model:
        std::string Name;                // name of unit
        std::string UnitType;            // type of unit
        int MaxIterIndex;                // used in PLR calculations for sensible load
        int NodeNumOfControlledZone;     // node number of control zone
        int RegulaFalsiFailedIndex;      // used in PLR calculations for sensible load
        Nandle FanPartLoadRatio;         // fan part-load ratio for time step
        Nandle CoolCoilWaterFlowRatio;   // holds ratio of max cool coil water flow rate, may be < 1 when FlowLock is true
        Nandle HeatCoilWaterFlowRatio;   // holds ratio of max heat coil water flow rate, may be < 1 when FlowLock is true
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
        Nandle MaxCoolCoilFluidFlow;     // water flow rate for cooling coil [kg/s] - NOT USED in PTHP
        Nandle MaxHeatCoilFluidFlow;     // water or steam mass flow rate for heating coil [kg/s]
        int CoolCoilLoopNum;             // plant loop index for water cooling coil - NOT USED in PTHP
        int CoolCoilLoopSide;            // plant loop side  index for water cooling coil - NOT USED in PTHP
        int CoolCoilBranchNum;           // plant loop branch index for water cooling coil - NOT USED in PTHP
        int CoolCoilCompNum;             // plant loop component index for water cooling coil - NOT USED in PTHP
        int HeatCoilLoopNum;             // plant loop index for water heating coil
        int HeatCoilLoopSide;            // plant loop side  index for water heating coil
        int HeatCoilBranchNum;           // plant loop branch index for water heating coil
        int HeatCoilCompNum;             // plant loop component index for water heating coil
        int CoolCoilFluidInletNode;      // water cooling coil water inlet node number NOT USED in PTHP
        int CoolCoilFluidOutletNodeNum;  // water cooling coil water outlet node number NOT USED in PTHP
        int CoolCoilInletNodeNum;        // cooling coil air inlet node number
        int CoolCoilOutletNodeNum;       // cooling coil air outlet node number
        int HeatCoilFluidInletNode;      // heating coil fluid (e.g., water or steam) inlet node number
        int HeatCoilFluidOutletNodeNum;  // heating coil fluid (e.g., water or steam) outlet node number
        int HeatCoilInletNodeNum;        // heating coil air inlet node number
        int HeatCoilOutletNodeNum;       // heating coil air outlet node number

        // end of the additional variables for variable speed water source heat pump

        // Default Constructor
        PTUnitData()
            : UnitType_Num(0), ZoneEquipType(0), useVSCoilModel(false), SchedPtr(0), MaxCoolAirVolFlow(0.0), MaxHeatAirVolFlow(0.0),
              MaxNoCoolHeatAirVolFlow(0.0), CoolOutAirVolFlow(0.0), HeatOutAirVolFlow(0.0), NoCoolHeatOutAirVolFlow(0.0), CoolOutAirMassFlow(0.0),
              HeatOutAirMassFlow(0.0), NoCoolHeatOutAirMassFlow(0.0), OutsideAirNode(0), AirReliefNode(0), OAMixIndex(0), FanType_Num(0), FanIndex(0),
              FanSchedPtr(0), FanAvailSchedPtr(0), DXCoolCoilType_Num(0), CoolCoilCompIndex(0), DXCoolCoilIndexNum(0), CondenserNodeNum(0),
              DXHeatCoilIndexNum(0), DXHeatCoilType_Num(0), ACHeatCoilCap(0.0), ACHeatCoilIndex(0), SuppCoilFluidInletNode(0),
              HWCoilSteamOutletNode(0), SuppHeatCoilType_Num(0), ACHeatCoilType_Num(0), SuppHeatCoilIndex(0), SupHeatCoilCap(0),
              SupCoilAirInletNode(0), MaxSATSupHeat(0.0), MaxOATSupHeat(0.0), OpMode(0), FanPlace(0), CoolConvergenceTol(0.0),
              HeatConvergenceTol(0.0), MinOATCompressorCooling(0.0), MinOATCompressorHeating(0.0), IterErrIndex(0), WaterCyclingMode(0),
              PTObjectIndex(0), MaxONOFFCyclesperHour(0.0), HPTimeConstant(0.0), OnCyclePowerFraction(0.0), FanDelayTime(0.0),
              DesignHeatingCapacity(0.0), DesignCoolingCapacity(0.0), DesignSuppHeatingCapacity(0.0), ATMixerExists(false), ATMixerIndex(0),
              ATMixerType(0), ATMixerPriNode(0), ATMixerSecNode(0), ATMixerOutNode(0), TotHeatEnergyRate(0.0), TotHeatEnergy(0.0),
              TotCoolEnergyRate(0.0), TotCoolEnergy(0.0), SensHeatEnergyRate(0.0), SensHeatEnergy(0.0), SensCoolEnergyRate(0.0), SensCoolEnergy(0.0),
              LatHeatEnergyRate(0.0), LatHeatEnergy(0.0), LatCoolEnergyRate(0.0), LatCoolEnergy(0.0), ElecPower(0.0), ElecConsumption(0.0),
              CompPartLoadRatio(0.0), LastMode(0), AirFlowControl(0), ControlType(0), validASHRAECoolCoil(false), validASHRAEHeatCoil(false),
              simASHRAEModel(false), CompPartLoadFrac(0.0), PlantCoilOutletNode(0), SuppCoilLoopNum(0), SuppCoilLoopSide(0), SuppCoilBranchNum(0),
              SuppCoilCompNum(0), MaxSuppCoilFluidFlow(0.0), HotWaterCoilMaxIterIndex(0), HotWaterCoilMaxIterIndex2(0), ActualFanVolFlowRate(0.0),
              HeatingSpeedRatio(1.0), CoolingSpeedRatio(1.0), NoHeatCoolSpeedRatio(1.0), AvailStatus(0), HeatCoolMode(0), NumOfSpeedCooling(0),
              NumOfSpeedHeating(0), IdleSpeedRatio(0.0), IdleVolumeAirRate(0.0), IdleMassFlowRate(0.0), FanVolFlow(0.0), CheckFanFlow(true),
              HeatVolumeFlowRate(MaxSpedLevels, 0.0), HeatMassFlowRate(MaxSpedLevels, 0.0), CoolVolumeFlowRate(MaxSpedLevels, 0.0),
              CoolMassFlowRate(MaxSpedLevels, 0.0), MSHeatingSpeedRatio(MaxSpedLevels, 0.0), MSCoolingSpeedRatio(MaxSpedLevels, 0.0), CompSpeedNum(0),
              CompSpeedRatio(0.0), ErrIndexCyc(0), ErrIndexVar(0), ZonePtr(0), HVACSizingIndex(0), FirstPass(true), HeatCoilWaterFlowRate(0.0),
              ControlZoneMassFlowFrac(1.0),
              // variables used in SZVAV model:
              MaxIterIndex(0), NodeNumOfControlledZone(0), RegulaFalsiFailedIndex(0), FanPartLoadRatio(0.0), CoolCoilWaterFlowRatio(0.0),
              HeatCoilWaterFlowRatio(0.0), ControlZoneNum(0), AirInNode(0), AirOutNode(0), MaxCoolAirMassFlow(0.0), MaxHeatAirMassFlow(0.0),
              MaxNoCoolHeatAirMassFlow(0.0), DesignMinOutletTemp(0.0), DesignMaxOutletTemp(0.0), LowSpeedCoolFanRatio(0.0), LowSpeedHeatFanRatio(0.0),
              MaxCoolCoilFluidFlow(0.0), MaxHeatCoilFluidFlow(0.0), CoolCoilLoopNum(0), CoolCoilLoopSide(0), CoolCoilBranchNum(0), CoolCoilCompNum(0),
              HeatCoilLoopNum(0), HeatCoilLoopSide(0), HeatCoilBranchNum(0), HeatCoilCompNum(0), CoolCoilFluidInletNode(0),
              CoolCoilFluidOutletNodeNum(0), CoolCoilInletNodeNum(0), CoolCoilOutletNodeNum(0), HeatCoilFluidInletNode(0),
              HeatCoilFluidOutletNodeNum(0), HeatCoilInletNodeNum(0), HeatCoilOutletNodeNum(0)
        {
        }
    };

    struct PTUnitNumericFieldData
    {
        // Members
        Array1D_string FieldNames;

        // Default Constructor
        PTUnitNumericFieldData()
        {
        }
    };

    // Object Data
    extern Array1D<PTUnitData> PTUnit;
    extern Array1D<PTUnitNumericFieldData> PTUnitUNumericFields; // holds PT unit numeric input fields character field name

    // Functions

    void clear_state();

    void SimPackagedTerminalUnit(std::string const &CompName,   // name of the packaged terminal heat pump
                                 int const ZoneNum,             // number of zone being served
                                 bool const FirstHVACIteration, // TRUE if 1st HVAC simulation of system timestep
                                 Nandle &QUnitOut,              // sensible capacity delivered to zone
                                 Nandle &LatOutputProvided,     // Latent add/removal by packaged terminal unit (kg/s), dehumid = negative
                                 int const PTUnitType,          // indicates whether PTAC, PTHP or PTWSHP
                                 int &CompIndex                 // index to Packaged Terminal Heat Pump
    );

    void SimPTUnit(int const PTUnitNum,           // number of the current Packaged Terminal Heat Pump being simulated
                   int const ZoneNum,             // number of zone being served
                   bool const FirstHVACIteration, // TRUE if 1st HVAC simulation of system timestep
                   Nandle &QSensUnitOut,          // sensible delivered capacity [W]
                   Nandle &OnOffAirFlowRatio,     // ratio of compressor ON airflow to AVERAGE airflow over timestep
                   Nandle const QZnReq,           // cooling/heating needed by zone [W]
                   Nandle &QLatUnitOut            // Latent delivered capacity [kg/s], dehumidification = negative
    );

    void GetPTUnit();

    void InitPTUnit(int const PTUnitNum,           // number of the current PTHP unit being simulated
                    int const ZoneNum,             // zone number where the current PTHP unit is located
                    bool const FirstHVACIteration, // TRUE on first HVAC iteration
                    Nandle &OnOffAirFlowRatio,     // ratio of compressor ON airflow to average airflow over timestep
                    Nandle &ZoneLoad               // cooling or heating needed by zone [watts]
    );

    void SetOnOffMassFlowRate(int const PTUnitNum,       // number of the current PTHP unit being simulated
                              Nandle const PartLoadFrac, // coil operating part-load ratio
                              Nandle &OnOffAirFlowRatio  // ratio of coil on to coil off air flow rate
    );

    void SizePTUnit(int const PTUnitNum);

    void ControlPTUnitOutput(int const PTUnitNum,           // Unit index in fan coil array
                             bool const FirstHVACIteration, // flag for 1st HVAC iteration in the time step
                             int const OpMode,              // operating mode: CycFanCycCoil | ContFanCycCoil
                             Nandle const QZnReq,           // cooling or heating output needed by zone [W]
                             int const ZoneNum,             // Index to zone number
                             Nandle &PartLoadFrac,          // unit part load fraction
                             Nandle &OnOffAirFlowRatio,     // ratio of compressor ON airflow to AVERAGE airflow over timestep
                             Nandle &SupHeaterLoad,         // Supplemental heater load [W]
                             bool &HXUnitOn                 // flag to enable heat exchanger
    );

    void CalcPTUnit(int const PTUnitNum,           // Unit index in fan coil array
                    bool const FirstHVACIteration, // flag for 1st HVAC iteration in the time step
                    Nandle const PartLoadFrac,     // compressor part load fraction
                    Nandle &LoadMet,               // load met by unit (W)
                    Nandle const QZnReq,           // Zone load (W) unused1208
                    Nandle &OnOffAirFlowRatio,     // ratio of compressor ON airflow to AVERAGE airflow over timestep
                    Nandle &SupHeaterLoad,         // supplemental heater load (W)
                    bool const HXUnitOn            // flag to enable heat exchanger
    );

    void HeatPumpRunFrac(int const PTUnitNum, // PTAC Index Number
                         Nandle const PLR,    // part load ratio
                         bool &errFlag,       // part load factor out of range flag
                         Nandle &RuntimeFrac  // the required run time fraction to meet part load
    );

    Nandle HotWaterCoilResidual(Nandle const HWFlow,       // hot water flow rate in kg/s
                                Array1D<Nandle> const &Par // Par(5) is the requested coil load
    );

    Nandle SupSATResidual(Nandle &TempSupHeater,     // supplemental heater load at maximum SAT
                          Array1D<Nandle> const &Par // par(1) = PTUnitNum
    );

    Nandle PLRResidual(Nandle const PartLoadFrac, // compressor cycling ratio (1.0 is continuous, 0.0 is off)
                       Array1D<Nandle> const &Par // par(1) = PTUnitNum
    );

    void SetAverageAirFlow(int const PTUnitNum,        // Unit index
                           Nandle const PartLoadRatio, // unit part load ratio
                           Nandle &OnOffAirFlowRatio   // ratio of compressor ON airflow to average airflow over timestep
    );

    void ReportPTUnit(int const PTUnitNum); // number of the current AC unit being simulated

    int GetPTUnitZoneInletAirNode(int const PTUnitCompIndex, int const PTUnitType);

    int GetPTUnitOutAirNode(int const PTUnitCompIndex, int const PTUnitType);

    int GetPTUnitReturnAirNode(int const PTUnitCompIndex, int const PTUnitType);

    int GetPTUnitMixedAirNode(int const PTUnitCompIndex, int const PTUnitType);

    //******************************************************************************

    void SimVariableSpeedHP(int const PTUnitNum,           // number of the current engine driven Heat Pump being simulated
                            int const ZoneNum,             // Controlled zone number
                            bool const FirstHVACIteration, // TRUE if 1st HVAC simulation of system timestep
                            Nandle const QZnReq,           // required zone load
                            Nandle const QLatReq,          // required latent load
                            Nandle &OnOffAirFlowRatio,     // ratio of compressor ON airflow to AVERAGE airflow over timestep
                            int const OpMode,              // operating mode: CycFanCycCoil | ContFanCycCoil
                            bool const HXUnitOn            // flag to enable heat exchanger
    );

    //******************************************************************************
    //******************************************************************************

    void ControlVSHPOutput(int const PTUnitNum,           // Unit index in fan coil array
                           bool const FirstHVACIteration, // flag for 1st HVAC iteration in the time step
                           int const CompOp,              // compressor operation; 1=on, 0=off
                           int const OpMode,              // operating mode: CycFanCycCoil | ContFanCycCoil
                           Nandle const QZnReq,           // cooling or heating output needed by zone [W]
                           Nandle const QLatReq,          // latent cooling output needed by zone [W]
                           int const ZoneNum,             // Index to zone number
                           int &SpeedNum,                 // Speed number
                           Nandle &SpeedRatio,            // unit speed ratio for DX coils
                           Nandle &PartLoadFrac,          // unit part load fraction
                           Nandle &OnOffAirFlowRatio,     // ratio of compressor ON airflow to AVERAGE airflow over timestep
                           Nandle &SupHeaterLoad,         // Supplemental heater load [W]
                           bool const HXUnitOn            // flag to enable heat exchanger
    );

    //******************************************************************************

    //******************************************************************************

    Nandle VSHPCyclingResidual(Nandle const PartLoadFrac, // compressor cycling ratio (1.0 is continuous, 0.0 is off)
                               Array1D<Nandle> const &Par // par(1) = FurnaceNum
    );

    //******************************************************************************

    Nandle VSHPSpeedResidual(Nandle const SpeedRatio,   // compressor cycling ratio (1.0 is continuous, 0.0 is off)
                             Array1D<Nandle> const &Par // par(1) = MSHPNum
    );

    //******************************************************************************

    void CalcVarSpeedHeatPump(int const PTUnitNum,           // Unit index in fan coil array
                              int const ZoneNum,             // Zone index
                              bool const FirstHVACIteration, // flag for 1st HVAC iteration in the time step
                              int const CompOp,              // Compressor on/off; 1=on, 0=off
                              int const SpeedNum,            // Speed number
                              Nandle const SpeedRatio,       // Compressor speed ratio
                              Nandle const PartLoadFrac,     // compressor part load fraction
                              Nandle &LoadMet,               // load met by unit (W)
                              Nandle &LatentLoadMet,         // Latent cooling load met (furnace outlet with respect to control zone humidity ratio)
                              Nandle const QZnReq,           // Zone load (W) unused1208
                              Nandle const QLatReq,          // Zone latent load []
                              Nandle &OnOffAirFlowRatio,     // ratio of compressor ON airflow to AVERAGE airflow over timestep
                              Nandle &SupHeaterLoad,         // supplemental heater load (W)
                              bool const HXUnitOn            // flag to enable heat exchanger
    );

    void SetVSHPAirFlow(int const PTUnitNum,                  // Unit index
                        int const ZoneNum,                    // Zone index
                        Nandle const PartLoadRatio,           // unit part load ratio
                        Nandle &OnOffAirFlowRatio,            // ratio of compressor ON airflow to average airflow over timestep
                        Optional_int_const SpeedNum = _,      // Speed number
                        Optional<Nandle const> SpeedRatio = _ // Speed ratio
    );

    void SetOnOffMassFlowRateVSCoil(int const PTUnitNum,           // index to furnace
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

    Nandle CalcPTUnitWaterFlowResidual(Nandle const PartLoadRatio, // coil PLR
                                       Array1D<Nandle> const &Par  // Function parameters
    );

    Nandle CalcPTUnitAirAndWaterFlowResidual(Nandle const PartLoadRatio, // coil PLR
                                             Array1D<Nandle> const &Par  // Function parameters
    );

} // namespace PackagedTerminalHeatPump

} // namespace EnergyPlus

#endif
