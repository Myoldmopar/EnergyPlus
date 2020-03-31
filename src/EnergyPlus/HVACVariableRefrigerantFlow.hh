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

#ifndef HVACVariableRefrigerantFlow_hh_INCLUDED
#define HVACVariableRefrigerantFlow_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>
#include <ObjexxFCL/Optional.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>
#include <EnergyPlus/PlantComponent.hh>
#include <EnergyPlus/SingleDuct.hh>

namespace EnergyPlus {

namespace HVACVariableRefrigerantFlow {

    // Compressor operation
    extern int const On;  // normal compressor operation
    extern int const Off; // signal DXCoil that compressor shouldn't run

    // Heat Recovery System used
    extern int const No;  // Heat Pump mode only
    extern int const Yes; // Heat Pump or Heat Recovery Mode (not available at this time)

    // Defrost strategy
    extern int const ReverseCycle; // uses reverse cycle defrost strategy
    extern int const Resistive;    // uses electric resistance heater for defrost

    // Defrost control
    extern int const Timed;    // defrost cycle is timed
    extern int const OnDemand; // defrost cycle occurs only when required

    // Thermostat Priority Control Type
    extern int const LoadPriority;             // total of zone loads dictate operation in cooling or heating
    extern int const ZonePriority;             // # of zones requireing cooling or heating dictate operation in cooling or heating
    extern int const ThermostatOffsetPriority; // zone with largest deviation from setpoint dictates operation
    extern int const ScheduledPriority;        // cooling and heating modes are scheduled
    extern int const MasterThermostatPriority; // Master zone thermostat dictates operation
    extern int const FirstOnPriority;          // first unit to respond dictates operation (not used at this time)

    // Water Systems
    extern int const CondensateDiscarded; // default mode where water is "lost"
    extern int const CondensateToTank;    // collect coil condensate from air and store in water storage tank

    extern int const WaterSupplyFromMains; // mains water line used as water source
    extern int const WaterSupplyFromTank;  // storage tank used as water source

    extern Nandle const MaxCap; // limit of zone terminal unit capacity

    // VRF System Types (strings used in integer conversions)
    extern int const NumVRFSystemTypes;
    extern int const VRF_HeatPump;
    extern Array1D_string const cVRFTypes;

    extern int const NumValidFuelTypes;
    extern Array1D_string const cValidFuelTypes;

    // Fuel Types
    extern int const FuelTypeElectric;   // Fuel type for electricity
    extern int const FuelTypeNaturalGas; // Fuel type for natural gas
    extern int const FuelTypePropaneGas; // Fuel type for propane gas
    extern int const FuelTypeDiesel;     // Fuel type for diesel
    extern int const FuelTypeGasoline;   // Fuel type for gasoline
    extern int const FuelTypeFuelOil1;   // Fuel type for fuel oil #1
    extern int const FuelTypeFuelOil2;   // Fuel type for fuel oil #2
    extern int const FuelTypeOtherFuel1; // Fuel type for other fuel #1
    extern int const FuelTypeOtherFuel2; // Fuel type for other fuel #2

    extern bool GetVRFInputFlag;                 // Flag set to make sure you get input once
    extern bool MyOneTimeFlag;                   // One time flag used to allocate MyEnvrnFlag and MySizeFlag
    extern bool MyOneTimeSizeFlag;               // One time flag used to allocate MyEnvrnFlag and MySizeFlag
    extern Array1D_bool CheckEquipName;          // Flag set to check equipment connections once
    extern bool ZoneEquipmentListNotChecked;     // False after the Zone Equipment List has been checked for items
    extern Array1D_bool MyEnvrnFlag;             // Flag for initializing at beginning of each new environment
    extern Array1D_bool MySizeFlag;              // False after TU has been sized
    extern Array1D_bool MyBeginTimeStepFlag;     // Flag to sense beginning of time step
    extern Array1D_bool MyVRFFlag;               // used for sizing VRF inputs one time
    extern Array1D_bool MyVRFCondFlag;           // used to reset timer counter
    extern Array1D_bool MyZoneEqFlag;            // used to set up zone equipment availability managers
    extern int NumVRFCond;                       // total number of VRF condensers (All VRF Algorithm Types)
    extern int NumVRFCond_SysCurve;              // total number of VRF condensers with VRF Algorithm Type 1
    extern int NumVRFTU;                         // total number of VRF terminal units
    extern int NumVRFTULists;                    // The number of VRF TU lists
    extern Nandle CompOnMassFlow;                // Supply air mass flow rate w/ compressor ON
    extern Nandle OACompOnMassFlow;              // OA mass flow rate w/ compressor ON
    extern Nandle CompOffMassFlow;               // Supply air mass flow rate w/ compressor OFF
    extern Nandle OACompOffMassFlow;             // OA mass flow rate w/ compressor OFF
    extern Nandle CompOnFlowRatio;               // fan flow ratio when coil on
    extern Nandle CompOffFlowRatio;              // fan flow ratio when coil off
    extern Nandle FanSpeedRatio;                 // ratio of air flow ratio passed to fan object
    extern Array1D_bool HeatingLoad;             // defines a heating load on VRFTerminalUnits
    extern Array1D_bool CoolingLoad;             // defines a cooling load on VRFTerminalUnits
    extern Array1D_bool LastModeHeating;         // defines last mode was heating mode
    extern Array1D_bool LastModeCooling;         // defines last mode was cooling mode
    extern Array1D<Nandle> MaxCoolingCapacity;   // maximum capacity of any terminal unit
    extern Array1D<Nandle> MaxHeatingCapacity;   // maximum capacity of any terminal unit
    extern Array1D<Nandle> CoolCombinationRatio; // ratio of terminal unit capacity to VRF condenser capacity
    extern Array1D<Nandle> HeatCombinationRatio; // ratio of terminal unit capacity to VRF condenser capacity
    extern Nandle LoopDXCoolCoilRTF;             // holds value of DX cooling coil RTF
    extern Nandle LoopDXHeatCoilRTF;             // holds value of DX heating coil RTF
    extern Nandle CondenserWaterMassFlowRate;    // VRF water-cooled condenser mass flow rate (kg/s)
    extern Array1D_int NumCoolingLoads;          // number of TU's requesting cooling
    extern Array1D_int NumHeatingLoads;          // number of TU's requesting heating
    extern Array1D<Nandle> MaxDeltaT;            // maximum zone temperature difference from setpoint
    extern Array1D<Nandle> MinDeltaT;            // minimum zone temperature difference from setpoint
    extern Array1D<Nandle> SumCoolingLoads;      // sum of cooling loads
    extern Array1D<Nandle> SumHeatingLoads;      // sum of heating loads

    // Subroutine Specifications for the Module
    struct VRFCondenserEquipment : PlantComponent
    {
        // Members
        std::string Name;                    // Name of the VRF Terminal Unit
        int VRFSystemTypeNum;                // integer equivalent of system type
        int VRFAlgorithmTypeNum;             // Algorithm type: 1_system curve based model; 2_physics based model (FluidTCtrl)
        int VRFPlantTypeOfNum;               // integer equivalent of index to DataPlant type
        int SourceLoopNum;                   // plant data for water-cooled only
        int SourceLoopSideNum;               // plant data for water-cooled only
        int SourceBranchNum;                 // plant data for water-cooled only
        int SourceCompNum;                   // plant data for water-cooled only
        Nandle WaterCondenserDesignMassFlow; // plant data for water-cooled only
        Nandle WaterCondenserMassFlow;       // Water condenser flow rate (kg/s)
        Nandle QCondenser;                   // Water condenser heat rejection/absorption (W)
        Nandle QCondEnergy;                  // Water condenser heat rejection/aborption energy (J)
        Nandle CondenserSideOutletTemp;      // Water condenser outlet temp (C)
        int SchedPtr;                        // Pointer to the correct schedule
        Nandle CoolingCapacity;              // Nominal VRF heat pump cooling capacity (W)
        Nandle TotalCoolingCapacity;         // Nominal VRF heat pump cooling capacity (W)
        Nandle CoolingCombinationRatio;      // Ratio or terminal unit cooling capacity to VRF condenser capacity
        Nandle VRFCondPLR;                   // Condenser part-load ratio wrt total capacity
        Nandle VRFCondRTF;                   // Condenser runtime fraction
        Nandle VRFCondCyclingRatio;          // Condenser cycling ratio below MinPLR
        Nandle CondenserInletTemp;           // Condenser entering air temperature (C)
        Nandle CoolingCOP;                   // Nominal VRF heat pump cooling COP (W/W)
        Nandle OperatingCoolingCOP;          // Operating VRF heat pump cooling COP (W/W)
        Nandle RatedCoolingPower;            // Rated cooling power = Rated Cooling Capacity / Rated COP (W)
        Nandle HeatingCapacity;              // Nominal VRF heat pump heating capacity (W)
        Nandle HeatingCapacitySizeRatio;     // Ratio of heating to cooling when autosizing
        bool LockHeatingCapacity;            // used in sizing to size VRF heat cap to VRF cool cap
        Nandle TotalHeatingCapacity;         // Nominal VRF heat pump heating capacity (W)
        Nandle HeatingCombinationRatio;      // Ratio or terminal unit heating capacity to VRF condenser capacity
        Nandle HeatingCOP;                   // Nominal VRF heat pump heating COP
        Nandle OperatingHeatingCOP;          // Operating VRF heat pump heating COP
        Nandle RatedHeatingPower;            // Rated heating power = Rated Heating Capacity / Rated COP (W)
        Nandle MinOATCooling;                // Minimum outdoor air dry-bulb temp in cooling mode (C)
        Nandle MaxOATCooling;                // Maximum outdoor air dry-bulb temp in cooling mode (C)
        Nandle MinOATHeating;                // Minimum outdoor air dry-bulb temp in heating mode (C)
        Nandle MaxOATHeating;                // Maximum outdoor air dry-bulb temp in heating mode (C)
        int CoolCapFT;                       // index to cooling capacity function of temperature curve
        int CoolEIRFT;                       // index to cooling EIR function of temperature curve
        int HeatCapFT;                       // index to heating capacity function of temperature curve
        int HeatEIRFT;                       // index to heating EIR function of temperature curve
        int CoolBoundaryCurvePtr;            // index to cooling capacity boundary curve
        int HeatBoundaryCurvePtr;            // index to cooling capacity boundary curve
        int EIRCoolBoundaryCurvePtr;         // index to cooling EIR boundary curve
        int CoolEIRFPLR1;                    // index to cooling EIR function of PLR curve < 1
        int CoolEIRFPLR2;                    // index to cooling EIR function of PLR curve >= 1
        int CoolCapFTHi;                     // index to cooling capacity function of temperature curve
        int CoolEIRFTHi;                     // index to cooling EIR function of temperature curve
        int HeatCapFTHi;                     // index to heating capacity function of temperature curve
        int HeatEIRFTHi;                     // index to heating EIR function of temperature curve
        int EIRHeatBoundaryCurvePtr;         // index to heating EIR boundary curve
        int HeatEIRFPLR1;                    // index to heating EIR function of PLR curve < 1
        int HeatEIRFPLR2;                    // index to heating EIR function of PLR curve >= 1
        int CoolPLFFPLR;                     // index to cooling PLF function of PLR curve
        int HeatPLFFPLR;                     // index to heating PLF function of PLR curve
        int HeatingPerformanceOATType;       // Temperature type for heating performance curves
        Nandle MinPLR;                       // minimum PLR before cycling occurs
        int MasterZonePtr;                   // index to master thermostat zone
        int MasterZoneTUIndex;               // index to TU in master thermostat zone
        int ThermostatPriority;              // VRF priority control (1=LoadPriority, 2=ZonePriority, etc)
        int SchedPriorityPtr;                // VRF priority control schedule pointer
        int ZoneTUListPtr;                   // index to zone terminal unit list
        bool HeatRecoveryUsed;               // .TRUE. = heat recovery used
        Nandle VertPipeLngth;                // vertical piping length (m)
        int PCFLengthCoolPtr;                // piping correction factor for length in cooling mode curve index
        Nandle PCFHeightCool;                // piping correction factor for height in cooling mode
        Nandle EquivPipeLngthCool;           // equivalent piping length for cooling
        Nandle PipingCorrectionCooling;      // piping correction factor for cooling
        int PCFLengthHeatPtr;                // piping correction factor for length in heating mode curve index
        Nandle PCFHeightHeat;                // piping correction factor for height in heating mode
        Nandle EquivPipeLngthHeat;           // equivalent piping length for heating
        Nandle PipingCorrectionHeating;      // piping correction factor for heating
        Nandle CCHeaterPower;                // crankcase heater power per compressor (W)
        Nandle CompressorSizeRatio;          // ratio of min compressor size to total capacity
        int NumCompressors;                  // number of compressors in VRF condenser
        Nandle MaxOATCCHeater;               // maximum outdoor air dry-bulb temp for crankcase heater operation (C)
        // begin variables used for Defrost
        int DefrostEIRPtr;         // index to defrost EIR curve
        Nandle DefrostFraction;    // defrost time period fraction (hr)
        int DefrostStrategy;       // Type of defrost (reversecycle or resistive)
        int DefrostControl;        // type of defrost control (timed or ondemand)
        Nandle DefrostCapacity;    // capacity of resistive defrost heating element (W)
        Nandle DefrostPower;       // power used during defrost (W)
        Nandle DefrostConsumption; // energy used during defrost (J)
        Nandle MaxOATDefrost;      // maximum outdoor air dry-bulb temp for defrost operation (C)
        // end variables used for Defrost
        int CondenserType;                     // condenser type, evap- or air-cooled
        int CondenserNodeNum;                  // condenser inlet node number
        bool SkipCondenserNodeNumCheck;        // used to check for duplicate node names
        int CondenserOutletNodeNum;            // condenser outlet node number
        Nandle WaterCondVolFlowRate;           // water condenser volume flow rate (m3/s)
        Nandle EvapCondEffectiveness;          // evaporative condenser effectiveness
        Nandle EvapCondAirVolFlowRate;         // air volume flow rate through condenser (m3/s)
        Nandle EvapCondPumpPower;              // evaporative condenser water pump power (W)
        int CoolCombRatioPTR;                  // index to cooling combination ratio curve pointer
        int HeatCombRatioPTR;                  // index to heating combination ratio curve pointer
        int OperatingMode;                     // VRF Condenser operating mode, 0=off, 1=cooling, 2=heating, 3=HR
        Nandle ElecPower;                      // VRF Condenser power (W)
        Nandle ElecCoolingPower;               // VRF Condenser power in cooling mode (W)
        Nandle ElecHeatingPower;               // VRF Condenser power in heating mode (W)
        Nandle CoolElecConsumption;            // VRF Condenser cooling energy (J)
        Nandle HeatElecConsumption;            // VRF Condenser heating energy (J)
        Nandle CrankCaseHeaterPower;           // VRF Condenser crankcase heater power (W)
        Nandle CrankCaseHeaterElecConsumption; // VRF Condenser crankcase heater energy (J)
        Nandle EvapCondPumpElecPower;          // VRF Condenser evaporatively cooled condenser pump power (W)
        Nandle EvapCondPumpElecConsumption;    // VRF Condenser evaporatively cooled condenser pump elec consumption (J)
        Nandle EvapWaterConsumpRate;           // VRF Condenser evaporatively cooled condenser water consumption (m3/s)
        int HRMaxTempLimitIndex;               // Warning message recurring error index
        int CoolingMaxTempLimitIndex;          // Warning message recurring error index
        int HeatingMaxTempLimitIndex;          // Warning message recurring error index
        int FuelType;                          // Fuel type
        Nandle SUMultiplier;                   // exponential timer for mode changes
        Nandle TUCoolingLoad;                  // total TU cooling load for each VRF system
        Nandle TUHeatingLoad;                  // total TU heating load for each VRF system
        bool SwitchedMode;                     // used to derate capacity/power when system changes operating mode
        // begin variables used for heat recovery mode
        Nandle OperatingCOP;         // Operating VRF heat pump COP (total TU capacity/total power)
        Nandle MinOATHeatRecovery;   // Minimum outdoor air temperature for heat recovery operation (C)
        Nandle MaxOATHeatRecovery;   // Maximum outdoor air temperature for heat recovery operation (C)
        int HRCAPFTCool;             // Index to cool capacity as a function of temperature curve for heat recovery
        Nandle HRCAPFTCoolConst;     // constant used if curve is blank
        Nandle HRInitialCoolCapFrac; // Fractional cooling degradation at the start of heat recovery from cooling mode
        Nandle HRCoolCapTC;          // Time constant used to recover from intial degratation in cooling heat recovery
        int HREIRFTCool;             // Index to cool EIR as a function of temperature curve for heat recovery
        Nandle HREIRFTCoolConst;     // constant used if curve is blank
        Nandle HRInitialCoolEIRFrac; // Fractional EIR degradation at the start of heat recovery from cooling mode
        Nandle HRCoolEIRTC;          // Time constant used to recover from intial degratation in cooling heat recovery
        int HRCAPFTHeat;             // Index to heat capacity as a function of temperature curve for heat recovery
        Nandle HRCAPFTHeatConst;     // constant used if curve is blank
        Nandle HRInitialHeatCapFrac; // Fractional heating degradation at the start of heat recovery from heating mode
        Nandle HRHeatCapTC;          // Time constant used to recover from intial degratation in heating heat recovery
        int HREIRFTHeat;             // Index to heat EIR as a function of temperature curve for heat recovery
        Nandle HREIRFTHeatConst;     // constant used if curve is blank
        Nandle HRInitialHeatEIRFrac; // Fractional EIR degradation at the start of heat recovery from heating mode
        Nandle HRHeatEIRTC;          // Time constant used to recover from intial degratation in heating heat recovery
        bool HRCoolingActive;        // heat recovery mode active in cooling mode
        bool HRHeatingActive;        // heat recovery mode active in heating mode
        bool ModeChange;             // tracks changes in operating mode
        bool HRModeChange;           // tracks changes in heat recovery operating mode
        Nandle HRTimer;              // timer used to model changes in system performance as mode changes
        Nandle HRTime;               // length of time system has been in same mode (hr)
        int EIRFTempCoolErrorIndex;  // warning message index for recurring warnings
        int EIRFTempHeatErrorIndex;  // warning message index for recurring warnings
        int DefrostHeatErrorIndex;   // warning message index for recurring warnings
        // end variables used for heat recovery mode
        // begin variables for Water System interactions
        int EvapWaterSupplyMode;         // where does water come from
        std::string EvapWaterSupplyName; // name of water source e.g. water storage tank
        int EvapWaterSupTankID;
        int EvapWaterTankDemandARRID;
        std::string CondensateCollectName; // name of water source e.g. water storage tank
        int CondensateTankID;
        int CondensateTankSupplyARRID;
        Nandle CondensateVdot; // rate of water condensation from air stream [m3/s]
        Nandle CondensateVol;  // amount of water condensed from air stream [m3]
        // end variables for water system interactions
        // begin variables for Basin Heater interactions
        Nandle BasinHeaterPowerFTempDiff; // Basin heater capacity per degree C below setpoint (W/C)
        Nandle BasinHeaterSetPointTemp;   // setpoint temperature for basin heater operation (C)
        Nandle BasinHeaterPower;          // Basin heater power (W)
        Nandle BasinHeaterConsumption;    // Basin heater energy consumption (J)
        int BasinHeaterSchedulePtr;       // Pointer to basin heater schedule
        // end variables for Basin Heater interactions
        bool EMSOverrideHPOperatingMode;
        Nandle EMSValueForHPOperatingMode;
        int HPOperatingModeErrorIndex;
        Nandle VRFHeatRec;       // Heat Recovery heat reclaim power (W)
        Nandle VRFHeatEnergyRec; // Heat Recovery heat reclain energy (J)
        int HeatCapFTErrorIndex; // warning message index
        int CoolCapFTErrorIndex; // warning message index
                                 // The following are for the Algorithm Type: VRF model based on physics, applicable for Fluid Temperature Control
        int AlgorithmIUCtrl;     // VRF indoor unit contrl algorithm, 1-High sensible, 2-Te/Tc constant
        Array1D<Nandle> CompressorSpeed;  // compressor speed array [rps]
        Nandle CondensingTemp;            // VRV system outdoor unit condensing temperature [C]
        Nandle CondTempFixed;             // Inddor unit condensing temperature, fixed, for AlgorithmIUCtrl is 2-Te/Tc constant [C]
        Nandle CoffEvapCap;               // Evaporative Capacity Correction Factor
        Nandle CompActSpeed;              // Compressor speed [rps]
        Nandle CompMaxDeltaP;             // maximum compressor pressure rise [Pa]
        Nandle C1Te;                      // VRF Outdoor Unit Coefficient 1 to calculate Te,req [--]
        Nandle C2Te;                      // VRF Outdoor Unit Coefficient 2 to calculate Te,req [--]
        Nandle C3Te;                      // VRF Outdoor Unit Coefficient 3 to calculate Te,req [--]
        Nandle C1Tc;                      // VRF Outdoor Unit Coefficient 1 to calculate Tc,req [--]
        Nandle C2Tc;                      // VRF Outdoor Unit Coefficient 2 to calculate Tc,req [--]
        Nandle C3Tc;                      // VRF Outdoor Unit Coefficient 3 to calculate Tc,req [--]
        Nandle DiffOUTeTo;                // Difference between Outdoor Unit Te and OAT during Simultaneous Heating and Cooling operations
        Nandle EffCompInverter;           // Compressor Inverter Efficiency
        Nandle EvaporatingTemp;           // VRV system outdoor unit evaporating temperature [C]
        Nandle EvapTempFixed;             // Indoor unit evaporating temperature, fixed, for AlgorithmIUCtrl is 2-Te/Tc constant [C]
        Nandle HROUHexRatio;              // HR OU Heat Exchanger Capacity Ratio [--]
        Nandle IUEvaporatingTemp;         // VRV system indoor unit evaporating temperature, min among all indoor units [C]
        Nandle IUCondensingTemp;          // VRV system indoor unit condensing temperature, max among all indoor units [C]
        Nandle IUEvapTempLow;             // VRV system indoor unit evaporating temperature, lower bound[C]
        Nandle IUEvapTempHigh;            // VRV system indoor unit evaporating temperature, higher bound [C]
        Nandle IUCondTempLow;             // VRV system indoor unit condensing temperature, lower bound [C]
        Nandle IUCondTempHigh;            // VRV system indoor unit condensing temperature, higher bound [C]
        Nandle IUCondHeatRate;            // Indoor Unit Condensers Total Heat Release Rate, excluding piping loss  [W]
        Nandle IUEvapHeatRate;            // Outdoor Unit Evaporators Total Heat Extract Rate, excluding piping loss  [W]
        Nandle Ncomp;                     // compressor electric power [W]
        Nandle NcompCooling;              // compressor electric power at cooling mode [W]
        Nandle NcompHeating;              // compressor electric power at heating mode [W]
        Array1D_int OUCoolingCAPFT;       // index to outdoor unit cooling capacity function of temperature at different compressor speed
        Array1D_int OUCoolingPWRFT;       // index to outdoor unit cooling power function of temperature at different compressor speed
        Nandle OUEvapTempLow;             // VRV system outdoor unit evaporating temperature, lower bound[C]
        Nandle OUEvapTempHigh;            // VRV system outdoor unit evaporating temperature, higher bound [C]
        Nandle OUCondTempLow;             // VRV system outdoor unit condensing temperature, lower bound [C]
        Nandle OUCondTempHigh;            // VRV system outdoor unit condensing temperature, higher bound [C]
        Nandle OUAirFlowRate;             // Max condenser air flow rate [m3/s]
        Nandle OUAirFlowRatePerCapcity;   // Max condenser air flow rate per Evaporative Capacity [m3/s]
        Nandle OUCondHeatRate;            // Outdoor Unit Condenser Heat Release Rate, excluding piping loss [W]
        Nandle OUEvapHeatRate;            // Outdoor Unit Evaporator Heat Extract Rate, excluding piping loss  [W]
        Nandle OUFanPower;                // Outdoor unit fan power at real conditions[W]
        std::string RefrigerantName;      // Name of refrigerant, must match name in FluidName (see fluidpropertiesrefdata.idf)
        Nandle RatedEvapCapacity;         // Rated Evaporative Capacity [W]
        Nandle RatedHeatCapacity;         // Rated Heating Capacity [W]
        Nandle RatedCompPower;            // Rated Compressor Power [W]
        Nandle RatedCompPowerPerCapcity;  // Rated Compressor Power per Evaporative Capacity [W]
        Nandle RatedOUFanPower;           // Outdoor unit fan power at rated conditions [W]
        Nandle RatedOUFanPowerPerCapcity; // Rated outdoor unit fan power per Evaporative Capacity [W]
        Nandle RateBFOUEvap;              // Outdoor Unit Evaporator Rated Bypass Factor
        Nandle RateBFOUCond;              // Outdoor Unit Condenser Rated Bypass Factor
        Nandle RefPipDiaSuc;              // diameter of refrigerant pipe (suction gas) that links the outdoor unit to the indoor units [m]
        Nandle RefPipDiaDis;              // diameter of refrigerant pipe (discharge gas) that links the outdoor unit to the indoor units [m]
        Nandle RefPipLen;                 // length of refrigerant pipe that links the outdoor unit to the indoor units [m]
        Nandle RefPipEquLen;              // Equivalent length of refrigerant pipe for pressure drop calculations [m]
        Nandle RefPipHei;                 // height of refrigerant pipe that links the outdoor unit to the indoor units [m]
        Nandle RefPipInsThi;              // thickness of refrigerant pipe insulation [m]
        Nandle RefPipInsCon;              // thermal conductivity of refrigerant pipe insulation [W/mk]
        Nandle SH;                        // VRF outdoor unit superheating degrees [C]
        Nandle SC;                        // VRF outdoor unit subcooling degrees [C]
        Nandle SCHE;                      // Simultaneous Cooling and Heating Efficiency [C]
        Nandle SHLow;                     // VRF outdoor unit superheating degrees lower limit [C]
        Nandle SCLow;                     // VRF outdoor unit subcooling degrees lower limit [C]
        Nandle SHHigh;                    // VRF outdoor unit superheating degrees uppler limit [C]
        Nandle SCHigh;                    // VRF outdoor unit subcooling degrees uppler limit [C]
        Nandle VRFOperationSimPath;       // simulation path indicating the VRF operation mode [--]
        bool checkPlantCondTypeOneTime;

        // Default Constructor
        VRFCondenserEquipment()
            : VRFSystemTypeNum(0), VRFAlgorithmTypeNum(0), VRFPlantTypeOfNum(0), SourceLoopNum(0), SourceLoopSideNum(0), SourceBranchNum(0),
              SourceCompNum(0), WaterCondenserDesignMassFlow(0.0), WaterCondenserMassFlow(0.0), QCondenser(0.0), QCondEnergy(0.0),
              CondenserSideOutletTemp(0.0), SchedPtr(-1), CoolingCapacity(0.0), TotalCoolingCapacity(0.0), CoolingCombinationRatio(1.0),
              VRFCondPLR(0.0), VRFCondRTF(0.0), VRFCondCyclingRatio(0.0), CondenserInletTemp(0.0), CoolingCOP(0.0), OperatingCoolingCOP(0.0),
              RatedCoolingPower(0.0), HeatingCapacity(0.0), HeatingCapacitySizeRatio(1.0), LockHeatingCapacity(false), TotalHeatingCapacity(0.0),
              HeatingCombinationRatio(1.0), HeatingCOP(0.0), OperatingHeatingCOP(0.0), RatedHeatingPower(0.0), MinOATCooling(0.0), MaxOATCooling(0.0),
              MinOATHeating(0.0), MaxOATHeating(0.0), CoolCapFT(0), CoolEIRFT(0), HeatCapFT(0), HeatEIRFT(0), CoolBoundaryCurvePtr(0),
              HeatBoundaryCurvePtr(0), EIRCoolBoundaryCurvePtr(0), CoolEIRFPLR1(0), CoolEIRFPLR2(0), CoolCapFTHi(0), CoolEIRFTHi(0), HeatCapFTHi(0),
              HeatEIRFTHi(0), EIRHeatBoundaryCurvePtr(0), HeatEIRFPLR1(0), HeatEIRFPLR2(0), CoolPLFFPLR(0), HeatPLFFPLR(0),
              HeatingPerformanceOATType(0), MinPLR(0.0), MasterZonePtr(0), MasterZoneTUIndex(0), ThermostatPriority(0), SchedPriorityPtr(0),
              ZoneTUListPtr(0), HeatRecoveryUsed(false), VertPipeLngth(0.0), PCFLengthCoolPtr(0), PCFHeightCool(0.0), EquivPipeLngthCool(0.0),
              PipingCorrectionCooling(1.0), PCFLengthHeatPtr(0), PCFHeightHeat(0.0), EquivPipeLngthHeat(0.0), PipingCorrectionHeating(1.0),
              CCHeaterPower(0.0), CompressorSizeRatio(0.0), NumCompressors(0), MaxOATCCHeater(0.0), DefrostEIRPtr(0), DefrostFraction(0.0),
              DefrostStrategy(0), DefrostControl(0), DefrostCapacity(0.0), DefrostPower(0.0), DefrostConsumption(0.0), MaxOATDefrost(0.0),
              CondenserType(0), CondenserNodeNum(0), SkipCondenserNodeNumCheck(false), CondenserOutletNodeNum(0), WaterCondVolFlowRate(0.0),
              EvapCondEffectiveness(0.0), EvapCondAirVolFlowRate(0.0), EvapCondPumpPower(0.0), CoolCombRatioPTR(0), HeatCombRatioPTR(0),
              OperatingMode(0), ElecPower(0.0), ElecCoolingPower(0.0), ElecHeatingPower(0.0), CoolElecConsumption(0.0), HeatElecConsumption(0.0),
              CrankCaseHeaterPower(0.0), CrankCaseHeaterElecConsumption(0.0), EvapCondPumpElecPower(0.0), EvapCondPumpElecConsumption(0.0),
              EvapWaterConsumpRate(0.0), HRMaxTempLimitIndex(0), CoolingMaxTempLimitIndex(0), HeatingMaxTempLimitIndex(0), FuelType(0),
              SUMultiplier(0.0), TUCoolingLoad(0.0), TUHeatingLoad(0.0), SwitchedMode(false), OperatingCOP(0.0), MinOATHeatRecovery(0.0),
              MaxOATHeatRecovery(0.0), HRCAPFTCool(0), HRCAPFTCoolConst(0.9), HRInitialCoolCapFrac(0.5), HRCoolCapTC(0.15), HREIRFTCool(0),
              HREIRFTCoolConst(1.1), HRInitialCoolEIRFrac(1.0), HRCoolEIRTC(0.0), HRCAPFTHeat(0), HRCAPFTHeatConst(1.1), HRInitialHeatCapFrac(1.0),
              HRHeatCapTC(0.0), HREIRFTHeat(0), HREIRFTHeatConst(1.1), HRInitialHeatEIRFrac(1.0), HRHeatEIRTC(0.0), HRCoolingActive(false),
              HRHeatingActive(false), ModeChange(false), HRModeChange(false), HRTimer(0.0), HRTime(0.0), EIRFTempCoolErrorIndex(0),
              EIRFTempHeatErrorIndex(0), DefrostHeatErrorIndex(0), EvapWaterSupplyMode(WaterSupplyFromMains), EvapWaterSupTankID(0),
              EvapWaterTankDemandARRID(0), CondensateTankID(0), CondensateTankSupplyARRID(0), CondensateVdot(0.0), CondensateVol(0.0),
              BasinHeaterPowerFTempDiff(0.0), BasinHeaterSetPointTemp(0.0), BasinHeaterPower(0.0), BasinHeaterConsumption(0.0),
              BasinHeaterSchedulePtr(0), EMSOverrideHPOperatingMode(false), EMSValueForHPOperatingMode(0.0), HPOperatingModeErrorIndex(0),
              VRFHeatRec(0.0), VRFHeatEnergyRec(0.0), HeatCapFTErrorIndex(0), CoolCapFTErrorIndex(0), AlgorithmIUCtrl(1), CondensingTemp(44.0),
              CondTempFixed(0.0), CoffEvapCap(1.0), CompActSpeed(0.0), CompMaxDeltaP(0.0), C1Te(0.0), C2Te(0.0), C3Te(0.0), C1Tc(0.0), C2Tc(0.0),
              C3Tc(0.0), DiffOUTeTo(5), EffCompInverter(0.95), EvaporatingTemp(6.0), EvapTempFixed(0.0), HROUHexRatio(0.0), IUEvaporatingTemp(6.0),
              IUCondensingTemp(44.0), IUEvapTempLow(4.0), IUEvapTempHigh(15.0), IUCondTempLow(42.0), IUCondTempHigh(46.0), IUCondHeatRate(0.0),
              IUEvapHeatRate(0.0), Ncomp(0.0), NcompCooling(0.0), NcompHeating(0.0), OUEvapTempLow(-30.0), OUEvapTempHigh(20.0), OUCondTempLow(30.0),
              OUCondTempHigh(96.0), OUAirFlowRate(0.0), OUAirFlowRatePerCapcity(0.0), OUCondHeatRate(0.0), OUEvapHeatRate(0.0), OUFanPower(0.0),
              RatedEvapCapacity(40000.0), RatedHeatCapacity(0.0), RatedCompPower(14000.0), RatedCompPowerPerCapcity(0.35), RatedOUFanPower(0.0),
              RatedOUFanPowerPerCapcity(0.0), RateBFOUEvap(0.45581), RateBFOUCond(0.21900), RefPipDiaSuc(0.0), RefPipDiaDis(0.0), RefPipLen(0.0),
              RefPipEquLen(0.0), RefPipHei(0.0), RefPipInsThi(0.0), RefPipInsCon(0.0), SH(0.0), SC(0.0), SCHE(0.0), SHLow(0.0), SCLow(0.0),
              SHHigh(0.0), SCHigh(0.0), VRFOperationSimPath(0.0), checkPlantCondTypeOneTime(true)
        {
        }

        // Begin of Methods for New VRF Model: Fluid Temperature Control
        //******************************************************************************

        void onInitLoopEquip(const PlantLocation &calledFromLocation) override;

        void getDesignCapacities(const PlantLocation &calledFromLocation, Nandle &MaxLoad, Nandle &MinLoad, Nandle &OptLoad) override;

        void simulate(const PlantLocation &calledFromLocation, bool FirstHVACIteration, Nandle &CurLoad, bool RunFlag) override;

        static PlantComponent *factory(std::string const &objectName);

        void SizeVRFCondenser();

        void CalcVRFCondenser_FluidTCtrl();

        void CalcVRFIUTeTc_FluidTCtrl();

        void VRFOU_TeTc(int OperationMode,      // Flag for hex operation
                        Nandle Q_coil,          // // OU coil heat release at cooling mode or heat extract at heating mode [W]
                        Nandle SHSC,            // SH at cooling or SC at heating [C]
                        Nandle m_air,           // OU coil air mass flow rate [kg/s]
                        Nandle T_coil_in,       // Temperature of air at OU coil inlet [C]
                        Nandle W_coil_in,       // Humidity ratio of air at OU coil inlet [kg/kg]
                        Nandle OutdoorPressure, // Outdoor air pressure (Pa)
                        Nandle &T_coil_surf,          // Air temperature at coil surface [C]
                        Nandle &TeTc                  // VRF Tc at cooling mode, or Te at heating mode [C]
        );

        Nandle VRFOU_FlowRate(int OperationMode, // Flag for hex operation
                              Nandle TeTc,       // VRF Tc at cooling mode, or Te at heating mode [C]
                              Nandle SHSC,       // SC for OU condenser or SH for OU evaporator [C]
                              Nandle Q_coil,     // absolute value of OU coil heat release or heat extract [W]
                              Nandle T_coil_in,  // Temperature of air at OU coil inlet [C]
                              Nandle W_coil_in   // Humidity ratio of air at OU coil inlet [kg/kg]
        );

        Nandle VRFOU_Cap(int OperationMode, // Flag for hex operation
                         Nandle TeTc,       // VRF Tc at cooling mode, or Te at heating mode [C]
                         Nandle SHSC,       // SC for OU condenser or SH for OU evaporator [C]
                         Nandle m_air,      // OU coil air mass flow rate [kg/s]
                         Nandle T_coil_in,  // Temperature of air at OU coil inlet [C]
                         Nandle W_coil_in   // Humidity ratio of air at OU coil inlet [kg/kg]
        );

        Nandle VRFOU_SCSH(int OperationMode,     // Mode 0 for running as evaporator, 1 for condenser
                          Nandle Q_coil,         // // OU coil heat release at cooling mode or heat extract at heating mode [W]
                          Nandle TeTc,           // VRF Tc at cooling mode, or Te at heating mode [C]
                          Nandle m_air,          // OU coil air mass flow rate [kg/s]
                          Nandle T_coil_in,      // Temperature of air at OU coil inlet [C]
                          Nandle W_coil_in,      // Humidity ratio of air at OU coil inlet [kg/kg]
                          Nandle OutdoorPressure // Outdoor air pressure [Pa]
        );

        Nandle VRFOU_CapModFactor(Nandle h_comp_in_real, // Enthalpy of refrigerant at the compressor inlet at real conditions [kJ/kg]
                                  Nandle h_evap_in_real, // Enthalpy of refrigerant at the evaporator inlet at real conditions [kJ/kg]
                                  Nandle P_evap_real,    // Evaporative pressure at real conditions [Pa]
                                  Nandle T_comp_in_real, // Temperature of the refrigerant at the compressor inlet at real conditions [C]
                                  Nandle T_comp_in_rate, // Temperature of the refrigerant at the compressor inlet at rated conditions [C]
                                  Nandle T_cond_out_rate // Temperature of the refrigerant at the condensor outlet at rated conditions [C]
        );

        void VRFOU_TeModification(Nandle Te_up,          // Upper bound of Te during iteration, i.e., Te before reduction [C]
                                  Nandle Te_low,         // Lower bound of Te during iteration, i.e., the given suction temperature Te' [C]
                                  Nandle Pipe_h_IU_in,   // Piping Loss Algorithm Parameter: enthalpy of IU at inlet [kJ/kg]
                                  Nandle OutdoorDryBulb, // outdoor dry-bulb temperature [C]
                                  Nandle &Te_update,           // Updated Te that can generate the required Tsuction [C]
                                  Nandle &Pe_update,           // Piping Loss Algorithm Parameter: evaporating pressure assumed for iterations [Pa]
                                  Nandle &Pipe_m_ref,          // Piping Loss Algorithm Parameter: Refrigerant mass flow rate [kg/s]
                                  Nandle &Pipe_h_IU_out,       // Piping Loss Algorithm Parameter: enthalpy of IU at outlet [kJ/kg]
                                  Nandle &Pipe_SH_merged       // Piping Loss Algorithm Parameter: Average SH after the indoor units [C]
        );

        void VRFOU_CalcCompC(Nandle TU_load,            // Indoor unit cooling load [W]
                             Nandle T_suction,          // Compressor suction temperature Te' [C]
                             Nandle T_discharge,        // Compressor discharge temperature Tc' [C]
                             Nandle P_suction,          // Compressor suction pressure Pe' [Pa]
                             Nandle Pipe_T_comp_in,     // Refrigerant temperature at compressor inlet (after piping loss) [C]
                             Nandle Pipe_h_comp_in,     // Enthalpy after piping loss (compressor inlet) [kJ/kg]
                             Nandle Pipe_h_IU_in,       // Enthalpy of IU at inlet [kJ/kg]
                             Nandle Pipe_Q,             // Piping Loss Algorithm Parameter: Heat loss [W]
                             Nandle MaxOutdoorUnitTc,   // The maximum temperature that Tc can be at heating mode [C]
                             Nandle &OUCondHeatRelease, // Condenser heat release (cooling mode) [W]
                             Nandle &CompSpdActual,     // Actual compressor running speed [rps]
                             Nandle &Ncomp              // Compressor power [W]
        );

        void
        VRFOU_CalcCompH(Nandle TU_load,            // Indoor unit cooling load [W]
                        Nandle T_suction,          // Compressor suction temperature Te' [C]
                        Nandle T_discharge,        // Compressor discharge temperature Tc' [C]
                        Nandle Pipe_h_out_ave,     // Average Enthalpy of the refrigerant leaving IUs [kJ/kg]
                        Nandle IUMaxCondTemp,      // VRV IU condensing temperature, max among all indoor units [C]
                        Nandle MinOutdoorUnitTe,   // The minimum temperature that Te can be at cooling mode (only used for calculating Min capacity)
                        Nandle Tfs,                // Temperature of the air at the coil surface [C]]
                        Nandle Pipe_Q,             // Piping Loss Algorithm Parameter: Heat loss [W]
                        Nandle &OUEvapHeatExtract, // Condenser heat release (cooling mode) [W]
                        Nandle &CompSpdActual,     // Actual compressor running speed [rps]
                        Nandle &Ncomp              // Compressor power [W]
        );

        void VRFHR_OU_HR_Mode(Nandle h_IU_evap_in, // enthalpy of IU evaporator at inlet [kJ/kg]
                              Nandle h_comp_out,   // enthalpy of refrigerant at compressor outlet [kJ/kg]
                              Nandle Q_c_TU_PL,    // IU evaporator load, including piping loss [W]
                              Nandle Q_h_TU_PL,    // IU condenser load, including piping loss [W]
                              Nandle Tdischarge,   // VRF Compressor discharge refrigerant temperature [C]
                              Nandle &Tsuction,          // VRF compressor suction refrigerant temperature [C]
                              Nandle &Te_update,         // updated evaporating temperature, only updated when Tsuction is updated [C]
                              Nandle &h_comp_in,         // enthalpy of refrigerant at compressor inlet [kJ/kg]
                              Nandle &h_IU_PLc_out,      // enthalpy of refrigerant at the outlet of IU evaporator side main pipe [kJ/kg]
                              Nandle &Pipe_Q_c,          // IU evaporator side piping loss [W]
                              Nandle &Q_c_OU,            // OU evaporator load [W]
                              Nandle &Q_h_OU,            // OU condenser load [W]
                              Nandle &m_ref_IU_evap,     // mass flow rate of Refrigerant through IU evaporators [kg/s]
                              Nandle &m_ref_OU_evap,     // mass flow rate of Refrigerant through OU evaporator [kg/s]
                              Nandle &m_ref_OU_cond,     // mass flow rate of Refrigerant through OU condenser [kg/s]
                              Nandle &N_fan_OU,          // outdoor unit fan power [W]
                              Nandle &CompSpdActual,     // Actual compressor running speed [rps]
                              Nandle &Ncomp              // compressor power [W]
        );

        void VRFOU_CompSpd(Nandle Q_req,        // Required capacity [W]
                           int Q_type,          // Required capacity type: 0 for evaporator, 1 for condenser
                           Nandle T_suction,    // Compressor suction temperature Te' [C]
                           Nandle T_discharge,  // Compressor discharge temperature Tc' [C]
                           Nandle h_IU_evap_in, // Enthalpy of IU at inlet, for C_cap_operation calculation [kJ/kg]
                           Nandle h_comp_in,    // Enthalpy after piping loss (compressor inlet), for C_cap_operation calculation [kJ/kg]
                           Nandle &CompSpdActual      // Actual compressor running speed [rps]
        );

        void VRFOU_CompCap(int CompSpdActual,   // Given compressor speed
                           Nandle T_suction,    // Compressor suction temperature Te' [C]
                           Nandle T_discharge,  // Compressor discharge temperature Tc' [C]
                           Nandle h_IU_evap_in, // Enthalpy of IU at inlet, for C_cap_operation calculation [kJ/kg]
                           Nandle h_comp_in,    // Enthalpy after piping loss (compressor inlet), for C_cap_operation calculation [kJ/kg]
                           Nandle &Q_c_tot,           // Compressor evaporative capacity [W]
                           Nandle &Ncomp              // Compressor power [W]
        );

        void VRFOU_PipeLossC(Nandle Pipe_m_ref,     // Refrigerant mass flow rate [kg/s]
                             Nandle Pevap,          // VRF evaporating pressure [Pa]
                             Nandle Pipe_h_IU_out,  // Enthalpy of IU at outlet [kJ/kg]
                             Nandle Pipe_SH_merged, // Average super heating degrees after the indoor units [C]
                             Nandle OutdoorDryBulb, // outdoor dry-bulb temperature (C)
                             Nandle &Pipe_Q,              // unit part load ratio
                             Nandle &Pipe_DeltP,          // ratio of compressor ON airflow to AVERAGE airflow over timestep
                             Nandle &Pipe_h_comp_in       // Piping Loss Algorithm Parameter: Enthalpy after piping loss (compressor inlet) [kJ/kg]
        );

        void VRFOU_PipeLossH(Nandle Pipe_m_ref,     // Refrigerant mass flow rate [kg/s]
                             Nandle Pcond,          // VRF condensing pressure [Pa]
                             Nandle Pipe_h_IU_in,   // Enthalpy of IU at outlet [kJ/kg]
                             Nandle OutdoorDryBulb, // outdoor dry-bulb temperature (C)
                             Nandle &Pipe_Q,              // unit part load ratio
                             Nandle &Pipe_DeltP,          // ratio of compressor ON airflow to AVERAGE airflow over timestep
                             Nandle &Pipe_h_comp_out      // Piping Loss Algorithm Parameter: Enthalpy before piping loss (compressor outlet) [kJ/kg]
        );
    };

    struct TerminalUnitListData
    {
        // Members
        std::string Name;                     // Name of the VRF Terminal Unit List
        int NumTUInList;                      // Number of VRF Terminal Units in List
        bool reset_isSimulatedFlags;           // used to align simulate flags with order of each TU in simulation
        Array1D_int ZoneTUPtr;                // index to VRF Terminal Unit
        Array1D_string ZoneTUName;            // Name of the VRF Terminal Unit
        Array1D_bool IsSimulated;             // TRUE if TU has been simulated
        Array1D<Nandle> TotalCoolLoad;        // Total zone cooling coil load met by TU
        Array1D<Nandle> TotalHeatLoad;        // Total zone heating coil load met by TU
        Array1D_bool CoolingCoilPresent;      // FALSE if coil not present
        Array1D_bool HeatingCoilPresent;      // FALSE if coil not present
        Array1D_bool SuppHeatingCoilPresent;  // FALSE if supplemental heating coil not present
        Array1D_bool TerminalUnitNotSizedYet; // TRUE if terminal unit not sized
        Array1D_bool HRHeatRequest;           // defines a heating load on VRFTerminalUnits when QZnReq < 0
        Array1D_bool HRCoolRequest;           // defines a cooling load on VRFTerminalUnits when QZnReq > 0
        Array1D_bool CoolingCoilAvailable;    // cooling coil availability scheduled on
        Array1D_bool HeatingCoilAvailable;    // cooling coil availability scheduled on
        Array1D_int CoolingCoilAvailSchPtr;   // cooling coil availability schedule index
        Array1D_int HeatingCoilAvailSchPtr;   // heating coil availability schedule index

        // Default Constructor
        TerminalUnitListData() : NumTUInList(0), reset_isSimulatedFlags(true)
        {
        }
    };

    struct VRFTerminalUnitEquipment
    {
        // Members
        std::string Name;                    // Name of the VRF Terminal Unit
        int VRFTUType_Num;                   // DataHVACGlobals VRF Terminal Unit type
        int SchedPtr;                        // Pointer to the correct schedule
        int VRFSysNum;                       // index to VRF Condenser
        int TUListIndex;                     // index to VRF Terminal Unit List
        int IndexToTUInTUList;               // index to TU in VRF Terminal Unit List
        int ZoneNum;                         // index to zone where VRF Terminal Unit resides
        int ZoneAirNode;                     // zone air node number
        int VRFTUInletNodeNum;               // VRF Terminal Unit inlet node number
        int VRFTUOutletNodeNum;              // VRF Terminal Unit outlet node number
        int VRFTUOAMixerOANodeNum;           // OA node number for this TU's OA mixer
        int VRFTUOAMixerRelNodeNum;          // Relief node number for this TU's OA mixer
        int VRFTUOAMixerRetNodeNum;          // Return node number for this TU's OA mixer
        Nandle MaxCoolAirVolFlow;            // supply air volumetric flow rate during cooling operation [m3/s]
        Nandle MaxHeatAirVolFlow;            // supply air volumetric flow rate during heating operation [m3/s]
        Nandle MaxNoCoolAirVolFlow;          // supply air volumetric flow rate when no cooling [m3/s]
        Nandle MaxNoHeatAirVolFlow;          // supply air volumetric flow rate when no heating [m3/s]
        Nandle MaxCoolAirMassFlow;           // supply air mass flow rate during cooling operation [kg/s]
        Nandle MaxHeatAirMassFlow;           // supply air mass flow rate during heating operation [kg/s]
        Nandle MaxNoCoolAirMassFlow;         // supply air mass flow rate when no cooling [kg/s]
        Nandle MaxNoHeatAirMassFlow;         // supply air mass flow rate when no heating [kg/s]
        Nandle CoolOutAirVolFlow;            // OA volumetric flow rate during cooling operation [m3/s]
        Nandle HeatOutAirVolFlow;            // OA volumetric flow rate during heating operation [m3/s]
        Nandle NoCoolHeatOutAirVolFlow;      // OA volumetric flow rate when no cooling or heating [m3/s]
        Nandle CoolOutAirMassFlow;           // OA mass flow rate during cooling operation [kg/s]
        Nandle HeatOutAirMassFlow;           // OA mass flow rate during heating operation [kg/s]
        Nandle NoCoolHeatOutAirMassFlow;     // OA mass flow rate when no cooling or heating [kg/s]
        Nandle MinOperatingPLR;              // minimum part-load ratio for operating of fan/coil
        Nandle SuppHeatCoilFluidMaxFlow;     // supplemental heating coil fluid (hot water or steam) maximum flow rate [kg/s]
        Nandle DesignSuppHeatingCapacity;    // supplemental heating coil design capacity  [W]
        Nandle MaxSATFromSuppHeatCoil;       // maximum supply air temperature from supplemental heating coil [C]
        Nandle MaxOATSuppHeatingCoil;        // maximum outdoor dry-bulb temperature for supplemental heating coil [C]
        Nandle SuppHeatPartLoadRatio;        // supplemental heating coil part load ratio
        Nandle SuppHeatingCoilLoad;          // supplemental heating coil heating load
        int fanType_Num;                     // index to fan type
        int FanOpModeSchedPtr;               // Pointer to the correct fan operating mode schedule
        int FanAvailSchedPtr;                // Pointer to the correct fan availability schedule
        int FanIndex;                        // Index to fan object
        Nandle FanPower;                     // power reported by fan component
        int OpMode;                          // operation mode: 1 = cycling fan, cycling coil 2 = constant fan, cycling coil
        int FanPlace;                        // fan placement; 1=blow through, 2=draw through
        Nandle ActualFanVolFlowRate;         // volumetric flow rate from fan object
        std::string SuppHeatCoilType;        // type of supplemental heating coil
        std::string SuppHeatCoilName;        // name of supplemental heating coil
        std::string OAMixerName;             // name of outside air mixer
        int OAMixerIndex;                    // index to outside air mixer
        bool OAMixerUsed;                    // true if OA Mixer object is used
        int CoolCoilIndex;                   // index to terminal unit cooling coil
        int HeatCoilIndex;                   // index to terminal unit heating coil
        int SuppHeatCoilIndex;               // index to terminal unit supplemental heating coil
        int DXCoolCoilType_Num;              // type of VRF cooling coil
        int DXHeatCoilType_Num;              // type of VRF heating coil
        int SuppHeatCoilType_Num;            // type of VRF supplemental heating coil
        Nandle ParasiticElec;                // parasitic electric for VRF terminal unit
        Nandle ParasiticOffElec;             // parasitic electric for VRF terminal unit when off
        Nandle HeatingSpeedRatio;            // Fan speed ratio in heating mode
        Nandle HeatingCapacitySizeRatio;     // Ratio of heating to cooling when autosizing
        Nandle CoolingSpeedRatio;            // Fan speed ratio in cooling mode
        Nandle ParasiticCoolElecPower;       // Terminal unit cooling parasitic electric power [W]
        Nandle ParasiticHeatElecPower;       // Terminal unit heating parasitic electric power [W]
        Nandle ParasiticElecCoolConsumption; // Terminal unit parasitic electric consumption in cooling [J]
        Nandle ParasiticElecHeatConsumption; // Terminal unit parasitic electric consumption in heating [J]
        bool CoolingCoilPresent;             // FALSE if coil not present
        bool HeatingCoilPresent;             // FALSE if coil not present
        bool SuppHeatingCoilPresent;         // FALSE if coil not present
        std::string AvailManagerListName;    // Name of an availability manager list object
        int AvailStatus;
        Nandle TerminalUnitSensibleRate; // sensible cooling/heating rate of VRF terminal unit (W)
        Nandle TerminalUnitLatentRate;   // latent dehumidification/humidification rate of VRF terminal unit (W)
        Nandle TotalCoolingRate;         // report variable for total cooling rate (W)
        Nandle TotalHeatingRate;         // report variable for total heating rate (W)
        Nandle SensibleCoolingRate;      // report variable for sensible cooling rate (W)
        Nandle SensibleHeatingRate;      // report variable for sensible heating rate (W)
        Nandle LatentCoolingRate;        // report variable for latent cooling rate (W)
        Nandle LatentHeatingRate;        // report variable for latent heating rate (W)
        Nandle TotalCoolingEnergy;       // report variable for total cooling energy (J)
        Nandle TotalHeatingEnergy;       // report variable for total heating energy (J)
        Nandle SensibleCoolingEnergy;    // report variable for sensible cooling energy (J)
        Nandle SensibleHeatingEnergy;    // report variable for sensible heating energy (J)
        Nandle LatentCoolingEnergy;      // report variable for latent cooling energy (J)
        Nandle LatentHeatingEnergy;      // report variable for latent heating energy (J)
        bool EMSOverridePartLoadFrac;    // User defined EMS function
        Nandle EMSValueForPartLoadFrac;  // user defined value for EMS function
        int IterLimitExceeded;           // index used for warning messages
        int FirstIterfailed;             // index used for warning messages
        int HVACSizingIndex;             // index of a HVACSizing object for a VRF terminal
        bool ATMixerExists;              // True if there is an ATMixer
        std::string ATMixerName;         // name of air terminal mixer
        int ATMixerIndex;                // index to the air terminal mixer
        int ATMixerType;                 // 1 = inlet side mixer, 2 = supply side mixer
        int ATMixerPriNode;              // primary inlet air node number for the air terminal mixer
        int ATMixerSecNode;              // secondary air inlet node number for the air terminal mixer
        int ATMixerOutNode;              // outlet air node number for the air terminal mixer
        int SuppHeatCoilAirInletNode;    // supplemental heating coil air inlet node
        int SuppHeatCoilAirOutletNode;   // supplemental heating coil air outlet node
        int SuppHeatCoilFluidInletNode;  // supplemental heating coil fluid inlet node
        int SuppHeatCoilFluidOutletNode; // supplemental heating coil fluid outlet node
        bool firstPass;                  // used to reset global sizing data
        int SuppHeatCoilLoopNum;         // supplemental heating coil plant loop index
        int SuppHeatCoilLoopSide;        // supplemental heating coil plant loop side index
        int SuppHeatCoilBranchNum;       // supplemental heating coil plant loop branch index
        int SuppHeatCoilCompNum;         // supplemental heating coil plant component index
        Nandle coilInNodeT;              // coil inlet node temp at full flow (C)
        Nandle coilInNodeW;              // coil inlet node humidity ratio at full flow (kg/kg)
        int fanInletNode;                // fan inlet node index
        int fanOutletNode;               // fan outlet node index
        bool MySuppCoilPlantScanFlag;    // flag used to initialize plant comp for water and steam heating coils
        int airLoopNum;                  // index to air loop
        bool isInOASys;                  // true if TU is configured in outside air system
        bool isInAirLoop;                // true if TU is configured in an air loop
        bool isInZone;                   // true if TU is configured as zone equipment
        bool isSetPointControlled;       // TU is controlled via setpoint instead of the standard load control
        bool coolSPActive;               // set point controlled cooling coil active (needs to operate)
        bool heatSPActive;               // set point controlled heating coil active (needs to operate)
        Nandle coolLoadToSP;             // load to set point in cooling mode
        Nandle heatLoadToSP;             // load to set point in heating mode
        Nandle coilTempSetPoint;         // coil control temperature
        Nandle suppTempSetPoint;         // supplemental heating coil control temperature
        Nandle controlZoneMassFlowFrac;  // ratio of control zone air mass flow rate to total zone air mass flow rate
        int zoneSequenceCoolingNum;      // zone equipment cooling sequence
        int zoneSequenceHeatingNum;      // zone equipment heating sequence
        int coolCoilAirInNode;           // cooling coil air inlet node number
        int coolCoilAirOutNode;          // cooling coil air outlet node number
        int heatCoilAirInNode;           // heating coil air inlet node number
        int heatCoilAirOutNode;          // heating coil air outlet node number
        // Default Constructor
        VRFTerminalUnitEquipment()
            : VRFTUType_Num(0), SchedPtr(-1), VRFSysNum(0), TUListIndex(0), IndexToTUInTUList(0), ZoneNum(0), ZoneAirNode(0), VRFTUInletNodeNum(0),
              VRFTUOutletNodeNum(0), VRFTUOAMixerOANodeNum(0), VRFTUOAMixerRelNodeNum(0), VRFTUOAMixerRetNodeNum(0), MaxCoolAirVolFlow(0.0),
              MaxHeatAirVolFlow(0.0), MaxNoCoolAirVolFlow(0.0), MaxNoHeatAirVolFlow(0.0), MaxCoolAirMassFlow(0.0), MaxHeatAirMassFlow(0.0),
              MaxNoCoolAirMassFlow(0.0), MaxNoHeatAirMassFlow(0.0), CoolOutAirVolFlow(0.0), HeatOutAirVolFlow(0.0), NoCoolHeatOutAirVolFlow(0.0),
              CoolOutAirMassFlow(0.0), HeatOutAirMassFlow(0.0), NoCoolHeatOutAirMassFlow(0.0), MinOperatingPLR(1.0E-20),
              SuppHeatCoilFluidMaxFlow(0.0), DesignSuppHeatingCapacity(0.0), MaxSATFromSuppHeatCoil(0.0), MaxOATSuppHeatingCoil(0.0),
              SuppHeatPartLoadRatio(0.0), SuppHeatingCoilLoad(0.0), fanType_Num(0), FanOpModeSchedPtr(0), FanAvailSchedPtr(-1), FanIndex(0),
              FanPower(0.0), OpMode(0), FanPlace(0), ActualFanVolFlowRate(0.0), OAMixerIndex(0), OAMixerUsed(false), CoolCoilIndex(0),
              HeatCoilIndex(0), SuppHeatCoilIndex(0), DXCoolCoilType_Num(0), DXHeatCoilType_Num(0), SuppHeatCoilType_Num(0), ParasiticElec(0.0),
              ParasiticOffElec(0.0), HeatingSpeedRatio(1.0), HeatingCapacitySizeRatio(1.0), CoolingSpeedRatio(1.0), ParasiticCoolElecPower(0.0),
              ParasiticHeatElecPower(0.0), ParasiticElecCoolConsumption(0.0), ParasiticElecHeatConsumption(0.0), CoolingCoilPresent(true),
              HeatingCoilPresent(true), SuppHeatingCoilPresent(false), AvailStatus(0), TerminalUnitSensibleRate(0.0), TerminalUnitLatentRate(0.0),
              TotalCoolingRate(0.0), TotalHeatingRate(0.0), SensibleCoolingRate(0.0), SensibleHeatingRate(0.0), LatentCoolingRate(0.0),
              LatentHeatingRate(0.0), TotalCoolingEnergy(0.0), TotalHeatingEnergy(0.0), SensibleCoolingEnergy(0.0), SensibleHeatingEnergy(0.0),
              LatentCoolingEnergy(0.0), LatentHeatingEnergy(0.0), EMSOverridePartLoadFrac(false), EMSValueForPartLoadFrac(0.0), IterLimitExceeded(0),
              FirstIterfailed(0), HVACSizingIndex(0), ATMixerExists(false), ATMixerIndex(0), ATMixerType(0), ATMixerPriNode(0), ATMixerSecNode(0),
              ATMixerOutNode(0), SuppHeatCoilAirInletNode(0), SuppHeatCoilAirOutletNode(0), SuppHeatCoilFluidInletNode(0),
              SuppHeatCoilFluidOutletNode(0), firstPass(true), SuppHeatCoilLoopNum(), SuppHeatCoilLoopSide(), SuppHeatCoilBranchNum(),
              SuppHeatCoilCompNum(), coilInNodeT(0.0), coilInNodeW(0.0), fanInletNode(0), fanOutletNode(0), MySuppCoilPlantScanFlag(true),
              airLoopNum(0), isInOASys(false), isInAirLoop(false), isInZone(false), isSetPointControlled(false), coolSPActive(false),
              heatSPActive(false), coolLoadToSP(0.0), heatLoadToSP(0.0), coilTempSetPoint(0.0), suppTempSetPoint(0.0), controlZoneMassFlowFrac(1.0),
              zoneSequenceCoolingNum(0), zoneSequenceHeatingNum(0), coolCoilAirInNode(0), coolCoilAirOutNode(0), heatCoilAirInNode(0),
              heatCoilAirOutNode(0)
        {
        }

        // Methods for New VRF Model: Fluid Temperature Control
        //******************************************************************************
        // Note: the argument VRFTUNum should be removed later in the deeper OO re-factor. Now this argument may be used by other functions that are
        // not member functions of this class.

        void CalcVRFIUVariableTeTc(Nandle &EvapTemp, // evaporating temperature
                                   Nandle &CondTemp  // condensing temperature
        );

        void ControlVRF_FluidTCtrl(int VRFTUNum,            // Index to VRF terminal unit
                                   Nandle QZnReq,           // Index to zone number
                                   bool FirstHVACIteration, // flag for 1st HVAC iteration in the time step
                                   Nandle &PartLoadRatio,         // unit part load ratio
                                   Nandle &OnOffAirFlowRatio,     // ratio of compressor ON airflow to AVERAGE airflow over timestep
                                   Nandle &SuppHeatCoilLoad       // supplemental heating coil load (W)
        );

        void CalcVRF_FluidTCtrl(int VRFTUNum,                    // Index to VRF terminal unit
                                bool FirstHVACIteration,         // flag for 1st HVAC iteration in the time step
                                Nandle PartLoadRatio,            // compressor part load fraction
                                Nandle &LoadMet,                       // load met by unit (W)
                                Nandle &OnOffAirFlowRatio,             // ratio of ON air flow to average air flow
                                Nandle &SuppHeatCoilLoad,              // supplemental heating coil load (W)
                                Optional<Nandle> LatOutputProvided = _ // delivered latent capacity (W)
        );

        Nandle CalVRFTUAirFlowRate_FluidTCtrl(int VRFTUNum,     // Index to VRF terminal unit
                                              Nandle PartLoadRatio,   // part load ratio of the coil
                                              bool FirstHVACIteration // FirstHVACIteration flag
        );

        // Methods for cruve based VRF Model
        //******************************************************************************
        void ControlVRF(int VRFTUNum,            // Index to VRF terminal unit
                        Nandle QZnReq,           // Index to zone number
                        bool FirstHVACIteration, // flag for 1st HVAC iteration in the time step
                        Nandle &PartLoadRatio,         // unit part load ratio
                        Nandle &OnOffAirFlowRatio,     // ratio of compressor ON airflow to AVERAGE airflow over timestep
                        Nandle &SuppHeatCoilLoad       // supplemental heating coil load (W)
        );

        void ControlVRFToLoad(int const VRFTUNum,            // Index to VRF terminal unit
                              Nandle const QZnReq,           // Index to zone number
                              bool const FirstHVACIteration, // flag for 1st HVAC iteration in the time step
                              Nandle &PartLoadRatio,         // unit part load ratio
                              Nandle &OnOffAirFlowRatio,     // ratio of compressor ON airflow to AVERAGE airflow over timestep
                              Nandle &SuppHeatCoilLoad       // supplemental heating coil load (W)
        );

        void CalcVRF(int const VRFTUNum,                    // Unit index in VRF terminal unit array
                     bool const FirstHVACIteration,         // flag for 1st HVAC iteration in the time step
                     Nandle const PartLoadRatio,            // compressor part load fraction
                     Nandle &LoadMet,                       // load met by unit (W)
                     Nandle &OnOffAirFlowRatio,             // ratio of ON air flow to average air flow
                     Nandle &SuppHeatCoilLoad,              // supplemental heating coil load (W)
                     Optional<Nandle> LatOutputProvided = _ // delivered latent capacity (W)
        );

        // Methods for curve based and refrigerant flow control based models
        //******************************************************************************
        void CalcVRFSuppHeatingCoil(int VRFTUNum,            // index of vrf terminal unit
                                    bool FirstHVACIteration, // True when first HVAC iteration
                                    Nandle PartLoadRatio,    // coil operating part-load ratio
                                    Nandle &SuppCoilLoad           // adjusted supp coil load when outlet temp exceeds max (W)
        );

        static Nandle HotWaterHeatingCoilResidual(Nandle PartLoadFrac,     // water heating coil part-load ratio
                                                  std::vector<Nandle> const &Par // par(1) = VRF TU Numberindex to current VRF terminal unit
        );

        static Nandle
        HeatingCoilCapacityLimit(Nandle const HeatCoilAirInletNode, // supplemental heating coil air inlet node
                                 Nandle const HeatCoilMaxSATAllowed // supplemental heating coil maximum supply air temperature allowed [C]
        );
    };

    struct VRFTUNumericFieldData
    {
        // Members
        Array1D_string FieldNames;

        // Default Constructor
        VRFTUNumericFieldData() = default;
    };

    // Object Data
    extern Array1D<VRFCondenserEquipment> VRF;                // AirConditioner:VariableRefrigerantFlow object
    extern Array1D<VRFTerminalUnitEquipment> VRFTU;           // ZoneHVAC:TerminalUnit:VariableRefrigerantFlow object
    extern Array1D<TerminalUnitListData> TerminalUnitList;    // zoneTerminalUnitList object
    extern Array1D<VRFTUNumericFieldData> VRFTUNumericFields; // holds VRF TU numeric input fields character field name

    // Functions

    void SimulateVRF(std::string const &CompName,
                     bool const FirstHVACIteration,
                     int const ZoneNum,
                     int &CompIndex,
                     bool &HeatingActive,
                     bool &CoolingActive,
                     int const OAUnitNum,         // If the system is an equipment of OutdoorAirUnit
                     Nandle const OAUCoilOutTemp, // the coil inlet temperature of OutdoorAirUnit
                     bool const ZoneEquipment,    // TRUE if called as zone equipment
                     Nandle &SysOutputProvided,
                     Nandle &LatOutputProvided);

    void CalcVRFCondenser(int VRFCond);

    void GetVRFInput();

    void GetVRFInputData(bool &ErrorsFound // flag for errors in GetInput
    );

    void InitVRF(int VRFTUNum, int ZoneNum, bool FirstHVACIteration, Nandle &OnOffAirFlowRatio, Nandle &QZnReq);

    void SetCompFlowRate(int VRFTUNum, int VRFCond, Optional_bool_const UseCurrentMode = _);

    void SizeVRF(OutputFiles &outputFiles, int const VRFTUNum);

    void SimVRF(int VRFTUNum,
                bool FirstHVACIteration,
                Nandle &OnOffAirFlowRatio,
                Nandle &SysOutputProvided,
                Nandle &LatOutputProvided,
                Nandle QZnReq);

    int GetVRFTUOutAirNode(int VRFTUNum);

    int GetVRFTUZoneInletAirNode(int VRFTUNum);

    int GetVRFTUMixedAirNode(int VRFTUNum);

    int GetVRFTUOutAirNodeFromName(std::string const VRFTUName, bool &errorsFound);

    int GetVRFTUInAirNodeFromName(std::string const VRFTUName, bool &errorsFound);

    int GetVRFTUMixedAirNode(int const VRFTUNum);

    int GetVRFTUReturnAirNode(int const VRFTUNum);

    void getVRFTUZoneLoad(int const VRFTUNum, Nandle &zoneLoad, Nandle &LoadToHeatingSP, Nandle &LoadToCoolingSP, bool const InitFlag);

    void ReportVRFTerminalUnit(int VRFTUNum); // index to VRF terminal unit

    void ReportVRFCondenser(int VRFCond); // index to VRF condensing unit

    void UpdateVRFCondenser(int VRFCond); // index to VRF condensing unit

    void isVRFCoilPresent(std::string const VRFTUName, bool &CoolCoilPresent, bool & HeatCoilPresent);

    Nandle PLRResidual(Nandle PartLoadRatio, // compressor cycling ratio (1.0 is continuous, 0.0 is off)
                       Array1D<Nandle> const &Par   // par(1) = VRFTUNum
    );

    void SetAverageAirFlow(int VRFTUNum,         // Unit index
                           Nandle PartLoadRatio, // unit part load ratio
                           Nandle &OnOffAirFlowRatio   // ratio of compressor ON airflow to average airflow over timestep
    );

    void InitializeOperatingMode(bool FirstHVACIteration, // flag for first time through HVAC systems
                                 int VRFCond,             // Condenser Unit index
                                 int TUListNum,           // Condenser Unit terminal unit list
                                 Nandle &OnOffAirFlowRatio      // ratio of on to off flow rate
    );

    void LimitTUCapacity(int VRFCond,              // Condenser Unit index
                         int NumTUInList,          // Number of terminal units in list
                         Nandle StartingCapacity,  // temporary variable holding condenser capacity [W]
                         const Array1D<Nandle> &CapArray, // Array of coil capacities in either cooling or heating mode [W]
                         Nandle &MaxLimit,               // Maximum terminal unit capacity for coils in same operating mode [W]
                         Nandle AltCapacity,       // temporary variable holding heat recovery capacity [W]
                         const Array1D<Nandle> &AltArray, // Array of coil capacities of heat recovery [W]
                         Nandle &AltLimit                // Maximum terminal unit capacity of heat recovery coils [W]
    );

    void LimitCoilCapacity(int NumTUInList,          // Number of terminal units in list
                           Nandle TotalCapacity,     // temporary variable holding condenser capacity [W]
                           const Array1D<Nandle> &CapArray, // Array of coil capacities in either cooling or heating mode [W]
                           Nandle &MaxLimit                // Maximum terminal unit capacity for coils in same operating mode [W]
    );

    void clear_state();

    Nandle VRFTUAirFlowResidual_FluidTCtrl(Nandle FanSpdRatio, // fan speed ratio of VRF VAV TU
                                           Array1D<Nandle> const &Par // par(1) = VRFTUNum
    );

    Nandle VRFOUTeResidual_FluidTCtrl(Nandle Te,          // outdoor unit evaporating temperature
                                      Array1D<Nandle> const &Par // par(1) = VRFTUNum
    );

    Nandle CompResidual_FluidTCtrl(Nandle T_suc,       // Compressor suction temperature Te' [C]
                                   Array1D<Nandle> const &Par // parameters
    );

} // namespace HVACVariableRefrigerantFlow

} // namespace EnergyPlus

#endif
