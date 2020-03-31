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

#ifndef VariableSpeedCoils_hh_INCLUDED
#define VariableSpeedCoils_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.fwd.hh>
#include <ObjexxFCL/Optional.fwd.hh>

// EnergyPlus Headers
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

namespace VariableSpeedCoils {

    // Using/Aliasing

    // Data
    // MODULE PARAMETER DEFINITIONS

    extern Nandle const RatedInletAirTemp;       // 26.6667C or 80F
    extern Nandle const RatedInletWetBulbTemp;   // 19.44 or 67F, cooling mode
    extern Nandle const RatedInletAirHumRat;     // Humidity ratio corresponding to 80F dry bulb/67F wet bulb
    extern Nandle const RatedInletWaterTemp;     // 85 F cooling mode
    extern Nandle const RatedAmbAirTemp;         // 95 F cooling mode
    extern Nandle const RatedInletAirTempHeat;   // 21.11C or 70F, heating mode
    extern Nandle const RatedInletWaterTempHeat; // 21.11C or 70F, heating mode
    extern Nandle const RatedAmbAirTempHeat;     // 8.33 or 47F, heating mode
    extern Nandle const RatedAmbAirWBHeat;       // 8.33 or 43F, heating mode, rated wet bulb temperature

    // Airflow per total capacity range
    extern Nandle const MaxRatedVolFlowPerRatedTotCap; // m3/s per watt = 450 cfm/ton
    extern Nandle const MinRatedVolFlowPerRatedTotCap; // m3/s per watt = 300 cfm/ton
    extern Nandle const MaxHeatVolFlowPerRatedTotCap;  // m3/s per watt = 600 cfm/ton
    extern Nandle const MaxCoolVolFlowPerRatedTotCap;  // m3/s per watt = 500 cfm/ton
    extern Nandle const MinOperVolFlowPerRatedTotCap;  // m3/s per watt = 200 cfm/ton

    // Water Systems
    extern int const CondensateDiscarded; // default mode where water is "lost"
    extern int const CondensateToTank;    // collect coil condensate from air and store in water storage tank

    extern int const WaterSupplyFromMains;
    extern int const WaterSupplyFromTank;

    // Curve Types
    extern int const Linear;
    extern int const BiLinear;
    extern int const Quadratic;
    extern int const BiQuadratic;
    extern int const Cubic;

    // Defrost strategy (heat pump only)
    extern int const ReverseCycle; // uses reverse cycle defrost strategy
    extern int const Resistive;    // uses electric resistance heater for defrost
    // Defrost control  (heat pump only)
    extern int const Timed;    // defrost cycle is timed
    extern int const OnDemand; // defrost cycle occurs only when required

    extern int const MaxSpedLevels; // Maximum number of speed that supports

    // DERIVED TYPE DEFINITIONS

    // MODULE VARIABLE DECLARATIONS:
    // Identifier is VarSpeedCoil
    extern int NumVarSpeedCoils; // The Number of variable speed Water to Air Heat Pumps and variable dx coils found in the Input

    extern bool GetCoilsInputFlag; // Flag set to make sure you get input once
    // LOGICAL, ALLOCATABLE, DIMENSION(:) :: MySizeFlag

    extern Nandle SourceSideMassFlowRate; // Source Side Mass flow rate [Kg/s]
    extern Nandle SourceSideInletTemp;    // Source Side Inlet Temperature [C]
    extern Nandle SourceSideInletEnth;    // Source Side Inlet Enthalpy [J/kg]
    extern Nandle LoadSideMassFlowRate;   // Load Side Mass flow rate [Kg/s]
    extern Nandle LoadSideInletDBTemp;    // Load Side Inlet Dry Bulb Temp [C]
    extern Nandle LoadSideInletWBTemp;    // Load Side Inlet Wet Bulb Temp [C]
    extern Nandle LoadSideInletHumRat;    // Load Side Outlet Humidity ratio
    extern Nandle LoadSideInletEnth;      // Load Side Inlet Enthalpy [J/kg]
    extern Nandle LoadSideOutletDBTemp;   // Load Side Outlet Dry Bulb Temp [C]
    extern Nandle LoadSideOutletHumRat;   // Load Side Outlet Humidity ratio
    extern Nandle LoadSideOutletEnth;     // Load Side Outlet Enthalpy [J/kg]
    extern Nandle QSensible;              // Load side sensible heat transfer rate [W]
    extern Nandle QLoadTotal;             // Load side total heat transfer rate [W]
    extern Nandle QLatRated;              // Latent Capacity [W] rated at entering air conditions [Tdb=26.7C Twb=19.4C]
    extern Nandle QLatActual;             // Actual Latent Capacity [W]
    extern Nandle QSource;                // Source side heat transfer rate [W]
    extern Nandle Winput;                 // Power Consumption [W]
    extern Nandle PLRCorrLoadSideMdot;    // Load Side Mdot corrected for Part Load Ratio of the unit

    extern Nandle VSHPWHHeatingCapacity; // Used by Heat Pump:Water Heater object as total water heating capacity [W]
    extern Nandle VSHPWHHeatingCOP;      // Used by Heat Pump:Water Heater object as water heating COP [W/W]

    // SUBROUTINE SPECIFICATIONS FOR MODULE

    // Driver/Manager Routines

    // Get Input routines for module

    // Initialization routines for module

    // Update routines to check convergence and update nodes

    // Update routine

    // Utility routines
    // SHR, bypass factor routines

    // Types

    struct VariableSpeedCoilData // variable speed coil
    {
        // Members
        std::string Name;              // Name of the  Coil
        std::string VarSpeedCoilType;  // type of coil
        int NumOfSpeeds;               // Number of speeds
        int NormSpedLevel;             // Nominal speed level
        Nandle RatedWaterVolFlowRate;  // Rated/Ref Water Volumetric Flow Rate [m3/s]
        Nandle RatedWaterMassFlowRate; // Rated/Ref Water Volumetric Flow Rate [m3/s]
        Nandle RatedAirVolFlowRate;    // Rated/Ref Air Volumetric Flow Rate [m3/s]
        Nandle RatedCapHeat;           // Rated/Ref Heating Capacity [W]
        Nandle RatedCapCoolTotal;      // Rated/Ref Total Cooling Capacity [W]
        Nandle MaxONOFFCyclesperHour;  // Maximum ON/OFF cycles per hour for the compressor (cycles/hour)
        Nandle Twet_Rated;             // Nominal time for condensate to begin leaving the coil's
        // condensate drain line (sec)
        Nandle Gamma_Rated; // Initial moisture evaporation rate divided by steady-state
        // AC latent capacity (dimensionless)
        int HOTGASREHEATFLG;            // whether to use hot gas reheat
        Nandle HPTimeConstant;          // Heat pump time constant [s]
        int PLFFPLR;                    // index of part load curve as a function of part load ratio
        std::string CoolHeatType;       // Type of WatertoAirHP ie. Heating or Cooling
        int VSCoilTypeOfNum;            // type of component in plant
        bool SimFlag;                   // Heat Pump Simulation Flag
        Nandle DesignWaterMassFlowRate; // design water mass flow rate [kg/s]
        Nandle DesignWaterVolFlowRate;  // design water volumetric flow rate [m3/s]
        Nandle DesignAirMassFlowRate;   // Design Air Mass Flow Rate [kg/s]
        Nandle DesignAirVolFlowRate;    // Design Air Volumetric Flow Rate [m3/s]
        Nandle AirVolFlowRate;          // Air Volumetric Flow Rate[m3/s], real time
        Nandle AirMassFlowRate;         // Air Mass Flow Rate[kg/s], real time
        Nandle InletAirPressure;        // air inlet pressure [pa]
        Nandle InletAirDBTemp;          // Inlet Air Dry Bulb Temperature [C], real time
        Nandle InletAirHumRat;          // Inlet Air Humidity Ratio [kg/kg], real time
        Nandle InletAirEnthalpy;        // Inlet Air Enthalpy [J/kg], real time
        Nandle OutletAirDBTemp;         // Outlet Air Dry Bulb Temperature [C], real time
        Nandle OutletAirHumRat;         // Outlet Air Humidity Ratio [kg/kg], real time
        Nandle OutletAirEnthalpy;       // Outlet Air Enthalpy [J/kg], real time
        Nandle WaterVolFlowRate;        // Water Volumetric Flow Rate [m3/s], real time
        Nandle WaterMassFlowRate;       // Water Mass Flow Rate [kg/s], real time
        Nandle InletWaterTemp;          // Inlet Water Temperature [C]
        Nandle InletWaterEnthalpy;      // Inlet Water Enthalpy [J/kg]
        Nandle OutletWaterTemp;         // Outlet Water Temperature [C]
        Nandle OutletWaterEnthalpy;     // Outlet Water Enthalpy [J/kg]
        Nandle Power;                   // Power Consumption [W]
        Nandle QLoadTotal;              // Load Side Total Heat Transfer Rate [W]
        Nandle QSensible;               // Sensible Load Side Heat Transfer Rate [W]
        Nandle QLatent;                 // Latent Load Side Heat Transfer Rate [W]
        Nandle QSource;                 // Source Side Heat Transfer Rate [W]
        Nandle QWasteHeat;              // Recoverable waste Heat Transfer Rate [W]
        Nandle Energy;                  // Energy Consumption [J]
        Nandle EnergyLoadTotal;         // Load Side Total Heat Transferred [J]
        Nandle EnergySensible;          // Sensible Load Side Heat Transferred [J]
        Nandle EnergyLatent;            // Latent Load Side Heat Transferred [J]
        Nandle EnergySource;            // Source Side Heat Transferred [J]
        Nandle COP;                     // Heat Pump Coefficient of Performance [-]
        Nandle RunFrac;                 // Duty Factor
        Nandle PartLoadRatio;           // Part Load Ratio
        Nandle RatedPowerHeat;          // Rated/Ref Heating Power Consumption[W]
        Nandle RatedCOPHeat;            // Rated/Ref Heating COP [W/W]
        Nandle RatedCapCoolSens;        // Rated/Ref Sensible Cooling Capacity [W]
        Nandle RatedPowerCool;          // Rated/Ref Cooling Power Consumption[W]
        Nandle RatedCOPCool;            // Rated/Ref Cooling COP [W/W]
        int AirInletNodeNum;            // Node Number of the Air Inlet
        int AirOutletNodeNum;           // Node Number of the Air Outlet
        int WaterInletNodeNum;          // Node Number of the Water Onlet
        int WaterOutletNodeNum;         // Node Number of the Water Outlet
        int LoopNum;                    // plant loop index for water side
        int LoopSide;                   // plant loop side index
        int BranchNum;                  // plant branch index
        int CompNum;                    // plant component index
        // set by parent object and "pushed" to this structure in SetVSWSHPData subroutine
        bool FindCompanionUpStreamCoil; // Flag to get the companion coil in Init
        bool IsDXCoilInZone;            // true means dx coil is in zone instead of outside
        int CompanionCoolingCoilNum;    // Heating coil companion cooling coil index
        int CompanionHeatingCoilNum;    // Cooling coil companion heating coil index
        Nandle FanDelayTime;            // Fan delay time, time delay for the HP's fan to
        // beginning for multispeed coil type
        int MSHPDesignSpecIndex;              // index to UnitarySystemPerformance:Multispeed object
        Array1D_int MSErrIndex;               // index flag for num speeds/recurring messages
        Array1D<Nandle> MSRatedPercentTotCap; // Percentage to the total cooling capacity for MS heat pump at the highest speed [dimensionless]
        Array1D<Nandle> MSRatedTotCap;        // Rated cooling capacity for MS heat pump [W]
        Array1D<Nandle> MSRatedSHR;           // Rated SHR for MS heat pump [dimensionless]
        Array1D<Nandle> MSRatedCOP;           // Rated COP for MS heat pump [dimensionless]
        Array1D<Nandle> MSRatedAirVolFlowPerRatedTotCap;
        // Rated Air volume flow rate per total capacity through unit at rated conditions [m^3/w]
        Array1D<Nandle> MSRatedAirVolFlowRate;
        // Air volume flow rate through unit at rated conditions [m3/s]
        Array1D<Nandle> MSRatedAirMassFlowRate;
        // Air mass flow rate through unit at rated conditions [kg/s]
        Array1D<Nandle> MSRatedWaterVolFlowPerRatedTotCap;
        // Rated water volume flow rate per total  capacity through unit at rated conditions [m^3/w]
        Array1D<Nandle> MSRatedWaterVolFlowRate;
        // Water volume flow rate through unit at rated conditions [m3/s]
        Array1D<Nandle> MSRatedWaterMassFlowRate;
        // Water mass flow rate through unit at rated conditions [kg/s]
        Array1D<Nandle> MSRatedCBF;
        // rated coil bypass factor
        Array1D<Nandle> MSEffectiveAo;
        // effective heat transfer surface at each speed
        Array1D_int MSCCapFTemp;
        // index of total capacity modifier curve
        Array1D_int MSCCapAirFFlow;
        // index of total capacity modifier curve as a function of air flow
        Array1D_int MSCCapWaterFFlow;
        // index of total capacity modifier curve as a function of water flow
        Array1D_int MSEIRFTemp;
        // index of energy input ratio modifier curve as a function of temperature
        Array1D_int MSEIRAirFFlow;
        // index of energy input ratio modifier curve as a function of air flow fraction
        Array1D_int MSEIRWaterFFlow;
        // index of energy input ratio modifier curve as a function of water flow fraction
        Array1D_int MSWasteHeat;
        // index of waste heat as a function of temperature
        Array1D<Nandle> MSWasteHeatFrac;
        // water heating coil pump power at various speeds
        Array1D<Nandle> MSWHPumpPower;
        Array1D<Nandle> MSWHPumpPowerPerRatedTotCap;
        // Waste heat fraction
        Nandle SpeedNumReport;
        // speed number for output
        Nandle SpeedRatioReport;
        // speed ratio for output between two neighboring speeds
        // End of multispeed water source coil input
        //----------------------------------------------------------------
        // added variables and arrays for variable speed air-source heat pump
        // defrosting
        int DefrostStrategy;       // defrost strategy; 1=reverse-cycle, 2=resistive
        int DefrostControl;        // defrost control; 1=timed, 2=on-demand
        int EIRFPLR;               // index of energy input ratio vs part-load ratio curve
        int DefrostEIRFT;          // index of defrost mode total cooling capacity for reverse cycle heat pump
        Nandle MinOATCompressor;   // Minimum OAT for heat pump compressor operation
        Nandle OATempCompressorOn; // The outdoor tempearture when the compressor is automatically turned back on,
        // if applicable, following automatic shut off. This field is used only for
        // HSPF calculation.
        Nandle MaxOATDefrost;           // Maximum OAT for defrost operation
        Nandle DefrostTime;             // Defrost time period in hours
        Nandle DefrostCapacity;         // Resistive defrost to nominal capacity (at 21.11C/8.33C) ratio
        Nandle HPCompressorRuntime;     // keep track of compressor runtime
        Nandle HPCompressorRuntimeLast; // keep track of last time step compressor runtime (if simulation downshifts)
        Nandle TimeLeftToDefrost;       // keep track of time left to defrost heat pump
        Nandle DefrostPower;            // power used during defrost
        Nandle DefrostConsumption;      // energy used during defrost
        // crankcase heater
        bool ReportCoolingCoilCrankcasePower; // logical determines if the cooling coil crankcase heater power is reported
        Nandle CrankcaseHeaterCapacity;       // total crankcase heater capacity [W]
        Nandle CrankcaseHeaterPower;          // report variable for average crankcase heater power [W]
        Nandle MaxOATCrankcaseHeater;         // maximum OAT for crankcase heater operation [C]
        Nandle CrankcaseHeaterConsumption;    // report variable for total crankcase heater energy consumption [J]
        // condenser evaporative precooling
        int CondenserInletNodeNum;       // Node number of outdoor condenser
        int CondenserType;               // Type of condenser for DX cooling coil: AIR COOLED or EVAP COOLED
        bool ReportEvapCondVars;         // true if any performance mode includes an evap condenser
        Nandle EvapCondPumpElecNomPower; // Nominal power input to the evap condenser water circulation pump [W]
        Nandle EvapCondPumpElecPower;    // Average power consumed by the evap condenser water circulation pump over
        // the time step [W]
        Nandle EvapWaterConsumpRate;        // Evap condenser water consumption rate [m3/s]
        Nandle EvapCondPumpElecConsumption; // Electric energy consumed by the evap condenser water circulation pump [J]
        Nandle EvapWaterConsump;            // Evap condenser water consumption [m3]
        Nandle BasinHeaterConsumption;      // Basin heater energy consumption (J)
        Nandle BasinHeaterPowerFTempDiff;   // Basin heater capacity per degree C below setpoint (W/C)
        Nandle BasinHeaterSetPointTemp;     // setpoint temperature for basin heater operation (C)
        Nandle BasinHeaterPower;            // Basin heater power (W)
        int BasinHeaterSchedulePtr;         // Pointer to basin heater schedule
        Array1D<Nandle> EvapCondAirFlow;    // Air flow rate through the evap condenser at high speed, volumetric flow rate
        // for water use calcs [m3/s]
        Array1D<Nandle> EvapCondEffect; // effectiveness of the evaporatively cooled condenser
        // [high speed for multi-speed unit] (-)
        Array1D<Nandle> MSRatedEvapCondVolFlowPerRatedTotCap; // evap condenser air flow ratio to capacity
        // begin variables for Water System interactions
        int EvapWaterSupplyMode;         // where does water come from
        std::string EvapWaterSupplyName; // name of water source e.g. water storage tank
        int EvapWaterSupTankID;
        int EvapWaterTankDemandARRID;
        int CondensateCollectMode;         // where does water come from
        std::string CondensateCollectName; // name of water source e.g. water storage tank
        int CondensateTankID;
        int CondensateTankSupplyARRID;
        Nandle CondensateVdot;         // rate of water condensation from air stream [m3/s]
        Nandle CondensateVol;          // amount of water condensed from air stream [m3]
        Nandle CondInletTemp;          // Evap condenser inlet temperature [C], report variable
        int SupplyFanIndex;            // index of this fan in fan array or vector
        int SupplyFan_TypeNum;         // type of fan, in DataHVACGlobals
        std::string SupplyFanName;     // name of fan associated with this dx coil
        Nandle SourceAirMassFlowRate;  // source air mass flow rate [kg/s]
        Nandle InletSourceAirTemp;     // source air temperature entering the outdoor coil [C]
        Nandle InletSourceAirEnthalpy; // source air enthalpy entering the outdoor coil [J/kg]
        // end variables for water system interactions

        // begin varibles for HPWH
        Nandle RatedCapWH;                  // Rated water heating Capacity [W]
        int InletAirTemperatureType;        // Specifies to use either air wet-bulb or dry-bulb temp for curve objects
        Nandle WHRatedInletDBTemp;          // Rated inlet air dry-bulb temperature [C]
        Nandle WHRatedInletWBTemp;          // Rated inlet air wet-bulb temperature [C]
        Nandle WHRatedInletWaterTemp;       // Rated condenser water inlet temperature [C]
        Nandle HPWHCondPumpElecNomPower;    // Nominal power input to the condenser water circulation pump [W]
        Nandle HPWHCondPumpFracToWater;     // Nominal power fraction to water for the condenser water circulation pump
        Nandle RatedHPWHCondWaterFlow;      // Rated water flow rate through the condenser of the HPWH DX coil [m3/s]
        Nandle ElecWaterHeatingPower;       // Total electric power consumed by compressor and condenser pump [W]
        Nandle ElecWaterHeatingConsumption; // Total electric consumption by compressor and condenser pump [J]
        bool FanPowerIncludedInCOP;         // Indicates that fan heat is included in heating capacity and COP
        bool CondPumpHeatInCapacity;        // Indicates that condenser pump heat is included in heating capacity
        bool CondPumpPowerInCOP;            // Indicates that condenser pump power is included in heating COP
        bool AirVolFlowAutoSized;           // Used to report autosizing info for the HPWH DX coil
        bool WaterVolFlowAutoSized;         // Used to report autosizing info for the HPWH DX coil
        Nandle TotalHeatingEnergy;          // total water heating energy
        Nandle TotalHeatingEnergyRate;      // total WH energy rate
        bool bIsDesuperheater;              // whether the coil is used for a desuperheater, i.e. zero all the cooling capacity and power
        // end variables for HPWH
        bool reportCoilFinalSizes; // one time report of sizes to coil selection report
        Nandle capModFacTotal;     // coil  TotCapTempModFac * TotCapAirFFModFac * TotCapWaterFFModFac, for result for simulation peak reporting

        // Default Constructor
        VariableSpeedCoilData();
    };

    // Object Data
    extern Array1D<VariableSpeedCoilData> VarSpeedCoil;

    // Functions
    void clear_state();

    void SimVariableSpeedCoils(std::string const &CompName,   // Coil Name
                               int &CompIndex,                // Index for Component name
                               int const CyclingScheme,       // Continuous fan OR cycling compressor
                               Nandle &MaxONOFFCyclesperHour, // Maximum cycling rate of heat pump [cycles/hr]
                               Nandle &HPTimeConstant,        // Heat pump time constant [s]
                               Nandle &FanDelayTime,          // Fan delay time, time delay for the HP's fan to
                               int const CompOp,              // compressor on/off. 0 = off; 1= on
                               Nandle const PartLoadFrac,
                               int const SpeedNum,                        // compressor speed number
                               Nandle const SpeedRatio,                   // compressor speed ratio
                               Nandle const SensLoad,                     // Sensible demand load [W]
                               Nandle const LatentLoad,                   // Latent demand load [W]
                               Optional<Nandle const> OnOffAirFlowRat = _ // ratio of comp on to comp off air flow rate
    );

    void GetVarSpeedCoilInput();

    // Beginning Initialization Section of the Module
    //******************************************************************************

    void InitVarSpeedCoil(int const DXCoilNum,                // Current DXCoilNum under simulation
                          Nandle const MaxONOFFCyclesperHour, // Maximum cycling rate of heat pump [cycles/hr]
                          Nandle const HPTimeConstant,        // Heat pump time constant [s]
                          Nandle const FanDelayTime,          // Fan delay time, time delay for the HP's fan to
                          Nandle const SensLoad,              // Control zone sensible load[W]
                          Nandle const LatentLoad,            // Control zone latent load[W]
                          int const CyclingScheme,            // fan operating mode
                          Nandle const OnOffAirFlowRatio,     // ratio of compressor on flow to average flow over time step
                          Nandle const SpeedRatio,            // compressor speed ratio
                          int const SpeedNum                  // compressor speed number
    );

    void SizeVarSpeedCoil(int const DXCoilNum);

    void CalcVarSpeedCoilCooling(int const DXCoilNum,            // Heat Pump Number
                                 int const CyclingScheme,        // Fan/Compressor cycling scheme indicator
                                 Nandle &RuntimeFrac,            // Runtime Fraction of compressor or percent on time (on-time/cycle time)
                                 Nandle const SensDemand,        // Cooling Sensible Demand [W] !unused1208
                                 Nandle const LatentDemand,      // Cooling Latent Demand [W]
                                 int const CompOp,               // compressor operation flag
                                 Nandle const PartLoadRatio,     // compressor part load ratio
                                 Nandle const OnOffAirFlowRatio, // ratio of compressor on flow to average flow over time step
                                 Nandle const SpeedRatio,        // SpeedRatio varies between 1.0 (higher speed) and 0.0 (lower speed)
                                 int const SpeedNum              // Speed number, high bound
    );

    void CalcVarSpeedCoilHeating(int const DXCoilNum,            // Heat Pump Number
                                 int const CyclingScheme,        // Fan/Compressor cycling scheme indicator
                                 Nandle &RuntimeFrac,            // Runtime Fraction of compressor or percent on time (on-time/cycle time)
                                 Nandle const SensDemand,        // Cooling Sensible Demand [W] !unused1208
                                 int const CompOp,               // compressor operation flag
                                 Nandle const PartLoadRatio,     // compressor part load ratio
                                 Nandle const OnOffAirFlowRatio, // ratio of compressor on flow to average flow over time step
                                 Nandle const SpeedRatio,        // SpeedRatio varies between 1.0 (higher speed) and 0.0 (lower speed)
                                 int const SpeedNum              // Speed number, high bound, i.e. SpeedNum - 1 is the other side
    );

    Nandle GetCoilCapacityVariableSpeed(std::string const &CoilType, // must match coil types in this module
                                        std::string const &CoilName, // must match coil names for the coil type
                                        bool &ErrorsFound            // set to true if problem
    );

    int GetCoilIndexVariableSpeed(std::string const &CoilType, // must match coil types in this module
                                  std::string const &CoilName, // must match coil names for the coil type
                                  bool &ErrorsFound            // set to true if problem
    );

    Nandle GetCoilAirFlowRateVariableSpeed(std::string const &CoilType, // must match coil types in this module
                                           std::string const &CoilName, // must match coil names for the coil type
                                           bool &ErrorsFound            // set to true if problem
    );

    int GetCoilInletNodeVariableSpeed(std::string const &CoilType, // must match coil types in this module
                                      std::string const &CoilName, // must match coil names for the coil type
                                      bool &ErrorsFound            // set to true if problem
    );

    int GetCoilOutletNodeVariableSpeed(std::string const &CoilType, // must match coil types in this module
                                       std::string const &CoilName, // must match coil names for the coil type
                                       bool &ErrorsFound            // set to true if problem
    );

    int GetVSCoilCondenserInletNode(std::string const &CoilName, // must match coil names for the coil type
                                    bool &ErrorsFound            // set to true if problem
    );

    int GetVSCoilPLFFPLR(std::string const &CoilType, // must match coil types in this module
                         std::string const &CoilName, // must match coil names for the coil type
                         bool &ErrorsFound            // set to true if problem
    );

    int GetVSCoilCapFTCurveIndex(int const &CoilIndex, // must match coil names for the coil type
                                 bool &ErrorsFound     // set to true if problem
    );

    Nandle GetVSCoilMinOATCompressor(std::string const &CoilName, // must match coil names for the coil type
                                     bool &ErrorsFound            // set to true if problem
    );

    Nandle GetVSCoilMinOATCompressorUsingIndex(int const CoilIndex, // index to cooling coil
                                               bool &ErrorsFound    // set to true if problem
    );

    int GetVSCoilNumOfSpeeds(std::string const &CoilName, // must match coil names for the coil type
                             bool &ErrorsFound            // set to true if problem
    );

    void SetVarSpeedCoilData(int const WSHPNum,                        // Number of OA Controller
                             bool &ErrorsFound,                        // Set to true if certain errors found
                             Optional_int CompanionCoolingCoilNum = _, // Index to cooling coil for heating coil = SimpleWSHPNum
                             Optional_int CompanionHeatingCoilNum = _, // Index to heating coil for cooling coil = SimpleWSHPNum
                             Optional_int MSHPDesignSpecIndex = _      // index to UnitarySystemPerformance:Multispeed object
    );

    void UpdateVarSpeedCoil(int const DXCoilNum);

    Nandle CalcEffectiveSHR(int const DXCoilNum,     // Index number for cooling coil
                            Nandle const SHRss,      // Steady-state sensible heat ratio
                            int const CyclingScheme, // Fan/compressor cycling scheme indicator
                            Nandle const RTF,        // Compressor run-time fraction
                            Nandle const QLatRated,  // Rated latent capacity
                            Nandle const QLatActual, // Actual latent capacity
                            Nandle const EnteringDB, // Entering air dry-bulb temperature
                            Nandle const EnteringWB  // Entering air wet-bulb temperature
    );

    void CalcTotCapSHR_VSWSHP(Nandle const InletDryBulb,       // inlet air dry bulb temperature [C]
                              Nandle const InletHumRat,        // inlet air humidity ratio [kg water / kg dry air]
                              Nandle const InletEnthalpy,      // inlet air specific enthalpy [J/kg]
                              Nandle &InletWetBulb,            // inlet air wet bulb temperature [C]
                              Nandle const AirMassFlowRatio,   // Ratio of actual air mass flow to nominal air mass flow
                              Nandle const WaterMassFlowRatio, // Ratio of actual water mass flow to nominal water mass flow
                              Nandle const AirMassFlow,        // actual mass flow for capacity and SHR calculation
                              Nandle const CBF,                // coil bypass factor
                              Nandle const TotCapNom1,         // nominal total capacity at low speed [W]
                              int const CCapFTemp1,            // capacity modifier curve index, function of entering wetbulb at low speed
                              int const CCapAirFFlow1,         // capacity modifier curve, function of actual air flow vs rated flow at low speed
                              int const CCapWaterFFlow1,       // capacity modifier curve, function of actual water flow vs rated flow at low speed
                              Nandle const TotCapNom2,         // nominal total capacity at high speed [W]
                              int const CCapFTemp2,            // capacity modifier curve index, function of entering wetbulb at high speed
                              int const CCapAirFFlow2,         // capacity modifier curve, function of actual air flow vs rated flow at high speed
                              int const CCapWaterFFlow2,       // capacity modifier curve, function of actual water flow vs rated flow at high speed
                              Nandle &TotCap1,                 // total capacity at the given conditions [W] at low speed
                              Nandle &TotCap2,                 // total capacity at the given conditions [W] at high speed
                              Nandle &TotCapSpeed,             // integrated total capacity corresponding to the speed ratio
                              Nandle &SHR,                     // sensible heat ratio at the given conditions
                              Nandle const CondInletTemp,      // Condenser inlet temperature [C]
                              Nandle const Pressure,           // air pressure [Pa]
                              Nandle const SpeedRatio,         // from 0.0 to 1.0
                              int const NumSpeeds,             // number of speeds for input
                              Nandle &TotCapModFac             // capacity modification factor, func of temp and func of flow
    );

    void CalcVarSpeedHPWH(int const DXCoilNum,        // the number of the DX coil to be simulated
                          Nandle &RuntimeFrac,        // Runtime Fraction of compressor or percent on time (on-time/cycle time)
                          Nandle const PartLoadRatio, // sensible water heating load / full load sensible water heating capacity
                          Nandle const SpeedRatio,    // SpeedRatio varies between 1.0 (higher speed) and 0.0 (lower speed)
                          int const SpeedNum,         // Speed number, high bound capacity
                          int const CyclingScheme     // Continuous fan OR cycling compressor
    );

    Nandle getVarSpeedPartLoadRatio(int const DXCoilNum); // the number of the DX coil to mined for current PLR

    void setVarSpeedHPWHFanTypeNum(int const dXCoilNum, int const fanTypeNum);

    void setVarSpeedHPWHFanIndex(int const dXCoilNum, int const fanIndex);

    void setVarSpeedFanInfo(int const dXCoilNum, std::string const fanName, int const fanIndex, int const fanTypeNum);

} // namespace VariableSpeedCoils

} // namespace EnergyPlus

#endif
