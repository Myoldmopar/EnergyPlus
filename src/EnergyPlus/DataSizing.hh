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

#ifndef DataSizing_hh_INCLUDED
#define DataSizing_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>
#include <ObjexxFCL/Array2D.hh>
#include <ObjexxFCL/gio_Fmt.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

namespace DataSizing {

    // Using/Aliasing

    // Data
    // -only module should be available to other modules and routines.
    // Thus, all variables in this module must be PUBLIC.

    // MODULE PARAMETER DEFINITIONS:

    // parameters for outside air flow method
    extern int const NumOAFlowMethods;

    extern int const OAFlowNone;
    extern int const OAFlowPPer;
    extern int const OAFlow;
    extern int const OAFlowPerArea;
    extern int const OAFlowACH;
    extern int const OAFlowSum;
    extern int const OAFlowMax;

    extern Array1D_string const cOAFlowMethodTypes;

    // parameters for outside air
    extern int const AllOA;
    extern int const MinOA;

    // parameters for loop fluid type
    extern int const HeatingLoop;
    extern int const CoolingLoop;
    extern int const CondenserLoop;
    extern int const SteamLoop;

    // parameters for sizing
    extern int const NonCoincident;
    extern int const Coincident;

    // parameters for Cooling Peak Load Type
    extern int const SensibleCoolingLoad;
    extern int const TotalCoolingLoad;

    // parameters for Central Cooling Capacity Control Method
    extern int const VAV;
    extern int const Bypass;
    extern int const VT;
    extern int const OnOff;

    // paramters for supply air flow rate method
    extern int const SupplyAirTemperature;
    extern int const TemperatureDifference;

    // paramters for sizing
    extern int const FromDDCalc;
    extern int const InpDesAirFlow;
    extern int const DesAirFlowWithLim;

    // parameters for DOAs control method
    extern int const DOANeutralSup;
    extern int const DOANeutralDehumSup;
    extern int const DOACoolSup;

    // parameters for Type of Load to Size On
    extern int const Sensible;
    extern int const Latent;
    extern int const Total;
    extern int const Ventilation;

    // parameter for autosize
    extern Nandle const AutoSize;

    // parameter for (time-of-peak) sizing format
    extern ObjexxFCL::gio::Fmt PeakHrMinFmt;

    // Zone Outdoor Air Method
    extern int const ZOAM_FlowPerPerson; // set the outdoor air flow rate based on number of people in the zone
    extern int const ZOAM_FlowPerZone;   // sum the outdoor air flow rate per zone based on user input
    extern int const ZOAM_FlowPerArea;   // sum the outdoor air flow rate based on zone area
    extern int const ZOAM_FlowPerACH;    // sum the outdoor air flow rate based on number of air changes for the zone
    extern int const ZOAM_Sum;           // sum the outdoor air flow rate of the people component and the space floor area component
    extern int const ZOAM_Max;           // use the maximum of the outdoor air flow rate of the people component and
    // the space floor area component
    extern int const ZOAM_IAQP;                      // Use ASHRAE Standard 62.1-2007 IAQP to calculate the zone level outdoor air flow rates
    extern int const ZOAM_ProportionalControlSchOcc; // Use ASHRAE Standard 62.1-2004 or Trane Engineer's newsletter (volume 34-5)
                                                     // to calculate the zone level outdoor air flow rates based on scheduled occupancy
    extern int const ZOAM_ProportionalControlDesOcc; // Use ASHRAE Standard 62.1-2004 or Trane Engineer's newsletter (volume 34-5)
                                                     // to calculate the zone level outdoor air flow rates based on design occupancy

    // System Outdoor Air Method
    extern int const SOAM_ZoneSum; // Sum the outdoor air flow rates of all zones
    extern int const SOAM_VRP;     // Use ASHRAE Standard 62.1-2007 to calculate the system level outdoor air flow rates
    //  considering the zone air distribution effectiveness and the system ventilation efficiency
    extern int const SOAM_IAQP; // Use ASHRAE Standard 62.1-2007 IAQP to calculate the system level outdoor air flow rates
    // based on the CO2 setpoint
    extern int const SOAM_ProportionalControlSchOcc; // Use ASHRAE Standard 62.1-2004 or Trane Engineer's newsletter (volume 34-5)
    // to calculate the system level outdoor air flow rates based on scheduled occupancy
    extern int const SOAM_ProportionalControlDesOcc; // Use ASHRAE Standard 62.1-2004 or Trane Engineer's newsletter (volume 34-5)
    // to calculate the system level outdoor air flow rates based on design occupancy
    extern int const SOAM_IAQPGC; // Use ASHRAE Standard 62.1-2004 IAQP to calculate the system level outdoor air flow rates
    // based on the generic contaminant setpoint
    extern int const SOAM_IAQPCOM; // Take the maximum outdoor air rate from both CO2 and generic contaminant controls
    // based on the generic contaminant setpoint
    extern int const SOAM_ProportionalControlDesOARate; // Calculate the system level outdoor air flow rates based on design OA rate

    // Zone HVAC Equipment Supply Air Sizing Option
    extern int const None;
    extern int const SupplyAirFlowRate;
    extern int const FlowPerFloorArea;
    extern int const FractionOfAutosizedCoolingAirflow;
    extern int const FractionOfAutosizedHeatingAirflow;
    extern int const FlowPerCoolingCapacity;
    extern int const FlowPerHeatingCapacity;
    extern int const CoolingDesignCapacity;
    extern int const HeatingDesignCapacity;
    extern int const CapacityPerFloorArea;
    extern int const FractionOfAutosizedCoolingCapacity;
    extern int const FractionOfAutosizedHeatingCapacity;

    // Plant Coincident sizing factor options
    extern int const NoSizingFactorMode;
    extern int const GlobalHeatingSizingFactorMode;
    extern int const GlobalCoolingSizingFactorMode;
    extern int const LoopComponentSizingFactorMode;

    enum class zoneFanPlacement
    {
        zoneFanPlaceNotSet,
        zoneBlowThru,
        zoneDrawThru
    };

    // DERIVED TYPE DEFINITIONS:

    // INTERFACE BLOCK SPECIFICATIONS
    // na

    // MODULE VARIABLE DECLARATIONS:

    //  days; includes effects of user multiplier
    //  and user set flows)
    //  of user input multiplier and flows
    //  all design days, calculated only)
    //  using user input system flow rates.
    //  before applying user input sys flow rates.

    extern int NumOARequirements;                     // Number of OA Requirements objects
    extern int NumZoneAirDistribution;                // Number of zone air distribution objects
    extern int NumZoneSizingInput;                    // Number of Zone Sizing objects
    extern int NumSysSizInput;                        // Number of System Sizing objects
    extern int NumPltSizInput;                        // Number of Plant Sizing objects
    extern int CurSysNum;                             // Current Air System index (0 if not in air loop)
    extern int CurOASysNum;                           // Current outside air system index (0 if not in OA Sys)
    extern int CurZoneEqNum;                          // Current Zone Equipment index (0 if not simulating ZoneEq)
    extern int CurTermUnitSizingNum;                  // Current terminal unit sizing index for TermUnitSizing and TermUnitFinalZoneSizing
    extern int CurBranchNum;                          // Index of branch being simulated (or 0 if not air loop)
    extern int CurDuctType;                           // Duct type of current branch
    extern int CurLoopNum;                            // the current plant loop index
    extern int CurCondLoopNum;                        // the current condenser loop number
    extern int CurEnvirNumSimDay;                     // current environment number for day simulated
    extern int CurOverallSimDay;                      // current day of simulation
    extern int NumTimeStepsInAvg;                     // number of time steps in the averaging window for the design flow and load sequences
    extern int SaveNumPlantComps;                     // Number of components using water as an energy source or sink (e.g. water coils)
    extern int DataTotCapCurveIndex;                  // index to total capacity as a function of temperature curve
    extern Nandle DataTotCapCurveValue;               // value of total capacity as a function of temperature curve for CoilVRF_FluidTCtrl_*
    extern int DataPltSizCoolNum;                     // index to cooling plant sizing data
    extern int DataPltSizHeatNum;                     // index to heating plant sizing data
    extern int DataWaterLoopNum;                      // index to plant water loop
    extern int DataCoilNum;                           // index to coil object
    extern int DataFanOpMode;                         // fan operating mode (ContFanCycCoil or CycFanCycCoil)
    extern bool DataCoilIsSuppHeater;                 // TRUE if heating coil used as supplemental heater
    extern bool DataIsDXCoil;                         // TRUE if direct-expansion coil
    extern bool DataAutosizable;                      // TRUE if component is autosizable
    extern bool DataEMSOverrideON;                    // boolean determines if user relies on EMS to override autosizing
    extern bool TermUnitSingDuct;                     // TRUE if a non-induction single duct terminal unit
    extern bool TermUnitPIU;                          // TRUE if a powered induction terminal unit
    extern bool TermUnitIU;                           // TRUE if an unpowered induction terminal unit
    extern bool ZoneEqFanCoil;                        // TRUE if a 4 pipe fan coil unit is being simulated
    extern bool ZoneEqOutdoorAirUnit;                 // TRUE if an OutdoorAirUnit is being simulated
    extern bool ZoneEqUnitHeater;                     // TRUE if a unit heater is being simulated
    extern bool ZoneEqUnitVent;                       // TRUE if a unit ventilator is being simulated
    extern bool ZoneEqVentedSlab;                     // TRUE if a ventilated slab is being simulated
    extern bool ZoneEqDXCoil;                         // TRUE if a ZoneHVAC DX coil is being simulated
    extern bool ZoneEqUnitarySys;                     // TRUE if a zone UnitarySystem is being simulated
    extern bool ZoneCoolingOnlyFan;                   // TRUE if a ZoneHVAC DX cooling coil is only coil in parent
    extern bool ZoneHeatingOnlyFan;                   // TRUE if zone unit only does heating and contains a fam (such as Unit Heater)
    extern bool SysSizingRunDone;                     // True if a system sizing run is successfully completed.
    extern bool ZoneSizingRunDone;                    // True if a zone sizing run has been successfully completed.
    extern bool DataErrorsFound;                      // used for simulation termination when errors are found
    extern bool DataAutosizable;                      // TRUE if component is autosizable
    extern bool DataEMSOverrideON;                    // boolean determines if user relies on EMS to override autosizing
    extern bool DataScalableSizingON;                 // boolean determines scalable zone flow sizing is specified
    extern bool DataScalableCapSizingON;              // boolean determines scalable zone capacity sizing is specified
    extern bool DataSysScalableFlowSizingON;          // boolean determines scalable system flow sizing is specified
    extern bool DataSysScalableCapSizingON;           // boolean determines scalable system capacity sizing is specified
    extern Nandle DataCoilSizingAirInTemp;            // saves sizing data for use in coil object reporting
    extern Nandle DataCoilSizingAirInHumRat;          // saves sizing data for use in coil object reporting
    extern Nandle DataCoilSizingAirOutTemp;           // saves sizing data for use in coil object reporting
    extern Nandle DataCoilSizingAirOutHumRat;         // saves sizing data for use in coil object reporting
    extern Nandle DataCoilSizingFanCoolLoad;          // saves sizing data for use in coil object reporting
    extern Nandle DataCoilSizingCapFT;                // saves sizing data for use in coil object reporting
    extern bool DataDesAccountForFanHeat;             // include fan heat when true
    extern Nandle DataDesInletWaterTemp;              // coil inlet water temperture used for warning messages
    extern Nandle DataDesInletAirHumRat;              // coil inlet air humidity ratio used for warning messages
    extern Nandle DataDesInletAirTemp;                // coil inlet air temperature used for warning messages
    extern Nandle DataDesOutletAirTemp;               // coil outlet air temperature used for sizing
    extern Nandle DataDesOutletAirHumRat;             // coil air outlet humidity ratio used in sizing calculations [kg water / kg dry air]
    extern Nandle DataCoolCoilCap;                    // cooling coil capacity used for sizing with scalable inputs
    extern Nandle DataFlowUsedForSizing;              // air flow rate used for sizing with scalable inputs [m3/s]
    extern Nandle DataAirFlowUsedForSizing;           // air flow rate used for sizing with scalable inputs [m3/s]
    extern Nandle DataWaterFlowUsedForSizing;         // water flow rate used for sizing with scalable inputs [m3/s]
    extern Nandle DataCapacityUsedForSizing;          // capacity used for sizing with scalable inputs [W]
    extern Nandle DataDesignCoilCapacity;             // calculated capacity of coil at end of UA calculation
    extern Nandle DataHeatSizeRatio;                  // heating coil size as a ratio of cooling coil capacity
    extern Nandle DataEMSOverride;                    // value of EMS variable used to override autosizing
    extern Nandle DataBypassFrac;                     // value of bypass fraction for Coil:Cooling:DX:TwoStageWithHumidityControlMode coils
    extern Nandle DataFracOfAutosizedCoolingAirflow;  // fraction of design cooling supply air flow rate
    extern Nandle DataFracOfAutosizedHeatingAirflow;  // fraction of design heating supply air flow rate
    extern Nandle DataFlowPerCoolingCapacity;         // cooling supply air flow per unit cooling capacity
    extern Nandle DataFlowPerHeatingCapacity;         // heating supply air flow per unit heating capacity
    extern Nandle DataFracOfAutosizedCoolingCapacity; // fraction of autosized cooling capacity
    extern Nandle DataFracOfAutosizedHeatingCapacity; // fraction of autosized heating capacit
    extern Nandle DataAutosizedCoolingCapacity;       // Autosized cooling capacity used for multiplying flow per capacity to get flow rate
    extern Nandle DataAutosizedHeatingCapacity;       // Autosized heating capacit used for multiplying flow per capacity to get flow rate
    extern Nandle DataConstantUsedForSizing;          // base value used for sizing inputs that are ratios of other inputs
    extern Nandle DataFractionUsedForSizing;          // fractional value of base value used for sizing inputs that are ratios of other inputs
    extern Nandle DataNonZoneNonAirloopValue;         // used when equipment is not located in a zone or airloop
    extern int DataZoneUsedForSizing;                 // pointer to control zone for air loop equipment
    extern int DataZoneNumber;                        // a pointer to a zone served by zoneHVAC equipment
    extern int NumZoneHVACSizing;                     // Number of design specification zone HVAC sizing objects
    extern int NumAirTerminalSizingSpec;              // Number of design specification air terminal sizing objects
    extern int NumAirTerminalUnits;                   // Number of air terminal units (same as total number of zone inlet nodes)
    extern bool TermUnitSingDuct;                     // TRUE if a non-induction single duct terminal unit
    extern bool TermUnitPIU;                          // TRUE if a powered induction terminal unit
    extern bool TermUnitIU;                           // TRUE if an unpowered induction terminal unit
    extern bool ZoneEqFanCoil;                        // TRUE if a 4 pipe fan coil unit is being simulated
    extern bool ZoneEqUnitVent;                       // TRUE if a unit ventilator unit is being simulated
    extern bool ZoneEqDXCoil;                         // TRUE if a ZoneHVAC DX coil is being simulated
    extern bool ZoneCoolingOnlyFan;                   // TRUE if a ZoneHVAC DX cooling coil is only coil in parent
    extern bool ZoneHeatingOnlyFan;                   // TRUE if zone unit only does heating and contains a fam (such as Unit Heater)
    extern bool SysSizingRunDone;                     // True if a system sizing run is successfully completed.
    extern bool ZoneSizingRunDone;                    // True if a zone sizing run has been successfully completed.
    extern Nandle AutoVsHardSizingThreshold;          // criteria threshold used to determine if user hard size and autosize disagree 10%
    extern Nandle AutoVsHardSizingDeltaTempThreshold; // temperature criteria threshold for autosize versus hard size [C]
    extern Nandle DXCoolCap;                          // The ARI cooling capacity of a DX unit.
    extern Nandle UnitaryHeatCap;                     // the heating capacity of a unitary system
    extern Nandle SuppHeatCap;                        // the heating capacity of the supplemental heater in a unitary system
    extern Nandle GlobalHeatSizingFactor;             // the global heating sizing ratio
    extern Nandle GlobalCoolSizingFactor;             // the global cooling sizing ratio
    extern Array1D<Nandle> ZoneSizThermSetPtHi;       // highest zone thermostat setpoint during zone sizing calcs
    extern Array1D<Nandle> ZoneSizThermSetPtLo;       // lowest zone thermostat setpoint during zone sizing calcs
    extern Array1D_string CoolPeakDateHrMin;          // date:hr:min of cooling peak
    extern Array1D_string HeatPeakDateHrMin;          // date:hr:min of heating peak
    extern char SizingFileColSep;                     // Character to separate columns in sizing outputs
    extern int DataDesicDehumNum;                     // index to desiccant dehumidifier
    extern bool DataDesicRegCoil;                     // TRUE if heating coil desiccant regeneration coil
    extern bool HRFlowSizingFlag;                     // True, if it is a heat recovery heat exchanger flow sizing
    extern Nandle DataWaterCoilSizCoolDeltaT;         // used for sizing cooling coil water design flow rate
    extern Nandle DataWaterCoilSizHeatDeltaT;         // used for sizing heating coil water design flow rate
    extern bool DataNomCapInpMeth;                    // True if heating coil is sized by CoilPerfInpMeth == NomCap
    extern int DataFanEnumType;                       // Fan type used during sizing
    extern int DataFanIndex;                          // Fan index used during sizing
    extern zoneFanPlacement DataFanPlacement;         // identifies location of fan wrt coil

    // Types

    struct ZoneSizingInputData
    {
        // Members
        std::string ZoneName;  // name of a zone
        int ZoneNum;           // index of the zone
        int ZnCoolDgnSAMethod; // choice of how to get zone cooling design air temperature;
        //  1 = specify supply air temperature,
        //  2 = calculate from the temperature difference
        int ZnHeatDgnSAMethod; // choice of how to get zone heating design air temperature;
        //  1 = specify supply air temperature,
        //  2 = calculate from the temperature difference
        Nandle CoolDesTemp;              // zone design cooling supply air temperature [C]
        Nandle HeatDesTemp;              // zone design heating supply air temperature [C]
        Nandle CoolDesTempDiff;          // zone design cooling supply air temperature difference [deltaC]
        Nandle HeatDesTempDiff;          // zone design heating supply air temperature difference [deltaC]
        Nandle CoolDesHumRat;            // zone design cooling supply air humidity ratio [kgWater/kgDryAir]
        Nandle HeatDesHumRat;            // zone design heating supply air humidity ratio [kgWater/kgDryAir]
        std::string DesignSpecOAObjName; // name of the design specification outdoor air object
        int OADesMethod;                 // choice of how to calculate minimum outside air;
        //  1 = m3/s per person; 2 = m3/s per zone; 3 = m3/s per zone area;
        //  4 = sum of flow from 3 OA input fields;
        //  5 = max of flow from 3 OA input fields
        Nandle DesOAFlowPPer;    // design outside air flow per person in zone [m3/s]
        Nandle DesOAFlowPerArea; // design outside air flow per zone area [m3/s / m2]
        Nandle DesOAFlow;        // design outside air flow for the zone [m3/s]
        int CoolAirDesMethod;    // choice of how to get zone cooling design air flow rates;
        //  1 = calc from des day simulation; 2 = m3/s per zone, user input
        //  3 = apply limits to air flow rate from DD calc
        Nandle DesCoolAirFlow;           // design zone supply air flow rate [m3/s]
        Nandle DesCoolMinAirFlowPerArea; // design cooling minimum air flow rate per zone area [m3/s / m2]
        Nandle DesCoolMinAirFlow;        // design cooling minimum air flow rate [m3/s]
        Nandle DesCoolMinAirFlowFrac;    // design cooling minimum air flow rate fraction
        //  (of the cooling design air flow rate)
        int HeatAirDesMethod; // choice of how to get zone heating design air flow rates;
        //  1 = calc from des day simulation; 2 = m3/s per zone, user input
        //  3 = apply limits to air flow rate from DD calc
        Nandle DesHeatAirFlow;           // design zone heating supply air flow rate [m3/s]
        Nandle DesHeatMaxAirFlowPerArea; // design heating maximum air flow rate per zone area [m3/s / m2]
        Nandle DesHeatMaxAirFlow;        // design heating maximum air flow rate [m3/s]
        Nandle DesHeatMaxAirFlowFrac;    // design heating maximum air flow rate fraction
        //  (of the cooling design air flow rate)
        Nandle HeatSizingFactor; // the zone heating sizing ratio
        Nandle CoolSizingFactor; // the zone cooling sizing ratio
        Nandle ZoneADEffCooling;
        Nandle ZoneADEffHeating;
        std::string ZoneAirDistEffObjName; // name of the zone air distribution effectiveness object name
        int ZoneAirDistributionIndex;      // index to the zone air distribution object
        int ZoneDesignSpecOAIndex;         // index to the zone design spec OA object
        Nandle ZoneSecondaryRecirculation; // the zone secondary air recirculation fraction
        Nandle ZoneVentilationEff;         // zone ventilation efficiency
        bool AccountForDOAS;               // False: do nothing; True: calculate the effect of a DOA system on the zone sizing arrays
        int DOASControlStrategy;           // 1=supply neutral ventilation air; 2=supply neutral dehumidified ventilation air;
        // 3=supply cold ventilation air
        Nandle DOASLowSetpoint;  // Dedicated Outside Air Low Setpoint for Design [C]
        Nandle DOASHighSetpoint; // Dedicated Outside Air High Setpoint for Design [C]

        // Default Constructor
        ZoneSizingInputData()
            : ZoneNum(0), ZnCoolDgnSAMethod(0), ZnHeatDgnSAMethod(0), CoolDesTemp(0.0), HeatDesTemp(0.0), CoolDesTempDiff(0.0), HeatDesTempDiff(0.0),
              CoolDesHumRat(0.0), HeatDesHumRat(0.0), OADesMethod(0), DesOAFlowPPer(0.0), DesOAFlowPerArea(0.0), DesOAFlow(0.0), CoolAirDesMethod(0),
              DesCoolAirFlow(0.0), DesCoolMinAirFlowPerArea(0.0), DesCoolMinAirFlow(0.0), DesCoolMinAirFlowFrac(0.0), HeatAirDesMethod(0),
              DesHeatAirFlow(0.0), DesHeatMaxAirFlowPerArea(0.0), DesHeatMaxAirFlow(0.0), DesHeatMaxAirFlowFrac(0.0), HeatSizingFactor(0.0),
              CoolSizingFactor(0.0), ZoneADEffCooling(1.0), ZoneADEffHeating(1.0), ZoneAirDistributionIndex(0), ZoneDesignSpecOAIndex(0),
              ZoneSecondaryRecirculation(0.0), ZoneVentilationEff(0.0), AccountForDOAS(false), DOASControlStrategy(0), DOASLowSetpoint(0.0),
              DOASHighSetpoint(0.0)
        {
        }
    };

    struct ZoneSizingData
    {
        // Members
        std::string ZoneName;   // name of a zone
        std::string ADUName;    // Terminal Unit Name (air distribution unit or direct air unit) - only assigned for TermUnitFinalZoneSizing
        std::string CoolDesDay; // name of a cooling design day
        std::string HeatDesDay; // name of a heating design day
        int ZnCoolDgnSAMethod;  // choice of how to get zone cooling design air temperature;
        //  1 = specify supply air temperature,
        //  2 = calculate from the temperature difference
        int ZnHeatDgnSAMethod; // choice of how to get zone heating design air temperature;
        //  1 = specify supply air temperature,
        //  2 = calculate from the temperature difference
        Nandle CoolDesTemp;        // zone design cooling supply air temperature [C]
        Nandle HeatDesTemp;        // zone design heating supply air temperature [C]
        Nandle CoolDesTempDiff;    // zone design cooling supply air temperature difference [deltaC]
        Nandle HeatDesTempDiff;    // zone design heating supply air temperature difference [deltaC]
        Nandle CoolDesHumRat;      // zone design cooling supply air humidity ratio [kgWater/kgDryAir]
        Nandle HeatDesHumRat;      // zone design heating supply air humidity ratio [kgWater/kgDryAir]
        int ZoneDesignSpecOAIndex; // index to DesignSpecification:OutdoorAir object
        int OADesMethod;           // choice of how to calculate minimum outside air;
        //  1 = m3/s per person; 2 = m3/s per zone; 3 = m3/s per zone area;
        //  4 = sum of flow from 3 OA input fields;
        //  5 = max of flow from 3 OA input fields
        Nandle DesOAFlowPPer;    // design outside air flow per person in zone [m3/s]
        Nandle DesOAFlowPerArea; // design outside air flow per zone area [m3/s / m2]
        Nandle DesOAFlow;        // design outside air flow for the zone [m3/s]
        int CoolAirDesMethod;    // choice of how to get zone cooling design air flow rates;
        //  1 = calc from des day simulation; 2 = m3/s per zone, user input
        //  3 = apply limits to air flow rate from DD calc
        Nandle InpDesCoolAirFlow;        // design zone supply air flow rate [m3/s]
        Nandle DesCoolMinAirFlowPerArea; // design cooling minimum air flow rate per zone area [m3/s / m2]
        Nandle DesCoolMinAirFlow;        // design cooling minimum air flow rate [m3/s]
        Nandle DesCoolMinAirFlowFrac;    // design cooling minimum air flow rate fraction
        //  (of the cooling design air flow rate)
        int HeatAirDesMethod; // choice of how to get zone heating design air flow rates;
        //  1 = calc from des day simulation; 2 = m3/s per zone, user input
        //  3 = apply limits to air flow rate from DD calc
        Nandle InpDesHeatAirFlow;        // design zone heating supply air flow rate [m3/s]
        Nandle DesHeatMaxAirFlowPerArea; // design heating maximum air flow rate per zone area [m3/s / m2]
        Nandle DesHeatMaxAirFlow;        // design heating maximum air flow rate [m3/s]
        Nandle DesHeatMaxAirFlowFrac;    // design heating maximum air flow rate fraction
        //  (of the cooling design air flow rate)
        Nandle HeatSizingFactor; // the zone heating sizing ratio
        Nandle CoolSizingFactor; // the zone cooling sizing ratio
        bool AccountForDOAS;     // False: do nothing; True: calculate the effect of a DOA system on the zone sizing arrays
        int DOASControlStrategy; // 1=supply neutral ventilation air; 2=supply neutral dehumidified ventilation air;
        // 3=supply cold ventilation air
        Nandle DOASLowSetpoint;         // Dedicated Outside Air Low Setpoint for Design [C]
        Nandle DOASHighSetpoint;        // Dedicated Outside Air High Setpoint for Design [C]
        int ActualZoneNum;              // index into the Zone data array (in DataHeatBalance)
        Nandle DesHeatMassFlow;         // zone design heating air mass flow rate [kg/s]
        Nandle DesHeatMassFlowNoOA;     // zone design heating air mass flow rate without applying MinOA as a limit [kg/s]
        Nandle DesHeatOAFlowFrac;       // zone design heating OA air volume fraction [-]
        bool EMSOverrideDesHeatMassOn;  // true if EMS is acting on this structure
        Nandle EMSValueDesHeatMassFlow; // Value EMS directing to use for Design Heating air mass flow [kg/s]
        Nandle DesCoolMassFlow;         // zone design cooling air mass flow rate [kg/s]
        Nandle DesCoolMassFlowNoOA;     // zone design cooling air mass flow rate without applying MinOA as a limit [kg/s]
        Nandle DesCoolOAFlowFrac;       // zone design cooling OA air volume fraction [-]
        bool EMSOverrideDesCoolMassOn;  // true if EMS is acting on this structure
        Nandle EMSValueDesCoolMassFlow; // Value EMS directing to use for Design Cooling air mass flow [kg/s]
        Nandle DesHeatLoad;             // zone design heating load including sizing factor and scaled to match airflow sizing [W]
        Nandle NonAirSysDesHeatLoad;    // base zone design heating load including sizing factor [W]
        bool EMSOverrideDesHeatLoadOn;  // true if EMS is acting on this structure
        Nandle EMSValueDesHeatLoad;     // Value EMS directing to use for zone design heating load  [W]
        Nandle DesCoolLoad;             // zone design cooling load including sizing factor and scaled to match airflow sizing [W]
        Nandle NonAirSysDesCoolLoad;    // base zone design cooling load including sizing factor [W]
        bool EMSOverrideDesCoolLoadOn;  // true if EMS is acting on this structure
        Nandle EMSValueDesCoolLoad;     // Value EMS directing to use for zone design cooling load  [W]
        Nandle DesHeatDens;             // zone design heating air density [kg/m3]
        Nandle DesCoolDens;             // zone design cooling air density [kg/m3]
        Nandle DesHeatVolFlow;          // zone design heating air volume flow rate including sizing factor and scaled to match airflow sizing [m3/s]
        Nandle DesHeatVolFlowNoOA;      // zone design heating air volume flow rate including sizing factor and scaled to match airflow sizing without
                                        // MinOA limit [m3/s]
        Nandle NonAirSysDesHeatVolFlow; // base zone design heating air volume flow rate including sizing factor [m3/s]
        bool EMSOverrideDesHeatVolOn;   // true if EMS is acting on this structure
        Nandle EMSValueDesHeatVolFlow;  // Value EMS directing to use for Design Heating air volume flow [m3/s]
        Nandle DesCoolVolFlow;          // zone design cooling air volume flow rate [m3/s]
        Nandle DesCoolVolFlowNoOA;      // zone design cooling air volume flow rate without applying MinOA as a limit [m3/s]
        Nandle NonAirSysDesCoolVolFlow; // base zone design cooling air volume flow rate including sizing factor [m3/s]
        bool EMSOverrideDesCoolVolOn;   // true if EMS is acting on this structure
        Nandle EMSValueDesCoolVolFlow;  // Value EMS directing to use for Design cooling air volume flow [m3/s]
        Nandle DesHeatVolFlowMax;       // zone design heating maximum air volume flow rate [m3/s]
        Nandle DesCoolVolFlowMin;       // zone design cooling minimum air volume flow rate [m3/s]
        Nandle DesHeatCoilInTemp;       // zone heating coil design air inlet temperature [C]
        Nandle DesCoolCoilInTemp;       // zone cooling coil design air inlet temperature [C]
        Nandle DesHeatCoilInHumRat;     // zone heating coil design air inlet humidity ratio [kg/kg]
        Nandle DesCoolCoilInHumRat;     // zone cooling coil design air inlet humidity ratio [kg/kg]
        Nandle DesHeatCoilInTempTU;     // zone heating coil design air inlet temperature (supply air)([C]
        Nandle DesCoolCoilInTempTU;     // zone cooling coil design air inlet temperature (supply air)[C]
        Nandle DesHeatCoilInHumRatTU;   // zone heating coil design air inlet humidity ratio
        //  (supply air) [kg/kg]
        Nandle DesCoolCoilInHumRatTU; // zone cooling coil design air inlet humidity ratio
        //  (supply air) [kg/kg]
        Nandle HeatMassFlow;          // current zone heating air mass flow rate (HVAC time step)
        Nandle CoolMassFlow;          // current zone cooling air mass flow rate (HVAC time step)
        Nandle HeatLoad;              // current zone heating load (HVAC time step)
        Nandle CoolLoad;              // current zone heating load (HVAC time step)
        Nandle HeatZoneTemp;          // current zone temperature (heating, time step)
        Nandle HeatOutTemp;           // current outdoor temperature (heating, time step)
        Nandle HeatZoneRetTemp;       // current zone return temperature (heating, time step)
        Nandle HeatTstatTemp;         // current zone thermostat temperature (heating, time step)
        Nandle CoolZoneTemp;          // current zone temperature (cooling, time step)
        Nandle CoolOutTemp;           // current Outdoor temperature (cooling, time step)
        Nandle CoolZoneRetTemp;       // current zone return temperature (cooling, time step)
        Nandle CoolTstatTemp;         // current zone thermostat temperature (cooling, time step)
        Nandle HeatZoneHumRat;        // current zone humidity ratio (heating, time step)
        Nandle CoolZoneHumRat;        // current zone humidity ratio (cooling, time step)
        Nandle HeatOutHumRat;         // current outdoor humidity ratio (heating, time step)
        Nandle CoolOutHumRat;         // current outdoor humidity ratio (cooling, time step)
        Nandle ZoneTempAtHeatPeak;    // zone temp at max heating [C]
        Nandle ZoneRetTempAtHeatPeak; // zone return temp at max heating [C]
        Nandle OutTempAtHeatPeak;     // outdoor temperature at max heating [C]
        Nandle ZoneTempAtCoolPeak;    // zone temp at max cooling [C]
        Nandle ZoneRetTempAtCoolPeak; // zone return temp at max cooling [C]
        Nandle OutTempAtCoolPeak;     // outdoor temperature at max cooling [C]
        Nandle ZoneHumRatAtHeatPeak;  // zone humidity ratio at max heating [kg/kg]
        Nandle ZoneHumRatAtCoolPeak;  // zone humidity ratio at max cooling [kg/kg]
        Nandle OutHumRatAtHeatPeak;   // outdoor humidity at max heating [kg/kg]
        Nandle OutHumRatAtCoolPeak;   // outdoor humidity at max cooling [kg/kg]
        int TimeStepNumAtHeatMax;     // time step number (in day) at Heating peak
        int TimeStepNumAtCoolMax;     // time step number (in day) at cooling peak
        int HeatDDNum;                // design day index of design day causing heating peak
        int CoolDDNum;                // design day index of design day causing cooling peak
        std::string cHeatDDDate;      // date of design day causing heating peak
        std::string cCoolDDDate;      // date of design day causing cooling peak
        Nandle MinOA;                 // design minimum outside air in m3/s
        Nandle DesCoolMinAirFlow2;    // design cooling minimum air flow rate [m3/s] derived from
        //  DesCoolMinAirFlowPerArea
        Nandle DesHeatMaxAirFlow2; // design heating maximum air flow rate [m3/s] derived from
        //  DesHeatMaxAirFlowPerArea
        Array1D<Nandle> HeatFlowSeq;        // daily sequence of zone heating air mass flow rate (zone time step) [kg/s]
        Array1D<Nandle> HeatFlowSeqNoOA;    // daily sequence of zone heating air mass flow rate (zone time step) without MinOA limit [kg/s]
        Array1D<Nandle> CoolFlowSeq;        // daily sequence of zone cooling air mass flow rate (zone time step) [kg/s]
        Array1D<Nandle> CoolFlowSeqNoOA;    // daily sequence of zone cooling air mass flow rate (zone time step) without MinOA limit [kg/s]
        Array1D<Nandle> HeatLoadSeq;        // daily sequence of zone heating load (zone time step)
        Array1D<Nandle> CoolLoadSeq;        // daily sequence of zone cooling load (zone time step)
        Array1D<Nandle> HeatZoneTempSeq;    // daily sequence of zone temperatures (heating, zone time step)
        Array1D<Nandle> HeatOutTempSeq;     // daily sequence of outdoor temperatures (heating, zone time step)
        Array1D<Nandle> HeatZoneRetTempSeq; // daily sequence of zone return temperatures (heating,
        //  zone time step)
        Array1D<Nandle> HeatTstatTempSeq;   // daily sequence of zone thermostat temperatures (heating, zone time step)
        Array1D<Nandle> DesHeatSetPtSeq;    // daily sequence of indoor set point temperatures (zone time step)
        Array1D<Nandle> CoolZoneTempSeq;    // daily sequence of zone temperatures (cooling, zone time step)
        Array1D<Nandle> CoolOutTempSeq;     // daily sequence of outdoor temperatures (cooling, zone time step)
        Array1D<Nandle> CoolZoneRetTempSeq; // daily sequence of zone return temperatures (cooling,
        //  zone time step)
        Array1D<Nandle> CoolTstatTempSeq;  // daily sequence of zone thermostat temperatures (cooling, zone time step)
        Array1D<Nandle> DesCoolSetPtSeq;   // daily sequence of indoor set point temperatures (zone time step)
        Array1D<Nandle> HeatZoneHumRatSeq; // daily sequence of zone humidity ratios (heating, zone time step)
        Array1D<Nandle> CoolZoneHumRatSeq; // daily sequence of zone humidity ratios (cooling, zone time step)
        Array1D<Nandle> HeatOutHumRatSeq;  // daily sequence of outdoor humidity ratios (heating, zone time step)
        Array1D<Nandle> CoolOutHumRatSeq;  // daily sequence of outdoor humidity ratios (cooling, zone time step)
        Nandle ZoneADEffCooling;           // the zone air distribution effectiveness in cooling mode
        Nandle ZoneADEffHeating;           // the zone air distribution effectiveness in heating mode
        Nandle ZoneSecondaryRecirculation; // the zone secondary air recirculation fraction
        Nandle ZoneVentilationEff;         // zone ventilation efficiency
        Nandle ZonePrimaryAirFraction;     // the zone primary air fraction for cooling based calculations
        Nandle ZonePrimaryAirFractionHtg;  // the zone primary air fraction for heating based calculations
        Nandle ZoneOAFracCooling;          // OA fraction in cooling mode
        Nandle ZoneOAFracHeating;          // OA fraction in heating mode
        Nandle TotalOAFromPeople;          // Zone OA required due to people
        Nandle TotalOAFromArea;            // Zone OA required based on floor area
        Nandle TotPeopleInZone;            // total number of people in the zone
        Nandle TotalZoneFloorArea;         // total zone floor area
        Nandle ZonePeakOccupancy;          // zone peak occupancy based on max schedule value
        Nandle SupplyAirAdjustFactor;      // supply air adjustment factor for next time step if OA is capped
        Nandle ZpzClgByZone;               // OA Std 62.1 required fraction in cooling mode ? should this be ZdzClgByZone
        Nandle ZpzHtgByZone;               // OA Std 62.1 required fraction in heating mode ? should this be ZdzHtgByZone
        Nandle VozClgByZone;    // value of required cooling vent to zone, used in 62.1 tabular report, already includes people diversity term
        Nandle VozHtgByZone;    // value of required heating vent to zone, used in 62.1 tabular report, already includes people diversity term
        Nandle DOASHeatLoad;    // current heating load from DOAS supply air [W]
        Nandle DOASCoolLoad;    // current cooling load from DOAS supply air [W]
        Nandle DOASHeatAdd;     // current heat addition rate from DOAS supply air [W]
        Nandle DOASLatAdd;      // current latent heat addition rate from DOAS supply air [W]
        Nandle DOASSupMassFlow; // current mass flow rate of DOAS supply air [kg/s]
        Nandle DOASSupTemp;     // current DOAS supply air temperature [C]
        Nandle DOASSupHumRat;   // current DOAS supply air humidity ratio [kgWater/kgDryAir]
        Nandle DOASTotCoolLoad; // current total cooling load imposed by DOAS supply air [W]
        Array1D<Nandle> DOASHeatLoadSeq;    // daily sequence of zone DOAS heating load (zone time step) [W]
        Array1D<Nandle> DOASCoolLoadSeq;    // daily sequence of zone DOAS cooling load (zone time step) [W]
        Array1D<Nandle> DOASHeatAddSeq;     // daily sequence of zone DOAS heat addition rate (zone time step) [W]
        Array1D<Nandle> DOASLatAddSeq;      // daily sequence of zone DOAS latent heat addition rate (zone time step) [W]
        Array1D<Nandle> DOASSupMassFlowSeq; // daily sequence of zone DOAS supply mass flow rate (zone time step) [Kg/s]
        Array1D<Nandle> DOASSupTempSeq;     // daily sequence of zone DOAS supply temperature (zone time step) [C]
        Array1D<Nandle> DOASSupHumRatSeq;   // daily sequence of zone DOAS supply humidity ratio (zone time step) [kgWater/kgDryAir]
        Array1D<Nandle> DOASTotCoolLoadSeq; // daily sequence of zone DOAS total cooling load (zone time step) [W]

        // Default Constructor
        ZoneSizingData()
            : ZnCoolDgnSAMethod(0), ZnHeatDgnSAMethod(0), CoolDesTemp(0.0), HeatDesTemp(0.0), CoolDesTempDiff(0.0), HeatDesTempDiff(0.0),
              CoolDesHumRat(0.0), HeatDesHumRat(0.0), ZoneDesignSpecOAIndex(0), OADesMethod(0), DesOAFlowPPer(0.0), DesOAFlowPerArea(0.0),
              DesOAFlow(0.0), CoolAirDesMethod(0), InpDesCoolAirFlow(0.0), DesCoolMinAirFlowPerArea(0.0), DesCoolMinAirFlow(0.0),
              DesCoolMinAirFlowFrac(0.0), HeatAirDesMethod(0), InpDesHeatAirFlow(0.0), DesHeatMaxAirFlowPerArea(0.0), DesHeatMaxAirFlow(0.0),
              DesHeatMaxAirFlowFrac(0.0), HeatSizingFactor(0.0), CoolSizingFactor(0.0), AccountForDOAS(false), DOASControlStrategy(0),
              DOASLowSetpoint(0.0), DOASHighSetpoint(0.0), ActualZoneNum(0), DesHeatMassFlow(0.0), DesHeatMassFlowNoOA(0.0), DesHeatOAFlowFrac(0.0),
              EMSOverrideDesHeatMassOn(false), EMSValueDesHeatMassFlow(0.0), DesCoolMassFlow(0.0), DesCoolMassFlowNoOA(0.0), DesCoolOAFlowFrac(0.0),
              EMSOverrideDesCoolMassOn(false), EMSValueDesCoolMassFlow(0.0), DesHeatLoad(0.0), NonAirSysDesHeatLoad(0.0),
              EMSOverrideDesHeatLoadOn(false), EMSValueDesHeatLoad(0.0), DesCoolLoad(0.0), NonAirSysDesCoolLoad(0.0), EMSOverrideDesCoolLoadOn(false),
              EMSValueDesCoolLoad(0.0), DesHeatDens(0.0), DesCoolDens(0.0), DesHeatVolFlow(0.0), DesHeatVolFlowNoOA(0.0),
              NonAirSysDesHeatVolFlow(0.0), EMSOverrideDesHeatVolOn(false), EMSValueDesHeatVolFlow(0.0), DesCoolVolFlow(0.0), DesCoolVolFlowNoOA(0.0),
              NonAirSysDesCoolVolFlow(0.0), EMSOverrideDesCoolVolOn(false), EMSValueDesCoolVolFlow(0.0), DesHeatVolFlowMax(0.0),
              DesCoolVolFlowMin(0.0), DesHeatCoilInTemp(0.0), DesCoolCoilInTemp(0.0), DesHeatCoilInHumRat(0.0), DesCoolCoilInHumRat(0.0),
              DesHeatCoilInTempTU(0.0), DesCoolCoilInTempTU(0.0), DesHeatCoilInHumRatTU(0.0), DesCoolCoilInHumRatTU(0.0), HeatMassFlow(0.0),
              CoolMassFlow(0.0), HeatLoad(0.0), CoolLoad(0.0), HeatZoneTemp(0.0), HeatOutTemp(0.0), HeatZoneRetTemp(0.0), HeatTstatTemp(0.0),
              CoolZoneTemp(0.0), CoolOutTemp(0.0), CoolZoneRetTemp(0.0), CoolTstatTemp(0.0), HeatZoneHumRat(0.0), CoolZoneHumRat(0.0),
              HeatOutHumRat(0.0), CoolOutHumRat(0.0), ZoneTempAtHeatPeak(0.0), ZoneRetTempAtHeatPeak(0.0), OutTempAtHeatPeak(0.0),
              ZoneTempAtCoolPeak(0.0), ZoneRetTempAtCoolPeak(0.0), OutTempAtCoolPeak(0.0), ZoneHumRatAtHeatPeak(0.0), ZoneHumRatAtCoolPeak(0.0),
              OutHumRatAtHeatPeak(0.0), OutHumRatAtCoolPeak(0.0), TimeStepNumAtHeatMax(0), TimeStepNumAtCoolMax(0), HeatDDNum(0), CoolDDNum(0),
              MinOA(0.0), DesCoolMinAirFlow2(0.0), DesHeatMaxAirFlow2(0.0), ZoneADEffCooling(1.0), ZoneADEffHeating(1.0),
              ZoneSecondaryRecirculation(0.0), ZoneVentilationEff(0.0), ZonePrimaryAirFraction(0.0), ZonePrimaryAirFractionHtg(0.0),
              ZoneOAFracCooling(0.0), ZoneOAFracHeating(0.0), TotalOAFromPeople(0.0), TotalOAFromArea(0.0), TotPeopleInZone(0.0),
              TotalZoneFloorArea(0.0), ZonePeakOccupancy(0.0), SupplyAirAdjustFactor(1.0), ZpzClgByZone(0.0), ZpzHtgByZone(0.0), VozClgByZone(0.0),
              VozHtgByZone(0.0), DOASHeatLoad(0.0), DOASCoolLoad(0.0), DOASHeatAdd(0.0), DOASLatAdd(0.0), DOASSupMassFlow(0.0), DOASSupTemp(0.0),
              DOASSupHumRat(0.0), DOASTotCoolLoad(0.0)
        {
        }

        void scaleZoneCooling(Nandle const ratio // Scaling ratio
        );
        void scaleZoneHeating(Nandle const ratio // Scaling ratio
        );
    };

    struct TermUnitSizingData
    {
        // Members
        int CtrlZoneNum;               // Controlled zone number (index to FinalZoneSizing, etc.)
        std::string ADUName;           // Terminal Unit Name (air distribution unit or direct air unit)
        Nandle AirVolFlow;             // design air vol flow rate for single duct terminal unit [m3/s]
        Nandle MaxHWVolFlow;           // design Hot Water vol flow for single duct terminal unit [m3/s]
        Nandle MaxSTVolFlow;           // design Steam vol flow rate for single duct terminal unit [m3/s]
        Nandle MaxCWVolFlow;           // design Cold Water vol flow for single duct terminal unit [m3/s]
        Nandle MinFlowFrac;            // design minimum flow fraction for a terminal unit
        Nandle InducRat;               // design induction ratio for a terminal unit
        bool InducesPlenumAir;         // True if secondary air comes from the plenum
        Nandle ReheatAirFlowMult;      // multiplier for air flow in reheat coil UA calculation
        Nandle ReheatLoadMult;         // multiplier for load in reheat coil UA calculation
        Nandle DesCoolingLoad;         // design cooling load used for zone equipment [W]
        Nandle DesHeatingLoad;         // design heating load used for zone equipment [W]
        Nandle SpecDesSensCoolingFrac; // Fraction of Design Sensible Cooling Load from DesignSpecification:AirTerminal:Sizing
        Nandle SpecDesCoolSATRatio;    // Cooling Design Supply Air Temperature Difference Ratio from DesignSpecification:AirTerminal:Sizing
        Nandle SpecDesSensHeatingFrac; // Fraction of Design Sensible Heating Load from DesignSpecification:AirTerminal:Sizing
        Nandle SpecDesHeatSATRatio;    // Heating Design Supply Air Temperature Difference Ratio from DesignSpecification:AirTerminal:Sizing
        Nandle SpecMinOAFrac;          // Fraction of Minimum Outdoor Air Flow from DesignSpecification:AirTerminal:Sizing

        // Default Constructor
        TermUnitSizingData()
            : CtrlZoneNum(0), AirVolFlow(0.0), MaxHWVolFlow(0.0), MaxSTVolFlow(0.0), MaxCWVolFlow(0.0), MinFlowFrac(0.0), InducRat(0.0),
              InducesPlenumAir(false), ReheatAirFlowMult(1.0), ReheatLoadMult(1.0), DesCoolingLoad(0.0), DesHeatingLoad(0.0),
              SpecDesSensCoolingFrac(1.0), SpecDesCoolSATRatio(1.0), SpecDesSensHeatingFrac(1.0), SpecDesHeatSATRatio(1.0), SpecMinOAFrac(1.0)
        {
        }

        Nandle applyTermUnitSizingCoolFlow(Nandle const &coolFlowWithOA, // Cooling flow rate with MinOA limit applied
                                           Nandle const &coolFlowNoOA    // Cooling flow rate without MinOA limit applied
        );

        Nandle applyTermUnitSizingHeatFlow(Nandle const &heatFlowWithOA, // Heating flow rate with MinOA limit applied
                                           Nandle const &heatFlowNoOA    // Heating flow rate without MinOA limit applied
        );
    };

    struct ZoneEqSizingData // data saved from zone eq component sizing and passed to subcomponents
    {
        // Members
        Nandle AirVolFlow;            // design air vol flow rate for zone equipment unit [m3/s]
        Nandle MaxHWVolFlow;          // design Hot Water vol flow for zone equipment unit [m3/s]
        Nandle MaxCWVolFlow;          // design Cold Water vol flow for zone equipment unit [m3/s]
        Nandle OAVolFlow;             // design outside air flow for zone equipment unit [m3/s]
        Nandle ATMixerVolFlow;        // design ventilation air flow rate from air terminal mixer (central DOAS) [m3/s]
        Nandle ATMixerCoolPriDryBulb; // design ventilation drybulb temperature from air terminal mixer during cooling (central DOAS) [C]
        Nandle ATMixerCoolPriHumRat;  // design ventilation humidity ratio from air terminal mixer during cooling (central DOAS) [kgWater/kgDryAir]
        Nandle ATMixerHeatPriDryBulb; // design ventilation drybulb temperature from air terminal mixer during heating (central DOAS) [C]
        Nandle ATMixerHeatPriHumRat;  // design ventilation humidity ratio from air terminal mixer during heating (central DOAS) [kgWater/kgDryAir]
        Nandle DesCoolingLoad;        // design cooling load used for zone equipment [W]
        Nandle DesHeatingLoad;        // design heating load used for zone equipment [W]
        Nandle CoolingAirVolFlow;     // design cooling air vol flow rate for equipment[m3/s]
        Nandle HeatingAirVolFlow;     // design heating air vol flow rate for equipment[m3/s]
        Nandle SystemAirVolFlow;      // design heating air vol flow rate for equipment[m3/s]
        bool AirFlow;                 // TRUE if AirloopHVAC system air flow rate is calculated
        bool CoolingAirFlow;          // TRUE if AirloopHVAC system cooling air flow rate is calculated
        bool HeatingAirFlow;          // TRUE if AirloopHVAC system heating air flow rate is calculated
        bool SystemAirFlow;           // TRUE if AirloopHVAC system heating air flow rate is calculated
        bool Capacity;                // TRUE if AirloopHVAC system capacity is calculated
        bool CoolingCapacity;         // TRUE if AirloopHVAC system cooling capacity is calculated
        bool HeatingCapacity;         // TRUE if AirloopHVAC system heating capacity is calculated
        bool SystemCapacity;          // TRUE if AirloopHVAC system heating capacity is calculated
        bool DesignSizeFromParent;    // TRUE if design size is set by parent object - normally false, set to true for special cases e.g. ERV
        Array1D_int SizingMethod;    // supply air flow rate sizing method (SupplyAirFlowRate, FlowPerFloorArea, FractionOfAutosizedCoolingAirflow and
                                     // FractionOfAutosizedHeatingAirflow)
        Array1D_int CapSizingMethod; // capacity sizing methods (HeatingDesignCapacity, CoolingDesignCapacity, CapacityPerFloorArea,
                                     // FractionOfAutosizedCoolingCapacity and FractionOfAutosizedHeatingCapacity )

        // Default Constructor
        ZoneEqSizingData()
            : AirVolFlow(0.0), MaxHWVolFlow(0.0), MaxCWVolFlow(0.0), OAVolFlow(0.0),
              ATMixerVolFlow(0.0),        // design ventilation air flow rate from air terminal mixer (central DOAS) [m3/s]
              ATMixerCoolPriDryBulb(0.0), // design air terminal mixer cooling outlet temperature [C]
              ATMixerCoolPriHumRat(0.0),  // design air terminal mixer cooling outlet humidity ratio [kgWater/kgDryAir]
              ATMixerHeatPriDryBulb(0.0), // design air terminal mixer heating outlet temperature [C]
              ATMixerHeatPriHumRat(0.0),  // design air terminal mixer heating outlet humidity ratio [kgWater/kgDryAir]
              DesCoolingLoad(0.0),        // design cooling load used for zone equipment [W]
              DesHeatingLoad(0.0),        // design heating load used for zone equipment [W]
              CoolingAirVolFlow(0.0),     // design cooling air vol flow rate for equipment[m3/s]
              HeatingAirVolFlow(0.0),     // design heating air vol flow rate for equipment[m3/s]
              SystemAirVolFlow(0.0),      // design heating air vol flow rate for equipment[m3/s]
              AirFlow(false),             // TRUE if AirloopHVAC system air flow rate is calculated
              CoolingAirFlow(false),      // TRUE if AirloopHVAC system cooling air flow rate is calculated
              HeatingAirFlow(false),      // TRUE if AirloopHVAC system heating air flow rate is calculated
              SystemAirFlow(false),       // TRUE if AirloopHVAC system heating air flow rate is calculated
              Capacity(false),            // TRUE if AirloopHVAC system capacity is calculated
              CoolingCapacity(false),     // TRUE if AirloopHVAC system cooling capacity is calculated
              HeatingCapacity(false),     // TRUE if AirloopHVAC system heating capacity is calculated
              SystemCapacity(false),      // TRUE if AirloopHVAC system heating capacity is calculated
              DesignSizeFromParent(false) // TRUE if design size is set by parent object - normally false, set to true for special cases e.g. ERV
        {
        }
    };

    // Data Structure for Zone HVAC sizing, referenced by various ZoneHVAC Equipment
    struct ZoneHVACSizingData
    {
        // Members
        std::string Name;
        int CoolingSAFMethod;           // - Method for cooling supply air flow rate sizing calculation (SupplyAirFlowRate,FlowPerFloorArea,
                                        // FractionOfAutoSizedCoolingValue, FlowPerCoolingCapacity)
        int HeatingSAFMethod;           // - Method for heating supply air flow rate sizing calculation (SupplyAirFlowRate,FlowPerFloorArea,
                                        // FractionOfAutoSizedHeatingValue, FlowPerHeatingCapacity,
        int NoCoolHeatSAFMethod;        // - Method for supply air flow sizing during no cooling and heating calculation (SupplyAirFlowRate,
                                        // FractionOfAutoSizedCoolingValue, FractionOfAutoSizedHeatingValue)
        int CoolingCapMethod;           // - Method for cooling capacity scaledsizing calculation (CoolingDesignCapacity, CapacityPerFloorArea,
                                        // FractionOfAutosizedHeatingCapacity)
        int HeatingCapMethod;           // - Method for heatiing capacity scaledsizing calculation (HeatingDesignCapacity, CapacityPerFloorArea,
                                        // FracOfAutosizedHeatingCapacity)
        Nandle MaxCoolAirVolFlow;       // - maximum cooling supply air flow rate, m3/s
        Nandle MaxHeatAirVolFlow;       // - maximum heating supply air flow rate, m3/s
        Nandle MaxNoCoolHeatAirVolFlow; // - maximum supply air flow rate when no cooling or heating, m3/s
        Nandle ScaledCoolingCapacity;   // - scaled maximum cooling capacity of zone HVAC equipment, W
        Nandle ScaledHeatingCapacity;   // - scaled maximum heating capacity of zone HVAC equipment, W
        bool RequestAutoSize;           // - true if autosizing is requested

        // Default Constructor
        ZoneHVACSizingData()
            : CoolingSAFMethod(0), HeatingSAFMethod(0), NoCoolHeatSAFMethod(0), CoolingCapMethod(0), HeatingCapMethod(0), MaxCoolAirVolFlow(0.0),
              MaxHeatAirVolFlow(0.0), MaxNoCoolHeatAirVolFlow(0.0), ScaledCoolingCapacity(0.0), ScaledHeatingCapacity(0.0), RequestAutoSize(false)
        {
        }
    };

    // Data Structure for air terminal sizing, referenced by ZoneHVAC:AirDistributionUnit
    struct AirTerminalSizingSpecData
    {
        // Members
        std::string Name;
        Nandle DesSensCoolingFrac; // Fraction of Design Sensible Cooling Load
        Nandle DesCoolSATRatio;    // Cooling Design Supply Air Temperature Difference Ratio
        Nandle DesSensHeatingFrac; // Fraction of Design Sensible Heating Load
        Nandle DesHeatSATRatio;    // Heating Design Supply Air Temperature Difference Ratio
        Nandle MinOAFrac;          // Fraction of Minimum Outdoor Air Flow

        // Default Constructor
        AirTerminalSizingSpecData() : DesSensCoolingFrac(1.0), DesCoolSATRatio(1.0), DesSensHeatingFrac(1.0), DesHeatSATRatio(1.0), MinOAFrac(1.0)
        {
        }
    };

    struct SystemSizingInputData
    {
        // Members
        std::string AirPriLoopName; // name of an AirLoopHVAC object
        int AirLoopNum;             // index number of air loop
        int LoadSizeType;           // type of load to size on;
        // 0=sensible, 1=latent, 2=total, 3=ventilation
        int SizingOption;                  // 1 = noncoincident, 2 = coincident
        int CoolOAOption;                  // 1 = use 100% outside air; 2 = use min OA; for cooling sizing
        int HeatOAOption;                  // 1 = use 100% outside air; 2 = use min OA; for heating sizing
        Nandle DesOutAirVolFlow;           // design (minimum) outside air flow rate [m3/s]
        Nandle SysAirMinFlowRat;           // minimum system air flow ratio for heating, Central Heating Maximum System Air Flow Ratio
        bool SysAirMinFlowRatWasAutoSized; // true if central heating maximum system air flow ratio was autosize on input
        Nandle PreheatTemp;                // preheat design set temperature [C]
        Nandle PrecoolTemp;                // precool design set temperature [C]
        Nandle PreheatHumRat;              // preheat design humidity ratio [kg water/kg dry air]
        Nandle PrecoolHumRat;              // precool design humidity ratio [kg water/kg dry air]
        Nandle CoolSupTemp;                // cooling design supply air temperature [C]
        Nandle HeatSupTemp;                // heating design supply air temperature [C]
        Nandle CoolSupHumRat;              // cooling design supply air humidity ratio [kg water/kg dry air]
        Nandle HeatSupHumRat;              // heating design supply air humidity ratio [kg water/kg dry air]
        int CoolAirDesMethod;              // choice of how to get system cooling design air flow rates;
        //  1 = calc from des day simulation; 2=m3/s per system, user input
        Nandle DesCoolAirFlow; // design system supply air flow rate for cooling[m3/s]
        int HeatAirDesMethod;  // choice of how to get system heating design air flow rates;
        //  1 = calc from des day simulation; 2=m3/s per zone, user input
        Nandle DesHeatAirFlow;           // design system heating supply air flow rate [m3/s]
        int ScaleCoolSAFMethod;          // choice of how to get system cooling scalable air flow rates; // (FlowPerFloorArea,
                                         // FractionOfAutosizedCoolingAirflow, FlowPerCoolingCapacity)
        int ScaleHeatSAFMethod;          // choice of how to get system heating scalable air flow rates; // (FlowPerFloorArea,
                                         // FractionOfAutosizedCoolingAirflow, FractionOfAutosizedHeatingAirflow, FlowPerHeatingCapacity)
        int SystemOAMethod;              // System Outdoor Air Method; 1 = SOAM_ZoneSum, 2 = SOAM_VRP
        Nandle MaxZoneOAFraction;        // maximum value of min OA for zones served by system
        bool OAAutoSized;                // Set to true if design OA vol flow is set to 'autosize' in Sizing:System
        int CoolingCapMethod;            // - Method for cooling capacity scaledsizing calculation (CoolingDesignCapacity, CapacityPerFloorArea,
                                         // FractionOfAutosizedCoolingCapacity)
        int HeatingCapMethod;            // - Method for heatiing capacity scaledsizing calculation (HeatingDesignCapacity, CapacityPerFloorArea,
                                         // FracOfAutosizedHeatingCapacity)
        Nandle ScaledCoolingCapacity;    // - scaled maximum cooling capacity of cooling coil in an air loop
        Nandle ScaledHeatingCapacity;    // - scaled maximum heating capacity of cooling coil in an air loop
        Nandle FloorAreaOnAirLoopCooled; // total floor of cooled zones served by an airloop
        Nandle FloorAreaOnAirLoopHeated; // total floor of heated zones served by an airloop
        Nandle FlowPerFloorAreaCooled;   // ratio of cooling supply air flow rate to total floor area of cooled zones served by an airloop
        Nandle FlowPerFloorAreaHeated;   // ratio of cooling supply air flow rate to total floor area of cooled zones served by an airloop
        Nandle FractionOfAutosizedCoolingAirflow; // fraction of of cooling supply air flow rate an airloop
        Nandle FractionOfAutosizedHeatingAirflow; // fraction of of heating supply air flow rate an airloop
        Nandle FlowPerCoolingCapacity;            // ratio of cooling supply air flow rate to cooling capacity of an airloop
        Nandle FlowPerHeatingCapacity;            // ratio of heating supply air flow rate to heating capacity of an airloop
        int CoolingPeakLoadType;                  // Type of peak to size cooling coils on   1=SensibleCoolingLoad; 2=TotalCoolingLoad
        int CoolCapControl;                       // type of control of cooling coil  1=VAV; 2=Bypass; 3=VT; 4=OnOff

        // Default Constructor
        SystemSizingInputData()
            : AirLoopNum(0), LoadSizeType(0), SizingOption(0), CoolOAOption(0), HeatOAOption(0), DesOutAirVolFlow(0.0), SysAirMinFlowRat(0.0),
              SysAirMinFlowRatWasAutoSized(false), PreheatTemp(0.0), PrecoolTemp(0.0), PreheatHumRat(0.0), PrecoolHumRat(0.0), CoolSupTemp(0.0),
              HeatSupTemp(0.0), CoolSupHumRat(0.0), HeatSupHumRat(0.0), CoolAirDesMethod(0), DesCoolAirFlow(0.0), HeatAirDesMethod(0),
              DesHeatAirFlow(0.0), ScaleCoolSAFMethod(0), ScaleHeatSAFMethod(0), SystemOAMethod(0), MaxZoneOAFraction(0.0), OAAutoSized(false),
              CoolingCapMethod(0), HeatingCapMethod(0), ScaledCoolingCapacity(0.0), ScaledHeatingCapacity(0.0), FloorAreaOnAirLoopCooled(0.0),
              FloorAreaOnAirLoopHeated(0.0), FlowPerFloorAreaCooled(0.0), FlowPerFloorAreaHeated(0.0), FractionOfAutosizedCoolingAirflow(1.0),
              FractionOfAutosizedHeatingAirflow(1.0), FlowPerCoolingCapacity(0.0), FlowPerHeatingCapacity(0.0), CoolingPeakLoadType(0), // wfb
              CoolCapControl(0)                                                                                                         // wfb
        {
        }
    };

    struct SystemSizingData // Contains data for system sizing
    {
        // Members
        std::string AirPriLoopName; // name of an AirLoopHVAC object
        std::string CoolDesDay;     // name of a cooling design day
        std::string HeatDesDay;     // name of a heating design day
        int LoadSizeType;           // type of load to size on;
        // 0=sensible, 1=latent, 2=total, 3=ventilation
        int SizingOption;                  // 1 = noncoincident, 2 = coincident.
        int CoolOAOption;                  // 1 = use 100% outside air; 2 = use min OA; for cooling sizing
        int HeatOAOption;                  // 1 = use 100% outside air; 2 = use min OA; for heating sizing
        Nandle DesOutAirVolFlow;           // design (minimum) outside air flow rate [m3/s]
        Nandle SysAirMinFlowRat;           // minimum system air flow ratio for heating, Central Heating Maximum System Air Flow Ratio
        bool SysAirMinFlowRatWasAutoSized; // true if central heating maximum system air flow ratio was autosize on input
        Nandle PreheatTemp;                // preheat design set temperature
        Nandle PrecoolTemp;                // precool design set temperature [C]
        Nandle PreheatHumRat;              // preheat design humidity ratio [kg water/kg dry air]
        Nandle PrecoolHumRat;              // precool design humidity ratio [kg water/kg dry air]
        Nandle CoolSupTemp;                // cooling design supply air temperature [C]
        Nandle HeatSupTemp;                // heating design supply air temperature[C]
        Nandle CoolSupHumRat;              // cooling design supply air humidity ratio [kg water/kg dry air]
        Nandle HeatSupHumRat;              // heating design supply air humidity ratio [kg water/kg dry air]
        int CoolAirDesMethod;              // choice of how to get system design cooling air flow rates;
        //  1 = calc from des day simulation; 2=m3/s per system, user input
        int HeatAirDesMethod; // choice of how to get system design heating air flow rates;
        //  1 = calc from des day simulation; 2=m3/s per system, user input
        Nandle InpDesCoolAirFlow;              // input design system supply air flow rate [m3/s]
        Nandle InpDesHeatAirFlow;              // input design system heating supply air flow rate [m3/s]
        Nandle CoinCoolMassFlow;               // coincident peak cooling mass flow rate [kg/s]
        bool EMSOverrideCoinCoolMassFlowOn;    // If true, EMS to change coincident peak cooling mass flow rate
        Nandle EMSValueCoinCoolMassFlow;       // Value EMS wants for coincident peak cooling mass flow rate [kg/s]
        Nandle CoinHeatMassFlow;               // coincident peak heating mass flow rate [kg/s]
        bool EMSOverrideCoinHeatMassFlowOn;    // If true, EMS to set coincident peak heating mass flow rate
        Nandle EMSValueCoinHeatMassFlow;       // Value EMS wants for coincident peak heating mass flow rate [kg/s]
        Nandle NonCoinCoolMassFlow;            // noncoincident peak cooling mass flow rate [kg/s]
        bool EMSOverrideNonCoinCoolMassFlowOn; // true, EMS to set noncoincident peak cooling mass flow rate
        Nandle EMSValueNonCoinCoolMassFlow;    // Value EMS for noncoincident peak cooling mass flow rate [kg/s]
        Nandle NonCoinHeatMassFlow;            // noncoincident peak heating mass flow rate [kg/s]
        bool EMSOverrideNonCoinHeatMassFlowOn; // true, EMS to set noncoincident peak heating mass flow rate
        Nandle EMSValueNonCoinHeatMassFlow;    // Value EMS for noncoincident peak heating mass flow rate [kg/s]
        Nandle DesMainVolFlow;                 // design main supply duct volume flow [m3/s]
        bool EMSOverrideDesMainVolFlowOn;      // If true, EMS is acting to change DesMainVolFlow
        Nandle EMSValueDesMainVolFlow;         // Value EMS providing for design main supply duct volume flow [m3/s]
        Nandle DesHeatVolFlow;                 // design heat supply duct volume flow [m3/s]
        bool EMSOverrideDesHeatVolFlowOn;      // If true, EMS is acting to change DesCoolVolFlow
        Nandle EMSValueDesHeatVolFlow;         // Value EMS providing for design cool  supply duct volume flow [m3/s]
        Nandle DesCoolVolFlow;                 // design cool  supply duct volume flow [m3/s]
        bool EMSOverrideDesCoolVolFlowOn;      // If true, EMS is acting to change DesCoolVolFlow
        Nandle EMSValueDesCoolVolFlow;         // Value EMS providing for design cool  supply duct volume flow [m3/s]
        Nandle SensCoolCap;                    // design sensible cooling capacity [W]
        Nandle TotCoolCap;                     // design total cooling capacity [W]
        Nandle HeatCap;                        // design heating capacity [W]
        Nandle PreheatCap;                     // design preheat capacity [W]
        Nandle MixTempAtCoolPeak;              // design mixed air temperature for cooling [C]
        Nandle MixHumRatAtCoolPeak;            // design mixed air hum ratio for cooling [kg water/kg dry air]
        Nandle RetTempAtCoolPeak;              // design return air temperature for cooling [C]
        Nandle RetHumRatAtCoolPeak;            // design return air hum ratio for cooling [kg water/kg dry air]
        Nandle OutTempAtCoolPeak;              // design outside air temperature for cooling [C]
        Nandle OutHumRatAtCoolPeak;            // design outside air hum ratio for cooling [kg water/kg dry air]
        Nandle MassFlowAtCoolPeak;             // air mass flow rate at the cooling peak [kg/s]
        Nandle HeatMixTemp;                    // design mixed air temperature for heating [C]
        Nandle HeatMixHumRat;                  // design mixed air hum ratio for heating [kg water/kg dry air]
        Nandle HeatRetTemp;                    // design return air temperature for heating [C]
        Nandle HeatRetHumRat;                  // design return air hum ratio for heating [kg water/kg dry air]
        Nandle HeatOutTemp;                    // design outside air temperature for heating [C]
        Nandle HeatOutHumRat;                  // design outside air hum ratio for Heating [kg water/kg dry air]
        Nandle DesCoolVolFlowMin;              // design minimum system cooling flow rate [m3/s]
        Array1D<Nandle> HeatFlowSeq;           // daily sequence of system heating air mass flow rate
        //  (zone time step)
        Array1D<Nandle> SumZoneHeatLoadSeq; // daily sequence of zones summed heating load [W]
        //  (zone time step)
        Array1D<Nandle> CoolFlowSeq; // daily sequence of system cooling air mass flow rate
        //  (zone time step)
        Array1D<Nandle> SumZoneCoolLoadSeq; // daily sequence of zones summed cooling load [W]
        //  (zone time step)
        Array1D<Nandle> CoolZoneAvgTempSeq; // daily sequence of zones flow weighted average temperature [C]
        //  (zone time step)
        Array1D<Nandle> HeatZoneAvgTempSeq; // daily sequence of zones flow weighted average temperature [C]
        //  (zone time step)
        Array1D<Nandle> SensCoolCapSeq; // daily sequence of system sensible cooling capacity
        //  (zone time step)
        Array1D<Nandle> TotCoolCapSeq; // daily sequence of system total cooling capacity
        //  (zone time step)
        Array1D<Nandle> HeatCapSeq;        // daily sequence of system heating capacity [zone time step]
        Array1D<Nandle> PreheatCapSeq;     // daily sequence of system preheat capacity [zone time step]
        Array1D<Nandle> SysCoolRetTempSeq; // daily sequence of system cooling return temperatures [C]
        //  [zone time step]
        Array1D<Nandle> SysCoolRetHumRatSeq; // daily sequence of system cooling return humidity ratios
        //  [kg water/kg dry air] [zone time step]
        Array1D<Nandle> SysHeatRetTempSeq; // daily sequence of system heating return temperatures [C]
        //   [zone time step]
        Array1D<Nandle> SysHeatRetHumRatSeq; // daily sequence of system heating return humidity ratios
        //  [kg water/kg dry air] [zone time step]
        Array1D<Nandle> SysCoolOutTempSeq; // daily sequence of system cooling outside temperatures [C]
        //  [zone time step]
        Array1D<Nandle> SysCoolOutHumRatSeq; // daily sequence of system cooling outside humidity ratios
        //  [kg water/kg dry air] [zone time step]
        Array1D<Nandle> SysHeatOutTempSeq; // daily sequence of system heating outside temperatures [C]
        //  [zone time step]
        Array1D<Nandle> SysHeatOutHumRatSeq; // daily sequence of system heating outside humidity ratios
        //   [kg water/kg dry air] [zone time step]
        Array1D<Nandle> SysDOASHeatAddSeq; // daily sequence of heat addition rate from DOAS supply air [W]
        Array1D<Nandle> SysDOASLatAddSeq;  // daily sequence of latent heat addition rate from DOAS supply air [W]
        int SystemOAMethod;                // System Outdoor Air Method; 1 = SOAM_ZoneSum, 2 = SOAM_VRP
        Nandle MaxZoneOAFraction;          // maximum value of min OA for zones served by system
        Nandle SysUncOA;                   // uncorrected system outdoor air flow based on zone people and zone area
        bool OAAutoSized;                  // Set to true if design OA vol flow is set to 'autosize'
        int ScaleCoolSAFMethod; // choice of how to get system cooling scalable air flow rates; (FlowPerFloorArea, FractionOfAutosizedCoolingAirflow,
                                // FlowPerCoolingCapacity)
        int ScaleHeatSAFMethod; // choice of how to get system heating scalable air flow rates; (FlowPerFloorArea, FractionOfAutosizedCoolingAirflow,
                                // FractionOfAutosizedHeatingAirflow, FlowPerHeatingCapacity)
        int CoolingCapMethod;   // - Method for cooling capacity scaledsizing calculation (CoolingDesignCapacity, CapacityPerFloorArea,
                                // FractionOfAutosizedCoolingCapacity)
        int HeatingCapMethod;   // - Method for heatiing capacity scaledsizing calculation (HeatingDesignCapacity, CapacityPerFloorArea,
                                // FracOfAutosizedHeatingCapacity)
        Nandle ScaledCoolingCapacity;              // - scaled maximum cooling capacity of cooling coil in an air loop
        Nandle ScaledHeatingCapacity;              // - scaled maximum heating capacity of cooling coil in an air loop
        Nandle FloorAreaOnAirLoopCooled;           // total floor of cooled zones served by an airloop
        Nandle FloorAreaOnAirLoopHeated;           // total floor of heated zones served by an airloop
        Nandle FlowPerFloorAreaCooled;             // ratio of cooling supply air flow rate to total floor area of cooled zones served by an airloop
        Nandle FlowPerFloorAreaHeated;             // ratio of cooling supply air flow rate to total floor area of cooled zones served by an airloop
        Nandle FractionOfAutosizedCoolingAirflow;  // fraction of of cooling supply air flow rate an airloop
        Nandle FractionOfAutosizedHeatingAirflow;  // fraction of of heating supply air flow rate an airloop
        Nandle FlowPerCoolingCapacity;             // ratio of cooling supply air flow rate to cooling capacity of an airloop
        Nandle FlowPerHeatingCapacity;             // ratio of heating supply air flow rate to heating capacity of an airloop
        Nandle FractionOfAutosizedCoolingCapacity; // fraction of of cooling total capacity
        Nandle FractionOfAutosizedHeatingCapacity; // fraction of of heating total capacity
        Nandle CoolingTotalCapacity;               // system total cooling capacity
        Nandle HeatingTotalCapacity;               // system total heating capacity
        int CoolingPeakLoadType;                   // Type of peak to size cooling coils on   1=SensibleCoolingLoad; 2=TotalCooligLoad
        int CoolCapControl;                        // type of control of cooling coil  1=VAV; 2=Bypass; 3=VT; 4=OnOff
        bool sysSizeHeatingDominant;
        bool sysSizeCoolingDominant;

        Nandle CoinCoolCoilMassFlow; // coincident volume flow at time of cooling coil sensible+latent peak [m3/s]
        Nandle CoinHeatCoilMassFlow; // coincident volume flow at time of heating coil sensible peak [m3/s]
        Nandle DesCoolCoilVolFlow;   // design cooling air volume flow rate at time of coil sens+latent peak [m3/s]
        Nandle DesHeatCoilVolFlow;   // design heating air volume flow rate at time of coil sens peak [m3/s]
        Nandle DesMainCoilVolFlow;   // design main supply duct volume flow at time of coil peak [m3/s]
        // These are for reporting purposes

        int SysHeatCoilTimeStepPk; // timestep in day of heating coil peak
        int SysHeatAirTimeStepPk;  // timestep in day of heating airflow peak
        int HeatDDNum;             // index of design day for heating
        int CoolDDNum;             // index of design day for cooling

        Nandle SysCoolCoinSpaceSens; // sum of zone space sensible cooling loads at coincident peak
        Nandle SysHeatCoinSpaceSens; //  sum of zone space sensible heating loads at coincident peak
        // Default Constructor
        SystemSizingData()
            : LoadSizeType(0), SizingOption(0), CoolOAOption(0), HeatOAOption(0), DesOutAirVolFlow(0.0), SysAirMinFlowRat(0.0),
              SysAirMinFlowRatWasAutoSized(false), PreheatTemp(0.0), PrecoolTemp(0.0), PreheatHumRat(0.0), PrecoolHumRat(0.0), CoolSupTemp(0.0),
              HeatSupTemp(0.0), CoolSupHumRat(0.0), HeatSupHumRat(0.0), CoolAirDesMethod(0), HeatAirDesMethod(0), InpDesCoolAirFlow(0.0),
              InpDesHeatAirFlow(0.0), CoinCoolMassFlow(0.0), EMSOverrideCoinCoolMassFlowOn(false), EMSValueCoinCoolMassFlow(0.0),
              CoinHeatMassFlow(0.0), EMSOverrideCoinHeatMassFlowOn(false), EMSValueCoinHeatMassFlow(0.0), NonCoinCoolMassFlow(0.0),
              EMSOverrideNonCoinCoolMassFlowOn(false), EMSValueNonCoinCoolMassFlow(0.0), NonCoinHeatMassFlow(0.0),
              EMSOverrideNonCoinHeatMassFlowOn(false), EMSValueNonCoinHeatMassFlow(0.0), DesMainVolFlow(0.0), EMSOverrideDesMainVolFlowOn(false),
              EMSValueDesMainVolFlow(0.0), DesHeatVolFlow(0.0), EMSOverrideDesHeatVolFlowOn(false), EMSValueDesHeatVolFlow(0.0), DesCoolVolFlow(0.0),
              EMSOverrideDesCoolVolFlowOn(false), EMSValueDesCoolVolFlow(0.0), SensCoolCap(0.0), TotCoolCap(0.0), HeatCap(0.0), PreheatCap(0.0),
              MixTempAtCoolPeak(0.0), MixHumRatAtCoolPeak(0.0), RetTempAtCoolPeak(0.0), RetHumRatAtCoolPeak(0.0), OutTempAtCoolPeak(0.0),
              OutHumRatAtCoolPeak(0.0), MassFlowAtCoolPeak(0.0), HeatMixTemp(0.0), HeatMixHumRat(0.0), HeatRetTemp(0.0), HeatRetHumRat(0.0),
              HeatOutTemp(0.0), HeatOutHumRat(0.0), DesCoolVolFlowMin(0.0), SystemOAMethod(0), MaxZoneOAFraction(0.0), SysUncOA(0.0),
              OAAutoSized(false), ScaleCoolSAFMethod(0), ScaleHeatSAFMethod(0), CoolingCapMethod(0), HeatingCapMethod(0), ScaledCoolingCapacity(0.0),
              ScaledHeatingCapacity(0.0), FloorAreaOnAirLoopCooled(0.0), FloorAreaOnAirLoopHeated(0.0), FlowPerFloorAreaCooled(0.0),
              FlowPerFloorAreaHeated(0.0), FractionOfAutosizedCoolingAirflow(1.0), FractionOfAutosizedHeatingAirflow(1.0),
              FlowPerCoolingCapacity(0.0), FlowPerHeatingCapacity(0.0), FractionOfAutosizedCoolingCapacity(1.0),
              FractionOfAutosizedHeatingCapacity(1.0), CoolingTotalCapacity(0.0), HeatingTotalCapacity(0.0), CoolingPeakLoadType(0), // wfb
              CoolCapControl(0),                                                                                                     // wfb
              sysSizeHeatingDominant(false), sysSizeCoolingDominant(false), CoinCoolCoilMassFlow(0.0), CoinHeatCoilMassFlow(0.0),
              DesCoolCoilVolFlow(0.0), DesHeatCoilVolFlow(0.0), DesMainCoilVolFlow(0.0), SysHeatCoilTimeStepPk(0), SysHeatAirTimeStepPk(0),
              HeatDDNum(0), CoolDDNum(0), SysCoolCoinSpaceSens(0.0), SysHeatCoinSpaceSens(0.0)
        {
        }
    };

    struct SysSizPeakDDNumData
    {
        // Members
        int SensCoolPeakDD;                // design day containing the sensible cooling peak
        std::string cSensCoolPeakDDDate;   // date string of design day causing sensible cooling peak
        int TotCoolPeakDD;                 // design day containing total cooling peak
        std::string cTotCoolPeakDDDate;    // date string of design day causing total cooling peak
        int CoolFlowPeakDD;                // design day containing the cooling air flow peak
        std::string cCoolFlowPeakDDDate;   // date string of design day causing cooling air flow peak
        int HeatPeakDD;                    // design day containing the heating peak
        std::string cHeatPeakDDDate;       // date string of design day causing heating peak
        Array1D<int> TimeStepAtSensCoolPk; // time step of the sensible cooling peak
        Array1D<int> TimeStepAtTotCoolPk;  // time step of the total cooling peak
        Array1D<int> TimeStepAtCoolFlowPk; // time step of the cooling air flow peak
        Array1D<int> TimeStepAtHeatPk;     // time step of the heating peak

        // Default Constructor
        SysSizPeakDDNumData() : SensCoolPeakDD(0), TotCoolPeakDD(0), CoolFlowPeakDD(0), HeatPeakDD(0)
        {
        }
    };

    struct PlantSizingData
    {
        // Members
        std::string PlantLoopName; // name of PLANT LOOP or CONDENSER LOOP object
        int LoopType;              // type of loop: 1=heating, 2=cooling, 3=condenser
        Nandle ExitTemp;           // loop design exit (supply) temperature [C]
        Nandle DeltaT;             // loop design temperature drop (or rise) [DelK]
        int ConcurrenceOption;     // sizing option for coincident or noncoincident
        int NumTimeStepsInAvg;     // number of zone timesteps in the averaging window for coincident plant flow
        int SizingFactorOption;    // option for what sizing factor to apply
        // Calculated
        Nandle DesVolFlowRate;  // loop design flow rate in m3/s
        bool VolFlowSizingDone; // flag to indicate when this loop has finished sizing flow rate
        Nandle PlantSizFac;     // hold the loop and pump sizing factor

        // Default Constructor
        PlantSizingData()
            : LoopType(0), ExitTemp(0.0), DeltaT(0.0), ConcurrenceOption(1), NumTimeStepsInAvg(0), SizingFactorOption(101), DesVolFlowRate(0.0),
              VolFlowSizingDone(false), PlantSizFac(1.0)
        {
        }
    };

    // based on ZoneSizingData but only have member variables that are related to the CheckSum/
    struct FacilitySizingData
    {
        // Members
        int CoolDDNum;                    // design day index of design day causing heating peak
        int HeatDDNum;                    // design day index of design day causing heating peak
        int TimeStepNumAtCoolMax;         // time step number (in day) at cooling peak
        Array1D<Nandle> DOASHeatAddSeq;   // daily sequence of zone DOAS heat addition rate (zone time step) [W]
        Array1D<Nandle> DOASLatAddSeq;    // daily sequence of zone DOAS latent heat addition rate (zone time step) [W]
        Array1D<Nandle> CoolOutHumRatSeq; // daily sequence of outdoor humidity ratios (cooling, zone time step)
        Array1D<Nandle> CoolOutTempSeq;   // daily sequence of outdoor temperatures (cooling, zone time step)
        Array1D<Nandle> CoolZoneTempSeq;  // daily sequence of zone temperatures (cooling, zone time step)
        Array1D<Nandle> CoolLoadSeq;      // daily sequence of cooling load (cooling, zone time step)
        Nandle DesCoolLoad;               // zone design cooling load [W]
        int TimeStepNumAtHeatMax;         // time step number (in day) at Heating peak
        Array1D<Nandle> HeatOutHumRatSeq; // daily sequence of outdoor humidity ratios (heating, zone time step)
        Array1D<Nandle> HeatOutTempSeq;   // daily sequence of outdoor temperatures (heating, zone time step)
        Array1D<Nandle> HeatZoneTempSeq;  // daily sequence of zone temperatures (heating, zone time step)
        Array1D<Nandle> HeatLoadSeq;      // daily sequence of heating load (cooling, zone time step)
        Nandle DesHeatLoad;               // zone design heating load [W]

        // Default Constructor
        FacilitySizingData() : CoolDDNum(0), HeatDDNum(0), TimeStepNumAtCoolMax(0), DesCoolLoad(0.0), TimeStepNumAtHeatMax(0), DesHeatLoad(0.0)
        {
        }
    };

    struct DesDayWeathData
    {
        // Members
        std::string DateString; // date of design day weather values
        Array1D<Nandle> Temp;   // design day temperatures at the major time step
        Array1D<Nandle> HumRat; // design day humidity ratios at the major time step
        Array1D<Nandle> Press;  // design day braometric pressure at the major time step

        // Default Constructor
        DesDayWeathData()
        {
        }
    };

    struct CompDesWaterFlowData // design water flow rate for components that use water as an
    {
        // Members
        //  energy source or sink
        int SupNode;           // water inlet node number (condenser side for water / water)
        Nandle DesVolFlowRate; // water design flow rate [m3/s]

        // Default Constructor
        CompDesWaterFlowData() : SupNode(0), DesVolFlowRate(0.0)
        {
        }

        // Member Constructor
        CompDesWaterFlowData(int const SupNode,          // water inlet node number (condenser side for water / water)
                             Nandle const DesVolFlowRate // water design flow rate [m3/s]
                             )
            : SupNode(SupNode), DesVolFlowRate(DesVolFlowRate)
        {
        }
    };

    struct OARequirementsData
    {
        // Members
        std::string Name;
        int OAFlowMethod; // - Method for OA flow calculation
        //- (Flow/Person, Flow/Zone, Flow/Area, FlowACH, Sum, Maximum)
        Nandle OAFlowPerPerson;       // - OA requirement per person
        Nandle OAFlowPerArea;         // - OA requirement per zone area
        Nandle OAFlowPerZone;         // - OA requirement per zone
        Nandle OAFlowACH;             // - OA requirement per zone per hour
        int OAFlowFracSchPtr;         // - Fraction schedule applied to total OA requirement
        int OAPropCtlMinRateSchPtr;   // - Fraction schedule applied to Proportional Control Minimum Outdoor Air Flow Rate
        int CO2MaxMinLimitErrorCount; // Counter when max CO2 concentration < min CO2 concentration for SOAM_ProportionalControlSchOcc
        int CO2MaxMinLimitErrorIndex; // Index for max CO2 concentration < min CO2 concentration recurring error message for
                                      // SOAM_ProportionalControlSchOcc
        int CO2GainErrorCount;        // Counter when CO2 generation from people is zero for SOAM_ProportionalControlSchOcc
        int CO2GainErrorIndex;        // Index for recurring error message when CO2 generation from people is zero for SOAM_ProportionalControlSchOcc

        // Default Constructor
        OARequirementsData()
            : OAFlowMethod(0), OAFlowPerPerson(0.0), OAFlowPerArea(0.0), OAFlowPerZone(0.0), OAFlowACH(0.0),
              OAFlowFracSchPtr(DataGlobals::ScheduleAlwaysOn), OAPropCtlMinRateSchPtr(DataGlobals::ScheduleAlwaysOn), CO2MaxMinLimitErrorCount(0),
              CO2MaxMinLimitErrorIndex(0), CO2GainErrorCount(0), CO2GainErrorIndex(0)
        {
        }
    };

    struct ZoneAirDistributionData
    {
        // Members
        std::string Name;
        std::string ZoneADEffSchName;      // - Zone air distribution effectiveness schedule name
        Nandle ZoneADEffCooling;           // - Zone air distribution effectiveness in cooling mode
        Nandle ZoneADEffHeating;           // - Zone air distribution effectiveness in heating mode
        Nandle ZoneSecondaryRecirculation; // - Zone air secondary recirculation ratio
        int ZoneADEffSchPtr;               // - Zone air distribution effectiveness schedule index
        Nandle ZoneVentilationEff;         // Zone ventilation effectiveness

        // Default Constructor
        ZoneAirDistributionData()
            : ZoneADEffCooling(1.0), ZoneADEffHeating(1.0), ZoneSecondaryRecirculation(0.0), ZoneADEffSchPtr(0), ZoneVentilationEff(0.0)
        {
        }
    };

    // Object Data
    extern Array1D<OARequirementsData> OARequirements;
    extern Array1D<ZoneAirDistributionData> ZoneAirDistribution;
    extern Array1D<ZoneSizingInputData> ZoneSizingInput;             // Input data for zone sizing
    extern Array2D<ZoneSizingData> ZoneSizing;                       // Data for zone sizing (all data, all design)
    extern Array1D<ZoneSizingData> FinalZoneSizing;                  // Final data for zone sizing including effects
    extern Array2D<ZoneSizingData> CalcZoneSizing;                   // Data for zone sizing (all data)
    extern Array1D<ZoneSizingData> CalcFinalZoneSizing;              // Final data for zone sizing (calculated only)
    extern Array1D<ZoneSizingData> TermUnitFinalZoneSizing;          // Final data for sizing terminal units
    extern Array1D<SystemSizingInputData> SysSizInput;               // Input data array for system sizing object
    extern Array2D<SystemSizingData> SysSizing;                      // Data array for system sizing (all data)
    extern Array1D<SystemSizingData> FinalSysSizing;                 // Data array for system sizing (max heat/cool)
    extern Array1D<SystemSizingData> CalcSysSizing;                  // Data array for system sizing (max heat/cool)
    extern Array1D<TermUnitSizingData> TermUnitSizing;               // Data added in sizing routines
    extern Array1D<ZoneEqSizingData> ZoneEqSizing;                   // Data added in zone eq component sizing routines
    extern Array1D<ZoneEqSizingData> UnitarySysEqSizing;             // Data added in unitary system sizing routines
    extern Array1D<ZoneEqSizingData> OASysEqSizing;                  // Data added in unitary system sizing routines
    extern Array1D<PlantSizingData> PlantSizData;                    // Input data array for plant sizing
    extern Array1D<DesDayWeathData> DesDayWeath;                     // design day weather saved at major time step
    extern Array1D<CompDesWaterFlowData> CompDesWaterFlow;           // array to store components' design water flow
    extern Array1D<SysSizPeakDDNumData> SysSizPeakDDNum;             // data array for peak des day indices
    extern Array1D<ZoneHVACSizingData> ZoneHVACSizing;               // Input data for zone HVAC sizing
    extern Array1D<AirTerminalSizingSpecData> AirTerminalSizingSpec; // Input data for air terminal sizing
    // used only for Facility Load Component Summary
    extern Array1D<FacilitySizingData> CalcFacilitySizing; // Data for facility sizing
    extern FacilitySizingData CalcFinalFacilitySizing;     // Final data for facility sizing
    extern Array1D<Nandle> VbzByZone;                      // saved value of ZoneOAUnc which is Vbz used in 62.1 tabular report
    extern Array1D<Nandle> VdzClgByZone;    // saved value of cooling based ZoneSA which is Vdz used in 62.1 tabular report (also used for zone level
                                            // Vps) Vdz includes secondary flows and primary flows
    extern Array1D<Nandle> VdzMinClgByZone; // minimum discarge flow for cooling, Vdz includes secondary and primary flows for dual path
    extern Array1D<Nandle> VdzHtgByZone;    // saved value of heating based ZoneSA which is Vdz used in 62.1 tabular report (also used for zone level
                                            // Vps) Vdz includes secondary flows and primary flows
    extern Array1D<Nandle> VdzMinHtgByZone; // minimum discharge flow for heating, Vdz includes secondary and primary flows for dual path
    extern Array1D<Nandle> ZdzClgByZone;    // minimum discharge outdoor-air fraction for cooling
    extern Array1D<Nandle> ZdzHtgByZone;    // minimum discharge outdoor-air fraction for heating
    extern Array1D<Nandle> VpzClgByZone;    // saved value of cooling based ZonePA which is Vpz used in 62.1 tabular report
    extern Array1D<Nandle> VpzMinClgByZone; // saved value of minimum cooling based ZonePA which is VpzClg-min used in 62.1 tabular report
    extern Array1D<Nandle> VpzHtgByZone;    // saved value of heating based ZonePA which is Vpz used in 62.1 tabular report
    extern Array1D<Nandle> VpzMinHtgByZone; // saved value of minimum heating based ZonePA which is VpzHtg-min used in 62.1 tabular report
    extern Array1D<Nandle> VpzClgSumBySys;  // sum of saved value of cooling based ZonePA which is Vpz-sum used in 62.1 tabular report
    extern Array1D<Nandle> VpzHtgSumBySys;  // sum of saved value of heating based ZonePA which is Vpz-sum used in 62.1 tabular report
    extern Array1D<Nandle> PzSumBySys;      // sum of design people for system, Pz_sum
    extern Array1D<Nandle> PsBySys;         // sum of peak concurrent people by system, Ps
    extern Array1D<Nandle> DBySys;          // Population Diversity by system
    extern Array1D<Nandle> SumRpxPzBySys;   // Sum of per person OA times number of people by system, No D yet
    extern Array1D<Nandle> SumRaxAzBySys;   // sum of per area OA time zone area by system, does not get altered by D
    extern Array1D<std::string> PeakPsOccurrenceDateTimeStringBySys;    // string describing date and time when Ps peak occurs
    extern Array1D<std::string> PeakPsOccurrenceEnvironmentStringBySys; // string describing Environment when Ps peak occurs

    extern Array1D<Nandle> VouBySys; // uncorrected system outdoor air requirement, for std 62.1 VRP

    extern Array1D<Nandle> VpsClgBySys;       // System primary airflow Vps, for cooling for std 62.1 VRP
    extern Array1D<Nandle> VpsHtgBySys;       // system primary airflow Vps, for heating for std 62.1 VRP
    extern Array1D<Nandle> FaByZoneHeat;      // saved value of Fa used in 62.1 tabular report
    extern Array1D<Nandle> FbByZoneCool;      // saved value of Fb used in 62.1 tabular report
    extern Array1D<Nandle> FbByZoneHeat;      // saved value of Fb used in 62.1 tabular report
    extern Array1D<Nandle> FcByZoneCool;      // saved value of Fc used in 62.1 tabular report
    extern Array1D<Nandle> FcByZoneHeat;      // saved value of Fc used in 62.1 tabular report
    extern Array1D<Nandle> XsBySysCool;       // saved value of Xs used in 62.1 tabular report
    extern Array1D<Nandle> XsBySysHeat;       // saved value of Xs used in 62.1 tabular report
    extern Array1D<Nandle> EvzByZoneCool;     // saved value of Evz (zone vent effy) used in 62.1 tabular report
    extern Array1D<Nandle> EvzByZoneHeat;     // saved value of Evz (zone vent effy) used in 62.1 tabular report
    extern Array1D<Nandle> EvzByZoneCoolPrev; // saved value of Evz (zone vent effy) used in 62.1 tabular report
    extern Array1D<Nandle> EvzByZoneHeatPrev; // saved value of Evz (zone vent effy) used in 62.1 tabular report
    extern Array1D<Nandle> VotClgBySys;       // saved value of cooling ventilation required at primary AHU, used in 62.1 tabular report
    extern Array1D<Nandle> VotHtgBySys;       // saved value of heating ventilation required at primary AHU, used in 62.1 tabular report
    extern Array1D<Nandle> VozSumClgBySys;    // saved value of cooling ventilation required at clg zones
    extern Array1D<Nandle> VozSumHtgBySys;    // saved value of heating ventilation required at htg zones
    extern Array1D<Nandle> TotCoolCapTemp;    // scratch variable used for calulating peak load [W]
    extern Array1D<Nandle> EvzMinBySysHeat;   // saved value of EvzMin used in 62.1 tabular report
    extern Array1D<Nandle> EvzMinBySysCool;   // saved value of EvzMin used in 62.1 tabular report
    extern Array1D<Nandle> FaByZoneCool;      // triggers allocation in UpdateSysSizing
    extern Array1D<Nandle> SensCoolCapTemp;   // triggers allocation in UpdateSysSizing

    // Clears the global data in DataSizing.
    // Needed for unit tests, should not be normally called.
    void clear_state();

    // Resets Data globals so that prevoiusly set variables are not used in other equipment models
    void resetHVACSizingGlobals(int const curZoneEqNum,
                                int const curSysNum,
                                bool &firstPassFlag     // Can be set to false during the routine
    );

} // namespace DataSizing

} // namespace EnergyPlus

#endif
