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

#ifndef ZoneTempPredictorCorrector_hh_INCLUDED
#define ZoneTempPredictorCorrector_hh_INCLUDED

// C++ Headers
#include <vector>

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>
#include <ObjexxFCL/Array2D.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {
    class OutputFiles;

namespace ZoneTempPredictorCorrector {

    // Using/Aliasing

    // Data
    // MODULE PARAMETER DEFINITIONS:
    // Controls for PredictorCorrector
    // INTEGER, PUBLIC, PARAMETER :: iGetZoneSetPoints             = 1
    // INTEGER, PUBLIC, PARAMETER :: iPredictStep                  = 2
    // INTEGER, PUBLIC, PARAMETER :: iCorrectStep                  = 3
    // INTEGER, PUBLIC, PARAMETER :: iRevertZoneTimestepHistories  = 4
    // INTEGER, PUBLIC, PARAMETER :: iPushZoneTimestepHistories    = 5
    // INTEGER, PUBLIC, PARAMETER :: iPushSystemTimestepHistories  = 6

    extern Array1D_string const ValidControlTypes;

    extern Array1D_string const ValidComfortControlTypes;

    extern Array1D_string const cZControlTypes;

    extern int const iZC_TStat;
    extern int const iZC_TCTStat;
    extern int const iZC_OTTStat;
    extern int const iZC_HStat;
    extern int const iZC_TandHStat;
    extern int const iZC_StagedDual;
    extern Array1D_int const iZControlTypes;

    extern int const SglHeatSetPoint;
    extern int const SglCoolSetPoint;
    extern int const SglHCSetPoint;
    extern int const DualSetPoint;
    extern int const SglHeatSetPointFanger;
    extern int const SglCoolSetPointFanger;
    extern int const SglHCSetPointFanger;
    extern int const DualSetPointFanger;
    extern int const SglHeatSetPointPierce;
    extern int const SglCoolSetPointPierce;
    extern int const SglHCSetPointPierce;
    extern int const DualSetPointPierce;
    extern int const SglHeatSetPointKSU;
    extern int const SglCoolSetPointKSU;
    extern int const SglHCSetPointKSU;
    extern int const DualSetPointKSU;

    // Average method parameter with multiple people objects in a zone
    extern int const AverageMethodNum_NO;  // No multiple people objects
    extern int const AverageMethodNum_SPE; // Specific people object
    extern int const AverageMethodNum_OBJ; // People object average
    extern int const AverageMethodNum_PEO; // People number average

    // DERIVED TYPE DEFINITIONS:

    // INTERFACE BLOCK SPECIFICATIONS:
    // na

    // MODULE VARIABLE DECLARATIONS:

    extern int NumSingleTempHeatingControls;
    extern int NumSingleTempCoolingControls;
    extern int NumSingleTempHeatCoolControls;
    extern int NumDualTempHeatCoolControls;

    // Number of Thermal comfort control types
    extern int NumSingleFangerHeatingControls;
    extern int NumSingleFangerCoolingControls;
    extern int NumSingleFangerHeatCoolControls;
    extern int NumDualFangerHeatCoolControls;

    // Number of zone with staged controlled objects
    extern int NumStageCtrZone;
    // Number of zone with onoff thermostat
    extern int NumOnOffCtrZone;

    extern Array1D<Nandle> ZoneSetPointLast;
    extern Array1D<Nandle> TempIndZnLd;
    extern Array1D<Nandle> TempDepZnLd;
    extern Array1D<Nandle> ZoneAirRelHum; // Zone relative humidity in percent

    // Zone temperature history - used only for oscillation test
    extern Array2D<Nandle> ZoneTempHist;
    extern Array1D<Nandle> ZoneTempOscillate;
    extern Array1D<Nandle> ZoneTempOscillateDuringOccupancy;
    extern Array1D<Nandle> ZoneTempOscillateInDeadband;
    extern Nandle AnyZoneTempOscillate;
    extern Nandle AnyZoneTempOscillateDuringOccupancy;
    extern Nandle AnyZoneTempOscillateInDeadband;
    extern Nandle AnnualAnyZoneTempOscillate;
    extern Nandle AnnualAnyZoneTempOscillateDuringOccupancy;
    extern Nandle AnnualAnyZoneTempOscillateInDeadband;
    extern bool OscillationVariablesNeeded;

    // SUBROUTINE SPECIFICATIONS:

    // Types

    struct ZoneTempControlType
    {
        // Members
        std::string Name;          // Name of the zone
        std::string TempSchedName; // Name of the schedule which determines the zone temp setpoint
        int TempSchedIndex;
        std::string HeatTempSetptSchedName;
        int HeatTempSchedIndex;
        std::string CoolTempSetptSchedName;
        int CoolTempSchedIndex;

        // Default Constructor
        ZoneTempControlType() : TempSchedIndex(0), HeatTempSchedIndex(0), CoolTempSchedIndex(0)
        {
        }
    };

    struct ZoneComfortFangerControlType
    {
        // Members
        std::string Name;                  // Name of the zone
        std::string PMVSchedName;          // Name of the schedule which determines the zone temp setpoint
        int PMVSchedIndex;                 // Index to PMV dual set point schedule
        std::string HeatPMVSetptSchedName; // Name of PMV heating set point schedule
        int HeatPMVSchedIndex;             // Index to PMV heating set point schedule
        std::string CoolPMVSetptSchedName; // Name of PMV cooling set point schedule
        int CoolPMVSchedIndex;             // INdex to PMV cooling set point schedule

        // Default Constructor
        ZoneComfortFangerControlType() : PMVSchedIndex(0), HeatPMVSchedIndex(0), CoolPMVSchedIndex(0)
        {
        }
    };

    struct AdaptiveComfortDailySetPointSchedule
    {
        // Members
        bool initialized;
        Array1D<Nandle> ThermalComfortAdaptiveASH55_Upper_90;
        Array1D<Nandle> ThermalComfortAdaptiveASH55_Upper_80;
        Array1D<Nandle> ThermalComfortAdaptiveASH55_Central;
        Array1D<Nandle> ThermalComfortAdaptiveCEN15251_Upper_I;
        Array1D<Nandle> ThermalComfortAdaptiveCEN15251_Upper_II;
        Array1D<Nandle> ThermalComfortAdaptiveCEN15251_Upper_III;
        Array1D<Nandle> ThermalComfortAdaptiveCEN15251_Central;

        // Default Constructor
        AdaptiveComfortDailySetPointSchedule() : initialized(false)
        {
        }
    };

    // Object Data
    extern Array1D<ZoneTempControlType> SetPointSingleHeating;
    extern Array1D<ZoneTempControlType> SetPointSingleCooling;
    extern Array1D<ZoneTempControlType> SetPointSingleHeatCool;
    extern Array1D<ZoneTempControlType> SetPointDualHeatCool;
    extern Array1D<ZoneComfortFangerControlType> SetPointSingleHeatingFanger;
    extern Array1D<ZoneComfortFangerControlType> SetPointSingleCoolingFanger;
    extern Array1D<ZoneComfortFangerControlType> SetPointSingleHeatCoolFanger;
    extern Array1D<ZoneComfortFangerControlType> SetPointDualHeatCoolFanger;
    extern AdaptiveComfortDailySetPointSchedule AdapComfortDailySetPointSchedule;
    extern Array1D<Nandle> AdapComfortSetPointSummerDesDay;

    // Functions
    void clear_state();

    void ManageZoneAirUpdates(int const UpdateType,   // Can be iGetZoneSetPoints, iPredictStep, iCorrectStep
                              Nandle &ZoneTempChange, // Temp change in zone air btw previous and current timestep
                              bool const ShortenTimeStepSys,
                              bool const UseZoneTimeStepHistory, // if true then use zone timestep history, if false use system time step
                              Nandle const PriorTimeStep         // the old value for timestep length is passed for possible use in interpolating
    );

    void GetZoneAirSetPoints(OutputFiles &outputFiles);

    void InitZoneAirSetPoints();

    void PredictSystemLoads(bool const ShortenTimeStepSys,
                            bool const UseZoneTimeStepHistory, // if true then use zone timestep history, if false use system time step
                            Nandle const PriorTimeStep         // the old value for timestep length is passed for possible use in interpolating
    );

    void CalcZoneAirTempSetPoints();

    void CalculateMonthlyRunningAverageDryBulb(Array1D<Nandle> &runningAverageASH, Array1D<Nandle> &runningAverageCEN);

    void CalculateAdaptiveComfortSetPointSchl(Array1D<Nandle> const &runningAverageASH, Array1D<Nandle> const &runningAverageCEN);

    void CalcPredictedSystemLoad(int const ZoneNum, Nandle RAFNFrac);

    void CalcPredictedHumidityRatio(int const ZoneNum, Nandle RAFNFrac);

    void ReportMoistLoadsZoneMultiplier(Nandle &TotalLoad,
                                        Nandle &TotalHumidLoad,
                                        Nandle &TotalDehumidLoad,
                                        Nandle &MoistLoadSingleZone,
                                        Nandle &MoistLoadHumidSingleZone,
                                        Nandle &MoistLoadDehumidSingleZone,
                                        Nandle const ZoneMultiplier,
                                        Nandle const ZoneMultiplierList
    );

    void CorrectZoneAirTemp(Nandle &ZoneTempChange, // Temperature change in zone air between previous and current timestep
                            bool const ShortenTimeStepSys,
                            bool const UseZoneTimeStepHistory, // if true then use zone timestep history, if false use system time step history
                            Nandle const PriorTimeStep         // the old value for timestep length is passed for possible use in interpolating
    );

    void PushZoneTimestepHistories();

    void PushSystemTimestepHistories();

    void RevertZoneTimestepHistories();

    void CorrectZoneHumRat(int const ZoneNum);

    void DownInterpolate4HistoryValues(Nandle const OldTimeStep,
                                       Nandle const NewTimeStep,
                                       Nandle &oldVal0,
                                       Nandle &oldVal1,
                                       Nandle &oldVal2,
                                       Nandle &oldVal3,
                                       Nandle &oldVal4,
                                       Nandle &newVal0,
                                       Nandle &newVal1,
                                       Nandle &newVal2,
                                       Nandle &newVal3, // unused 1208
                                       Nandle &newVal4  // unused 1208
    );

    void InverseModelTemperature(int const ZoneNum,              // Zone number
                                 Nandle &SumIntGain,             // Zone sum of convective internal gains
                                 Nandle &SumIntGainExceptPeople, // Zone sum of convective internal gains except for people
                                 Nandle &SumHA,                  // Zone sum of Hc*Area
                                 Nandle &SumHATsurf,             // Zone sum of Hc*Area*Tsurf
                                 Nandle &SumHATref,              // Zone sum of Hc*Area*Tref, for ceiling diffuser convection correlation
                                 Nandle &SumMCp,                 // Zone sum of MassFlowRate*Cp
                                 Nandle &SumMCpT,                // Zone sum of MassFlowRate*Cp*T
                                 Nandle &SumSysMCp,              // Zone sum of air system MassFlowRate*Cp
                                 Nandle &SumSysMCpT,             // Zone sum of air system MassFlowRate*Cp*T
                                 Nandle &AirCap                  // Formerly CoefAirrat, coef in zone temp eqn with dim of "air power capacity"rd
    );

    void InverseModelHumidity(int const ZoneNum,              // Zone number
                              Nandle &LatentGain,             // Zone sum of latent gain
                              Nandle &LatentGainExceptPeople, // Zone sum of latent gain except for people
                              Nandle &ZoneMassFlowRate,       // Zone air mass flow rate
                              Nandle &MoistureMassFlowRate,   // Zone moisture mass flow rate
                              Nandle &H2OHtOfVap,             // Heat of vaporization of air
                              Nandle &RhoAir                  // Air density
    );

    void CalcZoneSums(int const ZoneNum,  // Zone number
                      Nandle &SumIntGain, // Zone sum of convective internal gains
                      Nandle &SumHA,      // Zone sum of Hc*Area
                      Nandle &SumHATsurf, // Zone sum of Hc*Area*Tsurf
                      Nandle &SumHATref,  // Zone sum of Hc*Area*Tref, for ceiling diffuser convection correlation
                      Nandle &SumMCp,     // Zone sum of MassFlowRate*Cp
                      Nandle &SumMCpT,    // Zone sum of MassFlowRate*Cp*T
                      Nandle &SumSysMCp,  // Zone sum of air system MassFlowRate*Cp
                      Nandle &SumSysMCpT  // Zone sum of air system MassFlowRate*Cp*T
    );

    void CalcZoneComponentLoadSums(int const ZoneNum,        // Zone number
                                   Nandle const TempDepCoef, // Dependent coefficient
                                   Nandle const TempIndCoef, // Independent coefficient
                                   Nandle &SumIntGains,      // Zone sum of convective internal gains
                                   Nandle &SumHADTsurfs,     // Zone sum of Hc*Area*(Tsurf - Tz)
                                   Nandle &SumMCpDTzones,    // zone sum of MassFlowRate*cp*(TremotZone - Tz) transfer air from other zone, Mixing
                                   Nandle &SumMCpDtInfil,    // Zone sum of MassFlowRate*Cp*(Tout - Tz) transfer from outside, ventil, earth tube
                                   Nandle &SumMCpDTsystem,   // Zone sum of air system MassFlowRate*Cp*(Tsup - Tz)
                                   Nandle &SumNonAirSystem,  // Zone sum of non air system convective heat gains
                                   Nandle &CzdTdt,           // Zone air energy storage term.
                                   Nandle &imBalance,        // put all terms in eq. 5 on RHS , should be zero
                                   Nandle &SumEnthalpyM,     // Zone sum of phase change material melting enthlpy
                                   Nandle &SumEnthalpyH      // Zone sum of phase change material freezing enthalpy
        );

    bool VerifyThermostatInZone(std::string const &ZoneName); // Zone to verify

    bool VerifyControlledZoneForThermostat(std::string const &ZoneName); // Zone to verify

    void DetectOscillatingZoneTemp();

    void AdjustAirSetPointsforOpTempCntrl(int const TempControlledZoneID, int const ActualZoneNum, Nandle &ZoneAirSetPoint);

    void AdjustOperativeSetPointsforAdapComfort(int const TempControlledZoneID, Nandle &ZoneAirSetPoint);

    void CalcZoneAirComfortSetPoints();

    void GetComfortSetPoints(int const PeopleNum,
                             int const ComfortControlNum,
                             Nandle const PMVSet,
                             Nandle &Tset // drybulb setpoint temperature for a given PMV value
    );

    Nandle PMVResidual(Nandle const Tset,
                       Array1D<Nandle> const &Par // par(1) = PMV set point
    );

    void AdjustCoolingSetPointforTempAndHumidityControl(int const TempControlledZoneID,
                                                        int const ActualZoneNum // controlled zone actual zone number
    );

    void OverrideAirSetPointsforEMSCntrl();

    void FillPredefinedTableOnThermostatSetpoints();

    std::tuple<Nandle, int, std::string>
    temperatureAndCountInSch(int const &scheduleIndex, bool const &isSummer, int const &dayOfWeek, int const &hourOfDay);

} // namespace ZoneTempPredictorCorrector

} // namespace EnergyPlus

#endif
