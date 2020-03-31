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

#ifndef ThermalComfort_hh_INCLUDED
#define ThermalComfort_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Optional.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

namespace ThermalComfort {

    // Using/Aliasing

    // Data
    // MODULE PARAMETER DEFINITIONS
    extern Nandle const TAbsConv;     // Converter for absolute temperature
    extern Nandle const ActLevelConv; // Converter for activity level (1Met = 58.2 W/m2)
    extern Nandle const BodySurfArea; // Dubois body surface area of the human body (m2)
    extern Nandle const RadSurfEff;   // Fraction of surface effective for radiation
    extern Nandle const StefanBoltz;  // Stefan-Boltzmann constant (W/m2K4)

    // DERIVED TYPE DEFINITIONS

    // MODULE VARIABLE DECLARATIONS:
    extern Nandle AbsAirTemp;                // Absolute air temperature; K
    extern Nandle AbsCloSurfTemp;            // Absolute clothing surface temperature; K
    extern Nandle AbsRadTemp;                // Absolute radiant temperature; K
    extern Nandle AcclPattern;               // The pattern of acclimation
    extern Nandle ActLevel;                  // Metabolic rate; w/m2
    extern Nandle AirVel;                    // Air velocity; m/s
    extern Nandle AirTemp;                   // Air temperature; C
    extern Nandle CloBodyRat;                // Ratio of clothed body
    extern Nandle CloInsul;                  // Clothing insulation
    extern Nandle CloPermeatEff;             // Clothing permeation efficiency
    extern Nandle CloSurfTemp;               // Clothing surface temperature; K
    extern Nandle CloThermEff;               // The Burton thermal efficiency factor for clothing
    extern Nandle CloUnit;                   // Clothing unit; CLO
    extern Nandle ConvHeatLoss;              // Convective heat loss
    extern Nandle CoreTempChange;            // Temperature change of core in 1 minute
    extern Nandle CoreTemp;                  // Body core temperature
    extern Nandle CoreTempNeut;              // Body core temperature of neutral state
    extern Nandle CoreThermCap;              // Thermal capacity of core
    extern Nandle DryHeatLoss;               // Heat loss from clothing surface due to both convection and radiation
    extern Nandle DryRespHeatLoss;           // Dry respiration heat loss
    extern Nandle EvapHeatLoss;              // Evaporative heat loss from skin
    extern Nandle EvapHeatLossDiff;          // Evaporative heat loss due to moisture diffusion through skin
    extern Nandle EvapHeatLossMax;           // Maximum evaporative heat loss
    extern Nandle EvapHeatLossRegComf;       // Evaporative heat loss due to regulatory sweating at the state of comfort
    extern Nandle EvapHeatLossRegSweat;      // Evaporative heat loss from regulatory sweating
    extern Nandle EvapHeatLossSweat;         // Evaporative heat loss from the sweat secreted
    extern Nandle EvapHeatLossSweatPrev;     // Old value of evaporative heat loss from the sweat secreted (KSU)
    extern Nandle H;                         // Combined heat transfer coefficient
    extern Nandle Hc;                        // Convective heat transfer coeffiency
    extern Nandle HcFor;                     // Convective heat transfer coeffiency - Forced
    extern Nandle HcNat;                     // Convective heat transfer coeffiency - Natural
    extern Nandle HeatFlow;                  // Heat flow from core to skin
    extern Nandle Hr;                        // Radiant heat transfer coeffiency
    extern Nandle IntHeatProd;               // Internal heat production
    extern int IterNum;                      // Number of iteration
    extern Nandle LatRespHeatLoss;           // Latent respiration heat loss
    extern int MaxZoneNum;                   // Number of zones
    extern int MRTCalcType;                  // The type of MRT calculation (ZoneAveraged or SurfaceWeighted)
    extern Nandle OpTemp;                    // Operative temperature
    extern int PeopleNum;                    // People number
    extern Nandle RadHeatLoss;               // Radiant heat loss
    extern Nandle RadTemp;                   // Radiant temperature; C
    extern Nandle RelHum;                    // Relative humidity; Fraction
    extern Nandle RespHeatLoss;              // The rate of respiratory heat loss
    extern Nandle SatSkinVapPress;           // Saturated vapor pressure at skin temperature
    extern Nandle ShivResponse;              // Metalbolic heat production due to shivering
    extern Nandle SkinComfTemp;              // Skin temperature required to achieve thermal comfort; C
    extern Nandle SkinComfVPress;            // Saturated water vapor pressure at required skin temperature; Torr
    extern Nandle SkinTemp;                  // Skin temperature
    extern Nandle SkinTempChange;            // Temperature change of skin in 1 minute
    extern Nandle SkinTempNeut;              // Skin temperature at neutral state
    extern Nandle SkinThermCap;              // Thermal capacity of Skin
    extern Nandle SkinWetDiff;               // Skin wettedness for nonsweating portion of skin
    extern Nandle SkinWetSweat;              // Skin wettedness required to evaporate regulatory sweat
    extern Nandle SkinWetTot;                // Total skin wettedness
    extern Nandle SkinVapPress;              // Vapor pressure at skin
    extern Nandle SurfaceTemp;               // Surface temperature when MRTType is 'SurfaceWeighted'
    extern Nandle ThermCndct;                // Thermal conductance of skin
    extern Nandle ThermSensTransCoef;        // Theraml sensation coefficient for PMV
    extern Nandle Time;                      // Time, hr
    extern Nandle TimeChange;                // Change of time, hr
    extern Nandle VapPress;                  // Vapor pressure; Torr  ?? BG Oct 2005 humm, this should be kPa
    extern Nandle VasoconstrictFac;          // Constriction factor of blood vessel
    extern Nandle VasodilationFac;           // Dilation factor of blood vessel
    extern Nandle WorkEff;                   // Energy cosumption by external work; w/m2
    extern int ZoneNum;                      // Zone number
    extern Nandle TemporarySixAMTemperature; // Temperature at 6am

    // time that any zone is not comfortable based on simple ASHRAE 55 using summer clothes
    extern Nandle AnyZoneTimeNotSimpleASH55Summer;
    // time that any zone is not comfortable based on simple ASHRAE 55 using winter clothes
    extern Nandle AnyZoneTimeNotSimpleASH55Winter;
    // time that any zone is not comfortable based on simple ASHRAE 55 using summer or winter clothes
    extern Nandle AnyZoneTimeNotSimpleASH55Either;

    // time that any zone has unmet met loads
    extern Nandle AnyZoneNotMetHeating;
    extern Nandle AnyZoneNotMetCooling;
    extern Nandle AnyZoneNotMetHeatingOccupied;
    extern Nandle AnyZoneNotMetCoolingOccupied;
    extern Nandle AnyZoneNotMetOccupied;
    // total time from beginning of simulation AnyZoneTimeNotSimpleASH55
    extern Nandle TotalAnyZoneTimeNotSimpleASH55Summer;
    extern Nandle TotalAnyZoneTimeNotSimpleASH55Winter;
    extern Nandle TotalAnyZoneTimeNotSimpleASH55Either;
    // total time from beginning of simulation any zone not met
    extern Nandle TotalAnyZoneNotMetHeating;
    extern Nandle TotalAnyZoneNotMetCooling;
    extern Nandle TotalAnyZoneNotMetHeatingOccupied;
    extern Nandle TotalAnyZoneNotMetCoolingOccupied;
    extern Nandle TotalAnyZoneNotMetOccupied;
    extern Array1D<Nandle> ZoneOccHrs;
    extern bool useEpwData;
    extern Array1D<Nandle> DailyAveOutTemp;

    extern Nandle runningAverageASH;

    // Subroutine Specifications for the Thermal Comfort module

    // Types

    struct ThermalComfortDataType
    {
        // Members
        Nandle FangerPMV;
        Nandle FangerPPD;
        Nandle CloSurfTemp; // clothing surface temp from iteration in FANGER calcs
        Nandle PiercePMVET;
        Nandle PiercePMVSET;
        Nandle PierceDISC;
        Nandle PierceTSENS;
        Nandle PierceSET;
        Nandle KsuTSV;
        Nandle ThermalComfortMRT;
        Nandle ThermalComfortOpTemp;
        Nandle ClothingValue;
        int ThermalComfortAdaptiveASH5590;
        int ThermalComfortAdaptiveASH5580;
        int ThermalComfortAdaptiveCEN15251CatI;
        int ThermalComfortAdaptiveCEN15251CatII;
        int ThermalComfortAdaptiveCEN15251CatIII;
        Nandle TComfASH55;
        Nandle TComfCEN15251;
        Nandle ASHRAE55RunningMeanOutdoorTemp;
        Nandle CEN15251RunningMeanOutdoorTemp;

        // Default Constructor
        ThermalComfortDataType()
            : FangerPMV(0.0), FangerPPD(0.0), CloSurfTemp(0.0), PiercePMVET(0.0), PiercePMVSET(0.0), PierceDISC(0.0), PierceTSENS(0.0),
              PierceSET(0.0), KsuTSV(0.0), ThermalComfortMRT(0.0), ThermalComfortOpTemp(0.0), ClothingValue(0.0),
              ThermalComfortAdaptiveASH5590(0), ThermalComfortAdaptiveASH5580(0), ThermalComfortAdaptiveCEN15251CatI(0),
              ThermalComfortAdaptiveCEN15251CatII(0), ThermalComfortAdaptiveCEN15251CatIII(0), TComfASH55(0.0), TComfCEN15251(0.0),
              ASHRAE55RunningMeanOutdoorTemp(0.0), CEN15251RunningMeanOutdoorTemp(0.0)
        {
        }
    };

    struct ThermalComfortInASH55Type
    {
        // Members
        // for debugging
        // REAL(r64)    :: dCurAirTemp
        // REAL(r64)    :: dCurMeanRadiantTemp
        // REAL(r64)    :: dOperTemp
        // REAL(r64)    :: dHumidRatio
        Nandle timeNotSummer;      // time when not in summer comfort range based on ASHRAE 55 simplified
        Nandle timeNotWinter;      // time when not in winter comfort range based on ASHRAE 55 simplified
        Nandle timeNotEither;      // time when  not in summer or winter comfort range based on ASHRAE 55 simplified
        Nandle totalTimeNotSummer; // sum for simulation for summer
        Nandle totalTimeNotWinter; // sum for simulation for winter
        Nandle totalTimeNotEither; // sum for simulation for either
        bool ZoneIsOccupied;       // flag if zone has people
        int warningIndex;          // variable to store pointer to the recurring warning
        int warningIndex2;         // variable to store pointer to the recurring warning
        bool Enable55Warning;      // flag if the warning should be able to be shown if appropriate

        // Default Constructor
        ThermalComfortInASH55Type()
            : timeNotSummer(0.0), timeNotWinter(0.0), timeNotEither(0.0), totalTimeNotSummer(0.0), totalTimeNotWinter(0.0), totalTimeNotEither(0.0),
              ZoneIsOccupied(false), warningIndex(0), warningIndex2(0), Enable55Warning(false)
        {
        }
    };

    struct ThermalComfortSetPointType
    {
        // Members
        Nandle notMetHeating;
        Nandle notMetCooling;
        Nandle notMetHeatingOccupied;
        Nandle notMetCoolingOccupied;
        Nandle totalNotMetHeating;
        Nandle totalNotMetCooling;
        Nandle totalNotMetHeatingOccupied;
        Nandle totalNotMetCoolingOccupied;

        // Default Constructor
        ThermalComfortSetPointType()
            : notMetHeating(0.0), notMetCooling(0.0), notMetHeatingOccupied(0.0), notMetCoolingOccupied(0.0), totalNotMetHeating(0.0),
              totalNotMetCooling(0.0), totalNotMetHeatingOccupied(0.0), totalNotMetCoolingOccupied(0.0)
        {
        }
    };

    struct AngleFactorData
    {
        // Members
        Array1D<Nandle> AngleFactor; // Angle factor of each surface
        std::string Name;            // Angle factor list name
        Array1D_string SurfaceName;  // Names of the Surfces
        Array1D_int SurfacePtr;      // ALLOCATABLE to the names of the Surfces
        int TotAngleFacSurfaces;     // Total number of surfaces
        std::string ZoneName;        // Name of zone the system is serving
        int ZonePtr;                 // Point to this zone in the Zone derived type

        // Default Constructor
        AngleFactorData() : TotAngleFacSurfaces(0), ZonePtr(0)
        {
        }
    };

    // Object Data
    extern Array1D<ThermalComfortInASH55Type> ThermalComfortInASH55;
    extern Array1D<ThermalComfortSetPointType> ThermalComfortSetPoint;
    extern Array1D<ThermalComfortDataType> ThermalComfortData;
    extern Array1D<AngleFactorData> AngleFactorList; // Angle Factor List data for each Angle Factor List

    // Functions

    void clear_state();

    void ManageThermalComfort(bool const InitializeOnly); // when called from ZTPC and calculations aren't needed

    void InitThermalComfort();

    void CalcThermalComfortFanger(Optional_int_const PNum = _,     // People number for thermal comfort control
                                  Optional<Nandle const> Tset = _, // Temperature setpoint for thermal comfort control
                                  Optional<Nandle> PMVResult = _   // PMV value for thermal comfort control
    );

    void CalcThermalComfortPierce();

    void CalcThermalComfortKSU();

    void DERIV(int &TempIndiceNum,         // Number of temperature indices  unused1208
               Array1D<Nandle> &Temp,      // Temperature unused1208
               Array1D<Nandle> &TempChange // Change of temperature
    );

    void RKG(int &NEQ, Nandle &H, Nandle &X, Array1D<Nandle> &Y, Array1D<Nandle> &DY, Array1D<Nandle> &C);

    void GetAngleFactorList();

    Nandle CalcAngleFactorMRT(int const AngleFacNum);

    Nandle CalcSurfaceWeightedMRT(int const ZoneNum, int const SurfNum);

    Nandle CalcSatVapPressFromTemp(Nandle const Temp);

    Nandle CalcRadTemp(int const PeopleListNum); // Type of MRT calculation (zone averaged or surface weighted)

    void CalcThermalComfortSimpleASH55();

    void ResetThermalComfortSimpleASH55();

    void CalcIfSetPointMet();

    void ResetSetPointMet();

    void CalcThermalComfortAdaptiveASH55(
        bool const initiate,                  // true if supposed to initiate
        Optional_bool_const wthrsim = _,      // true if this is a weather simulation
        Optional<Nandle const> avgdrybulb = _ // approximate avg drybulb for design day.  will be used as previous period in design day
    );

    void CalcThermalComfortAdaptiveCEN15251(
        bool const initiate,                  // true if supposed to initiate
        Optional_bool_const wthrsim = _,      // true if this is a weather simulation
        Optional<Nandle const> avgdrybulb = _ // approximate avg drybulb for design day.  will be used as previous period in design day
    );

    void DynamicClothingModel();

} // namespace ThermalComfort

} // namespace EnergyPlus

#endif
