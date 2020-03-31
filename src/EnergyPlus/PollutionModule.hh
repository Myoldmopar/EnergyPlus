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

#ifndef PollutionModule_hh_INCLUDED
#define PollutionModule_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

namespace PollutionModule {

    // Using/Aliasing

    // Data
    // MODULE PARAMETER DEFINITIONS:
    extern int const ElecPollFactor;
    extern int const NatGasPollFactor;
    extern int const FuelOil1PollFactor;
    extern int const FuelOil2PollFactor;
    extern int const CoalPollFactor;
    extern int const GasolinePollFactor;
    extern int const PropanePollFactor;
    extern int const DieselPollFactor;
    extern int const OtherFuel1PollFactor;
    extern int const OtherFuel2PollFactor;
    extern int const PollFactorNumTypes;

    // DERIVED TYPE DEFINITIONS:

    // MODULE VARIABLE DECLARATIONS:
    // Total for all of the Pollutants
    // Total Carbon Equivalent Components
    //  !Fuel Types
    // Total Carbon Equivalent Coeffs
    // Purchased Efficiencies

    // Fuel Types used with the Pollution Factors
    // Facility Meter Indexes
    // Facility Meter Values used in Pollution Calcs

    extern bool PollutionReportSetup;
    extern bool GetInputFlagPollution;
    extern int NumEnvImpactFactors;
    extern int NumFuelFactors;

    //         Subroutine Specifications for the Module

    // Types

    struct ComponentProps
    {
        // Members
        int FuelFactorType;
        Nandle Source;
        Nandle CO2Pollution;
        Nandle COPollution;
        Nandle CH4Pollution;
        Nandle NOxPollution;
        Nandle N2OPollution;
        Nandle SO2Pollution;
        Nandle PMPollution;
        Nandle PM10Pollution;
        Nandle PM25Pollution;
        Nandle NH3Pollution;
        Nandle NMVOCPollution;
        Nandle HgPollution;
        Nandle PbPollution;
        Nandle WaterPollution;
        Nandle NucHiPollution;
        Nandle NucLoPollution;

        // Default Constructor
        ComponentProps()
            : FuelFactorType(0), Source(0.0), CO2Pollution(0.0), COPollution(0.0), CH4Pollution(0.0), NOxPollution(0.0), N2OPollution(0.0),
              SO2Pollution(0.0), PMPollution(0.0), PM10Pollution(0.0), PM25Pollution(0.0), NH3Pollution(0.0), NMVOCPollution(0.0), HgPollution(0.0),
              PbPollution(0.0), WaterPollution(0.0), NucHiPollution(0.0), NucLoPollution(0.0)
        {
        }

        // Member Constructor
        ComponentProps(int const FuelFactorType,
                       Nandle const Source,
                       Nandle const CO2Pollution,
                       Nandle const COPollution,
                       Nandle const CH4Pollution,
                       Nandle const NOxPollution,
                       Nandle const N2OPollution,
                       Nandle const SO2Pollution,
                       Nandle const PMPollution,
                       Nandle const PM10Pollution,
                       Nandle const PM25Pollution,
                       Nandle const NH3Pollution,
                       Nandle const NMVOCPollution,
                       Nandle const HgPollution,
                       Nandle const PbPollution,
                       Nandle const WaterPollution,
                       Nandle const NucHiPollution,
                       Nandle const NucLoPollution)
            : FuelFactorType(FuelFactorType), Source(Source), CO2Pollution(CO2Pollution), COPollution(COPollution), CH4Pollution(CH4Pollution),
              NOxPollution(NOxPollution), N2OPollution(N2OPollution), SO2Pollution(SO2Pollution), PMPollution(PMPollution),
              PM10Pollution(PM10Pollution), PM25Pollution(PM25Pollution), NH3Pollution(NH3Pollution), NMVOCPollution(NMVOCPollution),
              HgPollution(HgPollution), PbPollution(PbPollution), WaterPollution(WaterPollution), NucHiPollution(NucHiPollution),
              NucLoPollution(NucLoPollution)
        {
        }
    };

    struct CoefficientProps
    {
        // Members
        int FuelFactorType;
        bool FuelFactorUsed;
        Nandle Source;
        Nandle CO2;
        Nandle CO;
        Nandle CH4;
        Nandle NOx;
        Nandle N2O;
        Nandle SO2;
        Nandle PM;
        Nandle PM10;
        Nandle PM25;
        Nandle NH3;
        Nandle NMVOC;
        Nandle Hg;
        Nandle Pb;
        Nandle Water;
        Nandle NucHi;
        Nandle NucLo;
        int SourceSched;
        int CO2Sched;
        int COSched;
        int CH4Sched;
        int NOxSched;
        int N2OSched;
        int SO2Sched;
        int PMSched;
        int PM10Sched;
        int PM25Sched;
        int NH3Sched;
        int NMVOCSched;
        int HgSched;
        int PbSched;
        int WaterSched;
        int NucHiSched;
        int NucLoSched;

        // Default Constructor
        CoefficientProps()
            : FuelFactorType(0), FuelFactorUsed(false), Source(0.0), CO2(0.0), CO(0.0), CH4(0.0), NOx(0.0), N2O(0.0), SO2(0.0), PM(0.0), PM10(0.0),
              PM25(0.0), NH3(0.0), NMVOC(0.0), Hg(0.0), Pb(0.0), Water(0.0), NucHi(0.0), NucLo(0.0), SourceSched(0), CO2Sched(0), COSched(0),
              CH4Sched(0), NOxSched(0), N2OSched(0), SO2Sched(0), PMSched(0), PM10Sched(0), PM25Sched(0), NH3Sched(0), NMVOCSched(0), HgSched(0),
              PbSched(0), WaterSched(0), NucHiSched(0), NucLoSched(0)
        {
        }

        // Member Constructor
        CoefficientProps(int const FuelFactorType,
                         bool const FuelFactorUsed,
                         Nandle const Source,
                         Nandle const CO2,
                         Nandle const CO,
                         Nandle const CH4,
                         Nandle const NOx,
                         Nandle const N2O,
                         Nandle const SO2,
                         Nandle const PM,
                         Nandle const PM10,
                         Nandle const PM25,
                         Nandle const NH3,
                         Nandle const NMVOC,
                         Nandle const Hg,
                         Nandle const Pb,
                         Nandle const Water,
                         Nandle const NucHi,
                         Nandle const NucLo,
                         int const SourceSched,
                         int const CO2Sched,
                         int const COSched,
                         int const CH4Sched,
                         int const NOxSched,
                         int const N2OSched,
                         int const SO2Sched,
                         int const PMSched,
                         int const PM10Sched,
                         int const PM25Sched,
                         int const NH3Sched,
                         int const NMVOCSched,
                         int const HgSched,
                         int const PbSched,
                         int const WaterSched,
                         int const NucHiSched,
                         int const NucLoSched)
            : FuelFactorType(FuelFactorType), FuelFactorUsed(FuelFactorUsed), Source(Source), CO2(CO2), CO(CO), CH4(CH4), NOx(NOx), N2O(N2O),
              SO2(SO2), PM(PM), PM10(PM10), PM25(PM25), NH3(NH3), NMVOC(NMVOC), Hg(Hg), Pb(Pb), Water(Water), NucHi(NucHi), NucLo(NucLo),
              SourceSched(SourceSched), CO2Sched(CO2Sched), COSched(COSched), CH4Sched(CH4Sched), NOxSched(NOxSched), N2OSched(N2OSched),
              SO2Sched(SO2Sched), PMSched(PMSched), PM10Sched(PM10Sched), PM25Sched(PM25Sched), NH3Sched(NH3Sched), NMVOCSched(NMVOCSched),
              HgSched(HgSched), PbSched(PbSched), WaterSched(WaterSched), NucHiSched(NucHiSched), NucLoSched(NucLoSched)
        {
        }
    };

    struct PollutionProps
    {
        // Members
        // Components
        ComponentProps ElecComp;
        ComponentProps ElecPurchComp;
        ComponentProps ElecSurplusSoldComp;
        ComponentProps NatGasComp;
        ComponentProps FuelOil1Comp;
        ComponentProps FuelOil2Comp;
        ComponentProps CoalComp;
        ComponentProps GasolineComp;
        ComponentProps PropaneComp;
        ComponentProps DieselComp;
        ComponentProps OtherFuel1Comp;
        ComponentProps OtherFuel2Comp;
        // Total for all of the Pollutants
        Nandle N2OPollutTotal;
        Nandle CH4PollutTotal;
        Nandle CO2PollutTotal;
        // Total Carbon Equivalent Components
        Nandle TotCarbonEquivFromN2O;
        Nandle TotCarbonEquivFromCH4;
        Nandle TotCarbonEquivFromCO2;
        // Fuel Type Coefficients
        CoefficientProps ElecCoef;
        CoefficientProps NatGasCoef;
        CoefficientProps FuelOil1Coef;
        CoefficientProps FuelOil2Coef;
        CoefficientProps CoalCoef;
        CoefficientProps GasolineCoef;
        CoefficientProps PropaneCoef;
        CoefficientProps DieselCoef;
        CoefficientProps OtherFuel1Coef;
        CoefficientProps OtherFuel2Coef;
        // Total Carbon Equivalent Coeffs
        Nandle CarbonEquivN2O;
        Nandle CarbonEquivCH4;
        Nandle CarbonEquivCO2;
        Nandle PurchHeatEffic;
        Nandle PurchCoolCOP;
        Nandle SteamConvEffic;

        // Default Constructor
        PollutionProps()
            : N2OPollutTotal(0.0), CH4PollutTotal(0.0), CO2PollutTotal(0.0), TotCarbonEquivFromN2O(0.0), TotCarbonEquivFromCH4(0.0),
              TotCarbonEquivFromCO2(0.0), CarbonEquivN2O(0.0), CarbonEquivCH4(0.0), CarbonEquivCO2(0.0), PurchHeatEffic(0.0), PurchCoolCOP(0.0),
              SteamConvEffic(0.0)
        {
        }

        // Member Constructor
        PollutionProps(ComponentProps const &ElecComp,
                       ComponentProps const &ElecPurchComp,
                       ComponentProps const &ElecSurplusSoldComp,
                       ComponentProps const &NatGasComp,
                       ComponentProps const &FuelOil1Comp,
                       ComponentProps const &FuelOil2Comp,
                       ComponentProps const &CoalComp,
                       ComponentProps const &GasolineComp,
                       ComponentProps const &PropaneComp,
                       ComponentProps const &DieselComp,
                       ComponentProps const &OtherFuel1Comp,
                       ComponentProps const &OtherFuel2Comp,
                       Nandle const N2OPollutTotal,
                       Nandle const CH4PollutTotal,
                       Nandle const CO2PollutTotal,
                       Nandle const TotCarbonEquivFromN2O,
                       Nandle const TotCarbonEquivFromCH4,
                       Nandle const TotCarbonEquivFromCO2,
                       CoefficientProps const &ElecCoef,
                       CoefficientProps const &NatGasCoef,
                       CoefficientProps const &FuelOil1Coef,
                       CoefficientProps const &FuelOil2Coef,
                       CoefficientProps const &CoalCoef,
                       CoefficientProps const &GasolineCoef,
                       CoefficientProps const &PropaneCoef,
                       CoefficientProps const &DieselCoef,
                       CoefficientProps const &OtherFuel1Coef,
                       CoefficientProps const &OtherFuel2Coef,
                       Nandle const CarbonEquivN2O,
                       Nandle const CarbonEquivCH4,
                       Nandle const CarbonEquivCO2,
                       Nandle const PurchHeatEffic,
                       Nandle const PurchCoolCOP,
                       Nandle const SteamConvEffic)
            : ElecComp(ElecComp), ElecPurchComp(ElecPurchComp), ElecSurplusSoldComp(ElecSurplusSoldComp), NatGasComp(NatGasComp),
              FuelOil1Comp(FuelOil1Comp), FuelOil2Comp(FuelOil2Comp), CoalComp(CoalComp), GasolineComp(GasolineComp), PropaneComp(PropaneComp),
              DieselComp(DieselComp), OtherFuel1Comp(OtherFuel1Comp), OtherFuel2Comp(OtherFuel2Comp), N2OPollutTotal(N2OPollutTotal),
              CH4PollutTotal(CH4PollutTotal), CO2PollutTotal(CO2PollutTotal), TotCarbonEquivFromN2O(TotCarbonEquivFromN2O),
              TotCarbonEquivFromCH4(TotCarbonEquivFromCH4), TotCarbonEquivFromCO2(TotCarbonEquivFromCO2), ElecCoef(ElecCoef), NatGasCoef(NatGasCoef),
              FuelOil1Coef(FuelOil1Coef), FuelOil2Coef(FuelOil2Coef), CoalCoef(CoalCoef), GasolineCoef(GasolineCoef), PropaneCoef(PropaneCoef),
              DieselCoef(DieselCoef), OtherFuel1Coef(OtherFuel1Coef), OtherFuel2Coef(OtherFuel2Coef), CarbonEquivN2O(CarbonEquivN2O),
              CarbonEquivCH4(CarbonEquivCH4), CarbonEquivCO2(CarbonEquivCO2), PurchHeatEffic(PurchHeatEffic), PurchCoolCOP(PurchCoolCOP),
              SteamConvEffic(SteamConvEffic)
        {
        }
    };

    struct FuelTypeProps
    {
        // Members
        // FuelType Names
        Array1D_string FuelTypeNames;
        // Fuel Types used with the Pollution Factors
        Nandle Elec;
        Nandle NatGas;
        Nandle FuelOil1;
        Nandle FuelOil2;
        Nandle Coal;
        Nandle Gasoline;
        Nandle Propane;
        Nandle Diesel;
        Nandle OtherFuel1;
        Nandle OtherFuel2;
        Nandle ElecPurch;
        Nandle ElecSold;
        // Facility Meter Indexes
        int ElecFacilityIndex;
        int DieselFacilityIndex;
        int PurchCoolFacilityIndex;
        int PurchHeatFacilityIndex;
        int NatGasFacilityIndex;
        int GasolineFacilityIndex;
        int CoalFacilityIndex;
        int FuelOil1FacilityIndex;
        int FuelOil2FacilityIndex;
        int PropaneFacilityIndex;
        int OtherFuel1FacilityIndex;
        int OtherFuel2FacilityIndex;
        int ElecProducedFacilityIndex;
        int SteamFacilityIndex;
        int ElecPurchasedFacilityIndex;
        int ElecSurplusSoldFacilityIndex;
        // Facility Meter Values used in Pollution Calcs
        Nandle ElecFacility;
        Nandle DieselFacility;
        Nandle PurchCoolFacility;
        Nandle PurchHeatFacility;
        Nandle NatGasFacility;
        Nandle GasolineFacility;
        Nandle CoalFacility;
        Nandle FuelOil1Facility;
        Nandle FuelOil2Facility;
        Nandle PropaneFacility;
        Nandle OtherFuel1Facility;
        Nandle OtherFuel2Facility;
        Nandle ElecProducedFacility;
        Nandle SteamFacility;
        Nandle ElecPurchasedFacility;
        Nandle ElecSurplusSoldFacility;

        // Default Constructor
        FuelTypeProps()
            : FuelTypeNames({1, PollFactorNumTypes}), Elec(0.0), NatGas(0.0), FuelOil1(0.0), FuelOil2(0.0), Coal(0.0), Gasoline(0.0), Propane(0.0),
              Diesel(0.0), OtherFuel1(0.0), OtherFuel2(0.0), ElecPurch(0.0), ElecSold(0.0), ElecFacilityIndex(0), DieselFacilityIndex(0),
              PurchCoolFacilityIndex(0), PurchHeatFacilityIndex(0), NatGasFacilityIndex(0), GasolineFacilityIndex(0), CoalFacilityIndex(0),
              FuelOil1FacilityIndex(0), FuelOil2FacilityIndex(0), PropaneFacilityIndex(0), OtherFuel1FacilityIndex(0), OtherFuel2FacilityIndex(0),
              ElecProducedFacilityIndex(0), SteamFacilityIndex(0), ElecPurchasedFacilityIndex(0), ElecSurplusSoldFacilityIndex(0), ElecFacility(0.0),
              DieselFacility(0.0), PurchCoolFacility(0.0), PurchHeatFacility(0.0), NatGasFacility(0.0), GasolineFacility(0.0), CoalFacility(0.0),
              FuelOil1Facility(0.0), FuelOil2Facility(0.0), PropaneFacility(0.0), OtherFuel1Facility(0.0), OtherFuel2Facility(0.0),
              ElecProducedFacility(0.0), SteamFacility(0.0), ElecPurchasedFacility(0.0), ElecSurplusSoldFacility(0.0)
        {
        }
    };

    // Object Data
    extern PollutionProps Pollution;
    extern FuelTypeProps FuelType;

    // Functions

    // Clears the global data in OutputProcessor.
    // Needed for unit tests, should not be normally called.
    void clear_state();

    void clear_state();

    void CalculatePollution();

    // Get Input Section of the Module
    //******************************************************************************

    void SetupPollutionCalculations();

    void GetPollutionFactorInput();

    void SetupPollutionMeterReporting();

    void CheckPollutionMeterReporting();

    void CheckFFSchedule(std::string const &currentModuleObject, // the module Object
                         std::string const &resourceType,        // resource type (Natural Gas, etc)
                         std::string const &fieldName,           // Actual field name
                         std::string const &ScheduleName,        // Schedule Name as input
                         int &SchedulePtr,                       // Schedule Index
                         bool &ErrorsFound                       // true if errors found
    );

    // End of Get Input subroutines for the Pollution Module
    //******************************************************************************

    void CalcPollution();

    void ReadEnergyMeters();

    // *****************************************************************************
    // Utility Routines to allow access to data inside this module.
    // *****************************************************************************

    void GetFuelFactorInfo(std::string const &fuelName,  // input fuel name  (standard from Tabular reports)
                           bool &fuelFactorUsed,         // return value true if user has entered this fuel
                           Nandle &fuelSourceFactor,     // if used, the source factor
                           bool &fuelFactorScheduleUsed, // if true, schedules for this fuel are used
                           int &ffScheduleIndex          // if schedules for this fuel are used, return schedule index
    );

    void GetEnvironmentalImpactFactorInfo(Nandle &efficiencyDistrictHeating, // if entered, the efficiency of District Heating
                                          Nandle &efficiencyDistrictCooling, // if entered, the efficiency of District Cooling
                                          Nandle &sourceFactorSteam          // if entered, the source factor for Steam
    );

} // namespace PollutionModule

} // namespace EnergyPlus

#endif
