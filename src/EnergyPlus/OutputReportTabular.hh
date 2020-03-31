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

#ifndef OutputReportTabular_hh_INCLUDED
#define OutputReportTabular_hh_INCLUDED

// C++ Headers
#include <fstream>
#include <string>

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>
#include <ObjexxFCL/Array1S.hh>
#include <ObjexxFCL/Array2D.hh>
#include <ObjexxFCL/Array2S.hh>
#include <ObjexxFCL/Array3D.hh>
#include <ObjexxFCL/Optional.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>
#include <EnergyPlus/OutputProcessor.hh>

namespace EnergyPlus {

namespace OutputReportTabular {

    // Using/Aliasing

    // Data
    // MODULE PARAMETER DEFINITIONS:

    extern int const MaxHeaderLength;
    extern int const MaxNoteLength;

    extern int const aggTypeSumOrAvg;
    extern int const aggTypeMaximum;
    extern int const aggTypeMinimum;
    extern int const aggTypeValueWhenMaxMin;
    extern int const aggTypeHoursZero;
    extern int const aggTypeHoursNonZero;
    extern int const aggTypeHoursPositive;
    extern int const aggTypeHoursNonPositive;
    extern int const aggTypeHoursNegative;
    extern int const aggTypeHoursNonNegative;
    extern int const aggTypeSumOrAverageHoursShown;
    extern int const aggTypeMaximumDuringHoursShown;
    extern int const aggTypeMinimumDuringHoursShown;

    extern int const tableStyleComma;
    extern int const tableStyleTab;
    extern int const tableStyleFixed;
    extern int const tableStyleHTML;
    extern int const tableStyleXML;

    extern int const unitsStyleNone; // no change to any units
    extern int const unitsStyleJtoKWH;
    extern int const unitsStyleJtoMJ;
    extern int const unitsStyleJtoGJ;
    extern int const unitsStyleInchPound;
    extern int const unitsStyleNotFound;

    extern int const stepTypeZone;
    extern int const stepTypeHVAC;

    extern int const cSensInst;
    extern int const cSensDelay;
    extern int const cSensRA;
    extern int const cLatent;
    extern int const cTotal;
    extern int const cPerc;
    extern int const cArea;
    extern int const cPerArea;

    extern int const rPeople;
    extern int const rLights;
    extern int const rEquip;
    extern int const rRefrig;
    extern int const rWaterUse;
    extern int const rHvacLoss;
    extern int const rPowerGen;
    extern int const rDOAS;
    extern int const rInfil;
    extern int const rZoneVent;
    extern int const rIntZonMix;
    extern int const rRoof;
    extern int const rIntZonCeil;
    extern int const rOtherRoof;
    extern int const rExtWall;
    extern int const rIntZonWall;
    extern int const rGrdWall;
    extern int const rOtherWall;
    extern int const rExtFlr;
    extern int const rIntZonFlr;
    extern int const rGrdFlr;
    extern int const rOtherFlr;
    extern int const rFeneCond;
    extern int const rFeneSolr;
    extern int const rOpqDoor;
    extern int const rGrdTot;

    // BEPS Report Related Variables
    // From Report:Table:Predefined - BEPS
    extern int const numResourceTypes;
    extern int const numSourceTypes;

    // MODULE VARIABLE DECLARATIONS:

    // The Binned table type is different and only references one variable and its structure is very
    // different from the others so it is has its own type.

    // arrays for time binned results

    extern int OutputTableBinnedCount;
    extern int BinResultsTableCount;
    extern int BinResultsIntervalCount;

    extern int const numNamedMonthly;
    // These reports are detailed/named in routine InitializePredefinedMonthlyTitles

    extern int MonthlyInputCount;
    extern int sizeMonthlyInput;
    extern int MonthlyFieldSetInputCount;
    extern int sizeMonthlyFieldSetInput;
    extern int MonthlyTablesCount;
    extern int MonthlyColumnsCount;
    extern Array1D_bool IsMonthGathered; // shown as true for any month used

    extern int TOCEntriesCount;
    extern int TOCEntriesSize;

    extern int UnitConvSize;

    extern bool WriteTabularFiles;

    // Allow up to five output files to be created
    extern int const maxNumStyles;

    // From Report:Table:Style
    extern int unitsStyle; // see list of parameters
    extern int numStyles;
    extern std::ofstream csv_stream;                   // CSV table stream
    extern std::ofstream tab_stream;                   // Tab table stream
    extern std::ofstream fix_stream;                   // Fixed table stream
    extern std::ofstream htm_stream;                   // HTML table stream
    extern std::ofstream xml_stream;                   // XML table stream
    extern Array1D<std::ofstream *> TabularOutputFile; // Table stream array
    extern Array1D_string del;                         // the delimiter to use
    extern Array1D_int TableStyle;                     // see list of parameters

    extern Nandle timeInYear;

    // Flags for predefined tabular reports
    extern bool displayTabularBEPS;
    extern bool displayLEEDSummary;
    extern bool displayTabularCompCosts; // added BTG 5/6/04 for component cost summary
    extern bool displayTabularVeriSum;   // added JG 2006-06-28 for input verification and summary report
    extern bool displayComponentSizing;
    extern bool displaySurfaceShadowing;
    extern bool displayDemandEndUse;
    extern bool displayAdaptiveComfort;
    extern bool displaySourceEnergyEndUseSummary;
    extern bool displayZoneComponentLoadSummary;
    extern bool displayAirLoopComponentLoadSummary;
    extern bool displayFacilityComponentLoadSummary;
    extern bool displayLifeCycleCostReport;
    extern bool displayTariffReport;
    extern bool displayEconomicResultSummary;
    extern bool displayHeatEmissionsSummary;
    extern bool displayEioSummary;

    // BEPS Report Related Variables
    // From Report:Table:Predefined - BEPS
    // arrays that hold the meter numbers that are initialized at get input
    extern Array1D_int meterNumTotalsBEPS;
    extern Array1D_int meterNumTotalsSource;
    extern Array1D_bool fuelfactorsused;
    extern Array1D_bool ffUsed;
    extern Array1D<Nandle> SourceFactors;
    extern Array1D_bool ffSchedUsed;
    extern Array1D_int ffSchedIndex;
    extern Array2D_int meterNumEndUseBEPS;
    extern Array3D_int meterNumEndUseSubBEPS;
    // arrays that hold the names of the resource and end uses
    extern Array1D_string resourceTypeNames;
    extern Array1D_string sourceTypeNames;
    extern Array1D_string endUseNames;
    // arrays that hold the actual values for the year
    extern Array1D<Nandle> gatherTotalsBEPS;
    extern Array1D<Nandle> gatherTotalsBySourceBEPS;
    extern Array1D<Nandle> gatherTotalsSource;
    extern Array1D<Nandle> gatherTotalsBySource;
    extern Array2D<Nandle> gatherEndUseBEPS;
    extern Array2D<Nandle> gatherEndUseBySourceBEPS;
    extern Array3D<Nandle> gatherEndUseSubBEPS;
    // arrays the hold the demand values
    extern Array1D<Nandle> gatherDemandTotal;
    extern Array2D<Nandle> gatherDemandEndUse;
    extern Array2D<Nandle> gatherDemandIndEndUse;
    extern Array3D<Nandle> gatherDemandEndUseSub;
    extern Array3D<Nandle> gatherDemandIndEndUseSub;
    extern Array1D_int gatherDemandTimeStamp;
    // to keep track of hours for the BEPS report gathering
    extern Nandle gatherElapsedTimeBEPS;
    // for normalization of results
    extern Nandle buildingGrossFloorArea;
    extern Nandle buildingConditionedFloorArea;
    // keep track if schedules are used in fuel factors
    extern bool fuelFactorSchedulesUsed;
    // for electic load components on BEPS report
    extern int meterNumPowerFuelFireGen;
    extern Nandle gatherPowerFuelFireGen;
    extern int meterNumPowerPV;
    extern Nandle gatherPowerPV;
    extern int meterNumPowerWind;
    extern Nandle gatherPowerWind;
    extern Nandle OverallNetEnergyFromStorage;
    extern int meterNumPowerHTGeothermal;
    extern Nandle gatherPowerHTGeothermal;
    extern int meterNumElecProduced;
    extern Nandle gatherElecProduced;
    extern int meterNumElecPurchased;
    extern Nandle gatherElecPurchased;
    extern int meterNumElecSurplusSold;
    extern Nandle gatherElecSurplusSold;
    extern int meterNumElecStorage;
    extern Nandle gatherElecStorage;
    extern int meterNumPowerConversion;
    extern Nandle gatherPowerConversion;
    // for on site thermal source components on BEPS report
    extern int meterNumWaterHeatRecovery;
    extern Nandle gatherWaterHeatRecovery;
    extern int meterNumAirHeatRecoveryCool;
    extern Nandle gatherAirHeatRecoveryCool;
    extern int meterNumAirHeatRecoveryHeat;
    extern Nandle gatherAirHeatRecoveryHeat;
    extern int meterNumHeatHTGeothermal;
    extern Nandle gatherHeatHTGeothermal;
    extern int meterNumHeatSolarWater;
    extern Nandle gatherHeatSolarWater;
    extern int meterNumHeatSolarAir;
    extern Nandle gatherHeatSolarAir;
    // for on site water components on BEPS report
    extern int meterNumRainWater;
    extern Nandle gatherRainWater;
    extern int meterNumCondensate;
    extern Nandle gatherCondensate;
    extern int meterNumGroundwater;
    extern Nandle gatherWellwater;
    extern int meterNumMains;
    extern Nandle gatherMains;
    extern int meterNumWaterEndUseTotal;
    extern Nandle gatherWaterEndUseTotal;
    // for source energy conversion factors on BEPS report
    extern Nandle sourceFactorElectric;
    extern Nandle sourceFactorNaturalGas;
    extern Nandle efficiencyDistrictCooling;
    extern Nandle efficiencyDistrictHeating;
    extern Nandle sourceFactorSteam;
    extern Nandle sourceFactorGasoline;
    extern Nandle sourceFactorDiesel;
    extern Nandle sourceFactorCoal;
    extern Nandle sourceFactorFuelOil1;
    extern Nandle sourceFactorFuelOil2;
    extern Nandle sourceFactorPropane;
    extern Nandle sourceFactorOtherFuel1;
    extern Nandle sourceFactorOtherFuel2;

    extern Array1D_int td;
    //(1)   Current year
    //(2)   Current month
    //(3)   Current day
    //(4)   Time difference with respect to UTC in minutes (0-59)
    //(5)   Hour of the day (0-23)
    //(6)   Minutes (0-59)
    //(7)   Seconds (0-59)
    //(8)   Milliseconds (0-999)

    // Design day name storage
    extern Array1D_string DesignDayName;
    extern int DesignDayCount;

    // arrays related to pulse and load component reporting
    extern Array2D_int radiantPulseTimestep;
    extern Array2D<Nandle> radiantPulseReceived;
    extern Array3D<Nandle> loadConvectedNormal;
    extern Array3D<Nandle> loadConvectedWithPulse;
    extern Array3D<Nandle> netSurfRadSeq;
    extern Array2D<Nandle> decayCurveCool;
    extern Array2D<Nandle> decayCurveHeat;
    extern Array3D<Nandle> ITABSFseq; // used for determining the radiant fraction on each surface
    extern Array3D<Nandle> TMULTseq;  // used for determining the radiant fraction on each surface

    extern Array3D<Nandle> peopleInstantSeq;
    extern Array3D<Nandle> peopleLatentSeq;
    extern Array3D<Nandle> peopleRadSeq;
    extern Array3D<Nandle> peopleDelaySeq;

    extern Array3D<Nandle> lightInstantSeq;
    extern Array3D<Nandle> lightRetAirSeq;
    extern Array3D<Nandle> lightLWRadSeq; // long wave thermal radiation
    extern Array3D<Nandle> lightSWRadSeq; // short wave visible radiation
    extern Array3D<Nandle> lightDelaySeq;

    extern Array3D<Nandle> equipInstantSeq;
    extern Array3D<Nandle> equipLatentSeq;
    extern Array3D<Nandle> equipRadSeq;
    extern Array3D<Nandle> equipDelaySeq;

    extern Array3D<Nandle> refrigInstantSeq;
    extern Array3D<Nandle> refrigRetAirSeq;
    extern Array3D<Nandle> refrigLatentSeq;

    extern Array3D<Nandle> waterUseInstantSeq;
    extern Array3D<Nandle> waterUseLatentSeq;

    extern Array3D<Nandle> hvacLossInstantSeq;
    extern Array3D<Nandle> hvacLossRadSeq;
    extern Array3D<Nandle> hvacLossDelaySeq;

    extern Array3D<Nandle> powerGenInstantSeq;
    extern Array3D<Nandle> powerGenRadSeq;
    extern Array3D<Nandle> powerGenDelaySeq;

    extern Array3D<Nandle> infilInstantSeq;
    extern Array3D<Nandle> infilLatentSeq;

    extern Array3D<Nandle> zoneVentInstantSeq;
    extern Array3D<Nandle> zoneVentLatentSeq;

    extern Array3D<Nandle> interZoneMixInstantSeq;
    extern Array3D<Nandle> interZoneMixLatentSeq;

    extern Array3D<Nandle> feneCondInstantSeq;
    // REAL(r64), DIMENSION(:,:,:),ALLOCATABLE,PUBLIC  :: feneSolarInstantSeq
    extern Array3D<Nandle> feneSolarRadSeq;
    extern Array3D<Nandle> feneSolarDelaySeq;

    extern Array3D<Nandle> surfDelaySeq;

    extern int maxUniqueKeyCount;

    // for the XML report must keep track fo the active sub-table name and report set by other routines
    extern std::string activeSubTableName;
    extern std::string activeReportNameNoSpace;
    extern std::string activeReportName;
    extern std::string activeForName;
    extern std::string prevReportName;

    // LineTypes for reading the stat file
    enum class StatLineType {
        Initialized, // used as a dummy placeholder
        StatisticsLine,
        LocationLine,
        LatLongLine,
        ElevationLine,
        StdPressureLine,
        DataSourceLine,
        WMOStationLine,
        DesignConditionsLine,
        heatingConditionsLine,
        coolingConditionsLine,
        stdHDDLine,
        stdCDDLine,
        maxDryBulbLine,
        minDryBulbLine,
        maxDewPointLine,
        minDewPointLine,
        wthHDDLine,
        wthCDDLine,
        KoppenLine,
        KoppenDes1Line,
        KoppenDes2Line,
        AshStdLine,
        AshStdDes1Line,
        AshStdDes2Line,
        AshStdDes3Line,
    };

    // Types

    struct OutputTableBinnedType
    {
        // Members
        std::string keyValue;   // the key value (usually an asterisk to indicate all variables
        std::string varOrMeter; // the name of the variable or meter
        Nandle intervalStart;   // The lowest value for the intervals being binned into.
        Nandle intervalSize;    // The size of the bins starting with Interval start.
        int intervalCount;      // The number of bins used. The number of hours below the start of
        // the lowest bin and above the value of the last bin are also shown.
        int resIndex; // result index - pointer to BinResults array
        int numTables;
        int typeOfVar;                     // 0=not found, 1=integer, 2=real, 3=meter
        OutputProcessor::StoreType avgSum; // Variable  is Averaged=1 or Summed=2
        OutputProcessor::TimeStepType stepType;                      // Variable time step is Zone=1 or HVAC=2
        OutputProcessor::Unit units;       // the units enumeration
        std::string ScheduleName;          // the name of the schedule
        int scheduleIndex;                 // index to the schedule specified - if no schedule use zero

        // Default Constructor
        OutputTableBinnedType()
            : intervalStart(0.0), intervalSize(0.0), intervalCount(0), resIndex(0), numTables(0), typeOfVar(0),
              avgSum(OutputProcessor::StoreType::Averaged), stepType(OutputProcessor::TimeStepType::TimeStepZone), scheduleIndex(0)
        {
        }
    };

    struct BinResultsType
    {
        // Members
        Array1D<Nandle> mnth; // monthly bins
        Array1D<Nandle> hrly; // hourly bins

        // Default Constructor
        BinResultsType() : mnth(12, 0.0), hrly(24, 0.0)
        {
        }
    };

    struct BinObjVarIDType
    {
        // Members
        std::string namesOfObj; // name of the object
        int varMeterNum;        // variable or meter number

        // Default Constructor
        BinObjVarIDType() : varMeterNum(0)
        {
        }
    };

    struct BinStatisticsType
    {
        // Members
        Nandle sum;     // sum of the variable
        Nandle sum2;    // sum of the variable squared
        int n;          // number of items in sum
        Nandle minimum; // minimum value
        Nandle maximum; // maximum value

        // Default Constructor
        BinStatisticsType() : sum(0.0), sum2(0.0), n(0), minimum(0.0), maximum(0.0)
        {
        }
    };

    struct NamedMonthlyType
    {
        // Members
        std::string title; // report title
        bool show;         // if report should be shown

        // Default Constructor
        NamedMonthlyType() : show(false)
        {
        }
    };

    struct MonthlyInputType
    {
        // Members
        std::string name;  // identifier
        int numFieldSet;   // number of monthly field sets
        int firstFieldSet; // pointer to the first field set
        int numTables;     // number of tables
        int firstTable;    // pointer to the first table
        int showDigits;    // the number of digits to be shown

        // Default Constructor
        MonthlyInputType() : numFieldSet(0), firstFieldSet(0), numTables(0), firstTable(0), showDigits(0)
        {
        }
    };

    struct MonthlyFieldSetInputType
    {
        // Members
        std::string variMeter;                // the name of the variable or meter
        std::string colHead;                  // the column header to use instead of the variable name (only for predefined)
        int aggregate;                        // the type of aggregation for the variable (see aggType parameters)
        OutputProcessor::Unit varUnits;       // Units enumeration
        std::string variMeterUpper;           // the name of the variable or meter uppercased
        int typeOfVar;                        // 0=not found, 1=integer, 2=real, 3=meter
        int keyCount;                         // noel
        OutputProcessor::StoreType varAvgSum; // Variable  is Averaged=1 or Summed=2
        OutputProcessor::TimeStepType varStepType;                      // Variable time step is Zone=1 or HVAC=2
        Array1D_string NamesOfKeys;           // keyNames !noel
        Array1D_int IndexesForKeyVar;         // keyVarIndexes !noel

        // Default Constructor
        MonthlyFieldSetInputType()
            : aggregate(0), varUnits(OutputProcessor::Unit::None), typeOfVar(0), keyCount(0), varAvgSum(OutputProcessor::StoreType::Averaged),
              varStepType(OutputProcessor::TimeStepType::TimeStepZone)
        {
        }
    };

    struct MonthlyTablesType
    {
        // Members
        std::string keyValue; // the key value - the object names that result in the variable
        int firstColumn;      // pointer to the monthly column array for the first item
        int numColumns;       // number of columns for the table

        // Default Constructor
        MonthlyTablesType() : firstColumn(0), numColumns(0)
        {
        }
    };

    struct MonthlyColumnsType
    {
        // Members
        std::string varName;               // name of variable
        std::string colHead;               // column header (not used for user defined monthly)
        int varNum;                        // variable or meter number
        int typeOfVar;                     // 0=not found, 1=integer, 2=real, 3=meter
        OutputProcessor::StoreType avgSum; // Variable  is Averaged=1 or Summed=2
        OutputProcessor::TimeStepType stepType;                      // Variable time step is Zone=1 or HVAC=2
        OutputProcessor::Unit units;       // the units string, may be blank
        int aggType;                       // index to the type of aggregation (see list of parameters)
        Array1D<Nandle> reslt;             // monthly results
        Array1D<Nandle> duration;          // the time during which results are summed for use in averages
        Array1D_int timeStamp;             // encoded timestamp of max or min
        Nandle aggForStep;                 // holds the aggregation for the HVAC time steps when smaller than
        // the zone timestep

        // Default Constructor
        MonthlyColumnsType()
            : varNum(0), typeOfVar(0), avgSum(OutputProcessor::StoreType::Averaged), stepType(OutputProcessor::TimeStepType::TimeStepZone), units(OutputProcessor::Unit::None), aggType(0),
              reslt(12, 0.0), duration(12, 0.0), timeStamp(12, 0), aggForStep(0.0)
        {
        }
    };

    struct TOCEntriesType
    {
        // Members
        std::string reportName;  // the name of the individual report
        std::string sectionName; // the name of the section containing individual reports
        bool isWritten;          // flag if the entry has been written to TOC

        // Default Constructor
        TOCEntriesType() : isWritten(false)
        {
        }
    };

    struct UnitConvType
    {
        // Members
        std::string siName; // the name abbreviation or symbol of the SI units
        std::string ipName; // the name abbreviation or symbol of the IP units
        Nandle mult;        // the multiplier used to convert from SI to IP in IP = (SI * mult) + offset
        Nandle offset;      // the offset used to convert from SI to IP in IP = (SI * mult) + offset
        std::string hint;   // the string used when multiple SI units match
        bool several;       // several different options for the SI unit to be converted into IP
        bool is_default;    // if part of a set of "several" this should be used as default

        // Default Constructor
        UnitConvType() : mult(1.0), offset(0.0), several(false), is_default(false)
        {
        }
    };

    struct CompLoadTablesType
    {
        // members
        int desDayNum;             // design day number
        int timeStepMax;           // times step of the day that the maximum occurs
        Array2D<Nandle> cells;     // main component table results (column, row)
        Array2D_bool cellUsed;     // flag if the cell is used for the table of results (column, row)
        std::string peakDateHrMin; // string containing peak timestamp
        Nandle outsideDryBulb;     // outside dry bulb temperature at peak
        Nandle outsideWetBulb;     // outside wet bulb temperature at peak
        Nandle outsideHumRatio;    // outside humidity ratio at peak
        Nandle zoneDryBulb;        // zone dry bulb temperature at peak
        Nandle zoneRelHum;         // zone relative humidity at peak
        Nandle zoneHumRatio;       // zone humidity ratio at peak

        Nandle supAirTemp;     // supply air temperature
        Nandle mixAirTemp;     // mixed air temperature
        Nandle mainFanAirFlow; // main fan air flow
        Nandle outsideAirFlow; // outside air flow
        Nandle designPeakLoad; // design peak load
        Nandle diffDesignPeak; // difference between Design and Peak Load

        Nandle peakDesSensLoad;    // peak design sensible load
        Nandle estInstDelSensLoad; // estimated instant plus delayed sensible load
        Nandle diffPeakEst;        // difference between the peak design sensible load and the estimated instant plus delayed sensible load
        Array1D_int zoneIndices;   // the zone numbers covered by the report

        Nandle outsideAirRatio;   // outside Air
        Nandle floorArea;         // floor area
        Nandle airflowPerFlrArea; // airflow per floor area
        Nandle airflowPerTotCap;  // airflow per total capacity
        Nandle areaPerTotCap;     // area per total capacity
        Nandle totCapPerArea;     // total capacity per area
        Nandle chlPumpPerFlow;    // chiller pump power per flow
        Nandle cndPumpPerFlow;    // condenser pump power per flow
        Nandle numPeople;         // number of people

        // default constructor
        CompLoadTablesType()
            : desDayNum(0), timeStepMax(0), outsideDryBulb(0.), outsideWetBulb(0.), outsideHumRatio(0.), zoneDryBulb(0.), zoneRelHum(0.),
              supAirTemp(0.), mixAirTemp(0.), mainFanAirFlow(0.), outsideAirFlow(0.), designPeakLoad(0.), diffDesignPeak(0.), peakDesSensLoad(0.),
              estInstDelSensLoad(0.), diffPeakEst(0.), outsideAirRatio(0.), floorArea(0.), airflowPerFlrArea(0.), airflowPerTotCap(0.),
              areaPerTotCap(0.), totCapPerArea(0.), chlPumpPerFlow(0.), cndPumpPerFlow(0.), numPeople(0.)

        {
        }
    };

    struct ZompComponentAreasType
    {
        // members
        Nandle floor;
        Nandle roof;
        Nandle ceiling;
        Nandle extWall;
        Nandle intZoneWall;
        Nandle grndCntWall;
        Nandle extFloor;
        Nandle intZoneFloor;
        Nandle grndCntFloor;
        Nandle fenestration;
        Nandle door;

        // default constructor
        ZompComponentAreasType()
            : floor(0.), roof(0.), ceiling(0.), extWall(0.), intZoneWall(0.), grndCntWall(0.), extFloor(0.), intZoneFloor(0.), grndCntFloor(0.),
              fenestration(0.), door(0.)
        {
        }
    };

    // Object Data
    extern Array1D<OutputTableBinnedType> OutputTableBinned;
    extern Array2D<BinResultsType> BinResults;      // table number, number of intervals
    extern Array1D<BinResultsType> BinResultsBelow; // time below the lowest defined bin
    extern Array1D<BinResultsType> BinResultsAbove; // time above the highest defined bin
    extern Array1D<BinObjVarIDType> BinObjVarID;
    extern Array1D<BinStatisticsType> BinStatistics;
    extern Array1D<NamedMonthlyType> namedMonthly; // for predefined monthly report titles
    extern Array1D<MonthlyFieldSetInputType> MonthlyFieldSetInput;
    extern Array1D<MonthlyInputType> MonthlyInput;
    extern Array1D<MonthlyTablesType> MonthlyTables;
    extern Array1D<MonthlyColumnsType> MonthlyColumns;
    extern Array1D<TOCEntriesType> TOCEntries;
    extern Array1D<UnitConvType> UnitConv;

    // Functions
    void clear_state();

    void UpdateTabularReports(OutputProcessor::TimeStepType t_timeStepType); // What kind of data to update (Zone, HVAC)

    //======================================================================================================================
    //======================================================================================================================

    //    GET INPUT ROUTINES

    //======================================================================================================================
    //======================================================================================================================

    void GetInputTabularMonthly();

    int AddMonthlyReport(std::string const &inReportName, int const inNumDigitsShown);

    void AddMonthlyFieldSetInput(int const inMonthReport, std::string const &inVariMeter, std::string const &inColHead, int const inAggregate);

    void InitializeTabularMonthly();

    bool isInvalidAggregationOrder();

    void GetInputTabularTimeBins();

    bool warningAboutKeyNotFound(int foundIndex, int inObjIndex, std::string const &moduleName);

    void GetInputTabularStyle(OutputFiles &outputFiles);

    int SetUnitsStyleFromString(std::string const &unitStringIn);

    void GetInputOutputTableSummaryReports();

    bool isCompLoadRepReq();

    bool hasSizingPeriodsDays();

    void InitializePredefinedMonthlyTitles();

    void CreatePredefinedMonthlyReports();

    void GetInputFuelAndPollutionFactors();

    //======================================================================================================================
    //======================================================================================================================

    //    OTHER INITIALIZATION ROUTINES

    //======================================================================================================================
    //======================================================================================================================

    void OpenOutputTabularFile();

    void CloseOutputTabularFile();

    void WriteTableOfContents();

    //======================================================================================================================
    //======================================================================================================================

    //    GATHER DATA EACH TIME STEP ROUTINES

    //======================================================================================================================
    //======================================================================================================================

    void GatherBinResultsForTimestep(OutputProcessor::TimeStepType t_timeStepType); // What kind of data to update (Zone, HVAC)

    void GatherMonthlyResultsForTimestep(OutputProcessor::TimeStepType t_timeStepType); // What kind of data to update (Zone, HVAC)

    void GatherBEPSResultsForTimestep(OutputProcessor::TimeStepType t_timeStepType); // What kind of data to update (Zone, HVAC)

    void GatherSourceEnergyEndUseResultsForTimestep(OutputProcessor::TimeStepType t_timeStepType); // What kind of data to update (Zone, HVAC)

    void GatherPeakDemandForTimestep(OutputProcessor::TimeStepType t_timeStepType); // What kind of data to update (Zone, HVAC)

    void GatherHeatGainReport(OutputProcessor::TimeStepType t_timeStepType); // What kind of data to update (Zone, HVAC)

    void GatherHeatEmissionReport(OutputProcessor::TimeStepType t_timeStepType);

    //======================================================================================================================
    //======================================================================================================================

    //    WRITE OUTPUT FILE ROUTINES

    //======================================================================================================================
    //======================================================================================================================

    void WriteTabularReports();

    void parseStatLine(const std::string & lineIn, StatLineType &lineType, bool & desConditionlinepassed, bool & heatingDesignlinepassed, bool & coolingDesignlinepassed, bool & isKoppen);

    void FillWeatherPredefinedEntries();

    std::string GetColumnUsingTabs(std::string const &inString, // Input String
                                   int const colNum             // Column number
    );

    void FillRemainingPredefinedEntries();

    void WriteMonthlyTables();

    void WriteTimeBinTables();

    void WriteBEPSTable();

    std::string ResourceWarningMessage(std::string resource);

    Nandle WaterConversionFunct(Nandle WaterTotal, Nandle ConversionFactor);

    void WriteSourceEnergyEndUseSummary();

    void WriteDemandEndUseSummary();

    void WriteCompCostTable();

    void WriteVeriSumTable();

    void WriteAdaptiveComfortTable();

    void WriteHeatEmissionTable();

    void WritePredefinedTables();

    void WriteComponentSizing();

    void WriteSurfaceShadowing();

    void WriteEioTables(OutputFiles &outputFiles);

    int unitsFromHeading(std::string &heading);

    std::vector<std::string> splitCommaString(std::string const &inputString);

    void AddTOCLoadComponentTableSummaries();

    void AllocateLoadComponentArrays();

    void DeallocateLoadComponentArrays();

    void ComputeLoadComponentDecayCurve(OutputFiles &outputFiles);

    void GatherComponentLoadsSurface();

    void GatherComponentLoadsHVAC();

    void WriteLoadComponentSummaryTables();

    void GetDelaySequences(int const &desDaySelected,
                           bool const &isCooling,
                           int const &zoneIndex,
                           Array1D<Nandle> &peopleDelaySeq,
                           Array1D<Nandle> &equipDelaySeq,
                           Array1D<Nandle> &hvacLossDelaySeq,
                           Array1D<Nandle> &powerGenDelaySeq,
                           Array1D<Nandle> &lightDelaySeq,
                           Array1D<Nandle> &feneSolarDelaySeq,
                           Array3D<Nandle> &feneCondInstantSeq,
                           Array2D<Nandle> &surfDelaySeq);

    Nandle MovingAvgAtMaxTime(Array1S<Nandle> const &dataSeq, int const &numTimeSteps, int const &maxTimeStep);

    void ComputeTableBodyUsingMovingAvg(Array2D<Nandle> &resultCells,
                                        Array2D_bool &resultCellsUsed,
                                        int const &desDaySelected,
                                        int const &timeOfMax,
                                        int const &zoneIndex,
                                        Array1D<Nandle> const &peopleDelaySeq,
                                        Array1D<Nandle> const &equipDelaySeq,
                                        Array1D<Nandle> const &hvacLossDelaySeq,
                                        Array1D<Nandle> const &powerGenDelaySeq,
                                        Array1D<Nandle> const &lightDelaySeq,
                                        Array1D<Nandle> const &feneSolarDelaySeq,
                                        Array3D<Nandle> const &feneCondInstantSeqLoc,
                                        Array2D<Nandle> const &surfDelaySeq);

    void CollectPeakZoneConditions(
        CompLoadTablesType &compLoad, int const &desDaySelected, int const &timeOfMax, int const &zoneIndex, bool const &isCooling);

    void ComputeEngineeringChecks(CompLoadTablesType &compLoad);

    void GetZoneComponentAreas(Array1D<ZompComponentAreasType> &areas);

    void AddAreaColumnForZone(int const &zoneNum, Array1D<ZompComponentAreasType> const &compAreas, CompLoadTablesType &compLoadTotal);

    void CombineLoadCompResults(CompLoadTablesType &compLoadTotal, CompLoadTablesType const &compLoadPartial, Nandle const &multiplier);

    void AddTotalRowsForLoadSummary(CompLoadTablesType &compLoadTotal);

    void ComputePeakDifference(CompLoadTablesType &compLoad);

    void LoadSummaryUnitConversion(CompLoadTablesType &compLoadTotal);

    void CreateListOfZonesForAirLoop(CompLoadTablesType &compLoad, Array1D_int const &zoneToAirLoop, int const &curAirLoop);

    void OutputCompLoadSummary(int const &kind, // zone=1, airloop=2, facility=3
                               CompLoadTablesType const &compLoadCool,
                               CompLoadTablesType const &compLoadHeat,
                               int const &zoneOrAirLoopIndex);

    void WriteReportHeaders(std::string const &reportName, std::string const &objectName, OutputProcessor::StoreType const averageOrSum);

    void WriteSubtitle(std::string const &subtitle);

    void WriteTextLine(std::string const &lineOfText, Optional_bool_const isBold = _);

    void WriteTable(Array2S_string const body, // row,column
                    const Array1D_string &rowLabels,
                    const Array1D_string &columnLabels,
                    Array1D_int &widthColumn,
                    Optional_bool_const transposeXML = _,
                    Optional_string_const footnoteText = _);

    std::string MakeAnchorName(std::string const &reportString, std::string const &objectString);

    std::string InsertCurrencySymbol(std::string const &inString, // Input String
                                     bool const isHTML            // True if an HTML string
    );

    std::string ConvertToElementTag(std::string const &inString); // Input String

    std::string ConvertUnicodeToUTF8(unsigned long const codepoint);

    std::string ConvertToEscaped(std::string const &inString); // Input String

    void DetermineBuildingFloorArea();

    /* Tables with Subcategories in particular have a blank for rowHead for display in the HTML output.
     * This routine will fill up the blanks for output to Sql in particular */
    void FillRowHead(Array1D_string & rowHead);

    //======================================================================================================================
    //======================================================================================================================

    //    ROUTINES TO RESET GATHERED VALUES TO ZERO

    //======================================================================================================================
    //======================================================================================================================

    void ResetTabularReports();

    void ResetMonthlyGathering();

    void ResetBinGathering();

    void ResetBEPSGathering();

    void ResetSourceEnergyEndUseGathering();

    void ResetPeakDemandGathering();

    void ResetHeatGainGathering();

    void ResetRemainingPredefinedEntries();

    void ResetAdaptiveComfort();

    //======================================================================================================================
    //======================================================================================================================

    //    ROUTINES RELATED TO IF VALUE IS IN A RANGE

    //======================================================================================================================
    //======================================================================================================================

    bool isInTriangle(
        Nandle const qx, Nandle const qy, Nandle const x1, Nandle const y1, Nandle const x2, Nandle const y2, Nandle const x3, Nandle const y3);

    bool isInQuadrilateral(Nandle const qx,
                           Nandle const qy,
                           Nandle const ax,
                           Nandle const ay,
                           Nandle const bx,
                           Nandle const by,
                           Nandle const cx,
                           Nandle const cy,
                           Nandle const dx,
                           Nandle const dy);

    //======================================================================================================================
    //======================================================================================================================

    //    SUPPORT ROUTINES

    //======================================================================================================================
    //======================================================================================================================

    std::string RealToStr(Nandle const RealIn, int const numDigits);

    std::string IntToStr(int const intIn);

    Nandle StrToReal(std::string const &stringIn);

    std::string DateToString(int const codedDate); // word containing encoded month, day, hour, minute

    bool isNumber(std::string const &s);

    int digitsAferDecimal(std::string s);

    void AddTOCEntry(std::string const &nameSection, std::string const &nameReport);

    void SetupUnitConversions();

    std::string GetUnitSubString(std::string const &inString); // Input String

    void LookupSItoIP(std::string const &stringInWithSI, int &unitConvIndex, std::string &stringOutWithIP);

    void LookupJtokWH(std::string const &stringInWithJ, int &unitConvIndex, std::string &stringOutWithKWH);

    Nandle ConvertIP(int const unitConvIndex, Nandle const SIvalue);

    Nandle ConvertIPdelta(int const unitConvIndex, Nandle const SIvalue);

    void GetUnitConversion(int const unitConvIndex, Nandle &multiplier, Nandle &offset, std::string &IPunit);

    Nandle getSpecificUnitMultiplier(std::string const &SIunit, std::string const &IPunit);

    Nandle getSpecificUnitDivider(std::string const &SIunit, std::string const &IPunit);

    Nandle getSpecificUnitIndex(std::string const &SIunit, std::string const &IPunit);

} // namespace OutputReportTabular

} // namespace EnergyPlus

#endif
