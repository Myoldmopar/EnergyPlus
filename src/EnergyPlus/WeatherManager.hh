// EnergyPlus, Copyright (c) 1996-2021, The Board of Trustees of the University of Illinois,
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

#ifndef WeatherManager_hh_INCLUDED
#define WeatherManager_hh_INCLUDED

// C++ Headers
#include <unordered_map>
#include <vector>

// ObjexxFCL Headers
#include <ObjexxFCL/Array2D.hh>
#include <ObjexxFCL/Array3D.hh>
#include <ObjexxFCL/Optional.hh>

// EnergyPlus Headers
#include <EnergyPlus/Data/BaseData.hh>
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EPVector.hh>
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

// Forward declarations
class BaseGroundTempsModel;
struct EnergyPlusData;

namespace Weather {

    // Following are Date Types read in from EPW file or IDF
    enum class DateType
    {
        InvalidDate = -1,
        MonthDay = 1,
        NthDayInMonth = 2,
        LastDayInMonth = 3
    };

    // Water mains temperatures calculation methods
    enum class WaterMainsTempCalcMethod
    {
        Schedule = 1,
        Correlation = 2,
        CorrelationFromWeatherFile = 3
    };

    enum class DesignDaySolarModel
    {
        ASHRAE_ClearSky = 0,     // Design Day solar model ASHRAE ClearSky (default)
        Zhang_Huang = 1,         // Design Day solar model Zhang Huang
        SolarModel_Schedule = 2, // Design Day solar model (beam and diffuse) from user entered schedule
        ASHRAE_Tau = 3,          // Design Day solar model ASHRAE tau (per 2009 HOF)
        ASHRAE_Tau2017 = 4       // Design Day solar model ASHRAE tau (per 2013 and 2017 HOF)
    };

    // Design Day Humidity Indicating Type
    enum class DDHumIndType
    {
        WetBulb = 0,   // Wetbulb (default)
        DewPoint = 1,  // Dewpoint
        Enthalpy = 2,  // Enthalpy
        HumRatio = 3,  // Humidity Ratio
        RelHumSch = 4, // relhum schedule
        WBProfDef = 5, // Wetbulb default profile
        WBProfDif = 6, // Wetbulb difference profile
        WBProfMul = 7  // Wetbulb multiplier profile
    };

    // Design Day DryBulb Range Type
    enum class DDDBRangeType
    {
        Default = 0,    // Default Multipliers
        Multiplier = 1, // Multiplier Schedule
        Difference = 2, // Difference Schedule
        Profile = 3,    // Temperature Profile
    };

    enum class EmissivityCalcType
    {
        ClarkAllenModel = 0,    // Use Clark & Allen model for sky emissivity calculation
        ScheduleValue = 1,      // User entered Schedule value for Weather Property
        DryBulbDelta = 2,       // User entered DryBulb difference Schedule value for Weather Property
        DewPointDelta = 3,      // User entered Dewpoint difference Schedule value for Weather Property
        BruntModel = 4,         // Use Brunt model for sky emissivity calculation
        IdsoModel = 5,          // Use Isdo model for sky emissivity calculation
        BerdahlMartinModel = 6, // Use Martin & Berdahl model for sky emissivity calculation
        SkyTAlgorithmA = 7,     // place holder
    };

    enum class WeekDay
    {
        Sunday = 1,
        Monday,
        Tuesday,
        Wednesday,
        Thursday,
        Friday,
        Saturday
    };

    struct EnvironmentData
    {
        // Members
        std::string Title;                          // Environment name
        std::string cKindOfEnvrn;                   // kind of environment
        DataGlobalConstants::KindOfSim KindOfEnvrn = DataGlobalConstants::KindOfSim::Unassigned; // Type of environment (see Parameters for KindOfSim in DataGlobals)
        int DesignDayNum = 0;                           // index in DesignDay structure and DesignDayInput
        int RunPeriodDesignNum = 0;                     // for WeatherFileDays, index in  RunPeriodDesign and RunPeriodDesignInput
        int SeedEnvrnNum = 0;           // for HVAC sizing sim, new environments are copies of original environments, this is the index for original
        int HVACSizingIterationNum = 0; // environments for HVAC sizing simulations are associated with iteration
        int TotalDays = 0;              // Number of days in environment
        int StartJDay = 0;              // Day of year of first day of environment
        int StartMonth = 0;
        int StartDay = 0;
        int StartYear = 0;
        int StartDate = 0;
        int EndMonth = 0;
        int EndDay = 0;
        int EndJDay = 0;
        int EndYear = 0;
        int EndDate = 0;
        int DayOfWeek = 0;         // Starting Day of Week for the (Weather) RunPeriod (User Input)
        bool UseDST = false;           // True if DaylightSavingTime is used for this RunPeriod
        bool UseHolidays = false;      // True if Holidays are used for this RunPeriod (from WeatherFile)
        bool ApplyWeekendRule = false; // True if "Weekend Rule" is to be applied to RunPeriod
        bool UseRain = true;          // True if Rain from weather file should be used (set rain to true)
        bool UseSnow = true;          // True if Snow from weather file should be used (set Snow to true)
        Array1D_int MonWeekDay = Array1D_int(12, 0);
        bool SetWeekDays = false;                // true when weekdays will be reset (after first year or on repeat)
        int NumSimYears = 1;                 // Total Number of times this period to be performed
        int CurrentCycle = 0;                // Current cycle through weather file in NumSimYears repeats
        int WP_Type1 = 0;                    // WeatherProperties SkyTemperature Pointer
        EmissivityCalcType SkyTempModel = EmissivityCalcType::ClarkAllenModel; // WeatherProperties SkyTemperature CalculationType
        bool UseWeatherFileHorizontalIR = true; // If false, horizontal IR and sky temperature are calculated with WP models
        int CurrentYear = 0;                 // Current year
        bool IsLeapYear = false;                 // True if current year is leap year.
        bool RollDayTypeOnRepeat = true;        // If repeating run period, increment day type on repeat.
        bool TreatYearsAsConsecutive = true;    // When year rolls over, increment year and recalculate Leap Year
        bool MatchYear = false;                  // for actual weather will be true
        bool ActualWeather = false;              // true when using actual weather data
        int RawSimDays = 0;                  // number of basic sim days.
    };

    struct DesignDayData
    {
        // Members
        std::string Title;              // Environment name
        Real64 MaxDryBulb = 0.0;              // Maximum Dry-Bulb Temperature (C)
        Real64 DailyDBRange = 0.0;            // Daily Temperature Range (deltaC)
        Real64 HumIndValue = 0.0;             // Humidity Indicating Value at Max Dry-bulb Temperature
        DDHumIndType HumIndType = DDHumIndType::WetBulb;        // Humidity Indicating type  (see Parameters)
        Real64 PressBarom = 0.0;              // Atmospheric/Barometric Pressure (Pascals)
        Real64 WindSpeed = 0.0;               // Wind Speed (m/s)
        Real64 WindDir = 0.0;                 // Wind Direction (degrees clockwise from North, N=0, E=90, S=180, W=270)
        Real64 SkyClear = 0.0;                // Sky Clearness (0 to 1)
        int RainInd = 0;                    // Rain Indicator (1 = raining and surfaces are wet, else 0)
        int SnowInd = 0;                    // Snow Indicator (1 = snow on ground, else  0)
        int DayOfMonth = 0;                 // Day of Month ( 1 - 31 )
        int Month = 0;                      // Month of Year ( 1 - 12 )
        int DayType = 0;                    // Day Type Sunday = 1 - Saturday = 7
        int DSTIndicator = 0;               // Daylight Saving Time Period Indicator (1=yes, 0=no) for this DesignDay
        DesignDaySolarModel SolarModel = DesignDaySolarModel::ASHRAE_ClearSky; // Solar Model for creating solar values for design day.
        DDDBRangeType DBTempRangeType = DDDBRangeType::Default;  // Drybulb Range Type (see Parameters)
        int TempRangeSchPtr = 0;            // Schedule pointer to a day schedule for dry-bulb temperature range multipliers
        int HumIndSchPtr = 0;               // Schedule pointer to a day schedule that specifies
        //    relative humidity (%) or wet-bulb range multipliers per HumIndType
        int BeamSolarSchPtr = 0;      // Schedule pointer to a day schedule for beam solar
        int DiffuseSolarSchPtr = 0;   // Schedule pointer to a day schedule for diffuse solar
        Real64 TauB = 0.0;              // beam pseudo optical depth for ASHRAE tau model
        Real64 TauD = 0.0;              // diffuse pseudo optical depth for ASHRAE tau model
        Real64 DailyWBRange = 0.0;      // daily range of wetbulb (deltaC)
        bool PressureEntered = false;     // true if a pressure was entered in design day data
        bool DewPointNeedsSet = false;    // true if the Dewpoint humidicating value needs to be set (after location determined)
        int maxWarmupDays = -1;        // Maximum warmup days between sizing periods
        bool suppressBegEnvReset = false; // true if this design day should be run without thermal history being reset at begin environment
    };

    struct RunPeriodData
    {
        // Members
        std::string title;
        std::string periodType;
        int totalDays = 365; // total number of days in requested period
        int startMonth = 1;
        int startDay = 1;
        int startJulianDate = 2457755; // Calculated start date (Julian or ordinal) for a weather file run period
        int startYear = 2017;       // entered in "consecutive"/real runperiod object
        int endMonth = 12;
        int endDay = 31;
        int endJulianDate = 2458119;     // Calculated end date (Julian or ordinal) for a weather file run period
        int endYear = 2017;           // entered in "consecutive"/real runperiod object
        int dayOfWeek = 1;         // Day of Week that the RunPeriod will start on (User Input)
        WeekDay startWeekDay = WeekDay::Sunday;  // Day of the week that the RunPeriod will start on (User Input)
        bool useDST = false;           // True if DaylightSavingTime is used for this RunPeriod
        bool useHolidays = false;      // True if Holidays are used for this RunPeriod (from WeatherFile)
        bool applyWeekendRule = false; // True if "Weekend Rule" is to be applied to RunPeriod
        bool useRain = true;          // True if Rain from weather file should be used (set rain to true)
        bool useSnow = true;          // True if Snow from weather file should be used (set Snow to true)
        Array1D_int monWeekDay = Array1D_int({1, 4, 4, 7, 2, 5, 7, 3, 6, 1, 4, 6});
        int numSimYears = 1;              // Total Number of years of simulation to be performed
        bool isLeapYear = false;              // True if Begin Year is leap year.
        bool RollDayTypeOnRepeat = true;     // If repeating run period, increment day type on repeat.
        bool TreatYearsAsConsecutive = true; // When year rolls over, increment year and recalculate Leap Year
        bool actualWeather = false;           // true when using actual weather data
    };

    // Derived Type for Storing Weather "Header" Data
    struct DayWeatherVariables {
        // Members
        int DayOfYear = 0;              // Day of year for weather data
        int DayOfYear_Schedule = 0;     // Day of year in schedule
        int Year = 0;                   // = 0 Year of weather data
        int Month = 0;                   // Month of weather data
        int DayOfMonth = 0;             // Day of month for weather data
        int DayOfWeek = 0;              // Day of week for weather data
        int DaylightSavingIndex = 0;    // Daylight Saving Time Period indicator (0=no,1=yes)
        int HolidayIndex = 0;           // Holiday indicator (0=no holiday, non-zero=holiday type)
        Real64 SinSolarDeclinAngle = 0.0; // Sine of the solar declination angle
        Real64 CosSolarDeclinAngle = 0.0; // Cosine of the solar declination angle
        Real64 EquationOfTime = 0.0;      // Value of the equation of time formula
    };

    struct SpecialDayData
    {
        // Members
        std::string Name;                  // Name
        DateType dateType = DateType::InvalidDate; // Date type as read in from IDF
        int Month = 0;                         // Start Month
        int Day = 0;                           // Start Day of month or Count for DateTypes=NthDayOfMonth
        int WeekDay = 0;                       // For Date types=NthDayOfMonth and LastDayOfMonth
        int CompDate = 0;                      // Start Date in "compressed date" format, only if Month/Day
        bool WthrFile = false;                     // True if this Special Day came from weather file (EPW)
        int Duration = 0;                      // Number of days this special Day is used for
        int DayType = 0;                       // Day Type desigation for this Special Day period
        int ActStMon = 0;
        int ActStDay = 0;
        bool Used = false; // Set to true in a run period after use (NthDayOfMonth and LastDayOfMonth only)
    };

    struct DataPeriodData
    {
        // Members
        std::string Name;      // DataPeriod Title
        std::string DayOfWeek; // Start Day of Week for DataPeriod
        int NumYearsData = 0;      // Number of years for which data is present in EPW.
        int WeekDay = 0;
        int StMon = 0;
        int StDay = 0;
        int StYear = 0;
        int EnMon = 0;
        int EnDay = 0;
        int EnYear = 0;
        int NumDays = 0;
        Array1D_int MonWeekDay = Array1D_int(12, 0);
        int DataStJDay = 0;
        int DataEnJDay = 0;
        bool HasYearData = false;
    };

    struct DaylightSavingPeriodData
    {
        // Members
        DateType StDateType; // Start Date type as from EPW or IDF
        int StWeekDay;       // For DateTypes=NthDayOfMonth or LastDayOfMonth
        int StMon;           // DaylightSavingTime (DST) Start Month
        int StDay;           // DaylightSavingTime (DST) Start Day
        DateType EnDateType; // End Date type as from EPW or IDF
        int EnMon;           // DaylightSavingTime (DST) End Month
        int EnDay;           // DaylightSavingTime (DST) End Day
        int EnWeekDay;       // For DateTypes=NthDayOfMonth or LastDayOfMonth

        // Default Constructor
        DaylightSavingPeriodData()
            : StDateType(DateType::InvalidDate), StWeekDay(0), StMon(0), StDay(0), EnDateType(DateType::InvalidDate), EnMon(0), EnDay(0), EnWeekDay(0)
        {
        }
    };

    // This struct carries the default missing data for those data elements that would be best replaced with the previous hour's data for missing
    // data.
    struct MissingData
    {
        // Members
        Real64 DryBulb = 0.0;      // Dry Bulb Temperature (C)
        Real64 DewPoint = 0.0;     // Dew Point Temperature (C)
        Real64 RelHumid = 0.0;        // Relative Humidity (%)
        Real64 StnPres = 0.0;      // Atmospheric Pressure (Pa)
        Real64 WindDir = 0.0;         // Wind Direction (deg)
        Real64 WindSpd = 0.0;      // Wind Speed/Velocity (m/s)
        Real64 TotSkyCvr = 0.0;       // Total Sky Cover (tenths)
        Real64 OpaqSkyCvr = 0.0;      // Opaque Sky Cover (tenths)
        Real64 Visibility = 0.0;   // Visibility (km)
        Real64 Ceiling = 0.0;         // Ceiling Height (m)
        Real64 PrecipWater = 0.0;     // Precipitable Water (mm)
        Real64 AerOptDepth = 0.0;  // Aerosol Optical Depth
        Real64 SnowDepth = 0.0;       // Snow Depth (cm)
        Real64 DaysLastSnow = 0.0;    // Number of Days since last snow
        Real64 Albedo = 0.0;       // Albedo
        Real64 LiquidPrecip = 0.0; // Rain/Liquid Precipitation (mm)
    };

    // This Derived type carries the counts of missing data items in the weather reading process. It will count only items that are on the source file
    // -- not those that are derived from data on the source file.
    struct MissingDataCounts
    {
        // Comments below illustrate the data that is being counted:
        int DryBulb = 0;      // Dry Bulb Temperature (C)
        int DewPoint = 0;     // Dew Point Temperature (C)
        int RelHumid = 0;     // Relative Humidity (%)
        int StnPres = 0;      // Atmospheric Pressure (Pa)
        int WindDir = 0;      // Wind Direction (deg)
        int WindSpd = 0;      // Wind Speed/Velocity (m/s)
        int DirectRad = 0;    // Direct Radiation (wh/m2)
        int DiffuseRad = 0;   // Diffuse Radiation (wh/m2)
        int TotSkyCvr = 0;    // Total Sky Cover (tenths)
        int OpaqSkyCvr = 0;   // Opaque Sky Cover (tenths)
        int Visibility = 0;   // Visibility (km)
        int Ceiling = 0;      // Ceiling Height (m)
        int PrecipWater = 0;  // Precipitable Water (mm)
        int AerOptDepth = 0;  // Aerosol Optical Depth
        int SnowDepth = 0;    // Snow Depth (cm)
        int DaysLastSnow = 0; // Number of Days since last snow
        int WeathCodes = 0;   // Weather codes invalid
        int Albedo = 0;       // Albedo
        int LiquidPrecip = 0; // Liquid Precip Depth
    };

    // This Derived type carries the counts of out of range items in the weather reading process. It will count only items that are on the source file
    // -- not those that are derived from data on the source file.
    struct RangeDataCounts
    {
        // Comments below illustrate the data that is being counted:
        int DryBulb = 0;    // Dry Bulb Temperature (C)
        int DewPoint = 0;   // Dew Point Temperature (C)
        int RelHumid = 0;   // Relative Humidity (%)
        int StnPres = 0;    // Atmospheric Pressure (Pa)
        int WindDir = 0;    // Wind Direction (deg)
        int WindSpd = 0;    // Wind Speed/Velocity (m/s)
        int DirectRad = 0;  // Direct Radiation (wh/m2)
        int DiffuseRad = 0; // Diffuse Radiation (wh/m2)
    };

    struct TypicalExtremeData
    {
        // Members
        std::string Title;       // Environment name
        std::string ShortTitle;  // Environment name
        std::string MatchValue;  // String to be matched for input/running these periods for design.
        std::string MatchValue1; // String to be also matched (synonym)
        std::string MatchValue2; // String to be also matched (synonym)
        std::string TEType;      // Typical or Extreme
        int TotalDays = 0;           // Number of days in environment
        int StartJDay = 0;           // Day of year of first day of environment
        int StartMonth = 0;
        int StartDay = 0;
        int EndMonth = 0;
        int EndDay = 0;
        int EndJDay = 0;
    };

    struct WeatherProperties
    {
        // Members
        std::string Name;         // Reference Name
        std::string ScheduleName; // Schedule Name or Algorithm Name
        bool IsSchedule = true;          // Default is using Schedule
        EmissivityCalcType CalculationType = EmissivityCalcType::ClarkAllenModel;
        int SchedulePtr = 0; // pointer to schedule when used
        bool UsedForEnvrn = false;
        bool UseWeatherFileHorizontalIR = true; // If false, horizontal IR and sky temperature are calculated with WP models
    };

    struct UnderwaterBoundary
    {
        std::string Name;
        Real64 distanceFromLeadingEdge = 0.0;
        int OSCMIndex = 0;
        int WaterTempScheduleIndex = 0;
        int VelocityScheduleIndex = 0;
    };

    // Functions
    void ManageWeather(EnergyPlusData &state);

    void ResetEnvironmentCounter(EnergyPlusData &state);

    bool GetNextEnvironment(EnergyPlusData &state, bool &Available, bool &ErrorsFound);

    void AddDesignSetToEnvironmentStruct(
        EnergyPlusData &state, int HVACSizingIterCount // Counter for number of times HVAC Sizing Simulation of Design Period set is being rerun
    );

    bool CheckIfAnyUnderwaterBoundaries(EnergyPlusData &state);

    Real64 calculateWaterBoundaryConvectionCoefficient(Real64 curWaterTemp, Real64 curWaterVelocity, Real64 distanceFromLeadingEdge);

    void UpdateUnderwaterBoundaries(EnergyPlusData &state);

    void ReadVariableLocationOrientation(EnergyPlusData &state);

    void UpdateLocationAndOrientation(EnergyPlusData &state);

    void SetupWeekDaysByMonth(EnergyPlusData &state, int StMon, int StDay, int StWeekDay, Array1D_int &WeekDays);

    void ResetWeekDaysByMonth(EnergyPlusData &state,
                              Array1D_int &WeekDays,
                              int AddLeapYear,
                              int StartMonth,
                              int StartMonthDay,
                              int EndMonth,
                              int EndMonthDay,
                              bool Rollover,
                              bool MidSimReset = false);

    void SetDSTDateRanges(EnergyPlusData &state,
                          Array1D_int const &MonWeekDay, // Weekday of each day 1 of month
                          Array1D_int &DSTIdx,           // DST Index for each julian day (1:366)
                          Optional_int DSTActStMon = _,
                          Optional_int DSTActStDay = _,
                          Optional_int DSTActEnMon = _,
                          Optional_int DSTActEnDay = _);

    void SetSpecialDayDates(EnergyPlusData &state, Array1D_int const &MonWeekDay); // Weekday of each day 1 of month

    void InitializeWeather(EnergyPlusData &state, bool &printEnvrnStamp); // Set to true when the environment header should be printed

    void UpdateWeatherData(EnergyPlusData &state);

    void SetCurrentWeather(EnergyPlusData &state);

    void ReadEPlusWeatherForDay(EnergyPlusData &state,
                                int DayToRead,          // =1 when starting out, otherwise signifies next day
                                int Environ,            // Environment being simulated
                                bool BackSpaceAfterRead // True if weather file is to be backspaced after read
    );

    Real64 interpolateWindDirection(Real64 prevHrWindDir, Real64 curHrWindDir, Real64 curHrWeight);

    void SetDayOfWeekInitialValues(int EnvironDayOfWeek, // Starting Day of Week for the (Weather) RunPeriod (User Input)
                                   int &currentDayOfWeek // Current Day of Week
    );

    void ErrorInterpretWeatherDataLine(
        EnergyPlusData &state, int WYear, int WMonth, int WDay, int WHour, int WMinute, std::string &SaveLine, std::string &Line);

    void InterpretWeatherDataLine(EnergyPlusData &state,
                                  std::string &Line,
                                  bool &ErrorFound, // True if an error is found, false otherwise
                                  int &WYear,
                                  int &WMonth,
                                  int &WDay,
                                  int &WHour,
                                  int &WMinute,
                                  Real64 &DryBulb,           // DryBulb
                                  Real64 &DewPoint,          // DewPoint
                                  Real64 &RelHum,            // RelHum
                                  Real64 &AtmPress,          // AtmPress
                                  Real64 &ETHoriz,           // ETHoriz
                                  Real64 &ETDirect,          // ETDirect
                                  Real64 &IRHoriz,           // IRHoriz
                                  Real64 &GLBHoriz,          // GLBHoriz
                                  Real64 &DirectRad,         // DirectRad
                                  Real64 &DiffuseRad,        // DiffuseRad
                                  Real64 &GLBHorizIllum,     // GLBHorizIllum
                                  Real64 &DirectNrmIllum,    // DirectNrmIllum
                                  Real64 &DiffuseHorizIllum, // DiffuseHorizIllum
                                  Real64 &ZenLum,            // ZenLum
                                  Real64 &WindDir,           // WindDir
                                  Real64 &WindSpeed,         // WindSpeed
                                  Real64 &TotalSkyCover,     // TotalSkyCover
                                  Real64 &OpaqueSkyCover,    // OpaqueSkyCover
                                  Real64 &Visibility,        // Visibility
                                  Real64 &CeilHeight,        // CeilHeight
                                  int &WObs,                 // PresWeathObs
                                  Array1D_int &WCodesArr,    // PresWeathConds
                                  Real64 &PrecipWater,       // PrecipWater
                                  Real64 &AerosolOptDepth,   // AerosolOptDepth
                                  Real64 &SnowDepth,         // SnowDepth
                                  Real64 &DaysSinceLastSnow, // DaysSinceLastSnow
                                  Real64 &Albedo,            // Albedo
                                  Real64 &LiquidPrecip       // LiquidPrecip
    );

    void SetUpDesignDay(EnergyPlusData &state, int EnvrnNum); // Environment number passed into the routine

    Real64 AirMass(Real64 CosZen); // COS( solar zenith), 0 - 1

    // Calculate sky temperature from weather data
    Real64 CalcSkyEmissivity(EnergyPlusData &state, EmissivityCalcType ESkyCalcType, Real64 OSky, Real64 DryBulb, Real64 DewPoint, Real64 RelHum);

    void ASHRAETauModel([[maybe_unused]] EnergyPlusData &state,
                        DesignDaySolarModel TauModelType, // ASHRAETau solar model type ASHRAE_Tau or ASHRAE_Tau2017
                        Real64 ETR,                       // extraterrestrial normal irradiance, W/m2
                        Real64 CosZen,                    // COS( solar zenith angle), 0 - 1
                        Real64 TauB,                      // beam tau factor
                        Real64 TauD,                      // dif tau factor
                        Real64 &IDirN,                    // returned: direct (beam) irradiance on normal surface, W/m2
                        Real64 &IDifH,                    // returned: diffuse irradiance on horiz surface, W/m2
                        Real64 &IGlbH                     // returned: global irradiance on horiz surface, W/m2
    );

    void AllocateWeatherData(EnergyPlusData &state);

    void CalculateDailySolarCoeffs(EnergyPlusData &state,
                                   int DayOfYear,                 // Day of year (1 - 366)
                                   Real64 &A,                     // ASHRAE "A" - Apparent solar irradiation at air mass = 0 [W/M**2]
                                   Real64 &B,                     // ASHRAE "B" - Atmospheric extinction coefficient
                                   Real64 &C,                     // ASHRAE "C" - Diffuse radiation factor
                                   Real64 &AnnVarSolConstant,     // Annual variation in the solar constant
                                   Real64 &EquationOfTime,        // Equation of Time
                                   Real64 &SineSolarDeclination,  // Sine of Solar Declination
                                   Real64 &CosineSolarDeclination // Cosine of Solar Declination
    );

    void CalculateSunDirectionCosines(EnergyPlusData &state,
                                      Real64 TimeValue,    // Current Time of Day
                                      Real64 EqOfTime,     // Equation of Time
                                      Real64 SinSolDeclin, // Sine of Solar Declination
                                      Real64 CosSolDeclin, // Cosine of Solar Declination
                                      Array1D<Real64> &SUNCOS);

    void DetermineSunUpDown(EnergyPlusData &state, Array1D<Real64> &SunDirectionCosines);

    void OpenWeatherFile(EnergyPlusData &state, bool &ErrorsFound);

    void OpenEPlusWeatherFile(EnergyPlusData &state,
                              bool &ErrorsFound, // Will be set to true if errors found
                              bool ProcessHeader // Set to true when headers should be processed (rather than just read)
    );

    void CloseWeatherFile(EnergyPlusData &state);

    void ResolveLocationInformation(EnergyPlusData &state, bool &ErrorsFound); // Set to true if no location evident

    void CheckLocationValidity(EnergyPlusData &state);

    void CheckWeatherFileValidity(EnergyPlusData &state);

    void ReportOutputFileHeaders(EnergyPlusData &state);

    void ReportWeatherAndTimeInformation(EnergyPlusData &state,
                                         bool &printEnvrnStamp); // Set to true when the environment header should be printed

    void ReadUserWeatherInput(EnergyPlusData &state);

    void GetRunPeriodData(EnergyPlusData &state,
                          int &nRunPeriods, // Total number of Run Periods requested
                          bool &ErrorsFound);

    void GetRunPeriodDesignData(EnergyPlusData &state, bool &ErrorsFound);

    void GetSpecialDayPeriodData(EnergyPlusData &state, bool &ErrorsFound); // will be set to true if severe errors are found in inputs

    void CalcSpecialDayTypes(EnergyPlusData &state);

    void GetDSTData(EnergyPlusData &state, bool &ErrorsFound); // will be set to true if severe errors are found in inputs

    void GetDesignDayData(EnergyPlusData &state,
                          int &TotDesDays, // Total number of Design days to Setup
                          bool &ErrorsFound);

    void GetLocationInfo(EnergyPlusData &state, bool &ErrorsFound);

    void GetWeatherProperties(EnergyPlusData &state, bool &ErrorsFound);

    void GetGroundTemps(EnergyPlusData &state, bool &ErrorsFound);

    void GetGroundReflectances(EnergyPlusData &state, bool &ErrorsFound);

    void GetSnowGroundRefModifiers(EnergyPlusData &state, bool &ErrorsFound);

    void GetWaterMainsTemperatures(EnergyPlusData &state, bool &ErrorsFound);

    void CalcWaterMainsTemp(EnergyPlusData &state);

    Real64 WaterMainsTempFromCorrelation(EnergyPlusData &state,
                                         Real64 AnnualOAAvgDryBulbTemp,        // annual average OA drybulb temperature
                                         Real64 MonthlyOAAvgDryBulbTempMaxDiff // monthly daily average OA drybulb temperature maximum difference
    );

    void GetWeatherStation(EnergyPlusData &state, bool &ErrorsFound);

    void DayltgCurrentExtHorizIllum(EnergyPlusData &state);

    void DayltgLuminousEfficacy(EnergyPlusData &state,
                                Real64 &DiffLumEff, // Luminous efficacy of sky diffuse solar radiation (lum/W)
                                Real64 &DirLumEff   // Luminous efficacy of beam solar radiation (lum/W)
    );

    Real64 GetSTM(Real64 Longitude); // Longitude from user input

    void ProcessEPWHeader(EnergyPlusData &state, std::string const &HeaderString, std::string &Line, bool &ErrorsFound);

    void SkipEPlusWFHeader(EnergyPlusData &state);

    void ReportMissing_RangeData(EnergyPlusData &state);

    void SetupInterpolationValues(EnergyPlusData &state);

    void SetupEnvironmentTypes(EnergyPlusData &state);

    bool isLeapYear(int Year);

    struct GregorianDate
    {
        int year;
        int month;
        int day;

        GregorianDate(int year, int month, int day) : year(year), month(month), day(day)
        {
        }
    };

    int computeJulianDate(int gyyyy, int gmm, int gdd);

    int computeJulianDate(GregorianDate gdate);

    GregorianDate computeGregorianDate(int jdate);

    WeekDay calculateDayOfWeek(EnergyPlusData &state, int year, int month, int day);

    int calculateDayOfYear(int Month, int Day, bool leapYear = false);

    bool validMonthDay(int month, int day, int leapYearAdd = 0);

    // derived type for processing and storing Dry-bulb weather or stat file
    struct AnnualMonthlyDryBulbWeatherData
    {
        // Members
        bool OADryBulbWeatherDataProcessed;             // if false stat or weather file OA Dry-bulb temp is not processed yet
        Real64 AnnualAvgOADryBulbTemp;                  // annual average outdoor air temperature (C)
        Real64 MonthlyAvgOADryBulbTempMaxDiff;          // monthly daily average OA drybulb temperature maximum difference (deltaC)
        Array1D<Real64> MonthlyDailyAverageDryBulbTemp; // monthly-daily average outdoor air temperatures (C)

        // Default Constructor
        AnnualMonthlyDryBulbWeatherData()
            : OADryBulbWeatherDataProcessed(false), AnnualAvgOADryBulbTemp(0.0), MonthlyAvgOADryBulbTempMaxDiff(0.0),
              MonthlyDailyAverageDryBulbTemp(12, 0.0)
        {
        }
        void CalcAnnualAndMonthlyDryBulbTemp(EnergyPlusData &state); // true if this is CorrelationFromWeatherFile
    };

    void ReportWaterMainsTempParameters(EnergyPlusData &state);

    void calcSky(EnergyPlusData &state,
                 Real64 &TmrHorizIRSky,
                 Real64 &TmrSkyTemp,
                 Real64 OpaqueSkyCover,
                 Real64 DryBulb,
                 Real64 DewPoint,
                 Real64 RelHum,
                 Real64 IRHoriz);

    struct WeatherManagerItself {
        std::vector<std::string> rawEPWLines;

        void readEPWLines();
    };

} // namespace WeatherManager

struct WeatherManagerData : BaseGlobalStruct
{

    // this will ultimately, hopefully, become the only variable in this module, with everything adapted over to WeatherManager members and methods
    Weather::WeatherManagerItself weather;

    // These were static variables within different functions. They were pulled out into the namespace
    // to facilitate easier unit testing of those functions.
    // These are purposefully not in the header file as an extern variable. No one outside of this should
    // use these. They are cleared by clear_state() for use by unit tests, but normal simulations should be unaffected.
    // This is purposefully in an anonymous namespace so nothing outside this implementation file can use it.
    bool GetBranchInputOneTimeFlag;
    bool GetEnvironmentFirstCall;
    bool PrntEnvHeaders;
    bool FirstCall;                 // Some things should only be done once
    bool WaterMainsParameterReport; // should only be done once
    bool PrintEnvrnStamp;           // Set to true when the environment header should be printed
    bool PrintDDHeader;

    Real64 const Sigma; // Stefan-Boltzmann constant

    int YearOfSim; // The Present year of Simulation.
    int const NumDaysInYear;
    int EnvironmentReportNbr;         // Report number for the environment stamp
    std::string EnvironmentReportChr; // Report number for the environment stamp (character -- for printing)
    bool WeatherFileExists;           // Set to true if a weather file exists
    std::string LocationTitle;        // Location Title from input File
    bool LocationGathered;            // flag to show if Location exists on Input File (we assume one is there and correct on weather file)

    Real64 WeatherFileLatitude;
    Real64 WeatherFileLongitude;
    Real64 WeatherFileTimeZone;
    Real64 WeatherFileElevation;
    Array1D<Real64> GroundTempsFCFromEPWHeader; // F or C factor method NOLINT(cert-err58-cpp)
    Array1D<Real64> GroundReflectances;         // User Specified Ground Reflectances !EPTeam: Using DP causes big diffs NOLINT(cert-err58-cpp)
    Real64 SnowGndRefModifier;                  // Modifier to ground reflectance during snow
    Real64 SnowGndRefModifierForDayltg;         // Modifier to ground reflectance during snow for daylighting
    Weather::WaterMainsTempCalcMethod WaterMainsTempsMethod; // Water mains temperature calculation method
    int WaterMainsTempsSchedule;                                    // Water mains temperature schedule
    Real64 WaterMainsTempsAnnualAvgAirTemp;                         // Annual average outdoor air temperature (C)
    Real64 WaterMainsTempsMaxDiffAirTemp;                           // Maximum difference in monthly average outdoor air temperatures (deltaC)
    std::string WaterMainsTempsScheduleName;                        // water mains tempeature schedule name
    bool wthFCGroundTemps;

    int TotRunPers;    // Total number of Run Periods (Weather data) to Setup
    int TotRunDesPers; // Total number of Run Design Periods (Weather data) to Setup

    int NumSpecialDays;
    Array1D_int SpecialDayTypes; // To hold holiday types given in input file NOLINT(cert-err58-cpp)
    Array1D_int WeekDayTypes;    // To hold Week day types using specified first day NOLINT(cert-err58-cpp)
    Array1D_int DSTIndex;        // To hold DST Index based on weather file or input NOLINT(cert-err58-cpp)

    int NumDataPeriods;

    int NumIntervalsPerHour;

    bool UseDaylightSaving;         // True if user says to use Weather File specified DaylightSaving Period
    bool UseSpecialDays;            // True if user says to use Weather File specified Special Days for current RunPeriod
    bool UseRainValues;             // True if rain values from weather file are to be used
    bool UseSnowValues;             // True if snow values from weather file are to be used
    bool EPWDaylightSaving;         // True if a DaylightSaving Time Period is input (EPW files)
    bool IDFDaylightSaving;         // True if a DaylightSaving Time Period is input (IDF files)
    bool DaylightSavingIsActive;    // True if a DaylightSavingPeriod should be used for Environment
    bool WFAllowsLeapYears;         // True if the Weather File (WF) header has "Yes" for Leap Years
    int curSimDayForEndOfRunPeriod; // normal=number days in sim, but different when repeating runperiods or multi-year files
    int Envrn;                      // Counter for environments
    int NumOfEnvrn;                 // Number of environments to be simulated
    int NumEPWTypExtSets;           // Number of Typical/Extreme on weather file.
    int NumWPSkyTemperatures;       // Number of WeatherProperty:SkyTemperature items in input file

    Array2D_bool TodayIsRain;             // Rain indicator, true=rain NOLINT(cert-err58-cpp)
    Array2D_bool TodayIsSnow;             // Snow indicator, true=snow NOLINT(cert-err58-cpp)
    Array2D<Real64> TodayOutDryBulbTemp;  // Dry bulb temperature of outside air NOLINT(cert-err58-cpp)
    Array2D<Real64> TodayOutDewPointTemp; // Dew Point Temperature of outside air NOLINT(cert-err58-cpp)
    Array2D<Real64> TodayOutBaroPress;    // Barometric pressure of outside air NOLINT(cert-err58-cpp)
    Array2D<Real64> TodayOutRelHum;       // Relative Humidity of outside air NOLINT(cert-err58-cpp)
    Array2D<Real64> TodayWindSpeed;       // Wind speed of outside air NOLINT(cert-err58-cpp)
    Array2D<Real64> TodayWindDir;         // Wind direction of outside air NOLINT(cert-err58-cpp)
    Array2D<Real64> TodaySkyTemp;         // Sky temperature NOLINT(cert-err58-cpp)
    Array2D<Real64> TodayHorizIRSky;      // Horizontal IR from Sky NOLINT(cert-err58-cpp)
    Array2D<Real64> TodayBeamSolarRad;    // Direct normal solar irradiance NOLINT(cert-err58-cpp)
    Array2D<Real64> TodayDifSolarRad;     // Sky diffuse horizontal solar irradiance NOLINT(cert-err58-cpp)
    Array2D<Real64> TodayAlbedo;          // Albedo NOLINT(cert-err58-cpp)
    Array2D<Real64> TodayLiquidPrecip;    // Liquid Precipitation Depth (mm) NOLINT(cert-err58-cpp)
    Array2D<Real64> TodayTotalSkyCover;   // Total Sky Cover(cert-err58-cpp)
    Array2D<Real64> TodayOpaqueSkyCover;  // Opaque Sky Cover(cert-err58-cpp)

    Array2D_bool TomorrowIsRain;             // Rain indicator, true=rain NOLINT(cert-err58-cpp)
    Array2D_bool TomorrowIsSnow;             // Snow indicator, true=snow NOLINT(cert-err58-cpp)
    Array2D<Real64> TomorrowOutDryBulbTemp;  // Dry bulb temperature of outside air NOLINT(cert-err58-cpp)
    Array2D<Real64> TomorrowOutDewPointTemp; // Dew Point Temperature of outside air NOLINT(cert-err58-cpp)
    Array2D<Real64> TomorrowOutBaroPress;    // Barometric pressure of outside air NOLINT(cert-err58-cpp)
    Array2D<Real64> TomorrowOutRelHum;       // Relative Humidity of outside air NOLINT(cert-err58-cpp)
    Array2D<Real64> TomorrowWindSpeed;       // Wind speed of outside air NOLINT(cert-err58-cpp)
    Array2D<Real64> TomorrowWindDir;         // Wind direction of outside air NOLINT(cert-err58-cpp)
    Array2D<Real64> TomorrowSkyTemp;         // Sky temperature NOLINT(cert-err58-cpp)
    Array2D<Real64> TomorrowHorizIRSky;      // Horizontal IR from Sky NOLINT(cert-err58-cpp)
    Array2D<Real64> TomorrowBeamSolarRad;    // Direct normal solar irradiance NOLINT(cert-err58-cpp)
    Array2D<Real64> TomorrowDifSolarRad;     // Sky diffuse horizontal solar irradiance NOLINT(cert-err58-cpp)
    Array2D<Real64> TomorrowAlbedo;          // Albedo NOLINT(cert-err58-cpp)
    Array2D<Real64> TomorrowLiquidPrecip;    // Liquid Precipitation Depth NOLINT(cert-err58-cpp)
    Array2D<Real64> TomorrowTotalSkyCover;   // Total Sky Cover {tenth of sky}(cert-err58-cpp)
    Array2D<Real64> TomorrowOpaqueSkyCover;  // Opaque Sky Cover {tenth of sky}(cert-err58-cpp)

    Array3D<Real64> DDDBRngModifier;      // Design Day Dry-bulb Temperature Range Modifier NOLINT(cert-err58-cpp)
    Array3D<Real64> DDHumIndModifier;     // Design Day relative humidity values or wet-bulb modifiers (per HumIndType) NOLINT(cert-err58-cpp)
    Array3D<Real64> DDBeamSolarValues;    // Design Day Beam Solar Values NOLINT(cert-err58-cpp)
    Array3D<Real64> DDDiffuseSolarValues; // Design Day Relative Humidity Values NOLINT(cert-err58-cpp)

    Array3D<Real64> DDSkyTempScheduleValues; // Sky temperature - DesignDay input NOLINT(cert-err58-cpp)

    int RptIsRain;  // Rain Report Value
    int RptIsSnow;  // Snow Report Value
    int RptDayType; // DayType Report Value

    Real64 HrAngle;                                       // Current Hour Angle
    Real64 SolarAltitudeAngle;                            // Angle of Solar Altitude (degrees)
    Real64 SolarAzimuthAngle;                             // Angle of Solar Azimuth (degrees)
    Real64 HorizIRSky;                                    // Horizontal Infrared Radiation Intensity (W/m2)
    Real64 TimeStepFraction;                              // Fraction of hour each time step represents
    Array1D<Real64> SPSiteDryBulbRangeModScheduleValue;   // reporting Drybulb Temperature Range Modifier Schedule Value NOLINT(cert-err58-cpp)
    Array1D<Real64> SPSiteHumidityConditionScheduleValue; // reporting Humidity Condition Schedule Value NOLINT(cert-err58-cpp)
    Array1D<Real64> SPSiteBeamSolarScheduleValue;         // reporting Beam Solar Schedule Value NOLINT(cert-err58-cpp)
    Array1D<Real64> SPSiteDiffuseSolarScheduleValue;      // reporting Diffuse Solar Schedule Value NOLINT(cert-err58-cpp)
    Array1D<Real64> SPSiteSkyTemperatureScheduleValue;    // reporting SkyTemperature Modifier Schedule Value NOLINT(cert-err58-cpp)
    Array1D_int SPSiteScheduleNamePtr;                    // SP Site Schedule Name Ptrs NOLINT(cert-err58-cpp)
    Array1D_string SPSiteScheduleUnits;                   // SP Site Schedule Units NOLINT(cert-err58-cpp)
    int NumSPSiteScheduleNamePtrs;                        // Number of SP Site Schedules (DesignDay only)
    // Number of hours of missing data
    Array1D<Real64> Interpolation;      // Interpolation values based on Number of Time Steps in Hour NOLINT(cert-err58-cpp)
    Array1D<Real64> SolarInterpolation; // Solar Interpolation values based on Number of Time Steps in Hour NOLINT(cert-err58-cpp)
    Array1D_int EndDayOfMonth;          // NOLINT(cert-err58-cpp)
    int LeapYearAdd;                    // Set during environment if leap year is active (adds 1 to number days in Feb)
    bool DatesShouldBeReset;            // True when weekdays should be reset
    bool StartDatesCycleShouldBeReset;  // True when start dates on repeat should be reset
    bool Jan1DatesShouldBeReset;        // True if Jan 1 should signal reset of dates
    bool RPReadAllWeatherData;          // True if need to read all weather data prior to simulation

    // Object Data
    // NOLINTNEXTLINE(cert-err58-cpp)
    Weather::DayWeatherVariables
        TodayVariables; // Today's daily weather variables | Derived Type for Storing Weather "Header" Data | Day of year for weather
                        // data | Year of weather data | Month of weather data | Day of month for weather data | Day of week for
                        // weather data | Daylight Saving Time Period indicator (0=no,1=yes) | Holiday indicator (0=no holiday,
                        // non-zero=holiday type) | Sine of the solar declination angle | Cosine of the solar declination angle |
                        // Value of the equation of time formula
    // NOLINTNEXTLINE(cert-err58-cpp)
    Weather::DayWeatherVariables
        TomorrowVariables; // Tomorrow's daily weather variables | Derived Type for Storing Weather "Header" Data | Day of year for
                           // weather data | Year of weather data | Month of weather data | Day of month for weather data | Day of
                           // week for weather data | Daylight Saving Time Period indicator (0=no,1=yes) | Holiday indicator (0=no
                           // holiday, non-zero=holiday type) | Sine of the solar declination angle | Cosine of the solar declination
                           // angle | Value of the equation of time formula
    // NOLINTNEXTLINE(cert-err58-cpp)
    EPVector<Weather::DayWeatherVariables> DesignDay; // Design day environments
    // NOLINTNEXTLINE(cert-err58-cpp)
    Weather::MissingData Missing; // Dry Bulb Temperature (C) | Dew Point Temperature (C) | Relative Humidity (%) | Atmospheric Pressure (Pa) |
                                         // Wind Direction (deg) | Wind Speed/Velocity (m/s) | Total Sky Cover (tenths) | Opaque Sky Cover (tenths) |
                                         // Visibility (km) | Ceiling Height (m) | Precipitable Water (mm) | Aerosol Optical Depth | Snow Depth (cm) |
                                         // Number of Days since last snow | Albedo | Rain/Liquid Precipitation (mm)
    Weather::MissingDataCounts Missed;              // NOLINT(cert-err58-cpp)
    Weather::RangeDataCounts OutOfRange;            // NOLINT(cert-err58-cpp)
    EPVector<Weather::DesignDayData> DesDayInput;   // Design day Input Data NOLINT(cert-err58-cpp)
    Array1D<Weather::EnvironmentData> Environment;  // Environment data NOLINT(cert-err58-cpp)
    Array1D<Weather::RunPeriodData> RunPeriodInput; // NOLINT(cert-err58-cpp)
    std::unordered_map<std::string, std::string> RunPeriodInputUniqueNames;
    EPVector<Weather::RunPeriodData> RunPeriodDesignInput; // NOLINT(cert-err58-cpp)
    std::unordered_map<std::string, std::string> RunPeriodDesignInputUniqueNames;
    EPVector<Weather::TypicalExtremeData> TypicalExtremePeriods; // NOLINT(cert-err58-cpp)
    Weather::DaylightSavingPeriodData EPWDST;                    // Daylight Saving Period Data from EPW file NOLINT(cert-err58-cpp)
    Weather::DaylightSavingPeriodData IDFDST;                    // Daylight Saving Period Data from IDF file NOLINT(cert-err58-cpp)
    Weather::DaylightSavingPeriodData DST;                       // Daylight Saving Period Data, if active NOLINT(cert-err58-cpp)
    EPVector<Weather::WeatherProperties> WPSkyTemperature;       // NOLINT(cert-err58-cpp)
    EPVector<Weather::SpecialDayData> SpecialDays;               // NOLINT(cert-err58-cpp)
    EPVector<Weather::DataPeriodData> DataPeriods;               // NOLINT(cert-err58-cpp)

    std::shared_ptr<BaseGroundTempsModel> siteShallowGroundTempsPtr;
    std::shared_ptr<BaseGroundTempsModel> siteBuildingSurfaceGroundTempsPtr;
    std::shared_ptr<BaseGroundTempsModel> siteFCFactorMethodGroundTempsPtr;
    std::shared_ptr<BaseGroundTempsModel> siteDeepGroundTempsPtr;

    std::vector<Weather::UnderwaterBoundary> underwaterBoundaries;
    Weather::AnnualMonthlyDryBulbWeatherData OADryBulbAverage; // processes outside air drybulb temperature

    // SetCurrentWeather static vars
    int NextHour;

    // ReadEPlusWeatherForDay static vars
    int CurDayOfWeek;
    Real64 ReadEPlusWeatherCurTime;
    bool LastHourSet;
    Real64 LastHrOutDryBulbTemp;
    Real64 LastHrOutDewPointTemp;
    Real64 LastHrOutBaroPress;
    Real64 LastHrOutRelHum;
    Real64 LastHrWindSpeed;
    Real64 LastHrWindDir;
    Real64 LastHrSkyTemp;
    Real64 LastHrHorizIRSky;
    Real64 LastHrBeamSolarRad;
    Real64 LastHrDifSolarRad;
    Real64 LastHrAlbedo;
    Real64 LastHrLiquidPrecip;
    Real64 LastHrTotalSkyCover;
    Real64 LastHrOpaqueSkyCover;
    Real64 NextHrBeamSolarRad;
    Real64 NextHrDifSolarRad;
    Real64 NextHrLiquidPrecip;

    // ProcessEPWHeader static vars
    std::string EPWHeaderTitle;

    void clear_state() override
    {
        this->YearOfSim = 1;             // The Present year of Simulation.
        this->EnvironmentReportNbr = 0;  // Report number for the environment stamp
        this->EnvironmentReportChr = ""; // Report number for the environment stamp (character -- for printing)
        this->WeatherFileExists = false; // Set to true if a weather file exists
        this->LocationTitle = "";        // Location Title from input File
        this->LocationGathered = false;  // flag to show if Location exists on Input File (we assume one is

        this->GetBranchInputOneTimeFlag = true;
        this->GetEnvironmentFirstCall = true;
        this->PrntEnvHeaders = true;
        this->WeatherFileLatitude = 0.0;
        this->WeatherFileLongitude = 0.0;
        this->WeatherFileTimeZone = 0.0;
        this->WeatherFileElevation = 0.0;
        this->siteShallowGroundTempsPtr.reset();
        this->siteBuildingSurfaceGroundTempsPtr.reset();
        this->siteFCFactorMethodGroundTempsPtr.reset();
        this->siteDeepGroundTempsPtr.reset();
        this->GroundTempsFCFromEPWHeader = Array1D<Real64>(12, 0.0);
        this->GroundReflectances = Array1D<Real64>(12, 0.2);

        this->SnowGndRefModifier = 1.0;              // Modifier to ground reflectance during snow
        this->SnowGndRefModifierForDayltg = 1.0;     // Modifier to ground reflectance during snow for daylighting
        this->WaterMainsTempsSchedule = 0;           // Water mains temperature schedule
        this->WaterMainsTempsAnnualAvgAirTemp = 0.0; // Annual average outdoor air temperature (C)
        this->WaterMainsTempsMaxDiffAirTemp = 0.0;   // Maximum difference in monthly average outdoor air temperatures (deltaC)
        this->WaterMainsTempsScheduleName = "";      // water mains tempeature schedule name
        this->wthFCGroundTemps = false;
        this->TotRunPers = 0;    // Total number of Run Periods (Weather data) to Setup
        this->TotRunDesPers = 0; // Total number of Run Design Periods (Weather data) to Setup
        this->NumSpecialDays = 0;

        this->SpecialDayTypes = Array1D<int>(366, 0);
        this->WeekDayTypes = Array1D<int>(366, 0);
        this->DSTIndex = Array1D<int>(366, 0);

        this->NumDataPeriods = 0;
        this->NumIntervalsPerHour = 1;
        this->UseDaylightSaving = true;             // True if user says to use Weather File specified DaylightSaving Period
        this->UseSpecialDays = true;                // True if user says to use Weather File specified Special Days for current RunPeriod
        this->UseRainValues = true;                 // True if rain values from weather file are to be used
        this->UseSnowValues = true;                 // True if snow values from weather file are to be used
        this->EPWDaylightSaving = false;            // True if a DaylightSaving Time Period is input (EPW files)
        this->IDFDaylightSaving = false;            // True if a DaylightSaving Time Period is input (IDF files)
        this->DaylightSavingIsActive = false;       // True if a DaylightSavingPeriod should be used for Environment
        this->WFAllowsLeapYears = false;            // True if the Weather File (WF) header has "Yes" for Leap Years
        this->curSimDayForEndOfRunPeriod = 0;       // normal=number days in sim, but different when repeating runperiods or multi-year files
        this->Envrn = 0;                            // Counter for environments
        this->NumOfEnvrn = 0;                       // Number of environments to be simulated
        this->NumEPWTypExtSets = 0;                 // Number of Typical/Extreme on weather file.
        this->NumWPSkyTemperatures = 0;             // Number of WeatherProperty:SkyTemperature items in input file
        this->TodayIsRain.deallocate();             // Rain indicator, true=rain
        this->TodayIsSnow.deallocate();             // Snow indicator, true=snow
        this->TodayOutDryBulbTemp.deallocate();     // Dry bulb temperature of outside air
        this->TodayOutDewPointTemp.deallocate();    // Dew Point Temperature of outside air
        this->TodayOutBaroPress.deallocate();       // Barometric pressure of outside air
        this->TodayOutRelHum.deallocate();          // Relative Humidity of outside air
        this->TodayWindSpeed.deallocate();          // Wind speed of outside air
        this->TodayWindDir.deallocate();            // Wind direction of outside air
        this->TodaySkyTemp.deallocate();            // Sky temperature
        this->TodayHorizIRSky.deallocate();         // Horizontal IR from Sky
        this->TodayBeamSolarRad.deallocate();       // Direct normal solar irradiance
        this->TodayDifSolarRad.deallocate();        // Sky diffuse horizontal solar irradiance
        this->TodayAlbedo.deallocate();             // Albedo
        this->TodayLiquidPrecip.deallocate();       // Liquid Precipitation Depth (mm)
        this->TodayTotalSkyCover.deallocate();      // Total Sky Cover
        this->TomorrowOpaqueSkyCover.deallocate();  // Opaque Sky Cover {tenth of sky}
        this->TomorrowIsRain.deallocate();          // Rain indicator, true=rain
        this->TomorrowIsSnow.deallocate();          // Snow indicator, true=snow
        this->TomorrowOutDryBulbTemp.deallocate();  // Dry bulb temperature of outside air
        this->TomorrowOutDewPointTemp.deallocate(); // Dew Point Temperature of outside air
        this->TomorrowOutBaroPress.deallocate();    // Barometric pressure of outside air
        this->TomorrowOutRelHum.deallocate();       // Relative Humidity of outside air
        this->TomorrowWindSpeed.deallocate();       // Wind speed of outside air
        this->TomorrowWindDir.deallocate();         // Wind direction of outside air
        this->TomorrowSkyTemp.deallocate();         // Sky temperature
        this->TomorrowHorizIRSky.deallocate();      // Horizontal IR from Sky
        this->TomorrowBeamSolarRad.deallocate();    // Direct normal solar irradiance
        this->TomorrowDifSolarRad.deallocate();     // Sky diffuse horizontal solar irradiance
        this->TomorrowAlbedo.deallocate();          // Albedo
        this->TomorrowLiquidPrecip.deallocate();    // Liquid Precipitation Depth
        this->TomorrowTotalSkyCover.deallocate();   // Total Sky Cover {tenth of sky}
        this->TomorrowOpaqueSkyCover.deallocate();  // Opaque Sky Cover {tenth of sky}
        this->DDDBRngModifier.deallocate();         // Design Day Dry-bulb Temperature Range Modifier
        this->DDHumIndModifier.deallocate();        // Design Day relative humidity values
        this->DDBeamSolarValues.deallocate();       // Design Day Beam Solar Values
        this->DDDiffuseSolarValues.deallocate();    // Design Day Relative Humidity Values
        this->DDSkyTempScheduleValues.deallocate(); // Sky temperature - DesignDay input
        this->RptIsRain = 0;                        // Rain Report Value
        this->RptIsSnow = 0;                        // Snow Report Value
        this->RptDayType = 0;                       // DayType Report Value

        this->HrAngle = 0.0;                                     // Current Hour Angle
        this->SolarAltitudeAngle = 0.0;                          // Angle of Solar Altitude (degrees)
        this->SolarAzimuthAngle = 0.0;                           // Angle of Solar Azimuth (degrees)
        this->HorizIRSky = 0.0;                                  // Horizontal Infrared Radiation Intensity (W/m2)
        this->TimeStepFraction = 0.0;                            // Fraction of hour each time step represents
        this->SPSiteDryBulbRangeModScheduleValue.deallocate();   // reporting Drybulb Temperature Range Modifier Schedule Value
        this->SPSiteHumidityConditionScheduleValue.deallocate(); // reporting Humidity Condition Schedule Value
        this->SPSiteBeamSolarScheduleValue.deallocate();         // reporting Beam Solar Schedule Value
        this->SPSiteDiffuseSolarScheduleValue.deallocate();      // reporting Diffuse Solar Schedule Value
        this->SPSiteSkyTemperatureScheduleValue.deallocate();    // reporting SkyTemperature Modifier Schedule Value
        this->SPSiteScheduleNamePtr.deallocate();                // SP Site Schedule Name Ptrs
        this->SPSiteScheduleUnits.deallocate();                  // SP Site Schedule Units
        this->NumSPSiteScheduleNamePtrs = 0;                     // Number of SP Site Schedules (DesignDay only)
        this->Interpolation.deallocate();                        // Interpolation values based on Number of Time Steps in Hour
        this->SolarInterpolation.deallocate();                   // Solar Interpolation values based on

        this->LeapYearAdd = 0;
        this->DatesShouldBeReset = false;
        this->StartDatesCycleShouldBeReset = false; // True when start dates on repeat should be reset
        this->Jan1DatesShouldBeReset = false;       // True if Jan 1 should signal reset of dates
        this->TodayVariables = Weather::DayWeatherVariables();
        this->TomorrowVariables = Weather::DayWeatherVariables();
        this->DesignDay.deallocate();
        this->Missing = Weather::MissingData();
        this->Missed = Weather::MissingDataCounts();
        this->OutOfRange = Weather::RangeDataCounts();
        this->DesDayInput.deallocate(); // Design day Input Data
        this->Environment.deallocate(); // Environment data
        this->RunPeriodInput.deallocate();
        this->RunPeriodInputUniqueNames.clear();
        this->RunPeriodDesignInput.deallocate();
        this->RunPeriodDesignInputUniqueNames.clear();
        this->TypicalExtremePeriods.deallocate();

        this->EPWDST.StDateType = Weather::DateType::InvalidDate;
        this->EPWDST.StWeekDay = 0;
        this->EPWDST.StMon = 0;
        this->EPWDST.StDay = 0;
        this->EPWDST.EnDateType = Weather::DateType::InvalidDate;
        this->EPWDST.EnMon = 0;
        this->EPWDST.EnDay = 0;
        this->EPWDST.EnWeekDay = 0;

        this->IDFDST.StDateType = Weather::DateType::InvalidDate;
        this->IDFDST.StWeekDay = 0;
        this->IDFDST.StMon = 0;
        this->IDFDST.StDay = 0;
        this->IDFDST.EnDateType = Weather::DateType::InvalidDate;
        this->IDFDST.EnMon = 0;
        this->IDFDST.EnDay = 0;
        this->IDFDST.EnWeekDay = 0;

        this->DST.StDateType = Weather::DateType::InvalidDate;
        this->DST.StWeekDay = 0;
        this->DST.StMon = 0;
        this->DST.StDay = 0;
        this->DST.EnDateType = Weather::DateType::InvalidDate;
        this->DST.EnMon = 0;
        this->DST.EnDay = 0;
        this->DST.EnWeekDay = 0;
        this->WPSkyTemperature.deallocate();
        this->SpecialDays.deallocate();
        this->DataPeriods.deallocate();

        this->underwaterBoundaries.clear();

        // ManageWeather static vars
        this->PrintEnvrnStamp = false;

        // InitializeWeather static vars
        this->FirstCall = true;
        this->WaterMainsParameterReport = true;

        // SetCurrentWeather static vars
        this->NextHour = 1;

        // ReadEPlusWeatherForDay static vars
        this->CurDayOfWeek = 1;
        this->ReadEPlusWeatherCurTime = 1.0;
        this->LastHourSet = false;

        // SetUpDesignDay static vars
        this->PrintDDHeader = true;

        this->LastHrOutDryBulbTemp = 0.0;
        this->LastHrOutDewPointTemp = 0.0;
        this->LastHrOutBaroPress = 0.0;
        this->LastHrOutRelHum = 0.0;
        this->LastHrWindSpeed = 0.0;
        this->LastHrWindDir = 0.0;
        this->LastHrSkyTemp = 0.0;
        this->LastHrHorizIRSky = 0.0;
        this->LastHrBeamSolarRad = 0.0;
        this->LastHrDifSolarRad = 0.0;
        this->LastHrAlbedo = 0.0;
        this->LastHrLiquidPrecip = 0.0;
        this->LastHrTotalSkyCover = 0.0;
        this->LastHrOpaqueSkyCover = 0.0;
        this->NextHrBeamSolarRad = 0.0;
        this->NextHrDifSolarRad = 0.0;
        this->NextHrLiquidPrecip = 0.0;

        // ProcessEPWHeader static vars
        this->EPWHeaderTitle = "";
    }

    // Default Constructor
    WeatherManagerData()
        : GetBranchInputOneTimeFlag(true), GetEnvironmentFirstCall(true), PrntEnvHeaders(true), FirstCall(true), WaterMainsParameterReport(true),
          PrintEnvrnStamp(false), Sigma(5.6697e-8), YearOfSim(1), NumDaysInYear(365), EnvironmentReportNbr(0), EnvironmentReportChr(""),
          WeatherFileExists(false), LocationGathered(false), WeatherFileLatitude(0.0), WeatherFileLongitude(0.0), WeatherFileTimeZone(0.0),
          WeatherFileElevation(0.0), GroundTempsFCFromEPWHeader(12, 0.0), GroundReflectances(12, 0.2), SnowGndRefModifier(1.0),
          SnowGndRefModifierForDayltg(1.0), WaterMainsTempsSchedule(0), WaterMainsTempsAnnualAvgAirTemp(0.0), WaterMainsTempsMaxDiffAirTemp(0.0),
          WaterMainsTempsScheduleName(""), wthFCGroundTemps(false), TotRunPers(0), TotRunDesPers(0), NumSpecialDays(0), SpecialDayTypes(366, 0),
          WeekDayTypes(366, 0), DSTIndex(366, 0), NumDataPeriods(0), NumIntervalsPerHour(1), UseDaylightSaving(true), UseSpecialDays(true),
          UseRainValues(true), UseSnowValues(true), EPWDaylightSaving(false), IDFDaylightSaving(false), DaylightSavingIsActive(false),
          WFAllowsLeapYears(false), curSimDayForEndOfRunPeriod(0), Envrn(0), NumOfEnvrn(0), NumEPWTypExtSets(0), NumWPSkyTemperatures(0),
          RptIsRain(0), RptIsSnow(0), RptDayType(0), HrAngle(0.0), SolarAltitudeAngle(0.0), SolarAzimuthAngle(0.0), HorizIRSky(0.0),
          TimeStepFraction(0.0), NumSPSiteScheduleNamePtrs(0), EndDayOfMonth(12, {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}), LeapYearAdd(0),
          DatesShouldBeReset(false), StartDatesCycleShouldBeReset(false), Jan1DatesShouldBeReset(false), RPReadAllWeatherData(false)
    {
    }
};
} // namespace EnergyPlus

#endif
