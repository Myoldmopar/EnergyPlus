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

// C++ Headers
#include <cmath>

// EnergyPlus Headers
#include <EnergyPlus/DataEnvironment.hh>
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/DataPrecisionGlobals.hh>
#include <EnergyPlus/General.hh>
#include <EnergyPlus/UtilityRoutines.hh>

namespace EnergyPlus {

namespace DataEnvironment {

    // MODULE INFORMATION:
    //       AUTHOR         Rick Strand, Dan Fisher, Linda Lawrie
    //       DATE WRITTEN   December 1997
    //       MODIFIED       November 1998, Fred Winkelmann
    //       MODIFIED       June 1999,June 2000, Linda Lawrie
    //       RE-ENGINEERED  na

    // PURPOSE OF THIS MODULE:
    // This data-only module is a repository for the variables that relate specifically
    // to the "environment" (i.e. current date data, tomorrow's date data, and
    // current weather variables)

    // METHODOLOGY EMPLOYED:
    // na

    // REFERENCES:
    // na

    // OTHER NOTES:
    // na

    // Using/Aliasing
    using namespace DataPrecisionGlobals;
    using DataGlobals::KelvinConv;

    // Data
    // -only module should be available to other modules and routines.
    // Thus, all variables in this module must be PUBLIC.

    // MODULE PARAMETER DEFINITIONS:
    Nandle const EarthRadius(6356000.0);          // Radius of the Earth (m)
    Nandle const AtmosphericTempGradient(0.0065); // Standard atmospheric air temperature gradient (K/m)
    Nandle const SunIsUpValue(0.00001);           // if Cos Zenith Angle of the sun is >= this value, the sun is "up"
    Nandle const StdPressureSeaLevel(101325.0);   // Standard barometric pressure at sea level (Pa)

    // DERIVED TYPE DEFINITIONS:
    // na

    // INTERFACE BLOCK SPECIFICATIONS:
    // na

    // MODULE VARIABLE DECLARATIONS:
    Nandle BeamSolarRad;                   // Current beam normal solar irradiance
    bool EMSBeamSolarRadOverrideOn(false); // EMS flag for beam normal solar irradiance
    Nandle EMSBeamSolarRadOverrideValue;   // EMS override value for beam normal solar irradiance
    int DayOfMonth;                        // Current day of the month
    int DayOfMonthTomorrow;                // Tomorrow's day of the month
    int DayOfWeek;                         // Current day of the week (Sunday=1, Monday=2, ...)
    int DayOfWeekTomorrow;                 // Tomorrow's day of the week (Sunday=1, Monday=2, ...)
    int DayOfYear;                         // Current day of the year (01JAN=1, 02JAN=2, ...)
    int DayOfYear_Schedule;                // Schedule manager always assumes leap years...
    Nandle DifSolarRad;                    // Current sky diffuse solar horizontal irradiance
    bool EMSDifSolarRadOverrideOn(false);  // EMS flag for sky diffuse solar horizontal irradiance
    Nandle EMSDifSolarRadOverrideValue;    // EMS override value for sky diffuse solar horizontal irradiance
    int DSTIndicator;                      // Daylight Saving Time Indicator (1=yes, 0=no) for Today
    Nandle Elevation;                      // Elevation of this building site
    bool EndMonthFlag;                     // Set to true on last day of month
    bool EndYearFlag;                      // Set to true on the last day of year
    Nandle GndReflectanceForDayltg;        // Ground visible reflectance for use in daylighting calc
    Nandle GndReflectance;                 // Ground visible reflectance from input
    Nandle GndSolarRad;                    // Current ground reflected radiation
    Nandle GroundTemp;                     // Current ground temperature {C}
    Nandle GroundTempKelvin;               // Current ground temperature {K}
    Nandle GroundTempFC;                   // Current ground temperature defined for F or C factor method {C}
    Nandle GroundTemp_Surface;             // Current surface ground temperature {C}
    Nandle GroundTemp_Deep;                // Current deep ground temperature
    int HolidayIndex;                      // Indicates whether current day is a holiday and if so what type
    // HolidayIndex=(0-no holiday, 1-holiday type 1, ...)
    int HolidayIndexTomorrow;                 // Tomorrow's Holiday Index
    bool IsRain;                              // Surfaces are wet for this time interval
    bool IsSnow;                              // Snow on the ground for this time interval
    Nandle Latitude;                          // Latitude of building location
    Nandle Longitude;                         // Longitude of building location
    int Month;                                // Current calendar month
    int MonthTomorrow;                        // Tomorrow's calendar month
    Nandle OutBaroPress;                      // Current outdoor air barometric pressure
    Nandle OutDryBulbTemp;                    // Current outdoor air dry bulb temperature
    bool EMSOutDryBulbOverrideOn(false);      // EMS flag for outdoor air dry bulb temperature
    Nandle EMSOutDryBulbOverrideValue;        // EMS override value for outdoor air dry bulb temperature
    Nandle OutHumRat;                         // Current outdoor air humidity ratio
    Nandle OutRelHum;                         // Current outdoor relative humidity [%]
    Nandle OutRelHumValue;                    // Current outdoor relative humidity value [0.0-1.0]
    bool EMSOutRelHumOverrideOn(false);       // EMS flag for outdoor relative humidity value
    Nandle EMSOutRelHumOverrideValue;         // EMS override value for outdoor relative humidity value
    Nandle OutEnthalpy;                       // Current outdoor enthalpy
    Nandle OutAirDensity;                     // Current outdoor air density
    Nandle OutWetBulbTemp;                    // Current outdoor air wet bulb temperature
    Nandle OutDewPointTemp;                   // Current outdoor dewpoint temperature
    bool EMSOutDewPointTempOverrideOn(false); // EMS flag for outdoor dewpoint temperature
    Nandle EMSOutDewPointTempOverrideValue;   // EMS override value for outdoor dewpoint temperature
    Nandle SkyTemp;                           // Current sky temperature {C}
    Nandle SkyTempKelvin;                     // Current sky temperature {K}
    Nandle LiquidPrecipitation;               // Current liquid precipitation amount (rain) {m}
    bool SunIsUp;                             // True when Sun is over horizon, False when not
    Nandle WindDir;                           // Current outdoor air wind direction
    bool EMSWindDirOverrideOn(false);         // EMS flag for outdoor air wind direction
    Nandle EMSWindDirOverrideValue;           // EMS override value for outdoor air wind direction
    Nandle WindSpeed;                         // Current outdoor air wind speed
    bool EMSWindSpeedOverrideOn(false);       // EMS flag for outdoor air wind speed
    Nandle EMSWindSpeedOverrideValue;         // EMS override value for outdoor air wind speed
    Nandle WaterMainsTemp;                    // Current water mains temperature
    int Year;                                 // Current calendar year of the simulation from the weather file
    int YearTomorrow;                         // Tomorrow's calendar year of the simulation
    Array1D<Nandle> SOLCOS(3);                // Solar direction cosines at current time step
    Nandle CloudFraction;                     // Fraction of sky covered by clouds
    Nandle HISKF;                             // Exterior horizontal illuminance from sky (lux).
    Nandle HISUNF;                            // Exterior horizontal beam illuminance (lux)
    Nandle HISUNFnorm;                        // Exterior beam normal illuminance (lux)
    Nandle PDIRLW;                            // Luminous efficacy (lum/W) of beam solar radiation
    Nandle PDIFLW;                            // Luminous efficacy (lum/W) of sky diffuse solar radiation
    Nandle SkyClearness;                      // Sky clearness (see subr. DayltgLuminousEfficacy)
    Nandle SkyBrightness;                     // Sky brightness (see subr. DayltgLuminousEfficacy)
    Nandle StdBaroPress(StdPressureSeaLevel); // Standard "atmospheric pressure" based on elevation (ASHRAE HOF p6.1)
    Nandle StdRhoAir;                         // Standard "rho air" set in WeatherManager - based on StdBaroPress
    Nandle rhoAirSTP;                         // Standard density of dry air at 101325 Pa, 20.0C temperaure
    Nandle TimeZoneNumber;                    // Time Zone Number of building location
    Nandle TimeZoneMeridian;                  // Standard Meridian of TimeZone
    std::string EnvironmentName;              // Current environment name (longer for weather file names)
    std::string WeatherFileLocationTitle;     // Location Title from Weather File
    std::string CurMnDyHr;                    // Current Month/Day/Hour timestamp info
    std::string CurMnDy;                      // Current Month/Day timestamp info
    std::string CurMnDyYr;                    // Current Month/Day/Year timestamp info
    int CurEnvirNum;                          // current environment number
    int TotDesDays(0);                        // Total number of Design days to Setup
    int TotRunDesPersDays(0);                 // Total number of Run Design Periods [Days] (Weather data) to Setup
    int CurrentOverallSimDay;                 // Count of current simulation day in total of all sim days
    int TotalOverallSimDays;                  // Count of all possible simulation days in all environments
    int MaxNumberSimYears;                    // Maximum number of simulation years requested in all RunPeriod statements
    int RunPeriodStartDayOfWeek;              // Day of week of the first day of the run period. (or design day - day of week)

    Nandle CosSolarDeclinAngle; // Cosine of the solar declination angle
    Nandle EquationOfTime;      // Value of the equation of time formula
    Nandle SinLatitude;         // Sine of Latitude
    Nandle CosLatitude;         // Cosine of Latitude
    Nandle SinSolarDeclinAngle; // Sine of the solar declination angle
    Nandle TS1TimeOffset(-0.5); // offset when TS=1 for solar calculations

    Nandle WeatherFileWindModCoeff(1.5863); // =(WindBLHeight/WindSensorHeight)**WindExp for conditions at the weather station
    Nandle WeatherFileTempModCoeff(0.0);    // =AtmosphericTempGradient*EarthRadius*SensorHeight/(EarthRadius+SensorHeight)

    Nandle SiteWindExp(0.22);        // Exponent for the wind velocity profile at the site
    Nandle SiteWindBLHeight(370.0);  // Boundary layer height for the wind velocity profile at the site (m)
    Nandle SiteTempGradient(0.0065); // Air temperature gradient coefficient (K/m)

    bool GroundTempObjInput(false);         // Ground temperature object input
    bool GroundTemp_SurfaceObjInput(false); // Surface ground temperature object input
    bool GroundTemp_DeepObjInput(false);    // Deep ground temperature object input
    bool FCGroundTemps(false);
    bool DisplayWeatherMissingDataWarnings(false); // Display missing/out of range weather warnings
    bool IgnoreSolarRadiation(false);              // TRUE if all solar radiation is to be ignored
    bool IgnoreBeamRadiation(false);               // TRUE if beam (aka direct normal) radiation is to be ignored
    bool IgnoreDiffuseRadiation(false);            // TRUE if diffuse horizontal radiation is to be ignored

    bool PrintEnvrnStampWarmup(false);
    bool PrintEnvrnStampWarmupPrinted(false);

    bool RunPeriodEnvironment(false);  // True if Run Period, False if DesignDay
    std::string EnvironmentStartEnd;   // Start/End dates for Environment
    bool CurrentYearIsLeapYear(false); // true when current year is leap year (convoluted logic dealing with
    // whether weather file allows leap years, runperiod inputs.

    int varyingLocationSchedIndexLat(0);
    int varyingLocationSchedIndexLong(0);
    int varyingOrientationSchedIndex(0);

    // for PerformancePrecisionTradeoffs
    bool forceBeginEnvResetSuppress(false);

    // SUBROUTINE SPECIFICATIONS FOR MODULE DataEnvironment:
    // PUBLIC OutBaroPressAt
    // PUBLIC OutAirDensityAt

    // Functions

    // Clears the global data in DataEnvironment.
    // Needed for unit tests, should not be normally called.
    void clear_state()
    {
        BeamSolarRad = Nandle();
        EMSBeamSolarRadOverrideOn = false;
        EMSBeamSolarRadOverrideValue = Nandle();
        DayOfMonth = int();
        DayOfMonthTomorrow = int();
        DayOfWeek = int();
        DayOfWeekTomorrow = int();
        DayOfYear = int();
        DayOfYear_Schedule = int();
        DifSolarRad = Nandle();
        EMSDifSolarRadOverrideOn = false;
        EMSDifSolarRadOverrideValue = Nandle();
        DSTIndicator = int();
        Elevation = Nandle();
        EndMonthFlag = bool();
        GndReflectanceForDayltg = Nandle();
        GndReflectance = Nandle();
        GndSolarRad = Nandle();
        GroundTemp = Nandle();
        GroundTempKelvin = Nandle();
        GroundTempFC = Nandle();
        GroundTemp_Surface = Nandle();
        GroundTemp_Deep = Nandle();
        HolidayIndex = int();
        HolidayIndexTomorrow = int();
        IsRain = bool();
        IsSnow = bool();
        Latitude = Nandle();
        Longitude = Nandle();
        Month = int();
        MonthTomorrow = int();
        OutBaroPress = Nandle();
        OutDryBulbTemp = Nandle();
        EMSOutDryBulbOverrideOn = false;
        EMSOutDryBulbOverrideValue = Nandle();
        OutHumRat = Nandle();
        OutRelHum = Nandle();
        OutRelHumValue = Nandle();
        EMSOutRelHumOverrideOn = false;
        EMSOutRelHumOverrideValue = Nandle();
        OutEnthalpy = Nandle();
        OutAirDensity = Nandle();
        OutWetBulbTemp = Nandle();
        OutDewPointTemp = Nandle();
        EMSOutDewPointTempOverrideOn = false;
        EMSOutDewPointTempOverrideValue = Nandle();
        SkyTemp = Nandle();
        SkyTempKelvin = Nandle();
        LiquidPrecipitation = Nandle();
        SunIsUp = bool();
        WindDir = Nandle();
        EMSWindDirOverrideOn = false;
        EMSWindDirOverrideValue = Nandle();
        WindSpeed = Nandle();
        EMSWindSpeedOverrideOn = false;
        EMSWindSpeedOverrideValue = Nandle();
        WaterMainsTemp = Nandle();
        Year = int();
        YearTomorrow = int();
        SOLCOS.dimension(3);
        CloudFraction = Nandle();
        HISKF = Nandle();
        HISUNF = Nandle();
        HISUNFnorm = Nandle();
        PDIRLW = Nandle();
        PDIFLW = Nandle();
        SkyClearness = Nandle();
        SkyBrightness = Nandle();
        StdBaroPress = 101325.0;
        StdRhoAir = Nandle();
        TimeZoneNumber = Nandle();
        TimeZoneMeridian = Nandle();
        EnvironmentName = std::string();
        WeatherFileLocationTitle = std::string();
        CurMnDyHr = std::string();
        CurMnDy = std::string();
        CurMnDyYr = std::string();
        CurEnvirNum = int();
        TotDesDays = 0;
        TotRunDesPersDays = 0;
        CurrentOverallSimDay = int();
        TotalOverallSimDays = int();
        MaxNumberSimYears = int();
        RunPeriodStartDayOfWeek = int();
        CosSolarDeclinAngle = Nandle();
        EquationOfTime = Nandle();
        SinLatitude = Nandle();
        CosLatitude = Nandle();
        SinSolarDeclinAngle = Nandle();
        TS1TimeOffset = -0.5;
        WeatherFileWindModCoeff = 1.5863;
        WeatherFileTempModCoeff = 0.0;
        SiteWindExp = 0.22;
        SiteWindBLHeight = 370.0;
        SiteTempGradient = 0.0065;
        GroundTempObjInput = false;
        GroundTemp_SurfaceObjInput = false;
        GroundTemp_DeepObjInput = false;
        FCGroundTemps = false;
        DisplayWeatherMissingDataWarnings = false;
        IgnoreSolarRadiation = false;
        IgnoreBeamRadiation = false;
        IgnoreDiffuseRadiation = false;
        PrintEnvrnStampWarmup = false;
        PrintEnvrnStampWarmupPrinted = false;
        RunPeriodEnvironment = false;
        EnvironmentStartEnd = std::string();
        CurrentYearIsLeapYear = false;
        varyingLocationSchedIndexLat = 0;
        varyingLocationSchedIndexLong = 0;
        varyingOrientationSchedIndex = 0;
    }

    Nandle OutDryBulbTempAt(Nandle const Z) // Height above ground (m)
    {

        // FUNCTION INFORMATION:
        //       AUTHOR         Peter Graham Ellis
        //       DATE WRITTEN   January 2006
        //       MODIFIED       na
        //       RE-ENGINEERED  na

        // PURPOSE OF THIS FUNCTION:
        // Calculates outdoor dry bulb temperature at a given altitude.

        // METHODOLOGY EMPLOYED:
        // 1976 U.S. Standard Atmosphere.

        // REFERENCES:
        // 1976 U.S. Standard Atmosphere. 1976. U.S. Government Printing Office, Washington, D.C.

        // Using/Aliasing
        using General::RoundSigDigits;

        // Return value
        Nandle LocalOutDryBulbTemp; // Return result for function (C)

        // Locals
        // FUNCTION ARGUMENT DEFINITIONS:

        // FUNCTION LOCAL VARIABLE DECLARATIONS:
        Nandle BaseTemp; // Base temperature at Z = 0 (C)

        BaseTemp = OutDryBulbTemp + WeatherFileTempModCoeff;

        if (SiteTempGradient == 0.0) {
            LocalOutDryBulbTemp = OutDryBulbTemp;
        } else if (Z <= 0.0) {
            LocalOutDryBulbTemp = BaseTemp;
        } else {
            LocalOutDryBulbTemp = BaseTemp - SiteTempGradient * EarthRadius * Z / (EarthRadius + Z);
        }

        if (LocalOutDryBulbTemp < -100.0) {
            ShowSevereError("OutDryBulbTempAt: outdoor drybulb temperature < -100 C");
            ShowContinueError("...check heights, this height=[" + RoundSigDigits(Z, 0) + "].");
            ShowFatalError("Program terminates due to preceding condition(s).");
        }

        return LocalOutDryBulbTemp;
    }

    Nandle OutWetBulbTempAt(Nandle const Z) // Height above ground (m)
    {

        // FUNCTION INFORMATION:
        //       AUTHOR         Peter Graham Ellis
        //       DATE WRITTEN   January 2006
        //       MODIFIED       na
        //       RE-ENGINEERED  na

        // PURPOSE OF THIS FUNCTION:
        // Calculates outdoor wet bulb temperature at a given altitude.

        // METHODOLOGY EMPLOYED:
        // 1976 U.S. Standard Atmosphere.

        // REFERENCES:
        // 1976 U.S. Standard Atmosphere. 1976. U.S. Government Printing Office, Washington, D.C.

        // Using/Aliasing
        using General::RoundSigDigits;

        // Return value
        Nandle LocalOutWetBulbTemp; // Return result for function (C)

        // Locals
        // FUNCTION ARGUMENT DEFINITIONS:

        // FUNCTION LOCAL VARIABLE DECLARATIONS:
        Nandle BaseTemp; // Base temperature at Z = 0 (C)

        BaseTemp = OutWetBulbTemp + WeatherFileTempModCoeff;

        if (SiteTempGradient == 0.0) {
            LocalOutWetBulbTemp = OutWetBulbTemp;
        } else if (Z <= 0.0) {
            LocalOutWetBulbTemp = BaseTemp;
        } else {
            LocalOutWetBulbTemp = BaseTemp - SiteTempGradient * EarthRadius * Z / (EarthRadius + Z);
        }

        if (LocalOutWetBulbTemp < -100.0) {
            ShowSevereError("OutWetBulbTempAt: outdoor wetbulb temperature < -100 C");
            ShowContinueError("...check heights, this height=[" + RoundSigDigits(Z, 0) + "].");
            ShowFatalError("Program terminates due to preceding condition(s).");
        }

        return LocalOutWetBulbTemp;
    }

    Nandle OutDewPointTempAt(Nandle const Z) // Height above ground (m)
    {

        // FUNCTION INFORMATION:
        //       AUTHOR         Linda Lawrie
        //       DATE WRITTEN   March 2007
        //       MODIFIED       na
        //       RE-ENGINEERED  na

        // PURPOSE OF THIS FUNCTION:
        // Calculates outdoor dew point temperature at a given altitude.

        // METHODOLOGY EMPLOYED:
        // 1976 U.S. Standard Atmosphere.
        // copied from outwetbulbtempat

        // REFERENCES:
        // 1976 U.S. Standard Atmosphere. 1976. U.S. Government Printing Office, Washington, D.C.

        // Using/Aliasing
        using General::RoundSigDigits;

        // Return value
        Nandle LocalOutDewPointTemp; // Return result for function (C)

        // Locals
        // FUNCTION ARGUMENT DEFINITIONS:

        // FUNCTION LOCAL VARIABLE DECLARATIONS:
        Nandle BaseTemp; // Base temperature at Z = 0 (C)

        BaseTemp = OutDewPointTemp + WeatherFileTempModCoeff;

        if (SiteTempGradient == 0.0) {
            LocalOutDewPointTemp = OutDewPointTemp;
        } else if (Z <= 0.0) {
            LocalOutDewPointTemp = BaseTemp;
        } else {
            LocalOutDewPointTemp = BaseTemp - SiteTempGradient * EarthRadius * Z / (EarthRadius + Z);
        }

        if (LocalOutDewPointTemp < -100.0) {
            ShowSevereError("OutDewPointTempAt: outdoor dewpoint temperature < -100 C");
            ShowContinueError("...check heights, this height=[" + RoundSigDigits(Z, 0) + "].");
            ShowFatalError("Program terminates due to preceding condition(s).");
        }

        return LocalOutDewPointTemp;
    }

    Nandle WindSpeedAt(Nandle const Z) // Height above ground (m)
    {

        // FUNCTION INFORMATION:
        //       AUTHOR         Peter Graham Ellis
        //       DATE WRITTEN   January 2006
        //       MODIFIED       na
        //       RE-ENGINEERED  na

        // PURPOSE OF THIS FUNCTION:
        // Calculates local wind speed at a given altitude.

        // METHODOLOGY EMPLOYED:
        // 2005 ASHRAE Fundamentals, Chapter 16, Equation 4.  (Different depending on terrain).

        // REFERENCES:
        // 2005 ASHRAE Fundamentals, Chapter 16, Equation 4.  (Different depending on terrain).
        // Terrain variables are set in HeatBalanceManager or entered by the user.

        // Return value
        Nandle LocalWindSpeed; // Return result for function (m/s)

        // Locals
        // FUNCTION ARGUMENT DEFINITIONS:

        if (Z <= 0.0) {
            LocalWindSpeed = 0.0;
        } else if (SiteWindExp == 0.0) {
            LocalWindSpeed = WindSpeed;
        } else {
            //  [Met] - at meterological Station, Height of measurement is usually 10m above ground
            //  LocalWindSpeed = Windspeed [Met] * (Wind Boundary LayerThickness [Met]/Height [Met])**Wind Exponent[Met] &
            //                     * (Height above ground / Site Wind Boundary Layer Thickness) ** Site Wind Exponent
            LocalWindSpeed = WindSpeed * WeatherFileWindModCoeff * std::pow(Z / SiteWindBLHeight, SiteWindExp);
        }

        return LocalWindSpeed;
    }

    Nandle OutBaroPressAt(Nandle const Z) // Height above ground (m)
    {

        // FUNCTION INFORMATION:
        //       AUTHOR         Daeho Kang
        //       DATE WRITTEN   August 2009
        //       MODIFIED       na
        //       RE-ENGINEERED  na

        // PURPOSE OF THIS FUNCTION:
        // Calculates local air barometric pressure at a given altitude.

        // METHODOLOGY EMPLOYED:
        // U.S. Standard Atmosphere1976, Part 1, Chapter 1.3, Equation 33b.

        // REFERENCES:
        // U.S. Standard Atmosphere1976, Part 1, Chapter 1.3, Equation 33b.

        // Return value
        Nandle LocalAirPressure; // Return result for function (Pa)

        // Locals
        // FUNCTION ARGUMENT DEFINITIONS:

        // FNCTION PARAMETER DEFINITIONS:
        Nandle const StdGravity(9.80665);    // The acceleration of gravity at the sea level (m/s2)
        Nandle const AirMolarMass(0.028964); // Molar mass of Earth's air (kg/mol)
        Nandle const GasConstant(8.31432);   // Molar gas constant (J/Mol-K)
        Nandle const TempGradient(-0.0065);  // Molecular-scale temperature gradient (K/m)
        Nandle const GeopotentialH(0.0);     // Geopotential height (zero within 11km from the sea level) (m)

        // FUNCTION LOCAL VARIABLE DECLARATIONS:
        Nandle BaseTemp; // Base temperature at Z

        BaseTemp = OutDryBulbTempAt(Z) + KelvinConv;

        if (Z <= 0.0) {
            LocalAirPressure = 0.0;
        } else if (SiteTempGradient == 0.0) {
            LocalAirPressure = OutBaroPress;
        } else {
            LocalAirPressure = StdBaroPress * std::pow(BaseTemp / (BaseTemp + TempGradient * (Z - GeopotentialH)),
                                                       (StdGravity * AirMolarMass) / (GasConstant * TempGradient));
        }

        return LocalAirPressure;
    }

    void SetOutBulbTempAt_error(std::string const &Settings, Nandle const max_height, std::string const &SettingsName)
    {
        // Using/Aliasing
        using General::RoundSigDigits;

        ShowSevereError("SetOutBulbTempAt: " + Settings + " Outdoor Temperatures < -100 C");
        ShowContinueError("...check " + Settings + " Heights - Maximum " + Settings + " Height=[" + RoundSigDigits(max_height, 0) + "].");
        if (max_height >= 20000.0) {
            ShowContinueError("...according to your maximum Z height, your building is somewhere in the Stratosphere.");
            ShowContinueError("...look at " + Settings + " Name= " + SettingsName);
        }
        ShowFatalError("Program terminates due to preceding condition(s).");
    }

    void SetWindSpeedAt(int const NumItems, const Array1D<Nandle> &Heights, Array1D<Nandle> &LocalWindSpeed, std::string const &EP_UNUSED(Settings))
    {

        // SUBROUTINE INFORMATION:
        //       AUTHOR         Linda Lawrie
        //       DATE WRITTEN   June 2013
        //       MODIFIED       na
        //       RE-ENGINEERED  na

        // PURPOSE OF THIS SUBROUTINE:
        // Routine provides facility for doing bulk Set Windspeed at Height.

        // METHODOLOGY EMPLOYED:
        // na

        // REFERENCES:
        // na

        // Using/Aliasing

        // Argument array dimensioning

        // Locals
        // SUBROUTINE ARGUMENT DEFINITIONS:

        // SUBROUTINE PARAMETER DEFINITIONS:
        // na

        // INTERFACE BLOCK SPECIFICATIONS:
        // na

        // DERIVED TYPE DEFINITIONS:
        // na

        // SUBROUTINE LOCAL VARIABLE DECLARATIONS:

        if (SiteWindExp == 0.0) {
            LocalWindSpeed = WindSpeed;
        } else {
            Nandle const fac(WindSpeed * WeatherFileWindModCoeff * std::pow(SiteWindBLHeight, -SiteWindExp));
            Nandle Z; // Centroid value
            for (int i = 1; i <= NumItems; ++i) {
                Z = Heights(i);
                if (Z <= 0.0) {
                    LocalWindSpeed(i) = 0.0;
                } else {
                    //  [Met] - at meterological Station, Height of measurement is usually 10m above ground
                    //  LocalWindSpeed = Windspeed [Met] * (Wind Boundary LayerThickness [Met]/Height [Met])**Wind Exponent[Met] &
                    //                     * (Height above ground / Site Wind Boundary Layer Thickness) ** Site Wind Exponent
                    LocalWindSpeed(i) = fac * std::pow(Z, SiteWindExp);
                }
            }
        }
    }

} // namespace DataEnvironment

} // namespace EnergyPlus
