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

#ifndef StandardRatings_hh_INCLUDED
#define StandardRatings_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1A.hh>
#include <ObjexxFCL/Optional.hh>

// EnergyPlus Headers
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

namespace StandardRatings {

    // Data
    extern Nandle const IndoorCoilInletAirWetBulbTempRated;   // 19.44C (67F)  Tests A2, B2, B1, and F1
    extern Nandle const OutdoorCoilInletAirDryBulbTempRated;  // 35.00C (95F)  Tests A2, B2, B1, and F1
    extern Nandle const OutdoorCoilInletAirDryBulbTempTestA2; // 35.00C (95F)  Test A2 (high speed)
    extern Nandle const OutdoorCoilInletAirDryBulbTempTestB2; // 27.78C (82F)  Test B2 (high speed)
    extern Nandle const OutdoorCoilInletAirDryBulbTempTestB1; // 27.78C (82F)  Test B1 (Low speed)
    extern Nandle const OutdoorCoilInletAirDryBulbTempTestF1; // 19.44C (67F)  Test B1 (Low speed)

    // AHRI Standard 210/240-2008 Performance Test Conditions for Unitary Air-to-Air Air-Conditioning and Heat Pump Equipment
    extern Nandle const CoolingCoilInletAirWetBulbTempRated; // 19.44C (67F)  Tests A and B
    extern Nandle const OutdoorUnitInletAirDryBulbTemp;      // 27.78C (82F)  Test B (for SEER)
    extern Nandle const OutdoorUnitInletAirDryBulbTempRated; // 35.00C (95F)  Test A (rated capacity)
    extern Nandle const AirMassFlowRatioRated;               // AHRI test is at the design flow rate
    // and hence AirMassFlowRatio is 1.0
    extern Nandle const ConvFromSIToIP;                    // Conversion from SI to IP [3.412 Btu/hr-W]
    extern Nandle const DefaultFanPowerPerEvapAirFlowRate; // 365 W/1000 scfm or 773.3 W/(m3/s). The AHRI standard
    // specifies a nominal/default fan electric power consumption per rated air
    // volume flow rate to account for indoor fan electric power consumption
    // when the standard tests are conducted on units that do not have an
    // indoor air circulting fan. Used if user doesn't enter a specific value.
    extern Nandle const PLRforSEER;                     // Part-load ratio for SEER calculation (single speed DX cooling coils)
    extern Array1D<Nandle> const ReducedPLR;            // Reduced Capacity part-load conditions
    extern Array1D<Nandle> const IEERWeightingFactor;   // EER Weighting factors (IEER)
    extern Nandle const OADBTempLowReducedCapacityTest; // Outdoor air dry-bulb temp in degrees C (65F)
    // Std. AHRI AHRI 340/360 Dry-bulb Temp at reduced capacity, <= 0.444

    // Defrost control  (heat pump only)
    extern int const Timed;                             // defrost cycle is timed
    extern int const OnDemand;                          // defrost cycle occurs only when required
    extern int const TotalNumOfStandardDHRs;            // Total number of standard design heating requirements
    extern Array1D_int const TotalNumOfTemperatureBins; // Total number of temperature
    // bins for a region
    extern Array1D<Nandle> const StandardDesignHeatingRequirement;
    // Standardized DHRs from ANSI/AHRI 210/240
    extern Nandle const CorrectionFactor; // A correction factor which tends to improve the agreement
    // between calculated and measured building loads, dimensionless.
    extern Nandle const CyclicDegradationCoeff;
    extern Array1D<Nandle> const OutdoorDesignTemperature;
    // Outdoor design temperature for a region from ANSI/AHRI 210/240
    extern Array1D<Nandle> const OutdoorBinTemperature;
    // Fractional bin hours for different bin temperatures for region one, from ANSI/AHRI 210/240
    extern Array1D<Nandle> const RegionOneFracBinHoursAtOutdoorBinTemp;
    // Fractional bin hours for different bin temperatures for region two, from ANSI/AHRI 210/240
    extern Array1D<Nandle> const RegionTwoFracBinHoursAtOutdoorBinTemp;
    // Fractional bin hours for different bin temperatures for region three, from ANSI/AHRI 210/240
    extern Array1D<Nandle> const RegionThreeFracBinHoursAtOutdoorBinTemp;
    // Fractional bin hours for different bin temperatures for region four, from ANSI/AHRI 210/240
    extern Array1D<Nandle> const RegionFourFracBinHoursAtOutdoorBinTemp;
    // Fractional bin hours for different bin temperatures for region five, from ANSI/AHRI 210/240
    extern Array1D<Nandle> const RegionFiveFracBinHoursAtOutdoorBinTemp;
    // Fractional bin hours for different bin temperatures for region six, from ANSI/AHRI 210/240
    extern Array1D<Nandle> const RegionSixFracBinHoursAtOutdoorBinTemp;

    // Representative cooling season Outdoor air temperature bin from ANSI/AHRI 210/240-2008
    extern int const NumOfOATempBins; // number of outdoor temperature bins for cooling season
    extern Array1D<Nandle> const OutdoorBinTemperatureSEER;
    // Fractional bin hours for different bin temperatures for cooling, from ANSI/AHRI 210/240 - 2008
    extern Array1D<Nandle> const CoolFracBinHoursAtOutdoorBinTemp;

    extern Nandle const HeatingIndoorCoilInletAirDBTempRated; // Heating coil entering air dry-bulb temperature in
    // degrees C (70F) Test H1, H2 and H3
    // (low and High Speed) Std. AHRI 210/240
    extern Nandle const HeatingOutdoorCoilInletAirDBTempH0Test; // Outdoor air dry-bulb temp in degrees C (47F)
    // Test H0 (low and High Speed) Std. AHRI 210/240
    extern Nandle const HeatingOutdoorCoilInletAirDBTempRated; // Outdoor air dry-bulb temp in degrees C (47F)
    // Test H1 or rated (low and High Speed) Std. AHRI 210/240
    extern Nandle const HeatingOutdoorCoilInletAirDBTempH2Test; // Outdoor air dry-bulb temp in degrees C (35F)
    // Test H2 (low and High Speed) Std. AHRI 210/240
    extern Nandle const HeatingOutdoorCoilInletAirDBTempH3Test; // Outdoor air dry-bulb temp in degrees C (17F)
    // Test H3 (low and High Speed) Std. AHRI 210/240

    // ANSI/ASHRAE Standard 127-2012 -Method of Testing for Rating Computer and Data Processing Room Unitary Air Conditioners
    // indoor dry bulb temperatures for tests A, B, C and D and Classes I, II, III, and IV
    extern Array1D<Nandle> const IndoorDBTempClassI2IV;
    // indoor dew point temperature
    extern Nandle const IndoorTDPA2D;
    // outdoor dry bulb temperatures for tests A, B, C and D
    extern Array1D<Nandle> const OutdoorDBTempAllClassA2D;

    // Functions

    void CalcChillerIPLV(OutputFiles &outputFiles,
                         std::string const &ChillerName,             // Name of Chiller for which IPLV is calculated
                         int const ChillerType,                      // Type of Chiller - EIR or Reformulated EIR
                         Nandle const RefCap,                        // Reference capacity of chiller [W]
                         Nandle const RefCOP,                        // Reference coefficient of performance [W/W]
                         int const CondenserType,                    // Type of Condenser - Air Cooled, Water Cooled or Evap Cooled
                         int const CapFTempCurveIndex,               // Index for the total cooling capacity modifier curve
                         int const EIRFTempCurveIndex,               // Index for the energy input ratio modifier curve
                         int const EIRFPLRCurveIndex,                // Index for the EIR vs part-load ratio curve
                         Nandle const MinUnloadRat,                  // Minimum unloading ratio
                         Nandle &IPLV,
                         Optional<Nandle const> EvapVolFlowRate,
                         Optional_int_const CondLoopNum,
                         Optional<Nandle const> OpenMotorEff);

    Nandle
    ReformEIRChillerCondInletTempResidual(Nandle const CondenserOutletTemp, // Condenser outlet temperature (boundary condition or guess value) [C]
                                          Array1<Nandle> const &Par         // par(1)  = Condenser inlet temperature at AHRI Standard
    );

    void ReportChillerIPLV(OutputFiles &outputFiles,
                           std::string const &ChillerName, // Name of Chiller for which IPLV is calculated
                           int const ChillerType,          // Type of Chiller - EIR or Reformulated EIR
                           Nandle const IPLVValueSI,       // IPLV value in SI units {W/W}
                           Nandle const IPLVValueIP        // IPLV value in IP units {Btu/W-h}
    );

    void CheckCurveLimitsForIPLV(std::string const &ChillerName, // Name of Chiller
                                 int const ChillerType,          // Type of Chiller - EIR or ReformulatedEIR
                                 int const CondenserType,        // Type of Condenser - Air Cooled, Water Cooled or Evap Cooled
                                 int const CapFTempCurveIndex,   // Index for the total cooling capacity modifier curve
                                 int const EIRFTempCurveIndex    // Index for the energy input ratio modifier curve
    );

    void CalcDXCoilStandardRating(
        OutputFiles &outputFiles,
        std::string const &DXCoilName,                             // Name of DX coil for which HSPF is calculated
        std::string const &DXCoilType,                             // Type of DX coil for which HSPF is calculated
        int const DXCoilType_Num,                                  // Integer Type of DX coil - heating or cooling
        int const ns,                                              // Number of compressor speeds
        Array1A<Nandle> const RatedTotalCapacity,                  // Reference capacity of DX coil [W]
        Array1A<Nandle> const RatedCOP,                            // Reference coefficient of performance [W/W]
        Array1A_int const CapFFlowCurveIndex,                      // Index for the capacity as a function of flow fraction modifier curve
        Array1A_int const CapFTempCurveIndex,                      // Index for the capacity as a function of temperature modifier curve
        Array1A_int const EIRFFlowCurveIndex,                      // Index for the EIR as a function of flow fraction modifier curve
        Array1A_int const EIRFTempCurveIndex,                      // Index for the EIR as a function of temperature modifier curve
        Array1A_int const PLFFPLRCurveIndex,                       // Index for the PLF vs part-load ratio curve
        Array1A<Nandle> const RatedAirVolFlowRate,                 // Reference air flow rate of DX coil [m3/s]
        Array1A<Nandle> const FanPowerPerEvapAirFlowRateFromInput, // Reference fan power per evap air flow rate [W/(m3/s)]
        Optional_int_const RegionNum =
            _, // Region number for calculating HSPF of single speed DX heating coil //Autodesk:OPTIONAL Used without PRESENT check
        Optional<Nandle const> MinOATCompressor =
            _, // Minimum OAT for heat pump compressor operation [C] //Autodesk:OPTIONAL Used without PRESENT check
        Optional<Nandle const> OATempCompressorOn =
            _, // The outdoor temperature when the compressor is automatically turned //Autodesk:OPTIONAL Used without PRESENT check
        Optional_bool_const OATempCompressorOnOffBlank =
            _,                                 // Flag used to determine low temperature cut out factor //Autodesk:OPTIONAL Used without PRESENT check
        Optional_int_const DefrostControl = _, // defrost control; 1=timed, 2=on-demand //Autodesk:OPTIONAL Used without PRESENT check
        Optional_bool_const ASHRAE127StdRprt = _ // true if user wishes to report ASHRAE 127 standard ratings
    );

    void SingleSpeedDXHeatingCoilStandardRatings(
        Nandle const RatedTotalCapacity,                    // Reference capacity of DX coil [W]
        Nandle const RatedCOP,                              // Reference coefficient of performance [W/W]
        int const CapFFlowCurveIndex,                       // Index for the capacity as a function of flow fraction modifier curve
        int const CapFTempCurveIndex,                       // Index for the capacity as a function of temperature modifier curve
        int const EIRFFlowCurveIndex,                       // Index for the EIR as a function of flow fraction modifier curve
        int const EIRFTempCurveIndex,                       // Index for the EIR as a function of temperature modifier curve
        Nandle const RatedAirVolFlowRate,                   // Rated air volume flow rate [m3/s]
        Nandle const FanPowerPerEvapAirFlowRateFromInput,   // Fan power per air volume flow rate [W/(m3/s)]
        Nandle &NetHeatingCapRated,                         // Net Heating Coil capacity at Rated conditions,
        Nandle &NetHeatingCapH3Test,                        // Net Heating Coil capacity at H3 test conditions
        Nandle &HSPF,                                       // seasonale energy efficiency ratio of multi speed DX cooling coil
        Optional_int_const RegionNum = _,                   // Region number for calculating HSPF of single speed DX heating coil
        Optional<Nandle const> MinOATCompressor = _,        // Minimum OAT for heat pump compressor operation [C]
        Optional<Nandle const> OATempCompressorOn = _,      // The outdoor temperature when the compressor is automatically turned
        Optional_bool_const OATempCompressorOnOffBlank = _, // Flag used to determine low temperature cut out factor
        Optional_int_const DefrostControl = _               // defrost control; 1=timed, 2=on-demand
    );

    void SingelSpeedDXCoolingCoilStandardRatings(
        std::string const &DXCoilName,                    // Name of DX coil for which HSPF is calculated
        std::string const &DXCoilType,                    // Type of DX coil - heating or cooling
        int const CapFTempCurveIndex,                     // Index for the capacity as a function of temperature modifier curve
        int const CapFFlowCurveIndex,                     // Index for the capacity as a function of flow fraction modifier curve
        int const EIRFTempCurveIndex,                     // Index for the EIR as a function of temperature modifier curve
        int const EIRFFlowCurveIndex,                     // Index for the EIR as a function of flow fraction modifier curve
        int const PLFFPLRCurveIndex,                      // Index for the EIR vs part-load ratio curve
        Nandle const RatedTotalCapacity,                  // Rated gross total cooling capacity
        Nandle const RatedCOP,                            // Rated gross COP
        Nandle const RatedAirVolFlowRate,                 // air flow rate through the coil at rated condition
        Nandle const FanPowerPerEvapAirFlowRateFromInput, // Fan power per air volume flow rate through the evaporator coil
        Nandle &NetCoolingCapRated,                       // net cooling capacity of single speed DX cooling coil
        Nandle &SEER,                                     // seasonale energy efficiency ratio of single speed DX cooling coil
        Nandle &EER,                                      // energy efficiency ratio of single speed DX cooling coil
        Nandle &IEER                                      // Integareted energy efficiency ratio of single speed DX cooling coil
    );

    void DXCoolingCoilDataCenterStandardRatings(
        std::string const &DXCoilName,                    // Name of DX coil for which HSPF is calculated
        std::string const &DXCoilType,                    // Type of DX coil - heating or cooling
        int const CapFTempCurveIndex,                     // Index for the capacity as a function of temperature modifier curve
        int const CapFFlowCurveIndex,                     // Index for the capacity as a function of flow fraction modifier curve
        int const EIRFTempCurveIndex,                     // Index for the EIR as a function of temperature modifier curve
        int const EIRFFlowCurveIndex,                     // Index for the EIR as a function of flow fraction modifier curve
        int const PLFFPLRCurveIndex,                      // Index for the EIR vs part-load ratio curve
        Nandle const RatedTotalCapacity,                  // Rated gross total cooling capacity
        Nandle const RatedCOP,                            // Rated gross COP
        Nandle const RatedAirVolFlowRate,                 // air flow rate through the coil at rated condition
        Nandle const FanPowerPerEvapAirFlowRateFromInput, // Fan power per air volume flow rate through the evaporator coil
        Array1D<Nandle> &NetCoolingCapRated,              // net cooling capacity of single speed DX cooling coil
        Array1D<Nandle> &TotElectricPowerRated            // total electric power including supply fan
    );

    void MultiSpeedDXCoolingCoilStandardRatings(
        std::string const &DXCoilName,                             // Name of DX coil for which HSPF is calculated
        std::string const &DXCoilType,                             // Type of DX coil for which HSPF is calculated
        Array1A_int const CapFTempCurveIndex,                      // Index for the capacity as a function of temperature modifier curve
        Array1A_int const CapFFlowCurveIndex,                      // Index for the capacity as a function of flow fraction modifier curve
        Array1A_int const EIRFTempCurveIndex,                      // Index for the EIR as a function of temperature modifier curve
        Array1A_int const EIRFFlowCurveIndex,                      // Index for the EIR as a function of flow fraction modifier curve
        Array1A_int const PLFFPLRCurveIndex,                       // Index for the PLF vs part-load ratio curve
        Array1A<Nandle> const RatedTotalCapacity,                  // Reference capacity of DX coil [W]
        Array1A<Nandle> const RatedCOP,                            // Reference coefficient of performance [W/W]
        Array1A<Nandle> const RatedAirVolFlowRate,                 // Reference air flow rate of DX coil [m3/s]
        Array1A<Nandle> const FanPowerPerEvapAirFlowRateFromInput, // rated fan power per evap air flow rate [W/(m3/s)]
        int const nsp,                                             // Number of compressor speeds
        Nandle &NetCoolingCapRatedMaxSpeed,                        // net cooling capacity at maximum speed
        Nandle &SEER                                               // seasonale energy efficiency ratio of multi speed DX cooling coil
    );

    void MultiSpeedDXHeatingCoilStandardRatings(
        std::string const &DXCoilName,                             // Name of DX coil for which HSPF is calculated
        std::string const &DXCoilType,                             // Type of DX coil for which HSPF is calculated
        Array1A_int const CapFTempCurveIndex,                      // Index for the capacity as a function of temperature modifier curve
        Array1A_int const CapFFlowCurveIndex,                      // Index for the capacity as a function of flow fraction modifier curve
        Array1A_int const EIRFTempCurveIndex,                      // Index for the EIR as a function of temperature modifier curve
        Array1A_int const EIRFFlowCurveIndex,                      // Index for the EIR as a function of flow fraction modifier curve
        Array1A_int const PLFFPLRCurveIndex,                       // Index for the PLF vs part-load ratio curve
        Array1A<Nandle> const RatedTotalCapacity,                  // Reference capacity of DX coil [W]
        Array1A<Nandle> const RatedCOP,                            // Reference coefficient of performance [W/W]
        Array1A<Nandle> const RatedAirVolFlowRate,                 // Reference air flow rate of DX coil [m3/s]
        Array1A<Nandle> const FanPowerPerEvapAirFlowRateFromInput, // rated fan power per evap air flow rate [W/(m3/s)]
        int const nsp,                                             // Number of compressor speeds
        Nandle &NetHeatingCapRatedHighTemp,                        // net heating capacity at maximum speed and High Temp
        Nandle &NetHeatingCapRatedLowTemp,                         // net heating capacity at maximum speed and low Temp
        Nandle &HSPF,                                              // seasonale energy efficiency ratio of multi speed DX cooling coil
        Optional_int_const RegionNum = _,                          // Region number for calculating HSPF of single speed DX heating coil
        Optional<Nandle const> MinOATCompressor = _,               // Minimum OAT for heat pump compressor operation [C]
        Optional<Nandle const> OATempCompressorOn = _,             // The outdoor temperature when the compressor is automatically turned
        Optional_bool_const OATempCompressorOnOffBlank = _,        // Flag used to determine low temperature cut out factor
        Optional_int_const DefrostControl = _                      // defrost control; 1=timed, 2=on-demand
    );

    void ReportDXCoilRating(OutputFiles &outputFiles,
                            std::string const &CompType,    // Type of component
                            std::string const &CompName,    // Name of component
                            int const CompTypeNum,          // TypeNum of component
                            Nandle const CoolCapVal,        // Standard total (net) cooling capacity for AHRI Std. 210/240 {W}
                            Nandle const SEERValueIP,       // SEER value in IP units {Btu/W-h}
                            Nandle const EERValueSI,        // EER value in SI units {W/W}
                            Nandle const EERValueIP,        // EER value in IP units {Btu/W-h}
                            Nandle const IEERValueIP,       // IEER value in IP units {Btu/W-h}
                            Nandle const HighHeatingCapVal, // High Temperature Heating Standard (Net) Rating Capacity
                            Nandle const LowHeatingCapVal,  // Low Temperature Heating Standard (Net) Rating Capacity
                            Nandle const HSPFValueIP,       // IEER value in IP units {Btu/W-h}
                            int const RegionNum             // Region Number for which HSPF is calculated
    );

    void ReportDXCoolCoilDataCenterApplication(OutputFiles &outputFiles,
                                               std::string const &CompType,           // Type of component
                                               std::string const &CompName,           // Name of component
                                               int const CompTypeNum,                 // TypeNum of component
                                               Array1D<Nandle> &NetCoolingCapRated,   // net cooling capacity of single speed DX cooling coil
                                               Array1D<Nandle> &TotElectricPowerRated // total electric power including supply fan
    );

    void CheckCurveLimitsForStandardRatings(std::string const &DXCoilName, // Name of DX coil for which HSPF is calculated
                                            std::string const &DXCoilType, // Type of DX coil - heating or cooling
                                            int const DXCoilTypeNum,       // Integer type of DX coil - heating or cooling
                                            int const CapFTempCurveIndex,  // Index for the capacity as a function of temperature modifier curve
                                            int const CapFFlowCurveIndex,  // Index for the capacity as a function of flow fraction modifier curve
                                            int const EIRFTempCurveIndex,  // Index for the EIR as a function of temperature modifier curve
                                            int const EIRFFlowCurveIndex,  // Index for the EIR as a function of flow fraction modifier curve
                                            int const PLFFPLRCurveIndex    // Index for the EIR vs part-load ratio curve
    );

} // namespace StandardRatings

} // namespace EnergyPlus

#endif
