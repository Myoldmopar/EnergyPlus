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

#ifndef DataHeatBalFanSys_hh_INCLUDED
#define DataHeatBalFanSys_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>

// EnergyPlus Headers
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

namespace DataHeatBalFanSys {

    // Data
    // -only module should be available to other modules and routines.
    // Thus, all variables in this module must be PUBLIC.

    // MODULE PARAMETER DEFINITIONS:
    extern int const UseSimpleAirFlow;
    extern Nandle const MaxRadHeatFlux; // [W/m2] max limit for radiant heat flux at a surface due to HVAC equipment

    // Controls for PredictorCorrector
    extern int const iGetZoneSetPoints;
    extern int const iPredictStep;
    extern int const iCorrectStep;
    extern int const iRevertZoneTimestepHistories;
    extern int const iPushZoneTimestepHistories;
    extern int const iPushSystemTimestepHistories;

    // DERIVED TYPE DEFINITIONS:

    // MODULE VARIABLE DECLARATIONS:
    extern Array1D<Nandle> SumConvHTRadSys;         // Sum of convection to zone air from hi temp radiant heaters
    extern Array1D<Nandle> SumLatentHTRadSys;       // Sum of latent gains from hi temp radiant heaters
    extern Array1D<Nandle> SumConvPool;             // Sum of convection to zone air from pools
    extern Array1D<Nandle> SumLatentPool;           // Sum of latent gains from pools
    extern Array1D<Nandle> QHTRadSysToPerson;       // Sum of radiant gains to people from hi temp radiant heaters
    extern Array1D<Nandle> QHWBaseboardToPerson;    // Sum of radiant gains to people from hot water baseboard heaters
    extern Array1D<Nandle> QSteamBaseboardToPerson; // Sum of radiant gains to people from steam baseboard heaters
    extern Array1D<Nandle> QElecBaseboardToPerson;  // Sum of radiant gains to people from electric baseboard heaters
    extern Array1D<Nandle> QCoolingPanelToPerson;   // Sum of radiant losses to people from cooling panels
    // Zone air drybulb conditions variables
    extern Array1D<Nandle> ZTAV;         // Zone Air Temperature Averaged over the Zone Time step
    extern Array1D<Nandle> MAT;          // MEAN AIR TEMPARATURE (C)
    extern Array1D<Nandle> TempTstatAir; // temperature of air near the thermo stat
    extern Array1D<Nandle> ZT;           // Zone Air Temperature Averaged over the System Time Increment
    extern Array1D<Nandle> XMAT;         // TEMPORARY ZONE TEMPERATURE TO TEST CONVERGENCE
    extern Array1D<Nandle> XM2T;
    extern Array1D<Nandle> XM3T;
    extern Array1D<Nandle> XM4T;
    extern Array1D<Nandle> DSXMAT; // Down Stepped MAT history storage
    extern Array1D<Nandle> DSXM2T; // Down Stepped MAT history storage
    extern Array1D<Nandle> DSXM3T; // Down Stepped MAT history storage
    extern Array1D<Nandle> DSXM4T; // Down Stepped MAT history storage
    extern Array1D<Nandle> XMPT;   // Zone air temperature at previous time step

    extern Array1D<Nandle> ZTAVComf; // Zone Air Temperature Averaged over the Zone Time step used
    // in thermal comfort models (currently Fang model only)
    extern Array1D<Nandle> ZoneAirHumRatAvgComf; // AIR Humidity Ratio averaged over the zone time
    // step used in thermal comfort models (currently Fang model only)

    // Zone Air moisture conditions variables
    extern Array1D<Nandle> ZoneAirHumRatAvg;  // AIR Humidity Ratio averaged over the zone time step
    extern Array1D<Nandle> ZoneAirHumRat;     // AIR Humidity Ratio
    extern Array1D<Nandle> WZoneTimeMinus1;   // Humidity ratio history terms for 3rd order derivative
    extern Array1D<Nandle> WZoneTimeMinus2;   // Time Minus 2 Zone Time Steps Term
    extern Array1D<Nandle> WZoneTimeMinus3;   // Time Minus 3 Zone Time Steps Term
    extern Array1D<Nandle> WZoneTimeMinus4;   // Time Minus 4 Zone Time Steps Term
    extern Array1D<Nandle> DSWZoneTimeMinus1; // DownStepped Humidity ratio history terms for 3rd order derivative
    extern Array1D<Nandle> DSWZoneTimeMinus2; // DownStepped Time Minus 2 Zone Time Steps Term
    extern Array1D<Nandle> DSWZoneTimeMinus3; // DownStepped Time Minus 3 Zone Time Steps Term
    extern Array1D<Nandle> DSWZoneTimeMinus4; // DownStepped Time Minus 4 Zone Time Steps Term
    extern Array1D<Nandle> WZoneTimeMinusP;   // Humidity ratio history terms at previous time step

    extern Array1D<Nandle> ZoneAirHumRatTemp;   // Temp zone air humidity ratio at time plus 1
    extern Array1D<Nandle> WZoneTimeMinus1Temp; // Zone air humidity ratio at previous timestep
    extern Array1D<Nandle> WZoneTimeMinus2Temp; // Zone air humidity ratio at timestep T-2
    extern Array1D<Nandle> WZoneTimeMinus3Temp; // Zone air humidity ratio at timestep T-3
    extern Array1D<Nandle> ZoneAirHumRatOld;    // Last Time Steps Zone AIR Humidity Ratio

    extern Array1D<Nandle> MCPI;                       // INFILTRATION MASS FLOW * AIR SPECIFIC HEAT
    extern Array1D<Nandle> MCPTI;                      // INFILTRATION MASS FLOW * AIR CP * AIR TEMPERATURE
    extern Array1D<Nandle> MCPV;                       // VENTILATION MASS FLOW * AIR SPECIFIC HEAT
    extern Array1D<Nandle> MCPTV;                      // VENTILATION MASS FLOW * AIR CP * AIR TEMPERATURE
    extern Array1D<Nandle> MCPM;                       // Mixing MASS FLOW * AIR SPECIFIC HEAT
    extern Array1D<Nandle> MCPTM;                      // Mixing MASS FLOW * AIR CP * AIR TEMPERATURE
    extern Array1D<Nandle> MCPE;                       // EARTHTUBE MASS FLOW * AIR SPECIFIC HEAT
    extern Array1D<Nandle> EAMFL;                      // OUTDOOR AIR MASS FLOW for EarthTube
    extern Array1D<Nandle> EAMFLxHumRat;               // OUTDOOR AIR MASS FLOW * Humidity Ratio for EarthTube (water vapor mass flow)
    extern Array1D<Nandle> MCPTE;                      // EARTHTUBE MASS FLOW * AIR CP * AIR TEMPERATURE
    extern Array1D<Nandle> MCPC;                       // COOLTOWER MASS FLOW * AIR SPECIFIC HEAT
    extern Array1D<Nandle> CTMFL;                      // OUTDOOR AIR MASS FLOW for cooltower
    extern Array1D<Nandle> MCPTC;                      // COOLTOWER MASS FLOW * AIR CP * AIR TEMPERATURE
    extern Array1D<Nandle> ThermChimAMFL;              // OUTDOOR AIR MASS FLOW for THERMALCHIMNEY
    extern Array1D<Nandle> MCPTThermChim;              // THERMALCHIMNEY MASS FLOW * AIR SPECIFIC HEAT
    extern Array1D<Nandle> MCPThermChim;               // THERMALCHIMNEY MASS FLOW * AIR CP * AIR TEMPERATURE
    extern Array1D<Nandle> ZoneLatentGain;             // Latent Energy from each Zone (People, equipment)
    extern Array1D<Nandle> ZoneLatentGainExceptPeople; // Added for hybrid model -- Latent Energy from each Zone (equipment)
    extern Array1D<Nandle> OAMFL;                      // OUTDOOR AIR MASS FLOW (kg/s) for infiltration
    extern Array1D<Nandle> VAMFL;                      // OUTDOOR AIR MASS FLOW (kg/s) for ventilation
    extern Array1D<Nandle> NonAirSystemResponse;       // Convective heat addition rate from non forced air
    // equipment such as baseboards plus heat from lights to
    extern Array1D<Nandle> SysDepZoneLoads; // Convective heat addition or subtraction rate from sources that
    // depend on what is happening with the HVAC system. Such as:
    // heat gain from lights to return air when return flow = 0; heat gain
    // from air flow windows to return air when return air flow = 0;
    // and heat removed by return air from refrigeration cases when
    // return air flow = 0.
    extern Array1D<Nandle> SysDepZoneLoadsLagged; // SysDepZoneLoads saved to be added to zone heat balance next
    // HVAC time step
    extern Array1D<Nandle>
        MDotCPOA; // Airbalance MASS FLOW * AIR SPECIFIC HEAT used at Air Balance Method = Quadrature in the ZoneAirBalance:OutdoorAir
    extern Array1D<Nandle> MDotOA; // Airbalance MASS FLOW rate used at Air Balance Method = Quadrature in the ZoneAirBalance:OutdoorAir

    extern Array1D<Nandle> MixingMassFlowZone;    // Mixing MASS FLOW (kg/s)
    extern Array1D<Nandle> MixingMassFlowXHumRat; // Mixing MASS FLOW * Humidity Ratio

    extern Array1D_bool ZoneMassBalanceFlag;  // zone mass flow balance flag
    extern Array1D_bool ZoneInfiltrationFlag; // Zone Infiltration flag
    extern Array1D_int ZoneReOrder;           // zone number reordered for zone mass balance

    // REAL Variables for the Heat Balance Simulation

    extern Array1D<Nandle> QRadSysSource;     // Current source/sink for a particular surface (radiant sys)
    extern Array1D<Nandle> TCondFDSourceNode; // Temperature of source/sink location in surface from CondFD algo
    extern Array1D<Nandle> QPVSysSource;      // Current source/sink for a surface (integrated PV sys)

    extern Array1D<Nandle> CTFTsrcConstPart; // Constant Outside Portion of the CTF calculation of
    // temperature at source
    extern Array1D<Nandle> CTFTuserConstPart; // Constant Outside Portion of the CTF calculation of
    // temperature at the user specified location
    extern Array1D<Nandle> QHTRadSysSurf; // Current radiant heat flux at a surface due to the presence
    // of high temperature radiant heaters
    extern Array1D<Nandle> QHWBaseboardSurf; // Current radiant heat flux at a surface due to the presence
    // of hot water baseboard heaters
    extern Array1D<Nandle> QSteamBaseboardSurf; // Current radiant heat flux at a surface due to the presence
    // of steam baseboard heaters
    extern Array1D<Nandle> QElecBaseboardSurf; // Current radiant heat flux at a surface due to the presence
    // of electric baseboard heaters
    extern Array1D<Nandle> QCoolingPanelSurf; // Current radiant heat flux at a surface due to the presence
    // of simple cooling panels
    extern Array1D<Nandle> QRadSurfAFNDuct;     // Current radiant heat flux at a surface due to radiation from AFN ducts
    extern Array1D<Nandle> QPoolSurfNumerator;  // Current pool heat flux impact at the surface (numerator of surface heat balance)
    extern Array1D<Nandle> PoolHeatTransCoefs;  // Current pool heat transfer coefficients (denominator of surface heat balance)
    extern Array1D<Nandle> RadSysTiHBConstCoef; // Inside heat balance coefficient that is constant
    extern Array1D<Nandle> RadSysTiHBToutCoef;  // Inside heat balance coefficient that modifies Toutside
    extern Array1D<Nandle> RadSysTiHBQsrcCoef;  // Inside heat balance coefficient that modifies source/sink
    extern Array1D<Nandle> RadSysToHBConstCoef; // Outside heat balance coefficient that is constant
    extern Array1D<Nandle> RadSysToHBTinCoef;   // Outside heat balance coefficient that modifies Toutside
    extern Array1D<Nandle> RadSysToHBQsrcCoef;  // Outside heat balance coefficient that modifies source/sink

    // Moisture variables to carry info from HB to the Zone Temp Predictor-Corrector for Fan System
    extern Array1D<Nandle> SumHmAW;   // SUM OF ZONE AREA*Moist CONVECTION COEFF*INSIDE Humidity Ratio
    extern Array1D<Nandle> SumHmARa;  // SUM OF ZONE AREA*Moist CONVECTION COEFF*Rho Air
    extern Array1D<Nandle> SumHmARaW; // SUM OF ZONE AREA*Moist CONVECTION COEFF*Rho Air* Inside Humidity Ration
    extern Array1D<Nandle> SumHmARaZ;

    extern Array1D<Nandle> TempZoneThermostatSetPoint;
    extern Array1D<Nandle> AdapComfortCoolingSetPoint;
    extern Array1D<Nandle> ZoneThermostatSetPointHi;
    extern Array1D<Nandle> ZoneThermostatSetPointLo;
    extern Array1D<Nandle> ZoneThermostatSetPointHiAver;
    extern Array1D<Nandle> ZoneThermostatSetPointLoAver;

    extern Array1D<Nandle> LoadCorrectionFactor; // PH 3/3/04

    extern Array1D<Nandle> AIRRAT; // "air power capacity"  PH 3/5/04
    extern Array1D<Nandle> ZTM1;   // zone air temperature at previous timestep
    extern Array1D<Nandle> ZTM2;   // zone air temperature at timestep T-2
    extern Array1D<Nandle> ZTM3;   // zone air temperature at previous T-3
    // Hybrid Modeling
    extern Array1D<Nandle> PreviousMeasuredZT1;     // Measured zone air temperature at previous timestep1
    extern Array1D<Nandle> PreviousMeasuredZT2;     // Measured zone air temperature at previous timestep2
    extern Array1D<Nandle> PreviousMeasuredZT3;     // Measured zone air temperature at previous timestep3
    extern Array1D<Nandle> PreviousMeasuredHumRat1; // Hybrid model zone humidity ratio at previous timestep
    extern Array1D<Nandle> PreviousMeasuredHumRat2; // Hybrid model zone humidity ratio at previous timestep
    extern Array1D<Nandle> PreviousMeasuredHumRat3; // Hybrid model zone humidity ratio at previous timestep
    // Exact and Euler solutions
    extern Array1D<Nandle> ZoneTMX; // TEMPORARY ZONE TEMPERATURE TO TEST CONVERGENCE in Exact and Euler method
    extern Array1D<Nandle> ZoneTM2; // TEMPORARY ZONE TEMPERATURE at timestep t-2 in Exact and Euler method
    extern Array1D<Nandle> ZoneT1;  // Zone temperature at the previous time step used in Exact and Euler method
    extern Array1D<Nandle> ZoneWMX; // TEMPORARY ZONE TEMPERATURE TO TEST CONVERGENCE in Exact and Euler method
    extern Array1D<Nandle> ZoneWM2; // TEMPORARY ZONE TEMPERATURE at timestep t-2 in Exact and Euler method
    extern Array1D<Nandle> ZoneW1;  // Zone temperature at the previous time step used in Exact and Euler method
    extern Array1D_int TempControlType;
    extern Array1D_int ComfortControlType;

    // Types

    struct ZoneComfortControlsFangerData
    {
        // Members
        int FangerType;      // Index for Fanger type
        Nandle LowPMV;       // Low PMV value
        Nandle HighPMV;      // High PMV Value
        int DualPMVErrCount; // Dual PMV setpoint error count
        int DualPMVErrIndex; // Dual PMV setpoint error index

        // Default Constructor
        ZoneComfortControlsFangerData() : FangerType(0), LowPMV(0.0), HighPMV(0.0), DualPMVErrCount(0), DualPMVErrIndex(0)
        {
        }
    };

    // Object Data
    extern Array1D<ZoneComfortControlsFangerData> ZoneComfortControlsFanger;

    void clear_state();

} // namespace DataHeatBalFanSys

} // namespace EnergyPlus

#endif
