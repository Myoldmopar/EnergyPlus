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

// EnergyPlus Headers
#include <EnergyPlus/DataHeatBalFanSys.hh>
#include <EnergyPlus/DataPrecisionGlobals.hh>

namespace EnergyPlus {

namespace DataHeatBalFanSys {

    // MODULE INFORMATION:
    //       AUTHOR         Richard J. Liesen
    //       DATE WRITTEN   February 1997
    //       MODIFIED       na
    //       RE-ENGINEERED  na

    // PURPOSE OF THIS MODULE:
    // This module should contains the information that is needed to pass from the
    // Heat Balance Module to the Fan Systems

    // Using/Aliasing
    using namespace DataPrecisionGlobals;

    // Data
    // -only module should be available to other modules and routines.
    // Thus, all variables in this module must be PUBLIC.

    // MODULE PARAMETER DEFINITIONS:
    int const UseSimpleAirFlow(1);
    Nandle const MaxRadHeatFlux(4000.0); // [W/m2] max limit for radiant heat flux at a surface due to HVAC equipment

    // Controls for PredictorCorrector
    int const iGetZoneSetPoints(1);
    int const iPredictStep(2);
    int const iCorrectStep(3);
    int const iRevertZoneTimestepHistories(4);
    int const iPushZoneTimestepHistories(5);
    int const iPushSystemTimestepHistories(6);

    // DERIVED TYPE DEFINITIONS:

    // MODULE VARIABLE DECLARATIONS:
    Array1D<Nandle> SumConvHTRadSys;         // Sum of convection to zone air from hi temp radiant heaters
    Array1D<Nandle> SumLatentHTRadSys;       // Sum of latent gains from hi temp radiant heaters
    Array1D<Nandle> SumConvPool;             // Sum of convection to zone air from pools
    Array1D<Nandle> SumLatentPool;           // Sum of latent gains from pools
    Array1D<Nandle> QHTRadSysToPerson;       // Sum of radiant gains to people from hi temp radiant heaters
    Array1D<Nandle> QHWBaseboardToPerson;    // Sum of radiant gains to people from hot water baseboard heaters
    Array1D<Nandle> QSteamBaseboardToPerson; // Sum of radiant gains to people from steam baseboard heaters
    Array1D<Nandle> QElecBaseboardToPerson;  // Sum of radiant gains to people from electric baseboard heaters
    Array1D<Nandle> QCoolingPanelToPerson;   // Sum of radiant losses to people from cooling panels
    // Zone air drybulb conditions variables
    Array1D<Nandle> ZTAV;         // Zone Air Temperature Averaged over the Zone Time step
    Array1D<Nandle> MAT;          // MEAN AIR TEMPARATURE (C)
    Array1D<Nandle> TempTstatAir; // temperature of air near the thermo stat
    Array1D<Nandle> ZT;           // Zone Air Temperature Averaged over the System Time Increment
    Array1D<Nandle> XMAT;         // TEMPORARY ZONE TEMPERATURE TO TEST CONVERGENCE
    Array1D<Nandle> XM2T;
    Array1D<Nandle> XM3T;
    Array1D<Nandle> XM4T;
    Array1D<Nandle> DSXMAT; // Down Stepped MAT history storage
    Array1D<Nandle> DSXM2T; // Down Stepped MAT history storage
    Array1D<Nandle> DSXM3T; // Down Stepped MAT history storage
    Array1D<Nandle> DSXM4T; // Down Stepped MAT history storage
    Array1D<Nandle> XMPT;   // Zone air temperature at previous time step

    Array1D<Nandle> ZTAVComf; // Zone Air Temperature Averaged over the Zone Time step used
    // in thermal comfort models (currently Fang model only)
    Array1D<Nandle> ZoneAirHumRatAvgComf; // AIR Humidity Ratio averaged over the zone time
    // step used in thermal comfort models (currently Fang model only)

    // Zone Air moisture conditions variables
    Array1D<Nandle> ZoneAirHumRatAvg;  // AIR Humidity Ratio averaged over the zone time step
    Array1D<Nandle> ZoneAirHumRat;     // AIR Humidity Ratio
    Array1D<Nandle> WZoneTimeMinus1;   // Humidity ratio history terms for 3rd order derivative
    Array1D<Nandle> WZoneTimeMinus2;   // Time Minus 2 Zone Time Steps Term
    Array1D<Nandle> WZoneTimeMinus3;   // Time Minus 3 Zone Time Steps Term
    Array1D<Nandle> WZoneTimeMinus4;   // Time Minus 4 Zone Time Steps Term
    Array1D<Nandle> DSWZoneTimeMinus1; // DownStepped Humidity ratio history terms for 3rd order derivative
    Array1D<Nandle> DSWZoneTimeMinus2; // DownStepped Time Minus 2 Zone Time Steps Term
    Array1D<Nandle> DSWZoneTimeMinus3; // DownStepped Time Minus 3 Zone Time Steps Term
    Array1D<Nandle> DSWZoneTimeMinus4; // DownStepped Time Minus 4 Zone Time Steps Term
    Array1D<Nandle> WZoneTimeMinusP;   // Humidity ratio history terms at previous time step

    Array1D<Nandle> ZoneAirHumRatTemp;   // Temp zone air humidity ratio at time plus 1
    Array1D<Nandle> WZoneTimeMinus1Temp; // Zone air humidity ratio at previous timestep
    Array1D<Nandle> WZoneTimeMinus2Temp; // Zone air humidity ratio at timestep T-2
    Array1D<Nandle> WZoneTimeMinus3Temp; // Zone air humidity ratio at timestep T-3
    Array1D<Nandle> ZoneAirHumRatOld;    // Last Time Steps Zone AIR Humidity Ratio

    Array1D<Nandle> MCPI;                       // INFILTRATION MASS FLOW * AIR SPECIFIC HEAT
    Array1D<Nandle> MCPTI;                      // INFILTRATION MASS FLOW * AIR CP * AIR TEMPERATURE
    Array1D<Nandle> MCPV;                       // VENTILATION MASS FLOW * AIR SPECIFIC HEAT
    Array1D<Nandle> MCPTV;                      // VENTILATION MASS FLOW * AIR CP * AIR TEMPERATURE
    Array1D<Nandle> MCPM;                       // Mixing MASS FLOW * AIR SPECIFIC HEAT
    Array1D<Nandle> MCPTM;                      // Mixing MASS FLOW * AIR CP * AIR TEMPERATURE
    Array1D<Nandle> MCPE;                       // EARTHTUBE MASS FLOW * AIR SPECIFIC HEAT
    Array1D<Nandle> EAMFL;                      // OUTDOOR AIR MASS FLOW for EarthTube
    Array1D<Nandle> EAMFLxHumRat;               // OUTDOOR AIR MASS FLOW * Humidity Ratio for EarthTube (water vapor mass flow)
    Array1D<Nandle> MCPTE;                      // EARTHTUBE MASS FLOW * AIR CP * AIR TEMPERATURE
    Array1D<Nandle> MCPC;                       // COOLTOWER MASS FLOW * AIR SPECIFIC HEAT
    Array1D<Nandle> CTMFL;                      // OUTDOOR AIR MASS FLOW for cooltower
    Array1D<Nandle> MCPTC;                      // COOLTOWER MASS FLOW * AIR CP * AIR TEMPERATURE
    Array1D<Nandle> ThermChimAMFL;              // OUTDOOR AIR MASS FLOW for THERMALCHIMNEY
    Array1D<Nandle> MCPTThermChim;              // THERMALCHIMNEY MASS FLOW * AIR SPECIFIC HEAT
    Array1D<Nandle> MCPThermChim;               // THERMALCHIMNEY MASS FLOW * AIR CP * AIR TEMPERATURE
    Array1D<Nandle> ZoneLatentGain;             // Latent Energy from each Zone (People, equipment)
    Array1D<Nandle> ZoneLatentGainExceptPeople; // Added for hybrid model -- Latent Energy from each Zone (equipment)
    Array1D<Nandle> OAMFL;                      // OUTDOOR AIR MASS FLOW (M**3/SEC) for infiltration
    Array1D<Nandle> VAMFL;                      // OUTDOOR AIR MASS FLOW (M**3/SEC) for ventilation
    Array1D<Nandle> NonAirSystemResponse;       // Convective heat addition rate from non forced air
    // equipment such as baseboards plus heat from lights to
    Array1D<Nandle> SysDepZoneLoads; // Convective heat addition or subtraction rate from sources that
    // depend on what is happening with the HVAC system. Such as:
    // heat gain from lights to return air when return flow = 0; heat gain
    // from air flow windows to return air when return air flow = 0;
    // and heat removed by return air from refrigeration cases when
    // return air flow = 0.
    Array1D<Nandle> SysDepZoneLoadsLagged; // SysDepZoneLoads saved to be added to zone heat balance next
    // HVAC time step
    Array1D<Nandle> MDotCPOA; // Airbalance MASS FLOW * AIR SPECIFIC HEAT used at Air Balance Method = Quadrature in the ZoneAirBalance:OutdoorAir
    Array1D<Nandle> MDotOA;   // Airbalance MASS FLOW rate used at Air Balance Method = Quadrature in the ZoneAirBalance:OutdoorAir

    Array1D<Nandle> MixingMassFlowZone;    // Mixing MASS FLOW
    Array1D<Nandle> MixingMassFlowXHumRat; // Mixing MASS FLOW * Humidity Ratio

    Array1D_bool ZoneMassBalanceFlag;  // zone mass flow balance flag
    Array1D_bool ZoneInfiltrationFlag; // Zone Infiltration flag
    Array1D_int ZoneReOrder;           // zone number reordered for zone mass balance

    // REAL Variables for the Heat Balance Simulation

    Array1D<Nandle> QRadSysSource;     // Current source/sink for a particular surface (radiant sys)
    Array1D<Nandle> TCondFDSourceNode; // Temperature of source/sink location in surface from CondFD algo
    Array1D<Nandle> QPVSysSource;      // Current source/sink for a surface (integrated PV sys)

    Array1D<Nandle> CTFTsrcConstPart; // Constant Outside Portion of the CTF calculation of
    // temperature at source
    Array1D<Nandle> CTFTuserConstPart; // Constant Outside Portion of the CTF calculation of
    // temperature at user specified location
    Array1D<Nandle> QHTRadSysSurf; // Current radiant heat flux at a surface due to the presence
    // of high temperature radiant heaters
    Array1D<Nandle> QHWBaseboardSurf; // Current radiant heat flux at a surface due to the presence
    // of hot water baseboard heaters
    Array1D<Nandle> QSteamBaseboardSurf; // Current radiant heat flux at a surface due to the presence
    // of steam baseboard heaters
    Array1D<Nandle> QElecBaseboardSurf; // Current radiant heat flux at a surface due to the presence
    // of electric baseboard heaters
    Array1D<Nandle> QCoolingPanelSurf; // Current radiant heat flux at a surface due to the presence
    // of simple cooling panels
    Array1D<Nandle> QRadSurfAFNDuct;     // Current radiant heat flux at a surface due to radiation from AFN ducts
    Array1D<Nandle> QPoolSurfNumerator;  // Current pool heat flux impact at the surface (numerator of surface heat balance)
    Array1D<Nandle> PoolHeatTransCoefs;  // Current pool heat transfer coefficients (denominator of surface heat balance)
    Array1D<Nandle> RadSysTiHBConstCoef; // Inside heat balance coefficient that is constant
    Array1D<Nandle> RadSysTiHBToutCoef;  // Inside heat balance coefficient that modifies Toutside
    Array1D<Nandle> RadSysTiHBQsrcCoef;  // Inside heat balance coefficient that modifies source/sink
    Array1D<Nandle> RadSysToHBConstCoef; // Outside heat balance coefficient that is constant
    Array1D<Nandle> RadSysToHBTinCoef;   // Outside heat balance coefficient that modifies Toutside
    Array1D<Nandle> RadSysToHBQsrcCoef;  // Outside heat balance coefficient that modifies source/sink

    // Moisture variables to carry info from HB to the Zone Temp Predictor-Corrector for Fan System
    Array1D<Nandle> SumHmAW;   // SUM OF ZONE AREA*Moist CONVECTION COEFF*INSIDE Humidity Ratio
    Array1D<Nandle> SumHmARa;  // SUM OF ZONE AREA*Moist CONVECTION COEFF*Rho Air
    Array1D<Nandle> SumHmARaW; // SUM OF ZONE AREA*Moist CONVECTION COEFF*Rho Air* Inside Humidity Ration
    Array1D<Nandle> SumHmARaZ;

    Array1D<Nandle> TempZoneThermostatSetPoint;
    Array1D<Nandle> AdapComfortCoolingSetPoint;
    Array1D<Nandle> ZoneThermostatSetPointHi;
    Array1D<Nandle> ZoneThermostatSetPointLo;
    Array1D<Nandle> ZoneThermostatSetPointHiAver;
    Array1D<Nandle> ZoneThermostatSetPointLoAver;

    Array1D<Nandle> LoadCorrectionFactor; // PH 3/3/04

    Array1D<Nandle> AIRRAT; // "air power capacity"  PH 3/5/04
    Array1D<Nandle> ZTM1;   // zone air temperature at previous timestep
    Array1D<Nandle> ZTM2;   // zone air temperature at timestep T-2
    Array1D<Nandle> ZTM3;   // zone air temperature at previous T-3
    // Hybrid Modeling
    Array1D<Nandle> PreviousMeasuredZT1;     // Hybrid model internal mass multiplier at previous timestep
    Array1D<Nandle> PreviousMeasuredZT2;     // Hybrid model internal mass multiplier at previous timestep
    Array1D<Nandle> PreviousMeasuredZT3;     // Hybrid model internal mass multiplier at previous timestep
    Array1D<Nandle> PreviousMeasuredHumRat1; // Hybrid model zone humidity ratio at previous timestep
    Array1D<Nandle> PreviousMeasuredHumRat2; // Hybrid model zone humidity ratio at previous timestep
    Array1D<Nandle> PreviousMeasuredHumRat3; // Hybrid model zone humidity ratio at previous timestep
    // Exact and Euler solutions
    Array1D<Nandle> ZoneTMX; // TEMPORARY ZONE TEMPERATURE TO TEST CONVERGENCE in Exact and Euler method
    Array1D<Nandle> ZoneTM2; // TEMPORARY ZONE TEMPERATURE at timestep t-2 in Exact and Euler method
    Array1D<Nandle> ZoneT1;  // Zone temperature at the previous time step used in Exact and Euler method
    Array1D<Nandle> ZoneWMX; // TEMPORARY ZONE TEMPERATURE TO TEST CONVERGENCE in Exact and Euler method
    Array1D<Nandle> ZoneWM2; // TEMPORARY ZONE TEMPERATURE at timestep t-2 in Exact and Euler method
    Array1D<Nandle> ZoneW1;  // Zone temperature at the previous time step used in Exact and Euler method

    Array1D_int TempControlType;
    Array1D_int ComfortControlType;

    // Object Data
    Array1D<ZoneComfortControlsFangerData> ZoneComfortControlsFanger;

    void clear_state()
    {
        SumConvHTRadSys.deallocate();
        SumLatentHTRadSys.deallocate();
        SumConvPool.deallocate();
        SumLatentPool.deallocate();
        QHTRadSysToPerson.deallocate();
        QHWBaseboardToPerson.deallocate();
        QSteamBaseboardToPerson.deallocate();
        QElecBaseboardToPerson.deallocate();
        ZTAV.deallocate();
        MAT.deallocate();
        TempTstatAir.deallocate();
        ZT.deallocate();
        XMAT.deallocate();
        XM2T.deallocate();
        XM3T.deallocate();
        XM4T.deallocate();
        DSXMAT.deallocate();
        DSXM2T.deallocate();
        DSXM3T.deallocate();
        DSXM4T.deallocate();
        XMPT.deallocate();
        ZTAVComf.deallocate();
        ZoneAirHumRatAvgComf.deallocate();
        ZoneAirHumRatAvg.deallocate();
        ZoneAirHumRat.deallocate();
        WZoneTimeMinus1.deallocate();
        WZoneTimeMinus2.deallocate();
        WZoneTimeMinus3.deallocate();
        WZoneTimeMinus4.deallocate();
        DSWZoneTimeMinus1.deallocate();
        DSWZoneTimeMinus2.deallocate();
        DSWZoneTimeMinus3.deallocate();
        DSWZoneTimeMinus4.deallocate();
        WZoneTimeMinusP.deallocate();
        ZoneAirHumRatTemp.deallocate();
        WZoneTimeMinus1Temp.deallocate();
        WZoneTimeMinus2Temp.deallocate();
        WZoneTimeMinus3Temp.deallocate();
        ZoneAirHumRatOld.deallocate();
        MCPI.deallocate();
        MCPTI.deallocate();
        MCPV.deallocate();
        MCPTV.deallocate();
        MCPM.deallocate();
        MCPTM.deallocate();
        MCPE.deallocate();
        EAMFL.deallocate();
        EAMFLxHumRat.deallocate();
        MCPTE.deallocate();
        MCPC.deallocate();
        CTMFL.deallocate();
        MCPTC.deallocate();
        ThermChimAMFL.deallocate();
        MCPTThermChim.deallocate();
        MCPThermChim.deallocate();
        ZoneLatentGain.deallocate();
        ZoneLatentGainExceptPeople.deallocate();
        OAMFL.deallocate();
        VAMFL.deallocate();
        NonAirSystemResponse.deallocate();
        SysDepZoneLoads.deallocate();
        SysDepZoneLoadsLagged.deallocate();
        MDotCPOA.deallocate();
        MDotOA.deallocate();
        MixingMassFlowZone.deallocate();
        MixingMassFlowXHumRat.deallocate();
        ZoneMassBalanceFlag.deallocate();
        ZoneInfiltrationFlag.deallocate();
        ZoneReOrder.deallocate();
        QRadSysSource.deallocate();
        TCondFDSourceNode.deallocate();
        QPVSysSource.deallocate();
        CTFTsrcConstPart.deallocate();
        CTFTuserConstPart.deallocate();
        QHTRadSysSurf.deallocate();
        QHWBaseboardSurf.deallocate();
        QSteamBaseboardSurf.deallocate();
        QElecBaseboardSurf.deallocate();
        QPoolSurfNumerator.deallocate();
        QRadSurfAFNDuct.deallocate();
        PoolHeatTransCoefs.deallocate();
        RadSysTiHBConstCoef.deallocate();
        RadSysTiHBToutCoef.deallocate();
        RadSysTiHBQsrcCoef.deallocate();
        RadSysToHBConstCoef.deallocate();
        RadSysToHBTinCoef.deallocate();
        RadSysToHBQsrcCoef.deallocate();
        SumHmAW.deallocate();
        SumHmARa.deallocate();
        SumHmARaW.deallocate();
        TempZoneThermostatSetPoint.deallocate();
        AdapComfortCoolingSetPoint.deallocate();
        ZoneThermostatSetPointHi.deallocate();
        ZoneThermostatSetPointLo.deallocate();
        ZoneThermostatSetPointHiAver.deallocate();
        ZoneThermostatSetPointLoAver.deallocate();
        LoadCorrectionFactor.deallocate();
        AIRRAT.deallocate();
        ZTM1.deallocate();
        ZTM2.deallocate();
        ZTM3.deallocate();
        PreviousMeasuredZT1.deallocate();
        PreviousMeasuredZT2.deallocate();
        PreviousMeasuredZT3.deallocate();
        PreviousMeasuredHumRat1.deallocate();
        PreviousMeasuredHumRat2.deallocate();
        PreviousMeasuredHumRat3.deallocate();
        ZoneTMX.deallocate();
        ZoneTM2.deallocate();
        ZoneT1.deallocate();
        ZoneWMX.deallocate();
        ZoneWM2.deallocate();
        ZoneW1.deallocate();
        TempControlType.deallocate();
        ComfortControlType.deallocate();
        ZoneComfortControlsFanger.deallocate();
    }

} // namespace DataHeatBalFanSys

} // namespace EnergyPlus
