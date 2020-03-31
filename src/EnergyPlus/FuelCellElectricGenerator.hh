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

#ifndef FuelCellElectricGenerator_hh_INCLUDED
#define FuelCellElectricGenerator_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>

// EnergyPlus Headers
#include <EnergyPlus/EnergyPlus.hh>
#include <EnergyPlus/PlantComponent.hh>

namespace EnergyPlus {

namespace FuelCellElectricGenerator {

    struct FCPowerModuleStruct
    {
        std::string Name;           // name of this PowerModule data
        int EffMode;                // mode for efficiency curves
        int EffCurveID;             // pointer to curve for efficiency
        Nandle NomEff;              // nominal efficiency
        Nandle NomPel;              // nominal power rate at rating point
        int NumCycles;              // number of start stop cycles
        Nandle CyclingDegradRat;    // rate of degradation from cycles
        Nandle NumRunHours;         // number of hours of operation
        Nandle OperateDegradRat;    // rate of degradation from run time (per hour)
        Nandle ThreshRunHours;      // number of hours before degradation starts
        Nandle UpTranLimit;         // power up transient limit
        Nandle DownTranLimit;       // power down tran limit
        Nandle StartUpTime;         // time for start up [hours]
        Nandle StartUpFuel;         // fuel use during start up
        Nandle StartUpElectConsum;  // electricity used during start up
        Nandle StartUpElectProd;    // electricity produced during start up
        Nandle ShutDownTime;        // time to shut down [hours]
        Nandle ShutDownFuel;        // fuel consumed during shut down
        Nandle ShutDownElectConsum; // Elect consumed during shut down
        Nandle ANC0;                // Ancilliary Loads constant term
        Nandle ANC1;                // Ancilliary Loads linear term
        int SkinLossMode;           // how are skin losses determined
        std::string ZoneName;
        int ZoneID; // "pointer" to zone with component in it
        Nandle RadiativeFract;
        Nandle QdotSkin;
        Nandle UAskin;
        int SkinLossCurveID;
        int WaterSupplyCurveID;            // pointer to curve for water use in reforming
        Nandle NdotDilutionAir;            // user defined constant flow of dilution air (kmol/sec)
        Nandle StackHeatLossToDilution;    // (watts)
        std::string DilutionInletNodeName; // dilution -> AirHR ?? added air heat recovery path
        int DilutionInletNode;             // pointer to node for inlet
        std::string DilutionExhaustNodeName;
        int DilutionExhaustNode; // pointer to node getting exhaust
        Nandle PelMin;           // minimum operating point for FCPM electrical power Pel
        Nandle PelMax;           // maximum operating point for FCPM electrical power Pel
        // Calculated values and input from elsewhere
        Nandle Pel; // current DC electrical power produced
        Nandle PelLastTimeStep;
        Nandle Eel;                         // power module efficiency
        Nandle QdotStackCool;               // Heat removed by stack cooler
        Nandle FractionalDayofLastStartUp;  // fractional days into simulation
        Nandle FractionalDayofLastShutDown; // fractional Days into simulations
        bool HasBeenOn;
        bool DuringShutDown;
        bool DuringStartUp;
        Nandle NdotFuel;           // molar fuel use rate.  (kmol/sec)
        Nandle TotFuelInEnthalphy; // Enthalpy of fuel coming into FCPM (watts)
        Nandle NdotProdGas;        // (kmol/sec)
        Array1D<Nandle> ConstitMolalFract;
        Array1D_int GasLibID; // lookup ID in Gas Phase ThermoChemistry Structure Array
        Nandle TprodGasLeavingFCPM;
        Nandle NdotAir;           // molar air use rate    (kmol/sec)
        Nandle TotAirInEnthalphy; // Enthalpy of air coming nto FCPM energy balance (watts)
        Nandle NdotLiqwater;      // molar water use rate (kmol/sec)
        Nandle TwaterInlet;
        Nandle WaterInEnthalpy;       // Enthalpy of liquid water used for reforming (watts)
        Nandle DilutionAirInEnthalpy; // Enthalpy of Dilution air coming into FCPM (watts)
        Nandle DilutionAirOutEnthalpy;
        Nandle PelancillariesAC;    // ancillary power (watts)
        Nandle TotProdGasEnthalphy; // Enthalphy of product gases leaving FCPM   (watts)
        Nandle WaterOutEnthalpy;    // enthalpy of vapor from water used for reforming
        int SeqSubstitIter;
        int RegulaFalsiIter;

        // Default Constructor
        FCPowerModuleStruct()
            : EffMode(0), EffCurveID(0), NomEff(0.0), NomPel(0.0), NumCycles(0), CyclingDegradRat(0.0), NumRunHours(0.0), OperateDegradRat(0.0),
              ThreshRunHours(0.0), UpTranLimit(0.0), DownTranLimit(0.0), StartUpTime(0.0), StartUpFuel(0.0), StartUpElectConsum(0.0),
              StartUpElectProd(0.0), ShutDownTime(0.0), ShutDownFuel(0.0), ShutDownElectConsum(0.0), ANC0(0.0), ANC1(0.0), SkinLossMode(0), ZoneID(0),
              RadiativeFract(0.0), QdotSkin(0.0), UAskin(0.0), SkinLossCurveID(0), WaterSupplyCurveID(0), NdotDilutionAir(0.0),
              StackHeatLossToDilution(0.0), DilutionInletNode(0), DilutionExhaustNode(0), PelMin(0.0), PelMax(0.0), Pel(0.0), PelLastTimeStep(0.0),
              Eel(0.0), QdotStackCool(0.0), FractionalDayofLastStartUp(0.0), FractionalDayofLastShutDown(0.0), HasBeenOn(true), DuringShutDown(false),
              DuringStartUp(false), NdotFuel(0.0), TotFuelInEnthalphy(0.0), NdotProdGas(0.0), ConstitMolalFract(14, 0.0), GasLibID(14, 0),
              TprodGasLeavingFCPM(0.0), NdotAir(0.0), TotAirInEnthalphy(0.0), NdotLiqwater(0.0), TwaterInlet(0.0), WaterInEnthalpy(0.0),
              DilutionAirInEnthalpy(0.0), DilutionAirOutEnthalpy(0.0), PelancillariesAC(0.0), TotProdGasEnthalphy(0.0), WaterOutEnthalpy(0.0),
              SeqSubstitIter(0), RegulaFalsiIter(0)
        {
        }
    };

    struct FCAirSupplyDataStruct
    {
        std::string Name;            // name of this
        std::string NodeName;        // Air supply node name
        int SupNodeNum;              // Air supply node ID
        int BlowerPowerCurveID;      // "pointer" to blower power quadratic
        Nandle BlowerHeatLossFactor; // alpha for blower heat loss fraction
        int AirSupRateMode;          // control for modeling method used to deterime supply air flow rate
        Nandle Stoics;               // excess air ratio
        int AirFuncPelCurveID;       // "pointer" to curve for air as function of power
        Nandle AirTempCoeff;         // coeff a3 in equ 16.
        int AirFuncNdotCurveID;      // "pointer" to curve for air as function of fuel flow rate
        int IntakeRecoveryMode;
        int ConstituentMode; // how are air data input
        int NumConstituents;
        Array1D_string ConstitName;
        Array1D<Nandle> ConstitMolalFract;
        // Calculated values and input from elsewhere
        Array1D_int GasLibID; // lookup ID in Gas Phase ThermoChemistry Structure Array
        Nandle O2fraction;
        Nandle TairIntoBlower;  // temperature entering blower
        Nandle TairIntoFCPM;    // temperature leaving blower and entering FCPM
        Nandle PairCompEl;      // power drawn by compressor
        Nandle QskinLoss;       // pumping losses for zone
        Nandle QintakeRecovery; // heat recovered on intake air by accessories

        // Default Constructor
        FCAirSupplyDataStruct()
            : SupNodeNum(0), BlowerPowerCurveID(0), BlowerHeatLossFactor(0.0), AirSupRateMode(0), Stoics(0.0), AirFuncPelCurveID(0),
              AirTempCoeff(0.0), AirFuncNdotCurveID(0), IntakeRecoveryMode(0), ConstituentMode(0), NumConstituents(0), ConstitName(14),
              ConstitMolalFract(14, 0.0), GasLibID(14, 0), O2fraction(0.0), TairIntoBlower(0.0), TairIntoFCPM(0.0), PairCompEl(0.0), QskinLoss(0.0),
              QintakeRecovery(0.0)
        {
        }
    };

    struct FCWaterSupplyDataStruct
    {
        std::string Name;          // name of this water supply module
        int WaterTempMode;         // temperature of water inlet determination
        std::string NodeName;      // node name for temperature at input
        int NodeNum;               // node number for temperature at input
        int SchedNum;              // water temperature at input
        int WaterSupRateCurveID;   // "pointer" to water flow rate curve as a function of fuel rate
        int PmpPowerCurveID;       // "pointer to Pump power curve as a function of water flow Rate
        Nandle PmpPowerLossFactor; // Pump heat loss factor
        // calculated data
        bool IsModeled;
        Nandle TwaterIntoCompress; // inlet Water Temperature
        Nandle TwaterIntoFCPM;     // pumped water temp
        Nandle PwaterCompEl;       // water pump power
        Nandle QskinLoss;          // pumping losses for zone

        // Default Constructor
        FCWaterSupplyDataStruct()
            : WaterTempMode(0), NodeNum(0), SchedNum(0), WaterSupRateCurveID(0), PmpPowerCurveID(0), PmpPowerLossFactor(0.0), IsModeled(true),
              TwaterIntoCompress(0.0), TwaterIntoFCPM(0.0), PwaterCompEl(0.0), QskinLoss(0.0)
        {
        }
    };

    struct FCAuxilHeatDataStruct
    {
        std::string Name; // name of this auxiliary heating module
        std::string ZoneName;
        int ZoneID;
        Nandle UASkin; // for skin losses to zone
        Nandle ExcessAirRAT;
        Nandle ANC0;
        Nandle ANC1;
        int SkinLossDestination; // control mode for where lost heat goes
        Nandle MaxPowerW;
        Nandle MinPowerW;
        Nandle MaxPowerkmolperSec;
        Nandle MinPowerkmolperSec;
        // calculated and from elsewhere
        int NumConstituents;
        Nandle TauxMix;
        Nandle NdotAuxMix;
        Array1D<Nandle> ConstitMolalFract;
        Array1D_int GasLibID; // lookup ID in Gas Phase ThermoChemistry Structure Array
        Nandle QskinLoss;     // Heat lost to room
        Nandle QairIntake;    // heat into intake air

        // Default Constructor
        FCAuxilHeatDataStruct()
            : ZoneID(0), UASkin(0.0), ExcessAirRAT(0.0), ANC0(0.0), ANC1(0.0), SkinLossDestination(0), MaxPowerW(0.0), MinPowerW(0.0),
              MaxPowerkmolperSec(0.0), MinPowerkmolperSec(0.0), NumConstituents(0), TauxMix(0.0), NdotAuxMix(0.0), ConstitMolalFract(14, 0.0),
              GasLibID(14, 0), QskinLoss(0.0), QairIntake(0.0)
        {
        }
    };

    struct FCExhaustHXDataStruct
    {
        std::string Name;                 // name of this exhaust gas heat recovery
        std::string WaterInNodeName;      // HR Water Inlet Node
        int WaterInNode;                  // HR Water Outlet Node ID
        std::string WaterOutNodeName;     // HR water outlet Node name
        int WaterOutNode;                 // HR Water outlet Node ID
        Nandle WaterVolumeFlowMax;        // HR water flow rate max avail
        std::string ExhaustOutNodeName;   // air node for exhaust flow
        int ExhaustOutNode;               // Exhaust Air node ID
        int HXmodelMode;                  // Heat Exchanger Calculation Method
        Nandle HXEffect;                  // Heat Exchanger Effectiveness (method 1)
        Nandle hxs0;                      // (method 2)
        Nandle hxs1;                      // (method 2)
        Nandle hxs2;                      // (method 2)
        Nandle hxs3;                      // (method 2)
        Nandle hxs4;                      // (method 2)
        Nandle h0gas;                     // (method 3)
        Nandle NdotGasRef;                // (method 3)
        Nandle nCoeff;                    // (method 3)
        Nandle AreaGas;                   // (method 3)
        Nandle h0Water;                   // (method 3)
        Nandle NdotWaterRef;              // (method 3)
        Nandle mCoeff;                    // (method 3)
        Nandle AreaWater;                 // (method 3)
        Nandle Fadjust;                   // (method 3)
        Nandle l1Coeff;                   // (method 4)
        Nandle l2Coeff;                   // (method 4)
        Nandle CondensationThresholdTemp; // (method 4) [degrees C]
        // calculated
        Nandle qHX;                     // heat flow from gas stream to water
        Nandle THXexh;                  // temperature of exhaust gases leaving heat exchanger.
        Nandle WaterMassFlowRateDesign; // Design level of water flow rate
        Nandle WaterMassFlowRate;       // water flow rate in plant loop
        Nandle WaterInletTemp;
        Nandle WaterVaporFractExh; // water vapor fraction in exhaust gas stream.
        Nandle CondensateRate;     // water condensation rate.
        Array1D<Nandle> ConstitMolalFract;
        Array1D_int GasLibID; // lookup ID in Gas Phase ThermoChemistry Structure Array
        Nandle NdotHXleaving;
        Nandle WaterOutletTemp;
        Nandle WaterOutletEnthalpy;

        // Default Constructor
        FCExhaustHXDataStruct()
            : WaterInNode(0), WaterOutNode(0), WaterVolumeFlowMax(0.0), ExhaustOutNode(0), HXmodelMode(0), HXEffect(0.0), hxs0(0.0), hxs1(0.0),
              hxs2(0.0), hxs3(0.0), hxs4(0.0), h0gas(0.0), NdotGasRef(0.0), nCoeff(0.0), AreaGas(0.0), h0Water(0.0), NdotWaterRef(0.0), mCoeff(0.0),
              AreaWater(0.0), Fadjust(0.0), l1Coeff(0.0), l2Coeff(0.0), CondensationThresholdTemp(0.0), qHX(0.0), THXexh(0.0),
              WaterMassFlowRateDesign(0.0), WaterMassFlowRate(0.0), WaterInletTemp(0.0), WaterVaporFractExh(0.0), CondensateRate(0.0),
              ConstitMolalFract(14, 0.0), GasLibID(14, 0), NdotHXleaving(0.0), WaterOutletTemp(0.0), WaterOutletEnthalpy(0.0)
        {
        }
    };

    struct BatteryDichargeDataStruct
    {
        std::string Name; // name of this battery data set
        Nandle NumInSeries;
        Nandle NumInParallel;
        Nandle NominalVoltage;
        Nandle LowVoltsDischarged; // not used
        int NumTablePairs;
        Array1D<Nandle> DischargeCurrent; // amps
        Array1D<Nandle> DischargeTime;    // hours
        // calculated variables
        Nandle k;    // parameter in Manwell McGowan model
        Nandle c;    // parameter in Manwell McGowan model
        Nandle qmax; // parameter in Manwell McGowan model

        // Default Constructor
        BatteryDichargeDataStruct()
            : NumInSeries(0.0), NumInParallel(0.0), NominalVoltage(0.0), LowVoltsDischarged(0.0), NumTablePairs(0), k(0.0), c(0.0), qmax(0.0)
        {
        }
    };

    struct FCElecStorageDataStruct
    {
        std::string Name; // name of this electrical storage module
        int StorageModelMode;
        Nandle StartingEnergyStored; // joules inside
        Nandle EnergeticEfficCharge; // for
        Nandle EnergeticEfficDischarge;
        Nandle MaxPowerDraw;  // for simple bucket method 0
        Nandle MaxPowerStore; // for simple bucket method 0
        Nandle NominalVoltage;
        Nandle NominalEnergyCapacity; // [J]
        // calculated and from elsewhere vars
        Nandle ThisTimeStepStateOfCharge; // [J]
        Nandle LastTimeStepStateOfCharge; // [J]
        Nandle PelNeedFromStorage;
        Nandle IdesiredDischargeCurrent;
        Nandle PelFromStorage; // power
        Nandle IfromStorage;   // current this timestepm
        Nandle PelIntoStorage;
        Nandle QairIntake; // heat into intake air
        // nested structures
        BatteryDichargeDataStruct Battery;

        // Default Constructor
        FCElecStorageDataStruct()
            : StorageModelMode(0), StartingEnergyStored(0.0), EnergeticEfficCharge(0.0), EnergeticEfficDischarge(0.0), MaxPowerDraw(0.0),
              MaxPowerStore(0.0), NominalVoltage(0.0), NominalEnergyCapacity(0.0), ThisTimeStepStateOfCharge(0.0), LastTimeStepStateOfCharge(0.0),
              PelNeedFromStorage(0.0), IdesiredDischargeCurrent(0.0), PelFromStorage(0.0), IfromStorage(0.0), PelIntoStorage(0.0), QairIntake(0.0)
        {
        }
    };

    struct FCInverterDataStruct
    {
        std::string Name; // name of this inverter
        int EffMode;      // efficiency calculation mode
        Nandle ConstEff;
        int EffQuadraticCurveID;
        // calculated and from elsewhere
        Nandle PCUlosses;
        Nandle QairIntake;

        // Default Constructor
        FCInverterDataStruct() : EffMode(0), ConstEff(0.0), EffQuadraticCurveID(0), PCUlosses(0.0), QairIntake(0.0)
        {
        }
    };

    struct FCReportDataStruct
    {
        // Members
        Nandle ACPowerGen;           // reporting: power (W)
        Nandle ACEnergyGen;          // reporting: energy (J)
        Nandle QdotExhaust;          // reporting: exhaust gas heat recovered (W)
        Nandle TotalHeatEnergyRec;   // reporting: total heat recovered (J)
        Nandle ExhaustEnergyRec;     // reporting: exhaust gas heat recovered (J)
        Nandle FuelEnergyLHV;        // reporting: Fuel Energy used in Lower Heating Value(J)
        Nandle FuelEnergyUseRateLHV; // reporting: Fuel Energy used in Lower Heating Value(W)
        Nandle FuelEnergyHHV;        // reporting: Fuel Energy used in Lower Heating Value(J)
        Nandle FuelEnergyUseRateHHV; // reporting: Fuel Energy used in Lower Heating Value(W)
        Nandle FuelRateMdot;         // (Kg/s)
        Nandle HeatRecInletTemp;     // reporting: Heat Recovery Loop Inlet Temperature (C)
        Nandle HeatRecOutletTemp;    // reporting: Heat Recovery Loop Outlet Temperature (C)
        Nandle HeatRecMdot;          // reporting: Heat Recovery Loop Mass flow rate (kg/s)
        // air supply and blower
        Nandle TairInlet;         // State point 1
        Nandle TairIntoFCPM;      // Temperature at State point 4
        Nandle NdotAir;           // air flow in kmol/sec
        Nandle TotAirInEnthalphy; // Enthalpy at State point 4
        Nandle BlowerPower;       // electrical power used by air supply blower
        Nandle BlowerEnergy;      // electrical energy used by air supply blower
        Nandle BlowerSkinLoss;    // heat rate of losses by blower
        // fuel supply and compressor
        Nandle TfuelInlet;           // State point 2 [C]
        Nandle TfuelIntoFCPM;        // state point 5 [C]
        Nandle NdotFuel;             // fuel flow in [kmol/sec]
        Nandle TotFuelInEnthalpy;    // state point 5 [W]
        Nandle FuelCompressPower;    // electrical power used by fuel supply compressor [W]
        Nandle FuelCompressEnergy;   // electrical energy used by fuel supply compressor [J]
        Nandle FuelCompressSkinLoss; // heat rate of losses.by fuel supply compressor [W]
        // reformer water supply
        Nandle TwaterInlet;           // State point 3
        Nandle TwaterIntoFCPM;        // State point 6
        Nandle NdotWater;             // water flow in kmol/sec (reformer water)
        Nandle WaterPumpPower;        // electrical power used by water pump [W]
        Nandle WaterPumpEnergy;       // electrical energy used by water pump [J]
        Nandle WaterIntoFCPMEnthalpy; // state point 6
        // product (exhaust) gas leaving power module
        Nandle TprodGas;      // State point 7 Product Gas temperature
        Nandle EnthalProdGas; // state point 7 product gas enthalpy
        Nandle NdotProdGas;   // point 7 flow rate [kmol/sec]
        Nandle NdotProdAr;    // argon flow rate at point 7
        Nandle NdotProdCO2;   // carbon dioxide flow rate at point 7
        Nandle NdotProdH2O;   // water vapor flow rate at point 7
        Nandle NdotProdN2;    // nitrogen flow rate at point 7
        Nandle NdotProdO2;    // oxygen flow rate at point 7
        // heat exchanger for water to exhaust heat recovery
        Nandle qHX;                // heat flow from gas stream to water [W]
        Nandle HXenergy;           // energy from gas stream to water [J]
        Nandle THXexh;             // temperature of exhaust gases leaving heat exchanger.
        Nandle WaterVaporFractExh; // water vapor fraction in exhaust gas stream
        // relative to water vapor entering HX  (NdotH2O/Ndoaux-mix)
        Nandle CondensateRate;     // water condensation rate [kmol/s]
        int SeqSubstIterations;    // number of iterations in SOFC loop
        int RegulaFalsiIterations; // number of iterations in Tproduct gas solving
        Nandle ACancillariesPower;
        Nandle ACancillariesEnergy;
        Nandle PCUlosses;            // power conditioning Unit losses
        Nandle DCPowerGen;           // Pel, Power module power level [W]
        Nandle DCPowerEff;           // Eel, power module efficiency []
        Nandle ElectEnergyinStorage; // State of charge in Electrical Storage [J]
        Nandle StoredPower;          // Power added to Electrical Storage [W]
        Nandle StoredEnergy;         // energy added to Electrical STorage [J]
        Nandle DrawnPower;           // Power drawn from Electrical STorage [W]
        Nandle DrawnEnergy;          // Energy drawn from Electrical STorage [J]
        Nandle SkinLossPower;        // heat loss to surrounding zone [W]
        Nandle SkinLossEnergy;       // heat loss to surround zone [J]
        Nandle SkinLossConvect;      // convective heat loss to zone [W]
        Nandle SkinLossRadiat;       // radiative heat loss to zone [W}
        Nandle ElectEfficiency;
        Nandle ThermalEfficiency;
        Nandle OverallEfficiency;
        Nandle ExergyEfficiency;

        // Default Constructor
        FCReportDataStruct()
            : ACPowerGen(0.0), ACEnergyGen(0.0), QdotExhaust(0.0), TotalHeatEnergyRec(0.0), ExhaustEnergyRec(0.0), FuelEnergyLHV(0.0),
              FuelEnergyUseRateLHV(0.0), FuelEnergyHHV(0.0), FuelEnergyUseRateHHV(0.0), FuelRateMdot(0.0), HeatRecInletTemp(0.0),
              HeatRecOutletTemp(0.0), HeatRecMdot(0.0), TairInlet(0.0), TairIntoFCPM(0.0), NdotAir(0.0), TotAirInEnthalphy(0.0), BlowerPower(0.0),
              BlowerEnergy(0.0), BlowerSkinLoss(0.0), TfuelInlet(0.0), TfuelIntoFCPM(0.0), NdotFuel(0.0), TotFuelInEnthalpy(0.0),
              FuelCompressPower(0.0), FuelCompressEnergy(0.0), FuelCompressSkinLoss(0.0), TwaterInlet(0.0), TwaterIntoFCPM(0.0), NdotWater(0.0),
              WaterPumpPower(0.0), WaterPumpEnergy(0.0), WaterIntoFCPMEnthalpy(0.0), TprodGas(0.0), EnthalProdGas(0.0), NdotProdGas(0.0),
              NdotProdAr(0.0), NdotProdCO2(0.0), NdotProdH2O(0.0), NdotProdN2(0.0), NdotProdO2(0.0), qHX(0.0), HXenergy(0.0), THXexh(0.0),
              WaterVaporFractExh(0.0), CondensateRate(0.0), SeqSubstIterations(0), RegulaFalsiIterations(0), ACancillariesPower(0.0),
              ACancillariesEnergy(0.0), PCUlosses(0.0), DCPowerGen(0.0), DCPowerEff(0.0), ElectEnergyinStorage(0.0), StoredPower(0.0),
              StoredEnergy(0.0), DrawnPower(0.0), DrawnEnergy(0.0), SkinLossPower(0.0), SkinLossEnergy(0.0), SkinLossConvect(0.0),
              SkinLossRadiat(0.0), ElectEfficiency(0.0), ThermalEfficiency(0.0), OverallEfficiency(0.0), ExergyEfficiency(0.0)
        {
        }
    };

    struct FCStackCoolerDataStruct
    {
        std::string Name;             // name of this stack cooler module
        std::string WaterInNodeName;  // HR Water Inlet Node
        int WaterInNode;              // HR Water Outlet Node ID
        std::string WaterOutNodeName; // HR water outlet Node name
        int WaterOutNode;             // HR Water outlet Node ID
        Nandle TstackNom;             // nominal fuel cell stack temperature
        Nandle TstackActual;          // actual fuel cell stack temperature
        Nandle r0;                    // stack cooling power coefficient r0
        Nandle r1;                    // stack cooling power coefficient r1
        Nandle r2;                    // stack cooling power coefficient r2
        Nandle r3;                    // stack cooling power coefficient r3
        Nandle MdotStackCoolant;      // stack coolant flow rate kg/s
        Nandle UAs_cool;              // stack heat transfer coef
        Nandle Fs_cogen;
        Nandle As_cogen;
        Nandle MdotCogenNom;
        Nandle hCogenNom;
        Nandle ns;
        Nandle PstackPumpEl;
        Nandle PmpPowerLossFactor;
        Nandle f0;
        Nandle f1;
        Nandle f2;
        // calculated and from elsewhere
        bool StackCoolerPresent; // control modeling
        Nandle qs_cool;
        Nandle qs_air;

        // Default Constructor
        FCStackCoolerDataStruct()
            : WaterInNode(0), WaterOutNode(0), TstackNom(0.0), TstackActual(0.0), r0(0.0), r1(0.0), r2(0.0), r3(0.0), MdotStackCoolant(0.0),
              UAs_cool(0.0), Fs_cogen(0.0), As_cogen(0.0), MdotCogenNom(0.0), hCogenNom(0.0), ns(0.0), PstackPumpEl(0.0), PmpPowerLossFactor(0.0),
              f0(0.0), f1(0.0), f2(0.0), StackCoolerPresent(false), qs_cool(0.0), qs_air(0.0)
        {
        }
    };

    struct FCDataStruct : PlantComponent
    {
        // Members
        // from input data and nested types for subsystems
        int TypeOf;
        std::string Name;                    // user identifier
        std::string NameFCPM;                // name of FC Power Module
        FCPowerModuleStruct FCPM;            // data for Power Module
        std::string NameFCAirSup;            // name of air supply module for fuel cell
        FCAirSupplyDataStruct AirSup;        // data for air supply module
        std::string NameFCFuelSup;           // name of fuel supply module
        int FuelSupNum;                      // index for fuel supply module structure
        std::string NameFCWaterSup;          // name of water supply module
        FCWaterSupplyDataStruct WaterSup;    // data for water supply module
        std::string NameFCAuxilHeat;         // name of auxiliary heating module
        FCAuxilHeatDataStruct AuxilHeat;     // data for auxiliary heating module
        std::string NameExhaustHX;           // name of Exhaust HX module
        FCExhaustHXDataStruct ExhaustHX;     // data for Exhaust heat exchanger module
        std::string NameElecStorage;         // name of Battery module
        FCElecStorageDataStruct ElecStorage; // data for Battery module
        std::string NameInverter;            // name of Inverter Module
        FCInverterDataStruct Inverter;       // data for Inverter module
        std::string NameStackCooler;         // name of Inverter Module
        FCStackCoolerDataStruct StackCooler; // data for Inverter module
        int CWLoopNum;                       // cooling water plant loop index number
        int CWLoopSideNum;                   // cooling water plant loop side index
        int CWBranchNum;                     // cooling water plant loop branch index
        int CWCompNum;                       // cooling water plant loop component index
        FCReportDataStruct Report;           // data for reporting as E+ output variables
        // calculated whole-system level variables
        Nandle ACPowerGen; // Net output from SOFC unit
        Nandle QconvZone;  // convective heat lost to surrounding zone
        Nandle QradZone;   // radiative heat lost to surrounding zone
        int DynamicsControlID;
        Nandle TimeElapsed; // used to track when timestep has changed
        bool MyEnvrnFlag_Init;
        bool MyWarmupFlag_Init;
        bool MyPlantScanFlag_Init;

        // Default Constructor
        FCDataStruct()
            : TypeOf(0), FuelSupNum(0), CWLoopNum(0), CWLoopSideNum(0), CWBranchNum(0), CWCompNum(0), ACPowerGen(0.0), QconvZone(0.0), QradZone(0.0),
              DynamicsControlID(0), TimeElapsed(0.0), MyEnvrnFlag_Init(true), MyWarmupFlag_Init(false), MyPlantScanFlag_Init(true)
        {
        }

        static PlantComponent *factory(std::string const &objectName);

        static PlantComponent *factory_exhaust(std::string const &objectName);

        void initialize();

        void getDesignCapacities(const PlantLocation &calledFromLocation, Nandle &MaxLoad, Nandle &MinLoad, Nandle &OptLoad) override;

        void setupOutputVars();

        void simulate(const PlantLocation &calledFromLocation, bool FirstHVACIteration, Nandle &CurLoad, bool RunFlag) override;

        void FigureAirHeatCap(Nandle FluidTemp, Nandle &Cp);

        void FigureAirEnthalpy(Nandle FluidTemp, Nandle &Hair);

        void FigureFuelHeatCap(Nandle FluidTemp, Nandle &Cp);

        void FigureFuelEnthalpy(Nandle FluidTemp, Nandle &Hfuel);

        void FigureProductGasesEnthalpy(Nandle FluidTemp, Nandle &HProdGases);

        void FigureProductGasHeatCap(Nandle FluidTemp, Nandle &Cp);

        void FigureAuxilHeatGasHeatCap(Nandle FluidTemp, Nandle &Cp);

        void FigureACAncillaries(Nandle &PacAncill);

        void FigurePowerConditioningLosses(Nandle Pdemand, Nandle &PpcuLosses);

        void FigureTransientConstraints(Nandle &Pel,       // DC power control setting for power module
                                        bool &Constrained, // true if transient constraints kick in (TODO: never used anywhere)
                                        Nandle &PelDiff    // if constrained then this is the difference, positive
        );

        Nandle FuelCellProductGasEnthResidual(Nandle TprodGas, Array1D<Nandle> const &Par);

        static void FigureGaseousWaterEnthalpy(Nandle FluidTemp, Nandle &HGasWater);

        static void FigureLiquidWaterEnthalpy(Nandle FluidTemp, Nandle &HLiqWater);

        static void FigureLiquidWaterHeatCap(Nandle FluidTemp, Nandle &Cp);

        void CalcFuelCellAuxHeater();

        void CalcFuelCellGenHeatRecovery();

        void CalcFuelCellGeneratorModel(bool RunFlag, Nandle MyLoad, bool FirstHVACIteration);

        void CalcUpdateHeatRecovery(bool FirstHVACIteration);

        void ManageElectStorInteractions(Nandle Pdemand,
                                         Nandle PpcuLosses,
                                         bool &Constrained, // TODO: This one is never used anywhere in the code
                                         Nandle &Pstorage,
                                         Nandle &PgridOverage // electricity that can't be stored and needs to go out
        );

        void SimFuelCellGenerator(bool RunFlag,  // simulate Generator when TRUE
                                  Nandle MyLoad, // demand on electric generator
                                  bool FirstHVACIteration);

        void UpdateFuelCellGeneratorRecords();
    };

    void clear_state();

    void getFuelCellInput();

    void FigureFuelCellZoneGains();

    extern bool getFuelCellInputFlag;
    extern int NumFuelCellGenerators;
    extern Array1D_bool CheckEquipName;
    extern Array1D<FCDataStruct> FuelCell;

} // namespace FuelCellElectricGenerator

} // namespace EnergyPlus

#endif
