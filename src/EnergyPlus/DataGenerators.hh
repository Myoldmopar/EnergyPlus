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

#ifndef DataGenerators_hh_INCLUDED
#define DataGenerators_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

namespace DataGenerators {

    // Using/Aliasing

    // Data
    // -only module should be available to other modules and routines.
    // Thus, all variables in this module must be PUBLIC.

    // MODULE PARAMETER DEFINITIONS:
    extern int const NormalizedCurveMode; // mode where efficiency curves are modifier curves
    extern int const DirectCurveMode;     // mode where efficiency curves are direct

    extern int const ConstantRateSkinLoss;  // fixed rate mode for skin losses
    extern int const UADTSkinLoss;          // UAdelta T mode for skin losses
    extern int const QuadraticFuelNdotSkin; // Quadratic function of fuel flow for skin losses

    extern int const QuadraticFuncofNdot;  // function of fuel rate mode for air flow
    extern int const ConstantStoicsAirRat; // Constant air ratio in stoics with fuel constituents
    extern int const QuadraticFuncofPel;   // function of electric power mode

    extern int const NoRecoveryOnAirIntake;  // mode for controlling intake air heat recovery
    extern int const RecoverBurnInvertBatt;  // mode for controlling intake air heat recovery
    extern int const RecoverAuxiliaryBurner; // mode for controlling intake air heat recovery
    extern int const RecoverInverterBatt;    // mode for controlling intake air heat recovery
    extern int const RecoverInverter;        // mode for controlling intake air heat recovery
    extern int const RecoverBattery;         // mode for controlling intake air heat recovery

    extern int const RegularAir;
    extern int const UserDefinedConstituents;

    extern int const FuelInTempFromNode;
    extern int const FuelInTempSchedule;

    extern int const WaterInReformMains;
    extern int const WaterInReformAirNode;
    extern int const WaterInReformWaterNode;
    extern int const WaterInReformSchedule;

    extern int const InverterEffConstant;
    extern int const InverterEffQuadratic;

    extern int const FixedEffectiveness;   // exhaust gas HX modeling mode
    extern int const LMTDempiricalUAeff;   // exhaust gas HX modeling mode
    extern int const LMTDfundementalUAeff; // exhaust gas HX modeling mode
    extern int const Condensing;           // exhaust gas HX modeling mode

    extern int const SimpleEffConstraints;         // electrical storage modeling mode
    extern int const LeadAcidBatterySaupe;         // electrical storage modeling mode
    extern int const LeadAcidBatterManwellMcGowan; // electrical storage modeling mode

    extern int const SurroundingZone;
    extern int const AirInletForFC;

    extern int const OpModeOff;      // CHP operating mode OFF
    extern int const OpModeStandby;  // CHP operating mode Stand By
    extern int const OpModeWarmUp;   // CHP operating mode Warm Up or start up
    extern int const OpModeNormal;   // CHP operating mode Normal
    extern int const OpModeCoolDown; // CHP operating mode Cool down or shut down

    extern int const fuelModeGaseousConstituents;
    extern int const fuelModeGenericLiquid;

    extern Nandle const MinProductGasTemp; // Minimum bound on search for product gas temps
    extern Nandle const MaxProductGasTemp; // Maximum bound on search for product gas temps

    extern int const NISTShomate;
    extern int const NASAPolynomial;

    extern Nandle const RinKJperMolpK; // R is ideal gas constant (kJ/mol-K)
    extern Nandle const InitHRTemp;    // Initialization temperature for heat recovery water

    extern Nandle const ImBalanceTol; // used as fraction of electrical power at power module

    extern int NumFuelConstit;
    extern int NumGeneratorFuelSups;
    extern int NumGensWDynamics;  // number of dynamics controls for generators

    struct GeneratorFuelSupplyDataStruct
    {
        // Members
        // user input data
        std::string Name;     // name of this fuel supply module
        int FuelTempMode;     // temperature of fuel node
        int FuelTypeMode;     // type of fuel, gasous or liquid
        std::string NodeName; // node name for temperature at input
        int NodeNum;          // node number for temperature at input
        int SchedNum;         // fuel temperature at input
        int CompPowerCurveID; // "pointer" to compressor power cubic curve
        Nandle CompPowerLossFactor;
        int NumConstituents; // number of constituents in fue supply
        Array1D_string ConstitName;
        Array1D<Nandle> ConstitMolalFract;
        // calculated data (except some for generic liquid)
        Array1D_int GasLibID;        // lookup ID in Gas Phase ThermoChemistry Structure Array
        Nandle LHV;                  // lower heating value of gaseous fuel (kJ/mol)
        Nandle LHVJperkg;            // lower heating value of gaseous fuel (J/kg)
        Nandle LHVliquid;            // userdefined lhv for generic liquid (J/kg)
        Nandle HHV;                  // higher heating value of fuel (J/kg)
        Nandle MW;                   // molecular weight g/mol
        Nandle eCO2;                 // mass flow based CO2 emmissions factor for complete combustion (-)
        Nandle KmolPerSecToKgPerSec; // conversion from moles to kilograms for this fuel. (
        Nandle StoicOxygenRate;
        Nandle TfuelIntoCompress; // inlet fuel temperature
        Nandle TfuelIntoFCPM;     // compressed fuel temp
        Nandle PfuelCompEl;       // fuel compressor power
        Nandle QskinLoss;         // pumping losses for zone
        Nandle CO2ProductGasCoef; // molar multiplier for stoic products of this fuel
        Nandle H2OProductGasCoef; // molar multiplier for stoic products of this fuel

        // Default Constructor
        GeneratorFuelSupplyDataStruct()
            : FuelTempMode(0), FuelTypeMode(0), NodeNum(0), SchedNum(0), CompPowerCurveID(0), CompPowerLossFactor(0.0), ConstitName(14),
              ConstitMolalFract(14, 0.0), GasLibID(14, 0), LHV(0.0), LHVJperkg(0.0), LHVliquid(0.0), HHV(0.0), MW(0.0), eCO2(0.0),
              KmolPerSecToKgPerSec(0.0), StoicOxygenRate(0.0), TfuelIntoCompress(0.0), TfuelIntoFCPM(0.0), PfuelCompEl(0.0), QskinLoss(0.0),
              CO2ProductGasCoef(0.0), H2OProductGasCoef(0.0)
        {
        }
    };

    struct GasPropertyDataStruct
    {
        // Members
        std::string ConstituentName;
        std::string ConstituentFormula;
        Nandle StdRefMolarEnthOfForm;
        int ThermoMode; // method of calculation for thermodynamics
        Nandle ShomateA;
        Nandle ShomateB;
        Nandle ShomateC;
        Nandle ShomateD;
        Nandle ShomateE;
        Nandle ShomateF;
        Nandle ShomateG;
        Nandle ShomateH;
        Nandle NumCarbons;
        Nandle NumHydrogens;
        Nandle NumOxygens;
        Nandle MolecularWeight;
        Nandle NASA_A1;
        Nandle NASA_A2;
        Nandle NASA_A3;
        Nandle NASA_A4;
        Nandle NASA_A5;
        Nandle NASA_A6;
        Nandle NASA_A7;

        // Default Constructor
        GasPropertyDataStruct()
            : StdRefMolarEnthOfForm(0.0), ThermoMode(0), ShomateA(0.0), ShomateB(0.0), ShomateC(0.0), ShomateD(0.0), ShomateE(0.0), ShomateF(0.0),
              ShomateG(0.0), ShomateH(0.0), NumCarbons(0.0), NumHydrogens(0.0), NumOxygens(0.0), MolecularWeight(0.0), NASA_A1(0.0), NASA_A2(0.0),
              NASA_A3(0.0), NASA_A4(0.0), NASA_A5(0.0), NASA_A6(0.0), NASA_A7(0.0)
        {
        }
    };

    struct GeneratorDynamicsManagerStruct
    {
        // Members
        // user input data
        std::string Name;
        Nandle PelMin;              // minimum operating point for electrical power Pel
        Nandle PelMax;              // maximum operating point for electrical power Pel
        Nandle UpTranLimit;         // power up transient limit W/s
        Nandle DownTranLimit;       // power down tran limit  W/s
        Nandle UpTranLimitFuel;     // fuel up transient limit kg/s
        Nandle DownTranLimitFuel;   // fuel down transient limit kg/s
        bool WarmUpByTimeDelay;     // Warm up mode control
        bool WarmUpByEngineTemp;    // Warm up mode control
        Nandle StartUpTimeDelay;    // time for start up [hours]
        Nandle WarmUpDelay;         // time for warm up delay [s]
        Nandle StartUpFuel;         // fuel use during start up
        Nandle StartUpElectConsum;  // electricity used during start up
        Nandle StartUpElectProd;    // electricity produced during start up
        Nandle ShutDownFuel;        // fuel consumed during shut down
        Nandle ShutDownElectConsum; // Elect consumed during shut down
        Nandle PcoolDown;           // power during cool down
        Nandle CoolDownDelay;       // time for cool down delay [hours]
        int NumCyclesInit;          // number of start stop cycles at beginning
        Nandle NumRunHoursInit;     // number of hours of operation beginning
        Nandle Pstandby;            // standby power [w]
        Nandle MCeng;               // aggregated thermal mass of engine [  ]
        Nandle MCcw;                // aggregated thermal mass of heat recovery [   ]
        Nandle kf;                  // coefficient k_f for warmup fuel flow rate
        Nandle TnomEngOp;           // nominal engine operating temperature [C]
        Nandle kp;                  // coefficient k_p for warmup power
        bool MandatoryFullCoolDown;
        bool WarmRestartOkay;
        int AvailabilitySchedID;
        // Calculated values and input from elsewhere
        int CurrentOpMode; // current operating mode, uses params like OpModeNormal
        int LastOpMode;
        Nandle FractionalDayofLastShutDown;
        Nandle FractionalDayofLastStartUp;
        bool HasBeenOn;
        bool DuringStartUp;
        bool DuringShutDown;
        Nandle FuelMdotLastTimestep;
        Nandle PelLastTimeStep;
        int NumCycles;
        Nandle PLRforSubtimestepStartUp;
        Nandle PLRforSubtimestepShutDown; // part load for not in shut down, shut down part is (1 - PLR)
        Nandle ElectEffNom;               // efficiency to use for control decisions
        Nandle ThermEffNom;               // thermal efficiency to use fo control decisions
        Nandle QdotHXMax;                 // Thermal power max
        Nandle QdotHXMin;                 // thermal power min
        Nandle QdotHXOpt;                 // thermal power nominal/optimal

        // Default Constructor
        GeneratorDynamicsManagerStruct()
            : PelMin(0.0), PelMax(0.0), UpTranLimit(0.0), DownTranLimit(0.0), UpTranLimitFuel(0.0), DownTranLimitFuel(0.0), WarmUpByTimeDelay(false),
              WarmUpByEngineTemp(true), StartUpTimeDelay(0.0), WarmUpDelay(0.0), StartUpFuel(0.0), StartUpElectConsum(0.0), StartUpElectProd(0.0),
              ShutDownFuel(0.0), ShutDownElectConsum(0.0), PcoolDown(0.0), CoolDownDelay(0.0), NumCyclesInit(0), NumRunHoursInit(0.0), Pstandby(0.0),
              MCeng(0.0), MCcw(0.0), kf(0.0), TnomEngOp(0.0), kp(0.0), MandatoryFullCoolDown(false), WarmRestartOkay(true), AvailabilitySchedID(0),
              CurrentOpMode(OpModeOff), LastOpMode(OpModeOff), FractionalDayofLastShutDown(0.0), FractionalDayofLastStartUp(0.0), HasBeenOn(false),
              DuringStartUp(false), DuringShutDown(false), FuelMdotLastTimestep(0.0), PelLastTimeStep(0.0), NumCycles(0),
              PLRforSubtimestepStartUp(0.0), PLRforSubtimestepShutDown(0.0), ElectEffNom(0.0), ThermEffNom(0.0), QdotHXMax(0.0), QdotHXMin(0.0),
              QdotHXOpt(0.0)
        {
        }
    };

    // Object Data
    extern Array1D<GasPropertyDataStruct> GasPhaseThermoChemistryData;
    extern Array1D<GeneratorFuelSupplyDataStruct> FuelSupply; // fuel supply (reused across various)
    extern Array1D<GeneratorDynamicsManagerStruct> GeneratorDynamics;

    void clear_state();

} // namespace DataGenerators

} // namespace EnergyPlus

#endif
