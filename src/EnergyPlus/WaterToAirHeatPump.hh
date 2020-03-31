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

#ifndef WaterToAirHeatPump_hh_INCLUDED
#define WaterToAirHeatPump_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

namespace WaterToAirHeatPump {

    // Using/Aliasing

    // Data
    // MODULE PARAMETER DEFINITIONS
    extern int const CompressorType_Reciprocating;
    extern int const CompressorType_Rotary;
    extern int const CompressorType_Scroll;

    // DERIVED TYPE DEFINITIONS

    // Output Variables Type definition

    // MODULE VARIABLE DECLARATIONS:
    extern int NumWatertoAirHPs; // The Number of Water to Air Heat Pumps found in the Input
    extern Array1D_bool CheckEquipName;

    extern int RefrigIndex;        // Refrigerant index
    extern int WaterIndex;         // Water index
    extern bool GetCoilsInputFlag; // Flag set to make sure you get input once
    // Subroutine Specifications for the Module
    // Driver/Manager Routines

    // Get Input routines for module

    // Initialization routines for module

    // Computational routines

    // Update routine to check convergence and update nodes

    // Utility routines

    // Types

    struct WatertoAirHPEquipConditions
    {
        // Members
        std::string Name;             // Name of the Water to Air Heat pump
        std::string WatertoAirHPType; // Type of WatertoAirHP ie. Heating or Cooling
        int WAHPPlantTypeOfNum;       // type of component in plant
        std::string Refrigerant;      // Refrigerant name
        bool SimFlag;
        Nandle InletAirMassFlowRate;    // Inlet Air Mass Flow through the Water to Air Heat Pump being Simulated [kg/s]
        Nandle OutletAirMassFlowRate;   // Outlet Air Mass Flow through the Water to Air Heat Pump being Simulated [kg/s]
        Nandle InletAirDBTemp;          // Inlet Air Dry Bulb Temperature [C]
        Nandle InletAirHumRat;          // Inlet Air Humidity Ratio [kg/kg]
        Nandle OutletAirDBTemp;         // Outlet Air Dry Bulb Temperature [C]
        Nandle OutletAirHumRat;         // Outlet Air Humidity Ratio [kg/kg]
        Nandle InletAirEnthalpy;        // Inlet Air Enthalpy [J/kg]
        Nandle OutletAirEnthalpy;       // Outlet Air Enthalpy [J/kg]
        Nandle InletWaterTemp;          // Inlet Water Temperature [C]
        Nandle OutletWaterTemp;         // Outlet Water Temperature [C]
        Nandle InletWaterMassFlowRate;  // Inlet Water Mass Flow Rate [kg/s]
        Nandle OutletWaterMassFlowRate; // Outlet Water Mass Flow Rate [kg/s]
        Nandle DesignWaterMassFlowRate; // Design Water Mass Flow Rate [kg/s]
        Nandle DesignWaterVolFlowRate;  // Design Water Volumetric Flow Rate [m3/s]
        Nandle InletWaterEnthalpy;      // Inlet Water Enthalpy [J/kg]
        Nandle OutletWaterEnthalpy;     // Outlet Water Enthalpy [J/kg]
        Nandle Power;                   // Power Consumption [W]
        Nandle Energy;                  // Energy Consumption [J]
        Nandle QSensible;               // Sensible Load Side Heat Transfer Rate [W]
        Nandle QLatent;                 // Latent Load Side Heat Transfer Rate [W]
        Nandle QSource;                 // Source Side Heat Transfer Rate [W]
        Nandle EnergySensible;          // Sensible Load Side Heat Transferred [J]
        Nandle EnergyLatent;            // Latent Load Side Heat Transferred [J]
        Nandle EnergySource;            // Source Side Heat Transferred [J]
        Nandle RunFrac;                 // Duty Factor
        Nandle PartLoadRatio;           // Part Load Ratio
        Nandle HeatingCapacity;         // Nominal Heating Capacity
        Nandle CoolingCapacity;         // Nominal Cooling Capacity
        Nandle QLoadTotal;              // Load Side Total Heat Transfer Rate [W]
        Nandle EnergyLoadTotal;         // Load Side Total Heat Transferred [J]
        Nandle Twet_Rated;              // Nominal Time for Condensate Removal to Begin [s]
        Nandle Gamma_Rated;             // Ratio of Initial Moisture Evaporation Rate and Steady-state Latent Capacity
        Nandle MaxONOFFCyclesperHour;   // Maximum cycling rate of heat pump [cycles/hr]
        Nandle HPTimeConstant;          // Heat pump time constant [s]
        Nandle FanDelayTime;            // Fan delay time, time delay for the HP's fan to
        // shut off after compressor cycle off [s]
        Nandle SourceSideUACoeff;      // Source Side Heat Transfer coefficient [W/C]
        Nandle LoadSideTotalUACoeff;   // Load Side Total Heat Transfer coefficient [W/C]
        Nandle LoadSideOutsideUACoeff; // Load Side Outside Heat Transfer coefficient [W/C]
        Nandle CompPistonDisp;         // Compressor Piston Displacement [m3/s]
        Nandle CompClearanceFactor;    // Compressor Clearance Factor
        Nandle CompSucPressDrop;       // Suction Pressure Drop [Pa]
        Nandle SuperheatTemp;          // Superheat Temperature [C]
        Nandle PowerLosses;            // Constant Part of the Compressor Power Losses [W]
        Nandle LossFactor;             // Compressor Power Loss Factor
        Nandle RefVolFlowRate;         // Refrigerant Volume Flow rate at the beginning
        // of the Compression [m3/s]
        Nandle VolumeRatio;   // Built-in-volume ratio [~]
        Nandle LeakRateCoeff; // Coefficient for the relationship between
        // Pressure Ratio and Leakage Rate [~]
        Nandle SourceSideHTR1;  // Source Side Heat Transfer Resistance coefficient 1 [~]
        Nandle SourceSideHTR2;  // Source Side Heat Transfer Resistance coefficient 2 [k/kW]
        Nandle HighPressCutoff; // High Pressure Cut-off [Pa]
        Nandle LowPressCutoff;  // Low Pressure Cut-off [Pa]
        int CompressorType;     // Type of Compressor ie. Reciprocating,Rotary or Scroll
        int AirInletNodeNum;    // air side coil inlet node number
        int AirOutletNodeNum;   // air side coil outlet node number
        int WaterInletNodeNum;  // water side coil inlet node number
        int WaterOutletNodeNum; // water side coil outlet node number
        int LowPressClgError;   // count for low pressure errors (cooling)
        int HighPressClgError;  // count for high pressure errors (cooling)
        int LowPressHtgError;   // count for low pressure errors (heating)
        int HighPressHtgError;  // count for high pressure errors (heating)
        int LoopNum;            // plant loop index for water side
        int LoopSide;           // plant loop side index
        int BranchNum;          // plant branch index
        int CompNum;            // plant component index

        // Default Constructor
        WatertoAirHPEquipConditions()
            : WAHPPlantTypeOfNum(0), SimFlag(false), InletAirMassFlowRate(0.0), OutletAirMassFlowRate(0.0), InletAirDBTemp(0.0), InletAirHumRat(0.0),
              OutletAirDBTemp(0.0), OutletAirHumRat(0.0), InletAirEnthalpy(0.0), OutletAirEnthalpy(0.0), InletWaterTemp(0.0), OutletWaterTemp(0.0),
              InletWaterMassFlowRate(0.0), OutletWaterMassFlowRate(0.0), DesignWaterMassFlowRate(0.0), DesignWaterVolFlowRate(0.0),
              InletWaterEnthalpy(0.0), OutletWaterEnthalpy(0.0), Power(0.0), Energy(0.0), QSensible(0.0), QLatent(0.0), QSource(0.0),
              EnergySensible(0.0), EnergyLatent(0.0), EnergySource(0.0), RunFrac(0.0), PartLoadRatio(0.0), HeatingCapacity(0.0), CoolingCapacity(0.0),
              QLoadTotal(0.0), EnergyLoadTotal(0.0), Twet_Rated(0.0), Gamma_Rated(0.0), MaxONOFFCyclesperHour(0.0), HPTimeConstant(0.0),
              FanDelayTime(0.0), SourceSideUACoeff(0.0), LoadSideTotalUACoeff(0.0), LoadSideOutsideUACoeff(0.0), CompPistonDisp(0.0),
              CompClearanceFactor(0.0), CompSucPressDrop(0.0), SuperheatTemp(0.0), PowerLosses(0.0), LossFactor(0.0), RefVolFlowRate(0.0),
              VolumeRatio(0.0), LeakRateCoeff(0.0), SourceSideHTR1(0.0), SourceSideHTR2(0.0), HighPressCutoff(0.0), LowPressCutoff(0.0),
              CompressorType(0), AirInletNodeNum(0), AirOutletNodeNum(0), WaterInletNodeNum(0), WaterOutletNodeNum(0), LowPressClgError(0),
              HighPressClgError(0), LowPressHtgError(0), HighPressHtgError(0), LoopNum(0), LoopSide(0), BranchNum(0), CompNum(0)
        {
        }
    };

    // Object Data
    extern Array1D<WatertoAirHPEquipConditions> WatertoAirHP;

    // Functions

    void SimWatertoAirHP(std::string const &CompName,   // component name
                         int &CompIndex,                // Index for Component name
                         Nandle const DesignAirflow,    // design air flow rate
                         int const CyclingScheme,       // cycling scheme--either continuous fan/cycling compressor or
                         bool const FirstHVACIteration, // first iteration flag
                         Nandle const RuntimeFrac,      // compressor run time fraction
                         Nandle &MaxONOFFCyclesperHour, // Maximum cycling rate of heat pump [cycles/hr]
                         Nandle &HPTimeConstant,        // Heat pump time constant [s]
                         Nandle &FanDelayTime,          // Fan delay time, time delay for the HP's fan to
                         bool const InitFlag,           // initialization flag used to suppress property routine errors
                         Nandle const SensLoad,         // sensible load
                         Nandle const LatentLoad,       // latent load
                         int const CompOp,
                         Nandle const PartLoadRatio);

    // Get Input Section of the Module
    //******************************************************************************

    void GetWatertoAirHPInput();

    // End of Get Input subroutines for the HB Module
    //******************************************************************************

    // Beginning Initialization Section of the Module
    //******************************************************************************

    void InitWatertoAirHP(int const HPNum, // index to main heat pump data structure
                          bool const InitFlag,
                          Nandle const MaxONOFFCyclesperHour, // Maximum cycling rate of heat pump [cycles/hr]
                          Nandle const HPTimeConstant,        // Heat pump time constant [s]
                          Nandle const FanDelayTime,          // Fan delay time, time delay for the HP's fan to
                          Nandle const SensLoad,
                          Nandle const LatentLoad,
                          Nandle const DesignAirFlow,
                          Nandle const PartLoadRatio);

    // End Initialization Section of the Module
    //******************************************************************************

    // Begin Algorithm Section of the Module
    //******************************************************************************

    void CalcWatertoAirHPCooling(int const HPNum,               // heat pump number
                                 int const CyclingScheme,       // fan/compressor cycling scheme indicator
                                 bool const FirstHVACIteration, // first iteration flag
                                 Nandle const RuntimeFrac,
                                 bool const InitFlag, // suppress property errors if true
                                 Nandle const SensDemand,
                                 int const CompOp,
                                 Nandle const PartLoadRatio);

    Nandle CalcCompSuctionTempResidual(Nandle const CompSuctionTemp, // HP compressor suction temperature (C)
                                       Array1D<Nandle> const &Par    // Function parameters
    );

    void CalcWatertoAirHPHeating(int const HPNum,               // heat pump number
                                 int const CyclingScheme,       // fan/compressor cycling scheme indicator
                                 bool const FirstHVACIteration, // first iteration flag
                                 Nandle const RuntimeFrac,
                                 bool const InitFlag, // first iteration flag
                                 Nandle const SensDemand,
                                 int const CompOp,
                                 Nandle const PartLoadRatio);

    // End Algorithm Section of the Module
    // *****************************************************************************

    // End Algorithm Section of the Module
    // *****************************************************************************

    // Beginning of Update subroutines for the WatertoAirHP Module
    // *****************************************************************************

    void UpdateWatertoAirHP(int const HPNum);

    //        End of Update subroutines for the WatertoAirHP Module
    // *****************************************************************************

    Nandle CalcEffectiveSHR(int const HPNum,         // Index number for cooling coil
                            Nandle const SHRss,      // Steady-state sensible heat ratio
                            int const CyclingScheme, // fan/compressor cycling scheme indicator
                            Nandle const RTF,        // Compressor run-time fraction
                            Nandle const QLatRated,  // Rated latent capacity
                            Nandle const QLatActual, // Actual latent capacity
                            Nandle const EnteringDB, // Entering air dry-bulb temperature
                            Nandle const EnteringWB  // Entering air wet-bulb temperature
    );

    Nandle DegradF(std::string &FluidName, // Name of glycol used in source side
                   Nandle &Temp,           // Temperature of the fluid
                   int &FluidIndex         // Index number for the fluid
    );

    int GetCoilIndex(std::string const &CoilType, // must match coil types in this module
                     std::string const &CoilName, // must match coil names for the coil type
                     bool &ErrorsFound            // set to true if problem
    );

    Nandle GetCoilCapacity(std::string const &CoilType, // must match coil types in this module
                           std::string const &CoilName, // must match coil names for the coil type
                           bool &ErrorsFound            // set to true if problem
    );

    int GetCoilInletNode(std::string const &CoilType, // must match coil types in this module
                         std::string const &CoilName, // must match coil names for the coil type
                         bool &ErrorsFound            // set to true if problem
    );

    int GetCoilOutletNode(std::string const &CoilType, // must match coil types in this module
                          std::string const &CoilName, // must match coil names for the coil type
                          bool &ErrorsFound            // set to true if problem
    );

} // namespace WaterToAirHeatPump

} // namespace EnergyPlus

#endif
