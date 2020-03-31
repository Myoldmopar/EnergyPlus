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

#ifndef ChillerGasAbsorption_hh_INCLUDED
#define ChillerGasAbsorption_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>
#include <EnergyPlus/Plant/PlantLocation.hh>
#include <EnergyPlus/PlantComponent.hh>

namespace EnergyPlus {

namespace ChillerGasAbsorption {

    struct GasAbsorberSpecs : PlantComponent
    {
        // Members
        // Parts of Type that do not correspond with IDD definition
        bool Available; // need an array of logicals--load identifiers of available equipment
        bool ON;        // simulate the machine at it's operating part load ratio
        bool InCoolingMode;
        bool InHeatingMode;
        // Part of Type that directly corresponds with IDD definition
        std::string Name;                 // user identifier
        std::string FuelType;             // Type of Fuel - DIESEL, GASOLINE, GAS
        Nandle NomCoolingCap;             // W - design nominal capacity of Absorber
        bool NomCoolingCapWasAutoSized;   // true if nominal capacity was autosize on input
        Nandle NomHeatCoolRatio;          // ratio of heating to cooling capacity
        Nandle FuelCoolRatio;             // ratio of fuel input to cooling output
        Nandle FuelHeatRatio;             // ratio of fuel input to heating output
        Nandle ElecCoolRatio;             // ratio of electricity input to cooling output
        Nandle ElecHeatRatio;             // ratio of electricity input to heating output
        int ChillReturnNodeNum;           // Node number on the inlet side of the plant
        int ChillSupplyNodeNum;           // Node number on the outlet side of the plant
        bool ChillSetPointErrDone;        // flag to report missing setpoint on CW outlet
        bool ChillSetPointSetToLoop;      // flag to use overall loop setpoint
        int CondReturnNodeNum;            // Node number on the inlet side of the condenser
        int CondSupplyNodeNum;            // Node number on the outlet side of the condenser
        int HeatReturnNodeNum;            // absorber steam inlet node number, water side
        int HeatSupplyNodeNum;            // absorber steam outlet node number, water side
        bool HeatSetPointErrDone;         // flag to report missing setpoint on HW outlet
        bool HeatSetPointSetToLoop;       // flag to use overall loop setpoint
        Nandle MinPartLoadRat;            // min allowed operating frac full load
        Nandle MaxPartLoadRat;            // max allowed operating frac full load
        Nandle OptPartLoadRat;            // optimal operating frac full load
        Nandle TempDesCondReturn;         // design secondary loop fluid temperature at the Absorber condenser side inlet
        Nandle TempDesCHWSupply;          // design chilled water supply temperature
        Nandle EvapVolFlowRate;           // m**3/s - design nominal water volumetric flow rate through the evaporator
        bool EvapVolFlowRateWasAutoSized; // true if evaporator flow rate was autosize on input
        Nandle CondVolFlowRate;           // m**3/s - design nominal water volumetric flow rate through the condenser
        bool CondVolFlowRateWasAutoSized; // true if condenser flow rate was autosize on input
        Nandle HeatVolFlowRate;           // m**3/s - design nominal water volumetric flow rate through the heater side
        bool HeatVolFlowRateWasAutoSized; // true if hot water flow rate was autosize on input
        Nandle SizFac;                    // sizing factor
        int CoolCapFTCurve;               // cooling capacity as a function of temperature curve (chilled water temp,
        // condenser water temp)
        int FuelCoolFTCurve; // Fuel-Input-to cooling output Ratio Function of Temperature Curve (chilled
        // water temp, condenser water temp)
        int FuelCoolFPLRCurve; // Fuel-Input-to cooling output Ratio Function of Part Load Ratio Curve
        int ElecCoolFTCurve;   // Electric-Input-to cooling output Ratio Function of Temperature Curve
        // (chilled water temp, condenser water temp)
        int ElecCoolFPLRCurve;   // Electric-Input-to cooling output Ratio Function of Part Load Ratio Curve
        int HeatCapFCoolCurve;   // Heating Capacity Function of Cooling Capacity Curve
        int FuelHeatFHPLRCurve;  // Fuel Input to heat output ratio during heating only function
        bool isEnterCondensTemp; // if using entering conderser water temperature is TRUE, exiting is FALSE
        bool isWaterCooled;      // if water cooled it is TRUE
        Nandle CHWLowLimitTemp;  // Chilled Water Lower Limit Temperature
        Nandle FuelHeatingValue;
        // Calculated design values
        Nandle DesCondMassFlowRate; // design nominal mass flow rate of water through the condenser [kg/s]
        Nandle DesHeatMassFlowRate; // design nominal mass flow rate of water through the hot water side [kg/s]
        Nandle DesEvapMassFlowRate; // design nominal mass flow rate of water through chilled water side [kg/s]
        // other values used during simulation
        int DeltaTempCoolErrCount; // error count for Delta Temp = 0 while cooling
        int DeltaTempHeatErrCount; // error count for Delta Temp = 0 while heating
        int CondErrCount;          // error count for poor Condenser Supply Estimate
        bool PossibleSubcooling;   // Flag to determine whether plant is overcooled
        // loop topology variables
        int CWLoopNum;     // chilled water plant loop index number
        int CWLoopSideNum; // chilled water plant loop side index
        int CWBranchNum;   // chilled water plant loop branch index
        int CWCompNum;     // chilled water plant loop component index
        int CDLoopNum;     // condenser water plant loop index number
        int CDLoopSideNum; // condenser water plant loop side index
        int CDBranchNum;   // condenser water plant loop branch index
        int CDCompNum;     // condenser water plant loop component index
        int HWLoopNum;     // hot water plant loop side index
        int HWLoopSideNum; // hot water plant loop side index
        int HWBranchNum;   // hot water plant loop branch index
        int HWCompNum;     // hot water plant loop component index
        bool envrnFlag;
        bool plantScanFlag;
        bool oneTimeFlag;
        Nandle oldCondSupplyTemp; // save the last iteration value of leaving condenser water temperature

        // Originally on report variable structure
        Nandle CoolingLoad;             // cooling load on the chiller (previously called QEvap)
        Nandle CoolingEnergy;           // variable to track total cooling load for period (was EvapEnergy)
        Nandle HeatingLoad;             // heating load on the chiller
        Nandle HeatingEnergy;           // heating energy
        Nandle TowerLoad;               // load on the cooling tower/condenser (previously called QCond)
        Nandle TowerEnergy;             // variable to track total tower load for a period (was CondEnergy)
        Nandle FuelUseRate;             // instantaneous use of gas for period
        Nandle FuelEnergy;              // variable to track total fuel used for a period
        Nandle CoolFuelUseRate;         // instantaneous use of gas for period for cooling
        Nandle CoolFuelEnergy;          // variable to track total fuel used for a period for cooling
        Nandle HeatFuelUseRate;         // instantaneous use of gas for period for heating
        Nandle HeatFuelEnergy;          // variable to track total fuel used for a period for heating
        Nandle ElectricPower;           // parasitic electric power used (was PumpingPower)
        Nandle ElectricEnergy;          // track the total electricity used for a period (was PumpingEnergy)
        Nandle CoolElectricPower;       // parasitic electric power used  for cooling
        Nandle CoolElectricEnergy;      // track the total electricity used for a period for cooling
        Nandle HeatElectricPower;       // parasitic electric power used  for heating
        Nandle HeatElectricEnergy;      // track the total electricity used for a period for heating
        Nandle ChillReturnTemp;         // reporting: evaporator inlet temperature (was EvapInletTemp)
        Nandle ChillSupplyTemp;         // reporting: evaporator outlet temperature (was EvapOutletTemp)
        Nandle ChillWaterFlowRate;      // reporting: evaporator mass flow rate (was Evapmdot)
        Nandle CondReturnTemp;          // reporting: condenser inlet temperature (was CondInletTemp)
        Nandle CondSupplyTemp;          // reporting: condenser outlet temperature (was CondOutletTemp)
        Nandle CondWaterFlowRate;       // reporting: condenser mass flow rate (was Condmdot)
        Nandle HotWaterReturnTemp;      // reporting: hot water return (inlet) temperature
        Nandle HotWaterSupplyTemp;      // reporting: hot water supply (outlet) temperature
        Nandle HotWaterFlowRate;        // reporting: hot water mass flow rate
        Nandle CoolPartLoadRatio;       // operating part load ratio (load/capacity for cooling)
        Nandle HeatPartLoadRatio;       // operating part load ratio (load/capacity for heating)
        Nandle CoolingCapacity;         // current capacity after temperature adjustment
        Nandle HeatingCapacity;         // current heating capacity
        Nandle FractionOfPeriodRunning; // fraction of the time period that the unit is operating
        Nandle FuelCOP;                 // reporting: cooling output/fuel input = CoolingLoad/CoolFuelUseRate

        // Default Constructor
        GasAbsorberSpecs()
            : Available(false), ON(false), InCoolingMode(false), InHeatingMode(false), NomCoolingCap(0.0), NomCoolingCapWasAutoSized(false),
              NomHeatCoolRatio(0.0), FuelCoolRatio(0.0), FuelHeatRatio(0.0), ElecCoolRatio(0.0), ElecHeatRatio(0.0), ChillReturnNodeNum(0),
              ChillSupplyNodeNum(0), ChillSetPointErrDone(false), ChillSetPointSetToLoop(false), CondReturnNodeNum(0), CondSupplyNodeNum(0),
              HeatReturnNodeNum(0), HeatSupplyNodeNum(0), HeatSetPointErrDone(false), HeatSetPointSetToLoop(false), MinPartLoadRat(0.0),
              MaxPartLoadRat(0.0), OptPartLoadRat(0.0), TempDesCondReturn(0.0), TempDesCHWSupply(0.0), EvapVolFlowRate(0.0),
              EvapVolFlowRateWasAutoSized(false), CondVolFlowRate(0.0), CondVolFlowRateWasAutoSized(false), HeatVolFlowRate(0.0),
              HeatVolFlowRateWasAutoSized(false), SizFac(0.0), CoolCapFTCurve(0), FuelCoolFTCurve(0), FuelCoolFPLRCurve(0), ElecCoolFTCurve(0),
              ElecCoolFPLRCurve(0), HeatCapFCoolCurve(0), FuelHeatFHPLRCurve(0), isEnterCondensTemp(false), isWaterCooled(false),
              CHWLowLimitTemp(0.0), FuelHeatingValue(0.0), DesCondMassFlowRate(0.0), DesHeatMassFlowRate(0.0), DesEvapMassFlowRate(0.0),
              DeltaTempCoolErrCount(0), DeltaTempHeatErrCount(0), CondErrCount(0), PossibleSubcooling(false), CWLoopNum(0), CWLoopSideNum(0),
              CWBranchNum(0), CWCompNum(0), CDLoopNum(0), CDLoopSideNum(0), CDBranchNum(0), CDCompNum(0), HWLoopNum(0), HWLoopSideNum(0),
              HWBranchNum(0), HWCompNum(0), envrnFlag(true), plantScanFlag(true), oneTimeFlag(true), oldCondSupplyTemp(0.0), CoolingLoad(0.0),
              CoolingEnergy(0.0), HeatingLoad(0.0), HeatingEnergy(0.0), TowerLoad(0.0), TowerEnergy(0.0), FuelUseRate(0.0), FuelEnergy(0.0),
              CoolFuelUseRate(0.0), CoolFuelEnergy(0.0), HeatFuelUseRate(0.0), HeatFuelEnergy(0.0), ElectricPower(0.0), ElectricEnergy(0.0),
              CoolElectricPower(0.0), CoolElectricEnergy(0.0), HeatElectricPower(0.0), HeatElectricEnergy(0.0), ChillReturnTemp(0.0),
              ChillSupplyTemp(0.0), ChillWaterFlowRate(0.0), CondReturnTemp(0.0), CondSupplyTemp(0.0), CondWaterFlowRate(0.0),
              HotWaterReturnTemp(0.0), HotWaterSupplyTemp(0.0), HotWaterFlowRate(0.0), CoolPartLoadRatio(0.0), HeatPartLoadRatio(0.0),
              CoolingCapacity(0.0), HeatingCapacity(0.0), FractionOfPeriodRunning(0.0), FuelCOP(0.0)
        {
        }

        static PlantComponent *factory(std::string const &objectName);

        void simulate(const PlantLocation &calledFromLocation, bool FirstHVACIteration, Nandle &CurLoad, bool RunFlag) override;

        void getDesignCapacities(const PlantLocation &calledFromLocation, Nandle &MaxLoad, Nandle &MinLoad, Nandle &OptLoad) override;

        void getSizingFactor(Nandle &SizFac) override;

        void onInitLoopEquip(const PlantLocation &calledFromLocation) override;

        void getDesignTemperatures(Nandle &TempDesCondIn, Nandle &TempDesEvapOut) override;

        void initialize();

        void setupOutputVariables();

        void size();

        void calculateChiller(Nandle &MyLoad);

        void calculateHeater(Nandle &MyLoad, bool RunFlag);

        void updateCoolRecords(Nandle MyLoad, // current load
                               bool RunFlag   // TRUE if Absorber operating
        );

        void updateHeatRecords(Nandle MyLoad, // current load
                               bool RunFlag   // TRUE if Absorber operating
        );
    };

    // Object Data
    extern Array1D<GasAbsorberSpecs> GasAbsorber; // dimension to number of machines

    void GetGasAbsorberInput();

    void clear_state();

} // namespace ChillerGasAbsorption

} // namespace EnergyPlus

#endif
