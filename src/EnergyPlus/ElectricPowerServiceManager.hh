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

#ifndef ElectricPowerServiceManager_hh_INCLUDED
#define ElectricPowerServiceManager_hh_INCLUDED

// C++ Headers
#include <memory>
#include <string>
#include <vector>

// ObjexxFCL Headers
//#include <ObjexxFCL/Array1.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataHeatBalance.hh>
#include <EnergyPlus/EMSManager.hh>
#include <EnergyPlus/EnergyPlus.hh>
#include <EnergyPlus/OutputProcessor.hh>
#include <EnergyPlus/Plant/PlantLocation.hh>

namespace EnergyPlus {

enum class ThermalLossDestination : int
{
    heatLossNotDetermined = 0,
    zoneGains,    // device thermal losses are added to a zone as internal gains
    lostToOutside // device thermal losses have no destination
};

void initializeElectricPowerServiceZoneGains();

class DCtoACInverter
// This class is for modelling a power conversion device that takes DC power in and produces AC power out.
// This class combines three separate input objects that have different methods of determining efficiency.
{

public: // Methods
    enum class InverterModelType : int
    {
        notYetSet,
        cECLookUpTableModel,
        curveFuncOfPower,
        simpleConstantEff,
        pvWatts,
    };

    // Constructor
    DCtoACInverter(std::string const &objectName);

    void simulate(Nandle const powerIntoInverter);

    void reinitAtBeginEnvironment();

    void reinitZoneGainsAtBeginEnvironment();

    void setPVWattsDCCapacity(Nandle const dcCapacity);

    Nandle pvWattsDCCapacity();

    Nandle thermLossRate() const;

    Nandle getLossRateForOutputPower(Nandle const powerOutOfInverter);

    Nandle aCPowerOut() const;

    Nandle aCEnergyOut() const;

    InverterModelType modelType() const;

    std::string const &name() const;

private: // Methods
    void calcEfficiency();

private:               // data
    std::string name_; // user identifier
    Nandle aCPowerOut_;
    Nandle aCEnergyOut_;
    Nandle efficiency_;
    Nandle dCPowerIn_;
    Nandle dCEnergyIn_;
    Nandle conversionLossPower_;
    Nandle conversionLossEnergy_;
    Nandle conversionLossEnergyDecrement_;
    Nandle thermLossRate_;
    Nandle thermLossEnergy_;
    Nandle qdotConvZone_;
    Nandle qdotRadZone_;
    Nandle ancillACuseRate_;
    Nandle ancillACuseEnergy_;
    InverterModelType modelType_; // type of inverter model used
    int availSchedPtr_;           // number for availability schedule.
    ThermalLossDestination heatLossesDestination_;
    int zoneNum_;                              // destination zone for heat losses from inverter.
    Nandle zoneRadFract_;                      // radiative fraction for thermal losses to zone
    Nandle nominalVoltage_;                    // CEC lookup table model
    std::vector<Nandle> nomVoltEfficiencyARR_; // eff at 10, 20, 30, 50, 75, & 100% CEC lookup table model
    int curveNum_;                             // curve index for eff as func of power
    Nandle ratedPower_;                        // rated, max continuous power output level for inverter
    Nandle minPower_;
    Nandle maxPower_;
    Nandle minEfficiency_;
    Nandle maxEfficiency_;
    Nandle standbyPower_;
    Nandle pvWattsDCtoACSizeRatio_;
    Nandle pvWattsInverterEfficiency_;

}; // DCtoACInverter

class ACtoDCConverter
// This class is for modelling a power conversion device that takes AC power in and produces DC power out.

{

public: // Methods
    // Constructor
    ACtoDCConverter(std::string const &objectName);

    void simulate(Nandle const powerOutFromConverter);

    void reinitAtBeginEnvironment();

    void reinitZoneGainsAtBeginEnvironment();

    Nandle thermLossRate() const;

    Nandle dCPowerOut() const;

    Nandle dCEnergyOut() const;

    Nandle aCPowerIn() const;

    Nandle getLossRateForInputPower(Nandle const powerIntoConverter); // AC power going into inverter

    std::string const &name() const;

private: // methods
    void calcEfficiency();

private: // data
    enum class ConverterModelType : int
    {
        notYetSet,
        curveFuncOfPower,
        simpleConstantEff
    };

    std::string name_; // user identifier
    Nandle efficiency_;
    Nandle aCPowerIn_;
    Nandle aCEnergyIn_;
    Nandle dCPowerOut_;
    Nandle dCEnergyOut_;
    Nandle conversionLossPower_;
    Nandle conversionLossEnergy_;
    Nandle conversionLossEnergyDecrement_;
    Nandle thermLossRate_;
    Nandle thermLossEnergy_;
    Nandle qdotConvZone_;
    Nandle qdotRadZone_;
    Nandle ancillACuseRate_;
    Nandle ancillACuseEnergy_;
    int availSchedPtr_;            // number for availability schedule.
    ConverterModelType modelType_; // type of inverter model used
    int curveNum_;                 // performance curve or table index
    ThermalLossDestination heatLossesDestination_;
    int zoneNum_;         // destination zone for heat losses from inverter.
    Nandle zoneRadFract_; // radiative fraction for thermal losses to zone
    Nandle standbyPower_;
    Nandle maxPower_;
};

class ElectricStorage
// This class is for modeling a device for storing electric power over time.
// This class combines two separate input objects that have different models.
{

public: // methods
    // Constructor
    ElectricStorage(std::string const &objectName);

    void timeCheckAndUpdate();

    void simulate(Nandle &powerCharge,
                  Nandle &powerDischarge,
                  bool &charging,
                  bool &discharging,
                  Nandle const controlSOCMaxFracLimit,
                  Nandle const controlSOCMinFracLimit);

    void calcAndReportSimpleBucketModel();

    void calcAndReportKineticBatteryModel();

    void reinitAtBeginEnvironment();

    void reinitZoneGainsAtBeginEnvironment();

    void reinitAtEndWarmup();

    Nandle drawnPower() const;

    Nandle storedPower() const;

    Nandle drawnEnergy() const;

    Nandle storedEnergy() const;

    bool determineCurrentForBatteryDischarge(Nandle &curI0,
                                             Nandle &curT0,
                                             Nandle &curVolt,
                                             Nandle const Pw,
                                             Nandle const q0,
                                             int const CurveNum,
                                             Nandle const k,
                                             Nandle const c,
                                             Nandle const qmax,
                                             Nandle const E0c,
                                             Nandle const InternalR);

    std::string const &name() const;

private:                            // methods
    void simulateSimpleBucketModel( // request charge discharge and
        Nandle &powerCharge,
        Nandle &powerDischarge,
        bool &charging,
        bool &discharging,
        Nandle const controlSOCMaxFracLimit,
        Nandle const controlSOCMinFracLimit);

    void simulateKineticBatteryModel(Nandle &powerCharge,
                                     Nandle &powerDischarge,
                                     bool &charging,
                                     bool &discharging,
                                     Nandle const controlSOCMaxFracLimit,
                                     Nandle const controlSOCMinFracLimit);

    void rainflow(int const numbin,           // numbin = constant value
                  Nandle const input,         // input = input value from other object (battery model)
                  std::vector<Nandle> &B1,    // stores values of points, calculated here - stored for next timestep
                  std::vector<Nandle> &X,     // stores values of two data point difference, calculated here - stored for next timestep
                  int &count,                 // calculated here - stored for next timestep in main loop
                  std::vector<Nandle> &Nmb,   // calculated here - stored for next timestep in main loop
                  std::vector<Nandle> &OneNmb // calculated here - stored for next timestep in main loop
                                              //	int const dim // end dimension of array
    );

    void shift(std::vector<Nandle> &A, int const m, int const n, std::vector<Nandle> &B
               //	int const dim // end dimension of arrays
    );

private: // data
    enum class StorageModelType : int
    {
        storageTypeNotSet = 0,
        simpleBucketStorage,
        kiBaMBattery
    };

    enum class BatteyDegredationModelType : int
    {
        degredationNotSet = 0,
        lifeCalculationYes,
        lifeCalculationNo
    };

    std::string name_;               // name of this electrical storage module
    Nandle storedPower_;             // [W]
    Nandle storedEnergy_;            // [J]
    Nandle drawnPower_;              // [W]
    Nandle drawnEnergy_;             // [J]
    Nandle decrementedEnergyStored_; // [J] this is the negative of StoredEnergy
    int maxRainflowArrayBounds_;
    int const maxRainflowArrayInc_ = 100;
    bool myWarmUpFlag_;
    StorageModelType storageModelMode_;            // type of model parameter, SimpleBucketStorage
    int availSchedPtr_;                            // availability schedule index.
    ThermalLossDestination heatLossesDestination_; // mode for where thermal losses go
    int zoneNum_;                                  // destination zone for heat losses from inverter.
    Nandle zoneRadFract_;                          // radiative fraction for thermal losses to zone
    Nandle startingEnergyStored_;                  // [J] joules inside at beginning of environment period
    Nandle energeticEfficCharge_;                  // [ ] efficiency of charging
    Nandle energeticEfficDischarge_;               // [ ] efficiency of discharging
    Nandle maxPowerDraw_;                          // [W] max rate of discharge
    Nandle maxPowerStore_;                         // [W] max rate of charge
    Nandle maxEnergyCapacity_;                     // [J] max storage capacity
    int parallelNum_;                              // [ ] number of battery modules in parallel
    int seriesNum_;                                // [ ] number of battery modules in series
    int numBattery_;                               // total number of batteries all together
    int chargeCurveNum_;                           // [ ] voltage change curve index number for charging
    int dischargeCurveNum_;                        // [ ] voltage change curve index number for discharging
    int cycleBinNum_;                              // [ ] number of cycle bins
    Nandle startingSOC_;                           // [ ] initial fractional state of charge
    Nandle maxAhCapacity_;                         // [Ah]maximum capacity
    Nandle availableFrac_;                         // [ ] fraction of available charge capacity
    Nandle chargeConversionRate_;                  // [1/h]change rate from bound charge energy to available charge
    Nandle chargedOCV_;                            // [V] fully charged open circuit voltage
    Nandle dischargedOCV_;                         // [V] fully discharged open circuit voltage
    Nandle internalR_;                             // [ohm]internal electric resistance
    Nandle maxDischargeI_;                         // [A] maximum discharging current
    Nandle cutoffV_;                               // [V] cut-off voltage
    Nandle maxChargeRate_;                         // [1/h]charge rate limit
    BatteyDegredationModelType lifeCalculation_;   // [ ]battery life calculation: Yes or No
    int lifeCurveNum_;                             // [ ]battery life curve name index number
    // calculated and from elsewhere vars
    Nandle thisTimeStepStateOfCharge_; // [J]
    Nandle lastTimeStepStateOfCharge_; // [J]
    Nandle pelNeedFromStorage_;        // [W]
    Nandle pelFromStorage_;            // [W]
    Nandle pelIntoStorage_;            // [W]
    Nandle qdotConvZone_;              // [W]
    Nandle qdotRadZone_;               // [W]
    Nandle timeElapsed_;               // [h]
    Nandle thisTimeStepAvailable_;     // [Ah] available charge at the current timestep
    Nandle thisTimeStepBound_;         // [Ah] bound charge at the current timestep
    Nandle lastTimeStepAvailable_;     // [Ah] available charge at the previous timestep
    Nandle lastTimeStepBound_;         // [Ah] bound charge at the previous timestep
    Nandle lastTwoTimeStepAvailable_;  // [Ah] available charge at the previous two timesteps
    Nandle lastTwoTimeStepBound_;      // [Ah] bound charge at the previous two timesteps
    // battery life calculation variables
    int count0_;
    std::vector<Nandle> b10_;
    std::vector<Nandle> x0_;
    std::vector<Nandle> nmb0_;
    std::vector<Nandle> oneNmb0_;
    // report
    Nandle electEnergyinStorage_; // [J] state of charge
    Nandle thermLossRate_;        // [W]
    Nandle thermLossEnergy_;      // [J]
    int storageMode_;             // [ ] mode of operation 0 for idle, 1 for discharging, 2 for charging
    Nandle absoluteSOC_;          // [Ah] total state of charge
    Nandle fractionSOC_;          // [ ] fractional state of charge
    Nandle batteryCurrent_;       // [A] total current
    Nandle batteryVoltage_;       // [V] total voltage
    Nandle batteryDamage_;        // [ ] fractional battery damage

}; // ElectricStorage

class ElectricTransformer
// This class is for modeling a power conversion device that changes from one voltage to another, or serves as an isolation transformer
{

public: // methods
    // Constructor
    ElectricTransformer(std::string const &objectName);

    Nandle getLossRateForOutputPower(Nandle const powerOutOfTransformer);

    Nandle getLossRateForInputPower(Nandle const powerIntoTransformer);

    void manageTransformers(Nandle const surplusPowerOutFromLoadCenters);

    void setupMeterIndices();

    void reinitAtBeginEnvironment();

    void reinitZoneGainsAtBeginEnvironment();

    std::string const &name() const;

private: // data
    enum class TransformerUse : int
    {
        usenotYetSet = 0,
        powerInFromGrid,              // condition power from grid going into building buss
        powerOutFromBldgToGrid,       // condition power from building buss going out to grid
        powerBetweenLoadCenterAndBldg // condition power from a load center going into building buss, or from building buss into load center for draws
    };
    enum class TransformerPerformanceInput : int
    {
        perfInputMethodNotSet = 0,
        lossesMethod,
        efficiencyMethod
    };

    std::string name_; // user identifier
    bool myOneTimeFlag_;
    int availSchedPtr_;                                // availability schedule index.
    TransformerUse usageMode_;                         // mode for transformer usage
    ThermalLossDestination heatLossesDestination_;     // mode for where thermal losses go
    int zoneNum_;                                      // destination zone for heat losses from inverter.
    Nandle zoneRadFrac_;                               // radiative fraction for thermal losses to zone
    Nandle ratedCapacity_;                             // rated capacity [VA]
    int phase_;                                        // phase
    Nandle factorTempCoeff_;                           // thermal coefficient of resistance for winding material
    Nandle tempRise_;                                  // full load temperature rise [C]
    Nandle eddyFrac_;                                  // fraction of eddy current losses []
    TransformerPerformanceInput performanceInputMode_; // performance input method
    Nandle ratedEfficiency_;                           // nameplate efficiency []
    Nandle ratedPUL_;                                  // per unit load for nameplate efficiency []
    Nandle ratedTemp_;                                 // reference temperature for nameplate efficiency [C]
    Nandle maxPUL_;                                    // per unit load for maximum efficiency []
    bool considerLosses_;                              // if true, consider transformer lossses in metering
    std::vector<std::string> wiredMeterNames_;         // names of the meters wired to transformer
    std::vector<int> wiredMeterPtrs_;                  // array of "pointers" to meters wired to transformer
    std::vector<bool> specialMeter_;                   // indicates whether a meter needs special consideration
    // Electricity:Facility and Electricity:HVAC are two special
    // meters because tranformer loss is part of them
    // calculated and from elsewhere vars
    Nandle ratedNL_;         // rated no load losses, user input or calculated [W]
    Nandle ratedLL_;         // rated load losses, user input or calculated [W]
    int overloadErrorIndex_; // used for warning message when transformer is overloaded
    // results and reporting
    Nandle efficiency_;                  // transformer efficiency
    Nandle powerIn_;                     // [W]
    Nandle energyIn_;                    // [J]
    Nandle powerOut_;                    // [W]
    Nandle energyOut_;                   // [J]
    Nandle noLoadLossRate_;              // [W]
    Nandle noLoadLossEnergy_;            // [J]
    Nandle loadLossRate_;                // [W]
    Nandle loadLossEnergy_;              // [J]
    Nandle totalLossRate_;               // [W]
    Nandle totalLossEnergy_;             // [J]
    Nandle thermalLossRate_;             // [W]
    Nandle thermalLossEnergy_;           // [J]
    Nandle elecUseMeteredUtilityLosses_; // [J] Energy consumption for a utility transformer (power in)
    // Positive values
    Nandle powerConversionMeteredLosses_; // [J] Energy consumption for a (cogeneration )transformer (power out from building to grid)
    // Negative values
    Nandle qdotConvZone_; // [W]
    Nandle qdotRadZone_;  // [W]
};                        // ElectricTransformer

class GeneratorController
// this class is used as part of the supervisory control and calling of electric power generators.  Each instances is for one generator
{

public: // Method
    // Constructor
    GeneratorController(std::string const &objectName,
                        std::string const &objectType,
                        Nandle const ratedElecPowerOutput,
                        std::string const &availSchedName,
                        Nandle const thermalToElectRatio);

    void simGeneratorGetPowerOutput(bool const runFlag,             // true if generator is on
                                    Nandle const myElecLoadRequest, // target electric power production request
                                    bool const FirstHVACIteration,  //
                                    Nandle &electricPowerOutput,    // Actual generator electric power output
                                    Nandle &thermalPowerOutput      // Actual generator thermal power output
    );

    void reinitAtBeginEnvironment();

public: // data // might make this class a friend of ElectPowerLoadCenter?
    enum class GeneratorType : int
    {
        notYetSet = 0,
        iCEngine,
        combTurbine,
        pV,
        fuelCell,
        microCHP,
        microturbine,
        windTurbine,
        pvWatts,
    };

    std::string name;          // user identifier
    std::string typeOfName;    // equipment type
    int compGenTypeOf_Num;     // Numeric designator for generator CompType (TypeOf), in DataGlobalConstants
    int compPlantTypeOf_Num;   // numeric designator for plant component, in DataPlant
    std::string compPlantName; // name of plant component if heat recovery
    GeneratorType generatorType;
    int generatorIndex;              // index in generator model data struct
    Nandle maxPowerOut;              // Maximum Power Output (W)
    std::string availSched;          // Operation Schedule.
    int availSchedPtr;               // pointer to operation schedule
    Nandle powerRequestThisTimestep; // Current Demand on Equipment (W)
    bool onThisTimestep;             // Indicator whether Generator on
    Nandle eMSPowerRequest;          // EMS actuator for current demand on equipment (W)
    bool eMSRequestOn;               // EMS actuating On if true.
    bool plantInfoFound;
    PlantLocation cogenLocation;
    Nandle nominalThermElectRatio; // Cogen: nominal ratio of thermal to elect production
    // results of component models for load center reporting
    Nandle dCElectricityProd; // Current DC Electric Produced from Equipment (J)
    Nandle dCElectProdRate;   // Current DC Electric Production Rate from Equipment (W)
    Nandle electricityProd;   // Current AC Electric Produced from Equipment (J)
    Nandle electProdRate;     // Current AC Electric Production Rate from Equipment (W)
    Nandle thermalProd;       // Current Thermal energy Produced from Equipment (J)
    Nandle thermProdRate;     // Current Thermal energy Production Rate from Equipment (W)

private:
    int errCountNegElectProd_; // error count for reccuring error when generators produce negative electric power

}; // class GeneratorController

class ElectPowerLoadCenter
// This class if for modeling a load center which can be thought of as a kind of subpanel that connects power equipment to a main panel
// multiple subpanels can be connected to the main panel and each ElectPowerLoadCenter object is a subpanel
// Each load center will contain other power conversion devices and/or generator(s).
{

public: // Methods
    // Constructor
    ElectPowerLoadCenter(int const objectNum);

    void manageElecLoadCenter(bool const firstHVACIteration, Nandle &remainingPowerDemand);

    void setupLoadCenterMeterIndices();

    void reinitAtBeginEnvironment();

    void reinitZoneGainsAtBeginEnvironment();

    std::string const &transformerName() const;

    std::string const &generatorListName() const;

    void updateLoadCenterGeneratorRecords();

private: // Methods
    void dispatchGenerators(bool const firstHVACIteration, Nandle &remainingPowerDemand);

    void dispatchStorage(Nandle const remainingPowerDemand);

    Nandle calcLoadCenterThermalLoad(); // returns heat rate called for from cogenerator(watts)

public: // data public for unit test
    enum class ElectricBussType : int
    {
        notYetSet = 0,
        aCBuss,
        dCBussInverter,
        aCBussStorage,
        dCBussInverterDCStorage,
        dCBussInverterACStorage
    };

    std::unique_ptr<ElectricStorage> storageObj;
    std::unique_ptr<ACtoDCConverter> converterObj;
    std::unique_ptr<ElectricTransformer> transformerObj;
    int numGenerators;                                                 // Number of Generators
    std::vector<std::unique_ptr<GeneratorController>> elecGenCntrlObj; // generator controller objects
    ElectricBussType bussType;                                         // is this load center powered by AC or DC generators
    Nandle thermalProd;                                                // Current thermal energy Produced from generators in load center (J)
    Nandle thermalProdRate;                                            // Current thermal energy production rate from generators in load center (W)
    bool inverterPresent;
    std::string inverterName; // hold name for verificaton and error messages
    std::unique_ptr<DCtoACInverter> inverterObj;
    Nandle subpanelFeedInRequest;
    // subpanel terms, interact with main panel
    Nandle subpanelFeedInRate; // Current AC electric power fed into main panel by load center, adjusted by inverter if any (W)
    Nandle subpanelDrawRate;   // Current AC electric power draw from main panel into load center (W)
    // storage operation terms,
    Nandle genElectricProd;       // Current electric produced by generators in the load center, DC or AC (J)
    Nandle genElectProdRate;      // Current electric power produced by generators in the load center, DC or AC (W)
    Nandle storOpCVGenRate;       // power from generators (and maybe inverter) going into storage operation control volume, DC or AC ( W )
    Nandle storOpCVDrawRate;      // power drawn from main panel into storage operation control volume after any converter, DC or AC ( W )
    Nandle storOpCVFeedInRate;    // power fed toward main panel from storage operation control volume before any inverter, DC or AC ( W )
    Nandle storOpCVChargeRate;    // power fed into storage device from storage operation control volume, before any storage losses, DC or AC ( W )
    Nandle storOpCVDischargeRate; // power drawn from storage device into storage operation control volume, after any storage losses, DC or AC ( W )
    bool storOpIsCharging;        // true if storage operation scheme is trying to charge
    bool storOpIsDischarging;     // true if storage operation scheme is trying to discharge

private: // data
    enum class GeneratorOpScheme : int
    {
        notYetSet = 0,
        baseLoad,
        demandLimit,
        trackElectrical,
        trackSchedule,
        trackMeter,
        thermalFollow,
        thermalFollowLimitElectrical
    };

    enum class StorageOpScheme : int
    {
        notYetSet = 0,
        facilityDemandStoreExcessOnSite, // legacy control behavior
        meterDemandStoreExcessOnSite,
        chargeDischargeSchedules,
        facilityDemandLeveling
    };

    std::string name_;                     // user identifier
    std::string generatorListName_;        // List name of available generators
    GeneratorOpScheme genOperationScheme_; // Name of Operation Scheme
    std::string demandMeterName_;          // Name of Demand Energy Meter for "on demand" operation
    int demandMeterPtr_;                   // "pointer" to Meter for electrical Demand to meet
    std::string generationMeterName_;      // Name of Generated Energy Meter for "on demand" operation
    bool generatorsPresent_;               // true if any generators
    bool myCoGenSetupFlag_;
    Nandle demandLimit_;            // Demand Limit in Watts(W) which the generator will operate above
    int trackSchedPtr_;             // "pointer" to schedule for electrical demand to meet.
    Nandle dCElectricityProd_;      // Current DC Elect produced (J) (if buss type DCbussInverter)
    Nandle dCElectProdRate_;        // Current DC Elect power produced (W) (if buss type DCbussInverter)
    Nandle dCpowerConditionLosses_; // current DC to AC inverter losses (W) (if DCbussInverter)
    bool storagePresent_;
    std::string storageName_;            // hold name for verificaton and error messages
    bool transformerPresent_;            // should only be transformers for on-site load center, not facility service
    std::string transformerName_;        // hold name for verificaton and error messages
    Nandle totalPowerRequest_;           // Total electric power request from the load center (W)
    Nandle totalThermalPowerRequest_;    // Total thermal power request from the load center (W)
    StorageOpScheme storageScheme_;      // what options are available for charging storage.
    std::string trackSorageOpMeterName_; // user name for a specific meter
    int trackStorageOpMeterIndex_;       // points to meter being
    bool converterPresent_;
    std::string converterName_;
    Nandle maxStorageSOCFraction_; // Fraction of storage capacity used as upper limit for controlling charging (don't overcharge the batteries)
    Nandle minStorageSOCFraction_; // Fraction of storage capacity used as lower limit for controlling discharging (dont drain the batteries too far)
    Nandle designStorageChargePower_;        // rate of electric power drawn from grid to go into storage
    bool designStorageChargePowerWasSet_;    // true if a value was input
    Nandle designStorageDischargePower_;     // rate of electric power exported to grid by being drawn from storage
    bool designStorageDischargePowerWasSet_; // true if value was input
    int storageChargeModSchedIndex_;         // index of fraction schedule for controlling charge rate over time
    int storageDischargeModSchedIndex_;      // index of fraction schedule for controlling discharge rate over time.
    Nandle facilityDemandTarget_;            // target utility demand level in Watts
    int facilityDemandTargetModSchedIndex_;  // index of fracton schedule for controlling target demand over time.
    bool eMSOverridePelFromStorage_;         // if true, EMS calling for override
    Nandle eMSValuePelFromStorage_;          // value EMS is directing to use, power from storage [W]
    bool eMSOverridePelIntoStorage_;         // if true, EMS calling for override
    Nandle eMSValuePelIntoStorage_;          // value EMS is directing to use, power into storage [W]

}; // class ElectPowerLoadCenter

class ElectricPowerServiceManager //
                                  // This class if the top level object for modeling complex electric power service.  It contains transformers and/or
                                  // load center(s).
{

public: // Creation
    // Default Constructor
    ElectricPowerServiceManager()
        : newEnvironmentInternalGainsFlag(true), numElecStorageDevices(0), getInputFlag_(true), newEnvironmentFlag_(true), numLoadCenters_(0),
          numTransformers_(0), setupMeterIndexFlag_(true), elecFacilityIndex_(0), elecProducedCoGenIndex_(0), elecProducedPVIndex_(0),
          elecProducedWTIndex_(0), elecProducedStorageIndex_(0), elecProducedPowerConversionIndex_(0), name_("Whole Building"),
          facilityPowerInTransformerPresent_(false), numPowerOutTransformers_(0), wholeBldgRemainingLoad_(0.0), electricityProd_(0.0),
          electProdRate_(0.0), electricityPurch_(0.0), electPurchRate_(0.0), electSurplusRate_(0.0), electricitySurplus_(0.0),
          electricityNetRate_(0.0), electricityNet_(0.0), totalBldgElecDemand_(0.0), totalHVACElecDemand_(0.0), totalElectricDemand_(0.0),
          elecProducedPVRate_(0.0), elecProducedWTRate_(0.0), elecProducedStorageRate_(0.0), elecProducedPowerConversionRate_(0.0),
          elecProducedCoGenRate_(0.0)
    {
    }

public: // Methods
    // Destructor
    ~ElectricPowerServiceManager()
    {
    }

    void manageElectricPowerService(bool const FirstHVACIteration,
                                    bool &SimElecCircuits,      // simulation convergence flag
                                    bool const UpdateMetersOnly // if true then don't resimulate generators, just update meters.
    );

    void reinitZoneGainsAtBeginEnvironment();

    void verifyCustomMetersElecPowerMgr();

private: // Methods
    void getPowerManagerInput();

    void setupMeterIndices();

    void reinitAtBeginEnvironment();

    void updateWholeBuildingRecords();

    void reportPVandWindCapacity();

    void sumUpNumberOfStorageDevices();

    void checkLoadCenters();

public: // data
    bool newEnvironmentInternalGainsFlag;
    int numElecStorageDevices;
    std::vector<std::unique_ptr<ElectPowerLoadCenter>> elecLoadCenterObjs;

private:                      // data
    bool getInputFlag_;       // control if object needs to get input and call factory methods
    bool newEnvironmentFlag_; // control if object needs to reinit at beginning of a new environment period
    int numLoadCenters_;
    int numTransformers_;
    bool setupMeterIndexFlag_; // control if object needs to make calls to GetMeterIndex
    int elecFacilityIndex_;
    int elecProducedCoGenIndex_;
    int elecProducedPVIndex_;
    int elecProducedWTIndex_;
    int elecProducedStorageIndex_;
    int elecProducedPowerConversionIndex_;
    std::string name_;
    bool facilityPowerInTransformerPresent_;
    std::string facilityPowerInTransformerName_; // hold name for verificaton and error messages
    std::unique_ptr<ElectricTransformer> facilityPowerInTransformerObj_;
    int numPowerOutTransformers_;
    std::string powerOutTransformerName_;
    std::unique_ptr<ElectricTransformer> powerOutTransformerObj_;
    Nandle wholeBldgRemainingLoad_;
    Nandle electricityProd_;                 // Current Electric Produced from Equipment (J)
    Nandle electProdRate_;                   // Current Electric Production Rate from Equipment (W)
    Nandle electricityPurch_;                // Current Purchased Electric (J)
    Nandle electPurchRate_;                  // Current Electric Purhcased Rate (W)
    Nandle electSurplusRate_;                // Current excess power (W)
    Nandle electricitySurplus_;              // Current excess energy (J)
    Nandle electricityNetRate_;              // Net elect rate, + is Purchased, - is Surplus (W)
    Nandle electricityNet_;                  // Net energy, + is Purchased, - is Surplus (J)
    Nandle totalBldgElecDemand_;             // Current Total Building Electric Demand (W)
    Nandle totalHVACElecDemand_;             // Current Total HVAC Electric Demand (W)
    Nandle totalElectricDemand_;             // Current Total Electric Demand (W)
    Nandle elecProducedPVRate_;              // Current Rate of PV Produced from the Arrays (W)
    Nandle elecProducedWTRate_;              // Current Rate of Wind Turbine Produced (W)
    Nandle elecProducedStorageRate_;         // Current Rate of power to(-)/from(+) storage
    Nandle elecProducedPowerConversionRate_; // Current rate of power loss from power conversion, negative (W)
    Nandle elecProducedCoGenRate_;           // Current Rate of Cogeneration generators produced ( W )
    Nandle pvTotalCapacity_;                 // for LEED report, total installed PV capacity
    Nandle windTotalCapacity_;               // for LEED report, total installed wind capacity

}; // class ElectricPowerServiceManager

extern std::unique_ptr<ElectricPowerServiceManager> facilityElectricServiceObj;

void createFacilityElectricPowerServiceObject();

void clearFacilityElectricPowerServiceObject();

} // namespace EnergyPlus
#endif // ElectricPowerServiceManager_hh_INCLUDED
