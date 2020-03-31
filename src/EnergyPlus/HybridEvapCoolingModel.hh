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

#ifndef HybridEvapCoolingModel_hh_INCLUDED
#define HybridEvapCoolingModel_hh_INCLUDED
#include <iostream>

#include <list>
#include <map>
#include <string>
#include <vector>
// ObjexxFCL Headers
#include <ObjexxFCL/Array.functions.hh>
#include <ObjexxFCL/Fmath.hh>

#define MINIMUM_LOAD_TO_ACTIVATE 0.5 // (kw) sets a minimum load to avoid the system fluttering on and off.
#define IMPLAUSIBLE_POWER 10000000
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

namespace HybridEvapCoolingModel {

    enum class SYSTEMOUTPUTS
    {
        VENTILATION_AIR_V,
        SUPPLY_MASS_FLOW,
        SYSTEM_FUEL_USE,
        SUPPLY_AIR_TEMP,
        MIXED_AIR_TEMP,
        SUPPLY_AIR_HR,
        MIXED_AIR_HR,
        OSUPPLY_FAN_POWER,
        OSECOND_FUEL_USE,
        OTHIRD_FUEL_USE,
        OWATER_USE,
        OEXTERNAL_STATIC_PRESSURE
    };

    class CModeSolutionSpace
    {
    public:
        std::vector<Nandle> MassFlowRatio;
        std::vector<Nandle> OutdoorAirFraction;
        void AddItem(Nandle X, Nandle Y)
        {
            MassFlowRatio.push_back(X);
            OutdoorAirFraction.push_back(Y);
        }
    };

    class CMode
    {
    public:
        CMode();

        // finish init above
        int ModeID;
        CModeSolutionSpace sol;
        std::string ModeName;
        int Tsa_curve_pointer;
        int HRsa_curve_pointer;
        int Psa_curve_pointer;
        int SFPsa_curve_pointer;
        int ESPsa_curve_pointer;
        int SFUsa_curve_pointer;
        int TFUsa_curve_pointer;
        int WUsa_curve_pointer;

        Nandle Max_Msa;
        Nandle Min_Msa;
        Nandle Min_OAF;
        Nandle Max_OAF;
        Nandle Minimum_Outdoor_Air_Temperature;
        Nandle Maximum_Outdoor_Air_Temperature;
        Nandle Minimum_Outdoor_Air_Humidity_Ratio;
        Nandle Maximum_Outdoor_Air_Humidity_Ratio;
        Nandle Minimum_Outdoor_Air_Relative_Humidity;
        Nandle Maximum_Outdoor_Air_Relative_Humidity;
        Nandle Minimum_Return_Air_Temperature;
        Nandle Maximum_Return_Air_Temperature;
        Nandle Minimum_Return_Air_Humidity_Ratio;
        Nandle Maximum_Return_Air_Humidity_Ratio;
        Nandle Minimum_Return_Air_Relative_Humidity;
        Nandle Maximum_Return_Air_Relative_Humidity;
        Nandle ModelScalingFactor;
        int MODE_BLOCK_OFFSET_Alpha;
        int BLOCK_HEADER_OFFSET_Alpha;
        int MODE1_BLOCK_OFFSET_Number;
        int MODE_BLOCK_OFFSET_Number;
        int BLOCK_HEADER_OFFSET_Number;
        bool ValidPointer(int curve_pointer);
        bool ValidateArrays(Array1D_string Alphas,
                            Array1D_string cAlphaFields,
                            Array1D<Nandle> Numbers,
                            Array1D_string cNumericFields,
                            std::string cCurrentModuleObject);
        bool ParseMode(int ModeCounter,
                       std::vector<CMode> *OperatingModes,
                       Nandle ScalingFactor,
                       Array1D_string Alphas,
                       Array1D_string cAlphaFields,
                       Array1D<Nandle> Numbers,
                       Array1D_string cNumericFields,
                       Array1D<bool> lAlphaBlanks,
                       std::string cCurrentModuleObject);
        void InitializeCurve(int curveType, int CurveID);
        Nandle CalculateCurveVal(Nandle Tosa, Nandle Wosa, Nandle Tra, Nandle Wra, Nandle Msa, Nandle OSAF, int curveType);
        bool InitializeOSAFConstraints(Nandle minOSAF, Nandle maxOSAF);
        bool InitializeMsaRatioConstraints(Nandle minMsa, Nandle maxMsa);
        bool InitializeOutdoorAirTemperatureConstraints(Nandle min, Nandle max);
        bool InitializeOutdoorAirHumidityRatioConstraints(Nandle min, Nandle max);
        bool InitializeOutdoorAirRelativeHumidityConstraints(Nandle min, Nandle max);
        bool InitializeReturnAirTemperatureConstraints(Nandle min, Nandle max);
        bool InitializeReturnAirHumidityRatioConstraints(Nandle min, Nandle max);
        bool InitializeReturnAirRelativeHumidityConstraints(Nandle min, Nandle max);
        bool GenerateSolutionSpace(Nandle ResolutionMsa, Nandle ResolutionOSA);
        bool MeetsOAEnvConstraints(Nandle Tosa, Nandle Wosa, Nandle RHos);

    private:
    };

    class CSetting
    {
    public:
        CSetting()
            : Runtime_Fraction(0), Mode(0), Outdoor_Air_Fraction(0), Unscaled_Supply_Air_Mass_Flow_Rate(0), ScaledSupply_Air_Mass_Flow_Rate(0),
              Supply_Air_Ventilation_Volume(0), ScaledSupply_Air_Ventilation_Volume(0), Supply_Air_Mass_Flow_Rate_Ratio(0), SupplyAirTemperature(0),
              Mixed_Air_Temperature(0), SupplyAirW(0), Mixed_Air_W(0), TotalSystem(0), SensibleSystem(0), LatentSystem(0), TotalZone(0),
              SensibleZone(0), LatentZone(0), ElectricalPower(IMPLAUSIBLE_POWER), SupplyFanElectricPower(0), SecondaryFuelConsumptionRate(0),
              ThirdFuelConsumptionRate(0), WaterConsumptionRate(0), ExternalStaticPressure(0)
        {
        }
        Nandle Runtime_Fraction;
        Nandle Mode;
        Nandle Outdoor_Air_Fraction;
        Nandle Unscaled_Supply_Air_Mass_Flow_Rate;
        Nandle ScaledSupply_Air_Mass_Flow_Rate;
        Nandle Supply_Air_Ventilation_Volume;
        Nandle ScaledSupply_Air_Ventilation_Volume;
        Nandle Supply_Air_Mass_Flow_Rate_Ratio;
        Nandle SupplyAirTemperature;
        Nandle Mixed_Air_Temperature;
        Nandle SupplyAirW;
        Nandle Mixed_Air_W;
        Nandle TotalSystem;
        Nandle SensibleSystem;
        Nandle LatentSystem;
        Nandle TotalZone;    // W
        Nandle SensibleZone; // W
        Nandle LatentZone;   // W
        Nandle ElectricalPower;
        Nandle SupplyFanElectricPower;
        Nandle SecondaryFuelConsumptionRate;
        Nandle ThirdFuelConsumptionRate;
        Nandle WaterConsumptionRate;
        Nandle ExternalStaticPressure;

        CMode oMode;
    };

    class CStepInputs
    {
    public:
        CStepInputs()
            : Tosa(0), Tra(0), RHosa(0), RHra(0), RequestedCoolingLoad(0), RequestedHeatingLoad(0), ZoneMoistureLoad(0), ZoneDehumidificationLoad(0),
              MinimumOA(0)
        {
        }
        Nandle Tosa;
        Nandle Tra;
        Nandle RHosa;
        Nandle RHra;
        Nandle RequestedCoolingLoad;
        Nandle RequestedHeatingLoad;
        Nandle ZoneMoistureLoad;
        Nandle ZoneDehumidificationLoad;
        Nandle MinimumOA;
    };

    class Model // begin declaration of the class
    {
    public: // begin public section
        Model();

        // Default Constructor
        std::string Name;     // user identifier
        std::string Schedule; // Availability Schedule Name
        bool Initialized;     // initialization flag ensures the system object is initialized only once.
        int ZoneNum;  // stores the current zone associated with the system, this is currently not used but is expected to be used in the next set of
                      // functionality additions.
        int SchedPtr; // Pointer to the correct schedule
        int ZoneNodeNum;                  // index of zone air node in node structure
        std::string AvailManagerListName; // Name of an availability manager list object
        int AvailStatus;

        Nandle SystemMaximumSupplyAirFlowRate;           // taken from IDF N1, the system max supply flow rate in m3/s.
        Nandle ScalingFactor;                            // taken from IDF N3, linear scaling factor.
        Nandle ScaledSystemMaximumSupplyAirMassFlowRate; // the scaled system max supply mass flow rate in m3/s.
        Nandle ScaledSystemMaximumSupplyAirVolumeFlowRate; // the scaled system max supply volume flow rate in m3/s.
        std::string FirstFuelType;             // First fuel type, currently electricity is only option
        std::string SecondFuelType;             // Second fuel type
        std::string ThirdFuelType;             // Third fuel type

        int UnitOn;                          // feels like it should be a bool but its an output and I couldn't get it to work as a bool
        Nandle UnitTotalCoolingRate;         // unit output to zone, total cooling rate [W]
        Nandle UnitTotalCoolingEnergy;       // unit output to zone, total cooling energy [J]
        Nandle UnitSensibleCoolingRate;      // unit sensible cooling rate [W]
        Nandle UnitSensibleCoolingEnergy;    // unit sensible cooling energy [J]
        Nandle UnitLatentCoolingRate;        // unit latent cooling rate [W]
        Nandle UnitLatentCoolingEnergy;      // unit latent cooling energy [J]
        Nandle SystemTotalCoolingRate;       // system output to zone, total cooling rate [W]
        Nandle SystemTotalCoolingEnergy;     // system output to zone, total cooling energy [J]
        Nandle SystemSensibleCoolingRate;    // system sensible cooling rate [W]
        Nandle SystemSensibleCoolingEnergy;  // system sensible cooling energy [J]
        Nandle SystemLatentCoolingRate;      // system latent cooling rate [W]
        Nandle SystemLatentCoolingEnergy;    // system latent cooling energy [J]
        Nandle UnitTotalHeatingRate;         // unit output to zone, total heating rate [W]
        Nandle UnitTotalHeatingEnergy;       // unit output to zone, total heating energy [J]
        Nandle UnitSensibleHeatingRate;      // unit sensible heating rate [W]
        Nandle UnitSensibleHeatingEnergy;    // unit sensible heating energy [J]
        Nandle UnitLatentHeatingRate;        // unit latent heating rate [W]
        Nandle UnitLatentHeatingEnergy;      // unit latent heating energy [J]
        Nandle SystemTotalHeatingRate;       // system output to zone, total heating rate [W]
        Nandle SystemTotalHeatingEnergy;     // system output to zone, total heating energy [J]
        Nandle SystemSensibleHeatingRate;    // system sensible heating rate [W]
        Nandle SystemSensibleHeatingEnergy;  // system sensible heating energy [J]
        Nandle SystemLatentHeatingRate;      // system latent heating rate [W]
        Nandle SystemLatentHeatingEnergy;    // system latent heating energy [J]
        Nandle SupplyFanElectricPower;       //
        Nandle SupplyFanElectricEnergy;      //
        Nandle SecondaryFuelConsumptionRate; //
        Nandle SecondaryFuelConsumption;     //
        Nandle ThirdFuelConsumptionRate;     //
        Nandle ThirdFuelConsumption;         //
        Nandle WaterConsumptionRate;         //
        Nandle WaterConsumption;             //
        Nandle QSensZoneOut;                 // W
        Nandle QLatentZoneOut;               // W
        Nandle QLatentZoneOutMass;           // kg/s
        Nandle ExternalStaticPressure;       //
        Nandle RequestedHumdificationMass;
        Nandle RequestedHumdificationLoad;
        Nandle RequestedHumdificationEnergy;
        Nandle RequestedDeHumdificationMass;
        Nandle RequestedDeHumdificationLoad;
        Nandle RequestedDeHumdificationEnergy;
        Nandle RequestedLoadToHeatingSetpoint;
        Nandle RequestedLoadToCoolingSetpoint;
        int TsaMin_schedule_pointer;
        int TsaMax_schedule_pointer;
        int RHsaMin_schedule_pointer;
        int RHsaMax_schedule_pointer;
        int PrimaryMode;
        Nandle PrimaryModeRuntimeFraction;
        Nandle averageOSAF;
        int ErrorCode;
        bool StandBy;
        int InletNode;
        int OutletNode;
        int SecondaryInletNode;       // This is usually OA node feeding into the purge/secondary side
        int SecondaryOutletNode;      // This outlet node of the secondary side and inlet to the secondary fan
        Nandle FinalElectricalPower;  // Output fuel use in W
        Nandle FinalElectricalEnergy; // Output fuel energy use in J
        Nandle InletMassFlowRate; // Inlet is primary process air node at inlet to cooler
        Nandle InletVolumetricFlowRate; // Inlet is primary process air node at inlet to cooler
        Nandle InletTemp;
        Nandle InletWetBulbTemp;
        Nandle InletHumRat;
        Nandle InletEnthalpy;
        Nandle InletPressure;
        Nandle InletRH;
        Nandle OutletVolumetricFlowRate;
        Nandle OutletMassFlowRate; // Inlet is primary process air node at inlet to cooler
        Nandle OutletTemp;
        Nandle OutletWetBulbTemp;
        Nandle OutletHumRat;
        Nandle OutletEnthalpy;
        Nandle OutletPressure;
        Nandle OutletRH;
        Nandle SecInletMassFlowRate; // Inlet is primary process air node at inlet to cooler
        Nandle SecInletTemp;
        Nandle SecInletWetBulbTemp;
        Nandle SecInletHumRat;
        Nandle SecInletEnthalpy;
        Nandle SecInletPressure;
        Nandle SecInletRH;
        Nandle SecOutletVolumetricFlowRate;
        Nandle SecOutletMassFlowRate; // Inlet is primary process air node at inlet to cooler
        Nandle SecOutletTemp;
        Nandle SecOutletWetBulbTemp;
        Nandle SecOutletHumRat;
        Nandle SecOutletEnthalpy;
        Nandle SecOutletPressure;
        Nandle SecOutletRH;
        Nandle Wsa;
        Nandle SupplyVentilationAir;
        Nandle SupplyVentilationVolume;

        bool OutdoorAir;
        Nandle MinOA_Msa;
        int OARequirementsPtr; // Index to DesignSpecification:OutdoorAir object

        Nandle Tsa;
        int ModeCounter;
        bool CoolingRequested;
        bool HeatingRequested;
        bool VentilationRequested;
        bool DehumidificationRequested;
        bool HumidificationRequested;
        // non-initializer
        std::vector<int> Tsa_curve_pointer;
        std::vector<int> HRsa_curve_pointer;
        std::vector<int> Psa_curve_pointer;
        std::vector<CMode> OperatingModes;
        std::vector<CSetting> CurrentOperatingSettings;

        CSetting OptimalSetting;
        CSetting oStandBy;

        std::vector<CSetting> Settings;
        // methods
        int CurrentPrimaryMode();
        Nandle CurrentPrimaryRuntimeFraction();
        Nandle CalculatePartRuntimeFraction(Nandle MinOA_Msa,
                                            Nandle Mvent,
                                            Nandle RequestedCoolingLoad,
                                            Nandle RequestedHeatingLoad,
                                            Nandle SensibleRoomORZone,
                                            Nandle RequestedDehumidificationLoad,
                                            Nandle RequestedMoistureLoad,
                                            Nandle LatentRoomORZone);
        bool ParseMode(Array1D_string Alphas,
                       Array1D_string cAlphaFields,
                       Array1D<Nandle> Numbers,
                       Array1D_string cNumericFields,
                       Array1D<bool> lAlphaBlanks,
                       std::string cCurrentModuleObject);
        void
        doStep(Nandle RequestedLoad, Nandle ZoneHeatingLoad, Nandle OutputRequiredToHumidify, Nandle OutputRequiredToDehumidify, Nandle DesignMinVR);
        void Initialize(int ZoneNumber);
        void InitializeModelParams();
        void ResetOutputs();
        bool MeetsSupplyAirTOC(Nandle Tosa);
        bool MeetsSupplyAirRHOC(Nandle Wosa);
        Nandle CheckVal_T(Nandle T);
        Nandle CheckVal_W(Nandle W, Nandle T, Nandle P); // pascals
        bool SetStandByMode(CMode Mode0, Nandle Tosa, Nandle Wosa, Nandle Tra, Nandle Wra);
        Nandle CalculateTimeStepAverage(SYSTEMOUTPUTS val);
        int SetOperatingSetting(CStepInputs StepIns);
        void DetermineCoolingVentilationOrHumidificationNeeds(CStepInputs &StepIns);

    private: // begin private section
        // number of times in a day it failed resulting in a warning.
        std::vector<int> SAT_OC_MetinMode_v;
        std::vector<int> SAHR_OC_MetinMode_v;
        bool WarnOnceFlag;
        Nandle ResolutionMsa;
        Nandle ResolutionOSA;
        int count_EnvironmentConditionsNotMet;
        int count_EnvironmentConditionsMetOnce;
        int count_SAHR_OC_MetOnce;
        int count_SAT_OC_MetOnce;
        int count_DidWeMeetLoad;
        int count_DidWeNotMeetLoad;

        bool optimal_EnvCondMet;
        bool RunningPeakCapacity_EnvCondMet;

        std::vector<Nandle> PolygonXs;
        std::vector<Nandle> PolygonYs;
    };
} // namespace HybridEvapCoolingModel
} // namespace EnergyPlus

#endif
