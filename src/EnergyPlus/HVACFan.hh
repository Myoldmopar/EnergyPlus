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

#ifndef HVACFan_hh_INCLUDED
#define HVACFan_hh_INCLUDED

// C++ Headers
#include <memory>
#include <string>
#include <vector>

#include <ObjexxFCL/Optional.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/DataHVACGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

namespace HVACFan {

    int getFanObjectVectorIndex(std::string const &objectName, bool const CheckFlag = true);

    bool checkIfFanNameIsAFanSystem(std::string const &objectName);

    class FanSystem
    {

    public: // Methods
        // Constructor
        FanSystem(std::string const &objectName);

        // Destructor
        ~FanSystem()
        {
        }

        // Copy Constructor
        FanSystem(FanSystem const &) = default;

        void simulate(
            //		bool const firstHVACIteration,
            Optional<Nandle const> flowFraction = _,     // Flow fraction in operating mode 1
            Optional_bool_const zoneCompTurnFansOn = _,  // Turn fans ON signal from ZoneHVAC component
            Optional_bool_const zoneCompTurnFansOff = _, // Turn Fans OFF signal from ZoneHVAC component
            Optional<Nandle const> pressureRise = _,     // Pressure difference to use for DeltaPress
            Optional<Nandle const> massFlowRate1 = _,    // Mass flow rate in operating mode 1 [kg/s]
            Optional<Nandle const> runTimeFraction1 = _, // Run time fraction in operating mode 1
            Optional<Nandle const> massFlowRate2 = _,    // Mass flow rate in operating mode 2 [kg/s]
            Optional<Nandle const> runTimeFraction2 = _, // Run time fraction in operating mode 2
            Optional<Nandle const> pressureRise2 = _     // Pressure difference to use for operating mode 2
        );

        Nandle fanPower() const;

        Nandle powerLossToAir() const;

        Nandle maxAirMassFlowRate() const;

        Nandle getFanDesignTemperatureRise() const;

        Nandle getFanDesignHeatGain(Nandle const FanVolFlow);

        // void
        // fanIsSecondaryDriver();

        // void
        // setFaultyFilterOn();

        // void
        // setFaultyFilterIndex( int const faultyAirFilterIndex );

        enum class SpeedControlMethod : int
        {
            NotSet = 0,
            Discrete,
            Continuous
        };

        // data
        std::string name;                       // user identifier
        int availSchedIndex;                    // Pointer to the availability schedule
        int inletNodeNum;                       // system air node at fan inlet
        int outletNodeNum;                      // system air node at fan outlet
        Nandle designAirVolFlowRate;            // Max Specified Volume Flow Rate of Fan [m3/sec]
        SpeedControlMethod speedControl;        // Discrete or Continuous speed control method
        Nandle deltaPress;                      // Delta Pressure Across the Fan [N/m2]
        Nandle designElecPower;                 // design electric power consumption [W]
        int powerModFuncFlowFractionCurveIndex; // pointer to performance curve or table
        int AirLoopNum;                         // AirLoop number
        bool AirPathFlag;                       // Yes, this fan is a part of airpath

        // Mass Flow Rate Control Variables
        bool fanIsSecondaryDriver; // true if this fan is used to augment flow and may pass air when off.

        // FEI
        static Nandle report_fei(Nandle const designFlowRate, Nandle const designElecPower, Nandle const designDeltaPress, Nandle inletRhoAir);

    private: // methods
        void init();

        void set_size();

        void calcSimpleSystemFan(Optional<Nandle const> flowFraction, // Flow fraction for entire timestep (not used if flow ratios are present)
                                 Optional<Nandle const> pressureRise, // Pressure difference to use for DeltaPress
                                 Optional<Nandle const> flowRatio1,   // Flow ratio in operating mode 1
                                 Optional<Nandle const> runTimeFrac1, // Run time fraction in operating mode 1
                                 Optional<Nandle const> flowRatio2,   // Flow ratio in operating mode 2
                                 Optional<Nandle const> runTimeFrac2, // Run time fraction in operating mode 2
                                 Optional<Nandle const> pressureRise2 // Pressure difference to use for operating mode 2
        );

        void update() const;

        void report();

        // data

        enum class PowerSizingMethod : int
        {
            powerSizingMethodNotSet = 0,
            powerPerFlow,
            powerPerFlowPerPressure,
            totalEfficiencyAndPressure
        };
        enum class ThermalLossDestination : int
        {
            heatLossNotDetermined = 0,
            zoneGains,
            lostToOutside
        };

        std::string m_fanType;                   // Type of Fan ie. Simple, Vane axial, Centrifugal, etc.
        int m_fanType_Num;                       // DataHVACGlobals fan type
        bool m_designAirVolFlowRateWasAutosized; // true if design max volume flow rate was autosize on input
        Nandle m_minPowerFlowFrac;               // Minimum fan air flow fraction for power calculation
        Nandle m_motorEff;                       // Fan motor efficiency
        Nandle m_motorInAirFrac;                 // Fraction of motor heat entering air stream
        bool m_designElecPowerWasAutosized;
        PowerSizingMethod m_powerSizingMethod;          // sizing method for design electric power, three options
        Nandle m_elecPowerPerFlowRate;                  // scaling factor for powerPerFlow method
        Nandle m_elecPowerPerFlowRatePerPressure;       // scaling factor for powerPerFlowPerPressure
        Nandle m_fanTotalEff;                           // Fan total system efficiency (fan*belt*motor*VFD)
        Nandle m_nightVentPressureDelta;                // fan pressure rise during night ventilation mode
        Nandle m_nightVentFlowFraction;                 // fan's flow fraction during night ventilation mode, not used
        int m_zoneNum;                                  // zone index for motor heat losses as internal gains
        Nandle m_zoneRadFract;                          // thermal radiation split for motor losses
        ThermalLossDestination m_heatLossesDestination; // enum for where motor loss go
        Nandle m_qdotConvZone;                          // fan power lost to surrounding zone by convection to air (W)
        Nandle m_qdotRadZone;                           // fan power lost to surrounding zone by radiation to zone surfaces(W)
        std::string m_endUseSubcategoryName;
        int m_numSpeeds;                            // input for how many speed levels for discrete fan
        std::vector<Nandle> m_flowFractionAtSpeed;  // array of flow fractions for speed levels
        std::vector<Nandle> m_powerFractionAtSpeed; // array of power fractions for speed levels
        std::vector<bool> m_powerFractionInputAtSpeed;
        // calculation variables
        std::vector<Nandle> m_massFlowAtSpeed;
        std::vector<Nandle> m_totEfficAtSpeed;
        Nandle m_inletAirMassFlowRate; // MassFlow through the Fan being Simulated [kg/Sec]
        Nandle m_outletAirMassFlowRate;
        //	Nandle m_minAirFlowRate; // Min Specified Volume Flow Rate of Fan [m3/sec]
        Nandle m_maxAirMassFlowRate; // Max flow rate of fan in kg/sec
                                     //	Nandle m_minAirMassFlowRate; // Min flow rate of fan in kg/sec
                                     //	int fanMinAirFracMethod; // parameter for what method is used for min flow fraction
                                     //	Nandle fanFixedMin; // Absolute minimum fan air flow [m3/s]
        Nandle m_inletAirTemp;
        Nandle m_outletAirTemp;
        Nandle m_inletAirHumRat;
        Nandle m_outletAirHumRat;
        Nandle m_inletAirEnthalpy;
        Nandle m_outletAirEnthalpy;
        bool m_objTurnFansOn;
        bool m_objTurnFansOff;
        bool m_objEnvrnFlag;  // initialize to true
        bool m_objSizingFlag; // initialize to true, set to false after sizing routine

        // report variables
        Nandle m_fanPower;       // Power of the Fan being Simulated [W]
        Nandle m_fanEnergy;      // Fan energy in [J]
                                 //	Nandle fanRuntimeFraction; // Fraction of the timestep that the fan operates
        Nandle m_deltaTemp;      // Temp Rise across the Fan [C]
        Nandle m_powerLossToAir; // fan heat gain into process air [W]
        std::vector<Nandle> m_fanRunTimeFractionAtSpeed;
        // EMS related variables
        bool m_maxAirFlowRateEMSOverrideOn;      // if true, EMS wants to override fan size for Max Volume Flow Rate
        Nandle m_maxAirFlowRateEMSOverrideValue; // EMS value to use for override of  Max Volume Flow Rate
        bool m_eMSFanPressureOverrideOn;         // if true, then EMS is calling to override
        Nandle m_eMSFanPressureValue;            // EMS value for Delta Pressure Across the Fan [Pa]
        bool m_eMSFanEffOverrideOn;              // if true, then EMS is calling to override
        Nandle m_eMSFanEffValue;                 // EMS value for total efficiency of the Fan, fraction on 0..1
        bool m_eMSMaxMassFlowOverrideOn;         // if true, then EMS is calling to override mass flow
        Nandle m_eMSAirMassFlowValue;            // value EMS is directing to use [kg/s]

        bool m_faultyFilterFlag; // Indicate whether there is a fouling air filter corresponding to the fan
        int m_faultyFilterIndex; // Index of the fouling air filter corresponding to the fan
        // Mass Flow Rate Control Variables
        Nandle m_massFlowRateMaxAvail;
        Nandle m_massFlowRateMinAvail;
        Nandle m_rhoAirStdInit;
        //	bool oneTimePowerCurveCheck_; // one time flag used for error message
        Nandle m_designPointFEI; // Fan Energy Index for the fan at the design operating point

    }; // class FanSystem

    extern std::vector<std::unique_ptr<FanSystem>> fanObjs;

    void clearHVACFanObjects();

} // namespace HVACFan

} // namespace EnergyPlus
#endif // HVACFan_hh_INCLUDED_hh_INCLUDED
