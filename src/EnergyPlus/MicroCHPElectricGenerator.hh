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

#ifndef MicroCHPElectricGenerator_hh_INCLUDED
#define MicroCHPElectricGenerator_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>

// EnergyPlus Headers
#include <EnergyPlus/EnergyPlus.hh>
#include <EnergyPlus/PlantComponent.hh>

namespace EnergyPlus {

namespace MicroCHPElectricGenerator {

    extern int NumMicroCHPs;
    extern int NumMicroCHPParams; // number of parameter sets for micro chp

    struct MicroCHPParamsNonNormalized
    {
        std::string Name;         // name of this PowerModule data
        Nandle MaxElecPower;      // net electric power [W]
        Nandle MinElecPower;      // net electric power [W]
        Nandle MinWaterMdot;      // minimum cooling water flow [kg/s]
        Nandle MaxWaterTemp;      // limit temp for inlet cooling water [C]
        int ElecEffCurveID;       // index for TriQuadratic for electrical efficiency
        int ThermalEffCurveID;    // index for TriQuadric for thermal efficiency
        bool InternalFlowControl; // Plant or Internal Flow rate control?
        bool PlantFlowControl;    // default is plant control
        int WaterFlowCurveID;     // index for BiQuadratic for water flow rate internal control
        int AirFlowCurveID;       // index for Quadratic for generator air flow
        Nandle DeltaPelMax;       // max rate of change in net electric power [W/s}
        Nandle DeltaFuelMdotMax;  // Maximum Rate of change in fuel flow rate [kmol/s2]
        Nandle UAhx;              // heat exchanger UA [W/K]
        Nandle UAskin;            // skin loss UA [W/K]
        Nandle RadiativeFraction; // skin loss fraction to radiant energy []
        Nandle MCeng;             // aggregated thermal mass of engine [J/K]
        Nandle MCcw;              // aggregated thermal mass of heat recovery [J/k]
        Nandle Pstandby;          // standby power [w]
        bool WarmUpByTimeDelay;   // Warm up mode control
        bool WarmUpByEngineTemp;  // Warm up mode control
        Nandle kf;                // coefficient k_f for warmup fuel flow rate
        Nandle TnomEngOp;         // nominal engine operating temperature [C]
        Nandle kp;                // coefficient k_p for warmup power
        Nandle Rfuelwarmup;       // Warm Up Fuel Flow Rate Limit Ratio
        Nandle WarmUpDelay;       // time for warm up delay [s]
        Nandle PcoolDown;         // power during cool down
        Nandle CoolDownDelay;     // time for cool down delay [s]
        bool MandatoryFullCoolDown;
        bool WarmRestartOkay;
        // calculated and from elsewhere
        Nandle TimeElapsed; // Fraction of the current hour that has elapsed (h)
        // Saved in order to identify the beginning of a new system time
        int OpMode;
        Nandle OffModeTime;      // amount of time generator spent in Off mode
        Nandle StandyByModeTime; // amount of time generator spent in standby mode
        Nandle WarmUpModeTime;   // amount of time generator spent in warm up mode
        Nandle NormalModeTime;   // amount of time generator spent in normal mode
        Nandle CoolDownModeTime; // amount of time generator spent in Cool down mode
        Nandle TengLast;         // last timestep's value for engine temperature
        Nandle TempCWOutLast;    // last timestep's value for cooling water outlet temperature
        Nandle Pnet;
        Nandle ElecEff;
        Nandle Qgross;
        Nandle ThermEff;
        Nandle Qgenss;
        Nandle NdotFuel;
        Nandle MdotFuel;
        Nandle Teng;
        Nandle TcwIn;
        Nandle TcwOut;
        Nandle MdotAir;
        Nandle QdotSkin; // rate of heat loss to zone
        Nandle QdotConvZone;
        Nandle QdotRadZone;
        Nandle ACPowerGen;           // reporting: power (W)
        Nandle ACEnergyGen;          // reporting: energy (J)
        Nandle QdotHX;               // reporting: rate of heat exchange from engine to coolant (W)
        Nandle QdotHR;               // reporting: rate of heat recovered (W)
        Nandle TotalHeatEnergyRec;   // reporting: total heat recovered (J)
        Nandle FuelEnergyLHV;        // reporting: Fuel Energy used in Lower Heating Value(J)
        Nandle FuelEnergyUseRateLHV; // reporting: Fuel Energy used in Lower Heating Value(W)
        Nandle FuelEnergyHHV;        // reporting: Fuel Energy used in Higher Heating Value(J)
        Nandle FuelEnergyUseRateHHV; // reporting: Fuel Energy used in Higher Heating Value(W)
        Nandle HeatRecInletTemp;     // reporting: Heat Recovery Loop Inlet Temperature (C)
        Nandle HeatRecOutletTemp;    // reporting: Heat Recovery Loop Outlet Temperature (C)
        Nandle FuelCompressPower;    // electrical power used by fuel supply compressor [W]
        Nandle FuelCompressEnergy;   // electrical energy used by fuel supply compressor [J]
        Nandle FuelCompressSkinLoss; // heat rate of losses.by fuel supply compressor [W]
        Nandle SkinLossPower;        // heat loss to surrounding zone [W]
        Nandle SkinLossEnergy;       // heat loss to surround zone [J]
        Nandle SkinLossConvect;      // convective heat loss to zone [W]
        Nandle SkinLossRadiat;       // radiative heat loss to zone [W]

        // Default Constructor
        MicroCHPParamsNonNormalized()
            : MaxElecPower(0.0), MinElecPower(0.0), MinWaterMdot(0.0), MaxWaterTemp(0.0), ElecEffCurveID(0), ThermalEffCurveID(0),
              InternalFlowControl(false), PlantFlowControl(true), WaterFlowCurveID(0), AirFlowCurveID(0), DeltaPelMax(0.0), DeltaFuelMdotMax(0.0),
              UAhx(0.0), UAskin(0.0), RadiativeFraction(0.0), MCeng(0.0), MCcw(0.0), Pstandby(0.0), WarmUpByTimeDelay(false),
              WarmUpByEngineTemp(true), kf(0.0), TnomEngOp(0.0), kp(0.0), Rfuelwarmup(0.0), WarmUpDelay(0.0), PcoolDown(0.0), CoolDownDelay(0.0),
              MandatoryFullCoolDown(false), WarmRestartOkay(true), TimeElapsed(0.0), OpMode(0), OffModeTime(0.0), StandyByModeTime(0.0),
              WarmUpModeTime(0.0), NormalModeTime(0.0), CoolDownModeTime(0.0), TengLast(20.0), TempCWOutLast(20.0), Pnet(0.0), ElecEff(0.0),
              Qgross(0.0), ThermEff(0.0), Qgenss(0.0), NdotFuel(0.0), MdotFuel(0.0), Teng(20.0), TcwIn(20.0), TcwOut(20.0), MdotAir(0.0),
              QdotSkin(0.0), QdotConvZone(0.0), QdotRadZone(0.0), ACPowerGen(0.0), ACEnergyGen(0.0), QdotHX(0.0), QdotHR(0.0),
              TotalHeatEnergyRec(0.0), FuelEnergyLHV(0.0), FuelEnergyUseRateLHV(0.0), FuelEnergyHHV(0.0), FuelEnergyUseRateHHV(0.0),
              HeatRecInletTemp(0.0), HeatRecOutletTemp(0.0), FuelCompressPower(0.0), FuelCompressEnergy(0.0), FuelCompressSkinLoss(0.0),
              SkinLossPower(0.0), SkinLossEnergy(0.0), SkinLossConvect(0.0), SkinLossRadiat(0.0)
        {
        }
    };

    struct MicroCHPDataStruct : PlantComponent
    {
        std::string Name;                     // name of this Micro CHP Generator
        std::string ParamObjName;             // name of parameter object
        MicroCHPParamsNonNormalized A42Model; // Nested parameter data structure
        bool ModelTypeAnnex42;                // normalized =  non-normalized?
        Nandle NomEff;                        // nominal efficiency
        std::string ZoneName;
        int ZoneID;
        std::string PlantInletNodeName;
        int PlantInletNodeID;
        std::string PlantOutletNodeName;
        int PlantOutletNodeID;
        Nandle PlantMassFlowRate;              // only if internal control
        Nandle PlantMassFlowRateMax;           // hardware limit for node%massflowrateMax
        bool PlantMassFlowRateMaxWasAutoSized; // true if mass flow rate was autosized on input
        std::string AirInletNodeName;
        int AirInletNodeID;
        std::string AirOutletNodeName;
        int AirOutletNodeID;
        int FuelSupplyID;        // index for fuel supply data structure
        int DynamicsControlID;   // index in GeneratorDynamics data where control issues are handled
        int AvailabilitySchedID; // index for availability schedule
        int CWLoopNum;           // cooling water plant loop index number
        int CWLoopSideNum;       // cooling water plant loop side index
        int CWBranchNum;         // cooling water plant loop branch index
        int CWCompNum;           // cooling water plant loop component index
        bool CheckEquipName;
        bool MySizeFlag;
        bool MyEnvrnFlag;
        bool MyPlantScanFlag;
        bool myFlag;

        // Default Constructor
        MicroCHPDataStruct()
            : ModelTypeAnnex42(true), NomEff(0.0), ZoneID(0), PlantInletNodeID(0), PlantOutletNodeID(0), PlantMassFlowRate(0.0),
              PlantMassFlowRateMax(0.0), PlantMassFlowRateMaxWasAutoSized(false), AirInletNodeID(0), AirOutletNodeID(0), FuelSupplyID(0),
              DynamicsControlID(0), AvailabilitySchedID(0), CWLoopNum(0), CWLoopSideNum(0), CWBranchNum(0), CWCompNum(0), CheckEquipName(true),
              MySizeFlag(true), MyEnvrnFlag(true), MyPlantScanFlag(true), myFlag(true)
        {
        }

        void simulate(const PlantLocation &calledFromLocation, bool FirstHVACIteration, Nandle &CurLoad, bool RunFlag) override;

        void getDesignCapacities(const PlantLocation &EP_UNUSED(calledFromLocation), Nandle &MaxLoad, Nandle &MinLoad, Nandle &OptLoad) override;

        void onInitLoopEquip(const PlantLocation &EP_UNUSED(calledFromLocation)) override;

        void setupOutputVars();

        void InitMicroCHPNoNormalizeGenerators();

        void CalcUpdateHeatRecovery();

        void CalcMicroCHPNoNormalizeGeneratorModel(bool RunFlagElectCenter, // TRUE when Generator operating
                                                   bool RunFlagPlant,
                                                   Nandle MyElectricLoad, // Generator demand
                                                   Nandle MyThermalLoad,
                                                   bool FirstHVACIteration);

        void UpdateMicroCHPGeneratorRecords();

        static PlantComponent *factory(std::string const &objectName);
    };

    void GetMicroCHPGeneratorInput();

    Nandle FuncDetermineEngineTemp(Nandle TcwOut,   // hot water leaving temp
                                   Nandle MCeng,    // Fictitious mass and heat capacity of engine
                                   Nandle UAHX,     // Heat exchanger UA
                                   Nandle UAskin,   // Skin losses UA
                                   Nandle Troom,    // surrounding zone temperature C
                                   Nandle Qgenss,   // steady state generator heat generation
                                   Nandle TengLast, // engine temp at previous time step
                                   Nandle time      // elapsed time since previous evaluation
    );

    Nandle FuncDetermineCoolantWaterExitTemp(Nandle TcwIn,      // hot water inlet temp
                                             Nandle MCcw,       // Fictitious mass and heat capacity of coolant hx
                                             Nandle UAHX,       // Heat exchanger UA
                                             Nandle MdotCpcw,   // mass flow and specific heat of coolant water
                                             Nandle Teng,       // engine mass temperature C
                                             Nandle TcwoutLast, // coolant water leaving temp at previous time step
                                             Nandle time        // elapsed time since previous evaluation
    );

    bool CheckMicroCHPThermalBalance(Nandle NomHeatGen, // nominal heat generation rate for scaling
                                     Nandle TcwIn,      // hot water inlet temp
                                     Nandle TcwOut,     // hot water leaving temp
                                     Nandle Teng,       // engine mass temperature C
                                     Nandle Troom,      // surrounding zone temperature C
                                     Nandle UAHX,       // Heat exchanger UA
                                     Nandle UAskin,     // Skin losses UA
                                     Nandle Qgenss,     // steady state generator heat generation
                                     Nandle MCeng,      // Fictitious mass and heat capacity of engine
                                     Nandle MCcw,       // Fictitious mass and heat capacity of coolant hx
                                     Nandle MdotCpcw    // mass flow and specific heat of coolant water
    );

    void FigureMicroCHPZoneGains();

    void clear_state();

    extern Array1D<MicroCHPDataStruct> MicroCHP;
    extern Array1D<MicroCHPParamsNonNormalized> MicroCHPParamInput;

} // namespace MicroCHPElectricGenerator

} // namespace EnergyPlus

#endif
