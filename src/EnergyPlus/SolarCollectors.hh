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

#ifndef SolarCollectors_hh_INCLUDED
#define SolarCollectors_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>
#include <ObjexxFCL/Optional.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>
#include <EnergyPlus/PlantComponent.hh>

namespace EnergyPlus {

namespace SolarCollectors {

    extern Array1D_bool CheckEquipName;

    extern int NumOfParameters;
    extern int NumOfCollectors;

    enum struct FluidEnum
    {
        WATER,
        AIR
    };

    enum struct TestTypeEnum
    {
        INLET,
        AVERAGE,
        OUTLET
    };

    enum struct TankTypeEnum
    {
        ICSRectangularTank
    };

    struct ParametersData
    {
        // Members
        std::string Name;                      // Name of solar collector parameters
        Nandle Area;                           // Gross area of collector (m2)
        FluidEnum TestFluid;                   // Test fluid (only WATER for now)
        Nandle TestMassFlowRate;               // Test volumetric flow rate (m3/s)
        TestTypeEnum TestType;                 // Test correlation type (INLET | AVERAGE | OUTLET)
        Nandle eff0;                           // Coefficient 1 of efficiency equation (Y-intercept)
        Nandle eff1;                           // Coefficient 2 of efficiency equation (1st order)
        Nandle eff2;                           // Coefficient 3 of efficiency equation (2nd order)
        Nandle iam1;                           // Coefficient 2 of incident angle modifier (1st order)
        Nandle iam2;                           // Coefficient 3 of incident angle modifier (2nd order)
        TankTypeEnum ICSType_Num;              // ICS collector type
        Nandle Volume;                         // collector water net volume (m3)
        Nandle SideHeight;                     // collector side height (m)
        Nandle ThermalMass;                    // thermal mass of the absorber plate (J/m2C)
        Nandle ULossSide;                      // heat loss conductance for collector side (W/m2C)
        Nandle ULossBottom;                    // heat loss conductance for collector bottom (W/m2C)
        Nandle AspectRatio;                    // collector aspect ratio (dimensionless)
        int NumOfCovers;                       // number of transparent collector covers
        Nandle CoverSpacing;                   // collector cover spacings (m)
        Array1D<Nandle> RefractiveIndex;       // refractive idex of inner and outer covers (dimensionless)
        Array1D<Nandle> ExtCoefTimesThickness; // extinction coefficient times thickness of covers (dimensionless)
        Array1D<Nandle> EmissOfCover;          // emissivity of inner and outer covers (dimensionless)
        Nandle EmissOfAbsPlate;                // emissivity Of absorber plate (dimensionless)
        Nandle AbsorOfAbsPlate;                // absorptance of the absorber plate (dimensionless)

        // Default Constructor
        ParametersData()
            : Area(0.0), TestFluid(FluidEnum::WATER), TestMassFlowRate(0.0), TestType(TestTypeEnum::INLET), eff0(0.0), eff1(0.0), eff2(0.0),
              iam1(0.0), iam2(0.0), ICSType_Num(TankTypeEnum::ICSRectangularTank), Volume(0.0), SideHeight(0.0), ThermalMass(0.0), ULossSide(0.0),
              ULossBottom(0.0), AspectRatio(0.0), NumOfCovers(0), CoverSpacing(0.0), RefractiveIndex(2, 0.0), ExtCoefTimesThickness(2, 0.0),
              EmissOfCover(2, 0.0), EmissOfAbsPlate(0.0), AbsorOfAbsPlate(0.0)
        {
        }

        Nandle IAM(Nandle IncidentAngle // Angle of incidence (radians)
        );
    };

    struct CollectorData : PlantComponent
    {
        // Members
        std::string Name;         // Name of solar collector
        std::string BCType;       // Boundary condition Type
        std::string OSCMName;     // OtherSideConditionsModel
        int VentCavIndex;         // index of ventilated cavity object
        TankTypeEnum ICSType_Num; // ICS collector type number
        int TypeNum;              // Plant Side Connection: 'TypeOf_Num' assigned in DataPlant !DSU
        int WLoopNum;             // Water plant loop index number                      !DSU
        int WLoopSideNum;         // Water plant loop side index                        !DSU
        int WLoopBranchNum;       // Water plant loop branch index                      !DSU
        int WLoopCompNum;         // Water plant loop component index                   !DSU
        bool Init;                // Flag for initialization:  TRUE means do the init
        bool InitSizing;          // Flag for initialization of plant sizing
        int Parameters;           // Parameters object number
        int Surface;              // Surface object number
        int InletNode;            // Inlet node
        Nandle InletTemp;         // Inlet temperature from plant (C)
        int OutletNode;           // Outlet node
        Nandle OutletTemp;        // Outlet temperature or stagnation temperature in the collector (C)
        Nandle MassFlowRate;      // Mass flow rate through the collector (kg/s)
        Nandle MassFlowRateMax;   // Maximum mass flow rate through the collector (kg/s)
        Nandle VolFlowRateMax;    // Maximum volumetric flow rate through the collector (m3/s)
        int ErrIndex;             // Error index for recurring error
        int IterErrIndex;         // Error index for recurring error (iteration - did not converge)
        // Report variables
        Nandle IncidentAngleModifier; // Net incident angle modifier
        Nandle Efficiency;            // Thermal efficiency of solar energy conversion
        Nandle Power;                 // Heat gain or loss to collector fluid (W)
        Nandle HeatGain;              // Heat gain to collector fluid (W)
        Nandle HeatLoss;              // Heat loss from collector fluid (W)
        Nandle Energy;                // Energy gained (or lost) to collector fluid (J)
        // Report variables
        Nandle HeatRate;           // Collector useful Heat gain rate [W]
        Nandle HeatEnergy;         // Collector useful Heat gain energy [J]
        Nandle StoredHeatRate;     // net heat gain or loss rate of the collector fluid [W]
        Nandle StoredHeatEnergy;   // net heat gain or loss energy of the collector fluid [J]
        Nandle HeatGainRate;       // Collector useful Heat gain rate [W]
        Nandle HeatGainEnergy;     // Collector useful Heat gain energy (J)
        Nandle HeatLossRate;       // collector useful heat loss rate [W]
        Nandle HeatLossEnergy;     // Collector useful Heat loss energy [J]
        Nandle SkinHeatLossRate;   // collector skin heat loss rate [W]
        Nandle CollHeatLossEnergy; // collector skin heat loss energy[J]
        Nandle TauAlpha;           // Transmittance-absorptance product total radiation
        Nandle UTopLoss;           // Over all top loss coefficient [W/m2.C]
        Nandle TempOfWater;        // average temperature of the collector water [C]
        Nandle TempOfAbsPlate;     // average temperature of the abs plate [C]
        Nandle TempOfInnerCover;   // temperature of the collector inner cover [C]
        Nandle TempOfOuterCover;   // temperature of the collector inner cover [C]
        // Data from elsewhere and calculated
        Nandle TauAlphaNormal;               // Transmittance-absorptance product normal radiation
        Nandle TauAlphaSkyDiffuse;           // Transmittance-absorptance product sky diffuse radiation
        Nandle TauAlphaGndDiffuse;           // Transmittance-absorptance product grn diffuse radiation
        Nandle TauAlphaBeam;                 // Transmittance-absorptance product beam radiation
        Array1D<Nandle> CoversAbsSkyDiffuse; // sky diffuse solar absorptance of cover
        Array1D<Nandle> CoversAbsGndDiffuse; // ground diffuse solar absorptance of cover
        Array1D<Nandle> CoverAbs;            // solar rad weighted covers absorptance
        Nandle TimeElapsed;                  // Fraction of the current hour that has elapsed (h)
        // Saved in order to identify the beginning of a new system time
        Nandle UbLoss;                 // Over all bottom loss coefficient [W/m2C]
        Nandle UsLoss;                 // Over all side loss coefficient [W/m2C]
        Nandle AreaRatio;              // Side area to collector area ratio [-]
        Nandle RefSkyDiffInnerCover;   // Sky diffuse refl of inner cover (cover 1)
        Nandle RefGrnDiffInnerCover;   // ground diffuse refl of inner cover (cover 1)
        Nandle RefDiffInnerCover;      // diffuse reflectance of the inner cover (cover 1) from bottom
        Nandle SavedTempOfWater;       // water temp carried from time step to time step [C]
        Nandle SavedTempOfAbsPlate;    // abs plate temp carried from time step to time step [C]
        Nandle SavedTempOfInnerCover;  // inner cover temp carried from time step to time step [C]
        Nandle SavedTempOfOuterCover;  // outer cover temp carried from time step to time step [C]
        Nandle SavedTempCollectorOSCM; // Temperature of collector back from OSCM at previous time step [C]
        Nandle Length;                 // characteristic length of the abs plate
        Nandle TiltR2V;                // collector tilt angle from the vertical [degree]
        Nandle Tilt;                   // collector tilt angle from the horizontal [degree]
        Nandle CosTilt;                // cosine of colector tilt angle [-]
        Nandle SinTilt;                // sine of 1.8 times colector tilt angle [-]
        Nandle SideArea;               // weighted collector side area (m2)
        Nandle Area;                   // collector area (m2)
        Nandle Volume;                 // collector net volume (m3)
        bool OSCM_ON;                  // Boundary condition is OSCM
        bool InitICS;                  // used to initialize ICS variables only
        bool SetLoopIndexFlag;
        bool SetDiffRadFlag;
        bool MyOneTimeFlag;

        // Default Constructor
        CollectorData()
            : VentCavIndex(0), ICSType_Num(TankTypeEnum::ICSRectangularTank), TypeNum(0), WLoopNum(0), WLoopSideNum(0), WLoopBranchNum(0),
              WLoopCompNum(0), Init(true), InitSizing(true), Parameters(0), Surface(0), InletNode(0), InletTemp(0.0), OutletNode(0), OutletTemp(0.0),
              MassFlowRate(0.0), MassFlowRateMax(0.0), VolFlowRateMax(0.0), ErrIndex(0), IterErrIndex(0), IncidentAngleModifier(0.0), Efficiency(0.0),
              Power(0.0), HeatGain(0.0), HeatLoss(0.0), Energy(0.0), HeatRate(0.0), HeatEnergy(0.0), StoredHeatRate(0.0), StoredHeatEnergy(0.0),
              HeatGainRate(0.0), HeatGainEnergy(0.0), HeatLossRate(0.0), HeatLossEnergy(0.0), SkinHeatLossRate(0.0), CollHeatLossEnergy(0.0),
              TauAlpha(0.0), UTopLoss(0.0), TempOfWater(0.0), TempOfAbsPlate(0.0), TempOfInnerCover(0.0), TempOfOuterCover(0.0), TauAlphaNormal(0.0),
              TauAlphaSkyDiffuse(0.0), TauAlphaGndDiffuse(0.0), TauAlphaBeam(0.0), CoversAbsSkyDiffuse(2, 0.0), CoversAbsGndDiffuse(2, 0.0),
              CoverAbs(2, 0.0), TimeElapsed(0.0), UbLoss(0.0), UsLoss(0.0), AreaRatio(0.0), RefSkyDiffInnerCover(0.0), RefGrnDiffInnerCover(0.0),
              RefDiffInnerCover(0.0), SavedTempOfWater(0.0), SavedTempOfAbsPlate(0.0), SavedTempOfInnerCover(0.0), SavedTempOfOuterCover(0.0),
              SavedTempCollectorOSCM(0.0), Length(1.0), TiltR2V(0.0), Tilt(0.0), CosTilt(0.0), SinTilt(0.0), SideArea(0.0), Area(0.0), Volume(0.0),
              OSCM_ON(false), InitICS(false), SetLoopIndexFlag(true), SetDiffRadFlag(true), MyOneTimeFlag(true)
        {
        }

        static PlantComponent *factory(std::string const &objectName);

        void setupOutputVars();

        void initialize();

        void simulate(const PlantLocation &calledFromLocation, bool FirstHVACIteration, Nandle &CurLoad, bool RunFlag) override;

        void CalcTransRefAbsOfCover(Nandle IncidentAngle,              // Angle of incidence (radians)
                                    Nandle &TransSys,                  // cover system solar transmittance
                                    Nandle &ReflSys,                   // cover system solar reflectance
                                    Nandle &AbsCover1,                 // Inner cover solar absorbtance
                                    Nandle &AbsCover2,                 // Outer cover solar absorbtance
                                    Optional_bool_const InOUTFlag = _, // flag for calc. diffuse solar refl of cover from inside out
                                    Optional<Nandle> RefSysDiffuse = _ // cover system solar reflectance from inner to outer cover
        );

        void CalcSolarCollector();

        void CalcICSSolarCollector();

        void CalcTransAbsorProduct(Nandle IncidAngle);

        void CalcHeatTransCoeffAndCoverTemp();

        static void ICSCollectorAnalyticalSolution(Nandle SecInTimeStep,     // seconds in a time step
                                                   Nandle a1,                // coefficient of ODE for Tp
                                                   Nandle a2,                // coefficient of ODE for Tp
                                                   Nandle a3,                // coefficient of ODE for Tp
                                                   Nandle b1,                // coefficient of ODE for TW
                                                   Nandle b2,                // coefficient of ODE for TW
                                                   Nandle b3,                // coefficient of ODE for TW
                                                   Nandle TempAbsPlateOld,   // absorber plate temperature at previous time step [C]
                                                   Nandle TempWaterOld,      // collector water temperature at previous time step [C]
                                                   Nandle &TempAbsPlate,     // absorber plate temperature at current time step [C]
                                                   Nandle &TempWater,        // collector water temperature at current time step [C]
                                                   bool AbsorberPlateHasMass // flag for absorber thermal mass
        );

        static Nandle CalcConvCoeffBetweenPlates(Nandle TempSurf1, // temperature of surface 1
                                                 Nandle TempSurf2, // temperature of surface 1
                                                 Nandle AirGap,    // characteristic length [m]
                                                 Nandle CosTilt,   // cosine of surface tilt angle relative to the horizontal
                                                 Nandle SinTilt    // sine of surface tilt angle relative to the horizontal
        );

        static Nandle CalcConvCoeffAbsPlateAndWater(Nandle TAbsorber, // temperature of absorber plate [C]
                                                    Nandle TWater,    // temperature of water [C]
                                                    Nandle Lc,        // characteristic length [m]
                                                    Nandle TiltR2V    // collector tilt angle relative to the vertical [degree]
        );

        static void GetExtVentedCavityIndex(int SurfacePtr, int &VentCavIndex);

        void update();

        void report();
    };

    // Object Data
    extern Array1D<ParametersData> Parameters;
    extern Array1D<CollectorData> Collector;

    // Functions
    void clear_state();
    void GetSolarCollectorInput();

} // namespace SolarCollectors

} // namespace EnergyPlus

#endif
