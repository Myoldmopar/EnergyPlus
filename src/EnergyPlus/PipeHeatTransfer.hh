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

#ifndef PipeHeatTransfer_hh_INCLUDED
#define PipeHeatTransfer_hh_INCLUDED

// C++ Headers
#include <memory>

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>
#include <ObjexxFCL/Array4D.hh>
#include <ObjexxFCL/Optional.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>
#include <EnergyPlus/GroundTemperatureModeling/GroundTemperatureModelManager.hh>
#include <EnergyPlus/PlantComponent.hh>

namespace EnergyPlus {

namespace PipeHeatTransfer {

    // Using/Aliasing
    using namespace GroundTemperatureManager;

    // Data
    // MODULE PARAMETER DEFINITIONS

    extern int const None;
    extern int const ZoneEnv;
    extern int const ScheduleEnv;
    extern int const OutsideAirEnv;
    extern int const GroundEnv;

    extern int const PreviousTimeIndex;
    extern int const CurrentTimeIndex;
    extern int const TentativeTimeIndex;

    extern Nandle const InnerDeltaTime; // one minute time step in seconds

    // DERIVED TYPE DEFINITIONS

    // the model data structures

    // MODULE VARIABLE DECLARATIONS:
    extern int nsvNumOfPipeHT;          // Number of Pipe Heat Transfer objects
    extern int nsvInletNodeNum;         // module variable for inlet node number
    extern int nsvOutletNodeNum;        // module variable for outlet node number
    extern int nsvPipeHTNum;            // object index
    extern Nandle nsvMassFlowRate;      // pipe mass flow rate
    extern Nandle nsvVolumeFlowRate;    // pipe volumetric flow rate
    extern Nandle nsvDeltaTime;         // time change from last update
    extern Nandle nsvInletTemp;         // pipe inlet temperature
    extern Nandle nsvOutletTemp;        // pipe outlet temperature
    extern Nandle nsvEnvironmentTemp;   // environmental temperature (surrounding pipe)
    extern Nandle nsvEnvHeatLossRate;   // heat loss rate from pipe to the environment
    extern Nandle nsvFluidHeatLossRate; // overall heat loss from fluid to pipe
    extern int nsvNumInnerTimeSteps;    // the number of "inner" time steps for our model

    extern bool GetPipeInputFlag; // First time, input is "gotten"

    // SUBROUTINE SPECIFICATIONS FOR MODULE

    // Types

    struct PipeHTData : public PlantComponent
    {

        virtual ~PipeHTData()
        {
        }

        // Members
        // Input data
        std::string Name;
        std::string Construction;    // construction object name
        std::string Environment;     // keyword:  'Schedule', 'OutdoorAir', 'Zone'
        std::string EnvrSchedule;    // temperature schedule for environmental temp
        std::string EnvrVelSchedule; // temperature schedule for environmental temp
        std::string EnvrZone;        // zone providing environmental temp
        std::string EnvrAirNode;     // outside air node providing environmental temp
        Nandle Length;               // total pipe length [m]
        Nandle PipeID;               // pipe inside diameter [m]
        std::string InletNode;       // inlet node name
        std::string OutletNode;      // outlet node name
        int InletNodeNum;            // inlet node number
        int OutletNodeNum;           // outlet node number
        int TypeOf;                  // Type of pipe
        // derived data
        int ConstructionNum; // construction ref number
        int EnvironmentPtr;
        int EnvrSchedPtr;              // pointer to schedule used to set environmental temp
        int EnvrVelSchedPtr;           // pointer to schedule used to set environmental temp
        int EnvrZonePtr;               // pointer to zone number used to set environmental temp
        int EnvrAirNodeNum;            // pointer to outside air node used to set environmental temp
        int NumSections;               // total number of nodes along pipe length
        Nandle FluidSpecHeat;          // fluid Cp [J/kg.K]
        Nandle FluidDensity;           // density [kg/m3]
        Nandle MaxFlowRate;            // max flow rate (from loop/node data)
        Nandle FluidSectionVol;        // volume of each pipe section (node) [m^3]
        Nandle InsideArea;             // pipe section inside surface area [m^2]
        Nandle OutsideArea;            // pipe section outside surface area [m^2]
        Nandle SectionArea;            // cross sectional area [m^2]
        Nandle PipeHeatCapacity;       // heat capacity of pipe section [J/m.K]
        Nandle PipeOD;                 // pipe outside diameter [m]
        Nandle PipeCp;                 // pipe materail Cp [J/kg.K]
        Nandle PipeDensity;            // pipe material density [kg/m3]
        Nandle PipeConductivity;       // pipe material thermal conductivity [W/m.K]
        Nandle InsulationOD;           // insulation outside diameter [m]
        Nandle InsulationCp;           // insulation  specific heat [J/kg.K]
        Nandle InsulationDensity;      // insulation density [kg/m3]
        Nandle InsulationConductivity; // insulation conductivity [W/m.K]
        Nandle InsulationThickness;    // insulation thickness [m]
        Nandle InsulationResistance;   // Insulation thermal resistance [m2.K/W]
        Nandle CurrentSimTime;         // Current simulation time [hr]
        Nandle PreviousSimTime;        // simulation time the report data was last updated
        Array1D<Nandle> TentativeFluidTemp;
        Array1D<Nandle> FluidTemp; // arrays for fluid and pipe temperatures at each node
        Array1D<Nandle> PreviousFluidTemp;
        Array1D<Nandle> TentativePipeTemp;
        Array1D<Nandle> PipeTemp;
        Array1D<Nandle> PreviousPipeTemp;
        int NumDepthNodes;            // number of soil grid points in the depth direction
        int PipeNodeDepth;            // soil depth grid point where pipe is located
        int PipeNodeWidth;            // soil width grid point where pipe is located
        Nandle PipeDepth;             // pipe burial depth [m]
        Nandle DomainDepth;           // soil grid depth [m]
        Nandle dSregular;             // grid spacing in cartesian domain [m]
        Nandle OutdoorConvCoef;       // soil to air convection coefficient [W/m2.K]
        std::string SoilMaterial;     // name of soil material:regular object
        int SoilMaterialNum;          // soil material index in material data structure
        int MonthOfMinSurfTemp;       // month of minimum ground surface temperature
        Nandle MinSurfTemp;           // minimum annual surface temperature [C]
        Nandle SoilDensity;           // density of soil [kg/m3]
        Nandle SoilDepth;             // thickness of soil [m]
        Nandle SoilCp;                // specific heat of soil [J/kg.K]
        Nandle SoilConductivity;      // thermal conductivity of soil [W/m.K]
        Nandle SoilRoughness;         // ground surface roughness
        Nandle SoilThermAbs;          // ground surface thermal absorptivity
        Nandle SoilSolarAbs;          // ground surface solar absorptivity
        Nandle CoefS1;                // soil surface finite difference coefficient
        Nandle CoefS2;                // soil surface finite difference coefficient
        Nandle CoefA1;                // soil finite difference coefficient
        Nandle CoefA2;                // soil finite difference coefficient
        Nandle FourierDS;             // soil Fourier number based on grid spacing
        Nandle SoilDiffusivity;       // soil thermal diffusivity [m2/s]
        Nandle SoilDiffusivityPerDay; // soil thermal diffusivity [m2/day]
        Array4D<Nandle> T;            // soil temperature array
        bool BeginSimInit;            // begin sim and begin environment flag
        bool BeginSimEnvrn;           // begin sim and begin environment flag
        bool FirstHVACupdateFlag;
        bool BeginEnvrnupdateFlag;
        bool SolarExposed;       // Flag to determine if solar is included at ground surface
        Nandle SumTK;            // Sum of thickness/conductivity over all material layers
        Nandle ZoneHeatGainRate; // Lagged energy summation for zone heat gain {W}
        int LoopNum;             // PlantLoop index where this pipe lies
        int LoopSideNum;         // PlantLoop%LoopSide index where this pipe lies
        int BranchNum;           // ..LoopSide%Branch index where this pipe lies
        int CompNum;             // ..Branch%Comp index where this pipe lies
        bool CheckEquipName;
        std::shared_ptr<BaseGroundTempsModel> groundTempModel;
        bool OneTimeInit;

        // Report data
        Nandle FluidInletTemp;          // inlet temperature [C]
        Nandle FluidOutletTemp;         // outlet temperature [C]
        Nandle MassFlowRate;            // mass flow rate [kg/s]
        Nandle FluidHeatLossRate;       // overall heat transfer rate from fluid to pipe [W]
        Nandle FluidHeatLossEnergy;     // energy transferred from fluid to pipe [J]
        Nandle PipeInletTemp;           // pipe temperature at inlet [C]
        Nandle PipeOutletTemp;          // pipe temperature at Oulet [C]
        Nandle EnvironmentHeatLossRate; // overall heat transfer rate from pipe to environment [W]
        Nandle EnvHeatLossEnergy;       // energy transferred from pipe to environment [J]
        Nandle VolumeFlowRate;

        // Default Constructor
        PipeHTData()
            : Length(0.0), PipeID(0.0), InletNodeNum(0), OutletNodeNum(0), TypeOf(0), ConstructionNum(0), EnvironmentPtr(0), EnvrSchedPtr(0),
              EnvrVelSchedPtr(0), EnvrZonePtr(0), EnvrAirNodeNum(0), NumSections(0), FluidSpecHeat(0.0), FluidDensity(0.0), MaxFlowRate(0.0),
              FluidSectionVol(0.0), InsideArea(0.0), OutsideArea(0.0), SectionArea(0.0), PipeHeatCapacity(0.0), PipeOD(0.0), PipeCp(0.0),
              PipeDensity(0.0), PipeConductivity(0.0), InsulationOD(0.0), InsulationCp(0.0), InsulationDensity(0.0), InsulationConductivity(0.0),
              InsulationThickness(0.0), InsulationResistance(0.0), CurrentSimTime(0.0), PreviousSimTime(0.0), NumDepthNodes(0), PipeNodeDepth(0),
              PipeNodeWidth(0), PipeDepth(0.0), DomainDepth(0.0), dSregular(0.0), OutdoorConvCoef(0.0), SoilMaterialNum(0), MonthOfMinSurfTemp(0),
              MinSurfTemp(0.0), SoilDensity(0.0), SoilDepth(0.0), SoilCp(0.0), SoilConductivity(0.0), SoilRoughness(0.0), SoilThermAbs(0.0),
              SoilSolarAbs(0.0), CoefS1(0.0), CoefS2(0.0), CoefA1(0.0), CoefA2(0.0), FourierDS(0.0), SoilDiffusivity(0.0), SoilDiffusivityPerDay(0.0),
              BeginSimInit(true), BeginSimEnvrn(true), FirstHVACupdateFlag(true), BeginEnvrnupdateFlag(true), SolarExposed(true), SumTK(0.0),
              ZoneHeatGainRate(0.0), LoopNum(0), LoopSideNum(0), BranchNum(0), CompNum(0), CheckEquipName(true), OneTimeInit(true),
              FluidInletTemp(0.0), FluidOutletTemp(0.0), MassFlowRate(0.0), FluidHeatLossRate(0.0), FluidHeatLossEnergy(0.0), PipeInletTemp(0.0),
              PipeOutletTemp(0.0), EnvironmentHeatLossRate(0.0), EnvHeatLossEnergy(0.0), VolumeFlowRate(0.0)

        {
        }

        static PlantComponent *factory(int objectType, std::string objectName);

        void clear_state();

        void simulate(const PlantLocation &calledFromLocation, bool const FirstHVACIteration, Nandle &CurLoad, bool const RunFlag) override;

        void PushInnerTimeStepArrays();

        void InitPipesHeatTransfer(bool const FirstHVACIteration // component number
        );

        Nandle TBND(Nandle const z,       // Current Depth
                    Nandle const DayOfSim // Current Simulation Day
        );

        void CalcBuriedPipeSoil();

        void CalcPipesHeatTransfer(Optional_int_const LengthIndex = _);

        Nandle OutsidePipeHeatTransCoef();

        Nandle CalcPipeHeatTransCoef(Nandle const Temperature,  // Temperature of water entering the surface, in C
                                     Nandle const MassFlowRate, // Mass flow rate, in kg/s
                                     Nandle const Diameter      // Pipe diameter, m
        );

        void ReportPipesHeatTransfer(); // Index for the surface under consideration

        void UpdatePipesHeatTransfer();

        void ValidatePipeConstruction(std::string const &PipeType,         // module object of pipe (error messages)
                                      std::string const &ConstructionName, // construction name of pipe (error messages)
                                      std::string const &FieldName,        // fieldname of pipe (error messages)
                                      int const ConstructionNum,           // pointer into construction data
                                      bool &ErrorsFound                    // set to true if errors found here
        );

        static void CalcZonePipesHeatGain();
    };

    // Object Data
    extern Array1D<PipeHTData> PipeHT;

    void GetPipesHeatTransfer();

} // namespace PipeHeatTransfer

} // namespace EnergyPlus

#endif
