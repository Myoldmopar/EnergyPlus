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

#ifndef SurfaceGroundHeatExchanger_hh_INCLUDED
#define SurfaceGroundHeatExchanger_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>
#include <EnergyPlus/PlantComponent.hh>

namespace EnergyPlus {

namespace SurfaceGroundHeatExchanger {

    // Using/Aliasing

    // Data
    // MODULE PARAMETER DEFINITIONS
    extern Nandle const SmallNum;        // Very small number to avoid div0 errors
    extern Nandle const StefBoltzmann;   // Stefan-Boltzmann constant
    extern Nandle const SurfaceHXHeight; // Surface Height above ground -- used in height dependent calcs.

    extern int const SurfCond_Ground;
    extern int const SurfCond_Exposed;

    namespace loc {
        extern int const MaxCTFTerms; // Maximum number of CTF terms allowed to still allow stability //Note Duplicate of DataHeatBalance::MaxCTFTerms
                                      // to avoid static initialization order bug: Keep them in sync
    }                                 // namespace loc

    // DERIVED TYPE DEFINITIONS

    // MODULE VARIABLE DECLARATIONS:
    // utility variables initialized once
    // extern int NumOfSurfaceGHEs; // Number of surface GHE ground heat exchangers
    // extern bool NoSurfaceGroundTempObjWarning; // This will cause a warning to be issued if no "surface" ground
    //// temperature object was input.
    //// Utility variables - initialized for each instance of a surface GHE
    // extern int InletNodeNum; // inlet node number
    // extern int OutletNodeNum; // oulet node number
    // extern int ConstructionNum; // construction index number
    // extern int TopRoughness; // roughness of top layer
    // extern int BtmRoughness; // roughness of bottom layer
    extern Nandle nsvInletTemp;  // water inlet temperature
    extern Nandle nsvOutletTemp; // water outlet temperature
    extern Nandle FlowRate;      // water mass flow rate
    extern Nandle TopSurfTemp;   // Top  surface temperature
    extern Nandle BtmSurfTemp;   // Bottom  surface temperature
    extern Nandle TopSurfFlux;   // Top  surface heat flux
    extern Nandle BtmSurfFlux;   // Bottom  surface heat flux
    extern Nandle SourceFlux;    // total heat transfer rate, Watts
    extern Nandle SourceTemp;    // total heat transfer rate, Watts
    extern Nandle TopThermAbs;   // Thermal absortivity of top layer
    extern Nandle BtmThermAbs;   // Thermal absortivity of bottom layer
    extern Nandle TopSolarAbs;   // Solar absortivity of top layer
    extern Array1D_bool CheckEquipName;

    // weather data records updated every zone time step
    extern Nandle PastBeamSolarRad;    // Previous beam normal solar irradiance
    extern Nandle PastSolarDirCosVert; // Previous vertical component of solar normal
    extern Nandle PastDifSolarRad;     // Previous sky diffuse solar horizontal irradiance
    extern Nandle PastGroundTemp;      // Previous ground temperature
    extern bool PastIsRain;            // Previous Surfaces are wet for this time interval
    extern bool PastIsSnow;            // Previous Snow on the ground for this time interval
    extern Nandle PastOutBaroPress;    // Previous outdoor air barometric pressure
    extern Nandle PastOutDryBulbTemp;  // Previous outdoor air dry bulb temperature
    extern Nandle PastOutHumRat;       // Previous outdoor air humidity ratio
    extern Nandle PastOutAirDensity;   // Previous outdoor air density
    extern Nandle PastOutWetBulbTemp;  // Previous outdoor air wet bulb temperature
    extern Nandle PastOutDewPointTemp; // Previous outdoor dewpoint temperature
    extern Nandle PastSkyTemp;         // Previous sky temperature
    extern Nandle PastWindSpeed;       // Previous outdoor air wind speed
    extern Nandle PastCloudFraction;   // Previous Fraction of sky covered by clouds

    // get input flag
    extern bool GetInputFlag;

    // time keeping variables used for keeping track of average flux over each time step
    extern Array1D<Nandle> QRadSysSrcAvg;      // Average source over the time step
    extern Array1D<Nandle> LastSysTimeElapsed; // record of system time
    extern Array1D<Nandle> LastTimeStepSys;    // previous time step size

    // SUBROUTINE SPECIFICATIONS FOR MODULE PlantSurfaceGroundHeatExchangers

    // Types

    struct SurfaceGroundHeatExchangerData : PlantComponent
    {

        virtual ~SurfaceGroundHeatExchangerData()
        {
        }

        // Members
        // Input data
        std::string Name;             // name of surface GHE
        std::string ConstructionName; // name of the associated construction
        std::string InletNode;        // surface GHE inlet fluid node
        std::string OutletNode;       // surface GHE outlet fluid node
        Nandle DesignMassFlowRate;
        Nandle TubeDiameter;  // hydronic tube inside diameter
        Nandle TubeSpacing;   // tube spacing
        Nandle SurfaceLength; // active length of surface GHE
        Nandle SurfaceWidth;  // active width of surface GHE
        Nandle TopThermAbs;   // Thermal absortivity of top layer
        Nandle TopSolarAbs;   // solar absortivity of top layer
        Nandle BtmThermAbs;   // Thermal absortivity of bottom layer
        int LowerSurfCond;    // Type of lower surf. boundary condition
        int TubeCircuits;     // number of circuits in total
        int ConstructionNum;  // construction index number
        int InletNodeNum;     // inlet node number
        int OutletNodeNum;    // oulet node number
        int TopRoughness;     // roughness of top layer
        int BtmRoughness;     // roughness of bottom layer
        int FrozenErrIndex1;  // recurring error index
        int FrozenErrIndex2;  // recurring error index
        int ConvErrIndex1;    // recurring error index
        int ConvErrIndex2;    // recurring error index
        int ConvErrIndex3;    // recurring error index
        // loop topology variables
        int LoopNum;
        int LoopSideNum;
        int BranchNum;
        int CompNum;

        // QTF Constants
        Nandle TsrcConstCoef;
        Nandle TsrcVarCoef;
        Nandle QbtmConstCoef;
        Nandle QbtmVarCoef;
        Nandle QtopConstCoef;
        Nandle QtopVarCoef;
        // conventional CTF terms
        int NumCTFTerms; // number of terms for surface
        // could be allocated rather than hard dimensioning.
        Array1D<Nandle> CTFin;    // surf flux in ctf - X
        Array1D<Nandle> CTFout;   // surf flux in ctf - Z
        Array1D<Nandle> CTFcross; // surf flux in ctf - Y
        Array1D<Nandle> CTFflux;  // surf flux in ctf - F
        // QTF coefficients
        Array1D<Nandle> CTFSourceIn;   // surf flux in ctf - Wi
        Array1D<Nandle> CTFSourceOut;  // surf flux out ctf - Wo
        Array1D<Nandle> CTFTSourceOut; // surf flux in qtf - x
        Array1D<Nandle> CTFTSourceIn;  // surf flux in qtf - y
        Array1D<Nandle> CTFTSourceQ;   // surf flux in qtf - f
        // History data
        Array1D<Nandle> TbtmHistory;
        Array1D<Nandle> TtopHistory;
        Array1D<Nandle> TsrcHistory;
        Array1D<Nandle> QbtmHistory;
        Array1D<Nandle> QtopHistory;
        Array1D<Nandle> QsrcHistory;
        Nandle QSrc;
        Nandle QSrcAvg;
        Nandle LastQSrc;
        Nandle LastSysTimeElapsed;
        Nandle LastTimeStepSys;

        // Report data
        Nandle InletTemp;            // water inlet temperature
        Nandle OutletTemp;           // water outlet temperature
        Nandle MassFlowRate;         // water mass flow rate
        Nandle TopSurfaceTemp;       // Top surface temperature
        Nandle BtmSurfaceTemp;       // Bottom  surface temperature
        Nandle TopSurfaceFlux;       // Top  surface heat flux
        Nandle BtmSurfaceFlux;       // Bottom  surface heat flux
        Nandle HeatTransferRate;     // total fluid heat transfer rate, Watts
        Nandle SurfHeatTransferRate; // total surface heat transfer rate, Watts
        Nandle Energy;               // cumulative energy, Joules
        Nandle SurfEnergy;           // cumulative energy, Joules
        Nandle SourceTemp;           // Source temperature

        bool MyFlag;
        bool InitQTF;
        bool MyEnvrnFlag;
        Nandle SurfaceArea; // surface GHE surface area

        // Default Constructor
        SurfaceGroundHeatExchangerData()
            : DesignMassFlowRate(0.0), TubeDiameter(0.0), TubeSpacing(0.0), SurfaceLength(0.0), SurfaceWidth(0.0), TopThermAbs(0.0), TopSolarAbs(0.0),
              BtmThermAbs(0.0), LowerSurfCond(0), TubeCircuits(0), ConstructionNum(0), InletNodeNum(0), OutletNodeNum(0), TopRoughness(0),
              BtmRoughness(0), FrozenErrIndex1(0), FrozenErrIndex2(0), ConvErrIndex1(0), ConvErrIndex2(0), ConvErrIndex3(0), LoopNum(0),
              LoopSideNum(0), BranchNum(0), CompNum(0),

              TsrcConstCoef(0.0), TsrcVarCoef(0.0), QbtmConstCoef(0.0), QbtmVarCoef(0.0), QtopConstCoef(0.0), QtopVarCoef(0.0), NumCTFTerms(0),
              CTFin({0, loc::MaxCTFTerms - 1}, 0.0), CTFout({0, loc::MaxCTFTerms - 1}, 0.0), CTFcross({0, loc::MaxCTFTerms - 1}, 0.0),
              CTFflux({0, loc::MaxCTFTerms - 1}, 0.0), CTFSourceIn({0, loc::MaxCTFTerms - 1}, 0.0), CTFSourceOut({0, loc::MaxCTFTerms - 1}, 0.0),
              CTFTSourceOut({0, loc::MaxCTFTerms - 1}, 0.0), CTFTSourceIn({0, loc::MaxCTFTerms - 1}, 0.0),
              CTFTSourceQ({0, loc::MaxCTFTerms - 1}, 0.0), TbtmHistory({0, loc::MaxCTFTerms - 1}, 0.0), TtopHistory({0, loc::MaxCTFTerms - 1}, 0.0),
              TsrcHistory({0, loc::MaxCTFTerms - 1}, 0.0), QbtmHistory({0, loc::MaxCTFTerms - 1}, 0.0), QtopHistory({0, loc::MaxCTFTerms - 1}, 0.0),
              QsrcHistory({0, loc::MaxCTFTerms - 1}, 0.0), QSrc(0.0), QSrcAvg(0.0), LastQSrc(0.0), LastSysTimeElapsed(0.0), LastTimeStepSys(0.0),

              InletTemp(0.0), OutletTemp(0.0), MassFlowRate(0.0), TopSurfaceTemp(0.0), BtmSurfaceTemp(0.0), TopSurfaceFlux(0.0), BtmSurfaceFlux(0.0),
              HeatTransferRate(0.0), SurfHeatTransferRate(0.0), Energy(0.0), SurfEnergy(0.0), SourceTemp(0.0),

              MyFlag(true), InitQTF(true), MyEnvrnFlag(true), SurfaceArea(0.0)
        {
        }

        void simulate(const PlantLocation &calledFromLocation, bool const FirstHVACIteration, Nandle &CurLoad, bool const RunFlag) override;

        static PlantComponent *factory(int const objectType, std::string const objectName);

        void InitSurfaceGroundHeatExchanger();

        //==============================================================================

        void CalcSurfaceGroundHeatExchanger(bool const FirstHVACIteration // TRUE if 1st HVAC simulation of system timestep
        );

        //==============================================================================

        void CalcBottomFluxCoefficents(Nandle const Tbottom, // current bottom (lower) surface temperature
                                       Nandle const Ttop     // current top (upper) surface temperature
        );

        //==============================================================================

        void CalcTopFluxCoefficents(Nandle const Tbottom, // current bottom (lower) surface temperature
                                    Nandle const Ttop     // current top (upper) surface temperature
        );

        //==============================================================================

        void CalcSourceTempCoefficents(Nandle const Tbottom, // current bottom (lower) surface temperature
                                       Nandle const Ttop     // current top (upper) surface temperature
        );

        //==============================================================================

        Nandle CalcSourceFlux(); // component number

        //==============================================================================

        void UpdateHistories(Nandle const TopFlux,    // current top (top) surface flux
                             Nandle const BottomFlux, // current bottom (bottom) surface flux
                             Nandle const SourceFlux, // current source surface flux
                             Nandle const SourceTemp  // current source temperature
        );

        //==============================================================================

        Nandle CalcHXEffectTerm(Nandle const Temperature,  // Temperature of water entering the surface, in C
                                Nandle const WaterMassFlow // Mass flow rate, in kg/s
        );

        //==============================================================================

        void CalcTopSurfTemp(Nandle const FluxTop,             // top surface flux
                             Nandle &TempTop,                  // top surface temperature
                             Nandle const ThisDryBulb,         // dry bulb temperature
                             Nandle const ThisWetBulb,         // wet bulb temperature
                             Nandle const ThisSkyTemp,         // sky temperature
                             Nandle const ThisBeamSolarRad,    // beam solar radiation
                             Nandle const ThisDifSolarRad,     // diffuse solar radiation
                             Nandle const ThisSolarDirCosVert, // vertical component of solar normal
                             Nandle const ThisWindSpeed,       // wind speed
                             bool const ThisIsRain,            // rain flag
                             bool const ThisIsSnow             // snow flag
        );

        //==============================================================================

        void CalcBottomSurfTemp(Nandle const FluxBtm,       // bottom surface flux
                                Nandle &TempBtm,            // bottom surface temperature
                                Nandle const ThisDryBulb,   // dry bulb temperature
                                Nandle const ThisWindSpeed, // wind speed
                                Nandle const ThisGroundTemp // ground temperature
        );

        //==============================================================================

        void UpdateSurfaceGroundHeatExchngr(); // Index for the surface

        //==============================================================================

        void ReportSurfaceGroundHeatExchngr(); // Index for the surface under consideration
    };

    // Object Data
    extern Array1D<SurfaceGroundHeatExchangerData> SurfaceGHE;

    void GetSurfaceGroundHeatExchanger();

    //==============================================================================

} // namespace SurfaceGroundHeatExchanger

} // namespace EnergyPlus

#endif
