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

#ifndef WaterCoils_hh_INCLUDED
#define WaterCoils_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array2A.hh>
#include <ObjexxFCL/Optional.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

namespace WaterCoils {

    // Using/Aliasing

    // Data
    // PRIVATE ! Everything private unless explicitly made public

    // MODULE PARAMETER DEFINITIONS

    extern int const MaxPolynomOrder;
    extern int const MaxOrderedPairs;

    extern Nandle const PolyConvgTol;
    extern Nandle const MinWaterMassFlowFrac;
    extern Nandle const MinAirMassFlow;

    // coil types in this module
    extern int const WaterCoil_SimpleHeating;
    extern int const WaterCoil_DetFlatFinCooling;
    extern int const WaterCoil_Cooling;

    extern int const CoilType_Cooling;
    extern int const CoilType_Heating;

    extern int const CoilModel_Simple;
    extern int const CoilModel_Cooling;
    extern int const CoilModel_Detailed;

    // Parameters for Heat Exchanger Configuration
    extern int const CounterFlow;
    extern int const CrossFlow;
    extern int const SimpleAnalysis;
    extern int const DetailedAnalysis;

    // Water Systems
    extern int const CondensateDiscarded; // default mode where water is "lost"
    extern int const CondensateToTank;    // collect coil condensate from air and store in water storage tank

    // Parameters for COIL:Water:SimpleHeating Coil Performance Input Method
    extern int const UAandFlow; // for Coil Performance Input Method = UA and Design Water Flow Rate
    extern int const NomCap;    // for Coil Performance Input Method = Nominal Capacity

    // Parameters Subroutine CoolingCoil: design calc or simulation calc.
    extern int const DesignCalc; // ignore on/off check in CoolingCoil
    extern int const SimCalc;    // pay attention to on/off check in CoolingCoil

    // DERIVED TYPE DEFINITIONS

    // MODULE VARIABLE DECLARATIONS:
    extern int NumWaterCoils; // The Number of WaterCoils found in the Input
    extern Array1D_bool MySizeFlag;
    extern Array1D_bool MyUAAndFlowCalcFlag;
    extern Array1D_bool MyCoilDesignFlag;
    extern Array1D_bool CoilWarningOnceFlag;
    extern Array1D_int WaterTempCoolCoilErrs; // error counting for detailed coils
    extern Array1D_int PartWetCoolCoilErrs;   // error counting for detailed coils
    extern bool GetWaterCoilsInputFlag;       // Flag set to make sure you get input once
    extern Array1D_bool CheckEquipName;

    // Subroutine Specifications for the Module
    // Driver/Manager Routines

    // Get Input routines for module

    // Initialization routines for module

    // Algorithms for the module

    // Update routine to check convergence and update nodes

    // Reporting routines for module

    // Other routines

    // Types

    struct WaterCoilEquipConditions
    {
        // Members
        std::string Name;            // Name of the WaterCoil
        std::string WaterCoilTypeA;  // Type of WaterCoil ie. Heating or Cooling
        std::string WaterCoilModelA; // Type of WaterCoil ie. Simple, Detailed, etc.
        int WaterCoilType;           // Type of WaterCoil ie. Heating or Cooling
        int WaterCoilModel;          // Type of WaterCoil ie. Simple, Detailed, etc.
        int WaterCoilType_Num;
        std::string Schedule;             // WaterCoil Operation Schedule
        int SchedPtr;                     // Pointer to the correct schedule
        bool RequestingAutoSize;          // True if this coil has appropriate autosize fields
        Nandle InletAirMassFlowRate;      // MassFlow through the WaterCoil being Simulated [kg/s]
        Nandle OutletAirMassFlowRate;     // MassFlow throught the WaterCoil being Simulated[kg/s]
        Nandle InletAirTemp;              // Inlet Air Temperature Operating Condition [C]
        Nandle OutletAirTemp;             // Outlet Air Temperature Operating Condition [C]
        Nandle InletAirHumRat;            // Inlet Air Humidity Ratio Operating Condition
        Nandle OutletAirHumRat;           // Outlet Air Humidity Ratio Calculated Condition
        Nandle InletAirEnthalpy;          // Inlet Air enthalpy [J/kg]
        Nandle OutletAirEnthalpy;         // Outlet Air enthalpy [J/kg]
        Nandle TotWaterCoilLoad;          // Total Load on the Coil [W]
        Nandle SenWaterCoilLoad;          // Sensible Load on the Coil [W]
        Nandle TotWaterHeatingCoilEnergy; // Total Heating Coil energy of the Coil [J]
        Nandle TotWaterCoolingCoilEnergy; // Total Cooling Coil energy of the Coil [J]
        Nandle SenWaterCoolingCoilEnergy; // Sensible Cooling Coil energy of the Coil [J]
        Nandle DesWaterHeatingCoilRate;   // Design Heating Coil Rate used for sizing [W]
        Nandle TotWaterHeatingCoilRate;   // Total Heating Coil Rate on the Coil [W]
        Nandle DesWaterCoolingCoilRate;   // Design Cooling Coil Rate used for sizing [W]
        Nandle TotWaterCoolingCoilRate;   // Total Cooling Coil Rate on the Coil [W]
        Nandle SenWaterCoolingCoilRate;   // Sensible Cooling Coil Rate on the Coil [W]
        Nandle UACoil;                    // WaterCoil UA Value
        Nandle LeavingRelHum;             // Simple Coil Latent Model requires User input for leaving RH
        Nandle DesiredOutletTemp;
        Nandle DesiredOutletHumRat;
        Nandle InletWaterTemp;          // Inlet Water Temperature [C]
        Nandle OutletWaterTemp;         // Outlet Water Temperature [C]
        Nandle InletWaterMassFlowRate;  // Inlet Water Mass Flow Rate [Kg/s]
        Nandle OutletWaterMassFlowRate; // Outlet Water Mass Flow Rate [Kg/s]
        Nandle MaxWaterVolFlowRate;     // Maximum water Volume flow rate [m3/s]
        Nandle MaxWaterMassFlowRate;    // Maximum water mass flow rate [Kg/s]
        Nandle InletWaterEnthalpy;      // Inlet Water Enthalpy
        Nandle OutletWaterEnthalpy;     // Outlet Water Enthalpy
        // These are the additional Geometry and Design Variables for Detailed Flat Fin Coil
        Nandle TubeOutsideSurfArea; // Tube Primary Surface Area
        Nandle TotTubeInsideArea;   // Total Tube inside Surface Area
        Nandle FinSurfArea;         // Fin Surface Area
        Nandle MinAirFlowArea;
        Nandle CoilDepth;
        Nandle FinDiam; // Fin Diameter or the Coil Height
        Nandle FinThickness;
        Nandle TubeInsideDiam;  // Inner diameter of Tubes
        Nandle TubeOutsideDiam; // Outer Diameter of the Tubes
        Nandle TubeThermConductivity;
        Nandle FinThermConductivity;
        Nandle FinSpacing; // Fin Spacing or Distance
        Nandle TubeDepthSpacing;
        int NumOfTubeRows;
        int NumOfTubesPerRow;
        // BEGIN calculated parameters for detailed flat fin coil
        Nandle EffectiveFinDiam;
        Nandle TotCoilOutsideSurfArea;
        Nandle CoilEffectiveInsideDiam;
        Nandle GeometryCoef1;
        Nandle GeometryCoef2;
        Array1D<Nandle> DryFinEfficncyCoef;
        Nandle SatEnthlCurveConstCoef;
        Nandle SatEnthlCurveSlope;
        Nandle EnthVsTempCurveAppxSlope;
        Nandle EnthVsTempCurveConst;
        Nandle MeanWaterTempSaved;
        Nandle InWaterTempSaved;
        Nandle OutWaterTempSaved;
        Nandle SurfAreaWetSaved;
        Nandle SurfAreaWetFraction;
        // END calculated parameters for detailed flat fin coil
        // Design Input Variables to the Design Detailed Simple inputs model
        Nandle DesInletWaterTemp;   // Entering water temperature at Design(C)
        Nandle DesAirVolFlowRate;   // Entering Air Volume Flow Rate Design( m3/s)
        Nandle DesInletAirTemp;     // Entering air dry bulb temperature at Design(C)
        Nandle DesInletAirHumRat;   // Entering air humidity ratio at design conditions
        Nandle DesTotWaterCoilLoad; // Total heat transfer rate at Design(Watt)
        Nandle DesSenWaterCoilLoad; // Sensible heat transfer rate at Design(Watt)
        // BEGIN calculated parameters for Design Detailed Simple inputs model
        Nandle DesAirMassFlowRate;   // Design Air MassFlow through the WaterCoil [kg/Sec]
        Nandle UACoilTotal;          // Overall external dry UA (W/C)
        Nandle UACoilInternal;       // Overall internal UA(W/C)
        Nandle UACoilExternal;       // Overall external heat transfer coefficient(W/C)
        Nandle UACoilInternalDes;    // Overall design internal UA(W/C)
        Nandle UACoilExternalDes;    // Overall design external heat transfer coefficient(W/C)
        Nandle DesOutletAirTemp;     // Leaving air temperature at rating(C)
        Nandle DesOutletAirHumRat;   // Humidity ratio of air leaving at design capacity.
        Nandle DesOutletWaterTemp;   // Temp of Liquid Leaving the Coil at design Capacity
        int HeatExchType;            // Heat exchanger configuration, default to Cross Flow
        int CoolingCoilAnalysisMode; // Mode Of analysis, Simple=1 and Detailed =2
        //    Simple= AllWet-AllDry, Detailed= PartWet-PartDry
        Nandle UACoilInternalPerUnitArea; // Internal overall heat transfer coefficient(W/m2 C)
        Nandle UAWetExtPerUnitArea;       // External overall heat transfer coefficient(W/m2 C)
        Nandle UADryExtPerUnitArea;       // External overall heat transfer coefficient(W/m2 C)
        Nandle SurfAreaWetFractionSaved;  // Previous saved value, for numerical efficiency.
        // END calculated parameters for Design Inputs Detailed coil
        // variables for simple heating coil with variable UA
        Nandle UACoilVariable;                 // WaterCoil UA value when variable (simple heating coil only)
        Nandle RatioAirSideToWaterSideConvect; // "r" value for coil,
        Nandle AirSideNominalConvect;          // nominal rating point air side convection term (fin_effic*(hc*A))
        Nandle LiquidSideNominalConvect;       // nominal rating point water side convection term (hc*A)
        int Control;                           // Const Vol =1;  Variable Vol = 2
        int AirInletNodeNum;
        int AirOutletNodeNum;
        int WaterInletNodeNum;
        int WaterOutletNodeNum;
        int WaterLoopNum;       // Plant loop index
        int WaterLoopSide;      // Plant loop side index
        int WaterLoopBranchNum; // Plant loop branch index
        int WaterLoopCompNum;   // Plant loop Comp index
        // begin variables for Water System interactions
        int CondensateCollectMode;         // where does water come from
        std::string CondensateCollectName; // name of water source e.g. water storage tank
        int CondensateTankID;              // index "pointer" to Storage TAnk array WaterStorage
        int CondensateTankSupplyARRID;     // index pointe to supply Vdot array in WaterStorage
        Nandle CondensateVdot;             // rate of water condensation from air stream [m3/s]
        Nandle CondensateVol;              // amount of water condensed from air stream [m3]
        // end variables for water system interactions
        // COIL:Water:SimpleHeating Coil Performance Input Method
        int CoilPerfInpMeth;            // 1 = UA and Design Water Flow Rate; 2 = Nominal Capacity

        // Operational fault parameters
        bool FaultyCoilFoulingFlag;     // True if the coil has fouling fault
        int FaultyCoilFoulingIndex;     // Index of the fault object corresponding to the coil
        Nandle FaultyCoilFoulingFactor; // Coil fouling factor [m2K/W]
        Nandle OriginalUACoilVariable;
        Nandle OriginalUACoilExternal;
        Nandle OriginalUACoilInternal;

        bool DesiccantRegenerationCoil; // true if it is a regeneration air heating coil defined in Desiccant Dehumidifier system
        int DesiccantDehumNum;          // index to desiccant dehumidifier object
        Nandle DesignWaterDeltaTemp;    // water deltaT for coil sizing [K]
        bool UseDesignWaterDeltaTemp;   // is true, the DesignWaterDeltaTemp is used for sizing coil design water flow rate
        std::string ControllerName;     // controller name used by water coil
        int ControllerIndex;            // controller index used by water coil
        bool reportCoilFinalSizes;      // one time report of sizes to coil summary report
        bool AirLoopDOASFlag;           // True when this coil is used AirLoopDOAS

        // Default Constructor
        WaterCoilEquipConditions()
            : WaterCoilType(0), WaterCoilModel(0), WaterCoilType_Num(0), SchedPtr(0), RequestingAutoSize(false), InletAirMassFlowRate(0.0),
              OutletAirMassFlowRate(0.0), InletAirTemp(0.0), OutletAirTemp(0.0), InletAirHumRat(0.0), OutletAirHumRat(0.0), InletAirEnthalpy(0.0),
              OutletAirEnthalpy(0.0), TotWaterCoilLoad(0.0), SenWaterCoilLoad(0.0), TotWaterHeatingCoilEnergy(0.0), TotWaterCoolingCoilEnergy(0.0),
              SenWaterCoolingCoilEnergy(0.0), DesWaterHeatingCoilRate(0.0), TotWaterHeatingCoilRate(0.0), DesWaterCoolingCoilRate(0.0),
              TotWaterCoolingCoilRate(0.0), SenWaterCoolingCoilRate(0.0), UACoil(0.0), LeavingRelHum(0.0), DesiredOutletTemp(0.0),
              DesiredOutletHumRat(0.0), InletWaterTemp(0.0), OutletWaterTemp(0.0), InletWaterMassFlowRate(0.0), OutletWaterMassFlowRate(0.0),
              MaxWaterVolFlowRate(0.0), MaxWaterMassFlowRate(0.0), InletWaterEnthalpy(0.0), OutletWaterEnthalpy(0.0), TubeOutsideSurfArea(0.0),
              TotTubeInsideArea(0.0), FinSurfArea(0.0), MinAirFlowArea(0.0), CoilDepth(0.0), FinDiam(0.0), FinThickness(0.0), TubeInsideDiam(0.0),
              TubeOutsideDiam(0.0), TubeThermConductivity(0.0), FinThermConductivity(0.0), FinSpacing(0.0), TubeDepthSpacing(0.0), NumOfTubeRows(0),
              NumOfTubesPerRow(0), EffectiveFinDiam(0.0), TotCoilOutsideSurfArea(0.0), CoilEffectiveInsideDiam(0.0), GeometryCoef1(0.0),
              GeometryCoef2(0.0), DryFinEfficncyCoef(5, 0.0), SatEnthlCurveConstCoef(0.0), SatEnthlCurveSlope(0.0), EnthVsTempCurveAppxSlope(0.0),
              EnthVsTempCurveConst(0.0), MeanWaterTempSaved(0.0), InWaterTempSaved(0.0), OutWaterTempSaved(0.0), SurfAreaWetSaved(0.0),
              SurfAreaWetFraction(0.0), DesInletWaterTemp(0.0), DesAirVolFlowRate(0.0), DesInletAirTemp(0.0), DesInletAirHumRat(0.0),
              DesTotWaterCoilLoad(0.0), DesSenWaterCoilLoad(0.0), DesAirMassFlowRate(0.0), UACoilTotal(0.0), UACoilInternal(0.0), UACoilExternal(0.0),
              UACoilInternalDes(0.0), UACoilExternalDes(0.0), DesOutletAirTemp(0.0), DesOutletAirHumRat(0.0), DesOutletWaterTemp(0.0),
              HeatExchType(0), CoolingCoilAnalysisMode(0), UACoilInternalPerUnitArea(0.0), UAWetExtPerUnitArea(0.0), UADryExtPerUnitArea(0.0),
              SurfAreaWetFractionSaved(0.0), UACoilVariable(0.0), RatioAirSideToWaterSideConvect(1.0), AirSideNominalConvect(0.0),
              LiquidSideNominalConvect(0.0), Control(0), AirInletNodeNum(0), AirOutletNodeNum(0), WaterInletNodeNum(0), WaterOutletNodeNum(0),
              WaterLoopNum(0), WaterLoopSide(0), WaterLoopBranchNum(0), WaterLoopCompNum(0), CondensateCollectMode(CondensateDiscarded),
              CondensateTankID(0), CondensateTankSupplyARRID(0), CondensateVdot(0.0), CondensateVol(0.0), CoilPerfInpMeth(0),
              FaultyCoilFoulingFlag(false), FaultyCoilFoulingIndex(0), FaultyCoilFoulingFactor(0.0),
              DesiccantRegenerationCoil(false), DesiccantDehumNum(0), DesignWaterDeltaTemp(0.0), UseDesignWaterDeltaTemp(false), ControllerName(""),
              ControllerIndex(0), reportCoilFinalSizes(true), AirLoopDOASFlag(false)
        {
        }
    };

    struct WaterCoilNumericFieldData
    {
        // Members
        Array1D_string FieldNames;

        // Default Constructor
        WaterCoilNumericFieldData()
        {
        }
    };

    // Object Data
    extern Array1D<WaterCoilEquipConditions> WaterCoil;
    extern Array1D<WaterCoilNumericFieldData> WaterCoilNumericFields;

    // Functions
    void clear_state();

    void SimulateWaterCoilComponents(std::string const &CompName,
                                     bool const FirstHVACIteration,
                                     int &CompIndex,
                                     Optional<Nandle> QActual = _,
                                     Optional_int_const FanOpMode = _,
                                     Optional<Nandle const> PartLoadRatio = _);

    // Get Input Section of the Module
    //******************************************************************************

    void GetWaterCoilInput();

    // End of Get Input subroutines for the HB Module
    //******************************************************************************

    // Beginning Initialization Section of the Module
    //******************************************************************************

    void InitWaterCoil(OutputFiles &outputFiles,
                       int const CoilNum,
                       bool const FirstHVACIteration // unused1208
    );

    void                                   // refactor for coil report
    CalcAdjustedCoilUA(int const CoilNum); // refactor for coil report

    void SizeWaterCoil(int const CoilNum);

    // End Initialization Section of the Module
    //******************************************************************************

    // Begin Algorithm Section of the Module
    //******************************************************************************

    void CalcSimpleHeatingCoil(int const CoilNum,          // index to heating coil
                               int const FanOpMode,        // fan operating mode
                               Nandle const PartLoadRatio, // part-load ratio of heating coil
                               int const CalcMode          // 1 = design calc; 2 = simulation calculation
    );

    void CalcDetailFlatFinCoolingCoil(int const CoilNum,
                                      int const CalcMode,
                                      int const FanOpMode,       // fan operating mode
                                      Nandle const PartLoadRatio // part-load ratio of heating coil
    );

    void CoolingCoil(int const CoilNum,
                     bool const FirstHVACIteration,
                     int const CalcMode,
                     int const FanOpMode,       // fan operating mode
                     Nandle const PartLoadRatio // part-load ratio of heating coil
    );

    // End Algorithm Section of the Module

    // Coil Completely Dry Subroutine for Cooling Coil

    void CoilCompletelyDry(int const CoilNum,
                           Nandle const WaterTempIn,  // Entering water temperature
                           Nandle const AirTempIn,    // Entering air dry bulb temperature
                           Nandle const CoilUA,       // Overall heat transfer coefficient
                           Nandle &OutletWaterTemp,   // Leaving water temperature
                           Nandle &OutletAirTemp,     // Leaving air dry bulb temperature
                           Nandle &OutletAirHumRat,   // Leaving air humidity ratio
                           Nandle &Q,                 // Heat transfer rate
                           int const FanOpMode,       // fan operating mode
                           Nandle const PartLoadRatio // part-load ratio of heating coil
    );

    // Coil Completely Wet Subroutine for Cooling Coil

    void CoilCompletelyWet(int const CoilNum,            // Number of Coil
                           Nandle const WaterTempIn,     // Water temperature IN to this function (C)
                           Nandle const AirTempIn,       // Air dry bulb temperature IN to this function(C)
                           Nandle const AirHumRat,       // Air Humidity Ratio IN to this funcation (C)
                           Nandle const UAInternalTotal, // Internal overall heat transfer coefficient(W/m2 C)
                           Nandle const UAExternalTotal, // External overall heat transfer coefficient(W/m2 C)
                           Nandle &OutletWaterTemp,      // Leaving water temperature (C)
                           Nandle &OutletAirTemp,        // Leaving air dry bulb temperature(C)
                           Nandle &OutletAirHumRat,      // Leaving air humidity ratio
                           Nandle &TotWaterCoilLoad,     // Total heat transfer rate(W)
                           Nandle &SenWaterCoilLoad,     // Sensible heat transfer rate(W)
                           Nandle &SurfAreaWetFraction,  // Fraction of surface area wet
                           Nandle &AirInletCoilSurfTemp, // Surface temperature at air entrance(C)
                           int const FanOpMode,          // fan operating mode
                           Nandle const PartLoadRatio    // part-load ratio of heating coil
    );

    // Coil Part Wet Part Dry Subroutine for Cooling Coil

    void CoilPartWetPartDry(int const CoilNum,             // Number of Coil
                            bool const FirstHVACIteration, // Saving Old values
                            Nandle const InletWaterTemp,   // Entering liquid temperature(C)
                            Nandle const InletAirTemp,     // Entering air dry bulb temperature(C)
                            Nandle const AirDewPointTemp,  // Entering air dew point(C)
                            Nandle &OutletWaterTemp,       // Leaving liquid temperature(C)
                            Nandle &OutletAirTemp,         // Leaving air dry bulb temperature(C)
                            Nandle &OutletAirHumRat,       // Leaving air humidity ratio
                            Nandle &TotWaterCoilLoad,      // Total heat transfer rate (W)
                            Nandle &SenWaterCoilLoad,      // Sensible heat transfer rate (W)
                            Nandle &SurfAreaWetFraction,   // Fraction of surface area wet
                            int const FanOpMode,           // fan operating mode
                            Nandle const PartLoadRatio     // part-load ratio of heating coil
    );

    // Calculating coil UA for Cooling Coil

    Nandle CalcCoilUAbyEffectNTU(int const CoilNum,
                                 Nandle const CapacityStream1,     // Capacity rate of stream1.(W/C)
                                 Nandle const EnergyInStreamOne,   // Inlet state of stream1.(C)
                                 Nandle const CapacityStream2,     // Capacity rate of stream2.(W/C)
                                 Nandle const EnergyInStreamTwo,   // Inlet state of stream2.(C)
                                 Nandle const DesTotalHeatTransfer // Heat transfer rate(W)
    );

    // Calculating coil outlet stream conditions and coil UA for Cooling Coil

    void CoilOutletStreamCondition(int const CoilNum,
                                   Nandle const CapacityStream1,   // Capacity rate of stream1(W/C)
                                   Nandle const EnergyInStreamOne, // Inlet state of stream1 (C)
                                   Nandle const CapacityStream2,   // Capacity rate of stream2 (W/C)
                                   Nandle const EnergyInStreamTwo, // Inlet state of stream2 (C)
                                   Nandle const CoilUA,            // Heat transfer rateW)
                                   Nandle &EnergyOutStreamOne,     // Outlet state of stream1 (C)
                                   Nandle &EnergyOutStreamTwo      // Outlet state of stream2 (C)
    );

    // Subroutine for caculating outlet condition if coil is wet , for Cooling Coil

    void WetCoilOutletCondition(int const CoilNum,
                                Nandle const AirTempIn,      // Entering air dry bulb temperature(C)
                                Nandle const EnthAirInlet,   // Entering air enthalpy(J/kg)
                                Nandle const EnthAirOutlet,  // Leaving air enthalpy(J/kg)
                                Nandle const UACoilExternal, // Heat transfer coefficient for external surface (W/C)
                                Nandle &OutletAirTemp,       // Leaving air dry bulb temperature(C)
                                Nandle &OutletAirHumRat,     // Leaving air humidity ratio
                                Nandle &SenWaterCoilLoad     // Sensible heat transfer rate(W)
    );

    // Beginning of Update subroutines for the WaterCoil Module
    // *****************************************************************************

    void UpdateWaterCoil(int const CoilNum);

    //        End of Update subroutines for the WaterCoil Module
    // *****************************************************************************

    // Beginning of Reporting subroutines for the WaterCoil Module
    // *****************************************************************************

    void ReportWaterCoil(int const CoilNum);

    //        End of Reporting subroutines for the WaterCoil Module
    // *****************************************************************************

    // Beginning of Coil Utility subroutines for the Detailed Model
    // *****************************************************************************

    void CalcDryFinEffCoef(Nandle const OutTubeEffFinDiamRatio, Array1D<Nandle> &PolynomCoef);

    void CalcIBesselFunc(Nandle const BessFuncArg, int const BessFuncOrd, Nandle &IBessFunc, int &ErrorCode);

    void CalcKBesselFunc(Nandle const BessFuncArg, int const BessFuncOrd, Nandle &KBessFunc, int &ErrorCode);

    void CalcPolynomCoef(Array2<Nandle> const &OrderedPair, Array1D<Nandle> &PolynomCoef);

    Nandle SimpleHeatingCoilUAResidual(Nandle const UA,           // UA of coil
                                       Array1D<Nandle> const &Par // par(1) = design coil load [W]
    );

    Nandle SimpleCoolingCoilUAResidual(Nandle const UA,           // UA of coil
                                       Array1D<Nandle> const &Par // par(1) = design coil load [W]
    );

    // Iterate Routine for Cooling Coil

    void CoilAreaFracIter(Nandle &NewSurfAreaWetFrac,       // Out Value of variable
                          Nandle const SurfAreaFracCurrent, // Driver Value
                          Nandle const ErrorCurrent,        // Objective Function
                          Nandle &SurfAreaFracPrevious,     // First Previous value of Surf Area Fraction
                          Nandle &ErrorPrevious,            // First Previous value of error
                          Nandle &SurfAreaFracLast,         // Second Previous value of Surf Area Fraction
                          Nandle &ErrorLast,                // Second Previous value of error
                          int const IterNum,                // Number of Iterations
                          int &icvg                         // Iteration convergence flag
    );

    void CheckWaterCoilSchedule(std::string const &CompType, // unused1208
                                std::string const &CompName,
                                Nandle &Value,
                                int &CompIndex);

    Nandle GetCoilMaxWaterFlowRate(std::string const &CoilType, // must match coil types in this module
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

    int GetCoilWaterInletNode(std::string const &CoilType, // must match coil types in this module
                              std::string const &CoilName, // must match coil names for the coil type
                              bool &ErrorsFound            // set to true if problem
    );

    int GetCoilWaterOutletNode(std::string const &CoilType, // must match coil types in this module
                               std::string const &CoilName, // must match coil names for the coil type
                               bool &ErrorsFound            // set to true if problem
    );

    void SetCoilDesFlow(std::string const &CoilType, // must match coil types in this module
                        std::string const &CoilName, // must match coil names for the coil type
                        Nandle const CoilDesFlow,    // coil volumetric air flow rate [m3/s]
                        bool &ErrorsFound            // set to true if problem
    );

    Nandle GetWaterCoilDesAirFlow(std::string const &CoilType, // must match coil types in this module
                                  std::string const &CoilName, // must match coil names for the coil type
                                  bool &ErrorsFound            // set to true if problem
    );

    void CheckActuatorNode(int const ActuatorNodeNum, // input actuator node number
                           int &iNodeType,            // Cooling or Heating or 0
                           bool &NodeNotFound         // true if matching water inlet node not found
    );

    void CheckForSensorAndSetPointNode(int const SensorNodeNum, // controller sensor node number
                                       int const ControlledVar, // controlled variable type
                                       bool &NodeNotFound       // true if matching air outlet node not found
    );

    Nandle TdbFnHRhPb(Nandle const H,  // specific enthalpy {J/kg}
                      Nandle const RH, // relative humidity value (0.0-1.0)
                      Nandle const PB  // barometric pressure {Pascals}
    );

    Nandle EnthalpyResidual(Nandle const Tprov,        // test value of Tdb [C]
                            Array1D<Nandle> const &Par // Par(1) = desired enthaply H [J/kg]
    );

    Nandle EstimateHEXSurfaceArea(int const CoilNum); // coil number, [-]

    int GetWaterCoilIndex(std::string const &CoilType, // must match coil types in this module
                          std::string const &CoilName, // must match coil names for the coil type
                          bool &ErrorsFound            // set to true if problem
    );

    Nandle GetWaterCoilCapacity(std::string const &CoilType, // must match coil types in this module
                                std::string const &CoilName, // must match coil names for the coil type
                                bool &ErrorsFound            // set to true if problem
    );

    void UpdateWaterToAirCoilPlantConnection(int const CoilTypeNum,
                                             std::string const &CoilName,
                                             int const EquipFlowCtrl, // Flow control mode for the equipment
                                             int const LoopNum,       // Plant loop index for where called from
                                             int const LoopSide,      // Plant loop side index for where called from
                                             int &CompIndex,          // Chiller number pointer
                                             bool const FirstHVACIteration,
                                             bool &InitLoopEquip // If not zero, calculate the max load for operating conditions
    );

    int GetWaterCoilAvailScheduleIndex(std::string const &CoilType, // must match coil types in this module
                                       std::string const &CoilName, // must match coil names for the coil type
                                       bool &ErrorsFound            // set to true if problem
    );

    // sets data to a coil that is used as a regeneration air heating coil in
    // desiccant dehumidification system
    void SetWaterCoilData(int const CoilNum,                           // index of hot water heating Coil
                          bool &ErrorsFound,                           // Set to true if certain errors found
                          Optional_bool DesiccantRegenerationCoil = _, // Flag that this coil is used as regeneration air heating coil
                          Optional_int DesiccantDehumIndex = _         // Index for the desiccant dehum system where this caoil is used
    );

    // estimate heating coil design inlet water temperature for autosizing UA-value
    void EstimateCoilInletWaterTemp(int const CoilNum,                // index to heating coil
                                    int const FanOpMode,              // fan operating mode
                                    Nandle const PartLoadRatio,       // part-load ratio of heating coil
                                    Nandle const UAMax,               // maximum UA-Value
                                    Nandle &DesCoilInletWaterTempUsed // estimated coil design inlet water temperature
    );

    // End of Coil Utility subroutines
    // *****************************************************************************

} // namespace WaterCoils

} // namespace EnergyPlus

#endif
