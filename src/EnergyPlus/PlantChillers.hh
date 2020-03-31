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

#ifndef PlantChillers_hh_INCLUDED
#define PlantChillers_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>
#include <EnergyPlus/PlantComponent.hh>

namespace EnergyPlus {

namespace PlantChillers {

    // Parameters for use in Chillers
    extern int const AirCooled;
    extern int const WaterCooled;
    extern int const EvapCooled;
    extern Nandle const KJtoJ; // convert Kjoules to joules

    // chiller flow modes
    extern int const FlowModeNotSet;
    extern int const ConstantFlow;
    extern int const NotModulated;
    extern int const LeavingSetPointModulated;

    extern int NumElectricChillers;     // number of Electric chillers specified in input
    extern int NumEngineDrivenChillers; // number of EngineDriven chillers specified in input
    extern int NumGTChillers;           // number of GT chillers specified in input
    extern int NumConstCOPChillers;

    extern bool GetEngineDrivenInput; // then TRUE, calls subroutine to read input file.
    extern bool GetElectricInput;     // then TRUE, calls subroutine to read input file.
    extern bool GetGasTurbineInput;   // then TRUE, calls subroutine to read input file.
    extern bool GetConstCOPInput;

    struct BaseChillerSpecs : PlantComponent // NOTE: This base class is abstract, derived classes must override pure virtual methods
    {
        // Members
        std::string Name;      // user identifier
        Nandle MinPartLoadRat; // (GT MIN) min allowed operating frac full load
        Nandle MaxPartLoadRat; // (GT MAX) max allowed operating frac full load
        Nandle OptPartLoadRat; // (GT BEST) optimal operating frac full load
        Nandle TempDesCondIn;  // C - (GT ADJTC(1)The design secondary loop fluid
        // temperature at the chiller condenser side inlet
        Nandle TempRiseCoef;              // (GT ADJTC(2)) correction factor for off ChillDesign oper.
        Nandle TempDesEvapOut;            // C - (GT ADJTC(3)The design primary loop fluid
        int CondenserType;                // Type of Condenser - Air or Water Cooled
        Nandle NomCap;                    // design nominal capacity of chiller
        bool NomCapWasAutoSized;          // true if NomCap was autosize on input
        Nandle COP;                       // COP
        int FlowMode;                     // one of 3 modes for componet flow during operation
        bool ModulatedFlowSetToLoop;      // True if the setpoint is missing at the outlet node
        bool ModulatedFlowErrDone;        // true if setpoint warning issued
        bool HRSPErrDone;                 // TRUE if set point warning issued for heat recovery loop
        int EvapInletNodeNum;             // Node number on the inlet side of the plant
        int EvapOutletNodeNum;            // Node number on the outlet side of the plant
        int CondInletNodeNum;             // Node number on the inlet side of the condenser
        int CondOutletNodeNum;            // Node number on the outlet side of the condenser
        Nandle EvapVolFlowRate;           // m**3/s - design nominal water volumetric flow rate through the evaporator
        bool EvapVolFlowRateWasAutoSized; // true if autosized design evap flow rate on input
        Nandle EvapMassFlowRateMax;       // kg/s - design water mass flow rate through evaporator
        Nandle CondVolFlowRate;           // m**3/s - design nominal water volumetric flow rate through the condenser
        bool CondVolFlowRateWasAutoSized; // true if previous was autosized
        Nandle CondMassFlowRateMax;       // kg/s - design water mass flow rate through condenser
        int CWLoopNum;                    // chilled water plant loop index number
        int CWLoopSideNum;                // chilled water plant loop side index
        int CWBranchNum;                  // chilled water plant loop branch index
        int CWCompNum;                    // chilled water plant loop component index
        int CDLoopNum;                    // condenser water plant loop index number
        int CDLoopSideNum;                // condenser water plant loop side index
        int CDBranchNum;                  // condenser water plant loop branch index
        int CDCompNum;                    // condenser water plant loop component index
        Nandle SizFac;                    // sizing factor
        Nandle BasinHeaterPowerFTempDiff; // Basin heater capacity per degree C below setpoint (W/C)
        Nandle BasinHeaterSetPointTemp;   // Setpoint temperature for basin heater operation (C)
        int BasinHeaterSchedulePtr;       // Pointer to basin heater schedule
        int ErrCount1;                    // for recurring error messages
        int ErrCount2;                    // for recurring error messages
        std::string MsgBuffer1;           // - buffer to print warning messages on following time step
        std::string MsgBuffer2;           // - buffer to print warning messages on following time step
        Nandle MsgDataLast;               // value of data when warning occurred (passed to Recurring Warn)
        bool PrintMessage;                // logical to determine if message is valid
        int MsgErrorCount;                // number of occurrences of warning
        bool CheckEquipName;
        bool PossibleSubcooling; // flag to indicate chiller is doing less cooling that requested
        int CondMassFlowIndex;
        // Operational fault parameters
        bool FaultyChillerSWTFlag;         // True if the chiller has SWT sensor fault
        int FaultyChillerSWTIndex;         // Index of the fault object corresponding to the chiller
        Nandle FaultyChillerSWTOffset;     // Chiller SWT sensor offset
        bool FaultyChillerFoulingFlag;     // True if the chiller has fouling fault
        int FaultyChillerFoulingIndex;     // Index of the fault object corresponding to the chiller
        Nandle FaultyChillerFoulingFactor; // Chiller fouling factor
        bool MyFlag;
        bool MyEnvrnFlag;
        Nandle TimeStepSysLast;
        Nandle CurrentEndTimeLast;
        Nandle CondMassFlowRate;  // Kg/s - condenser mass flow rate, water side
        Nandle EvapMassFlowRate;  // Kg/s - evaporator mass flow rate, water side
        Nandle CondOutletTemp;    // C - condenser outlet temperature, air or water side
        Nandle EvapOutletTemp;    // C - evaporator outlet temperature, water side
        Nandle QEvaporator;       // W - rate of heat transfer to the evaporator coil
        Nandle QCondenser;        // W - rate of heat transfer to the condenser coil
        Nandle Energy;            // J - chiller energy use
        Nandle EvaporatorEnergy;  // J - rate of heat transfer to the evaporator coil
        Nandle CondenserEnergy;   // J - rate of heat transfer to the condenser coil
        Nandle QHeatRecovered;    // W - rate of heat transfer to the Heat Recovery coil
        Nandle HeatRecOutletTemp; // C - Heat Rec outlet temperature, water side
        Nandle AvgCondSinkTemp;   // condenser temperature value for use in curves [C]
        Nandle BasinHeaterPower;  // Basin heater power (W)
        Nandle Power;
        Nandle CondInletTemp;
        Nandle EvapInletTemp;
        Nandle BasinHeaterConsumption; // Basin heater energy consumption (J)
        int plantTypeOfNum;

        // Default Constructor
        BaseChillerSpecs()
            : MinPartLoadRat(0.0), MaxPartLoadRat(1.0), OptPartLoadRat(1.0), TempDesCondIn(0.0), TempRiseCoef(0.0), TempDesEvapOut(0.0),
              CondenserType(0), NomCap(0.0), NomCapWasAutoSized(false), COP(0.0), FlowMode(FlowModeNotSet), ModulatedFlowSetToLoop(false),
              ModulatedFlowErrDone(false), HRSPErrDone(false), EvapInletNodeNum(0), EvapOutletNodeNum(0), CondInletNodeNum(0), CondOutletNodeNum(0),
              EvapVolFlowRate(0.0), EvapVolFlowRateWasAutoSized(false), EvapMassFlowRateMax(0.0), CondVolFlowRate(0.0),
              CondVolFlowRateWasAutoSized(false), CondMassFlowRateMax(0.0), CWLoopNum(0), CWLoopSideNum(0), CWBranchNum(0), CWCompNum(0),
              CDLoopNum(0), CDLoopSideNum(0), CDBranchNum(0), CDCompNum(0), SizFac(0.0), BasinHeaterPowerFTempDiff(0.0), BasinHeaterSetPointTemp(0.0),
              BasinHeaterSchedulePtr(0), ErrCount1(0), ErrCount2(0), MsgDataLast(0.0), PrintMessage(false), MsgErrorCount(0), CheckEquipName(true),
              PossibleSubcooling(false), CondMassFlowIndex(0), FaultyChillerSWTFlag(false), FaultyChillerSWTIndex(0), FaultyChillerSWTOffset(0.0),
              FaultyChillerFoulingFlag(false), FaultyChillerFoulingIndex(0), FaultyChillerFoulingFactor(1.0), MyFlag(true), MyEnvrnFlag(true),
              TimeStepSysLast(0.0), CurrentEndTimeLast(0.0), CondMassFlowRate(0.0), EvapMassFlowRate(0.0), CondOutletTemp(0.0),
              EvapOutletTemp(0.0),    // C - evaporator outlet temperature, water side
              QEvaporator(0.0),       // W - rate of heat transfer to the evaporator coil
              QCondenser(0.0),        // W - rate of heat transfer to the condenser coil
              Energy(0.0),            // J - chiller energy use
              EvaporatorEnergy(0.0),  // J - rate of heat transfer to the evaporator coil
              CondenserEnergy(0.0),   // J - rate of heat transfer to the condenser coil
              QHeatRecovered(0.0),    // W - rate of heat transfer to the Heat Recovery coil
              HeatRecOutletTemp(0.0), // C - Heat Rec outlet temperature, water side
              AvgCondSinkTemp(0.0),   // condenser temperature value for use in curves [C]
              BasinHeaterPower(0.0),  // Basin heater power (W)
              Power(0.0), CondInletTemp(0.0), EvapInletTemp(0.0), BasinHeaterConsumption(0.0), plantTypeOfNum(0)

        {
        }

        void getDesignCapacities(const PlantLocation &EP_UNUSED(calledFromLocation),
                                 Nandle &EP_UNUSED(MaxLoad),
                                 Nandle &EP_UNUSED(MinLoad),
                                 Nandle &EP_UNUSED(OptLoad)) override;

        void getSizingFactor(Nandle &EP_UNUSED(SizFac)) override;

        void onInitLoopEquip(const PlantLocation &EP_UNUSED(calledFromLocation)) override;

        void getDesignTemperatures(Nandle &EP_UNUSED(TempDesCondIn), Nandle &EP_UNUSED(TempDesEvapOut)) override;

        virtual void initialize(bool RunFlag, Nandle MyLoad) = 0;

        virtual void size() = 0;
    };

    struct ElectricChillerSpecs : BaseChillerSpecs
    {
        // Members
        // temperature at the chiller evaporator side outlet
        Array1D<Nandle> CapRatCoef;                // (Electric RCAVC() ) coeff of cap ratio poly fit
        Array1D<Nandle> PowerRatCoef;              // (Electric ADJEC() ) coeff of power rat poly fit
        Array1D<Nandle> FullLoadCoef;              // (Electric RPWRC() ) coeff of full load poly. fit
        Nandle TempLowLimitEvapOut;                // C - low temperature shut off
        Nandle DesignHeatRecVolFlowRate;           // m3/s, Design Water mass flow rate through heat recovery loop
        bool DesignHeatRecVolFlowRateWasAutoSized; // true if previous was input autosize.
        Nandle DesignHeatRecMassFlowRate;          // kg/s, Design Water mass flow rate through heat recovery loop
        bool HeatRecActive;                        // True entered Heat Rec Vol Flow Rate >0
        int HeatRecInletNodeNum;                   // Node number on the heat recovery inlet side of the condenser
        int HeatRecOutletNodeNum;                  // Node number on the heat recovery outlet side of the condenser
        Nandle HeatRecCapacityFraction;            // user input for heat recovery capacity fraction []
        Nandle HeatRecMaxCapacityLimit;            // Capacity limit for Heat recovery, one time calc [W]
        int HeatRecSetPointNodeNum;                // index for system node with the heat recover leaving setpoint
        int HeatRecInletLimitSchedNum;             // index for schedule for the inlet high limit for heat recovery operation
        int HRLoopNum;                             // heat recovery water plant loop side index
        int HRLoopSideNum;                         // heat recovery water plant loop side index
        int HRBranchNum;                           // heat recovery water plant loop branch index
        int HRCompNum;                             // heat recovery water plant loop component index
        std::string EndUseSubcategory;             // identifier use for the end use subcategory
        Nandle CondOutletHumRat;                   // kg/kg - condenser outlet humditiy ratio, air side
        Nandle ActualCOP;
        Nandle QHeatRecovery;
        Nandle EnergyHeatRecovery;
        Nandle HeatRecInletTemp;
        Nandle HeatRecOutletTemp;
        Nandle HeatRecMdot;
        Nandle ChillerCondAvgTemp; // the effective condenser temperature for chiller performance [C]

        // Default Constructor
        ElectricChillerSpecs()
            : CapRatCoef(3, 0.0), PowerRatCoef(3, 0.0), FullLoadCoef(3, 0.0), TempLowLimitEvapOut(0.0), DesignHeatRecVolFlowRate(0.0),
              DesignHeatRecVolFlowRateWasAutoSized(false), DesignHeatRecMassFlowRate(0.0), HeatRecActive(false), HeatRecInletNodeNum(0),
              HeatRecOutletNodeNum(0), HeatRecCapacityFraction(0.0), HeatRecMaxCapacityLimit(0.0), HeatRecSetPointNodeNum(0),
              HeatRecInletLimitSchedNum(0), HRLoopNum(0), HRLoopSideNum(0), HRBranchNum(0), HRCompNum(0), CondOutletHumRat(0.0), ActualCOP(0.0),
              QHeatRecovery(0.0), EnergyHeatRecovery(0.0), HeatRecInletTemp(0.0), HeatRecOutletTemp(0.0), HeatRecMdot(0.0), ChillerCondAvgTemp(0.0)
        {
        }

        static void getInput();

        void setupOutputVariables();

        static ElectricChillerSpecs *factory(std::string const &chillerName);

        void simulate(const PlantLocation &calledFromLocation, bool FirstHVACIteration, Nandle &CurLoad, bool RunFlag) override;

        void initialize(bool RunFlag, Nandle MyLoad) override;

        void size() override;

        void calculate(Nandle &MyLoad,   // operating load
                       bool RunFlag,     // TRUE when chiller operating
                       int EquipFlowCtrl // Flow control mode for the equipment
        );

        void update(Nandle MyLoad, // current load
                    bool RunFlag   // TRUE if chiller operating
        );

        void calcHeatRecovery(Nandle &QCond,         // current condenser load
                              Nandle CondMassFlow,   // current condenser Mass Flow
                              Nandle _CondInletTemp, // current condenser Inlet Temp
                              Nandle &QHeatRec       // amount of heat recovered
        );
    };

    struct EngineDrivenChillerSpecs : BaseChillerSpecs
    {
        // Members
        std::string FuelType; // Type of Fuel - DIESEL, GASOLINE, GAS
        // temperature at the chiller evaporator side outlet
        Array1D<Nandle> CapRatCoef;                // (EngineDriven RCAVC() ) coeff of cap ratio poly fit
        Array1D<Nandle> PowerRatCoef;              // (EngineDriven ADJEC() ) coeff of power rat poly fit
        Array1D<Nandle> FullLoadCoef;              // (EngineDriven RPWRC() ) coeff of full load poly. fit
        Nandle TempLowLimitEvapOut;                // C - low temperature shut off
        int ClngLoadtoFuelCurve;                   // Coeff of Shaft Power to Fuel Energy Input Coeff Poly Fit
        int RecJacHeattoFuelCurve;                 // Curve Index for Ratio of Recoverable Jacket Heat to
        int RecLubeHeattoFuelCurve;                // Curve Index for Ratio of Recoverable Lube Oil Heat to
        int TotExhausttoFuelCurve;                 // Curve Index for Total Exhaust heat Input to Fuel Energy Input Coeffs Poly Fit
        Nandle ExhaustTemp;                        // (TEXDC) Exhaust Gas Temp to Fuel Energy Input
        int ExhaustTempCurve;                      // Curve Index for Exhaust Gas Temp to Fuel Energy Input Coeffs Poly Fit
        Nandle UA;                                 // (UACDC) exhaust gas Heat Exchanger UA to Capacity
        Array1D<Nandle> UACoef;                    // Heat Exchanger UA Coeffs Poly Fit
        Nandle MaxExhaustperPowerOutput;           // MAX EXHAUST FLOW PER W DSL POWER OUTPUT COEFF
        Nandle DesignMinExitGasTemp;               // Steam Saturation Temperature
        Nandle FuelHeatingValue;                   // Heating Value of Fuel in kJ/kg
        Nandle DesignHeatRecVolFlowRate;           // m3/s, Design Water mass flow rate through heat recovery loop
        bool DesignHeatRecVolFlowRateWasAutoSized; // true if user input was autosize for heat recover design flow rate
        Nandle DesignHeatRecMassFlowRate;          // kg/s, Design Water mass flow rate through heat recovery loop
        bool HeatRecActive;                        // True entered Heat Rec Vol Flow Rate >0
        int HeatRecInletNodeNum;                   // Node number on the heat recovery inlet side of the condenser
        int HeatRecOutletNodeNum;                  // Node number on the heat recovery outlet side of the condenser
        Nandle HeatRecCapacityFraction;            // user input for heat recovery capacity fraction []
        Nandle HeatRecMaxTemp;                     // Max Temp that can be produced in heat recovery
        int HRLoopNum;                             // heat recovery water plant loop side index
        int HRLoopSideNum;                         // heat recovery water plant loop side index
        int HRBranchNum;                           // heat recovery water plant loop branch index
        int HRCompNum;                             // heat recovery water plant loop component index

        // engine driven:
        Nandle HeatRecInletTemp;    // Inlet Temperature of the heat recovery fluid
        Nandle HeatRecMdotActual;   // reporting: Heat Recovery Loop Mass flow rate
        Nandle QTotalHeatRecovered; // total heat recovered (W)
        Nandle QJacketRecovered;    // heat recovered from jacket (W)
        Nandle QLubeOilRecovered;   // heat recovered from lube (W)
        Nandle QExhaustRecovered;   // exhaust gas heat recovered (W)
        Nandle FuelEnergyUseRate;   // Fuel Energy used (W)
        Nandle TotalHeatEnergyRec;  // total heat recovered (J)
        Nandle JacketEnergyRec;     // heat recovered from jacket (J)
        Nandle LubeOilEnergyRec;    // heat recovered from lube (J)
        Nandle ExhaustEnergyRec;    // exhaust gas heat recovered (J)
        Nandle FuelEnergy;          // Fuel Energy used (J)
        Nandle FuelMdot;            // Fuel Amount used (Kg/s)
        Nandle ExhaustStackTemp;    // Exhaust Stack Temperature (C)

        Nandle HeatRecOutletTemp; // reporting: Heat Recovery Loop Outlet Temperature (C)
        Nandle HeatRecMdot;       // reporting: Heat Recovery Loop Mass flow rate (kg/s)
        Nandle FuelCOP;           // reporting: Fuel COP [delivered cooling rate/fuel energy input rate] (W/W)

        // Default Constructor
        EngineDrivenChillerSpecs()
            : CapRatCoef(3, 0.0), PowerRatCoef(3, 0.0), FullLoadCoef(3, 0.0), TempLowLimitEvapOut(0.0), ClngLoadtoFuelCurve(0),
              RecJacHeattoFuelCurve(0), RecLubeHeattoFuelCurve(0), TotExhausttoFuelCurve(0), ExhaustTemp(0.0), ExhaustTempCurve(0), UA(0.0),
              UACoef(2, 0.0), MaxExhaustperPowerOutput(0.0), DesignMinExitGasTemp(0.0), FuelHeatingValue(0.0), DesignHeatRecVolFlowRate(0.0),
              DesignHeatRecVolFlowRateWasAutoSized(false), DesignHeatRecMassFlowRate(0.0), HeatRecActive(false), HeatRecInletNodeNum(0),
              HeatRecOutletNodeNum(0), HeatRecCapacityFraction(0.0), HeatRecMaxTemp(0.0), HRLoopNum(0), HRLoopSideNum(0), HRBranchNum(0),
              HRCompNum(0), HeatRecInletTemp(0.0), HeatRecMdotActual(0.0), QTotalHeatRecovered(0.0), QJacketRecovered(0.0),

              // engine driven:
              QLubeOilRecovered(0.0), QExhaustRecovered(0.0), FuelEnergyUseRate(0.0), TotalHeatEnergyRec(0.0), JacketEnergyRec(0.0),
              LubeOilEnergyRec(0.0), ExhaustEnergyRec(0.0), FuelEnergy(0.0), FuelMdot(0.0), ExhaustStackTemp(0.0), HeatRecOutletTemp(0.0),
              HeatRecMdot(0.0), FuelCOP(0.0)
        {
        }

        static EngineDrivenChillerSpecs *factory(std::string const &chillerName);

        static void getInput();

        void simulate(const PlantLocation &calledFromLocation, bool FirstHVACIteration, Nandle &CurLoad, bool RunFlag) override;

        void setupOutputVariables();

        void initialize(bool RunFlag, Nandle MyLoad) override;

        void size() override;

        void calculate(Nandle &MyLoad,   // operating load
                       bool RunFlag,     // TRUE when chiller operating
                       int EquipFlowCtrl // Flow control mode for the equipment
        );

        void calcHeatRecovery(Nandle EnergyRecovered, // Amount of heat recovered
                              Nandle &HeatRecRatio    // Max Heat recovery ratio
        );

        void update(Nandle MyLoad, // current load
                    bool RunFlag   // TRUE if chiller operating
        );
    };

    struct GTChillerSpecs : BaseChillerSpecs
    {
        // Members
        std::string FuelType;         // Type of Fuel - DIESEL, GASOLINE, GAS
        Array1D<Nandle> CapRatCoef;   // (GT RCAVC() ) coeff of cap ratio poly fit
        Array1D<Nandle> PowerRatCoef; // (GT ADJEC() ) coeff of power rat poly fit
        Array1D<Nandle> FullLoadCoef; // (GT RPWRC() ) coeff of full load poly. fit
        Nandle TempLowLimitEvapOut;   // C - low temperature shut off
        // "special" GT chiller input parameters
        Nandle FuelEnergyIn;                      // (EFUEL) Amount of Fuel Energy Required to run gas turbine
        Array1D<Nandle> PLBasedFuelInputCoef;     // (FUL1GC) Part Load Ratio Based Fuel Input Coefficients Poly Fit
        Array1D<Nandle> TempBasedFuelInputCoef;   // (FUL2GC) Ambient Temperature Based Fuel Input Coeff Poly Fit
        Nandle ExhaustFlow;                       // (FEX) Exhaust Gas Flow Rate cubic meters per second
        Array1D<Nandle> ExhaustFlowCoef;          // (FEXGC) Exhaust Gas Flow Rate Input Coef Poly Fit
        Nandle ExhaustTemp;                       // (TEX) Exhaust Gas Temperature in C
        Array1D<Nandle> PLBasedExhaustTempCoef;   // (TEX1GC) Part Load Ratio Based Exhaust Temperature Input Coeffs Poly Fit
        Array1D<Nandle> TempBasedExhaustTempCoef; // (TEX2GC) Ambient Temperature Based Exhaust Gas Temp to
        // Fuel Energy Input Coeffs Poly Fit
        Nandle HeatRecLubeEnergy;                  // (ELUBE) Recoverable Lube Oil Energy
        Nandle HeatRecLubeRate;                    // (ELUBE) Recoverable Lube Oil Rate of Recovery (W)
        Array1D<Nandle> HeatRecLubeEnergyCoef;     // (ELUBEGC)  Recoverable Lube Oil Energy Input Coef Poly Fit
        Nandle UAtoCapRat;                         // (UACGC) Heat Exchanger UA to Capacity
        Array1D<Nandle> UAtoCapCoef;               // Heat Exchanger UA to Capacity Coeffs Poly Fit
        Nandle GTEngineCapacity;                   // Capacity of GT Unit attached to Chiller
        bool GTEngineCapacityWasAutoSized;         // true if previous field was autosize on inpt
        Nandle MaxExhaustperGTPower;               // Max Exhaust Flow per KW Power Out
        Nandle DesignSteamSatTemp;                 // Steam Saturation Temperature
        Nandle ExhaustStackTemp;                   // Temperature of Exhaust Gases
        int HeatRecInletNodeNum;                   // Node number on the heat recovery inlet side of the condenser
        int HeatRecOutletNodeNum;                  // Node number on the heat recovery outlet side of the condenser
        Nandle HeatRecInletTemp;                   // Inlet Temperature of the heat recovery fluid
        Nandle HeatRecOutletTemp;                  // Outlet Temperature of the heat recovery fluid
        Nandle HeatRecMdot;                        // reporting: Heat Recovery Loop Mass flow rate
        Nandle DesignHeatRecVolFlowRate;           // m3/s, Design Water mass flow rate through heat recovery loop
        bool DesignHeatRecVolFlowRateWasAutoSized; // true if previous field was autosize on input
        Nandle DesignHeatRecMassFlowRate;          // kg/s, Design Water mass flow rate through heat recovery loop
        bool HeatRecActive;                        // True entered Heat Rec Vol Flow Rate >0
        Nandle FuelHeatingValue;                   // Heating Value of Fuel in kJ/kg
        Nandle HeatRecCapacityFraction;            // user input for heat recovery capacity fraction []
        Nandle engineCapacityScalar;               // user input for engine efficiency for sizing GTEngineCapacity []
        Nandle HeatRecMaxTemp;                     // Max Temp that can be produced in heat recovery
        int HRLoopNum;                             // heat recovery water plant loop side index
        int HRLoopSideNum;                         // heat recovery water plant loop side index
        int HRBranchNum;                           // heat recovery water plant loop branch index
        int HRCompNum;                             // heat recovery water plant loop component index

        Nandle FuelEnergyUsed;     // reporting: Fuel Energy used
        Nandle FuelEnergyUsedRate; // reporting: Fuel energy used rate (fuel consumption rate)
        Nandle FuelMassUsed;       // reporting: Fuel Amount used
        Nandle FuelMassUsedRate;   // reporting: Fuel amount used (fuel Mass consumption rate)
        Nandle FuelCOP;            // reporting: Fuel coefficient of performance (Qevap/FuelEnergyUsedRate)

        // Default Constructor
        GTChillerSpecs()
            : CapRatCoef(3, 0.0), PowerRatCoef(3, 0.0), FullLoadCoef(3, 0.0), TempLowLimitEvapOut(0.0), FuelEnergyIn(0.0),
              PLBasedFuelInputCoef(3, 0.0), TempBasedFuelInputCoef(3, 0.0), ExhaustFlow(0.0), ExhaustFlowCoef(3, 0.0), ExhaustTemp(0.0),
              PLBasedExhaustTempCoef(3, 0.0), TempBasedExhaustTempCoef(3, 0.0), HeatRecLubeEnergy(0.0), HeatRecLubeRate(0.0),
              HeatRecLubeEnergyCoef(3, 0.0), UAtoCapRat(0.0), UAtoCapCoef(3, 0.0), GTEngineCapacity(0.0), GTEngineCapacityWasAutoSized(false),
              MaxExhaustperGTPower(0.0), DesignSteamSatTemp(0.0), ExhaustStackTemp(0.0), HeatRecInletNodeNum(0), HeatRecOutletNodeNum(0),
              HeatRecInletTemp(0.0), HeatRecOutletTemp(0.0), HeatRecMdot(0.0), DesignHeatRecVolFlowRate(0.0),
              DesignHeatRecVolFlowRateWasAutoSized(false), DesignHeatRecMassFlowRate(0.0), HeatRecActive(false), FuelHeatingValue(0.0),
              HeatRecCapacityFraction(0.0), engineCapacityScalar(0.35), HeatRecMaxTemp(0.0), HRLoopNum(0), HRLoopSideNum(0), HRBranchNum(0),
              HRCompNum(0), FuelEnergyUsed(0.0), FuelEnergyUsedRate(0.0), FuelMassUsed(0.0), FuelMassUsedRate(0.0), FuelCOP(0.0)
        {
        }

        static GTChillerSpecs *factory(std::string const &chillerName);

        static void getInput();

        void simulate(const PlantLocation &calledFromLocation, bool FirstHVACIteration, Nandle &CurLoad, bool RunFlag) override;

        void setupOutputVariables();

        void initialize(bool RunFlag, Nandle MyLoad) override;

        void size() override;

        void calculate(Nandle &MyLoad,   // operating load
                       bool RunFlag,     // TRUE when chiller operating
                       int EquipFlowCtrl // Flow control mode for the equipment
        );

        void update(Nandle MyLoad, // current load
                    bool RunFlag   // TRUE if chiller operating
        );
    };

    struct ConstCOPChillerSpecs : BaseChillerSpecs
    {
        // Members
        Nandle ActualCOP;

        // Default Constructor
        ConstCOPChillerSpecs() : ActualCOP(0.0)
        {
        }

        static ConstCOPChillerSpecs *factory(std::string const &chillerName);

        static void getInput();

        void simulate(const PlantLocation &calledFromLocation, bool FirstHVACIteration, Nandle &CurLoad, bool RunFlag) override;

        void setupOutputVariables();

        void initialize(bool RunFlag, Nandle MyLoad) override;

        void size() override;

        void calculate(Nandle &MyLoad,
                       bool RunFlag,
                       int EquipFlowCtrl // Flow control mode for the equipment
        );

        void update(Nandle MyLoad, // unused1208
                    bool RunFlag   // unused1208
        );
    };

    // Object Data
    extern Array1D<ElectricChillerSpecs> ElectricChiller;         // dimension to number of machines
    extern Array1D<EngineDrivenChillerSpecs> EngineDrivenChiller; // dimension to number of machines
    extern Array1D<GTChillerSpecs> GTChiller;                     // dimension to number of machines
    extern Array1D<ConstCOPChillerSpecs> ConstCOPChiller;         // dimension to number of machines

    void clear_state();

} // namespace PlantChillers

} // namespace EnergyPlus

#endif
