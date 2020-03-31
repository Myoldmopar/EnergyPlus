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

#ifndef BoilerSteam_hh_INCLUDED
#define BoilerSteam_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>
#include <EnergyPlus/PlantComponent.hh>

namespace EnergyPlus {

namespace BoilerSteam {

    extern int NumBoilers; // Number of boilers

    struct BoilerSpecs : PlantComponent
    {
        // Members
        std::string Name;              // user identifier
        int FuelType;                  // resource type
        bool Available;                // TRUE if machine available in current time step
        bool ON;                       // TRUE: simulate the machine at it's operating part load ratio
        bool MissingSetPointErrDone;   // Missing outlet node setpoint message flag
        bool UseLoopSetPoint;          // Flag to use setpoint from loop
        Nandle DesMassFlowRate;        // kg/s - Boiler water design mass flow rate
        Nandle MassFlowRate;           // kg/s - Boiler water mass flow rate
        Nandle NomCap;                 // W - design nominal capacity of Boiler
        bool NomCapWasAutoSized;       // true if Nominal capacity was autosize on input
        Nandle Effic;                  // boiler efficiency at design conditions
        Nandle MinPartLoadRat;         // Minimum allowed operating part load ratio
        Nandle MaxPartLoadRat;         // Maximum allowed operating part load ratio
        Nandle OptPartLoadRat;         // Optimal operating part load ratio
        Nandle OperPartLoadRat;        // Actual operating part load ratio
        Nandle TempUpLimitBoilerOut;   // C - Boiler outlet maximum temperature limit
        Nandle BoilerMaxOperPress;     // Max Boiler Pressure
        Nandle BoilerPressCheck;       // Boiler Operating Pressure at Saturation Temperature
        Nandle SizFac;                 // sizing factor
        int BoilerInletNodeNum;        // Node number at the boiler inlet
        int BoilerOutletNodeNum;       // Node number at the boiler outlet
        Array1D<Nandle> FullLoadCoef;  // Coefficients of the fuel consumption/part load ratio curve
        int TypeNum;                   // Plant loop type identifier
        int LoopNum;                   // Plant loop index number
        int LoopSideNum;               // Loop side index number
        int BranchNum;                 // Branch index number
        int CompNum;                   // Plant loop component index number
        int PressErrIndex;             // index pointer for recurring errors
        int FluidIndex;                // Steam index
        std::string EndUseSubcategory; // identifier use for the end use subcategory
        bool myFlag;
        bool myEnvrnFlag;

        Nandle FuelUsed;           // W - Boiler fuel used
        Nandle BoilerLoad;         // W - Boiler Load
        Nandle BoilerMassFlowRate; // kg/s - Boiler mass flow rate
        Nandle BoilerOutletTemp;   // W - Boiler outlet temperature

        Nandle BoilerEnergy;    // J - Boiler energy integrated over time
        Nandle FuelConsumed;    // J - Boiler Fuel consumed integrated over time
        Nandle BoilerInletTemp; // C - Boiler inlet temperature

        std::string BoilerFuelTypeForOutputVariable;

        // Default Constructor
        BoilerSpecs()
            : FuelType(0), Available(false), ON(false), MissingSetPointErrDone(false), UseLoopSetPoint(false), DesMassFlowRate(0.0),
              MassFlowRate(0.0), NomCap(0.0), NomCapWasAutoSized(false), Effic(0.0), MinPartLoadRat(0.0), MaxPartLoadRat(0.0), OptPartLoadRat(0.0),
              OperPartLoadRat(0.0), TempUpLimitBoilerOut(0.0), BoilerMaxOperPress(0.0), BoilerPressCheck(0.0), SizFac(0.0), BoilerInletNodeNum(0),
              BoilerOutletNodeNum(0), FullLoadCoef(3, 0.0), TypeNum(0), LoopNum(0), LoopSideNum(0), BranchNum(0), CompNum(0), PressErrIndex(0),
              FluidIndex(0), myFlag(true), myEnvrnFlag(true), FuelUsed(0.0), BoilerLoad(0.0), BoilerMassFlowRate(0.0), BoilerOutletTemp(0.0),
              BoilerEnergy(0.0), FuelConsumed(0.0), BoilerInletTemp(0.0), BoilerFuelTypeForOutputVariable("")
        {
        }

        void initialize();

        void setupOutputVars();

        void autosize();

        void calculate(Nandle &MyLoad,   // W - hot water demand to be met by boiler
                       bool RunFlag,     // TRUE if boiler operating
                       int EquipFlowCtrl // Flow control mode for the equipment
        );

        void update(Nandle MyLoad,          // boiler operating load
                    bool RunFlag,           // boiler on when TRUE
                    bool FirstHVACIteration // TRUE if First iteration of simulation
        );

        void simulate(const PlantLocation &calledFromLocation, bool FirstHVACIteration, Nandle &CurLoad, bool RunFlag) override;

        void getDesignCapacities(const PlantLocation &EP_UNUSED(calledFromLocation), Nandle &MaxLoad, Nandle &MinLoad, Nandle &OptLoad) override;

        void getSizingFactor(Nandle &SizFac) override;

        void onInitLoopEquip(const PlantLocation &EP_UNUSED(calledFromLocation)) override;

        static PlantComponent *factory(std::string const &objectName);
    };

    // Object Data
    extern Array1D<BoilerSpecs> Boiler; // dimension to number of machines

    void clear_state();

    void GetBoilerInput();

} // namespace BoilerSteam

} // namespace EnergyPlus

#endif
