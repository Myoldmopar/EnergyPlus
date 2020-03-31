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

#ifndef HeatPumpWaterToWaterCOOLING_hh_INCLUDED
#define HeatPumpWaterToWaterCOOLING_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>
#include <EnergyPlus/PlantComponent.hh>

namespace EnergyPlus {

namespace HeatPumpWaterToWaterCOOLING {

    // MODULE PARAMETER DEFINITIONS
    extern std::string const ModuleCompName;
    extern std::string const ModuleCompNameUC;

    extern std::string GSHPRefrigerant; // refrigerent name and index
    extern int GSHPRefrigIndex;

    extern int NumGSHPs;                       // number of Gshps specified in input

    struct GshpPeCoolingSpecs : PlantComponent // Needs Some Modifications talk with Dr.Fisher and decide....
    {
        // Members
        std::string Name; // user identifier
        int WWHPPlantTypeOfNum;
        bool Available;                  // need an array of logicals--load identifiers of available equipment
        bool ON;                         // simulate the machine at it's operating part load ratio
        Nandle COP;                      // Coefficeint of Performance of the machine
        Nandle NomCap;                   // Nomial Capcity of the HeatPump
        Nandle MinPartLoadRat;           // Minimum operating Part Load Ratio
        Nandle MaxPartLoadRat;           // Maximum operating Part Load Ratio
        Nandle OptPartLoadRat;           // Optimal operating Part Load Ratio
        Nandle LoadSideVolFlowRate;      // Design Flow Rate on the Load side
        Nandle LoadSideDesignMassFlow;   // Design flow rate (kg/s)
        Nandle SourceSideVolFlowRate;    // Design Flow Rate on th Source Side
        Nandle SourceSideDesignMassFlow; // Design flow rate (kg/s)
        int SourceSideInletNodeNum;      // Node number on the inlet side of the plant
        int SourceSideOutletNodeNum;     // Node number on the outlet side of the plant
        int LoadSideInletNodeNum;        // Node number on the inlet side of the Load Side
        int LoadSideOutletNodeNum;       // Node number on the outlet side of the Load Side
        Nandle SourceSideUACoeff;        // Source Side heat transfer coeff
        Nandle LoadSideUACoeff;          // Load Side heat transfer coeff
        Nandle CompPistonDisp;           // compressor piston displacement
        Nandle CompClearanceFactor;      // compressor clearance factor
        Nandle CompSucPressDrop;         // deltap ,  compressor suction and discharge pressure drop
        Nandle SuperheatTemp;            // deltatsh , super heating
        Nandle PowerLosses;              // constant part of electro mechanical power losses
        Nandle LossFactor;               // loss factor used ot define the electro mechanical loss
        //  that is supposed to be proportional to the theoretical power
        Nandle HighPressCutoff; // Maximum Design Pressure on the Load Side
        Nandle LowPressCutoff;  // Minimum Design Pressure on the Source Side
        // Added by Arun 6-27-02
        // to implement cycletime - removed 9/10/2013 LKL
        bool IsOn;
        bool MustRun;
        // loop topology variables
        int SourceLoopNum;     // source side plant loop index number
        int SourceLoopSideNum; // source side plant loop side index
        int SourceBranchNum;   // source side plant loop branch index
        int SourceCompNum;     // source side plant loop component index
        int LoadLoopNum;       // load side plant loop index number
        int LoadLoopSideNum;   // load side plant loop side index
        int LoadBranchNum;     // load side plant loop branch index
        int LoadCompNum;       // load side plant loop component index
        int CondMassFlowIndex; // index for criteria in PullCompInterconnectTrigger

        // Members
        Nandle Power;                     // Power Consumption Watts
        Nandle Energy;                    // Energy Consumption Joules
        Nandle QLoad;                     // Load Side heat transfer rate Watts
        Nandle QLoadEnergy;               // Load Side heat transfer Joules
        Nandle QSource;                   // Source Side heat transfer rate Watts
        Nandle QSourceEnergy;             // Source Side heat transfer Joules
        Nandle LoadSideWaterInletTemp;    // Load Side outlet temperature °C
        Nandle SourceSideWaterInletTemp;  // Source Side outlet temperature °C
        Nandle LoadSideWaterOutletTemp;   // Load Side outlet temperature °C
        Nandle SourceSideWaterOutletTemp; // Source Side outlet temperature °C
        int Running;                      // On reporting Flag

        Nandle LoadSideWaterMassFlowRate;
        Nandle SourceSideWaterMassFlowRate;

        bool plantScanFlag;
        bool beginEnvironFlag;

        // Default Constructor
        GshpPeCoolingSpecs()
            : WWHPPlantTypeOfNum(0), Available(false), ON(false), COP(0.0), NomCap(0.0), MinPartLoadRat(0.0), MaxPartLoadRat(0.0), OptPartLoadRat(0.0),
              LoadSideVolFlowRate(0.0), LoadSideDesignMassFlow(0.0), SourceSideVolFlowRate(0.0), SourceSideDesignMassFlow(0.0),
              SourceSideInletNodeNum(0), SourceSideOutletNodeNum(0), LoadSideInletNodeNum(0), LoadSideOutletNodeNum(0), SourceSideUACoeff(0.0),
              LoadSideUACoeff(0.0), CompPistonDisp(0.0), CompClearanceFactor(0.0), CompSucPressDrop(0.0), SuperheatTemp(0.0), PowerLosses(0.0),
              LossFactor(0.0), HighPressCutoff(0.0), LowPressCutoff(0.0), IsOn(false), MustRun(false), SourceLoopNum(0), SourceLoopSideNum(0),
              SourceBranchNum(0), SourceCompNum(0), LoadLoopNum(0), LoadLoopSideNum(0), LoadBranchNum(0), LoadCompNum(0), CondMassFlowIndex(0),
              Power(0.0), Energy(0.0), QLoad(0.0), QLoadEnergy(0.0), QSource(0.0), QSourceEnergy(0.0), LoadSideWaterInletTemp(0.0),
              SourceSideWaterInletTemp(0.0), LoadSideWaterOutletTemp(0.0), SourceSideWaterOutletTemp(0.0),
              Running(0), LoadSideWaterMassFlowRate(0.0), SourceSideWaterMassFlowRate(0.0), plantScanFlag(true), beginEnvironFlag(true)
        {
        }

        virtual ~GshpPeCoolingSpecs() = default;

        static PlantComponent *factory(const std::string& objectName);

        void simulate(const PlantLocation &calledFromLocation, bool FirstHVACIteration, Nandle &CurLoad,
                      bool RunFlag) override;

        void getDesignCapacities(const PlantLocation &calledFromLocation,
                                 Nandle &MaxLoad,
                                 Nandle &MinLoad,
                                 Nandle &OptLoad) override;

        void onInitLoopEquip(const PlantLocation &EP_UNUSED(calledFromLocation)) override;

        void initialize();

        void calculate(Nandle &MyLoad);

        void update();
    };

    // Object Data
    extern Array1D<GshpPeCoolingSpecs> GSHP; // dimension to number of machines

    void clear_state();

    void GetGshpInput();

} // namespace HeatPumpWaterToWaterCOOLING

} // namespace EnergyPlus

#endif
