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

#ifndef HVACCooledBeam_hh_INCLUDED
#define HVACCooledBeam_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

namespace HVACCooledBeam {

    // Using/Aliasing

    // Data
    // MODULE PARAMETER DEFINITIONS:
    extern int const Passive_Cooled_Beam;
    extern int const Active_Cooled_Beam;
    extern Nandle const NomMassFlowPerBeam; // nominal water mass flow rate per beam [kg/s]
    extern Nandle const MinWaterVel;        // minimum water velocity [m/s]
    extern Nandle const Coeff2;
    // DERIVED TYPE DEFINITIONS:

    // MODULE VARIABLE DECLARATIONS:
    extern Array1D_bool CheckEquipName;

    // INTEGER :: NumPassiveCB = 0
    // INTEGER :: NumActiveCB = 0
    extern int NumCB;

    // SUBROUTINE SPECIFICATIONS FOR MODULE HVACCooledBeam:

    // Types

    struct CoolBeamData
    {
        // Members
        // input data
        std::string Name;            // name of unit
        std::string UnitType;        // type of unit = AirTerminal:SingleDuct:ConstantVolume:CooledBeam
        int UnitType_Num;            // index to type of unit = 1 (there's only 1 type so far)
        std::string CBType;          // type of cooled beam: active | passive
        int CBType_Num;              // index to type of cooled beam: passive=1; active=2
        std::string Sched;           // availability schedule
        int SchedPtr;                // index to schedule
        Nandle MaxAirVolFlow;        // m3/s (autosizable)
        Nandle MaxAirMassFlow;       // kg/s
        Nandle MaxCoolWaterVolFlow;  // m3/s
        Nandle MaxCoolWaterMassFlow; // kg/s
        int AirInNode;               // unit air inlet node number
        int AirOutNode;              // unit air outlet node number
        int CWInNode;                // chilled water inlet node
        int CWOutNode;               // chilled water outlet node
        int ADUNum;                  // index of corresponding air distribution unit
        Nandle NumBeams;             // number of beams in the zone
        Nandle BeamLength;           // length of individual beam [m]
        Nandle DesInletWaterTemp;    // design inlet water temperature [C]
        Nandle DesOutletWaterTemp;   // design outlet water Temperature [c]
        Nandle CoilArea;             // coil surface area per coil length [m2/m]
        Nandle a;                    // model parameter a
        Nandle n1;                   // model parameter n0
        Nandle n2;                   // model parameter n1
        Nandle n3;                   // model parameter n2
        Nandle a0;                   // model parameter a0
        Nandle K1;                   // model parameter K1
        Nandle n;                    // model parameter n
        Nandle Kin;                  // Coefficient of Induction Kin
        Nandle InDiam;               // Leaving Pipe Inside Diameter
        // time step variables
        Nandle TWIn;                // current inlet water temperature [C]
        Nandle TWOut;               // current outlet water temperature [C]
        Nandle EnthWaterOut;        // current outlet water enthalpy [J/kg]
        Nandle BeamFlow;            // supply air flow per beam [m3/s]
        Nandle CoolWaterMassFlow;   // chilled water mass flow rate [kg/s]
        Nandle BeamCoolingEnergy;   // Cooled beam cooling energy of all beams in the zone [J]
        Nandle BeamCoolingRate;     // Cooled beam cooling rate of all beams in the zone [W]
        Nandle SupAirCoolingEnergy; // Total cooling energy from supply air [J]
        Nandle SupAirCoolingRate;   // Total cooling rate from supply air [W]
        Nandle SupAirHeatingEnergy; // Total cooling energy from supply air [J]
        Nandle SupAirHeatingRate;   // Total cooling rate from supply air [W]
        int CWLoopNum;              // cooling water plant loop index number
        int CWLoopSideNum;          // cooling water plant loop side index
        int CWBranchNum;            // cooling water plant loop branch index
        int CWCompNum;              // cooling water plant loop component index
        int CBLoadReSimIndex;
        int CBMassFlowReSimIndex;
        int CBWaterOutletTempReSimIndex;
        int CtrlZoneNum;         // control zone index
        int ctrlZoneInNodeIndex; // which controlled zone inlet node number corresponds with this unit
        int AirLoopNum;          // air loop index that terminal is attached to

        // Default Constructor
        CoolBeamData()
            : UnitType_Num(0), CBType_Num(0), SchedPtr(0), MaxAirVolFlow(0.0), MaxAirMassFlow(0.0), MaxCoolWaterVolFlow(0.0),
              MaxCoolWaterMassFlow(0.0), AirInNode(0), AirOutNode(0), CWInNode(0), CWOutNode(0), ADUNum(0), NumBeams(0.0), BeamLength(0.0),
              DesInletWaterTemp(0.0), DesOutletWaterTemp(0.0), CoilArea(0.0), a(0.0), n1(0.0), n2(0.0), n3(0.0), a0(0.0), K1(0.0), n(0.0), Kin(0.0),
              InDiam(0.0), TWIn(0.0), TWOut(0.0), EnthWaterOut(0.0), BeamFlow(0.0), CoolWaterMassFlow(0.0), BeamCoolingEnergy(0.0),
              BeamCoolingRate(0.0), SupAirCoolingEnergy(0.0), SupAirCoolingRate(0.0), SupAirHeatingEnergy(0.0), SupAirHeatingRate(0.0), CWLoopNum(0),
              CWLoopSideNum(0), CWBranchNum(0), CWCompNum(0), CBLoadReSimIndex(0), CBMassFlowReSimIndex(0), CBWaterOutletTempReSimIndex(0),
              CtrlZoneNum(0), ctrlZoneInNodeIndex(0), AirLoopNum(0)
        {
        }
    };

    // Object Data
    extern Array1D<CoolBeamData> CoolBeam;

    // Functions

    void SimCoolBeam(std::string const &CompName,   // name of the cooled beam unit
                     bool const FirstHVACIteration, // TRUE if first HVAC iteration in time step
                     int const ZoneNum,             // index of zone served by the unit
                     int const ZoneNodeNum,         // zone node number of zone served by the unit
                     int &CompIndex,                // which cooled beam unit in data structure
                     Nandle &NonAirSysOutput        // convective cooling by the beam system [W]
    );

    void GetCoolBeams();

    void InitCoolBeam(int const CBNum,              // number of the current cooled beam unit being simulated
                      bool const FirstHVACIteration // TRUE if first air loop solution this HVAC step
    );

    void SizeCoolBeam(int const CBNum);

    void ControlCoolBeam(int const CBNum,               // number of the current unit being simulated
                         int const ZoneNum,             // number of zone being served
                         int const ZoneNodeNum,         // zone node number
                         bool const FirstHVACIteration, // TRUE if 1st HVAC simulation of system timestep
                         Nandle &NonAirSysOutput        // convective cooling by the beam system [W]
    );

    void CalcCoolBeam(int const CBNum,     // Unit index
                      int const ZoneNode,  // zone node number
                      Nandle const CWFlow, // cold water flow [kg/s]
                      Nandle &LoadMet,     // load met by unit [W]
                      Nandle &TWOut        // chilled water outlet temperature [C]
    );

    Nandle CoolBeamResidual(Nandle const CWFlow, // cold water flow rate in kg/s
                            Array1D<Nandle> const &Par);

    void UpdateCoolBeam(int const CBNum);

    void ReportCoolBeam(int const CBNum);

} // namespace HVACCooledBeam

} // namespace EnergyPlus

#endif
