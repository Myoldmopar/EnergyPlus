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

#ifndef GeneralRoutines_hh_INCLUDED
#define GeneralRoutines_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array2S.hh>
#include <ObjexxFCL/Optional.hh>

// EnergyPlus Headers
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

void ControlCompOutput(std::string const &CompName,               // the component Name
                       std::string const &CompType,               // Type of component
                       int &CompNum,                              // Index of component in component array
                       bool const FirstHVACIteration,             // flag for 1st HVAV iteration in the time step
                       Nandle const QZnReq,                       // zone load to be met
                       int const ActuatedNode,                    // node that controls unit output
                       Nandle const MaxFlow,                      // maximum water flow
                       Nandle const MinFlow,                      // minimum water flow
                       Nandle const ControlOffset,                // really the tolerance
                       int &ControlCompTypeNum,                   // Internal type num for CompType
                       int &CompErrIndex,                         // for Recurring error call
                       Optional_int_const TempInNode = _,         // inlet node for output calculation
                       Optional_int_const TempOutNode = _,        // outlet node for output calculation
                       Optional<Nandle const> AirMassFlow = _,    // air mass flow rate
                       Optional_int_const Action = _,             // 1=reverse; 2=normal
                       Optional_int_const EquipIndex = _,         // Identifier for equipment of Outdoor Air Unit "ONLY"
                       Optional_int_const LoopNum = _,            // for plant components, plant loop index
                       Optional_int_const LoopSide = _,           // for plant components, plant loop side index
                       Optional_int_const BranchIndex = _,        // for plant components, plant branch index
                       Optional_int_const ControlledZoneIndex = _ // controlled zone index for the zone containing the component
);

bool BBConvergeCheck(int const SimCompNum, Nandle const MaxFlow, Nandle const MinFlow);

void CheckSysSizing(std::string const &CompType, // Component Type (e.g. Chiller:Electric)
                    std::string const &CompName  // Component Name (e.g. Big Chiller)
);

void CheckThisAirSystemForSizing(int const AirLoopNum, bool &AirLoopWasSized);

void CheckZoneSizing(std::string const &CompType, // Component Type (e.g. Chiller:Electric)
                     std::string const &CompName  // Component Name (e.g. Big Chiller)
);

void CheckThisZoneForSizing(int const ZoneNum, // zone index to be checked
                            bool &ZoneWasSized);

void ValidateComponent(std::string const &CompType,  // Component Type (e.g. Chiller:Electric)
                       std::string const &CompName,  // Component Name (e.g. Big Chiller)
                       bool &IsNotOK,                // .TRUE. if this component pair is invalid
                       std::string const &CallString // Context of this pair -- for error message
);

void ValidateComponent(std::string const &CompType,    // Component Type (e.g. Chiller:Electric)
                       std::string const &CompValType, // Component "name" field type
                       std::string const &CompName,    // Component Name (e.g. Big Chiller)
                       bool &IsNotOK,                  // .TRUE. if this component pair is invalid
                       std::string const &CallString   // Context of this pair -- for error message
);

void CalcPassiveExteriorBaffleGap(const Array1D_int &SurfPtrARR, // Array of indexes pointing to Surface structure in DataSurfaces
                                  Nandle const VentArea,        // Area available for venting the gap [m2]
                                  Nandle const Cv,              // Oriface coefficient for volume-based discharge, wind-driven [--]
                                  Nandle const Cd,              // oriface coefficient for discharge,  bouyancy-driven [--]
                                  Nandle const HdeltaNPL,       // Height difference from neutral pressure level [m]
                                  Nandle const SolAbs,          // solar absorptivity of baffle [--]
                                  Nandle const AbsExt,          // thermal absorptance/emittance of baffle material [--]
                                  Nandle const Tilt,            // Tilt of gap [Degrees]
                                  Nandle const AspRat,          // aspect ratio of gap  Height/gap [--]
                                  Nandle const GapThick,        // Thickness of air space between baffle and underlying heat transfer surface
                                  int const Roughness,          // Roughness index (1-6), see DataHeatBalance parameters
                                  Nandle const QdotSource,      // Source/sink term, e.g. electricity exported from solar cell [W]
                                  Nandle &TsBaffle,             // Temperature of baffle (both sides) use lagged value on input [C]
                                  Nandle &TaGap,                // Temperature of air gap (assumed mixed) use lagged value on input [C]
                                  Optional<Nandle> HcGapRpt = _,
                                  Optional<Nandle> HrGapRpt = _,
                                  Optional<Nandle> IscRpt = _,
                                  Optional<Nandle> MdotVentRpt = _,
                                  Optional<Nandle> VdotWindRpt = _,
                                  Optional<Nandle> VdotBouyRpt = _);

//****************************************************************************

void PassiveGapNusseltNumber(Nandle const AspRat, // Aspect Ratio of Gap height to gap width
                             Nandle const Tilt,   // Tilt of gap, degrees
                             Nandle const Tso,    // Temperature of gap surface closest to outside (K)
                             Nandle const Tsi,    // Temperature of gap surface closest to zone (K)
                             Nandle const Gr,     // Gap gas Grashof number
                             Nandle &gNu          // Gap gas Nusselt number
);

void CalcBasinHeaterPower(Nandle const Capacity,     // Basin heater capacity per degree C below setpoint (W/C)
                          int const SchedulePtr,     // Pointer to basin heater schedule
                          Nandle const SetPointTemp, // setpoint temperature for basin heater operation (C)
                          Nandle &Power              // Basin heater power (W)
);

void TestAirPathIntegrity(bool &ErrFound);

void TestSupplyAirPathIntegrity(bool &ErrFound);

void TestReturnAirPathIntegrity(bool &ErrFound, Array2S_int ValRetAPaths);

} // namespace EnergyPlus

#endif
