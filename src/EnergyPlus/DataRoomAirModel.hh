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

#ifndef DataRoomAirModel_hh_INCLUDED
#define DataRoomAirModel_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>
#include <ObjexxFCL/Array2D.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {

namespace DataRoomAirModel {

    // Using/Aliasing

    // Data
    // module should be available to other modules and routines.  Thus,
    // all variables in this module must be PUBLIC.

    // MODULE PARAMETER DEFINITIONS
    extern std::string const cUserDefinedControlObject;
    extern std::string const cTempPatternConstGradientObject;
    extern std::string const cTempPatternTwoGradientObject;
    extern std::string const cTempPatternNDHeightObject;
    extern std::string const cTempPatternSurfMapObject;

    // Parameters to indicate room air model selected
    extern int const RoomAirModel_UserDefined;    // user defined patterns
    extern int const RoomAirModel_Mixing;         // mixing air model
    extern int const RoomAirModel_Mundt;          // Mundt nodal model
    extern int const RoomAirModel_UCSDDV;         // UCSD Displacement Ventilation model
    extern int const RoomAirModel_UCSDCV;         // UCSD-CV
    extern int const RoomAirModel_UCSDUFI;        // UCSD UFAD interior zone model
    extern int const RoomAirModel_UCSDUFE;        // UCSD UFAD exterior zone model
    extern int const RoomAirModel_AirflowNetwork; // RoomAirModel_AirflowNetwork interior zone model
    extern Array1D_string const ChAirModel;

    // Parameters to indicate air temperature coupling scheme
    extern int const DirectCoupling;   // direct coupling scheme
    extern int const IndirectCoupling; // indirect coupling scheme

    // Parameters to indicate type of air node, which is dependent on air models
    extern int const InletAirNode;              // air node at inlet (for Mundt and Rees&Haves Models)
    extern int const FloorAirNode;              // air node at floor (for Mundt and Rees&Haves Models)
    extern int const ControlAirNode;            // air node at control point (for Mundt Model)
    extern int const CeilingAirNode;            // air node at ceiling (for Mundt Model)
    extern int const MundtRoomAirNode;          // air node for vertical walls (for Mundt Model)
    extern int const ReturnAirNode;             // air node for return (for Mundt and Rees&Haves Models)
    extern int const AirflowNetworkRoomAirNode; // air node for airflow network based room air model
    extern int const PlumeAirNode1;             // air node for plume load (for Rees&Haves Model)
    extern int const PlumeAirNode2;             // air node for plume load (for Rees&Haves Model)
    extern int const PlumeAirNode3;             // air node for plume load (for Rees&Haves Model)
    extern int const PlumeAirNode4;             // air node for plume load (for Rees&Haves Model)
    extern int const RoomAirNode1;              // air node for vertical walls (for Rees&Haves Model)
    extern int const RoomAirNode2;              // air node for vertical walls (for Rees&Haves Model)
    extern int const RoomAirNode3;              // air node for vertical walls (for Rees&Haves Model)
    extern int const RoomAirNode4;              // air node for vertical walls (for Rees&Haves Model)

    // user-defined pattern two gradient interplotation modes
    extern int const OutdoorDryBulbMode;  // by outdoor air bulb.
    extern int const SensibleCoolingMode; // by sensible cooling load
    extern int const SensibleHeatingMode; // by sensible heating load
    extern int const ZoneAirTempMode;     // by zone air temperature
    extern int const DeltaOutdoorZone;    // by difference between zone and outdoor

    // user defined temperature pattern types
    extern int const ConstGradTempPattern;  // constant gradient in vertical direction
    extern int const TwoGradInterpPattern;  // two gradient interpolation
    extern int const NonDimenHeightPattern; // non-dimensionalized height
    extern int const SurfMapTempPattern;    // arbitrary surface mappings

    // Parameters to indicate type of control for the UCSD UFAD interior zone model
    // INTEGER, PARAMETER :: ConsFlow          = 1     ! constant supply air flow
    // INTEGER, PARAMETER :: VarFlowConsPress  = 2     ! variable supply air flow, constant supply plenum pressure
    // INTEGER, PARAMETER :: VarFlowVarPress   = 3     ! variable supply air flow, variable supply plenum pressure

    // parameters to indicate diffuser type
    extern int const Swirl;
    extern int const VarArea;
    extern int const DisplVent;
    extern int const LinBarGrille;
    extern int const Custom;

    // parameters for comfort calculations
    extern int const VComfort_Invalid;
    extern int const VComfort_Jet;
    extern int const VComfort_Recirculation;

    // DERIVED TYPE DEFINITIONS

    // Air Node Data

    // UCSD

    // END UCSD

    // begin NREL RoomAir DERIVED TYPES ******************************************

    // end NREL room air derived types*********************************

    // INTERFACE BLOCK SPECIFICATIONS
    // na

    // MODULE VARIABLE DECLARATIONS:
    extern int TotNumOfAirNodes;
    extern int TotNumOfRoomAFNNodes;
    extern Array1D_int TotNumOfZoneAirNodes;
    extern Array1D<Nandle> ConvectiveFloorSplit;
    extern Array1D<Nandle> InfiltratFloorSplit;
    // UCSD
    extern Array1D<Nandle> DVHcIn;
    extern int TotUCSDDV;                // Total number of UCSDDV zones
    extern Array1D_bool IsZoneDV;        // Is the air model for the zone UCSDDV?
    extern Array1D<Nandle> ZTOC;         // Temperature of occupied (lower) zone
    extern Array1D<Nandle> AvgTempGrad;  // vertical Average Temperature Gradient in the room
    extern Array1D<Nandle> ZTMX;         // Temperature of the mixing(upper) layer
    extern Array1D<Nandle> MaxTempGrad;  // maximum Average Temperature Gradient in the room
    extern Array1D<Nandle> HVACAirTemp;  // HVAC system temperature (DEG C)
    extern Array1D<Nandle> HVACMassFlow; // HVAC system mass flow rate (KG/S)
    extern Array1D<Nandle> ZTFloor;
    extern Array1D<Nandle> HeightTransition;
    extern Array1D<Nandle> FracMinFlow;
    extern Array1D_int ZoneDVMixedFlag;
    extern Array1D<Nandle> ZoneDVMixedFlagRep;
    extern Array1D_bool ZoneAirSystemON;
    extern Array1D<Nandle> TCMF; // comfort temperature
    extern Array1D<Nandle> ZoneCeilingHeight;
    extern Array1D<Nandle> MATFloor;    // [C] floor level mean air temp
    extern Array1D<Nandle> XMATFloor;   // [C] floor level mean air temp at t minus 1 zone time step
    extern Array1D<Nandle> XM2TFloor;   // [C] floor level mean air temp at t minus 2 zone time step
    extern Array1D<Nandle> XM3TFloor;   // [C] floor level mean air temp at t minus 3 zone time step
    extern Array1D<Nandle> XM4TFloor;   // [C] floor level mean air temp at t minus 4 zone time step
    extern Array1D<Nandle> DSXMATFloor; // [C] floor level mean air temp at t minus 1 system time step
    extern Array1D<Nandle> DSXM2TFloor; // [C] floor level mean air temp at t minus 2 system time step
    extern Array1D<Nandle> DSXM3TFloor; // [C] floor level mean air temp at t minus 3 system time step
    extern Array1D<Nandle> DSXM4TFloor; // [C] floor level mean air temp at t minus 4 system time step
    extern Array1D<Nandle> MATOC;       // [C] occupied mean air temp
    extern Array1D<Nandle> XMATOC;      // [C] occupied mean air temp at t minus 1 zone time step
    extern Array1D<Nandle> XM2TOC;      // [C] occupied mean air temp at t minus 2 zone time step
    extern Array1D<Nandle> XM3TOC;      // [C] occupied mean air temp at t minus 3 zone time step
    extern Array1D<Nandle> XM4TOC;      // [C] occupied mean air temp at t minus 4 zone time step
    extern Array1D<Nandle> DSXMATOC;    // [C] occupied mean air temp at t minus 1 system time step
    extern Array1D<Nandle> DSXM2TOC;    // [C] occupied mean air temp at t minus 2 system time step
    extern Array1D<Nandle> DSXM3TOC;    // [C] occupied mean air temp at t minus 3 system time step
    extern Array1D<Nandle> DSXM4TOC;    // [C] occupied mean air temp at t minus 4 system time step
    extern Array1D<Nandle> MATMX;       // [C] mixed (upper) mean air temp
    extern Array1D<Nandle> XMATMX;      // [C] mixed (upper) mean air temp at t minus 1 zone time step
    extern Array1D<Nandle> XM2TMX;      // [C] mixed (upper) mean air temp at t minus 2 zone time step
    extern Array1D<Nandle> XM3TMX;      // [C] mixed (upper) mean air temp at t minus 3 zone time step
    extern Array1D<Nandle> XM4TMX;      // [C] mixed (upper) mean air temp at t minus 4 zone time step
    extern Array1D<Nandle> DSXMATMX;    // [C] mixed  mean air temp at t minus 1 system time step
    extern Array1D<Nandle> DSXM2TMX;    // [C] mixed  mean air temp at t minus 2 system time step
    extern Array1D<Nandle> DSXM3TMX;    // [C] mixed  mean air temp at t minus 3 system time step
    extern Array1D<Nandle> DSXM4TMX;    // [C] mixed  mean air temp at t minus 4 system time step
    extern Array1D<Nandle> ZTM1Floor;   // [C] difference equation's Floor air temp at t minus 1
    extern Array1D<Nandle> ZTM2Floor;   // [C] difference equation's Floor air temp at t minus 2
    extern Array1D<Nandle> ZTM3Floor;   // [C] difference equation's Floor air temp at t minus 3
    extern Array1D<Nandle> ZTM1OC;      // [C] difference equation's Occupied air temp at t minus 1
    extern Array1D<Nandle> ZTM2OC;      // [C] difference equation's Occupied air temp at t minus 2
    extern Array1D<Nandle> ZTM3OC;      // [C] difference equation's Occupied air temp at t minus 3
    extern Array1D<Nandle> ZTM1MX;      // [C] difference equation's Mixed  air temp at t minus 1
    extern Array1D<Nandle> ZTM2MX;      // [C] difference equation's Mixed  air temp at t minus 1
    extern Array1D<Nandle> ZTM3MX;      // [C] difference equation's Mixed  air temp at t minus 1
    extern Array1D<Nandle> AIRRATFloor;
    extern Array1D<Nandle> AIRRATOC;
    extern Array1D<Nandle> AIRRATMX;
    // Euler and Exact solution algorithms
    extern Array1D<Nandle> Zone1Floor;  // [C] difference equation's Floor air temp at previous dt
    extern Array1D<Nandle> ZoneMXFloor; // [C] difference equation's Floor air temp at t minus 1
    extern Array1D<Nandle> ZoneM2Floor; // [C] difference equation's Floor air temp at t minus 2
    extern Array1D<Nandle> Zone1OC;     // [C] difference equation's Occupied air temp at previous dt
    extern Array1D<Nandle> ZoneMXOC;    // [C] difference equation's Occupied air temp at t minus 1
    extern Array1D<Nandle> ZoneM2OC;    // [C] difference equation's Occupied air temp at t minus 2
    extern Array1D<Nandle> Zone1MX;     // [C] difference equation's Mixed  air temp at previous dt
    extern Array1D<Nandle> ZoneMXMX;    // [C] difference equation's Mixed  air temp at t minus 1
    extern Array1D<Nandle> ZoneM2MX;    // [C] difference equation's Mixed  air temp at t minus 2
    // UCSD-CV
    extern Array1D<Nandle> CVHcIn;
    extern int TotUCSDCV;                   // Total number of UCSDDV zones
    extern Array1D_bool IsZoneCV;           // Is the air model for the zone UCSDDV?
    extern Array1D<Nandle> ZoneCVisMixing;  // Zone set to CV is actually using a mixing model
    extern Array1D<Nandle> ZTJET;           // Jet Temperatures
    extern Array1D<Nandle> ZTREC;           // Recirculation Temperatures
    extern Array1D<Nandle> RoomOutflowTemp; // Temperature of air flowing out of the room
    extern Array1D<Nandle> JetRecAreaRatio;
    extern Array1D<Nandle> Urec;           // Recirculation region average velocity
    extern Array1D<Nandle> Ujet;           // Jet region average velocity
    extern Array1D<Nandle> Qrec;           // Recirculation zone total flow rate
    extern Array1D<Nandle> Qtot;           // Total volumetric inflow rate through all active aperatures [m3/s]
    extern Array1D<Nandle> RecInflowRatio; // Ratio of the recirculation volumetric flow rate to the total inflow flow rate []
    extern Array1D<Nandle> Uhc;
    extern Array1D<Nandle> Ain;                     // Inflow aperture area
    extern Array1D<Nandle> Droom;                   // CV Zone average length
    extern Array1D<Nandle> Dstar;                   // CV Zone average length, wind direction corrected
    extern Array1D<Nandle> Tin;                     // Inflow air temperature
    extern Array1D<Nandle> TotArea;                 // Sum of the areas of all apertures in the zone
    extern Array2D_int AirflowNetworkSurfaceUCSDCV; // table for AirflowNetwork surfaces organization
    extern int CVNumAirflowNetworkSurfaces;         // total number of AirFlowNetwork surfaces.
    // Interzone surfaces counts twice.
    extern Array1D<Nandle> Rfr;          // Ration between inflow and recirculation air flows
    extern Array1D<Nandle> ZoneCVhasREC; // Airflow pattern is C(0), CR(1)
    extern bool UCSDModelUsed;
    extern bool MundtModelUsed;
    // UCSD-UF
    extern int TotUCSDUI;         // total number of UCSDUI zones
    extern int TotUCSDUE;         // total number of UCSDUE zones
    extern Array1D_bool IsZoneUI; // controls program flow, for interior or exterior UFAD model
    extern Array1D_int ZoneUFPtr;
    extern Array1D<Nandle> UFHcIn;
    extern Array1D_int ZoneUFMixedFlag;
    extern Array1D<Nandle> ZoneUFMixedFlagRep;
    extern Array1D<Nandle> ZoneUFGamma;
    extern Array1D<Nandle> ZoneUFPowInPlumes;            // [W]
    extern Array1D<Nandle> ZoneUFPowInPlumesfromWindows; // [W]
    extern Array1D<Nandle> Phi;                          // dimensionless measure of occupied subzone temperature

    // END UCSD

    // Begin NREL User-defined patterns

    extern int numTempDistContrldZones; // count of zones with user-defined patterns
    extern int NumAirTempPatterns;      // count of all different patterns in input file
    extern int NumConstantGradient;     // count of constant gradient patterns in input
    extern int NumTwoGradientInterp;    // count of two gradient interp patterns in input
    extern int NumNonDimensionalHeight; // count of ND height profile patterns in input
    extern int NumSurfaceMapping;       // count of generic surface map patterns in input

    extern bool UserDefinedUsed; // true if user-defined model used anywhere

    // End User-defined patterns

    // RoomAirflowNetwork
    extern int NumOfRoomAirflowNetControl; // count of RoomAirSettings:AirflowNetwork

    // Types

    struct AirModelData
    {
        // Members
        std::string AirModelName;
        std::string ZoneName;
        int ZonePtr;      // Pointer to the zone number for this statement
        int AirModelType; // 1 = Mixing, 2 = Mundt, 3 = Rees and Haves,
        // 4 = UCSDDV, 5 = UCSDCV, -1 = user defined
        // 6 = UCSDUFI, 7 = UCSDUFE, 8 = AirflowNetwork
        int TempCoupleScheme; // 1 = absolute (direct),
        // 2 = relative air model temperature passing scheme (indirect)
        bool SimAirModel; // FALSE if Mixing air model is currently used and
        // TRUE if other air models are currently used

        // Default Constructor
        AirModelData() : ZonePtr(0), AirModelType(RoomAirModel_Mixing), TempCoupleScheme(DirectCoupling), SimAirModel(false)
        {
        }
    };

    struct AirNodeData
    {
        // Members
        std::string Name; // name
        std::string ZoneName;
        int ZonePtr;               // Pointer to the zone number for this statement
        int ClassType;             // depending on type of model
        Nandle Height;             // height
        Nandle ZoneVolumeFraction; // portion of zone air volume associated with this node
        Array1D_bool SurfMask;     // limit of 60 surfaces at current sizing
        bool IsZone;               // TRUE if this node is zone node

        // Default Constructor
        AirNodeData() : ZonePtr(0), ClassType(0), Height(0.0), ZoneVolumeFraction(0), IsZone(false)
        {
        }
    };

    struct DVData
    {
        // Members
        std::string ZoneName;       // Name of zone
        int ZonePtr;                // Pointer to the zone number for this statement
        int SchedGainsPtr;          // Schedule for internal gain fraction to occupied zone
        std::string SchedGainsName; // Gains Schedule name
        Nandle NumPlumesPerOcc;     // Effective number of plumes per occupant
        Nandle ThermostatHeight;    // Height of thermostat/ temperature control sensor
        Nandle ComfortHeight;       // Height at which air temperature is measured for comfort purposes
        Nandle TempTrigger;         // Minimum temperature difference between TOC TMX for stratification

        // Default Constructor
        DVData() : ZonePtr(0), SchedGainsPtr(-1), NumPlumesPerOcc(1.0), ThermostatHeight(0.0), ComfortHeight(0.0), TempTrigger(0.0)
        {
        }
    };

    struct CVData
    {
        // Members
        std::string ZoneName;       // Name of zone
        int ZonePtr;                // Pointer to the zone number for this statement
        int SchedGainsPtr;          // Schedule for internal gain fraction to occupied zone
        std::string SchedGainsName; // Gains Schedule name
        int VforComfort;            // Use Recirculation or Jet velocity and temperatures
        // for comfort models

        // Default Constructor
        CVData() : ZonePtr(-1), SchedGainsPtr(-1), VforComfort(VComfort_Invalid)
        {
        }
    };

    struct CVFlow
    {
        // Members
        int FlowFlag; // Equal to 1 if the opening has inflow, else equal to 0.
        Nandle Width; // Width of the opening [m]
        Nandle Area;  // Area of the opening [m2]
        Nandle Fin;   // Inflow volume flux through the opening [m3/s]
        Nandle Uin;   // Inflow air velocity through the opening [m/s]
        Nandle Vjet;  // Average maximum jet velocity for the opening [m/s]
        Nandle Yjet;  // Y in "Y = aX + b" formula
        Nandle Ujet;  // Volume average jet region velocity [m/s]
        Nandle Yrec;  // Y in "Y = aX + b" formula
        Nandle Urec;  // Area-averaged velocity in the y-z plane with maximum flow [m/s]
        Nandle YQrec; // Y in "Y = aX + b" formula
        Nandle Qrec;  // Total flow rate for the recirculation regions in the plane of maximum flow [m3/s]

        // Default Constructor
        CVFlow()
            : FlowFlag(0), Width(0.0), Area(0.0), Fin(0.0), Uin(0.0), Vjet(0.0), Yjet(0.0), Ujet(0.0), Yrec(0.0), Urec(0.0), YQrec(0.0), Qrec(0.0)
        {
        }
    };

    struct CVDVParameters
    {
        // Members
        Nandle Width;
        Nandle Height;
        int Shadow;
        Nandle Zmin;
        Nandle Zmax;

        // Default Constructor
        CVDVParameters() : Width(0.0), Height(0.0), Shadow(0), Zmin(0.0), Zmax(0.0)
        {
        }
    };

    struct UFIData
    {
        // Members
        std::string ZoneName;    // Name of zone
        int ZonePtr;             // Pointer to the zone number for this statement
        int ZoneEquipPtr;        // Pointer to zone equip for this UFAD zone
        Nandle DiffusersPerZone; // Number of diffusers in this zone
        Nandle PowerPerPlume;    // Power in each plume [W]
        Nandle DiffArea;         // Effective area of a diffuser [m2]
        Nandle DiffAngle;        // angle between diffuser slots and vertical (degrees)
        Nandle HeatSrcHeight;    // height of heat source above floor [m]
        Nandle ThermostatHeight; // Height of thermostat/ temperature control sensor [m]
        Nandle ComfortHeight;    // Height at which air temperature is measured for
        // comfort purposes [m]
        Nandle TempTrigger; // Minimum temperature difference between TOC TMX
        // for stratification [deltaC]
        int DiffuserType;     // 1=Swirl, 2=variable area, 3=displacement, 4=linear bar grille, 5=custom
        Nandle TransHeight;   // user specified transition height [m]
        bool CalcTransHeight; // flag to calc trans height or use user specified input
        Nandle A_Kc;          // Coefficient A in Formula Kc = A*Gamma**B + C + D*Gamma + E*Gamma**2
        Nandle B_Kc;          // Coefficient A in Formula Kc = A*Gamma**B + C + D*Gamma + E*Gamma**2
        Nandle C_Kc;          // Coefficient A in Formula Kc = A*Gamma**B + C + D*Gamma + E*Gamma**2
        Nandle D_Kc;          // Coefficient A in Formula Kc = A*Gamma**B + C + D*Gamma + E*Gamma**2
        Nandle E_Kc;          // Coefficient A in Formula Kc = A*Gamma**B + C + D*Gamma + E*Gamma**2

        // Default Constructor
        UFIData()
            : ZonePtr(0), ZoneEquipPtr(0), DiffusersPerZone(0.0), PowerPerPlume(0.0), DiffArea(0.0), DiffAngle(0.0), HeatSrcHeight(0.0),
              ThermostatHeight(0.0), ComfortHeight(0.0), TempTrigger(0.0), DiffuserType(0), TransHeight(0.0), CalcTransHeight(false), A_Kc(0.0),
              B_Kc(0.0), C_Kc(0.0), D_Kc(0.0), E_Kc(0.0)
        {
        }
    };

    struct UFEData
    {
        // Members
        std::string ZoneName;    // Name of zone
        int ZonePtr;             // Pointer to the zone number for this statement
        int ZoneEquipPtr;        // Pointer to zone equip for this UFAD zone
        Nandle DiffusersPerZone; // Number of diffusers in this zone
        Nandle PowerPerPlume;    // Power in each plume [W]
        Nandle DiffArea;         // Effective area of a diffuser [m2]
        Nandle DiffAngle;        // angle between diffuser slots and vertical (degrees)
        Nandle HeatSrcHeight;    // height of heat source above floor [m]
        Nandle ThermostatHeight; // Height of thermostat/ temperature control sensor [m]
        Nandle ComfortHeight;    // Height at which air temperature is measured for
        // comfort purposes [m]
        Nandle TempTrigger; // Minimum temperature difference between TOC TMX
        // for stratification [deltaC]
        int DiffuserType;     // 1=Swirl, 2=variable area, 3=displacement, 4=linear bar grille, 5=custom
        Nandle TransHeight;   // user specified transition height [m]
        bool CalcTransHeight; // flag to calc trans height or use user specified input
        Nandle A_Kc;          // Coefficient A in Formula Kc = A*Gamma**B + C + D*Gamma + E*Gamma**2
        Nandle B_Kc;          // Coefficient A in Formula Kc = A*Gamma**B + C + D*Gamma + E*Gamma**2
        Nandle C_Kc;          // Coefficient A in Formula Kc = A*Gamma**B + C + D*Gamma + E*Gamma**2
        Nandle D_Kc;          // Coefficient A in Formula Kc = A*Gamma**B + C + D*Gamma + E*Gamma**2
        Nandle E_Kc;          // Coefficient A in Formula Kc = A*Gamma**B + C + D*Gamma + E*Gamma**2
        Nandle WinWidth;      // sum of widths of exterior windows in zone
        Nandle NumExtWin;     // number of exterior windows in the zone
        bool ShadeDown;       // signals shade up or down

        // Default Constructor
        UFEData()
            : ZonePtr(0), ZoneEquipPtr(0), DiffusersPerZone(0.0), PowerPerPlume(0.0), DiffArea(0.0), DiffAngle(0.0), HeatSrcHeight(0.0),
              ThermostatHeight(0.0), ComfortHeight(0.0), TempTrigger(0.0), DiffuserType(0), TransHeight(0.0), CalcTransHeight(false), A_Kc(0.0),
              B_Kc(0.0), C_Kc(0.0), D_Kc(0.0), E_Kc(0.0), WinWidth(0.0), NumExtWin(0.0), ShadeDown(true)
        {
        }
    };

    struct SurfMapPattern // nested structure in RoomAirPattern
    {
        // Members
        // user variables
        Array1D_string SurfName;  // user defined name
        Array1D<Nandle> DeltaTai; // (Tai - MAT ) offset from mean air temp
        int NumSurfs;             // number of surfaces in this pattern
        // calculated and from elsewhere
        Array1D_int SurfID; // index in HB surface structure array

        // Default Constructor
        SurfMapPattern() : NumSurfs(0)
        {
        }
    };

    struct ConstGradPattern // nested structure in RoomAirPattern
    {
        // Members
        // user variables
        std::string Name; // name
        Nandle Gradient;  // value of vertical gradient [C/m]

        // Default Constructor
        ConstGradPattern() : Gradient(0.0)
        {
        }
    };

    struct TwoVertGradInterpolPattern // nested structure in RoomAirPattern
    {
        // Members
        // user variables
        std::string Name;               // name
        Nandle TstatHeight;             // Height of thermostat/ temperature control sensor
        Nandle TleavingHeight;          // height of return air node where leaving zone
        Nandle TexhaustHeight;          // height of exhaust air node where leaving zone
        Nandle LowGradient;             // lower value of vertical gradient [C/m]
        Nandle HiGradient;              // upper value of vertical gradient [C/m]
        int InterpolationMode;          // control for interpolation mode
        Nandle UpperBoundTempScale;     // temperature value for HiGradient
        Nandle LowerBoundTempScale;     // temperature value for LowGradient
        Nandle UpperBoundHeatRateScale; // load value for HiGradient
        Nandle LowerBoundHeatRateScale; // load value for lowGradient

        // Default Constructor
        TwoVertGradInterpolPattern()
            : TstatHeight(0.0), TleavingHeight(0.0), TexhaustHeight(0.0), LowGradient(0.0), HiGradient(0.0), InterpolationMode(0),
              UpperBoundTempScale(0.0), LowerBoundTempScale(0.0), UpperBoundHeatRateScale(0.0), LowerBoundHeatRateScale(0.0)
        {
        }
    };

    struct TempVsHeightPattern // to be used as nested structure in RoomAirPattern
    {
        // Members
        Array1D<Nandle> ZetaPatrn;     // non dimensional height from floor,
        Array1D<Nandle> DeltaTaiPatrn; // Tai- MAT (TODO, check sign)

        // Default Constructor
        TempVsHeightPattern()
        {
        }
    };

    struct TemperaturePatternStruct // RoomAirPattern
    {
        // Members
        std::string Name;                        // unique identifier
        int PatrnID;                             // control ID for referencing in Schedules
        int PatternMode;                         // Control for what type of calcs in this pattern
        ConstGradPattern GradPatrn;              // Constant gradient pattern
        TwoVertGradInterpolPattern TwoGradPatrn; // Two gradient interpolation pattern
        TempVsHeightPattern VertPatrn;           // Vertical gradient profile pattern
        SurfMapPattern MapPatrn;                 // Generic Surface map pattern
        Nandle DeltaTstat;                       // (Tstat - MAT) offset   deg C
        Nandle DeltaTleaving;                    // (Tleaving - MAT) deg C
        Nandle DeltaTexhaust;                    // (Texhaust - MAT) deg C

        // Default Constructor
        TemperaturePatternStruct() : PatrnID(0), PatternMode(0), DeltaTstat(0.0), DeltaTleaving(0.0), DeltaTexhaust(0.0)
        {
        }
    };

    struct SurfaceAssocNestedStruct
    {
        // Members
        std::string Name;    // unique identifier
        int SurfID;          // id in HB surface structs
        Nandle TadjacentAir; // place to put resulting temperature value
        Nandle Zeta;         // non-dimensional height in zone ot

        // Default Constructor
        SurfaceAssocNestedStruct() : SurfID(0), TadjacentAir(23.0), Zeta(0.0)
        {
        }
    };

    struct AirPatternInfobyZoneStruct // becomes AirPatternZoneInfo
    {
        // Members
        // user variables
        bool IsUsed;                   // .TRUE. if user-defined patterns used in zone
        std::string Name;              // Name
        std::string ZoneName;          // Zone name in building
        int ZoneID;                    // Index of Zone in Heat Balance
        std::string AvailSched;        // Name of availability schedule
        int AvailSchedID;              // index of availability schedule
        std::string PatternCntrlSched; // name of schedule that selects pattern
        int PatternSchedID;            // index of pattern selecting schedule
        // calculated and from elsewhere
        Nandle ZoneHeight;                      // in meters, from Zone%CeilingHeight
        int ZoneNodeID;                         // index in Node array for this zone
        Array1D_int ExhaustAirNodeID;           // indexes in Node array
        Nandle TairMean;                        // comes from MAT
        Nandle Tstat;                           // temperature for thermostat
        Nandle Tleaving;                        // temperature for return air node
        Nandle Texhaust;                        // temperature for exhaust air node
        Array1D<SurfaceAssocNestedStruct> Surf; // nested struct w/ surface info
        int totNumSurfs;                        // total surfs for this zone
        int firstSurfID;                        // Index of first surface
        // report
        Nandle Gradient; // result for modeled gradient if using two-gradient interpolation

        // Default Constructor
        AirPatternInfobyZoneStruct()
            : IsUsed(false), ZoneID(0), AvailSchedID(0), PatternSchedID(0), ZoneHeight(0.0), ZoneNodeID(0), TairMean(23.0), Tstat(23.0),
              Tleaving(23.0), Texhaust(23.0), totNumSurfs(0), firstSurfID(0), Gradient(0.0)
        {
        }
    };

    struct AirflowLinkagesInfoNestedStruct // becomes link
    {
        // Members
        // user variables
        int AirflowNetworkLinkSimuID;    // point to this linkage in AirflowNetworkLinkSimu structure
        int AirflowNetworkLinkageDataID; // point to this linkage in AirflowNetworkLinkageData structure
        int AirflowNetworkLinkReportID;  // point to this linkage in AirflowNetworkLinkReport structure
        Nandle MdotIn;                   // mass flow rate of air into control volume(neg means leaving control volume) (kg / s)
        Nandle TempIn;                   // drybulb temperature of air into control volume
        Nandle HumRatIn;                 // humidity ratio of air into control volume

        // Default Constructor
        AirflowLinkagesInfoNestedStruct()
            : AirflowNetworkLinkSimuID(0), AirflowNetworkLinkageDataID(0), AirflowNetworkLinkReportID(0), MdotIn(0.0), TempIn(0.0), HumRatIn(0.0)
        {
        }
    };

    struct RoomAirflowNetworkNodeInternalGainsStruct // becomes IntGain
    {
        // Members
        // user variables
        int TypeOfNum;                    // Internal type
        std::string Name;                 // Intenral gain name
        bool UseRoomAirModelTempForGains; // TRUE if user inputs temp for gains
        bool FractionCheck;               // TRUE if a fraction of internal gain for each object is checked

        // Default Constructor
        RoomAirflowNetworkNodeInternalGainsStruct() : TypeOfNum(0), UseRoomAirModelTempForGains(false), FractionCheck(false)
        {
        }
    };

    struct RoomAirflowNetworkHVACStruct // becomes HVAC
    {
        // Members
        // user variables
        std::string Name;           // HVAC system name
        std::string ObjectTypeName; // HVAC object type name
        std::string SupplyNodeName; // HVAC system supply node name
        std::string ReturnNodeName; // HVAC system return node name
        int TypeOfNum;              // HVAC type num
        Nandle SupplyFraction;      // Supply flow fraction
        Nandle ReturnFraction;      // Return flow fraction
        int EquipConfigIndex;       // HVAC equipment configuration index
        int SupNodeNum;             // HVAC supply node number
        int RetNodeNum;             // HVAC return node number
        int CompIndex;              // Component index

        // Default Constructor
        RoomAirflowNetworkHVACStruct()
            : TypeOfNum(0),        // HVAC type num
              SupplyFraction(0),   // Supply flow fraction
              ReturnFraction(0),   // Return flow fraction
              EquipConfigIndex(0), // HVAC equipment configuration index
              SupNodeNum(0),       // HVAC supply node number
              RetNodeNum(0),       // HVAC return node number
              CompIndex(0)         // Component index
        {
        }
    };

    struct RoomAirflowNetworkAirNodeNestedStruct // becomes Node
    {
        // Members
        // user variables
        std::string Name;                                           // name of the node itself
        Nandle ZoneVolumeFraction;                                  // Zone volume fraction applied to this specific node
        std::string NodeSurfListName;                               // name of nodes' adjacent surface list
        bool HasSurfacesAssigned;                                   // True if this node has surfaces assigned
        Array1D<bool> SurfMask;                                     // Sized to num of surfs in Zone, true if surface is associated with this node
        std::string NodeIntGainsListName;                           // name of node's internal gains list
        bool HasIntGainsAssigned;                                   // True if this node has internal gain assigned
        int NumIntGains;                                            // Number of internal gain objects
        Array1D<int> IntGainsDeviceIndices;                         // sized to NumIntGains, index pointers to internal gains struct
        Array1D<Nandle> IntGainsFractions;                          // sized to NumIntGains, gain fractions to this node
        Array1D<RoomAirflowNetworkNodeInternalGainsStruct> IntGain; // Internal gain struct
        std::string NodeHVACListName;                               // name of node's HVAC list
        bool HasHVACAssigned;                                       // True if HVAC systems are assigned to this node
        int NumHVACs;                                               // Number of HVAC systems
        Array1D<RoomAirflowNetworkHVACStruct> HVAC;                 // HVAC struct
        int AirflowNetworkNodeID;                                   // pointer to AirflowNetworkNodeData structure
        int NumOfAirflowLinks;                                      // Number of intra zone links
        Array1D<AirflowLinkagesInfoNestedStruct> Link;              // Linkage struct
        Nandle AirVolume;                                           // air volume in control volume associated with this node(m3 / s)
        Nandle RhoAir;                                              // current density of air for nodal control volume
        Nandle CpAir;                                               // current heat capacity of air for nodal control volume

        Nandle AirTemp;     // node air temperature
        Nandle AirTempX1;   // node air temperature at t minus 1 zone timestep
        Nandle AirTempX2;   // node air temperature at t minus 2 zone timestep
        Nandle AirTempX3;   // node air temperature at t minus 3 zone timestep
        Nandle AirTempX4;   // node air temperature at t minus 4 zone timestep
        Nandle AirTempDSX1; // node air temperature at t minus 1 system timestep
        Nandle AirTempDSX2; // node air temperature at t minus 2 system timestep
        Nandle AirTempDSX3; // node air temperature at t minus 3 system timestep
        Nandle AirTempDSX4; // node air temperature at t minus 4 system timestep
        Nandle AirTempT1;   // node air temperature at the previous time step used in Exact and Euler method
        Nandle AirTempTMX;  // temporary node air temperature to test convergence used in Exact and Euler method
        Nandle AirTempTM2;  // node air temperature at time step t-2 used in Exact and Euler method

        Nandle HumRat;     // node air humidity ratio
        Nandle HumRatX1;   // node air humidity ratio at t minus 1 zone timestep
        Nandle HumRatX2;   // node air humidity ratio at t minus 2 zone timestep
        Nandle HumRatX3;   // node air humidity ratio at t minus 3 zone timestep
        Nandle HumRatX4;   // node air humidity ratio at t minus 4 zone timestep
        Nandle HumRatDSX1; // node air humidity ratio at t minus 1 system timestep
        Nandle HumRatDSX2; // node air humidity ratio at t minus 2 system timestep
        Nandle HumRatDSX3; // node air humidity ratio at t minus 3 system timestep
        Nandle HumRatDSX4; // node air humidity ratio at t minus 4 system timestep
        Nandle HumRatW1;   // node air humidity ratio at the previous time step used in Exact and Euler method
        Nandle HumRatWMX;  // temporary node air humidity ratio to test convergence used in Exact and Euler method
        Nandle HumRatWM2;  // node air humidity ratio at time step t-2 used in Exact and Euler method

        Nandle RelHumidity; // node air relative humidity

        // sensible heat balance terms for node
        Nandle SumIntSensibleGain; // rate of heat gain from internal sensible gains(after fraction)
        Nandle SumHA;              // sum of Hc * Area for surfaces associated with this node(surface convection sensible gain term)
        Nandle SumHATsurf;         // sum of Hc * Area * Temp for surfaces associated with this node for convective heat transfer
        Nandle SumHATref;          // sum of Hc * Area * Temp for surfaces associated with this node for radiation exchange
        Nandle SumLinkMCp;         // sum of mdor*Cp for incoming airflows for this node derived from the AirflowNetwork model
        Nandle SumLinkMCpT; // sum of mdor*Cp*T for incoming airflows and source temperature for this node derived from the AirflowNetwork model
        Nandle SumSysMCp;   // sum of mdor*Cp for incoming supply airflows for this node
        Nandle SumSysMCpT;  // sum of mdor*Cp*T for incoming supply airflows and temperature for this node
        Nandle SumSysM;     // sum of mdot for incoming supply airflows for this node
        Nandle SumSysMW;    // sum of mdot*W for incoming supply airflows and temperature for this node
        Nandle NonAirSystemResponse;     // sum of convective system load
        Nandle SysDepZoneLoadsLagged;    // sum of system lagged load
        Nandle SysDepZoneLoadsLaggedOld; // sum of system lagged load
        Nandle AirCap;                   // Air storage term for energy balalce at each node
        Nandle AirHumRat;                // Air storage term for moisture balalce at each node
        // latent moisture balance terms for node
        Nandle SumIntLatentGain; // rate of heat gain form internal latent gains(after fraction)
        Nandle SumHmAW;          // sum of AREA*Moist CONVECTION COEFF*INSIDE Humidity Ratio
        Nandle SumHmARa;         // SUM OF ZONE AREA*Moist CONVECTION COEFF*Rho Air
        Nandle SumHmARaW;        // SUM OF ZONE AREA*Moist CONVECTION COEFF*Rho Air* Inside Humidity Ratio
        Nandle SumLinkM;         // sum of mdor for incoming airflows for this node derived from the AirflowNetwork model
        Nandle SumLinkMW; // sum of mdor*Cp*T for incoming airflows and source humidity ratio for this node derived from the AirflowNetwork model

        // Default Constructor
        RoomAirflowNetworkAirNodeNestedStruct()
            : ZoneVolumeFraction(0.0), HasSurfacesAssigned(false), HasIntGainsAssigned(false), NumIntGains(0), HasHVACAssigned(false), NumHVACs(0),
              AirflowNetworkNodeID(0),              // pointer to AirflowNetworkNodeData structure
              NumOfAirflowLinks(0), AirVolume(0.0), // air volume in control volume associated with this node(m3 / s)
              RhoAir(0.0),                          // current density of air for nodal control volume
              CpAir(0.0),                           // current heat capacity of air for nodal control volume
              AirTemp(0.0),                         // node air temperature
              AirTempX1(0.0),                       // node air temperature at t minus 1 zone timestep
              AirTempX2(0.0),                       // node air temperature at t minus 2 zone timestep
              AirTempX3(0.0),                       // node air temperature at t minus 3 zone timestep
              AirTempX4(0.0),                       // node air temperature at t minus 4 zone timestep
              AirTempDSX1(0.0),                     // node air temperature at t minus 1 system timestep
              AirTempDSX2(0.0),                     // node air temperature at t minus 2 system timestep
              AirTempDSX3(0.0),                     // node air temperature at t minus 3 system timestep
              AirTempDSX4(0.0),                     // node air temperature at t minus 4 system timestep
              AirTempT1(0.0),                       // node air temperature at the previous time step used in Exact and Euler method
              AirTempTMX(0.0),                      // temporary node air temperature to test convergence used in Exact and Euler method
              AirTempTM2(0.0),                      // node air temperature at time step t-2 used in Exact and Euler method
              HumRat(0.0),                          // node air humidity ratio
              HumRatX1(0.0),                        // node air humidity ratio at t minus 1 zone timestep
              HumRatX2(0.0),                        // node air humidity ratio at t minus 2 zone timestep
              HumRatX3(0.0),                        // node air humidity ratio at t minus 3 zone timestep
              HumRatX4(0.0),                        // node air humidity ratio at t minus 4 zone timestep
              HumRatDSX1(0.0),                      // node air humidity ratio at t minus 1 system timestep
              HumRatDSX2(0.0),                      // node air humidity ratio at t minus 2 system timestep
              HumRatDSX3(0.0),                      // node air humidity ratio at t minus 3 system timestep
              HumRatDSX4(0.0),                      // node air humidity ratio at t minus 4 system timestep
              HumRatW1(0.0),                        // node air humidity ratio at the previous time step used in Exact and Euler method
              HumRatWMX(0.0),                       // temporary node air humidity ratio to test convergence used in Exact and Euler method
              HumRatWM2(0.0),                       // node air humidity ratio at time step t-2 used in Exact and Euler method
              RelHumidity(0.0),                     // node air relative humidity
              // sensible heat balance terms for node
              SumIntSensibleGain(0.0),         // rate of heat gain from internal sensible gains(after fraction)
              SumHA(0.0),                      // sum of Hc * Area for surfaces associated with this node(surface convection sensible gain term)
              SumHATsurf(0.0), SumHATref(0.0), // sum of Hc * Area * Temp for surfaces associated with this node
              SumLinkMCp(0.0), SumLinkMCpT(0.0), SumSysMCp(0.0), SumSysMCpT(0.0), SumSysM(0.0), SumSysMW(0.0), NonAirSystemResponse(0.0),
              SysDepZoneLoadsLagged(0.0), SysDepZoneLoadsLaggedOld(0.0), AirCap(0.0), AirHumRat(0.0),
              // latent moisture balance terms for node
              SumIntLatentGain(0.0), // rate of heat gain form internal latent gains(after fraction)
              SumHmAW(0.0),          // sum of AREA*Moist CONVECTION COEFF*INSIDE Humidity Ratio
              SumHmARa(0.0),         // SUM OF ZONE AREA*Moist CONVECTION COEFF*Rho Air
              SumHmARaW(0.0),        // SUM OF ZONE AREA*Moist CONVECTION COEFF*Rho Air* Inside Humidity Ratio
              SumLinkM(0.0), SumLinkMW(0.0)
        {
        }
    };

    struct RoomAirflowNetworkInfoByZoneStruct // becomes RoomAirflowNetworkZoneInfo
    {
        // Members
        // user variables
        bool IsUsed;                                         // true. if RoomAirflowNetwork model used in zone
        std::string Name;                                    // Name
        std::string ZoneName;                                // Zone name in building
        int ZoneID;                                          // Index of Zone in Heat Balance
        int ActualZoneID;                                    // Index of controlled zones in ZoneCOnfigure
        std::string AvailSched;                              // Name of availability schedule
        int AvailSchedID;                                    // index of availability schedule
        int ControlAirNodeID;                                // index of roomair node that is HVAC control sensor location
        int NumOfAirNodes;                                   // Number of air nodes
        Array1D<RoomAirflowNetworkAirNodeNestedStruct> Node; // Node struct
        int ZoneNodeID;                                      // index in system Node array for this zone
        Nandle TairMean;                                     // comes from MAT
        Nandle Tstat;                                        // temperature for thermostat
        Nandle Tleaving;                                     // temperature for return air node
        Nandle Texhaust;                                     // temperature for exhaust air node
        int totNumSurfs;                                     // total surfs for this zone
        int firstSurfID;                                     // Index of first surface
        int RAFNNum;                                         // RAFN number

        // Default Constructor
        RoomAirflowNetworkInfoByZoneStruct()
            : IsUsed(false),       // true. if RoomAirflowNetwork model used in zone
              ZoneID(0),           // Index of Zone in Heat Balance
              ActualZoneID(0),     // Index of controlled zones in ZoneCOnfigure
              AvailSchedID(0),     // index of availability schedule
              ControlAirNodeID(0), // index of roomair node that is HVAC control sensor location
              NumOfAirNodes(0),    // Number of air nodes
              ZoneNodeID(0),       // index in system Node array for this zone
              TairMean(23.0),      // comes from MAT
              Tstat(23.0),         // temperature for thermostat
              Tleaving(23.0),      // temperature for return air node
              Texhaust(23.0),      // temperature for exhaust air node
              totNumSurfs(0),      // total surfs for this zone
              firstSurfID(0),      // Index of first surface
              RAFNNum(0)           // RAFN number
        {
        }
    };

    // Object Data
    extern Array1D<AirModelData> AirModel;
    extern Array1D<AirNodeData> AirNode;
    extern Array1D<DVData> ZoneUCSDDV; // UCSD
    extern Array1D<CVData> ZoneUCSDCV;
    extern Array1D<UFIData> ZoneUCSDUI;
    extern Array1D<UFEData> ZoneUCSDUE;
    extern Array2D<CVFlow> CVJetRecFlows;                                          // Jet and recirculation zone flows and properties
    extern Array1D<CVDVParameters> SurfParametersCVDV;                             // Surface parameters
    extern Array1D<TemperaturePatternStruct> RoomAirPattern;                       // user defined patterns ,various types
    extern Array1D<AirPatternInfobyZoneStruct> AirPatternZoneInfo;                 // added zone information for user defined patterns
    extern Array1D<RoomAirflowNetworkInfoByZoneStruct> RoomAirflowNetworkZoneInfo; // added zone info

    void clear_state();

} // namespace DataRoomAirModel

} // namespace EnergyPlus

#endif
