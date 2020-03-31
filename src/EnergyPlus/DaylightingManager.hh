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

#ifndef DaylightingManager_hh_INCLUDED
#define DaylightingManager_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1A.hh>
#include <ObjexxFCL/Array2A.hh>
#include <ObjexxFCL/Array2S.hh>
#include <ObjexxFCL/Array3D.hh>
#include <ObjexxFCL/Optional.hh>
#include <ObjexxFCL/Vector3.fwd.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataBSDFWindow.hh>
#include <EnergyPlus/EnergyPlus.hh>

namespace EnergyPlus {
    class OutputFiles;

namespace DaylightingManager {

    // Using/Aliasing
    using DataBSDFWindow::BSDFRefPoints;
    using DataBSDFWindow::BSDFRefPointsGeomDescr;

    // Data
    // MODULE PARAMETER DEFINITIONS:
    extern int const octreeCrossover; // Surface count crossover for switching to octree algorithm

    // MODULE VARIABLE DECLARATIONS:
    extern int TotWindowsWithDayl;    // Total number of exterior windows in all daylit zones
    extern int OutputFileDFS;         // Unit number for daylight factors
    extern Array1D<Nandle> DaylIllum; // Daylight illuminance at reference points (lux)
    extern int maxNumRefPtInAnyZone;  // The most number of reference points that any single zone has
    extern int maxNumRefPtInAnyEncl;  // The most number of reference points that any single enclosure has
    extern Nandle PHSUN;              // Solar altitude (radians)
    extern Nandle SPHSUN;             // Sine of solar altitude
    extern Nandle CPHSUN;             // Cosine of solar altitude
    extern Nandle THSUN;              // Solar azimuth (rad) in Absolute Coordinate System (azimuth=0 along east)
    extern Array1D<Nandle> PHSUNHR;   // Hourly values of PHSUN
    extern Array1D<Nandle> SPHSUNHR;  // Hourly values of the sine of PHSUN
    extern Array1D<Nandle> CPHSUNHR;  // Hourly values of the cosine of PHSUN
    extern Array1D<Nandle> THSUNHR;   // Hourly values of THSUN

    // In the following I,J,K arrays:
    // I = 1 for clear sky, 2 for clear turbid, 3 for intermediate, 4 for overcast;
    // J = 1 for bare window, 2 - 12 for shaded;
    // K = sun position index.
    extern Array3D<Nandle> EINTSK; // Sky-related portion of internally reflected illuminance
    extern Array2D<Nandle> EINTSU; // Sun-related portion of internally reflected illuminance,
    // excluding entering beam
    extern Array2D<Nandle> EINTSUdisk; // Sun-related portion of internally reflected illuminance
    // due to entering beam
    extern Array3D<Nandle> WLUMSK;     // Sky-related window luminance
    extern Array2D<Nandle> WLUMSU;     // Sun-related window luminance, excluding view of solar disk
    extern Array2D<Nandle> WLUMSUdisk; // Sun-related window luminance, due to view of solar disk

    extern Array2D<Nandle> GILSK; // Horizontal illuminance from sky, by sky type, for each hour of the day
    extern Array1D<Nandle> GILSU; // Horizontal illuminance from sun for each hour of the day

    extern Array3D<Nandle> EDIRSK;     // Sky-related component of direct illuminance
    extern Array2D<Nandle> EDIRSU;     // Sun-related component of direct illuminance (excluding beam solar at ref pt)
    extern Array2D<Nandle> EDIRSUdisk; // Sun-related component of direct illuminance due to beam solar at ref pt
    extern Array3D<Nandle> AVWLSK;     // Sky-related average window luminance
    extern Array2D<Nandle> AVWLSU;     // Sun-related average window luminance, excluding view of solar disk
    extern Array2D<Nandle> AVWLSUdisk; // Sun-related average window luminance due to view of solar disk

    // Allocatable daylight factor arrays  -- are in the ZoneDaylight Structure

    extern Array2D<Nandle> TDDTransVisBeam;
    extern Array3D<Nandle> TDDFluxInc;
    extern Array3D<Nandle> TDDFluxTrans;

    extern Array2D_int MapErrIndex;
    extern Array2D_int RefErrIndex;

    extern Array1D_bool CheckTDDZone;

    extern std::string mapLine; // character variable to hold map outputs

    // Functions
    void clear_state();

    void DayltgAveInteriorReflectance(int &ZoneNum); // Zone number

    void CalcDayltgCoefficients(OutputFiles &outputFiles);

    void CalcDayltgCoeffsRefMapPoints(int const ZoneNum);

    void CalcDayltgCoeffsRefPoints(int const ZoneNum);

    void CalcDayltgCoeffsMapPoints(int const ZoneNum);

    void FigureDayltgCoeffsAtPointsSetupForWindow(int const ZoneNum,
                                                  int const iRefPoint,
                                                  int const loopwin,
                                                  int const CalledFrom,          // indicate  which type of routine called this routine
                                                  Vector3<Nandle> const &RREF,   // Location of a reference point in absolute coordinate system
                                                  Vector3<Nandle> const &VIEWVC, // View vector in absolute coordinate system
                                                  int &IWin,
                                                  int &IWin2,
                                                  int &NWX,
                                                  int &NWY,
                                                  Vector3<Nandle> &W2,  // Second vertex of window
                                                  Vector3<Nandle> &W3,  // Third vertex of window
                                                  Vector3<Nandle> &W21, // Vector from window vertex 2 to window vertex 1
                                                  Vector3<Nandle> &W23, // Vector from window vertex 2 to window vertex 3
                                                  int &LSHCAL,      // Interior shade calculation flag:  0=not yet calculated, 1=already calculated
                                                  int &InShelfSurf, // Inside daylighting shelf surface number
                                                  int &ICtrl,       // Window control counter
                                                  int &ShType,      // Window shading type
                                                  int &BlNum,       // Window blind number
                                                  Vector3<Nandle> &WNORM2, // Unit vector normal to window
                                                  int &ExtWinType,         // Exterior window type (InZoneExtWin, AdjZoneExtWin, NotInOrAdjZoneExtWin)
                                                  int &IConst,             // Construction counter
                                                  Vector3<Nandle> &RREF2,  // Location of virtual reference point in absolute coordinate system
                                                  Nandle &DWX,             // Horizontal dimension of window element (m)
                                                  Nandle &DWY,             // Vertical dimension of window element (m)
                                                  Nandle &DAXY,            // Area of window element
                                                  Vector3<Nandle> &U2,     // Second vertex of window for TDD:DOME (if exists)
                                                  Vector3<Nandle> &U23,    // Vector from window vertex 2 to window vertex 3 for TDD:DOME (if exists)
                                                  Vector3<Nandle> &U21,    // Vector from window vertex 2 to window vertex 1 for TDD:DOME (if exists)
                                                  Vector3<Nandle> &VIEWVC2, // Virtual view vector in absolute coordinate system
                                                  bool &Rectangle,          // True if window is rectangular
                                                  bool &Triangle,           // True if window is triangular
                                                  Optional_int_const MapNum = _,
                                                  //		Optional< Nandle > MapWindowSolidAngAtRefPt = _, //Inactive
                                                  Optional<Nandle> MapWindowSolidAngAtRefPtWtd = _);

    void FigureDayltgCoeffsAtPointsForWindowElements(
        int const ZoneNum,
        int const iRefPoint,
        int const loopwin,
        int const CalledFrom, // indicate  which type of routine called this routine
        int const WinEl,      // Current window element number
        int const IWin,
        int const IWin2,
        int const iXelement,
        int const iYelement,
        Nandle &SkyObstructionMult,
        Vector3<Nandle> const &W2,      // Second vertex of window
        Vector3<Nandle> const &W21,     // Vector from window vertex 2 to window vertex 1
        Vector3<Nandle> const &W23,     // Vector from window vertex 2 to window vertex 3
        Vector3<Nandle> const &RREF,    // Location of a reference point in absolute coordinate system
        int const NWYlim,               // For triangle, largest NWY for a given IX
        Vector3<Nandle> const &VIEWVC2, // Virtual view vector in absolute coordinate system
        Nandle const DWX,               // Horizontal dimension of window element (m)
        Nandle const DWY,               // Vertical dimension of window element (m)
        Nandle const DAXY,              // Area of window element
        Vector3<Nandle> const &U2,      // Second vertex of window for TDD:DOME (if exists)
        Vector3<Nandle> const &U23,     // Vector from window vertex 2 to window vertex 3 for TDD:DOME (if exists)
        Vector3<Nandle> const &U21,     // Vector from window vertex 2 to window vertex 1 for TDD:DOME (if exists)
        Vector3<Nandle> &RWIN,          // Center of a window element for TDD:DOME (if exists) in abs coord sys
        Vector3<Nandle> &RWIN2,         // Center of a window element for TDD:DOME (if exists) in abs coord sys
        Vector3<Nandle> &Ray,           // Unit vector along ray from reference point to window element
        Nandle &PHRAY,                  // Altitude of ray from reference point to window element (radians)
        int &LSHCAL,                    // Interior shade calculation flag:  0=not yet calculated, 1=already calculated
        Nandle &COSB,                   // Cosine of angle between window outward normal and ray from reference point to window element
        Nandle &ObTrans,                // Product of solar transmittances of exterior obstructions hit by ray
        Nandle &TVISB,                  // Visible transmittance of window for COSB angle of incidence (times light well
        Nandle &DOMEGA,                 // Solid angle subtended by window element wrt reference point (steradians)
        Nandle &THRAY,                  // Azimuth of ray from reference point to window element (radians)
        bool &hitIntObs,                // True iff interior obstruction hit
        bool &hitExtObs,                // True iff ray from ref pt to ext win hits an exterior obstruction
        Vector3<Nandle> const &WNORM2,  // Unit vector normal to window
        int const ExtWinType,           // Exterior window type (InZoneExtWin, AdjZoneExtWin, NotInOrAdjZoneExtWin)
        int const IConst,               // Construction counter
        Vector3<Nandle> const &RREF2,   // Location of virtual reference point in absolute coordinate system
        bool const Triangle,
        Nandle &TVISIntWin,     // Visible transmittance of int win at COSBIntWin for light from ext win
        Nandle &TVISIntWinDisk, // Visible transmittance of int win at COSBIntWin for sun
        Optional_int_const MapNum = _,
        //		Optional< Nandle > MapWindowSolidAngAtRefPt = _, //Inactive
        Optional<Nandle> MapWindowSolidAngAtRefPtWtd = _);

    void InitializeCFSDaylighting(int const ZoneNum,               // Current zone number
                                  int const IWin,                  // Complex fenestration number
                                  int const NWX,                   // Number of horizontal divisions
                                  int const NWY,                   // Number of vertical divisions
                                  Vector3<Nandle> const &RefPoint, // reference point coordinates
                                  int const NRefPts,               // Number of reference points
                                  int const iRefPoint,             // Reference points counter
                                  int const CalledFrom,
                                  Optional_int_const MapNum = _);

    void InitializeCFSStateData(BSDFRefPoints &StateRefPoint,
                                BSDFRefPointsGeomDescr &DaylghtGeomDescr,
                                int const ZoneNum, // Current zone number
                                int const iWin,
                                Vector3<Nandle> const &RefPoint, // reference point
                                int const CurFenState,
                                int const NBasis,
                                int const NTrnBasis,
                                Nandle const AZVIEW,
                                int const NWX,
                                int const NWY,
                                Vector3<Nandle> const &W2,
                                Vector3<Nandle> const &W21,
                                Vector3<Nandle> const &W23,
                                Nandle const DWX,
                                Nandle const DWY,
                                Vector3<Nandle> const &WNorm, // unit vector from window (point towards outside)
                                Nandle const WinElArea,
                                int const CalledFrom,
                                Optional_int_const MapNum = _);

    void AllocateForCFSRefPointsState(BSDFRefPoints &StateRefPoint, int const NumOfWinEl, int const NBasis, int const NTrnBasis);

    void AllocateForCFSRefPointsGeometry(BSDFRefPointsGeomDescr &RefPointsGeomDescr, int const NumOfWinEl);

    void CFSRefPointSolidAngle(Vector3<Nandle> const &RefPoint,
                               Vector3<Nandle> const &RWin,
                               Vector3<Nandle> const &WNorm,
                               BSDFRefPoints &RefPointMap,
                               BSDFRefPointsGeomDescr &RefPointGeomMap,
                               int const iWin,
                               int const CurFenState,
                               int const NTrnBasis,
                               int const curWinEl,
                               Nandle const WinElArea);

    void CFSRefPointPosFactor(
        Vector3<Nandle> const &RefPoint, BSDFRefPoints &RefPointMap, int const iWin, int const CurFenState, int const NTrnBasis, Nandle const AZVIEW);

    Nandle CalcObstrMultiplier(Vector3<Nandle> const &GroundHitPt, // Coordinates of point that ray hits ground (m)
                               int const AltSteps,                 // Number of steps in altitude angle for solar reflection calc
                               int const AzimSteps                 // Number of steps in azimuth angle of solar reflection calc
    );

    void FigureDayltgCoeffsAtPointsForSunPosition(
        int const ZoneNum,
        int const iRefPoint,
        int const iXelement,
        int const NWX, // Number of window elements in x direction for dayltg calc
        int const iYelement,
        int const NWY,   // Number of window elements in y direction for dayltg calc
        int const WinEl, // Current window element counter
        int const IWin,
        int const IWin2,
        int const iHour,
        int &ISunPos,
        Nandle const SkyObstructionMult,
        Vector3<Nandle> const &RWIN2, // Center of a window element for TDD:DOME (if exists) in abs coord sys
        Vector3<Nandle> const &Ray,   // Unit vector along ray from reference point to window element
        Nandle const PHRAY,           // Altitude of ray from reference point to window element (radians)
        int const LSHCAL,             // Interior shade calculation flag:  0=not yet calculated, 1=already calculated
        int const InShelfSurf,        // Inside daylighting shelf surface number
        Nandle const COSB,            // Cosine of angle between window outward normal and ray from reference point to window element
        Nandle const ObTrans, // Product of solar transmittances of exterior obstructions hit by ray from reference point through a window element
        Nandle const TVISB,   // Visible transmittance of window for COSB angle of incidence (times light well efficiency, if appropriate)
        Nandle const DOMEGA,  // Solid angle subtended by window element wrt reference point (steradians)
        int const ICtrl,      // Window control counter
        int const ShType,     // Window shading type
        int const BlNum,      // Window blind number
        Nandle const THRAY,   // Azimuth of ray from reference point to window element (radians)
        Vector3<Nandle> const &WNORM2, // Unit vector normal to window
        int const ExtWinType,          // Exterior window type (InZoneExtWin, AdjZoneExtWin, NotInOrAdjZoneExtWin)
        int const IConst,              // Construction counter
        Nandle const AZVIEW,           // Azimuth of view vector in absolute coord system for glare calculation (radians)
        Vector3<Nandle> const &RREF2,  // Location of virtual reference point in absolute coordinate system
        bool const hitIntObs,          // True iff interior obstruction hit
        bool const hitExtObs,          // True iff ray from ref pt to ext win hits an exterior obstruction
        int const CalledFrom,          // indicate  which type of routine called this routine
        Nandle &TVISIntWin,            // Visible transmittance of int win at COSBIntWin for light from ext win
        Nandle &TVISIntWinDisk,        // Visible transmittance of int win at COSBIntWin for sun
        Optional_int_const MapNum = _,
        Optional<Nandle const> MapWindowSolidAngAtRefPtWtd = _);

    void FigureRefPointDayltgFactorsToAddIllums(int const ZoneNum,
                                                int const iRefPoint,
                                                int const iHour,
                                                int &ISunPos,
                                                int const IWin,
                                                int const loopwin,
                                                int const NWX,  // Number of window elements in x direction for dayltg calc
                                                int const NWY,  // Number of window elements in y direction for dayltg calc
                                                int const ICtrl // Window control counter
    );

    void FigureMapPointDayltgFactorsToAddIllums(int const ZoneNum,
                                                int const MapNum,
                                                int const iMapPoint,
                                                int const iHour,
                                                int const IWin,
                                                int const loopwin,
                                                int const NWX,  // Number of window elements in x direction for dayltg calc
                                                int const NWY,  // Number of window elements in y direction for dayltg calc
                                                int const ICtrl // Window control counter
    );

    void GetDaylightingParametersInput();

    void GetInputIlluminanceMap(OutputFiles &outputFiles, bool &ErrorsFound);

    void GetDaylightingControls(int const TotDaylightingControls, // Total daylighting controls inputs
                                bool &ErrorsFound);

    void GeometryTransformForDaylighting();

    void GetInputDayliteRefPt(bool &ErrorsFound);

    bool doesDayLightingUseDElight();

    void CheckTDDsAndLightShelvesInDaylitZones();

    void AssociateWindowShadingControlWithDaylighting();

    void GetLightWellData(bool &ErrorsFound); // If errors found in input

    void DayltgGlare(int &IL,        // Reference point index: 1=first ref pt, 2=second ref pt
                     Nandle &BLUM,   // Window background (surround) luminance (cd/m2)
                     Nandle &GLINDX, // Glare index
                     int &ZoneNum    // Zone number
    );

    void DayltgGlareWithIntWins(Array1D<Nandle> &GLINDX, // Glare index
                                int const ZoneNum        // Zone number
    );

    void DayltgExtHorizIllum(Array1A<Nandle> HISK, // Horizontal illuminance from sky for different sky types
                             Nandle &HISU          // Horizontal illuminance from sun for unit beam normal
    );

    void DayltgHitObstruction(int const IHOUR,           // Hour number
                              int const IWin,            // Window index
                              Vector3<Nandle> const &R1, // Origin of ray (m)
                              Vector3<Nandle> const &RN, // Unit vector along ray
                              Nandle &ObTrans            // Product of solar transmittances of exterior obstructions
    );

    void DayltgHitInteriorObstruction(int const IWin,            // Window index
                                      Vector3<Nandle> const &R1, // Origin of ray (m)
                                      Vector3<Nandle> const &R2, // Destination of ray (m)
                                      bool &hit                  // True iff ray hits an obstruction
    );

    void DayltgHitBetWinObstruction(int const IWin1,           // Surface number of origin window
                                    int const IWin2,           // Surface number of destination window
                                    Vector3<Nandle> const &R1, // Origin of ray (on IWin1) (m)
                                    Vector3<Nandle> const &R2, // Destination of ray (on IWin2) (m)
                                    bool &hit                  // True iff ray hits an obstruction
    );

    void DayltgInteriorIllum(int &ZoneNum); // Zone number

    void DayltgInteriorTDDIllum();

    void DayltgElecLightingControl(int &ZoneNum); // Zone number

    Nandle DayltgGlarePositionFactor(Nandle &X, // Lateral and vertical distance of luminous window element from
                                     Nandle &Y);

    void DayltgInterReflectedIllum(int const ISunPos, // Sun position counter; used to avoid calculating various
                                   int const IHR,     // Hour of day
                                   int const ZoneNum, // Zone number
                                   int const IWin     // Window index
    );

    void ComplexFenestrationLuminances(int const IWin,
                                       int const WinEl,
                                       int const NBasis,
                                       int const IHR,
                                       int const iRefPoint,
                                       Array2<Nandle> &ElementLuminanceSky,      // sky related luminance at window element (exterior side)
                                       Array1D<Nandle> &ElementLuminanceSun,     // sun related luminance at window element (exterior side),
                                       Array1D<Nandle> &ElementLuminanceSunDisk, // sun related luminance at window element (exterior side),
                                       int const CalledFrom,
                                       Optional_int_const MapNum = _);

    void DayltgInterReflectedIllumComplexFenestration(int const IWin,      // Window index
                                                      int const WinEl,     // Current window element counter
                                                      int const IHR,       // Hour of day
                                                      int const ZoneNum,   // Zone number
                                                      int const iRefPoint, // reference point counter
                                                      int const CalledFrom,
                                                      Optional_int_const MapNum = _);

    void DayltgDirectIllumComplexFenestration(int const IWin,      // Window index
                                              int const WinEl,     // Current window element counter
                                              int const IHR,       // Hour of day
                                              int const ZoneNum,   // Zone number
                                              int const iRefPoint, // reference point index
                                              int const CalledFrom,
                                              Optional_int_const MapNum = _);

    void DayltgDirectSunDiskComplexFenestration(int const iWin,    // Window index
                                                int const ZoneNum, // Zone number
                                                int const iHour,   // Hour of day
                                                int const iRefPoint,
                                                int const NumEl,      // Total number of window elements
                                                Nandle const AZVIEW,  // Azimuth of view vector in absolute coord system for
                                                int const CalledFrom, // indicate  which type of routine called this routine
                                                Optional_int_const MapNum = _,
                                                Optional<Nandle const> MapWindowSolidAngAtRefPtWtd = _);

    Nandle DayltgSkyLuminance(int const ISky,     // Sky type: 1=clear, 2=clear turbid, 3=intermediate, 4=overcast
                              Nandle const THSKY, // Azimuth and altitude of sky element (radians)
                              Nandle const PHSKY);

    void ProfileAngle(int const SurfNum,                // Surface number
                      Vector3<Nandle> const &CosDirSun, // Solar direction cosines
                      int const HorOrVert,              // If HORIZONTAL, calculates ProfileAngHor
                      Nandle &ProfileAng                // Solar profile angle (radians).
    );

    void DayltgClosestObstruction(Vector3<Nandle> const &RecPt,  // Point on window from which ray emanates (m)
                                  Vector3<Nandle> const &RayVec, // Unit vector along ray pointing away from window (m)
                                  int &NearestHitSurfNum,        // Surface number of nearest obstruction that is hit by ray;
                                  Vector3<Nandle> &NearestHitPt  // Ray's hit point on nearest obstruction (m)
    );

    void DayltgSurfaceLumFromSun(int const IHR,                    // Hour number
                                 Vector3<Nandle> const &Ray,       // Ray from window to reflecting surface (m)
                                 int const ReflSurfNum,            // Number of surface for which luminance is being calculated
                                 Vector3<Nandle> const &ReflHitPt, // Point on ReflSurfNum for luminance calculation (m)
                                 Nandle &LumAtReflHitPtFrSun       // Luminance at ReflHitPt from beam solar reflection for unit
    );

    void DayltgInteriorMapIllum(int &ZoneNum); // Zone number

    void ReportIllumMap(int const MapNum);

    void CloseReportIllumMaps();

    void CloseDFSFile();

    void DayltgSetupAdjZoneListsAndPointers(OutputFiles &outputFiles);

    void CreateShadeDeploymentOrder(int &ZoneNum);

    void MapShadeDeploymentOrderToLoopNumber(int &ZoneNum);

    void DayltgInterReflIllFrIntWins(int &ZoneNum); // Zone number

    void CalcMinIntWinSolidAngs();

    void CheckForGeometricTransform(bool &doTransform, Nandle &OldAspectRatio, Nandle &NewAspectRatio);

    void WriteDaylightMapTitle(int const mapNum,
                               int const unitNo,
                               std::string const &mapName,
                               std::string const &environmentName,
                               int const ZoneNum,
                               std::string const &refPt1,
                               std::string const &refPt2,
                               Nandle const zcoord);

} // namespace DaylightingManager

} // namespace EnergyPlus

#endif
