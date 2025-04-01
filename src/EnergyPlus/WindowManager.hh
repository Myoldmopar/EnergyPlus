// EnergyPlus, Copyright (c) 1996-2025, The Board of Trustees of the University of Illinois,
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

#ifndef WindowManager_hh_INCLUDED
#define WindowManager_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1A.hh>
#include <ObjexxFCL/Array1D.hh>
#include <ObjexxFCL/Array2A.hh>

// EnergyPlus Headers
#include <EnergyPlus/Data/BaseData.hh>
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/DataSurfaces.hh>
#include <EnergyPlus/EnergyPlus.hh>
#include <EnergyPlus/WindowEquivalentLayer.hh>
#include <EnergyPlus/WindowManagerExteriorData.hh>
#include <EnergyPlus/WindowModel.hh>

namespace EnergyPlus {

// Forward declarations
struct EnergyPlusData;

namespace Window {

    int constexpr nume = 107; // Number of wavelength values in solar spectrum
    int constexpr numt3 = 81; // Number of wavelength values in the photopic response

    int constexpr maxGlassLayers = 5;
    int constexpr maxGapLayers = 5;
    int constexpr maxSpectralDataElements = 800; // Maximum number in Spectral Data arrays.

    int constexpr numPhis = 10;
    double constexpr dPhiDeg = 10.0;
    double constexpr dPhiRad = dPhiDeg * Constant::DegToRad;

    constexpr std::array<double, numPhis> cosPhis = {1.0,
                                                     0.98480775301220802,
                                                     0.93969262078590842,
                                                     0.86602540378443871,
                                                     0.76604444311897812,
                                                     0.64278760968653936,
                                                     0.50000000000000011,
                                                     0.34202014332566882,
                                                     0.17364817766693041,
                                                     0.0}; // 6.123233995736766E-17

    constexpr int maxPolyCoef = 6;

    class CWindowModel;
    class CWindowOpticalModel;
    class CWindowConstructionsSimplified;

    void InitWindowOpticalCalculations(EnergyPlusData &state);

    void InitGlassOpticalCalculations(EnergyPlusData &state);

    void W5InitGlassParameters(EnergyPlusData &state);

    //****************************************************************************
    // WINDOW 5 Optical Calculation Subroutines
    //****************************************************************************

    void SystemSpectralPropertiesAtPhi(EnergyPlusData &state,
                                       int iquasi,   // When there is no spectral data, this is the wavelength
                                       int ngllayer, // Number of glass layers in construction
                                       double wlbot, // Lowest and highest wavelength considered
                                       double wltop,
                                       std::array<int, maxGlassLayers> const &numpt,
                                       std::array<std::array<double, maxSpectralDataElements>, maxGlassLayers> const &wlt,
                                       std::array<std::array<double, maxSpectralDataElements>, maxGlassLayers> const &tPhi,
                                       std::array<std::array<double, maxSpectralDataElements>, maxGlassLayers> const &rfPhi,
                                       std::array<std::array<double, maxSpectralDataElements>, maxGlassLayers> const &rbPhi,
                                       std::array<double, nume> &stPhi,
                                       std::array<double, nume> &srfPhi,
                                       std::array<double, nume> &srbPhi,
                                       Array2D<double> &saPhi);

    void SystemPropertiesAtLambdaAndPhi(EnergyPlusData &state,
                                        int n,       // Number of glass layers
                                        double &tt,  // System transmittance
                                        double &rft, // System front and back reflectance
                                        double &rbt,
                                        Array1A<double> aft // System absorptance of each glass layer
    );

    double solarSpectrumAverage(EnergyPlusData const &state, gsl::span<double const> p);

    double visibleSpectrumAverage(EnergyPlusData const &state, gsl::span<double const> p);

    double Interpolate(gsl::span<double const> x, // Array of data points for independent variable
                       gsl::span<double const> y, // Array of data points for dependent variable
                       int npts,                  // Number of data pairs
                       double xin                 // Given value of x
    );

    void CalcWindowHeatBalance(EnergyPlusData &state,
                               int SurfNum,            // Surface number
                               double HextConvCoeff,   // Outside air film conductance coefficient
                               double &SurfInsideTemp, // Inside window surface temperature
                               double &SurfOutsideTemp // Outside surface temperature (C)
    );

    void CalcWindowHeatBalanceInternalRoutines(EnergyPlusData &state,
                                               int SurfNum,            // Surface number
                                               double HextConvCoeff,   // Outside air film conductance coefficient
                                               double &SurfInsideTemp, // Inside window surface temperature
                                               double &SurfOutsideTemp // Outside surface temperature (C)
    );

    void GetHeatBalanceEqCoefMatrixSimple(EnergyPlusData &state,
                                          int nglasslayer,           // Number of glass layers
                                          Array1D<double> const &hr, // Radiative conductance (W/m2-K)
                                          Array1A<double> &hgap,     // Gap gas conductive conductance (W/m2-K)
                                          Array2D<double> &Aface,    // Coefficient in equation Aface*thetas = Bface
                                          Array1D<double> &Bface     // Coefficient in equation Aface*thetas = Bface
    );

    void GetHeatBalanceEqCoefMatrix(EnergyPlusData &state,
                                    int SurfNum,
                                    int nglasslayer,
                                    DataSurfaces::WinShadingType ShadeFlag,
                                    double sconsh,
                                    double TauShIR,
                                    double EpsShIR1,
                                    double EpsShIR2,
                                    double RhoShIR1,
                                    double RhoShIR2,
                                    double ShGlReflFacIR,
                                    double RhoGlIR1,
                                    double RhoGlIR2,
                                    double hcv,                   // Convection coefficient from gap glass or shade/blind to gap air (W/m2-K)
                                    double TGapNew,               // Current-iteration average air temp in airflow gap (K)
                                    double TAirflowGapNew,        // Average air temp in airflow gap between glass panes (K)
                                    double hcvAirflowGap,         // Convection coefficient from airflow gap glass to airflow gap air (W/m2-K)
                                    Array1A<double> const &hcvBG, // Convection coefficient from gap glass or shade to gap gas (W/m2-K)
                                    Array1A<double> const &TGapNewBG,
                                    Array1A<double> const &AbsRadShadeFace,
                                    Array1D<double> const &hr,
                                    Array2D<double> &Aface,
                                    Array1D<double> &Bface);

    void SolveForWindowTemperatures(EnergyPlusData &state, int SurfNum); // Surface number

    void ExtOrIntShadeNaturalFlow(EnergyPlusData &state,
                                  int SurfNum,        // Surface number
                                  int iter,           // Iteration number for glass heat balance calculation
                                  double &VGap,       // Air velocity in glass-shade/blind gap (m/s)
                                  double &TGapNew,    // Current-iteration average air temp in glass-shade/blind gap (K)
                                  double &TGapOutlet, // Temperature of air leaving glass-shade/blind gap at top for upward
                                  double &hcv,        // Convection coefficient from gap glass or shade to gap air (W/m2-K)
                                  double &QConvGap    // Convective heat gain from glass-shade/blind gap for interior shade (W)
    );

    void BetweenGlassShadeNaturalFlow(EnergyPlusData &state,
                                      int SurfNum,             // Surface number
                                      int iter,                // Iteration number for glass heat balance calculation
                                      double &VGap,            // Gas velocity in gaps (m/s)
                                      Array1A<double> TGapNew, // Current-iteration average gas temp in gaps (K)
                                      Array1A<double> hcv      // Convection coefficient from gap glass or shade to gap gas (W/m2-K)
    );

    void BetweenGlassForcedFlow(EnergyPlusData &state,
                                int SurfNum,        // Surface number
                                int iter,           // Iteration number for glass heat balance calculation
                                double &VGap,       // Air velocity in airflow gap (m/s)
                                double &TGapNew,    // Current-iteration average air temp in airflow gap (K)
                                double &TGapOutlet, // Temperature of air leaving glass-shade/blind gap at top for upward
                                double &hcv,        // Convection coefficient from gap glass faces to gap air (W/m2-K)
                                double &QConvGap    // Convective heat gain from air flow gap (W)
    );

    void BetweenGlassShadeForcedFlow(EnergyPlusData &state,
                                     int SurfNum,             // Surface number
                                     int iter,                // Iteration number for glass heat balance calculation
                                     double &VGap,            // Air velocity in each gap (m/s)
                                     Array1A<double> TGapNew, // Current-iteration average gas temp in gaps (K)
                                     double &TGapOutletAve,   // Average of TGapOutlet(1) and TGapOutlet(2) (K)
                                     Array1A<double> hcv,     // Convection coefficient from gap glass or shade to gap gas (W/m2-K)
                                     double &QConvTot         // Sum of convective heat flow from gaps (W)
    );

    void LUdecomposition(EnergyPlusData &state,
                         Array2<double> &ajac, // As input: matrix to be decomposed;
                         int n,                // Dimension of matrix
                         Array1D_int &indx,    // Vector of row permutations
                         int &d                // +1 if even number of row interchange is even, -1
    );

    void LUsolution(EnergyPlusData &state,
                    Array2<double> const &a, // Matrix and vector in a.x = b;
                    int n,                   // Dimension of a and b
                    Array1D_int const &indx, // Vector of row permutations
                    Array1D<double> &b       // Matrix and vector in a.x = b;
    );

    constexpr double POLYF(double const X,                // Cosine of angle of incidence
                           std::array<double, 6> const &A // Polynomial coefficients
    )
    {
        return (X < 0.0 || X > 1.0) ? 0.0 : (X * (A[0] + X * (A[1] + X * (A[2] + X * (A[3] + X * (A[4] + X * A[5]))))));
    }

    constexpr Real64 POLYF(Real64 const X,                // Cosine of angle of incidence
                           std::array<Real64, 6> const &A // Polynomial coefficients
    )
    {
        return (X < 0.0 || X > 1.0) ? 0.0 : (X * (A[0] + X * (A[1] + X * (A[2] + X * (A[3] + X * (A[4] + X * A[5]))))));
    }

#ifdef GET_OUT
    constexpr double POLYF(double const X,          // Cosine of angle of incidence
                           Array1D<double> const &A // Polynomial coefficients
    )
    {
        if (X < 0.0 || X > 1.0) {
            return 0.0;
        } else {
            return X * (A(1) + X * (A(2) + X * (A(3) + X * (A(4) + X * (A(5) + X * A(6))))));
        }
    }
#endif // GET_OUT

    void WindowGasConductance(EnergyPlusData &state,
                              double tleft,  // Temperature of gap surface closest to outside (K)
                              double tright, // Temperature of gap surface closest to zone (K)
                              int IGap,      // Gap number
                              double &con,   // Gap gas conductance (W/m2-K)
                              double &pr,    // Gap gas Prandtl number
                              double &gr     // Gap gas Grashof number
    );

    void WindowGasPropertiesAtTemp(EnergyPlusData const &state,
                                   double tmean, // Temperature of gas in gap (K)
                                   int IGap,     // Gap number
                                   double &dens, // Gap gas density at tmean (kg/m3)
                                   double &visc  // Gap gas dynamic viscosity at tmean (g/m-s)
    );

    void StartingWindowTemps(EnergyPlusData &state,
                             int SurfNum,                // Surface number
                             Array1A<double> AbsRadShade // Short-wave radiation absorbed by shade/blind faces
    );

    void NusseltNumber(EnergyPlusData &state,
                       int SurfNum, // Surface number
                       double tso,  // Temperature of gap surface closest to outside (K)
                       double tsi,  // Temperature of gap surface closest to zone (K)
                       int IGap,    // Gap number
                       double gr,   // Gap gas Grashof number
                       double pr,   // Gap gas Prandtl number
                       double &gnu  // Gap gas Nusselt number
    );

    void TransAndReflAtPhi(double cs,                // Cosine of incidence angle
                           double tf0,               // Transmittance at zero incidence angle
                           double rf0,               // Front reflectance at zero incidence angle
                           double rb0,               // Back reflectance at zero incidence angle
                           double &tfp,              // Transmittance at cs
                           double &rfp,              // Front reflectance at cs
                           double &rbp,              // Back reflectance at cs
                           bool SimpleGlazingSystem, // .TRUE. if simple block model being used
                           double SimpleGlazingSHGC, // SHGC value to use in alternate model for simple glazing system
                           double SimpleGlazingU     // U-factor value to use in alternate model for simple glazing system
    );

    double InterpolateBetweenTwoValues(double X, double X0, double X1, double F0, double F1);

    double InterpolateBetweenFourValues(
        double X, double Y, double X1, double X2, double Y1, double Y2, double Fx1y1, double Fx1y2, double Fx2y1, double Fx2y2);

    void W5LsqFit(std::array<double, numPhis> const &ivars,       // Independent variables
                  std::array<double, numPhis> const &dvars,       // Dependent variables
                  std::array<double, Window::maxPolyCoef> &coeffs // Polynomial coeffients from fit
    );

    double DiffuseAverage(std::array<double, numPhis> const &props); // Property value at angles of incidence

    void CalcWinFrameAndDividerTemps(EnergyPlusData &state,
                                     int SurfNum,     // Surface number
                                     double tout,     // Outside air temperature (K)
                                     double tin,      // Inside air temperature (K)
                                     double HOutConv, // Outside convective air film conductance (W/m2-K)
                                     double HInConv,  // Inside convective air film conductance (W/m2-K)
                                     double Outir,    // Exterior IR irradiance from sky and ground
                                     int ConstrNum    // Construction number of window
    );

    void CalcNominalWindowCond(EnergyPlusData &state,
                               int ConstrNum,              // Construction number
                               int WinterSummerFlag,       // 1=winter, 2=summer
                               double &NominalConductance, // Nominal center-of-glass conductance, including air films
                               double &SHGC,               // Nominal center-of-glass solar heat gain coefficient for
                               double &TSolNorm,           // Overall beam solar transmittance at normal incidence
                               double &TVisNorm,           // Overall beam visible transmittance at normal incidence
                               int &errFlag                // Error flag
    );

    void EvalNominalWindowCond(EnergyPlusData &state,
                               double AbsBeamShadeNorm,     // Shade solar absorptance at normal incidence
                               Array1D<double> AbsBeamNorm, // Beam absorptance at normal incidence for each glass layer
                               Array1D<double> hgap,        // Conductive gap conductance [W/m2-K]
                               double &NominalConductance,  // Nominal center-of-glass conductance, including air films
                               double &SHGC,                // Nominal center-of-glass solar heat gain coefficient for
                               double TSolNorm              // Overall beam solar transmittance at normal incidence
    );

    void WindowTempsForNominalCond(EnergyPlusData &state,
                                   int ConstrNum,        // Construction number
                                   Array1A<double> hgap, // Gap gas conductive conductance (W/m2-K)
                                   double adjRatio       // adjusment Ratio to hcin
    );

    void StartingWinTempsForNominalCond(EnergyPlusData &state);

    void ReportGlass(EnergyPlusData &state);

    void CalcWindowBlindProperties(EnergyPlusData &state);

    void CalcWindowScreenProperties(EnergyPlusData &state);

    void BlindOpticsDiffuse(EnergyPlusData &state,
                            int BlindNum,      // Blind number
                            int ISolVis,       // 1 = solar and IR calculation; 2 = visible calculation
                            Array1A<double> c, // Slat properties
                            double b_el,       // Slat elevation (radians)
                            Array1A<double> p  // Blind properties
    );

    void BlindOpticsBeam(EnergyPlusData &state,
                         int BlindNum,      // Blind number
                         Array1A<double> c, // Slat properties (equivalent to BLD_PR)
                         double b_el,       // Slat elevation (radians)
                         double s_el,       // Solar profile angle (radians)
                         Array1A<double> p  // Blind properties (equivalent to ST_LAY)
    );

    inline double InterpSw(double const SwitchFac, // Switching factor: 0.0 if glazing is unswitched, = 1.0 if fully switched
                           double const A,         // Glazing property in unswitched state
                           double const B          // Glazing property in fully switched state
    )
    {
        // FUNCTION INFORMATION:
        //       AUTHOR         Fred Winkelmann
        //       DATE WRITTEN   February 1999

        // PURPOSE OF THIS FUNCTION:
        // For switchable glazing, calculates a weighted average of properties
        // A and B

        double locSwitchFac = std::clamp(SwitchFac, 0.0, 1.0);

        return (1.0 - locSwitchFac) * A + locSwitchFac * B;
    }

    void ViewFac(double s,         // Slat width (m)
                 double h,         // Distance between faces of adjacent slats (m)
                 double phib,      // Elevation angle of normal to slat (radians)
                 double phis,      // Profile angle of radiation source (radians)
                 Array2A<double> F // View factor array
    );

    void InvertMatrix(EnergyPlusData &state,
                      Array2D<double> &a, // Matrix to be inverted
                      Array2D<double> &y, // Inverse of matrix a
                      Array1D_int &indx,  // Index vector for LU decomposition
                      int n);

    // added for custom solar or visible spectrum
    void CheckAndReadCustomSprectrumData(EnergyPlusData &state);

    void initWindowModel(EnergyPlusData &state);

    struct WindowGap
    {
        int numGases = 0;
        std::array<Material::Gas, Material::maxMixGases> gases = {Material::Gas()};
        std::array<double, Material::maxMixGases> gasFracts = {0.0};
        double width = 0.0;
    };

} // namespace Window

struct WindowManagerData : BaseGlobalStruct
{

    //                                      Dens  dDens/dT  Con    dCon/dT   Vis    dVis/dT Prandtl dPrandtl/dT
    std::array<double, 8> const AirProps = {1.29, -0.4e-2, 2.41e-2, 7.6e-5, 1.73e-5, 1.0e-7, 0.72, 1.8e-3};

    // Air mass 1.5 terrestrial solar global spectral irradiance (W/m2-micron)
    // on a 37 degree tilted surface; corresponds
    // to wavelengths (microns) in following data block (ISO 9845-1 and ASTM E 892;
    // derived from Optics5 data file ISO-9845GlobalNorm.std, 10-14-99)

    // Solar spectrum wavelength values (microns)
    std::array<double, Window::nume> wle = {
        0.3000, 0.3050, 0.3100, 0.3150, 0.3200, 0.3250, 0.3300, 0.3350, 0.3400, 0.3450, 0.3500, 0.3600, 0.3700, 0.3800, 0.3900, 0.4000,
        0.4100, 0.4200, 0.4300, 0.4400, 0.4500, 0.4600, 0.4700, 0.4800, 0.4900, 0.5000, 0.5100, 0.5200, 0.5300, 0.5400, 0.5500, 0.5700,
        0.5900, 0.6100, 0.6300, 0.6500, 0.6700, 0.6900, 0.7100, 0.7180, 0.7244, 0.7400, 0.7525, 0.7575, 0.7625, 0.7675, 0.7800, 0.8000,
        0.8160, 0.8237, 0.8315, 0.8400, 0.8600, 0.8800, 0.9050, 0.9150, 0.9250, 0.9300, 0.9370, 0.9480, 0.9650, 0.9800, 0.9935, 1.0400,
        1.0700, 1.1000, 1.1200, 1.1300, 1.1370, 1.1610, 1.1800, 1.2000, 1.2350, 1.2900, 1.3200, 1.3500, 1.3950, 1.4425, 1.4625, 1.4770,
        1.4970, 1.5200, 1.5390, 1.5580, 1.5780, 1.5920, 1.6100, 1.6300, 1.6460, 1.6780, 1.7400, 1.8000, 1.8600, 1.9200, 1.9600, 1.9850,
        2.0050, 2.0350, 2.0650, 2.1000, 2.1480, 2.1980, 2.2700, 2.3600, 2.4500, 2.4940, 2.5370};

    // Solar spectrum values corresponding to wle
    std::array<double, Window::nume> e = {
        0.0,    9.5,    42.3,   107.8,  181.0,  246.0,  395.3,  390.1,  435.3,  438.9,  483.7,  520.3,  666.2,  712.5,  720.7,  1013.1,
        1158.2, 1184.0, 1071.9, 1302.0, 1526.0, 1599.6, 1581.0, 1628.3, 1539.2, 1548.7, 1586.5, 1484.9, 1572.4, 1550.7, 1561.5, 1501.5,
        1395.5, 1485.3, 1434.1, 1419.9, 1392.3, 1130.0, 1316.7, 1010.3, 1043.2, 1211.2, 1193.9, 1175.5, 643.1,  1030.7, 1131.1, 1081.6,
        849.2,  785.0,  916.4,  959.9,  978.9,  933.2,  748.5,  667.5,  690.3,  403.6,  258.3,  313.6,  526.8,  646.4,  746.8,  690.5,
        637.5,  412.6,  108.9,  189.1,  132.2,  339.0,  460.0,  423.6,  480.5,  413.1,  250.2,  32.5,   1.6,    55.7,   105.1,  105.5,
        182.1,  262.2,  274.2,  275.0,  244.6,  247.4,  228.7,  244.5,  234.8,  220.5,  171.5,  30.7,   2.0,    1.2,    21.2,   91.1,
        26.8,   99.5,   60.4,   89.1,   82.2,   71.5,   70.2,   62.0,   21.2,   18.5,   3.2};

    // Phototopic response function and corresponding wavelengths (microns)
    // (CIE 1931 observer; ISO/CIE 10527, CIE Standard Calorimetric Observers;
    // derived from Optics5 data file "CIE 1931 Color Match from E308.txt", which is
    // the same as WINDOW4 file Cie31t.dat)
    // Wavelength values for photopic response
    std::array<double, Window::numt3> wlt3 = {0.380, 0.385, 0.390, 0.395, 0.400, 0.405, 0.410, 0.415, 0.420, 0.425, 0.430, 0.435, 0.440, 0.445,
                                              0.450, 0.455, 0.460, 0.465, 0.470, 0.475, 0.480, 0.485, 0.490, 0.495, 0.500, 0.505, 0.510, 0.515,
                                              0.520, 0.525, 0.530, 0.535, 0.540, 0.545, 0.550, 0.555, 0.560, 0.565, 0.570, 0.575, 0.580, 0.585,
                                              0.590, 0.595, 0.600, 0.605, 0.610, 0.615, 0.620, 0.625, 0.630, 0.635, 0.640, 0.645, 0.650, 0.655,
                                              0.660, 0.665, 0.670, 0.675, 0.680, 0.685, 0.690, 0.695, 0.700, 0.705, 0.710, 0.715, 0.720, 0.725,
                                              0.730, 0.735, 0.740, 0.745, 0.750, 0.755, 0.760, 0.765, 0.770, 0.775, 0.780};

    // Photopic response corresponding to wavelengths in wlt3
    std::array<double, Window::numt3> y30 = {
        0.0000, 0.0001, 0.0001, 0.0002, 0.0004, 0.0006, 0.0012, 0.0022, 0.0040, 0.0073, 0.0116, 0.0168, 0.0230, 0.0298, 0.0380, 0.0480, 0.0600,
        0.0739, 0.0910, 0.1126, 0.1390, 0.1693, 0.2080, 0.2586, 0.3230, 0.4073, 0.5030, 0.6082, 0.7100, 0.7932, 0.8620, 0.9149, 0.9540, 0.9803,
        0.9950, 1.0000, 0.9950, 0.9786, 0.9520, 0.9154, 0.8700, 0.8163, 0.7570, 0.6949, 0.6310, 0.5668, 0.5030, 0.4412, 0.3810, 0.3210, 0.2650,
        0.2170, 0.1750, 0.1382, 0.1070, 0.0816, 0.0610, 0.0446, 0.0320, 0.0232, 0.0170, 0.0119, 0.0082, 0.0158, 0.0041, 0.0029, 0.0021, 0.0015,
        0.0010, 0.0007, 0.0005, 0.0004, 0.0002, 0.0002, 0.0001, 0.0001, 0.0001, 0.0000, 0.0000, 0.0000, 0.0000};

    int ngllayer = 0;                                                                   // Number of glass layers
    int nglface = 0;                                                                    // Number of glass faces
    int nglfacep = 0;                                                                   // Number of glass faces, + 2 if shade layer present
    double tout = 0.0;                                                                  // Outside air temperature (K)
    double tin = 0.0;                                                                   // Inside air temperature (previous timestep) (K)
    double tilt = 0.0;                                                                  // Window tilt (deg)
    double tiltr = 0.0;                                                                 // Window tilt (radians)
    double hcin = 0.0;                                                                  // Convective inside air film conductance (W/m2-K)
    double hcout = 0.0;                                                                 // Convective outside air film conductance (W/m2-K)
    double Ebout = 0.0;                                                                 // Sigma*(outside air temp)**4 (W/m2)
    double Outir = 0.0;                                                                 // IR radiance of window's exterior surround (W/m2)
    double Rmir = 0.0;                                                                  // IR radiance of window's interior surround (W/m2)
    double Rtot = 0.0;                                                                  // Total thermal resistance of window (m2-K/W)
    std::array<Window::WindowGap, Window::maxGlassLayers> gaps = {Window::WindowGap()}; // Gas thermal conductivity coefficients for each gap
    std::array<double, Window::maxGlassLayers> thick = {0.0};                           // Glass layer thickness (m)
    std::array<double, Window::maxGlassLayers> scon = {0.0};                            // Glass layer conductance--conductivity/thickness (W/m2-K)

    std::array<double, 2 *Window::maxGlassLayers> tir = {0.0};  // Front and back IR transmittance for each glass layer
    std::array<double, 2 *Window::maxGlassLayers> emis = {0.0}; // Front and back IR emissivity for each glass layer
    std::array<double, 2 *Window::maxGlassLayers> rir = {0.0};  // Front and back IR reflectance for each glass layer
                                                                //  (program calculates from tir and emis)
    std::array<double, 2 *Window::maxGlassLayers> AbsRadGlassFace = {
        0.0};                                                         // Solar radiation and IR radiation from internal gains absorbed by glass face
    std::array<double, 2 *Window::maxGlassLayers> thetas = {0.0};     // Glass surface temperatures (K)
    std::array<double, 2 *Window::maxGlassLayers> thetasPrev = {0.0}; // Previous-iteration glass surface temperatures (K)

    std::array<double, Window::maxGlassLayers> hrgap = {0.0}; // Radiative gap conductance

    double A23P = 0.0; // Intermediate variables in glass face
    double A32P = 0.0;
    double A45P = 0.0;
    double A54P = 0.0;
    double A67P = 0.0;
    double A76P = 0.0;
    double A23 = 0.0; // heat balance equations
    double A45 = 0.0;
    double A67 = 0.0;

    // TEMP MOVED FROM DataHeatBalance.hh -BLB

    // for each wavelenth in wle
    std::array<std::array<double, Window::maxGlassLayers>, Window::maxGlassLayers> top = {0.0};  // Transmittance matrix for subr. op
    std::array<std::array<double, Window::maxGlassLayers>, Window::maxGlassLayers> rfop = {0.0}; // Front reflectance matrix for subr. op
    std::array<std::array<double, Window::maxGlassLayers>, Window::maxGlassLayers> rbop = {0.0}; // Back transmittance matrix for subr. op

    std::unique_ptr<Window::CWindowModel> inExtWindowModel;       // Information about windows model (interior or exterior)
    std::unique_ptr<Window::CWindowOpticalModel> winOpticalModel; // Information about windows optical model (Simplified or BSDF)

    bool RunMeOnceFlag = false;
    bool lSimpleGlazingSystem = false; // true if using simple glazing system block model
    bool BGFlag = false;               // True if between-glass shade or blind
    bool locTCFlag = false;            // True if this surface is a TC window
    bool DoReport = false;
    bool HasWindows = false;
    bool HasComplexWindows = false;
    bool HasEQLWindows = false;     // equivalent layer window defined
    double SimpleGlazingSHGC = 0.0; // value of SHGC for simple glazing system block model
    double SimpleGlazingU = 0.0;    // value of U-factor for simple glazing system block model
    double tmpTrans = 0.0;          // solar transmittance calculated from spectral data
    double tmpTransVis = 0.0;       // visible transmittance calculated from spectral data
    double tmpReflectSolBeamFront = 0.0;
    double tmpReflectSolBeamBack = 0.0;
    double tmpReflectVisBeamFront = 0.0;
    double tmpReflectVisBeamBack = 0.0;

    std::array<int, Window::maxGlassLayers> LayerNum = {0}; // Glass layer number

    void init_constant_state([[maybe_unused]] EnergyPlusData &state) override
    {
    }

    void init_state([[maybe_unused]] EnergyPlusData &state) override
    {
    }

    void clear_state() override
    {
        this->ngllayer = 0;
        this->nglface = 0;
        this->nglfacep = 0;
        this->tout = 0.0;
        this->tin = 0.0;
        this->tilt = 0.0;
        this->tiltr = 0.0;
        this->hcin = 0.0;
        this->hcout = 0.0;
        this->Ebout = 0.0;
        this->Outir = 0.0;
        this->Rmir = 0.0;
        this->Rtot = 0.0;
        this->gaps = {Window::WindowGap()};
        this->thick = {0.0};
        this->scon = {0.0};
        this->tir = {0.0};
        this->emis = {0.0};
        this->rir = {0.0};
        this->AbsRadGlassFace = {0.0};
        this->thetas = {0.0};
        this->thetasPrev = {0.0};
        this->hrgap = {0.0};
        this->A23P = 0.0;
        this->A32P = 0.0;
        this->A45P = 0.0;
        this->A54P = 0.0;
        this->A67P = 0.0;
        this->A76P = 0.0;
        this->A23 = 0.0;
        this->A45 = 0.0;
        this->A67 = 0.0;
        this->top = {0.0};
        this->rfop = {0.0};
        this->rbop = {0.0};
        Window::CWindowConstructionsSimplified::clearState();
        this->RunMeOnceFlag = false;
        this->lSimpleGlazingSystem = false; // true if using simple glazing system block model
        this->BGFlag = false;               // True if between-glass shade or blind
        this->locTCFlag = false;            // True if this surface is a TC window
        this->DoReport = false;
        this->HasWindows = false;
        this->HasComplexWindows = false;
        this->HasEQLWindows = false; // equivalent layer window defined
        this->SimpleGlazingSHGC = 0.0;
        this->SimpleGlazingU = 0.0;
        this->tmpTrans = 0.0;    // solar transmittance calculated from spectral data
        this->tmpTransVis = 0.0; // visible transmittance calculated from spectral data
        this->tmpReflectSolBeamFront = 0.0;
        this->tmpReflectSolBeamBack = 0.0;
        this->tmpReflectVisBeamFront = 0.0;
        this->tmpReflectVisBeamBack = 0.0;
    }

    // Default Constructor
    WindowManagerData()
    {
        SimpleGlazingSHGC = 0.0;
        SimpleGlazingU = 0.0;
        tmpReflectSolBeamFront = 0.0;
        tmpReflectSolBeamBack = 0.0;
        tmpReflectVisBeamFront = 0.0;
        tmpReflectVisBeamBack = 0.0;
    }
};

} // namespace EnergyPlus

#endif
