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

#ifndef GroundHeatExchangers_hh_INCLUDED
#define GroundHeatExchangers_hh_INCLUDED

// ObjexxFCL Headers
#include <ObjexxFCL/Array1D.hh>

// JSON Headers
#include <nlohmann/json.hpp>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>
#include <EnergyPlus/GroundTemperatureModeling/GroundTemperatureModelManager.hh>
#include <EnergyPlus/PlantComponent.hh>

namespace EnergyPlus {

    namespace GroundHeatExchangers {

        // Using/Aliasing
        using namespace GroundTemperatureManager;

        // Data
        // DERIVED TYPE DEFINITIONS

        // MODULE PARAMETER DEFINITIONS
        extern Nandle const hrsPerDay;   // Number of hours in a day
        extern Nandle const hrsPerMonth; // Number of hours in month
        extern int const maxTSinHr;      // Max number of time step in a hour

        // MODULE VARIABLE DECLARATIONS:
        // na

        // Types

        struct thermoPhysicialPropsStruct {
            // Destructor
            virtual ~thermoPhysicialPropsStruct() {
            }

            Nandle k;           // Thermal conductivity [W/m-K]
            Nandle rho;         // Density [kg/m3]
            Nandle cp;          // Specific heat [J/kg-K]
            Nandle rhoCp;       // Specific heat capacity [J/kg-K]
            Nandle diffusivity; // Thermal diffusivity [m2/s]

            thermoPhysicialPropsStruct() : k(0.0), rho(0.0), cp(0.0), rhoCp(0.0), diffusivity(0.0) {
            }
        };

        struct pipePropsStruct : thermoPhysicialPropsStruct {
            // Destructor
            ~pipePropsStruct() {
            }

            // Members
            Nandle outDia;      // Outer diameter of the pipe [m]
            Nandle innerDia;    // Inner diameter of the pipe [m]
            Nandle outRadius;   // Outer radius of the pipe [m]
            Nandle innerRadius; // Inner radius of the pipe [m]
            Nandle thickness;   // Thickness of the pipe wall [m]

            pipePropsStruct() : outDia(0.0), innerDia(0.0), outRadius(0.0), innerRadius(0.0), thickness(0.0) {
            }
        };

        struct GLHEVertPropsStruct {
            // Destructor
            ~GLHEVertPropsStruct() {
            }

            // Members
            std::string name;                 // Name
            Nandle bhTopDepth;                // Depth of top of borehole {m}
            Nandle bhLength;                  // Length of borehole from top of borehole {m}
            Nandle bhDiameter;                // Diameter of borehole {m}
            thermoPhysicialPropsStruct grout; // Grout properties
            pipePropsStruct pipe;             // Pipe properties
            Nandle bhUTubeDist;               // U-tube, shank-to-shank spacking {m}

            GLHEVertPropsStruct() : bhTopDepth(0.0), bhLength(0.0), bhDiameter(0.0), bhUTubeDist(0.0) {
            }
        };

        struct MyCartesian {
            // Destructor
            ~MyCartesian() {
            }

            Nandle x;
            Nandle y;
            Nandle z;

            MyCartesian() : x(0.0), y(0.0), z(0.0) {
            }
        };

        struct GLHEVertSingleStruct {
            // Destructor
            ~GLHEVertSingleStruct() {
            }

            // Members
            std::string name;                           // Name
            Nandle xLoc;                                // X-direction location {m}
            Nandle yLoc;                                // Y-direction location {m}
            Nandle dl_i;                                // Discretized bh length between points
            Nandle dl_ii;                               // Discretized bh length between points
            Nandle dl_j;                                // Discretized bh length between points
            std::shared_ptr<GLHEVertPropsStruct> props; // Properties
            std::vector<MyCartesian>
                    pointLocations_i; // Discretized point locations for when computing temperature response of other boreholes on this bh
            std::vector<MyCartesian> pointLocations_ii; // Discretized point locations for when computing temperature response of this bh on itself
            std::vector<MyCartesian>
                    pointLocations_j; // Discretized point locations for when other bh are computing the temperature response of this bh on themselves

            GLHEVertSingleStruct() : xLoc(0.0), yLoc(0.0), dl_i(0.0), dl_ii(0.0), dl_j(0.0) {
            }
        };

        struct GLHEVertArrayStruct {
            // Destructor
            ~GLHEVertArrayStruct() {
            }

            // Members
            std::string name;                           // Name
            int numBHinXDirection;                      // Number of boreholes in X direction
            int numBHinYDirection;                      // Number of boreholes in Y direction
            Nandle bhSpacing;                           // Borehole center-to-center spacing {m}
            std::shared_ptr<GLHEVertPropsStruct> props; // Properties

            GLHEVertArrayStruct() : numBHinXDirection(0), numBHinYDirection(0), bhSpacing(0.0) {
            }
        };

        struct GLHEResponseFactorsStruct {
            // Destructor
            ~GLHEResponseFactorsStruct() {
            }

            // Members
            std::string name;                                              // Name
            int numBoreholes;                                              // Number of boreholes
            int numGFuncPairs;                                             // Number of g-function pairs
            Nandle gRefRatio;                                              // Reference ratio of g-function set
            Nandle maxSimYears;                                            // Maximum length of simulation in years
            Array1D<Nandle> time;                                          // response time in seconds
            Array1D<Nandle> LNTTS;                                         // natural log of Non Dimensional Time Ln(t/ts)
            Array1D<Nandle> GFNC;                                          // G-function ( Non Dimensional temperature response factors)
            std::shared_ptr<GLHEVertPropsStruct> props;                    // Properties
            std::vector<std::shared_ptr<GLHEVertSingleStruct>> myBorholes; // Boreholes used by this response factors object

            GLHEResponseFactorsStruct() : numBoreholes(0), numGFuncPairs(0), gRefRatio(0.0), maxSimYears(0.0) {
            }
        };

        struct GLHEBase : PlantComponent {
            // Destructor
            virtual ~GLHEBase() {
            }

            // Members
            bool available;   // need an array of logicals--load identifiers of available equipment
            bool on;          // simulate the machine at it's operating part load ratio
            std::string name; // user identifier
            int loopNum;
            int loopSideNum;
            int branchNum;
            int compNum;
            int inletNodeNum;  // Node number on the inlet side of the plant
            int outletNodeNum; // Node number on the outlet side of the plant
            thermoPhysicialPropsStruct soil;
            pipePropsStruct pipe;
            thermoPhysicialPropsStruct grout;
            std::shared_ptr<GLHEResponseFactorsStruct> myRespFactors;
            Nandle designFlow;            // Design volumetric flow rate			[m3/s]
            Nandle designMassFlow;        // Design mass flow rate				[kg/s]
            Nandle tempGround;            // The far field temperature of the ground   [degC]
            Array1D<Nandle> QnMonthlyAgg; // Monthly aggregated normalized heat extraction/rejection rate [W/m]
            Array1D<Nandle> QnHr;         // Hourly aggregated normalized heat extraction/rejection rate [W/m]
            Array1D<Nandle> QnSubHr; // Contains the sub-hourly heat extraction/rejection rate normalized by the total active length of bore holes  [W/m]
            int prevHour;
            int AGG;               // Minimum Hourly History required
            int SubAGG;            // Minimum sub-hourly History
            Array1D_int LastHourN; // Stores the Previous hour's N for past hours until the minimum sub-hourly history
            Nandle bhTemp;         // [degC]
            Nandle massFlowRate;   // [kg/s]
            Nandle outletTemp;     // [degC]
            Nandle inletTemp;      // [degC]
            Nandle aveFluidTemp;   // [degC]
            Nandle QGLHE;          // [W] heat transfer rate
            bool myFlag;
            bool myEnvrnFlag;
            bool gFunctionsExist;
            Nandle lastQnSubHr;
            Nandle HXResistance;    // The thermal resistance of the GHX, (K per W/m)
            Nandle totalTubeLength; // The total length of pipe. NumBoreholes * BoreholeDepth OR Pi * Dcoil * NumCoils
            Nandle timeSS;          // Steady state time
            Nandle timeSSFactor;    // Steady state time factor for calculation
            std::shared_ptr<BaseGroundTempsModel> groundTempModel;

            // some statics pulled out into member variables
            bool firstTime;
            int numErrorCalls;
            Nandle ToutNew;
            int PrevN;                // The saved value of N at previous time step
            bool updateCurSimTime; // Used to reset the CurSimTime to reset after WarmupFlag
            bool triggerDesignDayReset;

            GLHEBase()
                    : available(false), on(false), loopNum(0), loopSideNum(0), branchNum(0), compNum(0),
                      inletNodeNum(0), outletNodeNum(0), designFlow(0.0),
                      designMassFlow(0.0), tempGround(0.0), prevHour(1), AGG(0), SubAGG(0), bhTemp(0.0),
                      massFlowRate(0.0), outletTemp(0.0), inletTemp(0.0),
                      aveFluidTemp(0.0), QGLHE(0.0), myFlag(true), myEnvrnFlag(true), gFunctionsExist(false),
                      lastQnSubHr(0.0), HXResistance(0.0),
                      timeSS(0.0), timeSSFactor(0.0), firstTime(true), numErrorCalls(0), ToutNew(19.375), PrevN(1),
                      updateCurSimTime(true), triggerDesignDayReset(false) {
            }

            virtual void calcGFunctions() = 0;

            void calcAggregateLoad();

            void updateGHX();

            void calcGroundHeatExchanger();

            inline bool isEven(int const val);

            Nandle interpGFunc(Nandle);

            void makeThisGLHECacheAndCompareWithFileCache();

            virtual void makeThisGLHECacheStruct() = 0;

            virtual void readCacheFileAndCompareWithThisGLHECache() = 0;

            void onInitLoopEquip(const PlantLocation &calledFromLocation) override;

            void simulate(const PlantLocation &calledFromLocation, bool const FirstHVACIteration, Nandle &CurLoad,
                          bool const RunFlag) override;

            static PlantComponent *factory(int const objectType, std::string objectName);

            virtual Nandle getGFunc(Nandle) = 0;

            virtual void initGLHESimVars() = 0;

            virtual Nandle calcHXResistance() = 0;

            virtual void getAnnualTimeConstant() = 0;
        };

        struct GLHEVert : GLHEBase {
            // Destructor
            ~GLHEVert() {
            }

            // Members
            Nandle bhDiameter;  // Diameter of borehole {m}
            Nandle bhRadius;    // Radius of borehole {m}
            Nandle bhLength;    // Length of borehole {m}
            Nandle bhUTubeDist; // Distance between u-tube legs {m}

            // Parameters for the multipole method
            Nandle theta_1;
            Nandle theta_2;
            Nandle theta_3;
            Nandle sigma;

            nlohmann::json myCacheData;

            std::vector<Nandle> GFNC_shortTimestep;
            std::vector<Nandle> LNTTS_shortTimestep;

            GLHEVert() : bhDiameter(0.0), bhRadius(0.0), bhLength(0.0), bhUTubeDist(0.0), theta_1(0.0), theta_2(0.0),
                         theta_3(0.0), sigma(0.0) {
            }

            std::vector<Nandle> distances(MyCartesian const &point_i, MyCartesian const &point_j);

            Nandle calcResponse(std::vector<Nandle> const &dists, Nandle const &currTime);

            Nandle integral(MyCartesian const &point_i, std::shared_ptr<GLHEVertSingleStruct> const &bh_j,
                            Nandle const &currTime);

            Nandle
            doubleIntegral(std::shared_ptr<GLHEVertSingleStruct> const &bh_i,
                           std::shared_ptr<GLHEVertSingleStruct> const &bh_j, Nandle const &currTime);

            void calcShortTimestepGFunctions();

            void calcLongTimestepGFunctions();

            void calcGFunctions();

            Nandle calcHXResistance();

            void initGLHESimVars();

            void getAnnualTimeConstant();

            Nandle getGFunc(Nandle const time);

            void makeThisGLHECacheStruct();

            void readCacheFileAndCompareWithThisGLHECache();

            void writeGLHECacheToFile();

            Nandle calcBHAverageResistance();

            Nandle calcBHTotalInternalResistance();

            Nandle calcBHGroutResistance();

            Nandle calcPipeConductionResistance();

            Nandle calcPipeConvectionResistance();

            Nandle frictionFactor(Nandle const reynoldsNum);

            Nandle calcPipeResistance();

            void combineShortAndLongTimestepGFunctions();
        };

        struct GLHESlinky : GLHEBase {

            // Destructor
            ~GLHESlinky() {
            }

            // Members
            bool verticalConfig;  // HX Configuration Flag
            Nandle coilDiameter;  // Diameter of the slinky coils [m]
            Nandle coilPitch;     // Center-to-center slinky coil spacing [m]
            Nandle coilDepth;     // Average depth of the coil [m]
            Nandle trenchDepth;   // Trench depth from ground surface to trench bottom [m]
            Nandle trenchLength;  // Length of single trench [m]
            int numTrenches;      // Number of parallel trenches [m]
            Nandle trenchSpacing; // Spacing between parallel trenches [m]
            int numCoils;         // Number of coils
            int monthOfMinSurfTemp;
            Nandle maxSimYears;
            Nandle minSurfTemp;
            Array1D<Nandle> X0;
            Array1D<Nandle> Y0;
            Nandle Z0;

            GLHESlinky()
                    : verticalConfig(false), coilDiameter(0.0), coilPitch(0.0), coilDepth(0.0), trenchDepth(0.0),
                      trenchLength(0.0), numTrenches(0),
                      trenchSpacing(0.0), numCoils(0), monthOfMinSurfTemp(0), maxSimYears(0.0), minSurfTemp(0.0) {
            }

            Nandle calcHXResistance();

            void calcGFunctions();

            void initGLHESimVars();

            void getAnnualTimeConstant();

            Nandle doubleIntegral(int const m, int const n, int const m1, int const n1, Nandle const t, int const I0,
                                  int const J0);

            Nandle integral(int const m, int const n, int const m1, int const n1, Nandle const t, Nandle const eta,
                            Nandle const J0);

            Nandle distance(int const m, int const n, int const m1, int const n1, Nandle const eta, Nandle const theta);

            Nandle distanceToFictRing(int const m, int const n, int const m1, int const n1, Nandle const eta,
                                      Nandle const theta);

            Nandle distToCenter(int const m, int const n, int const m1, int const n1);

            Nandle nearFieldResponseFunction(int const m, int const n, int const m1, int const n1, Nandle const eta,
                                             Nandle const theta, Nandle const t);

            Nandle midFieldResponseFunction(int const m, int const n, int const m1, int const n1, Nandle const t);

            Nandle getGFunc(Nandle const time);

            void makeThisGLHECacheStruct();

            void readCacheFileAndCompareWithThisGLHECache();
        };

        void clear_state();

        void GetGroundHeatExchangerInput();

        std::shared_ptr<GLHEResponseFactorsStruct>
        BuildAndGetResponseFactorObjectFromArray(std::shared_ptr<GLHEVertArrayStruct> const &arrayObjectPtr);

        std::shared_ptr<GLHEResponseFactorsStruct>
        BuildAndGetResponseFactorsObjectFromSingleBHs(
                std::vector<std::shared_ptr<GLHEVertSingleStruct>> const &singleBHsForRFVect);

        void SetupBHPointsForResponseFactorsObject(std::shared_ptr<GLHEResponseFactorsStruct> &thisRF);

        std::shared_ptr<GLHEResponseFactorsStruct> GetResponseFactor(std::string const &objectName);

        std::shared_ptr<GLHEVertSingleStruct> GetSingleBH(std::string const &objectName);

        std::shared_ptr<GLHEVertPropsStruct> GetVertProps(std::string const &objectName);

        std::shared_ptr<GLHEVertArrayStruct> GetVertArray(std::string const &objectName);

        std::vector<Nandle>
        TDMA(std::vector<Nandle> a, std::vector<Nandle> b, std::vector<Nandle> c, std::vector<Nandle> d);

        // Object Data
        extern std::vector<GLHEVert> verticalGLHE;                                            // Vertical GLHEs
        extern std::vector<GLHESlinky> slinkyGLHE;                                            // Slinky GLHEs
        extern std::vector<std::shared_ptr<GLHEVertArrayStruct>> vertArraysVector;            // Vertical Arrays
        extern std::vector<std::shared_ptr<GLHEVertPropsStruct>> vertPropsVector;             // Vertical Properties
        extern std::vector<std::shared_ptr<GLHEResponseFactorsStruct>> responseFactorsVector; // Vertical Response Factors
        extern std::vector<std::shared_ptr<GLHEVertSingleStruct>> singleBoreholesVector;      // Vertical Single Boreholes

    } // namespace GroundHeatExchangers

} // namespace EnergyPlus

#endif
