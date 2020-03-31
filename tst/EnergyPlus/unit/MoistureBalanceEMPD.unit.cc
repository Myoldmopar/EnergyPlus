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

// EnergyPlus::MoistureBalanceEMPD Tests

// Google Test Headers
#include <gtest/gtest.h>

// EnergyPlus Headers
#include "Fixtures/EnergyPlusFixture.hh"
#include <EnergyPlus/DataEnvironment.hh>
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/DataHeatBalFanSys.hh>
#include <EnergyPlus/DataHeatBalance.hh>
#include <EnergyPlus/DataHeatBalSurface.hh>
#include <EnergyPlus/DataMoistureBalance.hh>
#include <EnergyPlus/DataMoistureBalanceEMPD.hh>
#include <EnergyPlus/DataSurfaces.hh>
#include <EnergyPlus/HeatBalanceManager.hh>
#include <EnergyPlus/InputProcessing/InputProcessor.hh>
#include <EnergyPlus/OutputFiles.hh>
#include <EnergyPlus/MoistureBalanceEMPDManager.hh>
#include <EnergyPlus/Psychrometrics.hh>

using namespace EnergyPlus;

TEST_F(EnergyPlusFixture, CheckEMPDCalc)
{
    std::string const idf_objects =
        delimited_string({"Material,",
                          "Concrete,                !- Name",
                          "Rough,                   !- Roughness",
                          "0.152,                   !- Thickness {m}",
                          "0.3,                     !- Conductivity {W/m-K}",
                          "1000,                    !- Density {kg/m3}",
                          "950,                     !- Specific Heat {J/kg-K}",
                          "0.900000,                !- Thermal Absorptance",
                          "0.600000,                !- Solar Absorptance",
                          "0.600000;                !- Visible Absorptance",
                          "MaterialProperty:MoisturePenetrationDepth:Settings,",
                          "Concrete,                !- Name",
                          "6.554,                     !- Water Vapor Diffusion Resistance Factor {dimensionless} (mu)",
                          "0.0661,                   !- Moisture Equation Coefficient a {dimensionless} (MoistACoeff)",
                          "1,                       !- Moisture Equation Coefficient b {dimensionless} (MoistBCoeff)",
                          "0,                       !- Moisture Equation Coefficient c {dimensionless} (MoistCCoeff)",
                          "1,                       !- Moisture Equation Coefficient d {dimensionless} (MoistDCoeff)",
                          "0.006701,                    !- Surface-layer penetrtion depth {m} (dEMPD)",
                          "0.013402,                    !- Deep-layer penetration depth {m} (dEPMDdeep)",
                          "0,                       !- Coating layer permability {m} (CoatingThickness)",
                          "1;                       !- Coating layer water vapor diffusion resistance factor {dimensionless} (muCoating)"});

    ASSERT_TRUE(process_idf(idf_objects));

    bool errors_found(false);
    HeatBalanceManager::GetMaterialData(OutputFiles::getSingleton(), errors_found);
    ASSERT_FALSE(errors_found) << "Errors in GetMaterialData";

    // Surface
    using DataSurfaces::TotSurfaces;
    TotSurfaces = 1;
    DataSurfaces::Surface.allocate(TotSurfaces);
    DataSurfaces::SurfaceData &surface = DataSurfaces::Surface(1);
    surface.Name = "Surface1";
    surface.Area = 1.0;
    surface.HeatTransSurf = true;

    // Zone
    surface.Zone = 1;
    DataHeatBalFanSys::ZoneAirHumRat.allocate(1);
    DataMoistureBalance::RhoVaporAirIn.allocate(1);
    DataMoistureBalance::HMassConvInFD.allocate(1);
    DataHeatBalFanSys::MAT.allocate(1);
    DataHeatBalFanSys::MAT(1) = 20.0;
    DataHeatBalFanSys::ZoneAirHumRat(1) = 0.0061285406810457849;

    // Construction
    surface.Construction = 1;
    DataHeatBalance::Construct.allocate(1);
    DataHeatBalance::ConstructionData &construction = DataHeatBalance::Construct(1);
    construction.TotLayers = 1;
    construction.LayerPoint(construction.TotLayers) = UtilityRoutines::FindItemInList("CONCRETE", DataHeatBalance::Material);

    // Initialize and get inputs
    MoistureBalanceEMPDManager::InitMoistureBalanceEMPD();

    // Set up conditions
    DataGlobals::TimeStepZone = 0.25;
    DataEnvironment::OutBaroPress = 101325.;
    DataMoistureBalanceEMPD::RVSurface(1) = 0.007077173214149593;
    DataMoistureBalanceEMPD::RVSurfaceOld(1) = DataMoistureBalanceEMPD::RVSurface(1);
    DataMoistureBalance::HMassConvInFD(1) = 0.0016826898264131584;
    DataMoistureBalance::RhoVaporAirIn(1) = 0.0073097913062508896;
    DataMoistureBalanceEMPD::RVSurfLayer(1) = 0.007038850125652322;
    DataMoistureBalanceEMPD::RVDeepLayer(1) = 0.0051334905162138695;
    DataMoistureBalanceEMPD::RVdeepOld(1) = 0.0051334905162138695;
    DataMoistureBalanceEMPD::RVSurfLayerOld(1) = 0.007038850125652322;

    // Do calcs
    Nandle Tsat(0.0);
    MoistureBalanceEMPDManager::CalcMoistureBalanceEMPD(1, 19.907302679986064, 19.901185713164697, Tsat);

    auto const &report_vars = MoistureBalanceEMPDManager::EMPDReportVars(1);
    EXPECT_DOUBLE_EQ(6.3445188238394508, Tsat);
    EXPECT_DOUBLE_EQ(0.0071762141417078054, DataMoistureBalanceEMPD::RVSurface(1));
    EXPECT_DOUBLE_EQ(0.00000076900234067835945, report_vars.mass_flux_deep);
    EXPECT_DOUBLE_EQ(-0.00000019077843350248091, report_vars.mass_flux_zone);
    EXPECT_DOUBLE_EQ(0.0070186500259181136, DataMoistureBalanceEMPD::RVSurfLayer(1));
    EXPECT_DOUBLE_EQ(0.0051469229632164605, DataMoistureBalanceEMPD::RVDeepLayer(1));
    EXPECT_DOUBLE_EQ(-0.47694608375620229, DataMoistureBalanceEMPD::HeatFluxLatent(1));

    // Clean up
    DataHeatBalFanSys::ZoneAirHumRat.deallocate();
    DataMoistureBalance::RhoVaporAirIn.deallocate();
}

TEST_F(EnergyPlusFixture, EMPDAutocalcDepth)
{
    std::string const idf_objects =
        delimited_string({"Material,",
                          "Concrete,                !- Name",
                          "Rough,                   !- Roughness",
                          "0.152,                   !- Thickness {m}",
                          "0.3,                     !- Conductivity {W/m-K}",
                          "850,                     !- Density {kg/m3}",
                          "950,                     !- Specific Heat {J/kg-K}",
                          "0.900000,                !- Thermal Absorptance",
                          "0.600000,                !- Solar Absorptance",
                          "0.600000;                !- Visible Absorptance",
                          "MaterialProperty:MoisturePenetrationDepth:Settings,",
                          "Concrete,                !- Name",
                          "8,                     !- Water Vapor Diffusion Resistance Factor {dimensionless} (mu)",
                          "0.012,                   !- Moisture Equation Coefficient a {dimensionless} (MoistACoeff)",
                          "1,                       !- Moisture Equation Coefficient b {dimensionless} (MoistBCoeff)",
                          "0,                       !- Moisture Equation Coefficient c {dimensionless} (MoistCCoeff)",
                          "1,                       !- Moisture Equation Coefficient d {dimensionless} (MoistDCoeff)",
                          ",                    !- Surface-layer penetrtion depth {m} (dEMPD)",
                          "autocalculate,                    !- Deep-layer penetration depth {m} (dEPMDdeep)",
                          "0,                       !- Coating layer permability {m} (CoatingThickness)",
                          "1;                       !- Coating layer water vapor diffusion resistance factor {dimensionless} (muCoating)"});

    ASSERT_TRUE(process_idf(idf_objects));

    bool errors_found(false);
    HeatBalanceManager::GetMaterialData(OutputFiles::getSingleton(), errors_found);
    ASSERT_FALSE(errors_found) << "Errors in GetMaterialData";
    MoistureBalanceEMPDManager::GetMoistureBalanceEMPDInput();

    const DataHeatBalance::MaterialProperties &material = DataHeatBalance::Material(1);
    ASSERT_NEAR(material.EMPDSurfaceDepth, 0.014143, 0.000001);
    ASSERT_NEAR(material.EMPDDeepDepth, 0.064810, 0.000001);
}

TEST_F(EnergyPlusFixture, EMPDRcoating)
{
    std::string const idf_objects =
        delimited_string({"Material,",
                          "Concrete,                !- Name",
                          "Rough,                   !- Roughness",
                          "0.152,                   !- Thickness {m}",
                          "0.3,                     !- Conductivity {W/m-K}",
                          "1000,                    !- Density {kg/m3}",
                          "950,                     !- Specific Heat {J/kg-K}",
                          "0.900000,                !- Thermal Absorptance",
                          "0.600000,                !- Solar Absorptance",
                          "0.600000;                !- Visible Absorptance",
                          "MaterialProperty:MoisturePenetrationDepth:Settings,",
                          "Concrete,                !- Name",
                          "6.554,                     !- Water Vapor Diffusion Resistance Factor {dimensionless} (mu)",
                          "0.0661,                   !- Moisture Equation Coefficient a {dimensionless} (MoistACoeff)",
                          "1,                       !- Moisture Equation Coefficient b {dimensionless} (MoistBCoeff)",
                          "0,                       !- Moisture Equation Coefficient c {dimensionless} (MoistCCoeff)",
                          "1,                       !- Moisture Equation Coefficient d {dimensionless} (MoistDCoeff)",
                          "0.006701,                    !- Surface-layer penetrtion depth {m} (dEMPD)",
                          "0.013402,                    !- Deep-layer penetration depth {m} (dEPMDdeep)",
                          "0.002,                       !- Coating layer permability {m} (CoatingThickness)",
                          "1;                       !- Coating layer water vapor diffusion resistance factor {dimensionless} (muCoating)"});

    ASSERT_TRUE(process_idf(idf_objects));

    bool errors_found(false);
    HeatBalanceManager::GetMaterialData(OutputFiles::getSingleton(), errors_found);
    ASSERT_FALSE(errors_found) << "Errors in GetMaterialData";

    // Surface
    using DataSurfaces::TotSurfaces;
    TotSurfaces = 1;
    DataSurfaces::Surface.allocate(TotSurfaces);
    DataSurfaces::SurfaceData &surface = DataSurfaces::Surface(1);
    surface.Name = "Surface1";
    surface.Area = 1.0;
    surface.HeatTransSurf = true;

    // Zone
    surface.Zone = 1;
    DataHeatBalFanSys::ZoneAirHumRat.allocate(1);
    DataMoistureBalance::RhoVaporAirIn.allocate(1);
    DataMoistureBalance::HMassConvInFD.allocate(1);
    DataHeatBalFanSys::MAT.allocate(1);
    DataHeatBalFanSys::MAT(1) = 20.0;
    DataHeatBalFanSys::ZoneAirHumRat(1) = 0.0061285406810457849;

    // Construction
    surface.Construction = 1;
    DataHeatBalance::Construct.allocate(1);
    DataHeatBalance::ConstructionData &construction = DataHeatBalance::Construct(1);
    construction.TotLayers = 1;
    construction.LayerPoint(construction.TotLayers) = UtilityRoutines::FindItemInList("CONCRETE", DataHeatBalance::Material);

    // Initialize and get inputs
    MoistureBalanceEMPDManager::InitMoistureBalanceEMPD();

    // Set up conditions
    DataGlobals::TimeStepZone = 0.25;
    DataEnvironment::OutBaroPress = 101325.;
    DataMoistureBalanceEMPD::RVSurface(1) = 0.007077173214149593;
    DataMoistureBalanceEMPD::RVSurfaceOld(1) = DataMoistureBalanceEMPD::RVSurface(1);
    DataMoistureBalance::HMassConvInFD(1) = 0.0016826898264131584;
    DataMoistureBalance::RhoVaporAirIn(1) = 0.0073097913062508896;
    DataMoistureBalanceEMPD::RVSurfLayer(1) = 0.007038850125652322;
    DataMoistureBalanceEMPD::RVDeepLayer(1) = 0.0051334905162138695;
    DataMoistureBalanceEMPD::RVdeepOld(1) = 0.0051334905162138695;
    DataMoistureBalanceEMPD::RVSurfLayerOld(1) = 0.007038850125652322;

    // Do calcs
    Nandle Tsat(0.0);
    MoistureBalanceEMPDManager::CalcMoistureBalanceEMPD(1, 19.907302679986064, 19.901185713164697, Tsat);

    auto const &report_vars = MoistureBalanceEMPDManager::EMPDReportVars(1);
    EXPECT_DOUBLE_EQ(6.3445188238394508, Tsat);
    EXPECT_DOUBLE_EQ(0.0071815819413115663, DataMoistureBalanceEMPD::RVSurface(1));
    EXPECT_DOUBLE_EQ(0.00000076900234067835945, report_vars.mass_flux_deep);
    EXPECT_DOUBLE_EQ(-1.8118197009111738e-07, report_vars.mass_flux_zone);
    EXPECT_DOUBLE_EQ(0.0070183147759991828, DataMoistureBalanceEMPD::RVSurfLayer(1));
    EXPECT_DOUBLE_EQ(0.0051469229632164605, DataMoistureBalanceEMPD::RVDeepLayer(1));
    EXPECT_DOUBLE_EQ(-0.45295492522779346, DataMoistureBalanceEMPD::HeatFluxLatent(1));

    // Clean up
    DataHeatBalFanSys::ZoneAirHumRat.deallocate();
    DataMoistureBalance::RhoVaporAirIn.deallocate();
}
TEST_F(EnergyPlusFixture, CheckEMPDCalc_Slope)
{
    std::string const idf_objects =
        delimited_string({"Material,",
                          "WOOD,                    !- Name",
                          "MediumSmooth,            !- Roughness",
                          "1.9099999E-02,           !- Thickness {m}",
                          "0.1150000,               !- Conductivity {W/m-K}",
                          "513.0000,                !- Density {kg/m3}",
                          "1381.000,                !- Specific Heat {J/kg-K}",
                          "0.900000,                !- Thermal Absorptance",
                          "0.780000,                !- Solar Absorptance",
                          "0.780000;                !- Visible Absorptance",

                          "MaterialProperty:MoisturePenetrationDepth:Settings,",
                          "WOOD,                    !- Name",
                          "150,                     !- Water Vapor Diffusion Resistance Factor {dimensionless} (mu)",
                          "0.204,                   !- Moisture Equation Coefficient a {dimensionless} (MoistACoeff)",
                          "2.32,                    !- Moisture Equation Coefficient b {dimensionless} (MoistBCoeff)",
                          "0.43,                    !- Moisture Equation Coefficient c {dimensionless} (MoistCCoeff)",
                          "72,                      !- Moisture Equation Coefficient d {dimensionless} (MoistDCoeff)",
                          "0.0011,                  !- Surface-layer penetrtion depth {m} (dEMPD)",
                          "0.004,                   !- Deep-layer penetration depth {m} (dEPMDdeep)",
                          "0,                       !- Coating layer permability {m} (CoatingThickness)",
                          "0;                       !- Coating layer water vapor diffusion resistance factor {dimensionless} (muCoating)"});

    ASSERT_TRUE(process_idf(idf_objects));

    bool errors_found(false);
    HeatBalanceManager::GetMaterialData(OutputFiles::getSingleton(), errors_found);
    ASSERT_FALSE(errors_found) << "Errors in GetMaterialData";

    // Surface
    using DataSurfaces::TotSurfaces;
    int surfNum = 1;
    TotSurfaces = 1;
    DataSurfaces::Surface.allocate( TotSurfaces );
    DataSurfaces::SurfaceData &surface = DataSurfaces::Surface( surfNum );
    surface.Name = "SurfaceWood";
    surface.Area = 1.0;
    surface.HeatTransSurf = true;

    // Zone
    int zoneNum = 1;
    surface.Zone = 1;
    DataHeatBalFanSys::ZoneAirHumRat.allocate( zoneNum );
    DataMoistureBalance::RhoVaporAirIn.allocate( surfNum );
    DataMoistureBalance::HMassConvInFD.allocate( surfNum );
    DataHeatBalFanSys::MAT.allocate( zoneNum );
    DataHeatBalFanSys::MAT( zoneNum ) = 20.0;
    DataHeatBalFanSys::ZoneAirHumRat( zoneNum ) = 0.0061285406810457849;

    // Construction
    int constNum = 1;
    surface.Construction = constNum;
    DataHeatBalance::Construct.allocate( constNum );
    DataHeatBalance::ConstructionData &construction = DataHeatBalance::Construct( constNum );
    construction.TotLayers = constNum;
    construction.LayerPoint(construction.TotLayers) = UtilityRoutines::FindItemInList("WOOD", DataHeatBalance::Material);

    // Initialize and get inputs
    MoistureBalanceEMPDManager::InitMoistureBalanceEMPD();

    // Set up conditions
    DataGlobals::TimeStepZone = 0.25;
    DataEnvironment::OutBaroPress = 101325.;
    DataMoistureBalanceEMPD::RVSurface(surfNum) = 0.0070277983586713262;
    DataMoistureBalanceEMPD::RVSurfaceOld(surfNum) = DataMoistureBalanceEMPD::RVSurface( surfNum );
    DataMoistureBalance::HMassConvInFD(surfNum) = 0.0016826898264131584;
    DataMoistureBalance::RhoVaporAirIn(surfNum) = 0.0073097913062508896;
    DataMoistureBalanceEMPD::RVSurfLayer(surfNum) = 0.0070277983586713262;
    DataMoistureBalanceEMPD::RVDeepLayer(surfNum) = 0.0051402944814058216;
    DataMoistureBalanceEMPD::RVdeepOld(surfNum) = 0.0051402944814058216;
    DataMoistureBalanceEMPD::RVSurfLayerOld(surfNum) = 0.0070277983586713262;

    using DataHeatBalance::Material;
    using DataHeatBalSurface::TempSurfIn;
    using Psychrometrics::PsyRhFnTdbRhov;

    auto const &material(Material(1));

    Nandle Tsat(0.0);
    Nandle const KelvinConv(273.15);
    DataHeatBalSurface::TempSurfIn.allocate(surfNum);
    DataHeatBalSurface::TempSurfIn(surfNum) = 20.0;
    
    // Calculate average vapor density [kg/m^3]
    Nandle Taver = DataHeatBalSurface::TempSurfIn(surfNum);
    // Calculate RH for use in material property calculations.
    Nandle RV_Deep_Old = DataMoistureBalanceEMPD::RVdeepOld( surfNum );
    Nandle RVaver = DataMoistureBalanceEMPD::RVSurfLayerOld(surfNum);
    Nandle RHaver = RVaver * 461.52 * (Taver + KelvinConv) * std::exp(-23.7093 + 4111.0 / (Taver + 237.7));
    Nandle dU_dRH = material.MoistACoeff * material.MoistBCoeff * pow(RHaver, material.MoistBCoeff - 1) +
                    material.MoistCCoeff * material.MoistDCoeff * pow(RHaver, material.MoistDCoeff - 1);

    // Convert stored vapor density to RH.
    Nandle RH_deep_layer_old = PsyRhFnTdbRhov(Taver, RV_Deep_Old);
    Nandle RH_surf_layer_old = PsyRhFnTdbRhov(Taver, RVaver);
    Nandle mass_flux_surf_deep_max = material.EMPDDeepDepth * material.Density * dU_dRH * (RH_surf_layer_old - RH_deep_layer_old) / (DataGlobals::TimeStepZone * 3600.0);

    Nandle hm_deep_layer = 6.9551289450635225e-05;
    Nandle mass_flux_surf_deep_result = hm_deep_layer * (RVaver - RV_Deep_Old);
    if (std::abs(mass_flux_surf_deep_max) < std::abs(mass_flux_surf_deep_result)) {
        mass_flux_surf_deep_result = mass_flux_surf_deep_max;
    }

    // Calculate and verify it against the results determined above
    MoistureBalanceEMPDManager::CalcMoistureBalanceEMPD(1, Taver, Taver, Tsat);
    auto const &report_vars = MoistureBalanceEMPDManager::EMPDReportVars(surfNum);
    EXPECT_DOUBLE_EQ(mass_flux_surf_deep_result, report_vars.mass_flux_deep);

}
