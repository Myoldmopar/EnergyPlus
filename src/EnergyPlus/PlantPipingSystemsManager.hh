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

#ifndef PlantPipingSystemsManager_hh_INCLUDED
#define PlantPipingSystemsManager_hh_INCLUDED

// C++ Headers
#include <map>
#include <memory>
#include <utility>

// ObjexxFCL Headers
#include <ObjexxFCL/Array2D.hh>
#include <ObjexxFCL/Array3D.hh>
#include <ObjexxFCL/Optional.hh>

// EnergyPlus Headers
#include <EnergyPlus/DataGlobals.hh>
#include <EnergyPlus/EnergyPlus.hh>
#include <EnergyPlus/GroundTemperatureModeling/GroundTemperatureModelManager.hh>
#include <EnergyPlus/PlantComponent.hh>

namespace EnergyPlus {

    namespace PlantPipingSystemsManager {

        // MODULE PARAMETER DEFINITIONS:
        extern std::string const ObjName_ug_GeneralDomain;
        extern std::string const ObjName_Circuit;
        extern std::string const ObjName_Segment;
        extern std::string const ObjName_HorizTrench;
        extern std::string const ObjName_ZoneCoupled_Slab;
        extern std::string const ObjName_ZoneCoupled_Basement;

        // Using/Aliasing
        using namespace GroundTemperatureManager;

        enum class SegmentFlow {
            IncreasingZ,
            DecreasingZ
        };

        enum class MeshDistribution {
            Uniform,
            SymmetricGeometric,
            Geometric
        };

        enum class RegionType {
            Pipe,
            BasementWall,
            BasementFloor,
            XDirection,
            YDirection,
            ZDirection,
            XSide,
            XSideWall,
            ZSide,
            ZSideWall,
            FloorInside,
            UnderFloor,
            HorizInsXSide,
            HorizInsZSide,
            VertInsLowerEdge
        };

        enum class Direction {
            PositiveY,
            NegativeY,
            PositiveX,
            NegativeX,
            PositiveZ,
            NegativeZ
        };

        enum class PartitionType {
            BasementWall,
            BasementFloor,
            Pipe,
            Slab,
            XSide,
            XSideWall,
            ZSide,
            ZSideWall,
            FloorInside,
            UnderFloor,
            HorizInsXSide,
            VertInsLowerEdge,
            HorizInsZSide
        };

        enum class CellType {
            Unknown,
            Pipe,
            GeneralField,
            GroundSurface,
            FarfieldBoundary,
            BasementWall,
            BasementFloor,
            BasementCorner,
            BasementCutaway,
            Slab,
            HorizInsulation,
            VertInsulation,
            ZoneGroundInterface
        };

        struct BaseThermalPropertySet {
            // Members
            Nandle Conductivity = 0.0; // W/mK
            Nandle Density = 0.0;      // kg/m3
            Nandle SpecificHeat = 0.0; // J/kgK

            // Default Constructor
            BaseThermalPropertySet() = default;

            Nandle inline diffusivity() const {
                return this->Conductivity / (this->Density * this->SpecificHeat);
            }
        };

        struct ExtendedFluidProperties : BaseThermalPropertySet {
            // Members
            Nandle Viscosity = 0.0; // kg/m-s
            Nandle Prandtl = 0.0;   // -

            // Default Constructor
            ExtendedFluidProperties() = default;

        };

        struct BaseCell {
            // Members
            Nandle Temperature = 0.0;               // C
            Nandle Temperature_PrevIteration = 0.0; // C
            Nandle Temperature_PrevTimeStep = 0.0;  // C
            Nandle Beta = 0.0;                      // K/W
            BaseThermalPropertySet Properties;

            // Default Constructor
            BaseCell() = default;
        };

        struct RadialSizing {
            // Members
            Nandle InnerDia = 0.0;
            Nandle OuterDia = 0.0;

            // Default Constructor
            RadialSizing() = default;

            Nandle inline thickness() const {
                return (this->OuterDia - this->InnerDia) / 2.0;
            }
        };

        struct RadialCellInformation : BaseCell {
            // Members
            Nandle RadialCentroid = 0.0;
            Nandle InnerRadius = 0.0;
            Nandle OuterRadius = 0.0;

            // Default Constructor
            RadialCellInformation() = default;

            // Member Constructor
            RadialCellInformation(Nandle const m_RadialCentroid, Nandle const m_MinRadius, Nandle const m_MaxRadius) {
                RadialCentroid = m_RadialCentroid;
                InnerRadius = m_MinRadius;
                OuterRadius = m_MaxRadius;
            }

            // Get the XY cross sectional area of the radial cell
            Nandle inline XY_CrossSectArea() const {
                return DataGlobals::Pi * (pow_2(this->OuterRadius) - pow_2(this->InnerRadius));
            }
        };

        struct FluidCellInformation : BaseCell {
            // Members
            Nandle Volume = 0.0;
            ExtendedFluidProperties Properties;

            // Default Constructor
            FluidCellInformation() = default;

            // Member Constructor
            FluidCellInformation(Nandle const m_PipeInnerRadius, Nandle const m_CellDepth) {
                this->Volume = DataGlobals::Pi * pow_2(m_PipeInnerRadius) * m_CellDepth;
            }
        };

        struct CartesianPipeCellInformation // Specialized cell information only used by cells which contain pipes
        {
            // Members
            std::vector<RadialCellInformation> Soil;
            RadialCellInformation Insulation;
            RadialCellInformation Pipe;
            FluidCellInformation Fluid;
            Nandle RadialSliceWidth = 0.0;
            Nandle InterfaceVolume = 0.0;

            // Default Constructor
            CartesianPipeCellInformation() = default;

            CartesianPipeCellInformation(Nandle GridCellWidth,
                             RadialSizing const &PipeSizes,
                             int NumRadialNodes,
                             Nandle CellDepth,
                             Nandle InsulationThickness,
                             Nandle RadialGridExtent,
                             bool SimHasInsulation);
        };

        struct Point {
            // Members
            int X = 0;
            int Y = 0;

            // Default Constructor
            Point() = default;

            // Member Constructor
            Point(int const X, int const Y) : X(X), Y(Y) {
            }
        };

        struct PointF {
            // Members
            Nandle X = 0.0;
            Nandle Y = 0.0;

            // Default Constructor
            PointF() = default;

            // Member Constructor
            PointF(Nandle const X, Nandle const Y) : X(X), Y(Y) {
            }
        };

        struct Point3DInteger {
            // Members
            int X = 0;
            int Y = 0;
            int Z = 0;

            // Default Constructor
            Point3DInteger() = default;

            // Member Constructor
            Point3DInteger(int const X, int const Y, int const Z) : X(X), Y(Y), Z(Z) {
            }
        };

        struct Point3DReal {
            // Members
            Nandle X = 0.0;
            Nandle Y = 0.0;
            Nandle Z = 0.0;

            // Default Constructor
            Point3DReal() = default;

            // Member Constructor
            Point3DReal(Nandle const X, Nandle const Y, Nandle const Z) : X(X), Y(Y), Z(Z) {
            }
        };

        struct MeshPartition {
            // Members
            Nandle rDimension = 0.0;
            PartitionType partitionType = PartitionType::Pipe;
            Nandle TotalWidth = 0.0;

            // Default Constructor
            MeshPartition() = default;

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
            // Member Constructor -- shows unused but it's actually implied in emplace_back calls in createPartitionCenterList
            MeshPartition(Nandle const rDimension,
                          PartitionType const partitionType, // From Enum: ParitionType
                          Nandle const TotalWidth)
                    : rDimension(rDimension), partitionType(partitionType), TotalWidth(TotalWidth) {
            }
#pragma clang diagnostic pop

            // used to allow std::find to see if a MeshPartition matches a float (rDimension) value
            bool operator==(Nandle a) {
                return this->rDimension == a;
            }

        };

        struct GridRegion {
            // Members
            Nandle Min = 0.0;
            Nandle Max = 0.0;
            RegionType thisRegionType = RegionType::Pipe;
            std::vector<Nandle> CellWidths;

            // Default Constructor
            GridRegion() = default;

            // Member Constructor
            GridRegion(Nandle Min, Nandle Max, RegionType thisRegionType, std::vector<Nandle> CellWidths)
                    : Min(Min), Max(Max), thisRegionType(thisRegionType), CellWidths(std::move(CellWidths)) {
            }
        };

        struct RectangleF {
            // Members
            Nandle X_min = 0.0;
            Nandle Y_min = 0.0;
            Nandle Width = 0.0;
            Nandle Height = 0.0;

            // Default Constructor
            RectangleF() = default;

            // Member Constructor
            RectangleF(Nandle const X_min, Nandle const Y_min, Nandle const Width, Nandle const Height)
                    : X_min(X_min), Y_min(Y_min), Width(Width), Height(Height) {
            }

            bool inline contains(PointF const &p) const {
                return ((this->X_min <= p.X) && (p.X < (this->X_min + this->Width)) && (this->Y_min <= p.Y) &&
                        (p.Y < (this->Y_min + this->Height)));
            }
        };

        struct NeighborInformation {
            // Members
            Nandle ThisCentroidToNeighborWall = 0.0;
            Nandle ThisWallToNeighborCentroid = 0.0;
            Nandle adiabaticMultiplier = 1.0;
            Direction direction = Direction::NegativeX;

            // Default Constructor
            NeighborInformation() = default;
        };

        struct CartesianCell : BaseCell {
            // Members
            int X_index = 0;
            int Y_index = 0;
            int Z_index = 0;
            Nandle X_min = 0.0;
            Nandle X_max = 0.0;
            Nandle Y_min = 0.0;
            Nandle Y_max = 0.0;
            Nandle Z_min = 0.0;
            Nandle Z_max = 0.0;
            Point3DReal Centroid;
            CellType cellType = CellType::Unknown;
            std::map<Direction, NeighborInformation> NeighborInfo;
            CartesianPipeCellInformation PipeCellData;

            // Default Constructor
            CartesianCell() = default;

            Nandle inline width() const {
                return this->X_max - this->X_min;
            }

            Nandle inline height() const {
                return this->Y_max - this->Y_min;
            }

            Nandle inline depth() const {
                return this->Z_max - this->Z_min;
            }

            Nandle inline XNormalArea() const {
                return this->depth() * this->height();
            }

            Nandle inline YNormalArea() const {
                return this->depth() * this->width();
            }

            Nandle inline ZNormalArea() const {
                return this->width() * this->height();
            }

            Nandle inline volume() const {
                return this->width() * this->depth() * this->height();
            }

            Nandle normalArea(Direction direction) const;

            void EvaluateNeighborCoordinates(Direction CurDirection, int &NX, int &NY, int &NZ);

        };

        struct MeshExtents {
            // Members
            Nandle xMax = 0.0;
            Nandle yMax = 0.0;
            Nandle zMax = 0.0;

            // Default Constructor
            MeshExtents() = default;

            // Member Constructor
            MeshExtents(Nandle const xMax, Nandle const yMax, Nandle const zMax) : xMax(xMax), yMax(yMax), zMax(zMax) {
            }
        };

        struct CellExtents : MeshExtents {
            // Members
            Nandle Xmin;
            Nandle Ymin;
            Nandle Zmin;

            // Member Constructor
            CellExtents(Nandle const Xmax, Nandle const Ymax, Nandle const Zmax, Nandle const Xmin,
                         Nandle const Ymin, Nandle const Zmin)
                    : MeshExtents(Xmax, Ymax, Zmax), Xmin(Xmin), Ymin(Ymin), Zmin(Zmin) {
            }
        };

        struct DistributionStructure {
            // Members
            MeshDistribution thisMeshDistribution = MeshDistribution::Uniform;
            int RegionMeshCount = 0;
            Nandle GeometricSeriesCoefficient = 0.0;

            // Default Constructor
            DistributionStructure() = default;
        };

        struct MeshProperties {
            // Members
            DistributionStructure X;
            DistributionStructure Y;
            DistributionStructure Z;

            // Default Constructor
            MeshProperties() = default;
        };

        struct SimulationControl {
            // Members
            Nandle MinimumTemperatureLimit = -1000;
            Nandle MaximumTemperatureLimit = 1000;
            Nandle Convergence_CurrentToPrevIteration = 0.0;
            int MaxIterationsPerTS = 0;

            // Default Constructor
            SimulationControl() = default;
        };

        struct BasementZoneInfo {
            // Members
            Nandle Depth = 0;  // m
            Nandle Width = 0;  // m
            Nandle Length = 0; // m
            bool ShiftPipesByWidth = false;
            std::string WallBoundaryOSCMName = "";
            int WallBoundaryOSCMIndex = 0;
            std::string FloorBoundaryOSCMName = "";
            int FloorBoundaryOSCMIndex = 0;
            std::vector<int> WallSurfacePointers;
            std::vector<int> FloorSurfacePointers;
            int BasementWallXIndex = -1;
            int BasementFloorYIndex = -1;

            // Default Constructor
            BasementZoneInfo() = default;
        };

        struct MeshPartitions {
            // Members
            std::vector<MeshPartition> X;
            std::vector<MeshPartition> Y;
            std::vector<MeshPartition> Z;

            // Default Constructor
            MeshPartitions() = default;
        };

        struct MoistureInfo {
            // Members
            Nandle Theta_liq = 0.3; // volumetric moisture content of the soil
            Nandle Theta_sat = 0.5; // volumetric moisture content of soil at saturation
            Nandle GroundCoverCoefficient = 0.408;
            Nandle rhoCP_soil_liq = 0.0;
            Nandle rhoCP_soil_transient = 0.0;
            Nandle rhoCP_soil_ice = 0.0;
            Nandle rhoCp_soil_liq_1 = 0.0;

            // Default Constructor
            MoistureInfo() = default;
        };

        struct CurSimConditionsInfo {
            // Members
            // Simulation conditions
            Nandle PrevSimTimeSeconds = -1.0;
            Nandle CurSimTimeSeconds = 0.0;
            Nandle CurSimTimeStepSize = 0.0;
            // Environmental conditions
            Nandle CurAirTemp = 10.0;
            Nandle CurWindSpeed = 2.6;
            Nandle CurIncidentSolar = 0.0;
            Nandle CurRelativeHumidity = 100.0;

            // Default Constructor
            CurSimConditionsInfo() = default;
        };

        struct Segment {
            // Members
            // ID
            std::string Name = "";
            // Misc inputs
            PointF PipeLocation;
            Point PipeCellCoordinates;
            SegmentFlow FlowDirection = SegmentFlow::IncreasingZ;
            // Reporting variables
            Nandle InletTemperature = 0.0;
            Nandle OutletTemperature = 0.0;
            Nandle FluidHeatLoss = 0.0;
            // Error handling flags
            bool PipeCellCoordinatesSet = false;
            // Other flags
            bool IsActuallyPartOfAHorizontalTrench = false;

            // Default Constructor
            Segment() = default;

            void initPipeCells(int x, int y);

            bool operator==(std::string const & a) {
                return this->Name == a;
            }

            static Segment *factory(std::string segmentName);
        };

        struct Circuit : public PlantComponent {

            // Members
            // ID
            std::string Name = "";
            // Inlet and outlet information
            std::string InletNodeName = "";
            std::string OutletNodeName = "";
            int InletNodeNum = 0;
            int OutletNodeNum = 0;
            Point3DInteger CircuitInletCell;
            Point3DInteger CircuitOutletCell;
            // Names and pointers to pipe segments found in this pipe circuit
            std::vector<Segment *> pipeSegments;
            // Pointer to the domain which contains this pipe circuit
            int ParentDomainIndex = 0;
            // Misc inputs
            RadialSizing PipeSize;
            RadialSizing InsulationSize;
            Nandle RadialMeshThickness = 0.0;
            bool HasInsulation = false;
            Nandle DesignVolumeFlowRate = 0.0;
            Nandle DesignMassFlowRate = 0.0;
            Nandle Convergence_CurrentToPrevIteration = 0.0;
            int MaxIterationsPerTS = 0;
            int NumRadialCells = 0;
            BaseThermalPropertySet PipeProperties;
            BaseThermalPropertySet InsulationProperties;
            // Flags
            bool NeedToFindOnPlantLoop = true;
            bool IsActuallyPartOfAHorizontalTrench = false;
            // Location of this pipe circuit in the PlantLoop topology
            int LoopNum = 0;
            int LoopSideNum = 0;
            int BranchNum = 0;
            int CompNum = 0;
            ExtendedFluidProperties CurFluidPropertySet; // is_used
            // Variables used to pass information from INIT-type routines to CALC-type routines
            Nandle CurCircuitInletTemp = 23.0;
            Nandle CurCircuitFlowRate = 0.1321;
            Nandle CurCircuitConvectionCoefficient = 0.0;
            // Reporting variables
            Nandle InletTemperature = 0.0;
            Nandle OutletTemperature = 0.0;
            Nandle FluidHeatLoss = 0.0;

            // Default Constructor
            Circuit() = default;

            virtual ~Circuit() = default;

            void initInOutCells(CartesianCell const &in, CartesianCell const &out);

            static PlantComponent *factory(int objectType, std::string objectName);

            void simulate(const PlantLocation &calledFromLocation, bool FirstHVACIteration, Nandle &CurLoad,
                          bool RunFlag) override;

            bool operator==(std::string const & a) {
                return this->Name == a;
            }

            static Circuit *factory(std::string circuit, bool & errorsFound);
        };

        struct ZoneCoupledSurfaceData {
            // Members
            // ID
            std::string Name;
            // Surface data
            int IndexInSurfaceArray;
            Nandle SurfaceArea;
            Nandle Width;
            Nandle Length;
            Nandle Depth;
            Nandle Conductivity;
            Nandle Density;
            Nandle InsulationConductivity;
            Nandle InsulationDensity;
            int Zone;

            // Default Constructor
            ZoneCoupledSurfaceData()
                    : IndexInSurfaceArray(0), SurfaceArea(0.0), Width(0.0), Length(0.0), Depth(0.0), Conductivity(0.0),
                      Density(0.0),
                      InsulationConductivity(0.0), InsulationDensity(0.0), Zone(0) {
            }
        };

        struct Domain {
            // Members
            // ID
            std::string Name;
            // Names and pointers to circuits found in this domain
            std::vector<Circuit *> circuits;
            int MaxIterationsPerTS;
            // Flag variables
            bool OneTimeInit;
            bool BeginSimInit;
            bool BeginSimEnvironment;
            bool DomainNeedsSimulation;
            bool DomainNeedsToBeMeshed;
            bool IsActuallyPartOfAHorizontalTrench;
            bool HasAPipeCircuit;
            bool HasZoneCoupledSlab;
            bool HasZoneCoupledBasement;
            // "Input" data structure variables
            MeshExtents Extents;
            MeshProperties Mesh;
            BaseThermalPropertySet GroundProperties;
            BaseThermalPropertySet SlabProperties;
            BaseThermalPropertySet BasementInterfaceProperties;
            BaseThermalPropertySet HorizInsProperties;
            BaseThermalPropertySet VertInsProperties;
            SimulationControl SimControls;
            std::shared_ptr<BaseGroundTempsModel> groundTempModel;
            BasementZoneInfo BasementZone;
            MoistureInfo Moisture;
            // "Internal" data structure variables
            MeshPartitions Partitions;
            CurSimConditionsInfo Cur;
            bool HasBasement;
            // Zone coupled variables
            std::vector<ZoneCoupledSurfaceData> ZoneCoupledSurfaces;
            int ZoneCoupledOSCMIndex;
            Nandle PerimeterOffset;
            bool SlabInGradeFlag;
            int SlabMaterialNum;
            Nandle SlabArea;
            Nandle SlabWidth;
            Nandle SlabLength;
            Nandle SlabThickness;
            int XIndex;
            int YIndex;
            int ZIndex;
            int x_max_index;
            int y_max_index;
            int z_max_index;
            bool HorizInsPresentFlag;
            int HorizInsMaterialNum;
            Nandle HorizInsThickness;
            Nandle HorizInsWidth;
            Nandle HeatFlux;
            Nandle WallHeatFlux;
            Nandle FloorHeatFlux;
            Nandle AggregateHeatFlux;
            Nandle AggregateWallHeatFlux;
            Nandle AggregateFloorHeatFlux;
            int NumHeatFlux;
            bool ResetHeatFluxFlag;
            Nandle ConvectionCoefficient;
            bool FullHorizInsPresent;
            bool VertInsPresentFlag;
            int VertInsMaterialNum;
            Nandle VertInsThickness;
            Nandle VertInsDepth;
            int XWallIndex;
            int YFloorIndex;
            int ZWallIndex;
            int InsulationXIndex;
            int InsulationYIndex;
            int InsulationZIndex;
            bool SimTimeStepFlag;
            bool SimHourlyFlag;
            bool SimDailyFlag;
            Nandle ZoneCoupledSurfaceTemp;
            Nandle BasementWallTemp;
            Nandle BasementFloorTemp;
            int NumDomainCells;
            int NumGroundSurfCells;
            int NumInsulationCells;
            int NumSlabCells;
            Array2D<Nandle> WeightingFactor;
            Array2D<Nandle> WeightedHeatFlux;
            Nandle TotalEnergyUniformHeatFlux = 0.0;
            Nandle TotalEnergyWeightedHeatFlux = 0.0;
            Nandle HeatFluxWeightingFactor = 0.0;
            std::vector<GridRegion> XRegions;
            std::vector<GridRegion> YRegions;
            std::vector<GridRegion> ZRegions;

            // Main 3D cells array
            Array3D<CartesianCell> Cells;

            // Dynamic indexes to available neighbor directions for a particular cell
            std::vector<Direction> NeighborFieldCells;
            std::vector<Direction> NeighborBoundaryCells;

            // Default Constructor
            Domain()
                    : MaxIterationsPerTS(10), OneTimeInit(true), BeginSimInit(true), BeginSimEnvironment(true),
                      DomainNeedsSimulation(true),
                      DomainNeedsToBeMeshed(true), IsActuallyPartOfAHorizontalTrench(false), HasAPipeCircuit(true),
                      HasZoneCoupledSlab(false),
                      HasZoneCoupledBasement(false), HasBasement(false), ZoneCoupledOSCMIndex(0), PerimeterOffset(0.0),
                      SlabInGradeFlag(false),
                      SlabMaterialNum(0), SlabArea(0.0), SlabWidth(0.0), SlabLength(0.0), SlabThickness(0.0), XIndex(0),
                      YIndex(0), ZIndex(0), x_max_index(0),
                      y_max_index(0), z_max_index(0), HorizInsPresentFlag(false), HorizInsMaterialNum(0),
                      HorizInsThickness(0.0254), HorizInsWidth(0.0),
                      HeatFlux(0.0), WallHeatFlux(0.0), FloorHeatFlux(0.0), AggregateHeatFlux(0.0),
                      AggregateWallHeatFlux(0.0), AggregateFloorHeatFlux(0.0),
                      NumHeatFlux(0), ResetHeatFluxFlag(true), ConvectionCoefficient(0.0), FullHorizInsPresent(false),
                      VertInsPresentFlag(false), VertInsMaterialNum(0),
                      VertInsThickness(0.0254), VertInsDepth(0.0), XWallIndex(0), YFloorIndex(0), ZWallIndex(0),
                      InsulationXIndex(0), InsulationYIndex(0),
                      InsulationZIndex(0), SimTimeStepFlag(false), SimHourlyFlag(false), SimDailyFlag(false),
                      ZoneCoupledSurfaceTemp(0.0),
                      BasementWallTemp(0.0), BasementFloorTemp(0.0), NumDomainCells(0), NumGroundSurfCells(0),
                      NumInsulationCells(0), NumSlabCells(0) {
                NeighborFieldCells.resize(6);
                NeighborBoundaryCells.resize(6);
            }

            void developMesh();

            void createPartitionCenterList();

            std::vector<GridRegion> createPartitionRegionList(std::vector<MeshPartition> const &ThesePartitionCenters,
                                                              bool PartitionsExist,
                                                              Nandle DirExtentMax);

            void createRegionList(std::vector<GridRegion> &Regions,
                                  std::vector<GridRegion> const &ThesePartitionRegions,
                                  Nandle DirExtentMax,
                                  RegionType DirDirection,
                                  bool PartitionsExist,
                                  Optional_int BasementWallXIndex = _,
                                  Optional_int BasementFloorYIndex = _,
                                  Optional_int XIndex = _,
                                  Optional_int XWallIndex = _,
                                  Optional_int InsulationXIndex = _,
                                  Optional_int YIndex = _,
                                  Optional_int YFloorIndex = _,
                                  Optional_int InsulationYIndex = _,
                                  Optional_int ZIndex = _,
                                  Optional_int ZWallIndex = _,
                                  Optional_int InsulationZIndex = _);

            void createCellArray(std::vector<Nandle> const &XBoundaryPoints, std::vector<Nandle> const &YBoundaryPoints,
                                 std::vector<Nandle> const &ZBoundaryPoints);

            void setupCellNeighbors();

            void setupPipeCircuitInOutCells();

            int getCellWidthsCount(RegionType dir);

            void getCellWidths(GridRegion &g, RegionType direction);

            void addNeighborInformation(int X,
                                        int Y,
                                        int Z,
                                        Direction direction,
                                        Nandle ThisCentroidToNeighborWall,
                                        Nandle ThisWallToNeighborCentroid,
                                        Nandle ThisAdiabaticMultiplier);

            Nandle GetBasementWallHeatFlux();

            Nandle GetBasementFloorHeatFlux();

            void UpdateBasementSurfaceTemperatures();

            Nandle GetZoneInterfaceHeatFlux();

            void UpdateZoneSurfaceTemperatures();

            Nandle GetAverageTempByType(CellType cellType);

            void InitializeSoilMoistureCalcs();

            void EvaluateSoilRhoCp(Nandle CellTemp, Nandle &rhoCp);

            void ShiftTemperaturesForNewTimeStep();

            void ShiftTemperaturesForNewIteration();

            bool IsConverged_CurrentToPrevIteration();

            bool CheckForOutOfRangeTemps();

            void EvaluateNeighborCharacteristics(CartesianCell &ThisCell,
                                                 Direction CurDirection,
                                                 Nandle &NeighborTemp,
                                                 Nandle &Resistance,
                                                 Nandle &AdiabaticMultiplier);

            void EvaluateCellNeighborDirections(CartesianCell const &cell, int &NumFieldCells, int &NumBoundaryCells);

            void DoEndOfIterationOperations(bool &Finished);

            void DoOneTimeInitializations(Circuit * thisCircuit);

            void DoStartOfTimeStepInitializations();

            void DoStartOfTimeStepInitializations(Circuit * thisCircuit);

            Nandle GetFarfieldTemp(CartesianCell const &cell);

            void PreparePipeCircuitSimulation(Circuit * thisCircuit);

            void PerformPipeCircuitSimulation(Circuit * thisCircuit);

            void
            PerformPipeCellSimulation(Circuit * thisCircuit, CartesianCell &ThisCell, Nandle FlowRate, Nandle EnteringTemp);

            void SimulateRadialToCartesianInterface(CartesianCell &ThisCell);

            void PerformTemperatureFieldUpdate();

            Nandle EvaluateFieldCellTemperature(CartesianCell &ThisCell);

            Nandle EvaluateGroundSurfaceTemperature(CartesianCell &cell);

            Nandle EvaluateBasementCellTemperature(CartesianCell &cell);

            Nandle EvaluateZoneInterfaceTemperature(CartesianCell &cell);

            Nandle EvaluateFarfieldBoundaryTemperature(CartesianCell &cell);

            void EvaluateFarfieldCharacteristics(CartesianCell &cell, Direction direction, Nandle &neighbortemp,
                                                 Nandle &resistance, Nandle &adiabaticMultiplier);

            void PerformIterationLoop();

            void PerformIterationLoop(Circuit * thisCircuit);

            void InitPipingSystems(Circuit * thisCircuit);

            void UpdatePipingSystems(Circuit * thisCircuit);

            void SetupZoneCoupledOutputVariables();

        };

        // Object Data
        extern std::vector<Domain> domains;
        extern std::vector<Circuit> circuits;
        extern std::vector<Segment> segments;

        void clear_state();

        void SimulateGroundDomains(OutputFiles &outputFiles, bool initOnly);

        void CheckIfAnySlabs();

        void CheckIfAnyBasements();

        void GetPipingSystemsAndGroundDomainsInput();

        void ReadGeneralDomainInputs(int IndexStart, int NumGeneralizedDomains, bool &ErrorsFound);

        void ReadZoneCoupledDomainInputs(int StartingDomainNumForZone, int NumZoneCoupledDomains, bool &ErrorsFound);

        void ReadBasementInputs(int StartingDomainNumForBasement, int NumBasements, bool &ErrorsFound);
        
        bool SiteGroundDomainUsingNoMassMat(Nandle const MaterialThickness,
                                            int const MaterialNum);
        
        void SiteGroundDomainNoMassMatError(std::string const &FieldName,
                                            std::string const &UserInputField,
                                            std::string const &ObjectName);

        void ReadPipeCircuitInputs(bool &ErrorsFound);

        void ReadPipeSegmentInputs(bool &ErrorsFound);

        void ReadHorizontalTrenchInputs(int StartingDomainNumForHorizontal,
                                        int StartingCircuitNumForHorizontal,
                                        bool &ErrorsFound);

        void SetupPipingSystemOutputVariables();

        void IssueSevereInputFieldError(std::string const &RoutineName,
                                        std::string const &ObjectName,
                                        std::string const &InstanceName,
                                        std::string const &FieldName,
                                        std::string const &FieldEntry,
                                        std::string const &Condition,
                                        bool &ErrorsFound);

        void IssueSevereInputFieldError(std::string const &RoutineName,
                                        std::string const &ObjectName,
                                        std::string const &InstanceName,
                                        std::string const &FieldName,
                                        Nandle FieldEntry,
                                        std::string const &Condition,
                                        bool &ErrorsFound);

        int GetSurfaceCountForOSCM(int OSCMIndex);

        std::vector<int> GetSurfaceIndecesForOSCM(int OSCMIndex);

        std::vector<ZoneCoupledSurfaceData> GetSurfaceDataForOSCM(int OSCMIndex);

        bool inline IsInRangeReal(Nandle const r, Nandle const lower, Nandle const upper) {
            return ((r >= lower) && (r <= upper));
        }

        bool inline IsInRange_BasementModel(Nandle const r, Nandle const lower, Nandle const upper) {
            return ((r >= lower) && (r < upper));
        }

        void ShiftPipeTemperaturesForNewIteration(CartesianCell &ThisPipeCell);

        std::vector<Nandle>
        CreateBoundaryList(std::vector<GridRegion> const &RegionList, Nandle DirExtentMax, RegionType DirDirection);

        void SimulateOuterMostRadialSoilSlice(Circuit * thisCircuit, CartesianCell &ThisCell);

        void SimulateAllInteriorRadialSoilSlices(CartesianCell &ThisCell);

        void SimulateInnerMostRadialSoilSlice(Circuit * thisCircuit, CartesianCell &ThisCell);

        void SimulateRadialInsulationCell(CartesianCell &ThisCell);

        void SimulateRadialPipeCell(Circuit * thisCircuit, CartesianCell &ThisCell);

        void SimulateFluidCell(Circuit * thisCircuit, CartesianCell &ThisCell, Nandle FlowRate, Nandle EnteringFluidTemp);

        bool IsConverged_PipeCurrentToPrevIteration(Circuit * thisCircuit, CartesianCell const &CellToCheck);

    } // namespace PlantPipingSystemsManager

} // namespace EnergyPlus

#endif
