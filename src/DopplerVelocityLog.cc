/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <optional>
#include <unordered_map>
#include <vector>

// TODO(hidmic): implement SVD in gazebo?
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>

#include <gz/common/Console.hh>
#include <gz/common/Event.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>

#include <gz/math/TimeVaryingVolumetricGrid.hh>

#include <gz/msgs/dvl_beam_state.pb.h>
#include <gz/msgs/dvl_kinematic_estimate.pb.h>
#include <gz/msgs/dvl_range_estimate.pb.h>
#include <gz/msgs/dvl_tracking_target.pb.h>
#include <gz/msgs/dvl_velocity_tracking.pb.h>
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/marker_v.pb.h>
#include <gz/msgs/Utility.hh>

#include <gz/rendering/Camera.hh>
#include <gz/rendering/GpuRays.hh>
#include <gz/rendering/RayQuery.hh>

#include <gz/sensors/DopplerVelocityLog.hh>
#include <gz/sensors/GaussianNoiseModel.hh>
#include <gz/sensors/Manager.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/RenderingEvents.hh>
#include <gz/sensors/RenderingSensor.hh>
#include <gz/sensors/SensorTypes.hh>

#include <gz/transport/Node.hh>

namespace gz
{
  namespace sensors
  {
    namespace
    {

      using RowMajorMatrix3d = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

      /// \brief Axis-aligned patch on a plane, using image frame conventions.
      template <typename T>
      class AxisAlignedPatch2
      {
        public: AxisAlignedPatch2() = default;

        public: AxisAlignedPatch2(
            const gz::math::Vector2<T> &_topLeft,
            const gz::math::Vector2<T> &_bottomRight)
          : topLeft(_topLeft), bottomRight(_bottomRight)
        {
        }

        /// \brief Scalar converting copy constructor
        public: template<typename U>
        // cppcheck-suppress noExplicitConstructor
        AxisAlignedPatch2(const AxisAlignedPatch2<U> &_other)
        {
          this->topLeft.X(static_cast<T>(_other.XMax()));
          this->topLeft.Y(static_cast<T>(_other.YMax()));
          this->bottomRight.X(static_cast<T>(_other.XMin()));
          this->bottomRight.Y(static_cast<T>(_other.YMin()));
        }

        public: T XMax() const { return this->topLeft.X(); }

        public: T XMin() const { return this->bottomRight.X(); }

        public: T XSize() const { return this->XMax() - this->XMin(); }

        public: T YMax() const { return this->topLeft.Y(); }

        public: T YMin() const { return this->bottomRight.Y(); }

        public: T YSize() const { return this->YMax() - this->YMin(); }

        /// \brief Merge patch with `_other`.
        /// \return a patch that includes both.
        public: AxisAlignedPatch2<T>& Merge(const AxisAlignedPatch2<T> &_other)
        {
          this->topLeft.Set(
              std::max(this->topLeft.X(), _other.topLeft.X()),
              std::max(this->topLeft.Y(), _other.topLeft.Y()));
          this->bottomRight.Set(
              std::min(this->bottomRight.X(), _other.bottomRight.X()),
              std::min(this->bottomRight.Y(), _other.bottomRight.Y()));
          return *this;
        }

        /// \brief Flip patch, sending each corner to the opposite quadrant.
        public: AxisAlignedPatch2<T> Flip() const
        {
          return {-this->bottomRight, -this->topLeft};
        }

        /// \brief Broadcast multiply corner coordinates by `_vector`
        /// coordinates.
        const AxisAlignedPatch2<T> operator*(gz::math::Vector2<T> _vector) const
        {
          return {this->topLeft * _vector, this->bottomRight * _vector};
        }

        /// \brief Broadcast divide corner coordinates by `_vector`
        /// coordinates.
        const AxisAlignedPatch2<T> operator/(gz::math::Vector2<T> _vector) const
        {
          return {this->topLeft / _vector, this->bottomRight / _vector};
        }

        /// \brief Broadcast sum corner coordinates with `_vector` coordinates.
        const AxisAlignedPatch2<T> operator+(gz::math::Vector2<T> _vector) const
        {
          return {this->topLeft + _vector, this->bottomRight + _vector};
        }

        /// \brief Broadcast substract corner coordinate with `_vector`
        /// coordinates.
        const AxisAlignedPatch2<T> operator-(gz::math::Vector2<T> _vector) const
        {
          return {this->topLeft - _vector, this->bottomRight - _vector};
        }

        /// \brief Upper-left corner i.e. (x, y) maxima
        private: gz::math::Vector2<T> topLeft;

        /// \brief Bottom-right corner i.e (x, y) minima
        private: gz::math::Vector2<T> bottomRight;
      };

      // Handy type definitions
      using AxisAlignedPatch2d = AxisAlignedPatch2<double>;
      using AxisAlignedPatch2i = AxisAlignedPatch2<int>;

      /// \brief Acoustic DVL beam, modelled as a circular cone with aperture
      /// angle α and its apex at the origin. Its axis of symmetry is nominally
      /// aligned with the x-axis of an x-forward, y-left, z-up frame
      /// (following usual Gazebo frame conventions, typically facing
      /// downwards).
      ///
      ///          +            The cone may be tilted w.r.t. the x-axis
      ///         /|\           and rotated about the same to accomodate
      ///        / | \          different beam arrangements, in that order.
      ///       /  |  \         That is, an extrinsic XY rotation applies.
      ///      /   v   \        /
      ///         x
      ///      |-------|
      ///          α
      class AcousticBeam
      {
        /// \brief Acoustic beam constructor.
        /// \param[in] _id ID of the beam. Ought to be unique.
        /// \param[in] _apertureAngle Aperture angle α of the beam.
        /// \param[in] _rotationAngle Rotation angle ψ of the beam
        /// i.e. a rotation about the x-axis of its frame.
        /// \param[in] _tiltAngle Tilt angle φ of the
        /// beam i.e. a rotation about the y-axis of its frame,
        /// away from the x-axis. Must lie in the (-90, 90) degrees
        /// interval.
        public: AcousticBeam(
            const int _id,
            const gz::math::Angle _apertureAngle,
            const gz::math::Angle _rotationAngle,
            const gz::math::Angle _tiltAngle)
          : id(_id), apertureAngle(_apertureAngle),
            normalizedRadius(std::atan(_apertureAngle.Radian() / 2.))
        {
          // Use extrinsic XY convention (as it is easier to reason about)
          using Quaterniond = gz::math::Quaterniond;
          this->transform.Rot() =
            Quaterniond::EulerToQuaternion(_rotationAngle.Radian(), 0., 0.) *
            Quaterniond::EulerToQuaternion(0., _tiltAngle.Radian(), 0.);
          this->axis = this->transform.Rot() * gz::math::Vector3d::UnitX;
          const gz::math::Angle azimuthAngle =
              std::atan2(this->axis.Y(), this->axis.X());
          const gz::math::Angle inclinationAngle =
              std::atan2(this->axis.Z(), std::sqrt(
                  std::pow(this->axis.X(), 2.) +
                  std::pow(this->axis.Y(), 2.)));
          const gz::math::Vector2d topLeft{
            (azimuthAngle + _apertureAngle / 2.).Radian(),
            (inclinationAngle + _apertureAngle / 2.).Radian()};
          const gz::math::Vector2d bottomRight{
            (azimuthAngle - _apertureAngle / 2.).Radian(),
            (inclinationAngle - _apertureAngle / 2.).Radian()};
          this->sphericalFootprint = AxisAlignedPatch2d{topLeft, bottomRight};
        }

        public: int Id() const { return this->id; }

        public: const gz::math::Pose3d &Transform() const
        {
          return this->transform;
        }

        public: const gz::math::Vector3d &Axis() const
        {
          return this->axis;
        }

        public: double NormalizedRadius() const
        {
          return this->normalizedRadius;
        }

        public: const gz::math::Angle &ApertureAngle() const
        {
          return this->apertureAngle;
        }

        public: const AxisAlignedPatch2d &SphericalFootprint() const
        {
          return this->sphericalFootprint;
        }

        private: int id;

        private: gz::math::Angle apertureAngle;

        private: double normalizedRadius;

        private: gz::math::Pose3d transform;

        private: gz::math::Vector3d axis;

        private: AxisAlignedPatch2d sphericalFootprint;
      };

      /// \brief DVL acoustic beam reflecting target.
      ///
      /// Pose is defined w.r.t. to the beams frame.
      struct TrackingTarget
      {
        gz::math::Pose3d pose;
        uint64_t entity;
      };

      /// \brief DVL tracking mode update info.
      ///
      /// Useful for performance comparison.
      struct TrackingModeInfo
      {
        double totalVariance{
          std::numeric_limits<double>::infinity()};
        size_t numBeamsLocked{0};
      };

      /// \brief DVL tracking mode multi-state switch
      ///
      /// An enum class-like POD type that can be used
      /// in boolean and switch contexts for control flow
      class TrackingModeSwitch
      {
        public: enum Value {Off, On, Best};

        public: static bool fromString(
          const std::string &_string,
          TrackingModeSwitch &_value)
        {
          if (_string == "never")
          {
            _value = TrackingModeSwitch::Off;
            return true;
          }
          if (_string == "always")
          {
            _value = TrackingModeSwitch::On;
            return true;
          }
          if (_string == "best")
          {
            _value = TrackingModeSwitch::Best;
            return true;
          }
          return false;
        }

        public: TrackingModeSwitch() = default;

        // cppcheck-suppress noExplicitConstructor
        public: TrackingModeSwitch(Value _value) : value(_value) {}

        public: operator Value() const { return value; }

        public: explicit operator bool() const { return value != Off; }

        private: Value value;
      };

      /// \brief A time-varying vector field built on
      /// per-axis time-varying volumetric data grids
      ///
      /// \see gz::math::InMemoryTimeVaryingVolumetricGrid
      template <typename T, typename V = T, typename P = T>
      class InMemoryTimeVaryingVectorField
      {
        public: using SessionT = gz::math::InMemorySession<T, P>;

        public: using GridT =
          gz::math::InMemoryTimeVaryingVolumetricGrid<T, V, P>;

        /// \brief Default constructor.
        public: InMemoryTimeVaryingVectorField() = default;

        /// \brief Constructor
        /// \param[in] _xData X-axis volumetric data grid.
        /// \param[in] _yData Y-axis volumetric data grid.
        /// \param[in] _zData Z-axis volumetric data grid.
        public: explicit InMemoryTimeVaryingVectorField(
            const GridT *_xData, const GridT *_yData, const GridT *_zData)
          : xData(_xData), yData(_yData), zData(_zData)
        {
          if (this->xData)
          {
            this->xSession = this->xData->CreateSession();
          }
          if (this->yData)
          {
            this->ySession = this->yData->CreateSession();
          }
          if (this->zData)
          {
            this->zSession = this->zData->CreateSession();
          }
        }

        /// \brief Advance vector field in time.
        /// \param[in] _now Time to step data grids to.
        public: void StepTo(const std::chrono::steady_clock::duration &_now)
        {
          const T now = std::chrono::duration<T>(_now).count();
          if (this->xData && this->xSession)
          {
            this->xSession = this->xData->StepTo(this->xSession.value(), now);
          }
          if (this->yData && this->ySession)
          {
            this->ySession = this->yData->StepTo(this->ySession.value(), now);
          }
          if (this->zData && this->zSession)
          {
            this->zSession = this->zData->StepTo(this->zSession.value(), now);
          }
        }

        /// \brief Look up vector field value, interpolating data grids.
        /// \param[in] _pos Vector field argument.
        /// \return vector field value at `_pos`
        public: gz::math::Vector3<V> LookUp(const gz::math::Vector3<P> &_pos)
        {
          auto outcome = gz::math::Vector3<V>::Zero;
          if (this->xData && this->xSession)
          {
            const auto interpolation =
                this->xData->LookUp(this->xSession.value(), _pos);
            outcome.X(interpolation.value_or(0.));
          }
          if (this->yData && this->ySession)
          {
            const auto interpolation =
                this->yData->LookUp(this->ySession.value(), _pos);
            outcome.Y(interpolation.value_or(0.));
          }
          if (this->zData && this->zSession)
          {
            const auto interpolation =
                this->zData->LookUp(this->ySession.value(), _pos);
            outcome.Z(interpolation.value_or(0.));
          }
          return outcome;
        }

        /// \brief Session for x-axis volumetric data grid, if any.
        private: std::optional<SessionT> xSession{std::nullopt};

        /// \brief Session for y-axis volumetric data grid, if any.
        private: std::optional<SessionT> ySession{std::nullopt};

        /// \brief Session for z-axis volumetric data grid, if any.
        private: std::optional<SessionT> zSession{std::nullopt};

        /// \brief X-axis volumetric data grid, if any.
        private: const GridT * xData{nullptr};

        /// \brief Y-axis volumetric data grid, if any.
        private: const GridT * yData{nullptr};

        /// \brief Z-axis volumetric data grid, if any.
        private: const GridT * zData{nullptr};
      };

    }

    using namespace gz::msgs;

    /// \brief Implementation for DopplerVelocityLog
    class DopplerVelocityLog::Implementation
    {
      /// \brief SDF DOM object
      public: sdf::ElementPtr sensorSdf;

      public: using DVLType = DVLVelocityTracking_DVLType;

      /// \brief Dictionary of known DVL types
      public: const std::unordered_map<std::string, DVLType> knownDVLTypes{
        {"piston", DVLVelocityTracking::DVL_TYPE_PISTON},
        {"phased_array", DVLVelocityTracking::DVL_TYPE_PHASED_ARRAY}};

      /// \brief Type of DVL
      public: DVLType dvlType;

      /// \brief Sensor entity ID (for world state lookup)
      public: uint64_t entityId{0};

      /// \brief true if Load() has been called and was successful
      public: bool initialized = false;

      /// \brief Initialize DVL sensor
      public: bool Initialize(DopplerVelocityLog *_sensor);

      /// \brief Initialize beam arrangement for DVL sensor
      ///
      /// This primarily creates rendering sensors.
      public: bool InitializeBeamArrangement(DopplerVelocityLog *_sensor);

      /// \brief Initialize tracking modes for DVL sensor
      ///
      /// This sets up bottom and/or water-mass tracking modes, as needed.
      public: bool InitializeTrackingModes(DopplerVelocityLog *_sensor);

      /// \brief Maximum range for DVL beams.
      public: double maximumRange;

      /// \brief Depth sensor resolution at 1 m distance, in meters.
      public: double resolution;

      /// \brief Whether bottom tracking mode is enabled
      /// and which variant if it is.
      public: TrackingModeSwitch bottomModeSwitch{TrackingModeSwitch::Off};

      /// \brief Perform bottom tracking.
      /// \param[in] _now Current simulation time.
      /// \param[out] _info Optional tracking mode info,
      /// useful for performance comparison.
      /// \return velocity tracking result.
      public: DVLVelocityTracking TrackBottom(
          const std::chrono::steady_clock::duration &_now,
          TrackingModeInfo *_info);

      /// \brief Whether water-mass tracking mode is enabled
      /// and which variant if it is.
      public: TrackingModeSwitch waterMassModeSwitch{TrackingModeSwitch::Off};

      /// \brief Perform water-mass tracking.
      /// \param[in] _now Current simulation time.
      /// \param[out] _info Optional tracking mode info,
      /// useful for performance comparison.
      /// \return velocity tracking result.
      public: DVLVelocityTracking TrackWaterMass(
          const std::chrono::steady_clock::duration &_now,
          TrackingModeInfo *_info);

      /// \brief Number of bins for water-mass sampling.
      public: int waterMassModeNumBins;

      /// \brief Water-mass sampling bin height, in meters.
      public: double waterMassModeBinHeight;

      /// \brief Distance to nearest water-mass boundary, in meters.
      public: double waterMassModeNearBoundary;

      /// \brief Distance to farthest water-mass boundary, in meters.
      public: double waterMassModeFarBoundary;

      /// \brief Bottom tracking mode noise model
      public:
      std::shared_ptr<gz::sensors::GaussianNoiseModel> bottomModeNoise;

      /// \brief Water-mass tracking mode noise model
      public:
      std::shared_ptr<gz::sensors::GaussianNoiseModel> waterMassModeNoise;

      /// \brief State of the world.
      public: const WorldState *worldState;

      /// \brief Water velocity vector field for water-mass sampling.
      public:
      std::optional<InMemoryTimeVaryingVectorField<double>> waterVelocity;

      /// \brief Water velocity data shape, as dimension names,
      /// for environmental data indexing.
      public: std::array<std::string, 3> waterVelocityShape;

      /// \brief Reference system in which coordinates for water-mass
      /// sampling are to be defined in.
      public: EnvironmentalData::ReferenceT waterVelocityReference;

      /// \brief Whether water velocity was updated since last use.
      public: bool waterVelocityUpdated{true};

      /// \brief Depth sensor (i.e. a GPU raytracing sensor).
      public: gz::rendering::GpuRaysPtr depthSensor;

      /// \brief Image sensor (i.e. a camera sensor) to aid ray querys.
      public: gz::rendering::CameraPtr imageSensor;

      /// \brief Depth sensor intrinsic constants
      public: struct {
        gz::math::Vector2d offset;  ///<! Azimuth and elevation offsets
        gz::math::Vector2d step;  ///<! Azimuth and elevation steps
      } depthSensorIntrinsics;

      /// \brief Callback for rendering sensor frames
      public: void OnNewFrame(
          const float *_scan, unsigned int _width,
          unsigned int _height, unsigned int _channels,
          const std::string & /*_format*/);

      /// \brief Connection from depth camera with new depth data.
      public: gz::common::ConnectionPtr depthConnection;

      /// \brief Connection to the Manager's scene change event.
      public: gz::common::ConnectionPtr sceneChangeConnection;

      /// \brief DVL acoustic beams' description
      public: std::vector<AcousticBeam> beams;

      /// \brief Rotation from sensor frame to reference frame.
      ///
      /// Useful to cope with different DVL frame conventions.
      public: gz::math::Quaterniond referenceFrameRotation;

      /// \brief Transform from sensor frame to acoustic beams' frame.
      ///
      /// I.e. x-forward, y-left, z-up (dvl sensor frame) rotates to
      /// x-down, y-left, z-forward (acoustic beams' frame).
      public: const gz::math::Pose3d beamsFrameTransform{
        gz::math::Vector3d::Zero,
        gz::math::Quaterniond{0., GZ_PI/2., 0.}};

      /// \brief DVL acoustic beams' targets
      public: std::vector<std::optional<TrackingTarget>> beamTargets;

      /// \brief DVL acoustic beams' patches in depth scan frame.
      public: std::vector<AxisAlignedPatch2i> beamScanPatches;

      /// \brief Node to create a topic publisher with.
      public: gz::transport::Node node;

      /// \brief Publisher for velocity tracking messages.
      public: gz::transport::Node::Publisher pub;

      /// \brief Flag to indicate if sensor should be publishing estimates.
      public: bool publishingEstimates = false;

      /// \brief Setup beam markers for a generic tracking mode.
      /// \param[in] _sensor (Outer) DVL sensor holding beam arrangement.
      /// \param[in] _namespace Namespace to tell markers apart.
      /// \return tracking mode beam markers
      public: gz::msgs::Marker_V SetupBeamMarkers(
          DopplerVelocityLog *_sensor,
          const std::string &_namespace);

      /// \brief Update beam markers based on tracking output
      /// both locally and remotely (by publishing).
      ///
      /// Beam markers are assumed to have been setup
      /// by calling `SetupBeamMarkers()`.
      ///
      /// \param[in] _sensor (Outer) DVL sensor performing the tracking.
      /// \param[in] _trackingMessage Velocity estimate message.
      /// \param[inout] _beamMarkers Beam markers to update.
      public: void UpdateBeamMarkers(
          DopplerVelocityLog *_sensor,
          const DVLVelocityTracking &_trackingMessage,
          gz::msgs::Marker_V *_beamMarkers);

      /// \brief Bottom tracking mode beam lobe markers.
      public: gz::msgs::Marker_V bottomModeBeamMarkers;

      /// \brief Whether to display bottom tracking mode beams.
      public: bool visualizeBottomModeBeams = false;

      /// \brief Water-mass tracking mode beam lobe markers.
      public: gz::msgs::Marker_V waterMassModeBeamMarkers;

      /// \brief Whether to display water-mass tracking mode beams.
      public: bool visualizeWaterMassModeBeams = false;
    };

    //////////////////////////////////////////////////
    DopplerVelocityLog::DopplerVelocityLog()
      : dataPtr(new Implementation())
    {
    }

    //////////////////////////////////////////////////
    DopplerVelocityLog::~DopplerVelocityLog()
    {
      this->dataPtr->depthConnection.reset();
      this->dataPtr->sceneChangeConnection.reset();
    }

    //////////////////////////////////////////////////
    bool DopplerVelocityLog::Load(const sdf::Sensor &_sdf)
    {
      if (!gz::sensors::RenderingSensor::Load(_sdf))
      {
        return false;
      }

      // Check if this sensor is of the right type
      if (_sdf.Type() != sdf::SensorType::CUSTOM)
      {
        gzerr << "Expected [" << this->Name() << "] sensor to be "
               << "a DVL but found a " << _sdf.TypeStr() << "."
               << std::endl;
        return false;
      }

      sdf::ElementPtr elem = _sdf.Element();
      if (!elem->HasAttribute("gz:type"))
      {
        gzerr << "Missing 'gz:type' attribute "
               << "for sensor [" << this->Name() << "]. "
               << "Aborting load." << std::endl;
        return false;
      }
      const auto type = elem->Get<std::string>("gz:type");
      if (type != "dvl")
      {
        gzerr << "Expected sensor [" << this->Name() << "] to be a "
               << "DVL but it is of '" << type << "' type. Aborting load."
               << std::endl;
        return false;
      }
      if (!elem->HasElement("gz:dvl"))
      {
        gzerr << "Missing 'gz:dvl' configuration for "
               << "sensor [" << this->Name() << "]. "
               << "Aborting load." << std::endl;
        return false;
      }
      this->dataPtr->sensorSdf = elem->GetElement("gz:dvl");

      // Instantiate interfaces
      this->dataPtr->pub =
          this->dataPtr->node.Advertise<DVLVelocityTracking>(this->Topic());
      if (!this->dataPtr->pub)
      {
        gzerr << "Unable to create publisher on topic "
               << "[" << this->Topic() << "] for sensor "
               << "[" << this->Name() << "]" << std::endl;
        return false;
      }

      // Setup sensors
      if (this->Scene())
      {
        if (!this->dataPtr->Initialize(this))
        {
          gzerr << "Failed to setup [" << this->Name() << "] sensor. "
                 << std::endl;
          return false;
        }
      }

      gzmsg << "Loaded [" << this->Name() << "] DVL sensor." << std::endl;
      this->dataPtr->sceneChangeConnection =
          gz::sensors::RenderingEvents::ConnectSceneChangeCallback(
          std::bind(&DopplerVelocityLog::SetScene,
                    this, std::placeholders::_1));

      return true;
    }

    //////////////////////////////////////////////////
    bool DopplerVelocityLog::Load(sdf::ElementPtr _sdf)
    {
      sdf::Sensor sdfSensor;
      sdfSensor.Load(_sdf);
      return this->Load(sdfSensor);
    }

    //////////////////////////////////////////////////
    bool
    DopplerVelocityLog::Implementation::Initialize(DopplerVelocityLog *_sensor)
    {
      gzmsg << "Initializing [" << _sensor->Name() << "] sensor." << std::endl;
      const auto dvlTypeName =
          this->sensorSdf->Get<std::string>("type", "piston").first;
      if (this->knownDVLTypes.count(dvlTypeName) == 0)
      {
        gzerr << "[" << _sensor->Name() << "] specifies an unknown"
               << " DVL type '" << dvlTypeName << "'" << std::endl;
        return false;
      }
      this->dvlType = this->knownDVLTypes.at(dvlTypeName);

      if (!this->InitializeBeamArrangement(_sensor))
      {
        gzerr << "Failed to initialize beam arrangement for "
               << "[" << _sensor->Name() << "] sensor."
               << std::endl;
        return false;
      }

      if (!this->InitializeTrackingModes(_sensor))
      {
        gzerr << "Failed to initialize velocity tracking modes "
               << "for [" << _sensor->Name() << "] sensor."
               << std::endl;
        return false;
      }

      gz::math::Pose3d referenceFrameTransform =
          this->sensorSdf->Get<gz::math::Pose3d>(
              "reference_frame", gz::math::Pose3d{}).first;

      this->referenceFrameRotation = referenceFrameTransform.Rot().Inverse();

      gzmsg << "Initialized [" << _sensor->Name() << "] sensor." << std::endl;
      this->initialized = true;
      return true;
    }

    bool
    DopplerVelocityLog::Implementation::
    InitializeTrackingModes(DopplerVelocityLog *_sensor)
    {
      sdf::ElementPtr trackingElement =
          this->sensorSdf->GetElement("tracking");
      if (!trackingElement)
      {
        gzerr << "No tracking modes specified for "
               << "[" << _sensor->Name() << "] sensor"
               << std::endl;
        return false;
      }

      if (trackingElement->HasElement("bottom_mode"))
      {
        sdf::ElementPtr bottomModeElement =
            trackingElement->GetElement("bottom_mode");

        const std::string switchOptionName =
            bottomModeElement->Get<std::string>("when", "always").first;

        TrackingModeSwitch switchOption;
        if (!TrackingModeSwitch::fromString(switchOptionName,
                                            switchOption))
        {
          gzerr << "Unknown bottom mode option '"
                << switchOptionName << "' for "
                << "[" << _sensor->Name() << "] sensor."
                << std::endl;
          return false;
        }
        this->bottomModeSwitch = switchOption;

        if (this->bottomModeSwitch)
        {
          if (bottomModeElement->HasElement("noise"))
          {
            sdf::ElementPtr bottomModeNoiseElement =
                bottomModeElement->GetElement("noise");
            gzmsg << "Initializing bottom tracking mode for "
                  << "[" << _sensor->Name() << "] sensor." << std::endl;

            sdf::Noise bottomModeNoiseSDF;
            bottomModeNoiseSDF.Load(bottomModeNoiseElement);
            if (bottomModeNoiseSDF.Type() != sdf::NoiseType::GAUSSIAN)
            {
              gzerr << "Unknown bottom mode noise type "
                    << "[" << _sensor->Name() << "] sensor."
                    << std::endl;
              return false;
            }
            gzmsg << "Setting bottom mode noise model for "
                  << "[" << _sensor->Name() << "] sensor."
                  << std::endl;
            this->bottomModeNoise.reset(
                new gz::sensors::GaussianNoiseModel());
            this->bottomModeNoise->Load(bottomModeNoiseSDF);
          }

          this->visualizeBottomModeBeams =
              bottomModeElement->Get<bool>("visualize", false).first;
          if (this->visualizeBottomModeBeams)
          {
            gzmsg << "Enabling bottom mode beams' visual aids for "
                  << "[" << _sensor->Name() << "] sensor." << std::endl;
            this->bottomModeBeamMarkers =
                this->SetupBeamMarkers(_sensor, "bottom_mode");
          }
        }
      }

      if (trackingElement->HasElement("water_mass_mode"))
      {
        sdf::ElementPtr waterMassModeElement =
            trackingElement->GetElement("water_mass_mode");

        const std::string switchOptionName =
            waterMassModeElement->Get<std::string>("when", "always").first;

        TrackingModeSwitch switchOption;
        if (!TrackingModeSwitch::fromString(switchOptionName,
                                            switchOption))
        {
          gzerr << "Unknown water mass mode option '"
                << switchOptionName << "' for "
                << "[" << _sensor->Name() << "] sensor."
                << std::endl;
          return false;
        }
        this->waterMassModeSwitch = switchOption;

        if (this->waterMassModeSwitch)
        {
          gzmsg << "Initializing water-mass tracking mode for "
                << "[" << _sensor->Name() << "] sensor." << std::endl;

          sdf::ElementPtr waterVelocityElement =
              waterMassModeElement->GetElement("water_velocity");

          this->waterVelocityShape[0] =
              waterVelocityElement->Get<std::string>("x");
          this->waterVelocityShape[1] =
              waterVelocityElement->Get<std::string>("y");
          this->waterVelocityShape[2] =
              waterVelocityElement->Get<std::string>("z");

          if (this->waterVelocityShape[0].empty() &&
              this->waterVelocityShape[1].empty() &&
              this->waterVelocityShape[2].empty())
          {
            gzerr << "No water velocity coordinates set for "
                  << "[" << _sensor->Name() << "] sensor."
                  << std::endl;
            return false;
          }
          gzmsg << "Sampling water velocity from ['"
                << this->waterVelocityShape[0]
                << "', '" << this->waterVelocityShape[1]
                << "', '" << this->waterVelocityShape[2]
                << "'] variables in environmental data for "
                << "[" << _sensor->Name() << "] sensor."
                << std::endl;

          this->waterMassModeNumBins =
              waterMassModeElement->Get<int>("bins", 5).first;
          gzmsg << "Using " << this->waterMassModeNumBins
                << " water-mass sampling bins for "
                << "[" << _sensor->Name() << "] sensor."
                << std::endl;

          sdf::ElementPtr waterMassModeBoundariesElement =
              waterMassModeElement->GetElement("boundaries");

          const double defaultNearBoundary = 0.2 * this->maximumRange;
          this->waterMassModeNearBoundary =
              waterMassModeBoundariesElement->Get<double>(
                  "near", defaultNearBoundary).first;

          const double defaultFarBoundary = 0.8 * this->maximumRange;
          this->waterMassModeFarBoundary =
              waterMassModeBoundariesElement->Get<double>(
                  "far", defaultFarBoundary).first;

          if (this->waterMassModeFarBoundary <= this->waterMassModeNearBoundary)
          {
            gzerr << "Far boundary for water mass mode "
                  << "cannot be less than the near boundary "
                  << "for [" << _sensor->Name() << "] sensor."
                  << std::endl;
            return false;
          }

          if (this->maximumRange < this->waterMassModeFarBoundary)
          {
            gzerr << "Far boundary for water mass mode "
                  << "cannot greater than the maximum range "
                  << "for [" << _sensor->Name() << "] sensor."
                  << std::endl;
            return false;
          }

          gzmsg << "Setting water-mass layer boundaries to ["
                << this->waterMassModeNearBoundary << ", "
                << this->waterMassModeFarBoundary << "] for "
                << "[" << _sensor->Name() << "] sensor."
                << std::endl;

          this->waterMassModeBinHeight = (
              this->waterMassModeFarBoundary -
              this->waterMassModeNearBoundary) /
              this->waterMassModeNumBins;

          if (waterMassModeElement->HasElement("noise"))
          {
            sdf::ElementPtr waterMassModeNoiseElement =
              waterMassModeElement->GetElement("noise");

            sdf::Noise waterMassModeNoiseSDF;
            waterMassModeNoiseSDF.Load(waterMassModeNoiseElement);
            if (waterMassModeNoiseSDF.Type() != sdf::NoiseType::GAUSSIAN)
            {
              gzerr << "Unknown water mass mode noise type "
                    << "[" << _sensor->Name() << "] sensor."
                    << std::endl;
              return false;
            }
            gzmsg << "Setting water mass mode noise model for "
                  << "[" << _sensor->Name() << "] sensor."
                  << std::endl;
            this->waterMassModeNoise.reset(
                new gz::sensors::GaussianNoiseModel());
            this->waterMassModeNoise->Load(waterMassModeNoiseSDF);
          }

          this->visualizeWaterMassModeBeams =
              waterMassModeElement->Get<bool>("visualize", false).first;
          if (this->visualizeWaterMassModeBeams)
          {
            gzmsg << "Enabling water mass mode beams' visual aids for "
                  << "[" << _sensor->Name() << "] sensor." << std::endl;
            this->waterMassModeBeamMarkers =
                this->SetupBeamMarkers(_sensor, "water_mass_mode");
          }
        }
      }

      return true;
    }

    //////////////////////////////////////////////////
    gz::msgs::Marker_V
    DopplerVelocityLog::Implementation::SetupBeamMarkers(
        DopplerVelocityLog *_sensor, const std::string &_namespace)
    {
      constexpr double epsilon = std::numeric_limits<double>::epsilon();
      const std::chrono::steady_clock::duration lifetime =
          std::chrono::duration_cast<std::chrono::seconds>(
              1.1 * std::chrono::duration<double>(
                  _sensor->UpdateRate() > epsilon ?
                  1. / _sensor->UpdateRate() : 0.001));

      gz::msgs::Marker_V beamMarkers;
      for (const AcousticBeam & beam : this->beams)
      {
        const double angularResolution =
            this->resolution / beam.NormalizedRadius();
        const size_t lobeNumTriangles =
            static_cast<size_t>(std::ceil(2. * GZ_PI / angularResolution));

        auto * beamLowerQuantileConeMarker = beamMarkers.add_marker();
        beamLowerQuantileConeMarker->set_id(3 * beam.Id());
        beamLowerQuantileConeMarker->set_ns(
            _sensor->Name() + "::" + _namespace + "::beams");
        beamLowerQuantileConeMarker->set_action(gz::msgs::Marker::ADD_MODIFY);
        beamLowerQuantileConeMarker->set_type(gz::msgs::Marker::TRIANGLE_FAN);
        beamLowerQuantileConeMarker->set_visibility(gz::msgs::Marker::GUI);
        *beamLowerQuantileConeMarker->
            mutable_lifetime() = gz::msgs::Convert(lifetime);

        gz::msgs::Set(
            beamLowerQuantileConeMarker->add_point(), gz::math::Vector3d::Zero);
        for (size_t i = 0; i < lobeNumTriangles; ++i)
        {
          gz::msgs::Set(
              beamLowerQuantileConeMarker->add_point(), gz::math::Vector3d{
                1.,
                -beam.NormalizedRadius() * std::cos(i * angularResolution),
                beam.NormalizedRadius() * std::sin(i * angularResolution)
              });
        }
        gz::msgs::Set(
            beamLowerQuantileConeMarker->add_point(),
            gz::math::Vector3d{1., -beam.NormalizedRadius(), 0.});

        auto * beamUpperQuantileConeMarker = beamMarkers.add_marker();
        *beamUpperQuantileConeMarker = *beamLowerQuantileConeMarker;
        beamUpperQuantileConeMarker->set_id(3 * beam.Id() + 1);

        auto * beamCapMarker = beamMarkers.add_marker();
        beamCapMarker->set_id(3 * beam.Id() + 2);
        beamCapMarker->set_ns(
            _sensor->Name() + "::" + _namespace + "::beams");
        beamCapMarker->set_action(gz::msgs::Marker::ADD_MODIFY);
        beamCapMarker->set_type(gz::msgs::Marker::TRIANGLE_FAN);
        beamCapMarker->set_visibility(gz::msgs::Marker::GUI);
        *beamCapMarker->mutable_lifetime() = gz::msgs::Convert(lifetime);

        gz::msgs::Set(beamCapMarker->add_point(),
                      gz::math::Vector3d{1., 0., 0.});
        for (size_t i = 0; i < lobeNumTriangles; ++i)
        {
          gz::msgs::Set(
              beamCapMarker->add_point(), gz::math::Vector3d{
                1.,
                beam.NormalizedRadius() * std::cos(i * angularResolution),
                beam.NormalizedRadius() * std::sin(i * angularResolution)
              });
        }
        gz::msgs::Set(
            beamCapMarker->add_point(),
            gz::math::Vector3d{1., beam.NormalizedRadius(), 0.});
      }
      return beamMarkers;
    }

    //////////////////////////////////////////////////
    bool
    DopplerVelocityLog::Implementation::
    InitializeBeamArrangement(DopplerVelocityLog *_sensor)
    {
      this->beams.clear();
      this->beamTargets.clear();

      int defaultBeamId = 0;
      sdf::ElementPtr arrangementElement =
          this->sensorSdf->GetElement("arrangement");
      if (!arrangementElement)
      {
        gzerr << "No beam arrangement specified for "
               << "[" << _sensor->Name() << "] sensor"
               << std::endl;
        return false;
      }
      const bool useDegrees =
          arrangementElement->Get("degrees", false).first;
      const gz::math::Angle angleUnit = useDegrees ? GZ_DTOR(1.) : 1.;

      sdf::ElementPtr beamElement = arrangementElement->GetElement("beam");
      while (beamElement)
      {
        // Fetch acoustic beam specification
        const auto beamId =
            beamElement->Get<int>("id", defaultBeamId).first;
        const auto beamApertureAngle = (
          beamElement->Get<gz::math::Angle>(
            "aperture", gz::math::Angle::HalfPi
          ).first * angleUnit).Normalized();
        const auto beamRotationAngle = (
          beamElement->Get<gz::math::Angle>(
            "rotation", gz::math::Angle::Zero
            ).first * angleUnit).Normalized();
        const auto beamTiltAngle = (
          beamElement->Get<gz::math::Angle>(
            "tilt", gz::math::Angle::Zero
            ).first * angleUnit).Normalized();
        if (std::abs(beamTiltAngle.Radian()) >=
            std::abs(gz::math::Angle::HalfPi.Radian()))
        {
          gzerr << "Invalid tilt angle for beam #" << beamId
                 << " of [" << _sensor->Name() << "]" << " sensor: "
                 << beamTiltAngle.Radian() << " rads "
                 << "(" << beamTiltAngle.Degree() << " degrees) "
                 << "not in the (-90, 90) degree interval." << std::endl;
          return false;
        }

        // Build acoustic beam
        this->beams.push_back(AcousticBeam{
            beamId, beamApertureAngle, beamRotationAngle,
            beamTiltAngle});

        gzmsg << "Adding acoustic beam #" << beamId
               << " to [" << _sensor->Name() << "] sensor. "
               << "Beam has a " << beamApertureAngle.Radian() << " rads "
               << "(" << beamApertureAngle.Degree() << " degrees) aperture "
               << "angle, it exhibits a " << beamTiltAngle.Radian()
               << " rads (" << beamTiltAngle.Degree() << " degrees) tilt, "
               << "and it is rotated " << beamRotationAngle.Radian()
               << " rads (" << beamRotationAngle.Degree() << " degrees)."
               << std::endl;

        defaultBeamId = this->beams.back().Id() + 1;
        beamElement = beamElement->GetNextElement("beam");
      }

      if (this->beams.size() < 3)
      {
        gzerr << "Expected at least three (3) beams "
               << "for [" << _sensor->Name() << "] sensor."
               << std::endl;
        return false;
      }
      // Add as many (still null) targets as beams
      this->beamTargets.resize(this->beams.size());

      this->depthSensor =
          _sensor->Scene()->CreateGpuRays(
              _sensor->Name() + "_depth_sensor");
      if (!this->depthSensor)
      {
        gzerr << "Failed to create depth sensor for "
               << "for [" << _sensor->Name() << "] sensor."
               << std::endl;
        return false;
      }

      // Aggregate all beams' footprint in spherical coordinates into one
      AxisAlignedPatch2d beamsSphericalFootprint;
      for (const auto & beam : this->beams)
      {
        beamsSphericalFootprint.Merge(beam.SphericalFootprint());
      }
      // Rendering sensors' FOV must be symmetric about its main axis
      beamsSphericalFootprint.Merge(beamsSphericalFootprint.Flip());

      this->resolution  =
          this->sensorSdf->Get<double>("resolution", 0.01).first;
      gzmsg << "Setting beams' resolution to " << this->resolution
            << " m at a 1 m distance for [" << _sensor->Name() << "] sensor."
            << std::endl;

      this->depthSensor->SetAngleMin(beamsSphericalFootprint.XMin());
      this->depthSensor->SetAngleMax(beamsSphericalFootprint.XMax());
      auto horizontalRayCount = static_cast<unsigned int>(
          std::ceil(beamsSphericalFootprint.XSize() /
                    this->resolution));
      if (horizontalRayCount % 2 == 0) ++horizontalRayCount;  // ensure odd
      this->depthSensor->SetRayCount(horizontalRayCount);

      this->depthSensor->SetVerticalAngleMin(
          beamsSphericalFootprint.YMin());
      this->depthSensor->SetVerticalAngleMax(
          beamsSphericalFootprint.YMax());
      auto verticalRayCount = static_cast<unsigned int>(
          std::ceil(beamsSphericalFootprint.YSize() /
                    this->resolution));
      if (verticalRayCount % 2 == 0) ++verticalRayCount;  // ensure odd
      this->depthSensor->SetVerticalRayCount(verticalRayCount);

      auto & intrinsics = this->depthSensorIntrinsics;
      intrinsics.offset.X(beamsSphericalFootprint.XMin());
      intrinsics.offset.Y(beamsSphericalFootprint.YMin());
      intrinsics.step.X(
        beamsSphericalFootprint.XSize() / (horizontalRayCount - 1));
      intrinsics.step.Y(
        beamsSphericalFootprint.YSize() / (verticalRayCount - 1));

      // Pre-compute scan indices covered by beam spherical
      // footprints for speed during scan iteration
      this->beamScanPatches.clear();
      for (const auto & beam : this->beams)
      {
        this->beamScanPatches.push_back(AxisAlignedPatch2i{
            (beam.SphericalFootprint() - intrinsics.offset) /
            intrinsics.step});
      }

      const double minimumRange =
          this->sensorSdf->Get<double>("minimum_range", 0.1).first;
      gzmsg << "Setting minimum range to " << minimumRange
            << " m for [" << _sensor->Name() << "] sensor." << std::endl;
      this->depthSensor->SetNearClipPlane(minimumRange);

      this->maximumRange =
          this->sensorSdf->Get<double>("maximum_range", 100.).first;
      gzmsg << "Setting maximum range to " << this->maximumRange
            << " m for [" << _sensor->Name() << "] sensor." << std::endl;
      this->depthSensor->SetFarClipPlane(this->maximumRange);

      this->depthSensor->SetVisibilityMask(GZ_VISIBILITY_ALL);
      this->depthSensor->SetClamp(false);

      _sensor->AddSensor(this->depthSensor);

      this->imageSensor =
          _sensor->Scene()->CreateCamera(
              _sensor->Name() + "_image_sensor");
      if (!this->imageSensor)
      {
        gzerr << "Failed to create image sensor for "
               << "for [" << _sensor->Name() << "] sensor."
               << std::endl;
        return false;
      }

      this->imageSensor->SetImageWidth(horizontalRayCount);
      this->imageSensor->SetImageHeight(verticalRayCount);

      this->imageSensor->SetNearClipPlane(minimumRange);
      this->imageSensor->SetFarClipPlane(this->maximumRange);
      this->imageSensor->SetAntiAliasing(2);

      this->imageSensor->SetAspectRatio(
          beamsSphericalFootprint.XSize() / beamsSphericalFootprint.YSize());
      this->imageSensor->SetHFOV(beamsSphericalFootprint.XSize());
      this->imageSensor->SetVisibilityMask(~GZ_VISIBILITY_GUI);

      _sensor->AddSensor(this->imageSensor);

      this->depthConnection =
          this->depthSensor->ConnectNewGpuRaysFrame(
              std::bind(&DopplerVelocityLog::Implementation::OnNewFrame, this,
                        std::placeholders::_1, std::placeholders::_2,
                        std::placeholders::_3, std::placeholders::_4,
                        std::placeholders::_5));

      return true;
    }

    //////////////////////////////////////////////////
    std::vector<gz::rendering::SensorPtr>
    DopplerVelocityLog::RenderingSensors() const
    {
      return {this->dataPtr->depthSensor, this->dataPtr->imageSensor};
    }

    //////////////////////////////////////////////////
    void DopplerVelocityLog::Implementation::OnNewFrame(
        const float *_scan, unsigned int _width,
        unsigned int _height, unsigned int _channels,
        const std::string & /*_format*/)
    {
      const auto & intrinsics = this->depthSensorIntrinsics;

      for (size_t i = 0; i < this->beams.size(); ++i)
      {
        const AxisAlignedPatch2i & beamScanPatch =
            this->beamScanPatches[i];
        const AcousticBeam & beam = this->beams[i];

        // Clear existing target, if any
        std::optional<TrackingTarget> & beamTarget = this->beamTargets[i];
        beamTarget.reset();

        // Iterate over the beam solid angle in camera coordinates
        for (auto v = beamScanPatch.YMin(); v < beamScanPatch.YMax(); ++v)
        {
          assert(v >= 0 && v < static_cast<int>(_height));
          const gz::math::Angle inclination =
              v * intrinsics.step.Y() + intrinsics.offset.Y();

          for (auto u = beamScanPatch.XMin(); u < beamScanPatch.XMax(); ++u)
          {
            assert(u >= 0 && u < static_cast<int>(_width));

            const float range = _scan[(u + v * _width) * _channels];
            if (!std::isfinite(range)) continue;

            const gz::math::Angle azimuth =
                u * intrinsics.step.X() + intrinsics.offset.X();

            // Convert to cartesian coordinates in the acoustic beams' frame
            const auto point = range * gz::math::Vector3d{
              std::cos(inclination.Radian()) * std::cos(azimuth.Radian()),
              std::cos(inclination.Radian()) * std::sin(azimuth.Radian()),
              std::sin(inclination.Radian())
            };

            // Track point if (a) it effectively lies within the
            // beam's aperture and (b) it is the closest seen so far
            const gz::math::Angle angle = std::acos(
                point.Normalized().Dot(beam.Axis()));
            if (angle < beam.ApertureAngle() / 2.)
            {
              if (beamTarget)
              {
                if (beamTarget->pose.Pos().Length() > point.Length())
                {
                  beamTarget->pose.Pos() = point;
                }
              }
              else
              {
                beamTarget = {
                  gz::math::Pose3d{
                    point,
                    gz::math::Quaterniond::Identity},
                  0
                };
              }
            }
          }
        }
      }
    }

    /////////////////////////////////////////////////
    void DopplerVelocityLog::SetScene(gz::rendering::ScenePtr _scene)
    {
      // APIs make it possible for the scene pointer to change
      if (this->Scene() != _scene)
      {
        // TODO(anyone) Remove camera from scene
        this->dataPtr->depthSensor = nullptr;
        this->dataPtr->imageSensor = nullptr;
        RenderingSensor::SetScene(_scene);
        if (!this->dataPtr->initialized)
        {
          if (!this->dataPtr->Initialize(this))
          {
            gzerr << "Failed to initialize "
                   << "[" << this->Name() << "]"
                   << " sensor." << std::endl;
          }
        }
      }
    }

    //////////////////////////////////////////////////
    void DopplerVelocityLog::SetWorldState(const WorldState &_state)
    {
      this->dataPtr->worldState = &_state;
    }

    //////////////////////////////////////////////////
    void
    DopplerVelocityLog::SetEnvironmentalData(const EnvironmentalData &_data)
    {
      if (this->dataPtr->waterMassModeSwitch)
      {
        gzmsg << "Updating water velocity data for "
              << "[" << this->Name() << "] sensor."
              << std::endl;

        using VectorFieldT = InMemoryTimeVaryingVectorField<double>;
        const auto & [xDim, yDim, zDim] = this->dataPtr->waterVelocityShape;

        const VectorFieldT::GridT *xData = nullptr;
        if (!xDim.empty())
        {
          if (!_data.frame.Has(xDim))
          {
            gzerr << "No '" << xDim << "' data found "
                  << "in the environment" << std::endl;
            return;
          }
          xData = &_data.frame[xDim];
        }
        const VectorFieldT::GridT *yData = nullptr;
        if (!yDim.empty())
        {
          if (!_data.frame.Has(yDim))
          {
            gzerr << "No '" << yDim << "' data found "
                  << "in the environment" << std::endl;
            return;
          }
          yData = &_data.frame[yDim];
        }
        const VectorFieldT::GridT *zData = nullptr;
        if (!zDim.empty())
        {
          if (!_data.frame.Has(zDim))
          {
            gzerr << "No '" << zDim << "' data found "
                  << "in the environment" << std::endl;
            return;
          }
          zData = &_data.frame[zDim];
        }

        this->dataPtr->waterVelocity = VectorFieldT(xData, yData, zData);
        this->dataPtr->waterVelocityReference = _data.reference;
        this->dataPtr->waterVelocityUpdated = true;

        gzmsg << "Water velocity data updated for "
              << "[" << this->Name() << "] sensor."
              << std::endl;
      }
    }

    //////////////////////////////////////////////////
    void DopplerVelocityLog::SetEntity(uint64_t _entityId)
    {
      this->dataPtr->entityId = _entityId;
    }

    //////////////////////////////////////////////////
    bool
    DopplerVelocityLog::Update(const std::chrono::steady_clock::duration &)
    {
      GZ_PROFILE("DopplerVelocityLog::Update");
      if (!this->dataPtr->initialized || this->dataPtr->entityId == 0)
      {
        gzerr << "Not initialized, update ignored." << std::endl;
        return false;
      }

      if (!this->dataPtr->pub.HasConnections())
      {
        if (this->dataPtr->publishingEstimates)
        {
          gzdbg << "Disabling estimates publication for sensor "
                << "[" << this->Name() << "] as no subscribers"
                << " were found." << std::endl;
          this->dataPtr->publishingEstimates = false;
        }

        if (!this->dataPtr->visualizeBottomModeBeams &&
            !this->dataPtr->visualizeWaterMassModeBeams)
        {
          // Skipping estimation process
          // as nothing is observing them.
          return false;
        }
      }
      else
      {
        if (!this->dataPtr->publishingEstimates)
        {
          gzdbg << "Enabling estimates publication for sensor "
                << "[" << this->Name() << "] as some subscribers "
                << "were found." << std::endl;
          this->dataPtr->publishingEstimates = true;
        }
      }

      const gz::math::Pose3d beamsFramePose =
          this->Pose() * this->dataPtr->beamsFrameTransform;
      this->dataPtr->depthSensor->SetLocalPose(beamsFramePose);
      this->dataPtr->imageSensor->SetLocalPose(beamsFramePose);

      // Generate sensor data
      this->Render();

      return true;
    }

    //////////////////////////////////////////////////
    DVLVelocityTracking
    DopplerVelocityLog::Implementation::TrackBottom(
        const std::chrono::steady_clock::duration &_now,
        TrackingModeInfo *_info)
    {
      // Boostrap velocity tracking message
      DVLVelocityTracking message;
      auto * headerMessage = message.mutable_header();
      *headerMessage->mutable_stamp() = gz::msgs::Convert(_now);
      message.set_type(this->dvlType);

      // Estimate DVL velocity by least squares using beam axes
      // and measured beam speeds ie.
      //
      //  | B0x B0y B0z |           | s0 |
      //  | B1x B1y B1z |  | vx |   | s1 |
      //  |  .   .   .  |  | vy | = | .  |
      //  |  .   .   .  |  | vz |   | .  |
      //  | Bnx Bny Bnz |           | sn |
      //
      // where Bk is the k-th beam axis, v is the velocity to estimate
      // and sk is the k-th beam measured speed.
      size_t numBeamsLocked = 0;
      double targetRange = std::numeric_limits<double>::infinity();
      Eigen::MatrixXd beamBasis(this->beams.size(), 3);
      Eigen::VectorXd beamSpeeds(this->beams.size());
      const EntityKinematicState & sensorStateInWorldFrame =
          this->worldState->kinematics.at(this->entityId);

      const double bottomModeNoiseVariance =
          this->bottomModeNoise ?
          std::pow(this->bottomModeNoise->StdDev(), 2.) : 0.0;

      for (size_t i = 0; i < this->beams.size(); ++i)
      {
        const AcousticBeam & beam = this->beams[i];
        auto * beamMessage = message.add_beams();
        beamMessage->set_id(beam.Id());

        auto & beamTarget = this->beamTargets[i];
        if (beamTarget)
        {
          const double beamRange = beamTarget->pose.Pos().Length();
          auto * beamRangeMessage = beamMessage->mutable_range();
          beamRangeMessage->set_mean(beamRange);

          // Use shortest beam range as target range
          targetRange = std::min(targetRange, beamRange);

          EntityKinematicState targetEntityStateInWorldFrame;
          if (this->worldState->kinematics.count(beamTarget->entity) > 0)
          {
            targetEntityStateInWorldFrame =
                this->worldState->kinematics.at(beamTarget->entity);
          }

          // Transform beam reflecting target pose
          // in the (global) world frame
          const gz::math::Pose3d targetPoseInWorldFrame =
              sensorStateInWorldFrame.pose *
              this->beamsFrameTransform *
              beamTarget->pose;

          // Compute beam reflecting target velocity
          // in the (global) world frame
          const gz::math::Vector3d targetVelocityInWorldFrame =
              targetEntityStateInWorldFrame.linearVelocity +
              targetEntityStateInWorldFrame.angularVelocity.Cross(
                  targetPoseInWorldFrame.Pos() -
                  targetEntityStateInWorldFrame.pose.Pos());

          // Compute DVL velocity w.r.t. target velocity in the sensor frame
          const gz::math::Vector3d relativeSensorVelocityInSensorFrame =
              sensorStateInWorldFrame.pose.Rot().RotateVectorReverse(
                  sensorStateInWorldFrame.linearVelocity -
                  targetVelocityInWorldFrame);

          // Estimate speed as measured by beam (incl. measurement noise)
          const gz::math::Vector3d beamAxisInSensorFrame =
              this->beamsFrameTransform.Rot() * beam.Axis();
          double beamSpeed =
              relativeSensorVelocityInSensorFrame.Dot(beamAxisInSensorFrame);
          if (this->bottomModeNoise)
          {
            beamSpeed = this->bottomModeNoise->Apply(beamSpeed);
          }

          const gz::math::Vector3d beamAxisInReferenceFrame =
              this->referenceFrameRotation * beamAxisInSensorFrame;

          // Set beam velocity mean and covariance in the reference frame
          auto * beamVelocityMessage = beamMessage->mutable_velocity();
          beamVelocityMessage->set_reference(
              DVLKinematicEstimate::DVL_REFERENCE_SHIP);
          *beamVelocityMessage->mutable_mean() =
              gz::msgs::Convert(beamAxisInReferenceFrame * beamSpeed);
          const auto beamBasisElement =
              Eigen::Vector3d{beamAxisInReferenceFrame.X(),
                              beamAxisInReferenceFrame.Y(),
                              beamAxisInReferenceFrame.Z()};
          const RowMajorMatrix3d beamVelocityCovarianceInReferenceFrame =
              bottomModeNoiseVariance *
              beamBasisElement * beamBasisElement.transpose();

          beamVelocityMessage->mutable_covariance()->Resize(9, 0.);
          std::copy(beamVelocityCovarianceInReferenceFrame.data(),
                    beamVelocityCovarianceInReferenceFrame.data() + 9,
                    beamVelocityMessage->mutable_covariance()->begin());

          // Build least squares problem in the reference frame
          beamBasis.row(numBeamsLocked) = beamBasisElement.transpose();
          beamSpeeds(numBeamsLocked) = beamSpeed;
          ++numBeamsLocked;
        }
        beamMessage->set_locked(beamTarget.has_value());
      }

      if (numBeamsLocked >= 3)
      {
        // Enough rows for a unique least squares solution
        const auto svdDecomposition =
            beamBasis.topRows(numBeamsLocked).jacobiSvd(
                Eigen::ComputeThinU | Eigen::ComputeThinV);

        // Estimate DVL velocity mean and covariance in the reference frame
        const Eigen::Vector3d velocityMeanInReferenceFrame =
            svdDecomposition.solve(beamSpeeds.head(numBeamsLocked));
        const Eigen::MatrixXd pseudoInverse = svdDecomposition.solve(
            Eigen::MatrixXd::Identity(numBeamsLocked, numBeamsLocked));
        // Use row-major 1D layout for covariance
        const RowMajorMatrix3d velocityCovarianceInReferenceFrame =
            pseudoInverse * bottomModeNoiseVariance * pseudoInverse.transpose();

        auto * velocityMessage = message.mutable_velocity();
        velocityMessage->set_reference(
            DVLKinematicEstimate::DVL_REFERENCE_SHIP);
        velocityMessage->mutable_mean()->set_x(
            velocityMeanInReferenceFrame.x());
        velocityMessage->mutable_mean()->set_y(
            velocityMeanInReferenceFrame.y());
        velocityMessage->mutable_mean()->set_z(
            velocityMeanInReferenceFrame.z());

        velocityMessage->mutable_covariance()->Resize(9, 0.);
        std::copy(velocityCovarianceInReferenceFrame.data(),
                  velocityCovarianceInReferenceFrame.data() + 9,
                  velocityMessage->mutable_covariance()->begin());

        if (_info)
        {
          // Compute total variance for scoring
          _info->totalVariance = velocityCovarianceInReferenceFrame.trace();
        }
      }

      if (numBeamsLocked > 0)
      {
        auto * targetMessage = message.mutable_target();
        targetMessage->set_type(DVLTrackingTarget::DVL_TARGET_BOTTOM);
        auto * rangeMessage = targetMessage->mutable_range();
        rangeMessage->set_mean(targetRange);
      }
      message.set_status(0);

      if (_info)
      {
        _info->numBeamsLocked = numBeamsLocked;
      }

      return message;
    }

    //////////////////////////////////////////////////
    DVLVelocityTracking
    DopplerVelocityLog::Implementation::TrackWaterMass(
        const std::chrono::steady_clock::duration &_now,
        TrackingModeInfo *_info)
    {
      // Boostrap velocity tracking message
      DVLVelocityTracking message;
      auto * headerMessage = message.mutable_header();
      *headerMessage->mutable_stamp() = gz::msgs::Convert(_now);
      message.set_type(this->dvlType);

      const double waterMassModeNoiseVariance =
          this->waterMassModeNoise ?
          std::pow(this->waterMassModeNoise->StdDev(), 2.) : 0.0;

      // Estimate DVL velocity by least squares using beam axes
      // and average beams speeds ie.
      //
      //  | B0x B0y B0z |               | s0 |
      //  | B1x B1y B1z |  | vx |       | s1 |
      //  |  .   .   .  |  | vy | = E [ | .  | ]
      //  |  .   .   .  |  | vz |       | .  |
      //  | Bnx Bny Bnz |               | sn |
      //
      // where Bk is the k-th beam axis, v is the velocity to estimate
      // and E[sk] is the expected k-th beam measured speed.
      size_t numBeamsLocked = 0;
      double meanTargetRange = std::numeric_limits<double>::infinity();
      double targetRangeVariance = std::numeric_limits<double>::infinity();

      Eigen::MatrixXd beamBasis(this->beams.size(), 3);
      Eigen::VectorXd averageBeamSpeeds(this->beams.size());

      const EntityKinematicState & sensorStateInWorldFrame =
          this->worldState->kinematics.at(this->entityId);

      for (size_t i = 0; i < this->beams.size(); ++i)
      {
        const AcousticBeam & beam = this->beams[i];
        auto * beamMessage = message.add_beams();
        beamMessage->set_id(beam.Id());

        const gz::math::Vector3d beamAxisInSensorFrame =
            this->beamsFrameTransform.Rot() * beam.Axis();

        // Discard beams that do not span both water mass boundaries
        const auto & beamTarget = this->beamTargets[i];
        if (beamTarget)
        {
          const double beamTargetBoundary = std::abs(
            (beamTarget->pose.Pos().Length() * beamAxisInSensorFrame).Z());
          if (beamTargetBoundary < this->waterMassModeFarBoundary)
          {
            // Bottom is too close for water mass tracking
            beamMessage->set_locked(false);
            continue;
          }
        }

        const gz::math::Vector3d beamAxisInReferenceFrame =
            this->referenceFrameRotation * beamAxisInSensorFrame;

        // Assume uniform water density distribution for range estimate.
        auto * beamRangeMessage = beamMessage->mutable_range();
        const double projectionScale =
            1. / -gz::math::Vector3d::UnitZ.Dot(beamAxisInSensorFrame);
        const double meanBeamRange =  projectionScale * (
            this->waterMassModeFarBoundary +
            this->waterMassModeNearBoundary) / 2.;
        const double beamRangeVariance = std::pow(
            projectionScale * (
                this->waterMassModeFarBoundary -
                this->waterMassModeNearBoundary), 2.) / 12.;
        beamRangeMessage->set_mean(meanBeamRange);
        beamRangeMessage->set_variance(beamRangeVariance);

        // Use shortest beam range as target range
        if (meanTargetRange > meanBeamRange)
        {
          meanTargetRange = meanBeamRange;
          targetRangeVariance = beamRangeVariance;
        }
        // Compute beam speed mean and variance using water mass bin samples
        double averageBeamSpeed = 0.;
        double beamSpeedRSS = 0.;
        for (int j = 0; j < this->waterMassModeNumBins; ++j)
        {
          // Compute offset to mid-bin plane in the sensor frame
          // (along the -z-axis)
          const double offsetToBinPlane = (
              this->waterMassModeBinHeight * j +
              this->waterMassModeBinHeight / 2 +
              this->waterMassModeNearBoundary);

          // Compute sample point as the intersection between
          // beam axis and mid-bin plane in the sensor frame
          const gz::math::Vector3d samplePointInSensorFrame =
              (offsetToBinPlane * beamAxisInSensorFrame) /
              -gz::math::Vector3d::UnitZ.Dot(beamAxisInSensorFrame);

          // Transform sample point to the (global) world frame
          const gz::math::Vector3d samplePointInWorldFrame =
              sensorStateInWorldFrame.pose.Pos() +
              sensorStateInWorldFrame.pose.Rot() *
              samplePointInSensorFrame;

          // Transform sample point to the environmental data frame
          const gz::math::Vector3d samplePointInDataFrame =
              this->worldState->origin.PositionTransform(
                  samplePointInWorldFrame,
                  gz::math::SphericalCoordinates::GLOBAL,
                  this->waterVelocityReference);

          // Sample water velocity in the world frame at sample point
          const gz::math::Vector3d sampledVelocityInWorldFrame =
              this->waterVelocity->LookUp(samplePointInDataFrame);

          // Compute DVL velocity w.r.t. sampled water velocity
          // in the sensor frame
          const gz::math::Vector3d relativeSensorVelocityInSensorFrame =
              sensorStateInWorldFrame.pose.Rot().RotateVectorReverse(
                  sensorStateInWorldFrame.linearVelocity -
                  sampledVelocityInWorldFrame);

          // Estimate speed as measured by beam (incl. measurement noise)
          double beamSpeed =
              relativeSensorVelocityInSensorFrame.Dot(beamAxisInSensorFrame);
          if (this->waterMassModeNoise)
          {
            this->waterMassModeNoise->Apply(beamSpeed);
          }

          const double prevAverageBeamSpeed = averageBeamSpeed;
          // Use cumulative average algorithm to avoid keeping samples
          averageBeamSpeed = (beamSpeed + j * prevAverageBeamSpeed) / (j + 1);
          // Use Welford's moving variance algorithm to avoid keeping samples
          beamSpeedRSS +=
              (beamSpeed - prevAverageBeamSpeed) *
              (beamSpeed - averageBeamSpeed);
        }
        const double beamSpeedVariance =
            beamSpeedRSS / (this->waterMassModeNumBins - 1);
        const auto beamBasisElement =
            Eigen::Vector3d{beamAxisInReferenceFrame.X(),
                            beamAxisInReferenceFrame.Y(),
                            beamAxisInReferenceFrame.Z()};
        const Eigen::Matrix3d beamVelocityCovarianceInReferenceFrame =
            beamSpeedVariance * beamBasisElement *
            beamBasisElement.transpose();

        // Set beam velocity mean and covariance in the reference frame
        auto * beamVelocityMessage = beamMessage->mutable_velocity();
        beamVelocityMessage->set_reference(
            DVLKinematicEstimate::DVL_REFERENCE_SHIP);
        *beamVelocityMessage->mutable_mean() =
            gz::msgs::Convert(beamAxisInReferenceFrame * averageBeamSpeed);

        // Use row-major 1D layout for covariance
        beamVelocityMessage->mutable_covariance()->Resize(9, 0.);
        std::copy(beamVelocityCovarianceInReferenceFrame.data(),
                  beamVelocityCovarianceInReferenceFrame.data() + 9,
                  beamVelocityMessage->mutable_covariance()->begin());

        // Build least squares problem in the reference frame
        beamBasis.row(numBeamsLocked) = beamBasisElement;
        averageBeamSpeeds(numBeamsLocked) = averageBeamSpeed;

        ++numBeamsLocked;

        beamMessage->set_locked(true);
      }

      if (numBeamsLocked >= 3)
      {
        // Enough rows for a unique least squares solution
        const auto svdDecomposition =
            beamBasis.topRows(numBeamsLocked).jacobiSvd(
                Eigen::ComputeThinU | Eigen::ComputeThinV);

        // Estimate DVL velocity mean and covariance in the reference frame
        const Eigen::Vector3d velocityMeanInReferenceFrame =
            svdDecomposition.solve(averageBeamSpeeds.head(numBeamsLocked));
        const Eigen::MatrixXd pseudoInverse = svdDecomposition.solve(
            Eigen::MatrixXd::Identity(numBeamsLocked, numBeamsLocked));
        // Use row-major 1D layout for covariance
        const RowMajorMatrix3d velocityCovarianceInReferenceFrame =
            pseudoInverse * waterMassModeNoiseVariance *
            pseudoInverse.transpose();

        auto * velocityMessage = message.mutable_velocity();
        velocityMessage->set_reference(
            DVLKinematicEstimate::DVL_REFERENCE_SHIP);
        velocityMessage->mutable_mean()->set_x(
            velocityMeanInReferenceFrame.x());
        velocityMessage->mutable_mean()->set_y(
            velocityMeanInReferenceFrame.y());
        velocityMessage->mutable_mean()->set_z(
            velocityMeanInReferenceFrame.z());

        velocityMessage->mutable_covariance()->Resize(9, 0.);
        std::copy(velocityCovarianceInReferenceFrame.data(),
                  velocityCovarianceInReferenceFrame.data() + 9,
                  velocityMessage->mutable_covariance()->begin());

        if (_info)
        {
          // Track total variance for scoring
          _info->totalVariance = velocityCovarianceInReferenceFrame.trace();
        }
      }

      if (numBeamsLocked > 0)
      {
        auto * targetMessage = message.mutable_target();
        targetMessage->set_type(DVLTrackingTarget::DVL_TARGET_WATER_MASS);
        auto * rangeMessage = targetMessage->mutable_range();
        rangeMessage->set_mean(meanTargetRange);
        rangeMessage->set_variance(targetRangeVariance);
      }
      message.set_status(0);

      if (_info)
      {
        // Track number of beams locked for scoring
        _info->numBeamsLocked = numBeamsLocked;
      }
      return message;
    }

    //////////////////////////////////////////////////
    void DopplerVelocityLog::PostUpdate(
        const std::chrono::steady_clock::duration &_now)
    {
      GZ_PROFILE("DopplerVelocityLog::PostUpdate");

      if (!this->dataPtr->worldState)
      {
        gzwarn << "No world state available, "
               << "cannot estimate velocities."
               << std::endl;
        return;
      }

      for (size_t i = 0; i < this->dataPtr->beams.size(); ++i)
      {
        auto & beamTarget = this->dataPtr->beamTargets[i];
        if (beamTarget)
        {
          // TODO(hidmic): use shader to fetch target entity id
          const gz::math::Vector2i pixel =
              this->dataPtr->imageSensor->Project(beamTarget->pose.Pos());
          auto visual = this->dataPtr->imageSensor->VisualAt(pixel);
          if (visual)
          {
            if (visual->HasUserData("gazebo-entity"))
            {
              auto user_data = visual->UserData("gazebo-entity");
              beamTarget->entity = std::get<uint64_t>(user_data);
            }
            else
            {
              gzdbg << "No entity associated to [" << visual->Name() << "]"
                    << " visual. Assuming it is static w.r.t. the world."
                    << std::endl;
            }
          }
        }
      }

      TrackingModeInfo bottomModeInfo;
      DVLVelocityTracking bottomModeMessage;
      if (this->dataPtr->bottomModeSwitch)
      {
        bottomModeMessage =
            this->dataPtr->TrackBottom(_now, &bottomModeInfo);
      }

      TrackingModeInfo waterMassModeInfo;
      DVLVelocityTracking waterMassModeMessage;
      if (this->dataPtr->waterMassModeSwitch)
      {
        if (this->dataPtr->waterVelocity)
        {
          this->dataPtr->waterVelocity->StepTo(_now);

          waterMassModeMessage =
              this->dataPtr->TrackWaterMass(_now, &waterMassModeInfo);
        }
        else if (this->dataPtr->waterVelocityUpdated)
        {
          gzwarn << "No water velocity available, "
                 << "skipping water-mass tracking."
                 << std::endl;
        }
        this->dataPtr->waterVelocityUpdated = false;
      }

      double bottomModeScore, waterMassModeScore;
      if (std::isfinite(bottomModeInfo.totalVariance) ||
          std::isfinite(waterMassModeInfo.totalVariance))
      {
        bottomModeScore = -bottomModeInfo.totalVariance;
        waterMassModeScore = -waterMassModeInfo.totalVariance;
      }
      else
      {
        bottomModeScore =
          static_cast<double>(bottomModeInfo.numBeamsLocked);
        waterMassModeScore =
          static_cast<double>(waterMassModeInfo.numBeamsLocked);
      }

      if (this->dataPtr->bottomModeSwitch == TrackingModeSwitch::On ||
          (this->dataPtr->bottomModeSwitch == TrackingModeSwitch::Best &&
           bottomModeScore >= waterMassModeScore))
      {
        if (this->dataPtr->publishingEstimates)
        {
          auto * headerMessage = bottomModeMessage.mutable_header();
          auto frame = headerMessage->add_data();
          frame->set_key("frame_id");
          frame->add_value(this->FrameId());
          this->AddSequence(headerMessage, "doppler_velocity_log");
          this->dataPtr->pub.Publish(bottomModeMessage);
        }

        if (this->dataPtr->visualizeBottomModeBeams)
        {
          this->dataPtr->UpdateBeamMarkers(
              this, bottomModeMessage,
              &this->dataPtr->bottomModeBeamMarkers);
        }
      }

      if (this->dataPtr->waterMassModeSwitch == TrackingModeSwitch::On ||
          (this->dataPtr->waterMassModeSwitch == TrackingModeSwitch::Best &&
           bottomModeScore < waterMassModeScore))
      {
        if (this->dataPtr->publishingEstimates)
        {
          auto * headerMessage = waterMassModeMessage.mutable_header();
          auto frame = headerMessage->add_data();
          frame->set_key("frame_id");
          frame->add_value(this->FrameId());
          this->AddSequence(headerMessage, "doppler_velocity_log");
          this->dataPtr->pub.Publish(waterMassModeMessage);
        }

        if (this->dataPtr->visualizeWaterMassModeBeams)
        {
          this->dataPtr->UpdateBeamMarkers(
              this, waterMassModeMessage,
              &this->dataPtr->waterMassModeBeamMarkers);
        }
      }
    }

    //////////////////////////////////////////////////
    void DopplerVelocityLog::Implementation::UpdateBeamMarkers(
        DopplerVelocityLog *_sensor,
        const DVLVelocityTracking &_trackingMessage,
        gz::msgs::Marker_V *_beamMarkersMessage)
    {
      auto * headerMessage = _beamMarkersMessage->mutable_header();
      _sensor->AddSequence(headerMessage, "doppler_velocity_log_viz");

      for (int i = 0; i < _trackingMessage.beams_size(); ++i)
      {
        auto * beamLowerQuantileConeMarker =
            _beamMarkersMessage->mutable_marker(3 * i);
        auto * beamUpperQuantileConeMarker =
            _beamMarkersMessage->mutable_marker(3 * i + 1);
        auto * beamCapMarker =
            _beamMarkersMessage->mutable_marker(3 * i + 2);

        beamLowerQuantileConeMarker->set_parent(
            this->depthSensor->Parent()->Name());
        beamUpperQuantileConeMarker->set_parent(
            this->depthSensor->Parent()->Name());
        beamCapMarker->set_parent(
            this->depthSensor->Parent()->Name());

        const gz::math::Pose3d beamLocalTransform =
            this->depthSensor->LocalPose() * this->beams[i].Transform();
        gz::msgs::Set(
            beamLowerQuantileConeMarker->mutable_pose(), beamLocalTransform);
        gz::msgs::Set(
            beamUpperQuantileConeMarker->mutable_pose(), beamLocalTransform);
        gz::msgs::Set(beamCapMarker->mutable_pose(), beamLocalTransform);

        const auto & beamMessage = _trackingMessage.beams(i);
        if (beamMessage.locked())
        {
          const double beamRangeStdDev =
              std::sqrt(beamMessage.range().variance());
          const double beamRangeLowerQuantile =
              beamMessage.range().mean() - 2. * beamRangeStdDev;
          const double beamRangeUpperQuantile =
              beamMessage.range().mean() + 2. * beamRangeStdDev;

          gz::msgs::Set(beamLowerQuantileConeMarker->mutable_scale(),
                        beamRangeLowerQuantile * gz::math::Vector3d::One);
          gz::msgs::Set(beamUpperQuantileConeMarker->mutable_scale(),
                        beamRangeUpperQuantile * gz::math::Vector3d::One);
          gz::msgs::Set(beamCapMarker->mutable_scale(),
                        beamRangeUpperQuantile * gz::math::Vector3d::One);

          const gz::math::Vector3d beamAxis =
              this->referenceFrameRotation * this->beamsFrameTransform.Rot() *
              this->beams[i].Axis();

          const double beamSpeed =
              gz::msgs::Convert(beamMessage.velocity().mean()).Dot(beamAxis);
          gz::math::Color beamLowerQuantileMarkerColor{0.f, 0.f, 0.f, 0.85f};
          // Linearly map beam speed in the [-1 m/s, 1 m/s] to full-scale hue.
          beamLowerQuantileMarkerColor.SetFromHSV(
              180.f + static_cast<float>(beamSpeed) * 360.f, 1.f, 0.75f);
          auto * beamLowerQuantileConeMaterial =
              beamLowerQuantileConeMarker->mutable_material();
          gz::msgs::Set(beamLowerQuantileConeMaterial->mutable_ambient(),
                        beamLowerQuantileMarkerColor);
          gz::msgs::Set(beamLowerQuantileConeMaterial->mutable_diffuse(),
                        beamLowerQuantileMarkerColor);
          gz::msgs::Set(beamLowerQuantileConeMaterial->mutable_emissive(),
                        beamLowerQuantileMarkerColor);
          gz::math::Color beamUpperQuantileMarkerColor =
              beamLowerQuantileMarkerColor;
          beamUpperQuantileMarkerColor.A(0.25f);
          auto * beamUpperQuantileConeMaterial =
              beamUpperQuantileConeMarker->mutable_material();
          gz::msgs::Set(beamUpperQuantileConeMaterial->mutable_ambient(),
                        beamUpperQuantileMarkerColor);
          gz::msgs::Set(beamUpperQuantileConeMaterial->mutable_diffuse(),
                        beamUpperQuantileMarkerColor);
          gz::msgs::Set(beamUpperQuantileConeMaterial->mutable_emissive(),
                        beamUpperQuantileMarkerColor);
          *beamCapMarker->mutable_material() =
              beamUpperQuantileConeMarker->material();
          beamLowerQuantileConeMarker->set_action(
              gz::msgs::Marker::ADD_MODIFY);
          beamUpperQuantileConeMarker->set_action(
              gz::msgs::Marker::ADD_MODIFY);
          beamCapMarker->set_action(gz::msgs::Marker::ADD_MODIFY);
        }
        else
        {
          beamLowerQuantileConeMarker->set_action(
              gz::msgs::Marker::DELETE_MARKER);
          beamUpperQuantileConeMarker->set_action(
              gz::msgs::Marker::DELETE_MARKER);
          beamCapMarker->set_action(gz::msgs::Marker::DELETE_MARKER);
        }
      }

      bool result;
      gz::msgs::Boolean reply;
      constexpr unsigned int timeout_ms = 1000u;
      bool outcome = this->node.Request(
          "/marker_array", *_beamMarkersMessage,
          timeout_ms, reply, result);
      if (!outcome || !result || !reply.data())
      {
        gzwarn << "Failed to render beam markers for ["
               << _sensor->Name() << "] sensor."
               << std::endl;
      }
    }

    //////////////////////////////////////////////////
    bool DopplerVelocityLog::HasConnections() const
    {
      return this->dataPtr->pub && this->dataPtr->pub.HasConnections();
    }
  }  // namespace sensors
}  // namespace gz
