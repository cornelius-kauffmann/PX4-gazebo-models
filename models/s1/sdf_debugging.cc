/*
 * Author: Cornelius Kauffmann
 * Date: 14.06.2024
 */

#include "LiftDrag.hh"

#include <algorithm>
#include <string>
#include <vector>
#include <cmath>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Wind.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::LiftDragPrivate
{
public:
    void Load(const EntityComponentManager &_ecm, const sdf::ElementPtr &_sdf);
    void Update(EntityComponentManager &_ecm);

    Model model{kNullEntity};
    double cla = 1.0;
    double cda = 0.01;
    double cma = 0.0;
    double alphaStall = GZ_PI_2;
    double claStall = 0.0;
    double cdaStall = 1.0;
    double cmaStall = 0.0;
    double cm_delta = 0.0;
    double rho = 1.2041; // Luftdichte kann durch Benutzereingabe überschrieben werden
    bool radialSymmetry = false;
    double area = 1.0;
    double alpha0 = 0.0;
    gz::math::Vector3d cp = math::Vector3d::Zero;
    math::Vector3d forward = math::Vector3d::UnitX;
    math::Vector3d upward = math::Vector3d::UnitZ;
    double controlJointRadToCL = 4.0;
    Entity linkEntity;
    Entity controlJointEntity;
    bool validConfig{false};
    sdf::ElementPtr sdfConfig;
    bool initialized{false};
};

void LiftDragPrivate::Load(const EntityComponentManager &_ecm, const sdf::ElementPtr &_sdf)
{
    this->cla = _sdf->Get<double>("cla", this->cla).first;
    this->cda = _sdf->Get<double>("cda", this->cda).first;
    this->cma = _sdf->Get<double>("cma", this->cma).first;
    this->alphaStall = _sdf->Get<double>("alpha_stall", this->alphaStall).first;
    this->claStall = _sdf->Get<double>("cla_stall", this->claStall).first;
    this->cdaStall = _sdf->Get<double>("cda_stall", this->cdaStall).first;
    this->cmaStall = _sdf->Get<double>("cma_stall", this->cmaStall).first;
    this->rho = _sdf->Get<double>("air_density", this->rho).first;
    this->radialSymmetry = _sdf->Get<bool>("radial_symmetry", this->radialSymmetry).first;
    this->area = _sdf->Get<double>("area", this->area).first;
    this->alpha0 = _sdf->Get<double>("a0", this->alpha0).first;
    this->cp = _sdf->Get<math::Vector3d>("cp", this->cp).first;
    this->cm_delta = _sdf->Get<double>("cm_delta", this->cm_delta).first;

    this->forward = _sdf->Get<math::Vector3d>("forward", this->forward).first;
    this->forward.Normalize();

    this->upward = _sdf->Get<math::Vector3d>("upward", this->upward).first;
    this->upward.Normalize();

    this->controlJointRadToCL = _sdf->Get<double>("control_joint_rad_to_cl", this->controlJointRadToCL).first;

    if (_sdf->HasElement("link_name"))
    {
        sdf::ElementPtr elem = _sdf->GetElement("link_name");
        auto linkName = elem->Get<std::string>();
        auto entities = entitiesFromScopedName(linkName, _ecm, this->model.Entity());

        if (entities.empty())
        {
            gzerr << "Link with name[" << linkName << "] not found. " << "The LiftDrag will not generate forces\n";
            this->validConfig = false;
            return;
        }
        else if (entities.size() > 1)
        {
            gzwarn << "Multiple link entities with name[" << linkName << "] found. " << "Using the first one.\n";
        }

        this->linkEntity = *entities.begin();
        if (!_ecm.EntityHasComponentType(this->linkEntity, components::Link::typeId))
        {
            this->linkEntity = kNullEntity;
            gzerr << "Entity with name[" << linkName << "] is not a link\n";
            this->validConfig = false;
            return;
        }
    }
    else
    {
        gzerr << "The LiftDrag system requires the 'link_name' parameter\n";
        this->validConfig = false;
        return;
    }

    if (_sdf->HasElement("control_joint_name"))
    {
        auto controlJointName = _sdf->Get<std::string>("control_joint_name");
        auto entities = entitiesFromScopedName(controlJointName, _ecm, this->model.Entity());

        if (entities.empty())
        {
            gzerr << "Joint with name[" << controlJointName << "] not found. " << "The LiftDrag will not generate forces\n";
            this->validConfig = false;
            return;
        }
        else if (entities.size() > 1)
        {
            gzwarn << "Multiple joint entities with name[" << controlJointName << "] found. Using the first one.\n";
        }

        this->controlJointEntity = *entities.begin();
        if (!_ecm.EntityHasComponentType(this->controlJointEntity, components::Joint::typeId))
        {
            this->controlJointEntity = kNullEntity;
            gzerr << "Entity with name[" << controlJointName << "] is not a joint\n";
            this->validConfig = false;
            return;
        }
    }

    // If we reached here, we have a valid configuration
    this->validConfig = true;
}

// Der Rest des Codes bleibt unverändert, da hier nur der Ladeprozess geändert wurde.
//////////////////////////////////////////////////
void LiftDragPrivate::Update(EntityComponentManager &_ecm)
{
  GZ_PROFILE("LiftDragPrivate::Update");
  // get linear velocity at cp in world frame
  const auto worldLinVel =
      _ecm.Component<components::WorldLinearVelocity>(this->linkEntity);
  const auto worldAngVel =
      _ecm.Component<components::WorldAngularVelocity>(this->linkEntity);
  const auto worldPose =
      _ecm.Component<components::WorldPose>(this->linkEntity);

  // get wind as a component from the _ecm
  components::WorldLinearVelocity *windLinearVel = nullptr;
  if(_ecm.EntityByComponents(components::Wind()) != kNullEntity){
    Entity windEntity = _ecm.EntityByComponents(components::Wind());
    windLinearVel =
        _ecm.Component<components::WorldLinearVelocity>(windEntity);
  }
  components::JointPosition *controlJointPosition = nullptr;
  if (this->controlJointEntity != kNullEntity)
  {
    controlJointPosition =
        _ecm.Component<components::JointPosition>(this->controlJointEntity);
  }

  if (!worldLinVel || !worldAngVel || !worldPose)
  {
    return;
  }

  const auto &pose = worldPose->Data();
  const auto cpWorld = pose.Rot().RotateVector(this->cp);
  auto vel = worldLinVel->Data() + worldAngVel->Data().Cross(
  cpWorld);
  if (windLinearVel != nullptr){
    vel -= windLinearVel->Data();
  }

  if (vel.Length() <= 0.01)
  {
    return;
  }

  const auto velI = vel.Normalized();
  const auto forwardI = pose.Rot().RotateVector(this->forward);
  if (forwardI.Dot(vel) <= 0.0){
    return;
  }

  math::Vector3d upwardI;
  if (this->radialSymmetry)
  {
    math::Vector3d tmp = forwardI.Cross(velI);
    upwardI = forwardI.Cross(tmp).Normalize();
  }
  else
  {
    upwardI = pose.Rot().RotateVector(this->upward);
  }

  const auto spanwiseI = forwardI.Cross(upwardI).Normalize();
  double sinSweepAngle = math::clamp(spanwiseI.Dot(velI), -1.0, 1.0);
  double cos2SweepAngle = 1.0 - sinSweepAngle * sinSweepAngle;
  double sweep = std::asin(sinSweepAngle);

  while (std::fabs(sweep) > 0.5 * GZ_PI)
  {
    sweep = sweep > 0 ? sweep - GZ_PI : sweep + GZ_PI;
  }

  const auto velInLDPlane = vel - vel.Dot(spanwiseI)*spanwiseI;
  const auto dragDirection = -velInLDPlane.Normalized();
  const auto liftI = spanwiseI.Cross(velInLDPlane).Normalized();
  const double cosAlpha = math::clamp(liftI.Dot(upwardI), -1.0, 1.0);
  double alpha = this->alpha0 - std::acos(cosAlpha);
  if (liftI.Dot(forwardI) >= 0.0)
    alpha = this->alpha0 + std::acos(cosAlpha);

  while (fabs(alpha) > 0.5 * GZ_PI)
  {
    alpha = alpha > 0 ? alpha - GZ_PI : alpha + GZ_PI;
  }

  const double speedInLDPlane = velInLDPlane.Length();
  const double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;

  double cl;
  if (alpha > this->alphaStall)
  {
    cl = (this->cla * this->alphaStall + this->claStall * (alpha - this->alphaStall)) * cos2SweepAngle;
    cl = std::max(0.0, cl);
  }
  else if (alpha < -this->alphaStall)
  {
    cl = (-this->cla * this->alphaStall + this->claStall * (alpha + this->alphaStall)) * cos2SweepAngle;
    cl = std::min(0.0, cl);
  }
  else
  {
    cl = this->cla * alpha * cos2SweepAngle;
  }

  if (controlJointPosition && !controlJointPosition->Data().empty())
  {
    cl += this->controlJointRadToCL * controlJointPosition->Data()[0];
  }

  math::Vector3d lift = cl * q * this->area * liftI;

  double cd;
  if (alpha > this->alphaStall)
  {
    cd = (this->cda * this->alphaStall + this->cdaStall * (alpha - this->alphaStall)) * cos2SweepAngle;
  }
  else if (alpha < -this->alphaStall)
  {
    cd = (-this->cda * this->alphaStall + this->cdaStall * (alpha + this->alphaStall)) * cos2SweepAngle;
  }
  else
  {
    cd = (this->cda * alpha) * cos2SweepAngle;
  }
  cd = std::fabs(cd);

  math::Vector3d drag = cd * q * this->area * dragDirection;

  double cm;
  if (alpha > this->alphaStall)
  {
    cm = (this->cma * this->alphaStall + this->cmaStall * (alpha - this->alphaStall)) * cos2SweepAngle;
    cm = std::max(0.0, cm);
  }
  else if (alpha < -this->alphaStall)
  {
    cm = (-this->cma * this->alphaStall + this->cmaStall * (alpha + this->alphaStall)) * cos2SweepAngle;
    cm = std::min(0.0, cm);
  }
  else
  {
    cm = this->cma * alpha * cos2SweepAngle;
  }

  if (controlJointPosition && !controlJointPosition->Data().empty())
  {
    cm += this->cm_delta * controlJointPosition->Data()[0];
  }

  math::Vector3d moment = cm * q * this->area * spanwiseI;
  math::Vector3d force = lift + drag;
  math::Vector3d torque = moment;
  force.Correct();
  torque.Correct();
  const auto totalTorque = torque + cpWorld.Cross(force);
  Link link(this->linkEntity);
  link.AddWorldWrench(_ecm, force, totalTorque);
}

void LiftDrag::Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm, EventManager &)
{
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "The LiftDrag system should be attached to a model entity. Failed to initialize." << std::endl;
    return;
  }
  this->dataPtr->sdfConfig = _sdf->Clone();
}

void LiftDrag::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  GZ_PROFILE("LiftDrag::PreUpdate");
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time [" << std::chrono::duration<double>(_info.dt).count() << "s]. System may not work properly." << std::endl;
  }

  if (!this->dataPtr->initialized)
  {
    this->dataPtr->Load(_ecm, this->dataPtr->sdfConfig);
    this->dataPtr->initialized = true;

    if (this->dataPtr->validConfig)
    {
      Link link(this->dataPtr->linkEntity);
      link.EnableVelocityChecks(_ecm, true);

      if ((this->dataPtr->controlJointEntity != kNullEntity) && !_ecm.Component<components::JointPosition>(this->dataPtr->controlJointEntity))
      {
        _ecm.CreateComponent(this->dataPtr->controlJointEntity, components::JointPosition());
      }
    }
  }

  if (_info.paused)
    return;

  if (this->dataPtr->initialized && this->dataPtr->validConfig)
  {
    this->dataPtr->Update(_ecm);
  }
}

GZ_ADD_PLUGIN(LiftDrag, System, LiftDrag::ISystemConfigure, LiftDrag::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(LiftDrag, "gz::sim::systems::LiftDrag")

