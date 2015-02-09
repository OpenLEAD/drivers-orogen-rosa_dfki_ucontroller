/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace rosa_dfki_ucontroller;

const double bar2meter = 10.1972;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (!_io_port.value().empty())
        m_driver.openURI(_io_port.get());

    setDriver(&m_driver);

    if (! TaskBase::configureHook())
        return false;
    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}

void Task::processIO()
{
    Packet packet = m_driver.getPacket();
    base::Time now = base::Time::now();
    base::Pressure p = packet.getPressure();
    _pressure_samples.write(
        base::samples::Pressure(now, p)
    );
    
    base::samples::RigidBodyState depth_sample;
    //conversion from pressure to meters
    depth_sample.position =  Eigen::Vector3d(0, 0, bar2meter*p.toBar());
    depth_sample.orientation = base::Orientation::Identity();
    depth_sample.time = now;
    depth_sample.sourceFrame = _world_frame.get();
    depth_sample.targetFrame = _sensor_frame.get();
    _depth_samples.write(depth_sample);
    
    
    _inductive_right.write(
        raw_io::Digital(now, packet.getCon21())
    );
    
    _inductive_left.write(
        raw_io::Digital(now, packet.getCon22())
    );
    
    _inductive_key_detached.write(
        raw_io::Digital(now, packet.getCon23())
    );
    
    _inductive_key_attached.write(
        raw_io::Digital(now, packet.getCon31())
    );
    
    _inclination_body.write(
        raw_io::Analog(now, packet.getAngleX())
    );
    
    _inclination_body_2.write(
        raw_io::Analog(now, packet.getAngleY())
    );
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
