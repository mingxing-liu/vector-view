#ifndef VECTORVIEW_H
#define VECTORVIEW_H

#define FORCE_SCALE 8E-2 // scale between the forces intensities and the vectors length (unity N^-1)
#define NOISE_THRESHOLD 1E-3
#define ARROW_LENGTH .05

// Gazebo includes
#include <gazebo.hh>
#include <rendering/Visual.hh>
#include <rendering/rendering.hh>
#include <msgs/msgs.hh>
#include <common/common.hh>
// general includes
#include <iostream>
#include <string>
#include <vector>
// #include <boost/shared_ptr.hpp>
// filter includes
#include "DspFilters/Dsp.h"
#include "DspFilters/Filter.h"
#include "DspFilters/ForceFilter.h"

#include <yarp/os/Thread.h>
#include <yarp/os/Network.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Vocab.h>

// A YARP Thread class that will be used to read the rpc port to send contact force
class RPCServerThread: public yarp::os::Thread
{
public:
    virtual bool        threadInit();
    virtual void        run();
    virtual void        threadRelease();
    yarp::os::Bottle    getCmd();
    void                setRobotName(std::string robotName);
    void                setDefaultLink(const std::string& defaultLink);
    void                updateForce(math::Vector3 force);
    virtual void        onStop();
private:
    yarp::os::RpcServer rpcPort;
    yarp::os::Bottle    m_cmd;
    yarp::os::Bottle    rpcReply;
    /// \brief Mutex to lock reading and writing of _cmd
    boost::mutex        m_lock;
    std::string         m_robotName;
    std::string         m_defaultLink;
    math::Vector3       forceBuffer;
};


namespace gazebo
{
  //typedef boost::shared_ptr<rendering::DynamicLines> LinePtr;
  typedef rendering::DynamicLines* LinePtr;

  class VectorView : public VisualPlugin
  {
  public:
    // CONSTRUCTOR AND DESTRUCTOR
    VectorView();
    ~VectorView();
    // FUNCTIONS
    void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf); // executed once the plugin is loaded
    void VectorViewUpdate(ConstContactsPtr &_msg);                 // executed everytime a message is published by the sensor: updates the vector visual
    void Init();

  private:
    // FUNCTIONS
    void FindName();     // find the topic, output history and collision names based on the visual
    void UpdateVector(math::Vector3 force);  // update visual from the vector
    // VARIABLES
    LinePtr forceVector; // the animated line representing the force
    rendering::VisualPtr visual;
    transport::SubscriberPtr subs;
    transport::NodePtr node;

    std::string collisionName;
    std::string topicName;

    Dsp::ForceFilter* filter;

    yarp::os::Network   yarpNet;
    RPCServerThread     rpcThread;
//    yarp::os::RpcServer rpcPort;
//    yarp::os::Bottle    command;
//    yarp::os::Bottle    rpcReply;
    /// \brief Mutex to lock access

    bool vectorViewInitialized;
  };
}

#endif
