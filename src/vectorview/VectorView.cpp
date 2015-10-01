#include "vectorview/VectorView.h"
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(VectorView)

VectorView::VectorView()
{
}

VectorView::~VectorView()
{
    rpcThread.stop();
//    if (rpcEnabled)
//        rpcPort.close();

}


void VectorView::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
{
  // get visual and names
  this->visual = _parent;
  // Filters setup
  filter = new Dsp::ForceFilter();
  // Initialize yarp network
  if ( !yarpNet.checkNetwork() ) {
      printf ( "ERROR Yarp Network was not found active in VectorView plugin\n" );
      return;
  }
  vectorViewInitialized = false;
}

void VectorView::Init()
{
    if (!vectorViewInitialized){
          this->FindName();

          // prepare rpc port
          rpcThread.setRobotName  ( "iCub" );
          rpcThread.setDefaultLink("");
          std::string linkName = this->visual->GetName();
          if (linkName.find("::") != std::string::npos)
            linkName.erase(0, linkName.find("::") + 2);
          if (linkName.find("::") != std::string::npos)
            linkName.erase(linkName.find("::"),linkName.length()-1);
          std::cout<<"linkName------------------------------"<<std::endl;
          std::cout<<linkName<<std::endl;
          if (linkName.find("iCub")== std::string::npos){

        //      bool vectorViewRpcPort;
        //      vectorViewRpcPort = rpcPort.open ( std::string ( "/vectorView/"+linkName+"/forceMeasurement/rpc:i" ).c_str() );
        //      if ( !vectorViewRpcPort ) {
        //          printf ( "ERROR opening RPC port /vectorView/" );
        //      }
        //      else
        //          printf ( "Opening RPC port /vectorView/" );


              rpcThread.setDefaultLink(linkName);
              // Starting RPC thread to read desired wrench to be applied
              if ( !rpcThread.start() ) {
                  std::cout << "ERROR: vectorView rpcThread did not start correctly" <<std::endl;
              }

          }



          // visual components initialization
          //this->forceVector.reset(this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP));
          this->forceVector = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
          this->forceVector->setMaterial("Gazebo/Blue");
          this->forceVector->setVisibilityFlags(GZ_VISIBILITY_GUI);
          for(int k = 0; k < 6; ++k) // -> needs three lines, so 6 points
            this->forceVector->AddPoint(math::Vector3::Zero);

          this->forceVector->Update();

          // define this plugin as a listener of the sensor topic defined in topic_path
          this->subs.reset();
          node.reset(new gazebo::transport::Node());
          node->Init();
          this->subs = node->Subscribe(topicName, &VectorView::VectorViewUpdate, this);
          std::cout << std::endl
                    << "-- VectorView plugin initialized" << std::endl
                    << "   topic path : " << topicName     << std::endl
                    << "   collsion   : " << collisionName << std::endl
                    << std::endl;

          vectorViewInitialized = true;
    }
}

void VectorView::UpdateVector(math::Vector3 force)
{
    rpcThread.updateForce(force);


  math::Vector3 begin          = math::Vector3::Zero;
//  math::Vector3 end            = begin + FORCE_SCALE*(visual->GetWorldPose().rot.RotateVectorReverse( force ));// points of forceVector are expressed in body frame, force is expressed in world frame
  //visual->GetRotation().RotateVectorReverse(force)));
//  math::Vector3 end            = begin + FORCE_SCALE*(visual->GetWorldPose().rot.RotateVectorReverse( force ));
//  math::Vector3 end            = begin + FORCE_SCALE*(visual->GetWorldPose().rot.RotateVector( force ));
//  math::Vector3 end            = begin + FORCE_SCALE*(visual->GetRotation().RotateVectorReverse(force));
//  math::Vector3 end            = begin + FORCE_SCALE*(visual->GetRotation().RotateVector(force));
  math::Vector3 end            = begin + FORCE_SCALE*(visual->GetWorldPose().rot.RotateVectorReverse(force));
  // draw a cute arrow, just as a vector should be represented
  this->forceVector->SetPoint(0, begin);
  this->forceVector->SetPoint(1, end);
  this->forceVector->SetPoint(2, end);
  this->forceVector->SetPoint(3, end - ARROW_LENGTH*math::Matrix3(1, 0, 0, 0, 0.9848, -0.1736, 0,  0.1736, 0.9848)*(end - begin).Normalize());
  this->forceVector->SetPoint(4, end);
  this->forceVector->SetPoint(5, end - ARROW_LENGTH*math::Matrix3(1, 0, 0, 0, 0.9848,  0.1736, 0, -0.1736, 0.9848)*(end - begin).Normalize());

}

// find out contact, output history file and collision names
void VectorView::FindName()
{
  std::vector<std::string> names;
  std::string name = this->visual->GetName();

  while (name.find("::") != std::string::npos)
  {
    names.push_back(std::string(name, 0, name.find("::")));
    name.erase(0, name.find("::") + 2);
  }
  topicName     = "~";
  collisionName = "";

  int i;
  for(i = 0; i < names.size(); ++i )
  {
    topicName     += "/" + names.at(i);
    collisionName += "::" + names.at(i);
  }
  topicName     += "/" + names.at(i-1) + "_contact";
  collisionName += "::" + names.at(i-1) + "_collision";
  collisionName.erase(0,2);
}

// called when a new message is received
void VectorView::VectorViewUpdate(ConstContactsPtr &message)
{
  math::Vector3 force = math::Vector3::Zero;

  // sum of all forces
  unsigned int n, m;
  for(n = 0; n < message->contact_size(); ++n)
  {
    for (m = 0; m < message->contact(n).wrench_size(); ++m)
    {

      if (message->contact(n).wrench(m).body_1_name().find(collisionName) != std::string::npos)
      {
        force = force - msgs::Convert(message->contact(n).wrench(m).body_1_wrench().force());
      }
      else
      {
        force = force - msgs::Convert(message->contact(n).wrench(m).body_2_wrench().force());
      }
    }
  }

  if(message->contact_size())
  {
    force = force/n; // we choose the mean of all contacts in the message
//    std::cout<<this->visual->GetName()<<" force="<<force<<std::endl;
  }

  filter->Filter(&force);

  // update visual DynamicLines
  if(force.GetLength() < NOISE_THRESHOLD)
    force = math::Vector3::Zero;

  this->UpdateVector(force);
}


// ############ RPCServerThread class ###############

void RPCServerThread::setRobotName ( std::string robotName )
{
    this->m_robotName = robotName;
}



void RPCServerThread::setDefaultLink(const std::string &defaultLink)
{
    this->m_defaultLink = defaultLink;
}

bool RPCServerThread::threadInit()
{

    printf ( "Starting RPCServerThread\n" );
    printf ( "Opening rpc port\n" );

    if ( !rpcPort.open ( std::string ( "/vectorView/"+m_defaultLink+"/forceMeasurement/rpc:i" ).c_str() ) ) {
        printf ( "ERROR opening RPC port /vectorView \n" );
        return false;
    }

    forceBuffer = math::Vector3::Zero;
    return true;
}

void RPCServerThread::updateForce(math::Vector3 force)
{
    forceBuffer = force;
}

void RPCServerThread::run()
{
    while ( !isStopping() ) {
        yarp::os::Bottle command;
        rpcPort.read ( command,true );
        std::string name = command.get ( 0 ).asString();

        if (m_defaultLink.find(name) != std::string::npos)
        {

      //      rpcReply.addString ( "[ACK] sending contact force at "+name );
//            rpcReply.addVocab ( yarp::os::Vocab::encode ( "many" ) );
            rpcReply.addDouble ( forceBuffer[0] );
            rpcReply.addDouble ( forceBuffer[1] );
            rpcReply.addDouble ( forceBuffer[2] );
            rpcPort.reply ( rpcReply );
            m_lock.lock();
            m_cmd = command;
            m_lock.unlock();

        }
        else
        {
            rpcReply.addString ( "[ERROR] no force sensored at "+name );
            rpcPort.reply ( rpcReply );
        }

        rpcReply.clear();
        command.clear();





//        yarp::os::Bottle command;
//        rpcPort.read ( command,true );
//        if ( command.get ( 0 ).asString() == "help" ) {
//            this->m_reply.addVocab ( yarp::os::Vocab::encode ( "many" ) );
//            this->m_reply.addString ( "Insert a command with the following format:" );
//            this->m_reply.addString ( "[link] [force] [torque] [duration]" );
//            this->m_reply.addString ( "e.g. chest 10 0 0 0 0 0 1");
//            this->m_reply.addString ( "[link]:     (string) Link ID of the robot as specified in robot's SDF" );
//            this->m_reply.addString ( "[force]:    (double x, y, z) Force components in N w.r.t. world reference frame" );
//            this->m_reply.addString ( "[torque]:   (double x, y, z) Torque components in N.m w.r.t world reference frame" );
//            this->m_reply.addString ( "[duration]: (double) Duration of the applied force in seconds" );
//            this->m_reply.addString ( "Note: The reference frame is the base/root robot frame with x pointing backwards and z upwards.");
//            this->m_rpcPort.reply ( this->m_reply );
//        } else {
//            if ( command.get ( 0 ).isString() \
//                    && ( command.get ( 1 ).isDouble() || command.get ( 1 ).isInt() )  && ( command.get ( 2 ).isDouble() || command.get ( 2 ).isInt() ) && ( command.get ( 3 ).isDouble() || command.get ( 3 ).isInt() ) \
//                    && ( command.get ( 4 ).isDouble() || command.get ( 4 ).isInt() )  && ( command.get ( 5 ).isDouble() || command.get ( 5 ).isInt() ) && ( command.get ( 6 ).isDouble() || command.get ( 6 ).isInt() )  \
//                    && ( command.get ( 7 ).isDouble() || command.get ( 7 ).isInt() ) ) {
//                this->m_reply.addString ( "[ACK] Correct command format" );
//                this->m_rpcPort.reply ( m_reply );
//                m_lock.lock();
//                m_cmd = command;
//                m_lock.unlock();
//            } else {
//                this->m_reply.clear();
//                this->m_reply.addString ( "ERROR: Incorrect command format" );
//                this->m_rpcPort.reply ( this->m_reply );
//            }
//        }
//        m_reply.clear();
//        command.clear();
    }
}
void RPCServerThread::threadRelease()
{
    yarp::os::Thread::threadRelease();
    rpcPort.close();
    printf ( "Goodbye from vectorView RPC thread\n" );
}
yarp::os::Bottle RPCServerThread::getCmd()
{
    return m_cmd;
}

void RPCServerThread::onStop()
{
    rpcPort.interrupt();
}
