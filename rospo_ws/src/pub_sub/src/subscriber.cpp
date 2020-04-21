#include "ros/ros.h"

#define RUN_PERIOD_DEFAULT 0.1

#define NAME_OF_THIS_NODE "subscriber"

#include "std_msgs/String.h" //vedi publisher.cpp


class Subscriber

{
  private: 
    ros::NodeHandle Handle;
    
    ros::Subscriber Subscriber;
    
    void MessageCallback(const std_msgs::String::ConstPtr& msg);
    
    bool shouldContinue;
    
  public:   
    void Prepare(void);
        
    void RunContinuously(void);
   
    void Shutdown(void);
    
};

//-----------------------------------------------------------------
//-----------------------------------------------------------------

void Subscriber::Prepare(void)
{
   Subscriber = Handle.subscribe("chat", 1000, &Subscriber::MessageCallback, this);
   /*Notifica al master che ha intenzione di ascoltare tutto ciò che verrà pubblicato
   * tramite il topic "chat", successivamente indica la lunghezza della coda
   * e il metodo da chiamare  nel caso che in quel topic venga pubblicato un messaggio
   * "this" indica che il metodo si trovs nella classe Subscriber */
   this->shouldContinue=true;
   ROS_INFO("Il ROSpo Nodo %s è pronto a sfrezzare.", ros::this_node::getName().c_str());
}


void Subscriber::RunContinuously(void)
{
  ROS_INFO("Il ROSpo Nodo %s viazza continuativamente.", ros::this_node::getName().c_str());
  while(this->shouldContinue){
    ros::spinOnce();
  }
  
  
  
  /* From ROS documentation:
   * "ros::spin() will not return until the node has been 
   * shutdown, either through a call to ros::shutdown() or a 
   * Ctrl-C." */
}


void Subscriber::MessageCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Ho rizevuto: [%s]", msg->data.c_str());
    if(!msg->data.compare(std::string("Stacca Stacca, ci stanno tracciando!")))
    {
        this->shouldContinue = false;

    }
  
}

void Subscriber::Shutdown(void)
{
  ROS_INFO("Il ROSpo Nodo %s è stanco va a dormire.", ros::this_node::getName().c_str());
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
    
  Subscriber subscriber;
   
  subscriber.Prepare();
  
  subscriber.RunContinuously();
 
  subscriber.Shutdown();
  
  return (0);
}