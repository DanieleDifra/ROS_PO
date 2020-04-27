
#include "ros/ros.h"

#define RUN_PERIOD_DEFAULT 1
 
#define NAME_OF_THIS_NODE "publisher"

#include "std_msgs/String.h" //include l'header del messaggio "std_msg::String"
#include "std_msgs/Float32.h"

class Publisher 
{
  private:
    ros::NodeHandle Handle; //handler per abilitare la chiamata ai metodi di ROS
    
    ros::Publisher Publisher; //ATTRIBUTO oggetto publisher
	  float a,b;
    void PeriodicTask(int count);// METODO 
    float operazione(int t);
    
    
  public:
    double RunPeriod; //ATTRIBUTO 
	
    void Prepare(float x, float y); //METODO 
    
    void RunPeriodically(float Period); //METODO 
    
    void Shutdown(void);//METODO 
	
};

//-----------------------------------------------------------------
//-----------------------------------------------------------------

void Publisher::Prepare(float x, float y)
{

  RunPeriod = RUN_PERIOD_DEFAULT;//salva in RunPeriod il valore della define
 
  Publisher = Handle.advertise<std_msgs::Float32>("risultato", 1000);//indica al master che questo nodo andrà a pubblicare sul topic: "chat", 
  //1000 è la lunghezza della coda di messaggi da pubblicare
   
  ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str()); //ROS_INFO va a scrivere sul terminale il messaggio tra (), è come una printf()

  this->a = x*y;
  this->b = x/y;
}

void Publisher::RunPeriodically(float Period)
{
  ros::Rate LoopRate(1.0/Period); //imposta ogni quanto tempo riparte il loop seguente
  
  ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);
  
  int count=0; //semplice counter per contare i messaggi pubblicati 
  
  while (ros::ok())//il ciclo gira finchè roscore è attivo
  {
    PeriodicTask(count);
	count++;
    ros::spinOnce();//alla fine di ogni loop serve per farlo ripartire
    LoopRate.sleep();//addormenta il ciclo per il periodo indicato precedentemente
  }
}

float Publisher::operazione(int t)
{
 return (this->a * t * t + this->b * t);
}

void Publisher::PeriodicTask(int count)
{
  std_msgs::Float32 msg;//crea un'instanza di un  messaggio (in questo caso di tipo std_msg::String
  
  msg.data = operazione(count);//salva nel campo "data" di msg la stringa creata (chiama il metodo str() di ss per creare una stringa)
  
  Publisher.publish(msg);//pubblica il messaggio nel topic 
  ROS_INFO("I've published: '%f'", msg.data);
}


void Publisher::Shutdown(void)
{
  ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
  
}

//-----------------------------------------------------------------
//-----------------------------------------------------------------


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);//chiama un metodo STATICO di ROS che inizializza il nodo e lo iscrive al master. Init è sempre la prima istruzione chiamare
  
  Publisher publisher; //crea un'istanza della classe "Publisher"
   
  publisher.Prepare(1,1);//chiama il metodo "Prepare" della classe "Publisher"

  publisher.RunPeriodically(publisher.RunPeriod);//chiama il metodo "RunPeriodically" della classe "Publisher" 
  
  publisher.Shutdown(); // chiama il metodo "Shutdown" della classe "Publisher"
  
  return (0);
}

