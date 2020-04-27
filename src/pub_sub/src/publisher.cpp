#include "ros/ros.h"
#include "ros/master.h"
#define RUN_PERIOD_DEFAULT 0.1

#define NAME_OF_THIS_NODE "publisher"

#include "std_msgs/String.h"//include l'header del messaggio String.msg


class Publisher //classe madre del nodo

{
  private: 
    ros::NodeHandle Handle;    
    ros::Publisher Publisher;

    void PeriodicTask(int count);/*metodo per rendere il mio nodo periodico 
    *(vogliamo che pubblici una volta al secondo)*/
    
     
    
    
  public:
    double RunPeriod;//attributo che rappresenta il periodo di ripetizione
    
    void Prepare(void);/*equivale al setup di arduino, è un metodo
     *che viene eseguito una sola volta*/
    
    void RunPeriodically(float Period);/*equivale al loop di arduiono,
    * viene ripetuto finchè tutto funziona regolarmente*/
    
    
    void Shutdown(void);//metodo richiamato per la chiusura del nodo
    
};

//-----------------------------------------------------------------
//      DEFINIZIONE DEI METODI
//-----------------------------------------------------------------

void Publisher::Prepare(void)
{
  RunPeriod = RUN_PERIOD_DEFAULT;/*assegna il periodo definito nel define
  * alla variabile RunPeriod*/
 
  Publisher = Handle.advertise<std_msgs::String>("chat", 1000);/*comunica al master che
  *il suo scopo è quello di pubblicare sul topic definito "chat"
  *1000 rappresenta la dimensione della coda di messaggi da pubblicare (massima?)*/
  
   ROS_INFO("Il ROSpo Nodo %s è pronto per fare faville.", ros::this_node::getName().c_str());
}

void Publisher::RunPeriodically(float Period)
{
  ros::Rate LoopRate(1.0/Period);/*definisce la lunghezza del tempo
  * di esecuzione del metodo periodico*/
  
  ROS_INFO("Il ROSpo Nodo %s zira periodicamente (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);
  int count=0;
  while (ros::ok() && ros::master::check())
  {
    PeriodicTask(count);//!!!funzione chiamata periodicamente!!!
    count++;
    
    ros::spinOnce();//necessario per far ripartire la funzione,richiama tutte le call aperte
    LoopRate.sleep();
  }
}

void Publisher::PeriodicTask(int count)
{
    std_msgs::String msg;//crea l'instanza del messagio di tipo std_msgs::String
    std::stringstream ss;//istanza di una classe predefefiniti di c++ per costruire stringhe
    ss<<"Zere e Franze hanno raccolto "<< count <<" aranze";//concateno le stringhe
    msg.data = ss.str();//salva nel campo data la stringa concatenata

    Publisher.publish(msg);//pubblica il messaggio sul topic
    ROS_INFO("Ho scritto al gnaro: '%s'",msg.data.c_str());
}

void Publisher::Shutdown(void)
{
  ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
  std_msgs::String msg;
  msg.data=std::string("Stacca Stacca, ci stanno tracciando!");
  Publisher.publish(msg);
  ROS_INFO("Ho scritto al gnaro: '%s'",msg.data.c_str());
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);/*chiama un metodo statico di ROS che inizializza
  * il nodo e lo iscrive al master.
  *!!!PRIMA ISTRUZIONE DA SCRIVERE NEL MAIL DI QUALUNQUE NODO!!!*/
  
  Publisher publisher;//creo l'istanza della mia classe Publisher
   
  publisher.Prepare();
  
  publisher.RunPeriodically(publisher.RunPeriod);
  
  publisher.Shutdown();
  
  return (0);
}
