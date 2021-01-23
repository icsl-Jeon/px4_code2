//
// Created by jbs on 21. 1. 22..
//


#include <px4_code2/server.h>


int main(int argc,char** argv){
    ros::init(argc,argv,"server_node");
    px4_code2::Server server;
    server.run();
    return 0 ;

}
