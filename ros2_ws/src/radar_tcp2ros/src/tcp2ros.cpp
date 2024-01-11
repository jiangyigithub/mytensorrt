
/* 
# =============================================================================
#   C O P Y R I G H T                                     
# _____________________________________________/\\\\\\\\\\\_____/\\\\\\\\\_____/\\\\\\\\\\\\____        
#   Copyright (c) 2021 by Robert Bosch GmbH.  _\/////\\\///____/\\\\\\\\\\\\\__\/\\\////////\\\__       
#   All rights reserved.                       _____\/\\\______/\\\/////////\\\_\/\\\______\//\\\_      
#                                               _____\/\\\_____\/\\\_______\/\\\_\/\\\_______\/\\\_     
#   This file is property of Robert Bosch GmbH.  _____\/\\\_____\/\\\\\\\\\\\\\\\_\/\\\_______\/\\\_    
#   Any unauthorized copy or use or distribution  _____\/\\\_____\/\\\/////////\\\_\/\\\_______\/\\\_   
#   is an offensive act against international law  _____\/\\\_____\/\\\_______\/\\\_\/\\\_______/\\\__  
#   and may be prosecuted under federal law.        __/\\\\\\\\\\\_\/\\\_______\/\\\_\/\\\\\\\\\\\\/___ 
#   Its content is company confidential.             _\///////////__\///________\///__\////////////_____
# _______________________________________________________________________________________________________
#   P R O J E C T   I N F O R M A T I O N
# -----------------------------------------------------------------------------
#   IAD - Infrastructure-based Autonomous Driving (CR/RIX)
# =============================================================================
*
* main.cpp
*
*  Created on: March 23, 2021
*      Author: zns5sgh
*     Contact: fixed-term.Simon.Zhang@cn.bosch.com
*
*/
 

#include "rclcpp/rclcpp.hpp"

#include <stdio.h>
#include <sys/types.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <readcan_msgs/msg/canfd.hpp>
#include <vector>
#include <bitset>
using namespace std;

int socket_init(const string can_ethernet_model_name, string ip){ 

	const char *p =ip.data();
	int socket_fd = socket(AF_INET, SOCK_STREAM,0);
    

    if(socket_fd == -1){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[tcp2ros]socket creation failed:");
        exit(EXIT_FAILURE);
    }
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;

    if (can_ethernet_model_name == "technica"){
        addr.sin_port = htons(1025);
        addr.sin_addr.s_addr = inet_addr(p);//"10.104.3.100"
	}
	else if(can_ethernet_model_name == "ZLG"){
        addr.sin_port = htons(8000);
        addr.sin_addr.s_addr = inet_addr(p);//"192.168.0.178"
	}
	else{
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[tcp2ros]socket creation failed: Unkown can_ethernet_model_name check if technica or ZLG");
        exit(EXIT_FAILURE);
	}
    

    int res = connect(socket_fd,(struct sockaddr*)&addr,sizeof(addr));

    if(res == -1){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[tcp2ros]bind link failed: Check ip or of model is correct");
        exit(EXIT_FAILURE);
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[tcp2ros]bind link succeed:");
return socket_fd;
}

bitset<8> Reverse_bit(bitset<8> origin_bit){
    int bit1 = origin_bit[0];
    int bit2 = origin_bit[1];
    int bit3 = origin_bit[2];
    int bit4 = origin_bit[3];
    int bit5 = origin_bit[4];
    int bit6 = origin_bit[5];
    int bit7 = origin_bit[6];
    int bit8 = origin_bit[7];
    origin_bit[7] = bit1;
    origin_bit[6] = bit2;
    origin_bit[5] = bit3;
    origin_bit[4] = bit4;
    origin_bit[3] = bit5;
    origin_bit[2] = bit6;
    origin_bit[1] = bit7;
    origin_bit[0] = bit8;
return origin_bit;
}

unsigned char Bitset_2_Char(const bitset<8> &bits){
    unsigned char ch;
    for (int j = 0; j < 8; ++j){
    if (bits.test( j))   // 第i + j位为1
        ch |= (1 << j);
    else
        ch &= ~(1 << j);
    }
    return ch;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto tcp2ros_node = std::make_shared<rclcpp::Node>("tcp2ros");
    
    // set parameter
    tcp2ros_node -> declare_parameter("can_ethernet_model_name");
    tcp2ros_node -> declare_parameter("ip");
    std::string can_ethernet_model_name = tcp2ros_node -> get_parameter("can_ethernet_model_name").as_string();
    std::string ip = tcp2ros_node -> get_parameter("ip").as_string();

    // create publisher
    auto tcp2ros_pub = tcp2ros_node -> create_publisher<readcan_msgs::msg::Canfd>("tcp/canfd", 100);
    readcan_msgs::msg::Canfd canfd_msg;

    int socket_fd = socket_init(can_ethernet_model_name, ip);

    while(rclcpp::ok()){
        if( can_ethernet_model_name == "technica" ){
            unsigned char buffer[1]={};
            std::vector<unsigned char> v;
            //std::vector<unsigned char> v_reverse;
            int count = 0,count_total=5;
            while(count != count_total){
                if(recv(socket_fd,buffer,sizeof(buffer),0)<0){
                    cout<<"rec failed"<<endl;
            }
            v.push_back(buffer[0]);
            bitset<8>char_bit (buffer[0]);
            char_bit  = Reverse_bit(char_bit);
            buffer[0] = Bitset_2_Char(char_bit);
            //v_reverse.push_back(buffer[0]);
            count++;
            if(count ==5 ){
                if(v[0]== 0x00 && v[1]== 0x00 ){
                //printf(" [Paired]---: ");
                    count_total = v[4]+5;
                }
                else{
                    cout << "Skipped" << endl;
                }
            }
        } 
        //v_reverse.erase(v_reverse.begin(),v_reverse.begin()+5);
    
        // cout<<"Size:"<<v.size()<<endl;
        for(int i=0;i<=v.size()-1;++i){
        //cout<<" NO"<<i+1<<":"; 
        // cout<<" ";

        printf("%3X",v[i]);
        }
    // for(int i=0;i<=v.size()-1;++i){
        //  char s = v[i];
        //  cout<<bitset<8>(s);
        //}
        cout<<endl;
        canfd_msg.id = v[2]*16*16+v[3];
        v.erase(v.begin(),v.begin()+5);
        canfd_msg.length = v.size();
        for(int i=0;i<=v.size()-1;++i){
            canfd_msg.data.push_back(v[i]); 
        }
        canfd_msg.header.stamp = tcp2ros_node -> now();
        canfd_msg.header.frame_id ="test";

        tcp2ros_pub->publish(canfd_msg);
        canfd_msg.data.clear();
        rclcpp::spin_some(tcp2ros_node);
    }
    ////////////////////////////////////////////////////////////////////////////////////////
    //  ZLG
    ////////////////////////////////////////////////////////////////////////////////////////
        if(can_ethernet_model_name=="ZLG"){
            unsigned char buffer[1]={};
            std::vector<unsigned char> v;
            int count = 0,count_total=7;
            while(count != count_total){
                    if(recv(socket_fd,buffer,sizeof(buffer),0)<0){
                    cout<<"rec failed"<<endl;
                    }
                    v.push_back(buffer[0]);
                    count++;
                    if(count==6){
                        if(v[0]== 0x55 && v[2]== 0x00 &&v[3]==0x00){
                            printf(" [Paired]---: ");
                            count_total = v[4]*16*16+v[5]+7;
                        }
                        else{
                            printf("Skipped ");
                            count=0;
                        }
                    }
            } 
            //cout<<"Size:"<<v.size()<<endl;
            if(v[1]==0x01){//CANFD msg
                int nSubVecSize = 80;
                for(size_t i = 6; i < v.size(); i += nSubVecSize){
                    vector<int> vecSmall;
                    auto last = std::min(v.size(),  i + nSubVecSize);
                    vecSmall.insert(vecSmall.begin(), v.begin() + i, v.begin() + last);

                    // todo 
                    /****
                        for(int i=0;i<=vecSmall.size()-1;++i){//cout<<" NO"<<i+1<<":"; 
                                                cout<<" ";
                                                printf("%X",vecSmall[i]);
                    ****/

                    canfd_msg.id = vecSmall[10]*16*16+vecSmall[11];
                    canfd_msg.length = vecSmall.size();
                    for(int i=16;i<=vecSmall.size()-1;++i){
                        canfd_msg.data.push_back(vecSmall[i]); 
                    }
                        canfd_msg.header.stamp =tcp2ros_node -> now();

                        canfd_msg.header.frame_id ="test";
                        tcp2ros_pub->publish(canfd_msg);
                        canfd_msg.data.clear();
                }
                        cout<<endl;
        }    
    }
        rclcpp::spin_some(tcp2ros_node);
    }
    close(socket_fd);
    rclcpp::shutdown();
    return 0;
}
