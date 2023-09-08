/*
tcp_client.h代码编写

*/

#ifndef _TCP_CLIENT_H_
#define _TCP_CLIENT_H_


#include "sys.h"
#include "lwip/tcp.h"
#include "lwip/pbuf.h"
#define TCP_CLIENT_RX_BUFSIZE  1500            //最大接收数据长度
#define TCP_CLIENT_TX_BUFSIZE  200              //最大发送数据长度
#define LWIP_SEND_DATA      0x80            //有数据发送
#define  TCP_CLIENT_PORT    8087            //远端端口
//tcp服务器连接状态
enum tcp_client_states
{
  ES_TCPCLIENT_NONE = 0,    //没有连接
  ES_TCPCLIENT_CONNECTED,  //连接到服务器了 
  ES_TCPCLIENT_CLOSING,    //关闭连接
};
//LWIP回调函数使用的结构体
struct tcp_client_struct
{
  uint8_t state;            //当前连接状
  struct tcp_pcb *pcb;      //指向当前的pcb
  struct pbuf *p;        //指向接收/或传输的pbuf
};  
void tcp_client_test( void ) ;                    //TCP Client测试函数
#endif
