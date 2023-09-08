#include "tcp_client.h"
//#include "delay.h"
//#include "usart1.h"
//#include "lcd.h"
//#include "malloc.h"
#include "string.h"
//#include "comm.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/memp.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h" 
#include "lwip/ip_addr.h" 
#include "lwip/opt.h" 
#include "lwip/ip.h" 
#include "lwip/udp.h"
#include "lwip/tcp.h"

//err_t tcp_client_recv(struct tcp_pcb *tpcb, struct pbuf *p, err_t err );

//uint8_t tcp_client_recvbuf[ TCP_CLIENT_RX_BUFSIZE ] ;                //接收数据缓冲区

//uint16_t  tcp_client_flag;
//extern void read_lwipdata(void); 
//uint8_t udp_demo_flag;
//#define _UDP_DEMO_H_
//#include "sys.h"
//#define UDP_DEMO_RX_BUFSIZE  2000              //定义udp最大接收数据长度 
//#define UDP_DEMO_PORT      8089              //定义udp连接的端口 



/*

*/

 


/*
tcp_recv函数的回调函数

*/
//err_t tcp_client_recv(struct tcp_pcb *tpcb, struct pbuf *p, err_t err )
//{
//  uint32_t data_len=0 ;
//  struct pbuf *q ;
//  struct tcp_client_struct *es ;
//  err_t ret_err ;
//  LWIP_ASSERT( "arg != NULL", arg!=NULL ) ;
//  es = ( struct tcp_client_struct* )arg ;
//  //如果从服务器接收到空的数据帧就关闭连接
//  if( p==NULL )
//  {
//    es->state = ES_TCPCLIENT_CLOSING ;                //需要关闭TCP连接了
//     es->p = p ;
//    ret_err = ERR_OK ;
//  }
//  //当接收到一个非空的数据帧,但是err!=ERR_OK
//  else if( err!=ERR_OK )
//  { 
//    if( p )
//      pbuf_free( p ) ;                        //释放接收pbuf
//    ret_err = err ;
//  }
//  //当处于连接状态时
//  else if( es->state==ES_TCPCLIENT_CONNECTED )
//  {
//    //当处于连接状态并且接收到的数据不为空时
//    if( p!=NULL )
//    {
//      memset( tcp_client_recvbuf, 0, TCP_CLIENT_RX_BUFSIZE ) ;      //数据接收缓冲区清零
//      //遍历完整个pbuf链表
//      for( q=p; q!=NULL; q=q->next )
//      {
//        if( q->len>( TCP_CLIENT_RX_BUFSIZE-data_len ) )
//          memcpy( tcp_client_recvbuf+data_len, q->payload, TCP_CLIENT_RX_BUFSIZE-data_len ) ; 
//        else
//          memcpy( tcp_client_recvbuf+data_len, q->payload, q->len ) ;
//        data_len += q->len ;
//        //超出TCP客户端接收数组,跳出
//        if( data_len>TCP_CLIENT_RX_BUFSIZE )
//          break ;
//      }
//      tcp_client_flag |= 1<<6 ;                    //标记接收到数据了
//       tcp_recved( tpcb,p->tot_len );                  //用于获取接收数据
//      pbuf_free( p ) ;                        //释放内存
//      ret_err = ERR_OK ;
//    }
//  }
//  //接收到数据但是连接已经关闭
//  else
//  { 
//    tcp_recved( tpcb, p->tot_len ) ;                    //用于获取接收数据
//    es->p = NULL ;
//    pbuf_free( p ) ;                          //释放内存
//    ret_err = ERR_OK ;
//  }
//  return ret_err ;
//}


