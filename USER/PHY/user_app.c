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

//uint8_t tcp_client_recvbuf[ TCP_CLIENT_RX_BUFSIZE ] ;                //�������ݻ�����

//uint16_t  tcp_client_flag;
//extern void read_lwipdata(void); 
//uint8_t udp_demo_flag;
//#define _UDP_DEMO_H_
//#include "sys.h"
//#define UDP_DEMO_RX_BUFSIZE  2000              //����udp���������ݳ��� 
//#define UDP_DEMO_PORT      8089              //����udp���ӵĶ˿� 



/*

*/

 


/*
tcp_recv�����Ļص�����

*/
//err_t tcp_client_recv(struct tcp_pcb *tpcb, struct pbuf *p, err_t err )
//{
//  uint32_t data_len=0 ;
//  struct pbuf *q ;
//  struct tcp_client_struct *es ;
//  err_t ret_err ;
//  LWIP_ASSERT( "arg != NULL", arg!=NULL ) ;
//  es = ( struct tcp_client_struct* )arg ;
//  //����ӷ��������յ��յ�����֡�͹ر�����
//  if( p==NULL )
//  {
//    es->state = ES_TCPCLIENT_CLOSING ;                //��Ҫ�ر�TCP������
//     es->p = p ;
//    ret_err = ERR_OK ;
//  }
//  //�����յ�һ���ǿյ�����֡,����err!=ERR_OK
//  else if( err!=ERR_OK )
//  { 
//    if( p )
//      pbuf_free( p ) ;                        //�ͷŽ���pbuf
//    ret_err = err ;
//  }
//  //����������״̬ʱ
//  else if( es->state==ES_TCPCLIENT_CONNECTED )
//  {
//    //����������״̬���ҽ��յ������ݲ�Ϊ��ʱ
//    if( p!=NULL )
//    {
//      memset( tcp_client_recvbuf, 0, TCP_CLIENT_RX_BUFSIZE ) ;      //���ݽ��ջ���������
//      //����������pbuf����
//      for( q=p; q!=NULL; q=q->next )
//      {
//        if( q->len>( TCP_CLIENT_RX_BUFSIZE-data_len ) )
//          memcpy( tcp_client_recvbuf+data_len, q->payload, TCP_CLIENT_RX_BUFSIZE-data_len ) ; 
//        else
//          memcpy( tcp_client_recvbuf+data_len, q->payload, q->len ) ;
//        data_len += q->len ;
//        //����TCP�ͻ��˽�������,����
//        if( data_len>TCP_CLIENT_RX_BUFSIZE )
//          break ;
//      }
//      tcp_client_flag |= 1<<6 ;                    //��ǽ��յ�������
//       tcp_recved( tpcb,p->tot_len );                  //���ڻ�ȡ��������
//      pbuf_free( p ) ;                        //�ͷ��ڴ�
//      ret_err = ERR_OK ;
//    }
//  }
//  //���յ����ݵ��������Ѿ��ر�
//  else
//  { 
//    tcp_recved( tpcb, p->tot_len ) ;                    //���ڻ�ȡ��������
//    es->p = NULL ;
//    pbuf_free( p ) ;                          //�ͷ��ڴ�
//    ret_err = ERR_OK ;
//  }
//  return ret_err ;
//}


