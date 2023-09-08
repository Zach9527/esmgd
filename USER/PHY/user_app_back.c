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
uint8_t tcp_client_recvbuf[ TCP_CLIENT_RX_BUFSIZE ] ;                //接收数据缓冲区

uint16_t  tcp_client_flag;
/*

目标是把通过网口接收到的数据存储到RxBuffer中

*/

//void udp_echoserver_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
//{
//	  struct udp_pcb* PCB;
//	  ip_addr_t  destAddr;
//	  uint16_t destPort;
//	  uint16_t *RxBuffer;
//	  uint16_t RxCount;
//	
//    destAddr = *addr;
//	  PCB = upcb;
//	  destPort = port;
//    /* Connect to the remote client */
//    udp_connect(upcb, addr, port);

//    /* Tell the client that we have accepted it */
////    udp_send(upcb, p);
//	  if(p != NULL)
//	  { 
//	 	/******将数据原样返回*******************/
////		  udp_sendto(upcb, p, &destAddr, port); /* 将收到的数据再发送出去 */	
//		  memcpy(RxBuffer+RxCount,(unsigned char*)p->payload,p->len);
//		  RxCount += p->len;
//	  }
// 
//    /* free the UDP connection, so we can accept new clients */
//    udp_disconnect(upcb);

//    /* Free the p buffer */
//    pbuf_free(p);

//}




/*

然后通过函数将收到的数据进行处理
读取网络中断接收到的数据到了RxBuffer中
并且存储到自己定义的指针中
*/
//查看缓冲区是否受到数据
//uint8_t udp_HasData(void)
//{
//	if(RxCount >= 4)
//		return RxCount;
//	return 0;
//}
////缓冲区读出来
//uint16_t udp_ReadData(uint8_t *buff)
//{
//	uint16_t len = RxCount;
//	if(len > 0)
//		memcpy(buff,RxBuffer,len);
//	memset(RxBuffer,0,255);
//	RxCount = 0;
//	return len;
//}

//void udp_SendData(uint8_t *buff,uint16_t len)
//{
//	struct pbuf *p;
//	p = pbuf_alloc(PBUF_RAW,len,PBUF_RAM);
//	p->payload = (void*)TxBuffer;
//	memcpy(TxBuffer,buff,len);
//	udp_sendto(PCB,p,&destAddr,destPort);
//	pbuf_free(p);
//}



///*
//解决了ping通的问题，现在就要开始想怎么处理接收到的数据，
//在原来的服务函数中是这样的
//*/
//void udp_echoserver_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
//{
//    /* Connect to the remote client */
//    udp_connect(upcb, addr, port);

//    /* Tell the client that we have accepted it */
//    udp_send(upcb, p);
// 
//    /* free the UDP connection, so we can accept new clients */
//    udp_disconnect(upcb);

//    /* Free the p buffer */
//    pbuf_free(p);

//}
///*
//其中udp_send函数就是收到后又把p发送了回去，跳转到udp_send函数
//*/
///**
// * @ingroup udp_raw
// * Sends the pbuf p using UDP. The pbuf is not deallocated.
// *
// *
// * @param pcb UDP PCB used to send the data.
// * @param p chain of pbuf's to be sent.
// *
// * The datagram will be sent to the current remote_ip & remote_port
// * stored in pcb. If the pcb is not bound to a port, it will
// * automatically be bound to a random port.
// *
// * @return lwIP error code.
// * - ERR_OK. Successful. No error occurred.
// * - ERR_MEM. Out of memory.
// * - ERR_RTE. Could not find route to destination address.
// * - ERR_VAL. No PCB or PCB is dual-stack
// * - More errors could be returned by lower protocol layers.
// *
// * @see udp_disconnect() udp_sendto()
// */
//err_t
//udp_send(struct udp_pcb *pcb, struct pbuf *p)
//{
//  LWIP_ERROR("udp_send: invalid pcb", pcb != NULL, return ERR_ARG);
//  LWIP_ERROR("udp_send: invalid pbuf", p != NULL, return ERR_ARG);

//  if (IP_IS_ANY_TYPE_VAL(pcb->remote_ip)) {
//    return ERR_VAL;
//  }

//  /* send to the packet using remote ip and port stored in the pcb */
//  return udp_sendto(pcb, p, &pcb->remote_ip, pcb->remote_port);
//}
//// 在udp_send函数中，能看到被封装好的udp_sendto()函数，这个是实际进行收发的函数。


///*
//tcp_client.h代码编写

//*/

//#ifndef _TCP_CLIENT_H_
//#define _TCP_CLIENT_H_
//#include "sys.h"
//#include "lwip/tcp.h"
//#include "lwip/pbuf.h"
//#define TCP_CLIENT_RX_BUFSIZE  1500            //最大接收数据长度
//#define TCP_CLIENT_TX_BUFSIZE  200              //最大发送数据长度
//#define LWIP_SEND_DATA      0x80            //有数据发送
//#define  TCP_CLIENT_PORT    8087            //远端端口
////tcp服务器连接状态
//enum tcp_client_states
//{
//  ES_TCPCLIENT_NONE = 0,    //没有连接
//  ES_TCPCLIENT_CONNECTED,  //连接到服务器了 
//  ES_TCPCLIENT_CLOSING,    //关闭连接
//};
////LWIP回调函数使用的结构体
//struct tcp_client_struct
//{
//  u8 state;            //当前连接状
//  struct tcp_pcb *pcb;      //指向当前的pcb
//  struct pbuf *p;        //指向接收/或传输的pbuf
//};  
//void tcp_client_test( void ) ;                    //TCP Client测试函数
//#endif

///*

//应用编写
//tcp_client.c代码编写
//*/

////TCP Client 测试全局状态标记变量
////bit7:0,没有数据要发送;1,有数据要发送
////bit6:0,没有收到数据;1,收到数据了
////bit5:0,没有连接上服务器;1,连接上服务器了
////bit4~0:保留
//u8 tcp_client_flag;   
////设置远端IP地址
//void tcp_client_set_remoteip()
//{
//  u8 *tbuf;
//  tbuf=mymalloc( SRAMIN, 100 ) ;                      //申请内存
//  if( tbuf==NULL )
//    return ;
//  //前三个IP保持和DHCP得到的IP一致
//  lwipdev.remoteip[ 0 ] = lwipdev.ip[ 0 ] ;
//  lwipdev.remoteip[ 1 ] = lwipdev.ip[ 1 ] ;
//  lwipdev.remoteip[ 2 ] = lwipdev.ip[ 2 ] ;
//  lwipdev.remoteip[ 3 ] = 113 ;
//  sprintf( ( char* )tbuf, "Remote IP:%d.%d.%d.", lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2] ) ;
//  LCD_ShowString( 30, 150, tbuf ) ;                    //远端IP
//  myfree( SRAMIN, tbuf ) ;
//}
////关闭与服务器的连接
//void tcp_client_connection_close( struct tcp_pcb *tpcb, struct tcp_client_struct *es )
//{
//  tcp_abort( tpcb ) ;                            //终止连接,删除pcb控制块
//  tcp_arg( tpcb, NULL ) ;
//  tcp_recv( tpcb, NULL ) ;
//  tcp_sent( tpcb, NULL ) ;
//  tcp_err( tpcb, NULL ) ;
//  tcp_poll( tpcb, NULL, 0 );
//  if( es )
//    mem_free( es ) ;
//  tcp_client_flag &= ~( 1<<5 ) ;                      //标记连接断开了
//}
// tcp_recv函数的回调函数
err_t tcp_client_recv( void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err )
{
  uint32_t data_len=0 ;
  struct pbuf *q ;
  struct tcp_client_struct *es ;
  err_t ret_err ;
  LWIP_ASSERT( "arg != NULL", arg!=NULL ) ;
  es = ( struct tcp_client_struct* )arg ;
  //如果从服务器接收到空的数据帧就关闭连接
  if( p==NULL )
  {
    es->state = ES_TCPCLIENT_CLOSING ;                //需要关闭TCP连接了
     es->p = p ;
    ret_err = ERR_OK ;
  }
  //当接收到一个非空的数据帧,但是err!=ERR_OK
  else if( err!=ERR_OK )
  { 
    if( p )
      pbuf_free( p ) ;                        //释放接收pbuf
    ret_err = err ;
  }
  //当处于连接状态时
  else if( es->state==ES_TCPCLIENT_CONNECTED )
  {
    //当处于连接状态并且接收到的数据不为空时
    if( p!=NULL )
    {
      memset( tcp_client_recvbuf, 0, TCP_CLIENT_RX_BUFSIZE ) ;      //数据接收缓冲区清零
      //遍历完整个pbuf链表
      for( q=p; q!=NULL; q=q->next )
      {
        if( q->len>( TCP_CLIENT_RX_BUFSIZE-data_len ) )
          memcpy( tcp_client_recvbuf+data_len, q->payload, TCP_CLIENT_RX_BUFSIZE-data_len ) ; 
        else
          memcpy( tcp_client_recvbuf+data_len, q->payload, q->len ) ;
        data_len += q->len ;
        //超出TCP客户端接收数组,跳出
        if( data_len>TCP_CLIENT_RX_BUFSIZE )
          break ;
      }
      tcp_client_flag |= 1<<6 ;                    //标记接收到数据了
       tcp_recved( tpcb,p->tot_len );                  //用于获取接收数据
      pbuf_free( p ) ;                        //释放内存
      ret_err = ERR_OK ;
    }
  }
  //接收到数据但是连接已经关闭
  else
  { 
    tcp_recved( tpcb, p->tot_len ) ;                    //用于获取接收数据
    es->p = NULL ;
    pbuf_free( p ) ;                          //释放内存
    ret_err = ERR_OK ;
  }
  return ret_err ;
}
//// tcp_err函数的回调函数
//void tcp_client_error( void *arg, err_t err )
//{

//}
////发送数据
//void tcp_client_senddata( struct tcp_pcb *tpcb, struct tcp_client_struct *es )
//{
//  struct pbuf *ptr ; 
//   err_t wr_err = ERR_OK ;
//  while( ( wr_err==ERR_OK )&&( es->p )&&( es->p->len<=tcp_sndbuf( tpcb ) ) )
//  {
//    ptr = es->p ;
//    wr_err = tcp_write( tpcb, ptr->payload, ptr->len, 1 ) ;          //数据加入到发送缓冲队列中
//    if( wr_err==ERR_OK )
//    {
//      es->p = ptr->next ;                      //指向下一个pbuf
//      //pbuf的ref加一
//      if( es->p )
//        pbuf_ref( es->p );
//      pbuf_free( ptr ) ;                        //释放ptr
//    }
//    else if( wr_err==ERR_MEM )
//      es->p = ptr ;
//    tcp_output( tpcb ) ;                        //发送缓冲队列中的数据发送
//  }
//}
//// tcp_sent的回调函数(从远端接收到ACK后发送数据)
//err_t tcp_client_sent( void *arg, struct tcp_pcb *tpcb, u16_t len )
//{
//  struct tcp_client_struct *es ;
//  LWIP_UNUSED_ARG( len ) ;
//  es = ( struct tcp_client_struct* )arg ;
//  if( es->p )
//    tcp_client_senddata( tpcb, es ) ;                    //发送数据
//  return ERR_OK ;
//}
//// tcp_poll的回调函数
//const u8 *tcp_client_sendbuf = "STM32F103 TCP Client send data\r\n" ;      //TCP服务器发送数据内容
//err_t tcp_client_poll( void *arg, struct tcp_pcb *tpcb )
//{
//  err_t ret_err ;
//  struct tcp_client_struct *es ; 
//  es = ( struct tcp_client_struct* )arg ;
//  //连接处于空闲可以发送数据
//  if( es!=NULL )
//  {
//    //判断是否有数据要发送
//    if( tcp_client_flag&( 1<<7 ) )
//    {
//      es->p = pbuf_alloc( PBUF_TRANSPORT, strlen( ( char* )tcp_client_sendbuf ), PBUF_POOL ) ; 
//      pbuf_take( es->p, ( char* )tcp_client_sendbuf, strlen( ( char* )tcp_client_sendbuf ) ) ; 
//      tcp_client_senddata( tpcb, es ) ;                  //将数据发送出去
//      tcp_client_flag &= ~( 1<<7 ) ;                  //清除数据发送标志
//      //释放内存
//      if( es->p )
//        pbuf_free( es->p ) ;
//    }
//    else if( es->state==ES_TCPCLIENT_CLOSING )
//       tcp_client_connection_close( tpcb, es ) ;              //关闭TCP连接
//    ret_err = ERR_OK ;
//  }
//  else
//  { 
//    tcp_abort( tpcb ) ;                          //终止连接,删除pcb控制块
//    ret_err = ERR_ABRT ;
//  }
//  return ret_err ;
//}
////连接建立后调用回调函数
//err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
//{
//  struct tcp_client_struct *es=NULL;  
//  if(err==ERR_OK)   
//  {
//    es = ( struct tcp_client_struct* )mem_malloc( sizeof( struct tcp_client_struct ) ) ; //申请内存
//    //内存申请成功
//    if( es )
//    {
//       es->state = ES_TCPCLIENT_CONNECTED ;            //状态为连接成功
//      es->pcb = tpcb ;
//      es->p = NULL ;
//      tcp_arg( tpcb, es ) ;                      //更新tpcb的callback_arg
//      tcp_recv( tpcb, tcp_client_recv ) ;                  //初始化tcp_recv回调功能
//      tcp_err( tpcb, tcp_client_error ) ;                  //初始化tcp_err()回调函数
//      tcp_sent( tpcb, tcp_client_sent ) ;                  //初始化tcp_sent回调功能
//      tcp_poll( tpcb, tcp_client_poll, 1 ) ;                //初始化tcp_poll回调功能
//       tcp_client_flag |= 1<<5 ;                    //标记连接到服务器了
//      err = ERR_OK ;
//    }
//    else
//    {
//      tcp_client_connection_close( tpcb, es ) ;              //关闭连接
//      err = ERR_MEM ;                        //返回内存分配错误
//    }
//  }
//  else
//    tcp_client_connection_close( tpcb, 0 ) ;                //关闭连接
//  return err ;
//}
////客户机测试
//void tcp_client_test()
//{
//   struct tcp_pcb *tcppcb ;                        //定义一个TCP服务器控制块
//  struct ip_addr rmtipaddr ;                        //远端ip地址
//  u8 *tbuf ;
//  u8 res=0 ;    
//  u8 t=0 ; 
//  u8 connflag=0 ;                            //连接标记
//  tcp_client_set_remoteip() ;                        //先选择IP
//  tbuf = mymalloc( SRAMIN, 200 ) ;                    //申请内存
//  //内存申请失败了,直接退出
//  if( tbuf==NULL )
//    return ;
//  sprintf( ( char* )tbuf, "Local IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 130, tbuf ) ;                    //服务器IP
//  sprintf( ( char* )tbuf, "Remote IP:%d.%d.%d.%d", lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2], lwipdev.remoteip[3] ) ;
//  LCD_ShowString( 30, 150, tbuf ) ;                    //远端IP
//  sprintf( ( char* )tbuf, "Remote Port:%d", TCP_CLIENT_PORT ) ;          //客户端端口号
//  LCD_ShowString( 30, 170, tbuf ) ;
//  LCD_ShowString( 30, 190, "STATUS:Disconnected" ) ;
//  tcppcb = tcp_new() ;                          //创建一个新的pcb
//  //创建成功
//  if( tcppcb )
//  {
//  IP4_ADDR( &rmtipaddr, lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2], lwipdev.remoteip[3] ) ;
//    tcp_connect( tcppcb, &rmtipaddr, TCP_CLIENT_PORT, tcp_client_connected ) ; 
//   }
//  else
//    res = 1 ;
//  while( res==0 )
//  {
//    //是否收到数据
//    if( tcp_client_flag&1<<6 )
//    {
//      LCD_ShowString( 30, 230, tcp_client_recvbuf ) ;            //显示接收到的数据
//      tcp_client_flag |= 1<<7 ;                    //标记要发送数据
//      tcp_client_flag &= ~( 1<<6 ) ;                  //标记数据已经被处理了
//    }
//    //是否连接上
//    if( tcp_client_flag&1<<5 )
//    {
//      if( connflag==0 )
//      { 
//        LCD_ShowString( 30, 190, "STATUS:Connected   " ) ;
//        LCD_ShowString( 30, 210, "Receive Data:" ) ;
//        connflag = 1 ;                        //标记连接了
//      }
//    }
//    else if( connflag )
//    {
//       LCD_ShowString( 30, 190, "STATUS:Disconnected" ) ;
//      connflag = 0 ;                          //标记连接断开了
//    } 
//    lwip_periodic_handle() ;
//    lwip_pkt_handle() ;
//    delay_ms( 2 ) ;
//    t ++ ;
//    if( t==200 )
//    {
//      //未连接上,则尝试重连
//      if( ( connflag==0 )&&( ( tcp_client_flag&1<<5 )==0 ) )
//      { 
//        tcp_client_connection_close( tcppcb, 0 ) ;            //关闭连接
//        tcppcb = tcp_new() ;                    //创建一个新的pcb
//        //创建成功
//        if( tcppcb )
//          tcp_connect( tcppcb, &rmtipaddr, TCP_CLIENT_PORT, tcp_client_connected ) ; 
//      }
//      t = 0 ;
//    }    
//  }
//  tcp_client_connection_close( tcppcb, 0 ) ;                  //关闭TCP Client连接
//  myfree( SRAMIN, tbuf ) ;
//}

//26.2.2 tcp_client.h代码编写


///*
//有关于TCP协议的知识在上一章已经有过描述，这里我们直接使用API来实现TCP服务器模式。
//*/
//27.1 实验例程
//27.1.1 tcp_server.c代码编写
//#include "tcp_server.h"
//#include "delay.h"
//#include "usart1.h"
//#include "lcd.h"
//#include "malloc.h"
//#include "string.h"
//#include "lwip/debug.h"
//#include "lwip/stats.h"
//#include "lwip/memp.h"
//#include "lwip/mem.h"
//#include "comm.h"
////TCP Server 测试全局状态标记变量
////bit7:0,没有数据要发送;1,有数据要发送
////bit6:0,没有收到数据;1,收到数据了.
////bit5:0,没有客户端连接上;1,有客户端连接上了.
////bit4~0:保留
//u8 tcp_server_flag;
////关闭tcp连接
//void tcp_server_connection_close( struct tcp_pcb *tpcb, struct tcp_server_struct *es )
//{
//  tcp_close( tpcb ) ;
//  tcp_arg( tpcb, NULL ) ;
//  tcp_sent( tpcb, NULL ) ;
//  tcp_recv( tpcb, NULL ) ;
//  tcp_err( tpcb, NULL ) ;
//  tcp_poll( tpcb, NULL, 0 ) ;
//  if( es )
//    mem_free( es ) ; 
//  tcp_server_flag &= ~( 1<<5 ) ;                      //标记连接断开了
//}
//// tcp_recv函数的回调函数
//u8 tcp_server_recvbuf[ TCP_SERVER_RX_BUFSIZE ] ;                //TCP Server接收数据缓冲区
//err_t tcp_server_recv( void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err )
//{
//  err_t ret_err ;
//  u32 data_len = 0 ;
//  struct pbuf *q ;
//    struct tcp_server_struct *es ;
//  LWIP_ASSERT( "arg != NULL", arg != NULL ) ;
//  es = ( struct tcp_server_struct* )arg ;
//  //从客户端接收到空数据
//  if( p==NULL )
//  {
//    es->state = ES_TCPSERVER_CLOSING ;                //需要关闭TCP连接了
//    es->p = p ; 
//    ret_err = ERR_OK ;
//  }
//  //从客户端接收到一个非空数据,但是由于某种原因err!=ERR_OK
//  else if( err!=ERR_OK )
//  {
//    if( p )
//      pbuf_free( p ) ;                        //释放接收pbuf
//    ret_err = err ;
//  }
//  //处于连接状态
//  else if( es->state==ES_TCPSERVER_ACCEPTED )
//  {
//    //当处于连接状态并且接收到的数据不为空时将其打印出来
//    if( p!=NULL )
//    {
//      memset( tcp_server_recvbuf, 0, TCP_SERVER_RX_BUFSIZE ) ;      //数据接收缓冲区清零
//      //遍历完整个pbuf链表
//      for( q=p; q!=NULL; q=q->next )
//      {
//        if( q->len>( TCP_SERVER_RX_BUFSIZE-data_len ) )
//          memcpy( tcp_server_recvbuf+data_len, q->payload, TCP_SERVER_RX_BUFSIZE-data_len ) ; 
//        else
//          memcpy(tcp_server_recvbuf+data_len, q->payload, q->len ) ;
//        data_len += q->len ;
//        //超出TCP客户端接收数组,跳出
//        if( data_len>TCP_SERVER_RX_BUFSIZE )
//          break ;
//      }
//      tcp_server_flag |= 1<<6 ;                    //标记接收到数据了
//      lwipdev.remoteip[ 0 ] = tpcb->remote_ip.addr&0xFF ;        //IADDR4
//      lwipdev.remoteip[ 1 ] = ( tpcb->remote_ip.addr>>8 )&0xFF ;    //IADDR3
//      lwipdev.remoteip[ 2 ] = ( tpcb->remote_ip.addr>>16 )&0xFF ;    //IADDR2
//      lwipdev.remoteip[ 3 ] = ( tpcb->remote_ip.addr>>24 )&0xFF ;    //IADDR1 
//       tcp_recved( tpcb, p->tot_len ) ;                  //用于获取接收数据
//      pbuf_free( p ) ;                        //释放内存
//      ret_err = ERR_OK ;
//    }
//  }
//  //服务器关闭了
//  else
//  {
//    tcp_recved( tpcb, p->tot_len  );                    //用于获取接收数据
//    es->p=  NULL ;
//    pbuf_free( p ) ;                          //释放内存
//    ret_err = ERR_OK ;
//  }
//  return ret_err ;
//}
//// tcp_err函数的回调函数
//void tcp_server_error( void *arg, err_t err )
//{  
//  LWIP_UNUSED_ARG( err ) ;
//  printf( "tcp error:%x\r\n", ( u32 )arg ) ;
//  //释放内存
//  if( arg!=NULL )
//    mem_free( arg ) ;
//}
////发送数据
//void tcp_server_senddata( struct tcp_pcb *tpcb, struct tcp_server_struct *es )
//{
//  struct pbuf *ptr ;
//  u16 plen ;
//  err_t wr_err = ERR_OK ;
//   while( ( wr_err==ERR_OK )&&( es->p )&&( es->p->len<=tcp_sndbuf( tpcb ) ) )
//   {
//    ptr = es->p ;
//    wr_err = tcp_write( tpcb, ptr->payload, ptr->len, 1 ) ;
//    if( wr_err==ERR_OK )
//    { 
//      plen = ptr->len ;
//      es->p = ptr->next ;                      //指向下一个pbuf
//      //pbuf的ref加一
//      if( es->p )
//        pbuf_ref( es->p ) ;
//      pbuf_free( ptr ) ;
//      tcp_recved( tpcb, plen ) ;                    //更新tcp窗口大小
//    }
//    else if( wr_err==ERR_MEM )
//      es->p = ptr ;
//   }
//}
//// tcp_poll的回调函数
//const u8 *tcp_server_sendbuf = "STM32F103 TCP Server send data\r\n" ;      //TCP服务器发送数据内容
//err_t tcp_server_poll( void *arg, struct tcp_pcb *tpcb )
//{
//  err_t ret_err;
//  struct tcp_server_struct *es ; 
//  es = ( struct tcp_server_struct* )arg ;
//  if( es!=NULL )
//  {
//    //判断是否有数据要发送
//    if( tcp_server_flag&( 1<<7 ) )
//    {
//      es->p = pbuf_alloc( PBUF_TRANSPORT, strlen( ( char* )tcp_server_sendbuf ), PBUF_POOL ) ; 
//      pbuf_take( es->p, ( char* )tcp_server_sendbuf, strlen( ( char* )tcp_server_sendbuf ) ) ;
//      tcp_server_senddata( tpcb, es ) ;                  //轮询的时候发送要发送的数据
//      tcp_server_flag &= ~( 1<<7 ) ;                  //清除数据发送标志位
//      if( es->p!=NULL )
//        pbuf_free( es->p ) ;                    //释放内存  
//    }
//    //关闭操作
//    else if( es->state==ES_TCPSERVER_CLOSING )
//      tcp_server_connection_close( tpcb, es ) ;              //关闭连接
//    ret_err = ERR_OK ;
//  }
//  else
//  {
//    tcp_abort( tpcb ) ;                          //终止连接,删除pcb控制块
//    ret_err = ERR_ABRT ; 
//  }
//  return ret_err ;
//}
//// tcp_sent的回调函数(远端收到ACK信号后发送数据)
//err_t tcp_server_sent( void *arg, struct tcp_pcb *tpcb, u16_t len )
//{
//  struct tcp_server_struct *es ;
//  LWIP_UNUSED_ARG( len ) ;
//  es = ( struct tcp_server_struct * )arg ;
//  if( es->p )
//    tcp_server_senddata( tpcb, es ) ;                    //发送数据
//  return ERR_OK ;
//}
////强制删除主动断开时的time wait
//extern void tcp_pcb_purge( struct tcp_pcb *pcb ) ;                //在 tcp.c里面
//extern struct tcp_pcb *tcp_active_pcbs ;                    //在 tcp.c里面
//extern struct tcp_pcb *tcp_tw_pcbs ;                      //在 tcp.c里面
//void tcp_server_remove_timewait()
//{
//  struct tcp_pcb *pcb, *pcb2 ; 
//  while( tcp_active_pcbs!=NULL )
//  {
//    lwip_periodic_handle() ;                      //继续轮询
//    lwip_pkt_handle() ;
//     delay_ms( 10 ) ;                          //等待tcp_active_pcbs为空  
//  }
//  pcb = tcp_tw_pcbs ;
//  //如果有等待状态的pcbs
//  while( pcb!=NULL )
//  {
//    tcp_pcb_purge( pcb ) ;
//    tcp_tw_pcbs = pcb->next ;
//    pcb2 = pcb ;
//    pcb = pcb->next ;
//    memp_free( MEMP_TCP_PCB, pcb2 ) ;
//  }
//}
//// tcp_accept的回调函数
//err_t tcp_server_accept( void *arg, struct tcp_pcb *newpcb, err_t err )
//{
//  err_t ret_err ;
//  struct tcp_server_struct *es ;
//   LWIP_UNUSED_ARG( arg ) ;
//  LWIP_UNUSED_ARG( err ) ;
//  tcp_setprio( newpcb, TCP_PRIO_MIN ) ;                                //设置新创建的pcb优先级
//  es = ( struct tcp_server_struct* )mem_malloc( sizeof( struct tcp_server_struct ) ) ;  //分配内存
//  //内存分配成功
//   if( es!=NULL )
//  {
//    es->state = ES_TCPSERVER_ACCEPTED ;                //接收连接
//    es->pcb = newpcb ;
//    es->p = NULL ;
//    tcp_arg( newpcb, es ) ;
//    tcp_recv( newpcb, tcp_server_recv ) ;                  //初始化tcp_recv的回调函数
//    tcp_err( newpcb, tcp_server_error ) ;                  //初始化tcp_err回调函数
//    tcp_poll( newpcb, tcp_server_poll, 1 ) ;                //初始化tcp_poll回调函数
//    tcp_sent( newpcb, tcp_server_sent ) ;                  //初始化发送回调函数
//    tcp_server_flag |= 1<<5 ;                      //标记有客户端连上了
//    lwipdev.remoteip[ 0 ] = newpcb->remote_ip.addr&0xFF ;        //IADDR4
//    lwipdev.remoteip[ 1 ] = ( newpcb->remote_ip.addr>>8 )&0xFF ;      //IADDR3
//    lwipdev.remoteip[ 2 ] = ( newpcb->remote_ip.addr>>16 )&0xFF ;      //IADDR2
//    lwipdev.remoteip[ 3 ] = ( newpcb->remote_ip.addr>>24 )&0xFF ;      //IADDR1
//    ret_err = ERR_OK ;
//  }
//  else
//    ret_err = ERR_MEM ;
//  return ret_err ;
//}
//// TCP Server 测试
//void tcp_server_test()
//{
//  err_t err ;  
//  struct tcp_pcb *tcppcbnew ;                        //定义一个TCP服务器控制块
//  struct tcp_pcb *tcppcbconn ;                      //定义一个TCP服务器控制块
//  u8 *tbuf ;
//  u8 res=0 ;
//  u8 connflag=0 ;                            //连接标记
//  tbuf = mymalloc( SRAMIN, 200 ) ;                    //申请内存
//  //内存申请失败了,直接退出
//  if( tbuf==NULL )
//    return ;
//  sprintf( ( char* )tbuf, "Server IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 130, tbuf ) ;                    //服务器IP
//  sprintf( ( char* )tbuf, "Server Port:%d", TCP_SERVER_PORT ) ;
//  LCD_ShowString( 30, 150, tbuf ) ;                    //服务器端口号
//  tcppcbnew = tcp_new() ;                        //创建一个新的pcb
//  //创建成功
//  if( tcppcbnew )
//  {
//    err = tcp_bind( tcppcbnew, IP_ADDR_ANY, TCP_SERVER_PORT ) ;      //将本地IP与指定端口号绑定
//    //绑定完成
//    if( err==ERR_OK )
//    {
//      tcppcbconn = tcp_listen( tcppcbnew ) ;              //设置tcppcb进入监听状态
//      tcp_accept( tcppcbconn, tcp_server_accept ) ;            //初始化tcp_accept的回调函数
//    }
//    else
//      res = 1 ;
//  }
//  else
//    res = 1 ;
//  while( res==0 )
//  {
//    //收到数据
//    if( tcp_server_flag&1<<6 )
//    {
//      tcp_server_flag |= 1<<7 ;                    //标记要发送数据
//      LCD_ShowString( 30, 210, tcp_server_recvbuf ) ;          //显示接收到的数据
//      tcp_server_flag &= ~( 1<<6 ) ;                  //标记数据已经被处理了
//    }
//    //是否连接上
//    if( tcp_server_flag&1<<5 )
//    {
//      if( connflag==0 )
//      { 
//      sprintf( ( char* )tbuf, "Client IP:%d.%d.%d.%d", lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2], lwipdev.remoteip[3] ) ;
//         LCD_ShowString( 30, 170, tbuf ) ;              //客户端IP
//        LCD_ShowString( 30, 190, "Receive Data:" ) ;          //提示消息
//        connflag = 1 ;                        //标记连接了
//      }
//    }
//    else if( connflag )
//      connflag = 0 ;                          //标记连接断开了
//    lwip_periodic_handle() ;
//    lwip_pkt_handle() ;
//    delay_ms( 2 ) ;
//  }   
//  tcp_server_connection_close( tcppcbnew, 0 ) ;                //关闭TCP Server连接
//  tcp_server_connection_close( tcppcbconn, 0 ) ;                //关闭TCP Server连接
//  tcp_server_remove_timewait() ; 
//  memset( tcppcbnew, 0, sizeof( struct tcp_pcb ) ) ;
//  memset( tcppcbconn, 0, sizeof( struct tcp_pcb ) ) ;
//  myfree( SRAMIN, tbuf ) ;
//}

//27.1.2 tcp_server.h代码编写
//#ifndef _TCP_SERVER_DEMO_H_
//#define _TCP_SERVER_DEMO_H_
//#include "sys.h"
//#include "lwip/tcp.h"
//#include "lwip/pbuf.h"
//#define TCP_SERVER_RX_BUFSIZE  2000          //定义tcp server最大接收数据长度
//#define TCP_SERVER_PORT      8088          //定义tcp server的端口
////tcp服务器连接状态
//enum tcp_server_states
//{
//  ES_TCPSERVER_NONE = 0,      //没有连接
//  ES_TCPSERVER_ACCEPTED,      //有客户端连接上了
//  ES_TCPSERVER_CLOSING,      //即将关闭连接
//};
////LWIP回调函数使用的结构体
//struct tcp_server_struct
//{
//  u8 state;              //当前连接状
//  struct tcp_pcb *pcb;        //指向当前的pcb
//  struct pbuf *p;          //指向接收/或传输的pbuf
//}; 
//void tcp_server_test( void ) ;                  //TCP Server测试函数
//#endif

//27.1.3 主函数代码编写
//#include "sys.h"
//#include "delay.h"
//#include "usart1.h"
//#include "tim.h"
//#include "lcd.h"
//#include "malloc.h"
//#include "dm9000.h"
//#include "lwip/netif.h"
//#include "comm.h"
//#include "lwipopts.h"
//#include "tcp_server.h"
//int main()
//{
//  u8 buf[ 30 ];
//   STM32_Clock_Init( 9 ) ;                                        //系统时钟设置
//  SysTick_Init( 72 ) ;                          //延时初始化
//  USART1_Init( 72, 115200 ) ;                      //串口初始化为115200
//  LCD_Init() ;                            //初始化LCD
//  TIM3_Init( 1000, 719 ) ;                        //定时器3频率为100hz
//  my_mem_init( SRAMIN ) ;                      //初始化内部内存池
//  while( lwip_comm_init() ) ;                      //lwip初始化
//  //等待DHCP获取成功/超时溢出
//  while( ( lwipdev.dhcpstatus!=2 )&&( lwipdev.dhcpstatus!=0xFF ) )
//  {
//    lwip_periodic_handle() ;                    //LWIP内核需要定时处理的函数
//    lwip_pkt_handle() ;
//  }
//  POINT_COLOR=RED;
//  LCD_ShowString( 30, 110, "LWIP Init Successed" ) ;
//  //打印动态IP地址
//  if( lwipdev.dhcpstatus==2 )
//    sprintf( ( char* )buf, "DHCP IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  //打印静态IP地址
//  else
//    sprintf( ( char* )buf, "Static IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 130, buf ) ; 
//  //得到网速
//  if( ( DM9000_Get_SpeedAndDuplex()&0x02 )==0x02 )
//    LCD_ShowString( 30, 150, "Ethernet Speed:10M" ) ;
//  else
//    LCD_ShowString( 30, 150, "Ethernet Speed:100M" ) ;
//   while( 1 )
//  {
//    tcp_server_test() ;                        //TCP服务器测试
//    lwip_periodic_handle() ;
//    lwip_pkt_handle() ;
//    delay_ms( 2 ) ;
//  }
//}



//有关于TCP协议的知识在上一章已经有过描述，这里我们直接使用API来实现TCP服务器模式。

//27.1 实验例程
//27.1.1 tcp_server.c代码编写
//#include "tcp_server.h"
//#include "delay.h"
//#include "usart1.h"
//#include "lcd.h"
//#include "malloc.h"
//#include "string.h"
//#include "lwip/debug.h"
//#include "lwip/stats.h"
//#include "lwip/memp.h"
//#include "lwip/mem.h"
//#include "comm.h"
////TCP Server 测试全局状态标记变量
////bit7:0,没有数据要发送;1,有数据要发送
////bit6:0,没有收到数据;1,收到数据了.
////bit5:0,没有客户端连接上;1,有客户端连接上了.
////bit4~0:保留
//u8 tcp_server_flag;
////关闭tcp连接
//void tcp_server_connection_close( struct tcp_pcb *tpcb, struct tcp_server_struct *es )
//{
//  tcp_close( tpcb ) ;
//  tcp_arg( tpcb, NULL ) ;
//  tcp_sent( tpcb, NULL ) ;
//  tcp_recv( tpcb, NULL ) ;
//  tcp_err( tpcb, NULL ) ;
//  tcp_poll( tpcb, NULL, 0 ) ;
//  if( es )
//    mem_free( es ) ; 
//  tcp_server_flag &= ~( 1<<5 ) ;                      //标记连接断开了
//}
//// tcp_recv函数的回调函数
//u8 tcp_server_recvbuf[ TCP_SERVER_RX_BUFSIZE ] ;                //TCP Server接收数据缓冲区
//err_t tcp_server_recv( void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err )
//{
//  err_t ret_err ;
//  u32 data_len = 0 ;
//  struct pbuf *q ;
//    struct tcp_server_struct *es ;
//  LWIP_ASSERT( "arg != NULL", arg != NULL ) ;
//  es = ( struct tcp_server_struct* )arg ;
//  //从客户端接收到空数据
//  if( p==NULL )
//  {
//    es->state = ES_TCPSERVER_CLOSING ;                //需要关闭TCP连接了
//    es->p = p ; 
//    ret_err = ERR_OK ;
//  }
//  //从客户端接收到一个非空数据,但是由于某种原因err!=ERR_OK
//  else if( err!=ERR_OK )
//  {
//    if( p )
//      pbuf_free( p ) ;                        //释放接收pbuf
//    ret_err = err ;
//  }
//  //处于连接状态
//  else if( es->state==ES_TCPSERVER_ACCEPTED )
//  {
//    //当处于连接状态并且接收到的数据不为空时将其打印出来
//    if( p!=NULL )
//    {
//      memset( tcp_server_recvbuf, 0, TCP_SERVER_RX_BUFSIZE ) ;      //数据接收缓冲区清零
//      //遍历完整个pbuf链表
//      for( q=p; q!=NULL; q=q->next )
//      {
//        if( q->len>( TCP_SERVER_RX_BUFSIZE-data_len ) )
//          memcpy( tcp_server_recvbuf+data_len, q->payload, TCP_SERVER_RX_BUFSIZE-data_len ) ; 
//        else
//          memcpy(tcp_server_recvbuf+data_len, q->payload, q->len ) ;
//        data_len += q->len ;
//        //超出TCP客户端接收数组,跳出
//        if( data_len>TCP_SERVER_RX_BUFSIZE )
//          break ;
//      }
//      tcp_server_flag |= 1<<6 ;                    //标记接收到数据了
//      lwipdev.remoteip[ 0 ] = tpcb->remote_ip.addr&0xFF ;        //IADDR4
//      lwipdev.remoteip[ 1 ] = ( tpcb->remote_ip.addr>>8 )&0xFF ;    //IADDR3
//      lwipdev.remoteip[ 2 ] = ( tpcb->remote_ip.addr>>16 )&0xFF ;    //IADDR2
//      lwipdev.remoteip[ 3 ] = ( tpcb->remote_ip.addr>>24 )&0xFF ;    //IADDR1 
//       tcp_recved( tpcb, p->tot_len ) ;                  //用于获取接收数据
//      pbuf_free( p ) ;                        //释放内存
//      ret_err = ERR_OK ;
//    }
//  }
//  //服务器关闭了
//  else
//  {
//    tcp_recved( tpcb, p->tot_len  );                    //用于获取接收数据
//    es->p=  NULL ;
//    pbuf_free( p ) ;                          //释放内存
//    ret_err = ERR_OK ;
//  }
//  return ret_err ;
//}
//// tcp_err函数的回调函数
//void tcp_server_error( void *arg, err_t err )
//{  
//  LWIP_UNUSED_ARG( err ) ;
//  printf( "tcp error:%x\r\n", ( u32 )arg ) ;
//  //释放内存
//  if( arg!=NULL )
//    mem_free( arg ) ;
//}
////发送数据
//void tcp_server_senddata( struct tcp_pcb *tpcb, struct tcp_server_struct *es )
//{
//  struct pbuf *ptr ;
//  u16 plen ;
//  err_t wr_err = ERR_OK ;
//   while( ( wr_err==ERR_OK )&&( es->p )&&( es->p->len<=tcp_sndbuf( tpcb ) ) )
//   {
//    ptr = es->p ;
//    wr_err = tcp_write( tpcb, ptr->payload, ptr->len, 1 ) ;
//    if( wr_err==ERR_OK )
//    { 
//      plen = ptr->len ;
//      es->p = ptr->next ;                      //指向下一个pbuf
//      //pbuf的ref加一
//      if( es->p )
//        pbuf_ref( es->p ) ;
//      pbuf_free( ptr ) ;
//      tcp_recved( tpcb, plen ) ;                    //更新tcp窗口大小
//    }
//    else if( wr_err==ERR_MEM )
//      es->p = ptr ;
//   }
//}
//// tcp_poll的回调函数
//const u8 *tcp_server_sendbuf = "STM32F103 TCP Server send data\r\n" ;      //TCP服务器发送数据内容
//err_t tcp_server_poll( void *arg, struct tcp_pcb *tpcb )
//{
//  err_t ret_err;
//  struct tcp_server_struct *es ; 
//  es = ( struct tcp_server_struct* )arg ;
//  if( es!=NULL )
//  {
//    //判断是否有数据要发送
//    if( tcp_server_flag&( 1<<7 ) )
//    {
//      es->p = pbuf_alloc( PBUF_TRANSPORT, strlen( ( char* )tcp_server_sendbuf ), PBUF_POOL ) ; 
//      pbuf_take( es->p, ( char* )tcp_server_sendbuf, strlen( ( char* )tcp_server_sendbuf ) ) ;
//      tcp_server_senddata( tpcb, es ) ;                  //轮询的时候发送要发送的数据
//      tcp_server_flag &= ~( 1<<7 ) ;                  //清除数据发送标志位
//      if( es->p!=NULL )
//        pbuf_free( es->p ) ;                    //释放内存  
//    }
//    //关闭操作
//    else if( es->state==ES_TCPSERVER_CLOSING )
//      tcp_server_connection_close( tpcb, es ) ;              //关闭连接
//    ret_err = ERR_OK ;
//  }
//  else
//  {
//    tcp_abort( tpcb ) ;                          //终止连接,删除pcb控制块
//    ret_err = ERR_ABRT ; 
//  }
//  return ret_err ;
//}
//// tcp_sent的回调函数(远端收到ACK信号后发送数据)
//err_t tcp_server_sent( void *arg, struct tcp_pcb *tpcb, u16_t len )
//{
//  struct tcp_server_struct *es ;
//  LWIP_UNUSED_ARG( len ) ;
//  es = ( struct tcp_server_struct * )arg ;
//  if( es->p )
//    tcp_server_senddata( tpcb, es ) ;                    //发送数据
//  return ERR_OK ;
//}
////强制删除主动断开时的time wait
//extern void tcp_pcb_purge( struct tcp_pcb *pcb ) ;                //在 tcp.c里面
//extern struct tcp_pcb *tcp_active_pcbs ;                    //在 tcp.c里面
//extern struct tcp_pcb *tcp_tw_pcbs ;                      //在 tcp.c里面
//void tcp_server_remove_timewait()
//{
//  struct tcp_pcb *pcb, *pcb2 ; 
//  while( tcp_active_pcbs!=NULL )
//  {
//    lwip_periodic_handle() ;                      //继续轮询
//    lwip_pkt_handle() ;
//     delay_ms( 10 ) ;                          //等待tcp_active_pcbs为空  
//  }
//  pcb = tcp_tw_pcbs ;
//  //如果有等待状态的pcbs
//  while( pcb!=NULL )
//  {
//    tcp_pcb_purge( pcb ) ;
//    tcp_tw_pcbs = pcb->next ;
//    pcb2 = pcb ;
//    pcb = pcb->next ;
//    memp_free( MEMP_TCP_PCB, pcb2 ) ;
//  }
//}
//// tcp_accept的回调函数
//err_t tcp_server_accept( void *arg, struct tcp_pcb *newpcb, err_t err )
//{
//  err_t ret_err ;
//  struct tcp_server_struct *es ;
//   LWIP_UNUSED_ARG( arg ) ;
//  LWIP_UNUSED_ARG( err ) ;
//  tcp_setprio( newpcb, TCP_PRIO_MIN ) ;                                //设置新创建的pcb优先级
//  es = ( struct tcp_server_struct* )mem_malloc( sizeof( struct tcp_server_struct ) ) ;  //分配内存
//  //内存分配成功
//   if( es!=NULL )
//  {
//    es->state = ES_TCPSERVER_ACCEPTED ;                //接收连接
//    es->pcb = newpcb ;
//    es->p = NULL ;
//    tcp_arg( newpcb, es ) ;
//    tcp_recv( newpcb, tcp_server_recv ) ;                  //初始化tcp_recv的回调函数
//    tcp_err( newpcb, tcp_server_error ) ;                  //初始化tcp_err回调函数
//    tcp_poll( newpcb, tcp_server_poll, 1 ) ;                //初始化tcp_poll回调函数
//    tcp_sent( newpcb, tcp_server_sent ) ;                  //初始化发送回调函数
//    tcp_server_flag |= 1<<5 ;                      //标记有客户端连上了
//    lwipdev.remoteip[ 0 ] = newpcb->remote_ip.addr&0xFF ;        //IADDR4
//    lwipdev.remoteip[ 1 ] = ( newpcb->remote_ip.addr>>8 )&0xFF ;      //IADDR3
//    lwipdev.remoteip[ 2 ] = ( newpcb->remote_ip.addr>>16 )&0xFF ;      //IADDR2
//    lwipdev.remoteip[ 3 ] = ( newpcb->remote_ip.addr>>24 )&0xFF ;      //IADDR1
//    ret_err = ERR_OK ;
//  }
//  else
//    ret_err = ERR_MEM ;
//  return ret_err ;
//}
//// TCP Server 测试
//void tcp_server_test()
//{
//  err_t err ;  
//  struct tcp_pcb *tcppcbnew ;                        //定义一个TCP服务器控制块
//  struct tcp_pcb *tcppcbconn ;                      //定义一个TCP服务器控制块
//  u8 *tbuf ;
//  u8 res=0 ;
//  u8 connflag=0 ;                            //连接标记
//  tbuf = mymalloc( SRAMIN, 200 ) ;                    //申请内存
//  //内存申请失败了,直接退出
//  if( tbuf==NULL )
//    return ;
//  sprintf( ( char* )tbuf, "Server IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 130, tbuf ) ;                    //服务器IP
//  sprintf( ( char* )tbuf, "Server Port:%d", TCP_SERVER_PORT ) ;
//  LCD_ShowString( 30, 150, tbuf ) ;                    //服务器端口号
//  tcppcbnew = tcp_new() ;                        //创建一个新的pcb
//  //创建成功
//  if( tcppcbnew )
//  {
//    err = tcp_bind( tcppcbnew, IP_ADDR_ANY, TCP_SERVER_PORT ) ;      //将本地IP与指定端口号绑定
//    //绑定完成
//    if( err==ERR_OK )
//    {
//      tcppcbconn = tcp_listen( tcppcbnew ) ;              //设置tcppcb进入监听状态
//      tcp_accept( tcppcbconn, tcp_server_accept ) ;            //初始化tcp_accept的回调函数
//    }
//    else
//      res = 1 ;
//  }
//  else
//    res = 1 ;
//  while( res==0 )
//  {
//    //收到数据
//    if( tcp_server_flag&1<<6 )
//    {
//      tcp_server_flag |= 1<<7 ;                    //标记要发送数据
//      LCD_ShowString( 30, 210, tcp_server_recvbuf ) ;          //显示接收到的数据
//      tcp_server_flag &= ~( 1<<6 ) ;                  //标记数据已经被处理了
//    }
//    //是否连接上
//    if( tcp_server_flag&1<<5 )
//    {
//      if( connflag==0 )
//      { 
//      sprintf( ( char* )tbuf, "Client IP:%d.%d.%d.%d", lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2], lwipdev.remoteip[3] ) ;
//         LCD_ShowString( 30, 170, tbuf ) ;              //客户端IP
//        LCD_ShowString( 30, 190, "Receive Data:" ) ;          //提示消息
//        connflag = 1 ;                        //标记连接了
//      }
//    }
//    else if( connflag )
//      connflag = 0 ;                          //标记连接断开了
//    lwip_periodic_handle() ;
//    lwip_pkt_handle() ;
//    delay_ms( 2 ) ;
//  }   
//  tcp_server_connection_close( tcppcbnew, 0 ) ;                //关闭TCP Server连接
//  tcp_server_connection_close( tcppcbconn, 0 ) ;                //关闭TCP Server连接
//  tcp_server_remove_timewait() ; 
//  memset( tcppcbnew, 0, sizeof( struct tcp_pcb ) ) ;
//  memset( tcppcbconn, 0, sizeof( struct tcp_pcb ) ) ;
//  myfree( SRAMIN, tbuf ) ;
//}

//27.1.2 tcp_server.h代码编写
//#ifndef _TCP_SERVER_DEMO_H_
//#define _TCP_SERVER_DEMO_H_
//#include "sys.h"
//#include "lwip/tcp.h"
//#include "lwip/pbuf.h"
//#define TCP_SERVER_RX_BUFSIZE  2000          //定义tcp server最大接收数据长度
//#define TCP_SERVER_PORT      8088          //定义tcp server的端口
////tcp服务器连接状态
//enum tcp_server_states
//{
//  ES_TCPSERVER_NONE = 0,      //没有连接
//  ES_TCPSERVER_ACCEPTED,      //有客户端连接上了
//  ES_TCPSERVER_CLOSING,      //即将关闭连接
//};
////LWIP回调函数使用的结构体
//struct tcp_server_struct
//{
//  u8 state;              //当前连接状
//  struct tcp_pcb *pcb;        //指向当前的pcb
//  struct pbuf *p;          //指向接收/或传输的pbuf
//}; 
//void tcp_server_test( void ) ;                  //TCP Server测试函数
//#endif

//27.1.3 主函数代码编写
//#include "sys.h"
//#include "delay.h"
//#include "usart1.h"
//#include "tim.h"
//#include "lcd.h"
//#include "malloc.h"
//#include "dm9000.h"
//#include "lwip/netif.h"
//#include "comm.h"
//#include "lwipopts.h"
//#include "tcp_server.h"
//int main()
//{
//  u8 buf[ 30 ];
//   STM32_Clock_Init( 9 ) ;                                        //系统时钟设置
//  SysTick_Init( 72 ) ;                          //延时初始化
//  USART1_Init( 72, 115200 ) ;                      //串口初始化为115200
//  LCD_Init() ;                            //初始化LCD
//  TIM3_Init( 1000, 719 ) ;                        //定时器3频率为100hz
//  my_mem_init( SRAMIN ) ;                      //初始化内部内存池
//  while( lwip_comm_init() ) ;                      //lwip初始化
//  //等待DHCP获取成功/超时溢出
//  while( ( lwipdev.dhcpstatus!=2 )&&( lwipdev.dhcpstatus!=0xFF ) )
//  {
//    lwip_periodic_handle() ;                    //LWIP内核需要定时处理的函数
//    lwip_pkt_handle() ;
//  }
//  POINT_COLOR=RED;
//  LCD_ShowString( 30, 110, "LWIP Init Successed" ) ;
//  //打印动态IP地址
//  if( lwipdev.dhcpstatus==2 )
//    sprintf( ( char* )buf, "DHCP IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  //打印静态IP地址
//  else
//    sprintf( ( char* )buf, "Static IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 130, buf ) ; 
//  //得到网速
//  if( ( DM9000_Get_SpeedAndDuplex()&0x02 )==0x02 )
//    LCD_ShowString( 30, 150, "Ethernet Speed:10M" ) ;
//  else
//    LCD_ShowString( 30, 150, "Ethernet Speed:100M" ) ;
//   while( 1 )
//  {
//    tcp_server_test() ;                        //TCP服务器测试
//    lwip_periodic_handle() ;
//    lwip_pkt_handle() ;
//    delay_ms( 2 ) ;
//  }
//}


//注：将lwip/core目录下的sys.c文件与lwip/include/lwip目录下的sys.h重命名为lwip_sys.c和lwip_sys.h，以免与SYSTEM目录下的sys.c重名，产生编译错误。

//24.5.4 arch目录下源码文件的修改
//（1）arch/cc.h文件代码（该文件主要完成协议使用的数据类型的定义）

//#ifndef _CC_H_
//#define _CC_H_
//#include "cpu.h"
//#include "stdio.h"
////定义与平台无关的数据类型
//typedef unsigned   char    u8_t;                    //无符号8位整数
//typedef signed     char    s8_t;                    //有符号8位整数
//typedef unsigned   short   u16_t;                    //无符号16位整数
//typedef signed     short   s16_t;                    //有符号16位整数
//typedef unsigned   long    u32_t;                    //无符号32位整数
//typedef signed     long    s32_t;                    //有符号32位整数
//typedef u32_t mem_ptr_t ;                        //内存地址型数据
//typedef int sys_prot_t ;                          //临界保护型数据
//#if OS_CRITICAL_METHOD == 1
//#define SYS_ARCH_DECL_PROTECT(lev)
//#define SYS_ARCH_PROTECT(lev)    CPU_INT_DIS()
//#define SYS_ARCH_UNPROTECT(lev)    CPU_INT_EN()
//#endif
////method 3 is used in this port
//#if OS_CRITICAL_METHOD == 3
//#define SYS_ARCH_DECL_PROTECT(lev)  u32_t lev
//#define SYS_ARCH_PROTECT(lev)    lev = OS_CPU_SR_Save()
//#define SYS_ARCH_UNPROTECT(lev)    OS_CPU_SR_Restore(lev)
//#endif
////根据不同的编译器定义一些符号
//#if defined (__ICCARM__)
//#define PACK_STRUCT_BEGIN
//#define PACK_STRUCT_STRUCT 
//#define PACK_STRUCT_END
//#define PACK_STRUCT_FIELD(x) x
//#define PACK_STRUCT_USE_INCLUDES
//#elif defined (__CC_ARM)
//#define PACK_STRUCT_BEGIN __packed
//#define PACK_STRUCT_STRUCT 
//#define PACK_STRUCT_END
//#define PACK_STRUCT_FIELD(x) x
//#elif defined (__GNUC__)
//#define PACK_STRUCT_BEGIN
//#define PACK_STRUCT_STRUCT __attribute__ ((__packed__))
//#define PACK_STRUCT_END
//#define PACK_STRUCT_FIELD(x) x
//#elif defined (__TASKING__)
//#define PACK_STRUCT_BEGIN
//#define PACK_STRUCT_STRUCT
//#define PACK_STRUCT_END
//#define PACK_STRUCT_FIELD(x) x
//#endif
////LWIP用printf调试时使用的一些数据类型
//#define U16_F "4d"
//#define S16_F "4d"
//#define X16_F "4x"
//#define U32_F "8ld"
//#define S32_F "8ld"
//#define X32_F "8lx"
////宏定义
//#ifndef LWIP_PLATFORM_ASSERT
//#define LWIP_PLATFORM_ASSERT(x) \
//    do \
//    {   printf("Assertion \"%s\" failed at line %d in %s\r\n", x, __LINE__, __FILE__); \
//    } while(0)
//#endif
//#ifndef LWIP_PLATFORM_DIAG
//#define LWIP_PLATFORM_DIAG(x) do {printf x;} while(0)
//#endif
//#endif

//（2）arch/cpu.h文件代码（负责定义CPU的大端小端模式）

//#ifndef _CPU_H_
//#define _CPU_H_
//#define BYTE_ORDER LITTLE_ENDIAN                //小端模式
//#endif

//（3）arch/perf.h文件代码（用于系统测量与统计）

//#ifndef _PERF_H_
//#define _PERF_H_
//#define PERF_START                        //空定义
//#define PERF_STOP(x)                        //空定义
//#endif

//（4）arch/sys_arch.h文件代码（为了与操作系统共存使用的获取时间的函数，用于为LWIP提供时钟）

//#ifndef _ARCH_SYS_ARCH_H_
//#define _ARCH_SYS_ARCH_H_
//#include "cc.h"
//u32_t sys_now( void ) ;
//#endif

//（5）arch/sys_arch.c文件代码

//#include "lwip/debug.h"
//#include "lwip/def.h"
//#include "lwip/lwip_sys.h"
//#include "lwip/mem.h"
//#include "tim.h"
////为LWIP提供计时
//extern uint32_t lwip_localtime;//lwip本地时间计数器,单位:ms
//u32_t sys_now()
//{
//  return lwip_localtime ;
//}

//24.5.5 app/comm目录下源码文件的修改
//（1）app/comm.c文件代码

//#include "lwip/tcpip.h" 
//#include "malloc.h"
//#include "delay.h"
//#include "usart1.h"
//__lwip_dev lwipdev ;                          //lwip控制结构体 
//struct netif lwip_netif ;                          //定义一个全局的网络接口
//extern u32 memp_get_memorysize( void ) ;                //在memp.c里面定义
//extern u8_t *memp_memory ;                      //在memp.c里面定义
//extern u8_t *ram_heap ;                        //在mem.c里面定义
//u32 TCPTimer=0 ;                            //TCP查询计时器
//u32 ARPTimer=0 ;                            //ARP查询计时器
//u32 lwip_localtime ;                          //lwip本地时间计数器,单位:ms
//#if LWIP_DHCP
//u32 DHCPfineTimer=0 ;                        //DHCP精细处理计时器
//u32 DHCPcoarseTimer=0 ;                        //DHCP粗糙处理计时器
//#endif
//u8 lwip_comm_mem_malloc()
//{
//  u32 mempsize ;
//  u32 ramheapsize ;
//  mempsize = memp_get_memorysize() ;                //得到memp_memory数组大小
//  memp_memory = mymalloc( SRAMIN, mempsize ) ;          //为memp_memory申请内存
////得到ram heap大小
//  ramheapsize = LWIP_MEM_ALIGN_SIZE( MEM_SIZE )+2*LWIP_MEM_ALIGN_SIZE( 4*3 )+MEM_ALIGNMENT ; 
//  ram_heap = mymalloc( SRAMIN, ramheapsize ) ;            //为ram_heap申请内存
//  //有申请失败的
//  if( !memp_memory||!ram_heap )
//  {
//    lwip_comm_mem_free() ;
//    return 1 ;
//  }
//  return 0 ;
//}
//void lwip_comm_mem_free()
//{   
//  myfree( SRAMIN, memp_memory ) ;
//  myfree( SRAMIN, ram_heap ) ;
//}
//void lwip_comm_default_ip_set( __lwip_dev *lwipx )
//{
//  //默认远端IP为:192.168.1.100
//  lwipx->remoteip[ 0 ] = 192 ;  
//  lwipx->remoteip[ 1 ] = 168 ;
//  lwipx->remoteip[ 2 ] = 1 ;
//  lwipx->remoteip[ 3 ] = 104 ;
//  //MAC地址设置(高三字节固定为:2.0.0,低三字节用STM32唯一ID)
//  lwipx->mac[ 0 ] = dm9000cfg.mac_addr[ 0 ] ;
//  lwipx->mac[ 1 ] = dm9000cfg.mac_addr[ 1 ] ;
//  lwipx->mac[ 2 ] = dm9000cfg.mac_addr[ 2 ] ;
//  lwipx->mac[ 3 ] = dm9000cfg.mac_addr[ 3 ] ;
//  lwipx->mac[ 4 ] = dm9000cfg.mac_addr[ 4 ] ;
//  lwipx->mac[ 5 ] = dm9000cfg.mac_addr[ 5 ] ; 
//  //默认本地IP为:192.168.1.30
//  lwipx->ip[ 0 ] = 192 ;  
//  lwipx->ip[ 1 ] = 168 ;
//  lwipx->ip[ 2 ] = 1 ;
//  lwipx->ip[ 3 ] = 30 ;
//  //默认子网掩码:255.255.255.0
//  lwipx->netmask[ 0 ] = 255 ;  
//  lwipx->netmask[ 1 ] = 255 ;
//  lwipx->netmask[ 2 ] = 255 ;
//  lwipx->netmask[ 3 ] = 0 ;
//  //默认网关:192.168.1.1
//  lwipx->gateway[ 0 ] = 192 ;
//  lwipx->gateway[ 1 ] = 168 ;
//  lwipx->gateway[ 2 ] = 1 ;
//  lwipx->gateway[ 3 ] = 1 ;
//  lwipx->dhcpstatus = 0 ;      //没有DHCP
//}
//u8 lwip_comm_init()
//{
//  struct netif *Netif_Init_Flag ;      //调用netif_add()函数时的返回值,用于判断网络初始化是否成功
//  struct ip_addr ipaddr ;        //ip地址
//  struct ip_addr netmask ;      //子网掩码
//  struct ip_addr gw ;          //默认网关
//  //内存申请失败
//  if( lwip_comm_mem_malloc() )
//    return 1 ;
//  //初始化DM9000AEP
//  if( DM9000_Init() )
//    return 2 ;
//  lwip_init() ;            //初始化LWIP内核
//  lwip_comm_default_ip_set( &lwipdev ) ; //设置默认IP等信息
////使用动态IP
//#if LWIP_DHCP
//  ipaddr.addr = 0 ;
//  netmask.addr = 0 ;
//  gw.addr = 0 ;
////使用静态IP
//#else
//  IP4_ADDR( &ipaddr, lwipdev.ip[ 0 ], lwipdev.ip[ 1 ], lwipdev.ip[ 2 ], lwipdev.ip[ 3 ] ) ;
//  IP4_ADDR( &netmask, lwipdev.netmask[ 0 ], lwipdev.netmask[1] , lwipdev.netmask[ 2 ], lwipdev.netmask[ 3 ] ) ;
//  IP4_ADDR( &gw, lwipdev.gateway[ 0 ], lwipdev.gateway[ 1 ], lwipdev.gateway[ 2 ], lwipdev.gateway[ 3 ] );
//#endif
//  //向网卡列表中添加一个网口
//  Netif_Init_Flag = netif_add( &lwip_netif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input ) ; 
////如果使用DHCP的话
//#if LWIP_DHCP
//  lwipdev.dhcpstatus = 0 ;        //DHCP标记为0
//  dhcp_start( &lwip_netif ) ;        //开启DHCP服务
//#endif
//  //网卡添加失败
//  if( Netif_Init_Flag==NULL )
//    return 3 ;
//  //网口添加成功后,设置netif为默认值,并且打开netif网口
//  else
//  {
//    netif_set_default( &lwip_netif ) ;    //设置netif为默认网口
//    netif_set_up( &lwip_netif ) ;      //打开netif网口
//  }
//  return 0 ;                //操作OK
//}
//void lwip_pkt_handle()
//{
//  ethernetif_input( &lwip_netif ) ;      //从网络缓冲区中读取接收到的数据包并将其发送给LWIP处理
//}
//void lwip_periodic_handle()
//{
//#if LWIP_TCP
//  //每250ms调用一次tcp_tmr()函数
//  if( lwip_localtime-TCPTimer>=TCP_TMR_INTERVAL )
//  {
//    TCPTimer =  lwip_localtime ;
//    tcp_tmr() ;
//  }
//#endif
//  //ARP每5s周期性调用一次
//  if( ( lwip_localtime-ARPTimer )>=ARP_TMR_INTERVAL )
//  {
//    ARPTimer = lwip_localtime ;
//    etharp_tmr() ;
//  }
////如果使用DHCP的话
//#if LWIP_DHCP
//  //每500ms调用一次dhcp_fine_tmr()
//  if( lwip_localtime-DHCPfineTimer>=DHCP_FINE_TIMER_MSECS )
//  {
//    DHCPfineTimer = lwip_localtime ;
//    dhcp_fine_tmr() ;
//    if( ( lwipdev.dhcpstatus!=2 )&&( lwipdev.dhcpstatus!=0xFF ) )
//      lwip_dhcp_process_handle() ;                                //DHCP处理
//  }
//  //每60s执行一次DHCP粗糙处理
//  if( lwip_localtime-DHCPcoarseTimer>=DHCP_COARSE_TIMER_MSECS )
//  {
//    DHCPcoarseTimer = lwip_localtime ;
//    dhcp_coarse_tmr() ;
//  }
//#endif
//}
////如果使能了DHCP
//#if LWIP_DHCP
//void lwip_dhcp_process_handle()
//{
//  u32 ip=0, netmask=0, gw=0 ;
//  switch( lwipdev.dhcpstatus )
//  {
//    //开启DHCP
//    case 0:
//      dhcp_start( &lwip_netif ) ;
//      lwipdev.dhcpstatus = 1 ;            //等待通过DHCP获取到的地址
//      break ;
//    //等待获取到IP地址
//    case 1:
//    {
//      ip = lwip_netif.ip_addr.addr ;          //读取新IP地址
//      netmask = lwip_netif.netmask.addr ;        //读取子网掩码
//      gw = lwip_netif.gw.addr ;            //读取默认网关 
//      //正确获取到IP地址的时候
//      if( ip!=0 )
//      {
//        lwipdev.dhcpstatus = 2 ;          //DHCP成功
//        //解析出通过DHCP获取到的IP地址
//        lwipdev.ip[ 3 ] = ( uint8_t )( ip>>24 ) ; 
//        lwipdev.ip[ 2 ] = ( uint8_t )( ip>>16 ) ;
//        lwipdev.ip[ 1 ] = ( uint8_t )( ip>>8 ) ;
//        lwipdev.ip[ 0 ] = ( uint8_t )( ip ) ;
//        //解析通过DHCP获取到的子网掩码地址
//        lwipdev.netmask[ 3 ] = ( uint8_t )( netmask>>24 ) ;
//        lwipdev.netmask[ 2 ] = ( uint8_t )( netmask>>16 ) ;
//        lwipdev.netmask[ 1 ] = ( uint8_t )( netmask>>8 ) ;
//        lwipdev.netmask[ 0 ] = ( uint8_t )( netmask ) ;
//        //解析出通过DHCP获取到的默认网关
//        lwipdev.gateway[ 3 ] = ( uint8_t )( gw>>24 ) ;
//        lwipdev.gateway[ 2 ] = ( uint8_t )( gw>>16 ) ;
//        lwipdev.gateway[ 1 ] = ( uint8_t )( gw>>8 ) ;
//        lwipdev.gateway[ 0 ] = ( uint8_t )( gw ) ;
//      }
//      //通过DHCP服务获取IP地址失败,且超过最大尝试次数
//      else if( lwip_netif.dhcp->tries>LWIP_MAX_DHCP_TRIES )
//      {
//        lwipdev.dhcpstatus = 0xFF ;          //DHCP超时失败
//        //使用静态IP地址
//        IP4_ADDR( &( lwip_netif.ip_addr ), lwipdev.ip[ 0 ], lwipdev.ip[ 1 ], lwipdev.ip[ 2 ], lwipdev.ip[ 3 ] ) ;
//        IP4_ADDR( &( lwip_netif.netmask ), lwipdev.netmask[ 0 ], lwipdev.netmask[ 1 ], lwipdev.netmask[ 2 ], lwipdev.netmask[ 3 ] ) ;
//        IP4_ADDR( &( lwip_netif.gw ), lwipdev.gateway[ 0 ], lwipdev.gateway[ 1 ], lwipdev.gateway[ 2 ], lwipdev.gateway[ 3 ] ) ;
//      }
//    }
//    break;
//    default :
//    break;
//  }
//}
//#endif

//（2）app/comm.h文件代码

//#ifndef _COMM_H_
//#define _COMM_H_
//#include "dm9000.h"
//#define LWIP_MAX_DHCP_TRIES  4            //DHCP服务器最大重试次数
////lwip控制结构体
//typedef struct  
//{
//  u8 mac[ 6 ] ;                    //MAC地址
//  u8 remoteip[ 4 ] ;                  //远端主机IP地址 
//  u8 ip[ 4 ] ;                    //本机IP地址
//  u8 netmask[ 4 ] ;                  //子网掩码
//  u8 gateway[ 4 ] ;                  //默认网关的IP地址
//  vu8 dhcpstatus ;                  //dhcp状态
//                          //0,未获取DHCP地址
//                          //1,进入DHCP获取状态
//                          //2,成功获取DHCP地址
//                          //0XFF,获取失败
//}__lwip_dev ;
//extern __lwip_dev lwipdev ;                //lwip控制结构体
//void lwip_pkt_handle( void ) ;
//void lwip_periodic_handle( void ) ;
//void lwip_comm_default_ip_set( __lwip_dev *lwipx ) ;
//u8 lwip_comm_mem_malloc( void ) ;
//void lwip_comm_mem_free( void ) ;
//u8 lwip_comm_init( void ) ;
//void lwip_dhcp_process_handle( void ) ;
//#endif

//注：函数中用到了未定义的数据类型，需要在sys.h中添加该类型的定义typedef volatile uint8_t vu8。

//（3）app/lwipopts文件代码

//#ifndef _LWIPOPTS_H_
//#define _LWIPOPTS_H_
//#define SYS_LIGHTWEIGHT_PROT  0
////NO_SYS==1:不使用操作系统
//#define NO_SYS          1      //不使用UCOS操作系统
//#define MEM_ALIGNMENT      4      //使用4字节对齐模式
////heap内存的大小,如果在应用中有大量数据发送的话这个值最好设置大一点
//#define MEM_SIZE        10*1024    //内存堆大小
////memp结构的pbuf数量,如果应用从ROM或者静态存储区发送大量数据时,这个值应该设置大一点
//#define MEMP_NUM_PBUF      10
//#define MEMP_NUM_UDP_PCB    6      //UDP协议控制块(PCB)数量.每个活动UDP"连接"需要一个PCB
//#define MEMP_NUM_TCP_PCB    10      //同时建立激活的TCP数量
//#define MEMP_NUM_TCP_PCB_LISTEN  6    //能够监听的TCP连接数量
//#define MEMP_NUM_TCP_SEG    20      //最多同时在队列中的TCP段数量
//#define MEMP_NUM_SYS_TIMEOUT  5      //能够同时激活的timeout个数
////Pbuf选项
////PBUF_POOL_SIZE:pbuf内存池个数
//#define PBUF_POOL_SIZE    10
////PBUF_POOL_BUFSIZE:每个pbuf内存池大小
//#define PBUF_POOL_BUFSIZE  1500
////TCP选项
//#define LWIP_TCP      1          //为1是使用TCP
//#define TCP_TTL        255        //生存时间
////当TCP的数据段超出队列时的控制位,当设备的内存过小的时候此项应为0
//#define TCP_QUEUE_OOSEQ    0
////最大TCP分段
//#define TCP_MSS        ( 1500-40 )    //TCP_MSS = (MTU - IP报头大小 - TCP报头大小
////TCP发送缓冲区大小(bytes)
//#define TCP_SND_BUF      ( 4*TCP_MSS )
////TCP_SND_QUEUELEN: TCP发送缓冲区大小(pbuf).这个值最小为(2 * TCP_SND_BUF/TCP_MSS) 
//#define TCP_SND_QUEUELEN  ( 4* TCP_SND_BUF/TCP_MSS )
////TCP发送窗口
//#define TCP_WND        ( 2*TCP_MSS )
////ICMP选项
//#define LWIP_ICMP      1        //使用ICMP协议
////DHCP选项
////当使用DHCP时此位应该为1,LwIP 0.5.1版本中没有DHCP服务
//#define LWIP_DHCP      1
////UDP选项
//#define LWIP_UDP      1        //使用UDP服务
//#define UDP_TTL        255        //UDP数据包生存时间
////静态选项
//#define LWIP_STATS      0
//#define LWIP_PROVIDE_ERRNO  1
////SequentialAPI选项
////LWIP_NETCONN==1:使能NETCON函数(要求使用api_lib.c)
//#define LWIP_NETCONN    0
////Socket API选项
////LWIP_SOCKET==1:使能Socket API(要求使用sockets.c)
//#define LWIP_SOCKET      0
//#define LWIP_COMPAT_MUTEX  1
//#define LWIP_SO_RCVTIMEO  1        //通过定义可以避免阻塞线程
////Lwip调试选项
////#define LWIP_DEBUG  1            //开启DEBUG选项
//#define ICMP_DEBUG    LWIP_DBG_OFF    //开启/关闭ICMPdebug
//#endif

//24.5.6 include/netif/ethernetif.h文件修改
//#ifndef _ETHERNETIF_H_
//#define _ETHERNETIF_H_
//#include "lwip/err.h"
//#include "lwip/netif.h"
////网卡的名字
//#define IFNAME0 'e'
//#define IFNAME1 'n'
//err_t ethernetif_init( struct netif *netif ) ;
//err_t ethernetif_input( struct netif *netif ) ;
//#endif

//24.5.7 netif/ethernetif.c文件修改
//#include "netif/ethernetif.h"
//#include "dm9000.h"
//#include "comm.h"
//#include "malloc.h"
//#include "netif/etharp.h"
//#include "string.h"
//static err_t low_level_init( struct netif *netif )
//{
//  netif->hwaddr_len = ETHARP_HWADDR_LEN ;        //设置MAC地址长度,为6个字节
//  //初始化MAC地址,不能与网络中其他设备MAC地址重复
//  netif->hwaddr[ 0 ] = lwipdev.mac[ 0 ] ;
//  netif->hwaddr[ 1 ] = lwipdev.mac[ 1 ] ;
//  netif->hwaddr[ 2 ] = lwipdev.mac[ 2 ] ;
//  netif->hwaddr[ 3 ] = lwipdev.mac[ 3 ] ;
//  netif->hwaddr[ 4 ] = lwipdev.mac[ 4 ] ;
//  netif->hwaddr[ 5 ] = lwipdev.mac[ 5 ] ;
//  netif->mtu = 1500 ;                  //最大允许传输单元,允许该网卡广播和ARP功能
//  netif->flags = NETIF_FLAG_BROADCAST|NETIF_FLAG_ETHARP|NETIF_FLAG_LINK_UP ;
//  return ERR_OK ;
//}
//static err_t low_level_output( struct netif *netif, struct pbuf *p )
//{
//  DM9000_SendPacket( p ) ;
//  return ERR_OK ;
//}
//static struct pbuf *low_level_input( struct netif *netif )
//{
//  struct pbuf *p ;
//  p = DM9000_Receive_Packet() ;
//  return p ;
//}
//err_t ethernetif_input( struct netif *netif )
//{
//  err_t err ;
//  struct pbuf *p ;
//  p = low_level_input( netif ) ;                    //调用low_level_input函数接收数据
//  if( p==NULL )
//    return ERR_MEM ;
//  err = netif->input( p, netif );                                    //调用netif结构体中的input字段(一个函数)来处理数据包
//  if( err!=ERR_OK )
//  {
//    LWIP_DEBUGF( NETIF_DEBUG, ( "ethernetif_input: IP input error\n" ) ) ;
//    pbuf_free( p ) ;
//    p = NULL ;
//  }
//  return err ;
//}
//err_t ethernetif_init( struct netif *netif )
//{
//  LWIP_ASSERT( "netif!=NULL", ( netif!=NULL ) ) ;
////LWIP_NETIF_HOSTNAME
//#if LWIP_NETIF_HOSTNAME
//  netif->hostname = "lwip" ;                    //初始化名称
//#endif
//  netif->name[ 0 ] = IFNAME0 ;                  //初始化变量netif的name字段
//  netif->name[ 1 ] = IFNAME1 ;                  //在文件外定义这里不用关心具体值
//  netif->output = etharp_output ;                  //IP层发送数据包函数
//  netif->linkoutput = low_level_output ;              //ARP模块发送数据包函数
//  low_level_init( netif ) ;                      //底层硬件初始化函数
//  return ERR_OK ;
//}

//24.5.8 其他文件修改
//（1）头文件修改：主要是将lwip/sys.h修改为lwip/lwip_sys.h，因为我们在移植的时候将lwip/core/sys.c和lwip/include/lwip目录下的一个文件名称从sys改为了lwip_sys，所以导致程序引用的头文件也需要修改，需要修改的头文件有：timers.c，init.c，lwip_sys.c，mem.c，pbuf.c和memp.c。

//（2）memp.c文件修改

//①修改memp_memory的定义，之前的定义是位于170行的这么几行代码。

//static u8_t memp_memory[MEM_ALIGNMENT - 1

//#define LWIP_MEMPOOL(name,num,size,desc) + ( (num) * (MEMP_SIZE + MEMP_ALIGN_SIZE(size) ) )

//#include "lwip/memp_std.h"

//];

//我们将这段代码屏蔽掉，重新添加定义u8_t *memp_memory;

//②添加memp_get_memorysize函数

//在333行插入以下函数代码

//u32_t memp_get_memorysize()

//{

//       u32_t length=0;

//       length=(

//                     MEM_ALIGNMENT-1 //全局型数组 为所有POOL分配的内存空间

//                     //MEMP_SIZE表示需要在每个POOL头部预留的空间  MEMP_SIZE = 0

//                     #define LWIP_MEMPOOL(name,num,size,desc)+((num)*(MEMP_SIZE+MEMP_ALIGN_SIZE(size)))

//                     #include "lwip/memp_std.h"

//                     );

//       return length;

//}

//24.6 主函数编写
//经过上面的步骤，我们已经成功移植了LWIP 1.4.1版本，现在我们通过编写主函数来初始化LWIP，让LWIP跑起来。

//（1）添加定时器驱动，我们这里采用通用定时器3来完成LWIP的定时功能。

//在tim.c文件中添加以下代码

//#include "tim.h"
//extern u32 lwip_localtime;                    //lwip本地时间计数器,单位:ms
//void TIM3_IRQHandler()
//{
//  //溢出中断
//  if( TIM3->SR&0x0001 )
//    lwip_localtime +=10 ;                  //加10
//  TIM3->SR &= ~( 1<<0 ) ;                  //清除中断标志位
//}
//void TIM3_Init( u16 arr, u16 psc )
//{
//  RCC->APB1ENR |= 1<<1 ;                  //TIM3时钟使能
//   TIM3->ARR = arr ;                    //设定计数器自动重装值//刚好1ms
//  TIM3->PSC = psc ;                    //预分频器7200,得到10Khz的计数时钟
//  TIM3->DIER |= 1<<0 ;                                        //允许更新中断
//  TIM3->CR1 |= 0x01 ;                    //使能定时器3
//    NVIC_Init( 1, 3, TIM3_IRQn, 2 ) ;                //组2
//}

//在tim.h文件中添加以下代码

//#ifndef _TIM_H_
//#define _TIM_H_
//#include "sys.h"
//void TIM3_Init( u16 arr, u16 psc ) ;                //定时器3初始化
//#endif

//（2）主函数添加以下代码

//#include "sys.h"
//#include "delay.h"
//#include "usart1.h"
//#include "tim.h"
//#include "lcd.h"
//#include "malloc.h"
//#include "dm9000.h"
//#include "lwip/netif.h"
//#include "comm.h"
//#include "lwipopts.h"
//int main()
//{
//  u8 buf[ 30 ];
//   STM32_Clock_Init( 9 ) ;                                        //系统时钟设置
//  SysTick_Init( 72 ) ;                          //延时初始化
//  USART1_Init( 72, 115200 ) ;                      //串口初始化为115200
//  LCD_Init() ;                            //初始化LCD
//  TIM3_Init( 1000, 719 ) ;                        //定时器3频率为100hz
//  my_mem_init( SRAMIN ) ;                      //初始化内部内存池
//  while( lwip_comm_init() ) ;                      //lwip初始化
//  //等待DHCP获取成功/超时溢出
//  while( ( lwipdev.dhcpstatus!=2 )&&( lwipdev.dhcpstatus!=0xFF ) )
//  {
//    lwip_periodic_handle() ;                    //LWIP内核需要定时处理的函数
//    lwip_pkt_handle() ;
//  }
//  POINT_COLOR=RED;
//  LCD_ShowString( 30, 110, "LWIP Init Successed" ) ;
//  //打印动态IP地址
//  if( lwipdev.dhcpstatus==2 )
//    sprintf( ( char* )buf, "DHCP IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  //打印静态IP地址
//  else
//    sprintf( ( char* )buf, "Static IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 130, buf ) ; 
//  //得到网速
//  if( ( DM9000_Get_SpeedAndDuplex()&0x02 )==0x02 )
//    LCD_ShowString( 30, 150, "Ethernet Speed:10M" ) ;
//  else
//    LCD_ShowString( 30, 150, "Ethernet Speed:100M" ) ;
//   while( 1 )
//  {
//    lwip_periodic_handle() ;
//    lwip_pkt_handle() ;
//    delay_ms( 2 ) ;
//  }
//}

//UDP协议概述
//UDP协议是TCP/IP协议栈中传输层协议，是一个简单的面向数据报的协议，在传输层中还有一个TCP协议，UDP不提供数据包分组，组装，无法对数据包进行排序，当报文发送出去之后无法知道是否安全，完整的到达，但是由于UDP不属于连接性协议，所以消耗资源小，处理速度快，通常用于音频，视频和普通数据传输中，UDP数据包结构如下图所示。

//图片

//端口号表示发送和接收进程，UDP使用端口号为不同的应用保留各自的数据传输通道，UDP和TCP都是采用端口号的形式对同一时刻多个应用同时发送和接受数据，而数据接收方则通过目标端口接受数据，有的网络只能使用预先预留或注册的静态端口，而一些网络可以使用没有被注册的动态端口，由于UDP包头使用两个字节存放端口号，所以端口的有效范围0~65535，一般，大于49151的端口号都代表动态端口。

//数据包的长度指的是包括包头和数据部分在内的总字节数，由于包头的长度固定，所以这个区域主要用于计算可变长度的数据部分，数据包的最大长度根据操作环境选择，理论上说，包括包头在内的数据报文最大长度为65535字节。

//UDP通过包头中的校验和来保证数据的完整性，校验和首先在数据发送方通过特殊的算法计算出，传递到接收方之后，需要重新计算，如果某个数据在输出过程中被篡改或某种原因损坏，那么发送方和接收方的校验和就会不一致，因此，UDP协议具有检测报文是否出错的能力。

//udp.c和udp.h这两个文件就是负责实现UDP传输协议的文件，与UDP报文处理有关的函数之间的关系如下图所示。

//图片

//LWIP协议中API编程方式是基于回调机制的，在我们初始化应用的时候必须为内核中不同的事件注册给出对应的回调函数，当对应的事件发生后这些回调函数就会被调用，udp.c中常用的API功能函数如下表所示。

//API函数

//函数功能

//udp_new

//新建一个UDP的PCB块

//udp_remove

//将一个PCB控制块从链表中删除，并释放这个控制块的内存

//udp_bind

//为UDP的PCN控制块绑定一个本地IP地址和端口号

//udp_connect

//连接到指定IP地址主机的指定端口上

//udp_disconnent

//断开连接，将控制块设置为非连接状态

//udp_send

//通过一个PCB控制块发送数据

//udp_recv

//需要创建一个回调函数，当接受到数据的时候被调用

//25.2 应用编写
//在LWIP/app/udp_demo目录下创建udp_demo.c和udp_demo.h文件。

//25.2.1 udp_demo.c代码编写
//#include "udp_demo.h" 
//#include "delay.h"
//#include "usart1.h"
//#include "lcd.h"
//#include "malloc.h"
//#include "string.h"
//#include "comm.h"
//#include "lwip/pbuf.h"
//#include "lwip/udp.h"
//#include "lwip/tcp.h"
////UDP 测试全局状态标记变量
////bit6:数据接收状态
////bit5:连接状态
//u8 udp_demo_flag;
////设置远端IP地址
//void udp_demo_set_remoteip()
//{
//  u8 *tbuf ;
//  LCD_Clear( WHITE ) ;
//  POINT_COLOR = RED ;
//  tbuf = mymalloc( SRAMIN, 100 ) ;                                  //申请内存
//  if( tbuf==NULL )
//    return ;
//  //前三个IP保持和DHCP得到的IP一致
//  lwipdev.remoteip[ 0 ] = lwipdev.ip[ 0 ] ;
//  lwipdev.remoteip[ 1 ] = lwipdev.ip[ 1 ] ;
//  lwipdev.remoteip[ 2 ] = lwipdev.ip[ 2 ] ;
//  lwipdev.remoteip[ 3 ] = 113 ;
//  sprintf( ( char* )tbuf, "Remote IP:%d.%d.%d.", lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2] ) ;
//  LCD_ShowString( 30, 150, tbuf ) ;                //远端IP
//  myfree( SRAMIN, tbuf ) ;
//}
//// UDP接收回调函数
//u8 udp_demo_recvbuf[ UDP_DEMO_RX_BUFSIZE ] ;            //UDP接收数据缓冲区
//void udp_demo_recv( void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip_addr *addr, u16_t port )
//{
//  u32 data_len=0 ;
//  struct pbuf *q ;
//  //接收到不为空的数据时
//  if( p!=NULL )
//  {
//    memset( udp_demo_recvbuf, 0, UDP_DEMO_RX_BUFSIZE ) ;    //数据接收缓冲区清零
//    //遍历完整个pbuf链表
//    for( q=p; q!=NULL; q=q->next )
//    {
//      //拷贝数据
//      if( q->len>( UDP_DEMO_RX_BUFSIZE-data_len ) )
//        memcpy( udp_demo_recvbuf+data_len, q->payload, UDP_DEMO_RX_BUFSIZE-data_len ) ;
//      else
//        memcpy( udp_demo_recvbuf+data_len, q->payload, q->len ) ;
//      data_len += q->len ;
//      //超出TCP客户端接收数组,跳出
//      if( data_len>UDP_DEMO_RX_BUFSIZE )
//        break ;  
//    }
//    upcb->remote_ip = *addr ;                  //记录远程主机的IP地址
//    upcb->remote_port = port ;                //记录远程主机的端口号
//    lwipdev.remoteip[ 0 ] = upcb->remote_ip.addr&0xFF ;      //IADDR4
//    lwipdev.remoteip[ 1 ] = ( upcb->remote_ip.addr>>8 )&0xFF ;  //IADDR3
//    lwipdev.remoteip[ 2 ] = ( upcb->remote_ip.addr>>16 )&0xFF ;  //IADDR2
//    lwipdev.remoteip[ 3 ] = ( upcb->remote_ip.addr>>24 )&0xFF ;  //IADDR1 
//    udp_demo_flag |= 1<<6 ;                  //标记接收到数据了
//    pbuf_free( p ) ;                      //释放内存
//  }
//  else
//  {
//    udp_disconnect( upcb ) ;
//    LCD_Clear( WHITE ) ;                    //清屏
//    udp_demo_flag &= ~( 1<<5 ) ;                //标记连接断开
//  }
//}
//// UDP服务器发送数据
//const u8 *tcp_demo_sendbuf="STM32F103 UDP send data\r\n";
//void udp_demo_senddata( struct udp_pcb *upcb )
//{
//  struct pbuf *ptr ;
//  ptr = pbuf_alloc( PBUF_TRANSPORT, strlen( ( char* )tcp_demo_sendbuf ), PBUF_POOL ) ;//申请内存
//  if( ptr )
//  {
//    ptr->payload = ( void* )tcp_demo_sendbuf ;
//    udp_send( upcb, ptr ) ;                    //udp发送数据 
//    pbuf_free( ptr ) ;                                        //释放内存
//  }
//}
////关闭UDP连接
//void udp_demo_connection_close( struct udp_pcb *upcb )
//{
//  udp_disconnect( upcb ) ;
//  udp_remove( upcb ) ;                      //断开UDP连接 
//  udp_demo_flag &= ~( 1<<5 ) ;                  //标记连接断开
//  LCD_Clear( WHITE ) ;                      //清屏
//}
//// UDP测试
//void udp_demo_test()
//{
//   err_t err ;
//  struct udp_pcb *udppcb ;                    //定义一个TCP服务器控制块
//  struct ip_addr rmtipaddr ;                                      //远端ip地址
//  u8 *tbuf ;
//  u8 res=0 ;
//  udp_demo_set_remoteip() ;                    //先选择IP
//  LCD_Clear( WHITE ) ;                      //清屏
//  tbuf = mymalloc( SRAMIN, 200 ) ;                //申请内存
//  //内存申请失败了,直接退出
//  if( tbuf==NULL )
//    return ;
//  sprintf( ( char* )tbuf, "Local IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 150, tbuf ) ;                //服务器IP
//  sprintf( ( char* )tbuf, "Remote IP:%d.%d.%d.%d", lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2], lwipdev.remoteip[3] ) ;
//  LCD_ShowString( 30, 170, tbuf ) ;                //远端IP
//  sprintf( ( char* )tbuf, "Remote Port:%d", UDP_DEMO_PORT ) ;
//  LCD_ShowString( 30, 190, tbuf ) ;                //客户端端口号
//  LCD_ShowString( 30, 210, "STATUS:Disconnected" ) ;
//  udppcb = udp_new() ;
//  //创建成功
//  if( udppcb )
//  { 
//    IP4_ADDR( &rmtipaddr, lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2], lwipdev.remoteip[3] ) ;
//    err = udp_connect( udppcb, &rmtipaddr, UDP_DEMO_PORT ) ;  //UDP客户端连接到指定IP地址和端口
//    if( err==ERR_OK )
//    {
//      err = udp_bind( udppcb, IP_ADDR_ANY, UDP_DEMO_PORT ) ; //绑定本地IP地址与端口号
//      //绑定完成
//      if( err==ERR_OK )
//      {
//        udp_recv( udppcb, udp_demo_recv, NULL ) ;      //注册接收回调函数
//        LCD_ShowString( 30, 210, "STATUS:Connected   " ) ;  //标记连接上了
//        udp_demo_flag |= 1<<5 ;              //标记已经连接上
//        LCD_ShowString( 30, 230, "Receive Data:" ) ;      //提示消息
//      }
//      else
//        res = 1 ;
//    }
//    else
//      res = 1 ;
//  }
//  else
//    res = 1 ;
//  while( res==0 )
//  {
//    //是否收到数据
//    if( udp_demo_flag&1<<6 )
//    {
//      LCD_ShowString( 30, 250, udp_demo_recvbuf ) ;      //显示接收到的数据
//      udp_demo_senddata( udppcb ) ;              //发送数据
//      udp_demo_flag &= ~( 1<<6 ) ;              //标记数据已经被处理了
//    } 
//    lwip_periodic_handle() ;
//    lwip_pkt_handle() ;
//    delay_ms( 2 ) ;
//  }
//  udp_demo_connection_close( udppcb ) ;
//  myfree( SRAMIN, tbuf ) ;
//}

//25.2.2 udp_demo.h代码编写
//#ifndef _UDP_DEMO_H_
//#define _UDP_DEMO_H_
//#include "sys.h"
//#define UDP_DEMO_RX_BUFSIZE  2000              //定义udp最大接收数据长度 
//#define UDP_DEMO_PORT      8089              //定义udp连接的端口 
//void udp_demo_test( void ) ;                    //UDP测试
//#endif

//25.2.3 主函数代码编写
//#include "sys.h"
//#include "delay.h"
//#include "usart1.h"
//#include "tim.h"
//#include "lcd.h"
//#include "malloc.h"
//#include "dm9000.h"
//#include "lwip/netif.h"
//#include "comm.h"
//#include "lwipopts.h"
//#include "udp_demo.h"
//int main()
//{
//  u8 buf[ 30 ];
//   STM32_Clock_Init( 9 ) ;                        //系统时钟设置
//  SysTick_Init( 72 ) ;                          //延时初始化
//  USART1_Init( 72, 115200 ) ;                      //串口初始化为115200
//  LCD_Init() ;                            //初始化LCD
//  TIM3_Init( 1000, 719 ) ;                        //定时器3频率为100hz
//  my_mem_init( SRAMIN ) ;                      //初始化内部内存池
//  while( lwip_comm_init() ) ;                      //lwip初始化
//  //等待DHCP获取成功/超时溢出
//  while( ( lwipdev.dhcpstatus!=2 )&&( lwipdev.dhcpstatus!=0xFF ) )
//  {
//    lwip_periodic_handle() ;                    //LWIP内核需要定时处理的函数
//    lwip_pkt_handle() ;
//  }
//  POINT_COLOR=RED;
//  LCD_ShowString( 30, 110, "LWIP Init Successed" ) ;
//  //打印动态IP地址
//  if( lwipdev.dhcpstatus==2 )
//    sprintf( ( char* )buf, "DHCP IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  //打印静态IP地址
//  else
//    sprintf( ( char* )buf, "Static IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 130, buf ) ; 
//  //得到网速
//  if( ( DM9000_Get_SpeedAndDuplex()&0x02 )==0x02 )
//    LCD_ShowString( 30, 150, "Ethernet Speed:10M" ) ;
//  else
//    LCD_ShowString( 30, 150, "Ethernet Speed:100M" ) ;
//   while( 1 )
//  {
//    udp_demo_test();
//    lwip_periodic_handle() ;
//    lwip_pkt_handle() ;
//    delay_ms( 2 ) ;
//  }
//}

//TCP协议概述
//TCP是一种面向连接的，可靠地，基于IP的传输层协议，面向连接就意味着两个实用TCP的应用在进行数据交换的时候必须先建立一个TCP连接，当应用层向TCP层发送用于传输的，用8位字节表示的数据流，TCP先把数据流分割成适当长度的报文段，最大传输段的长度MSS通常受计算机连接的网络的数据链路层的最大传输单元MTU控制，之后TCP才把数据包传给IP层，通过它来将数据传送给接收端的TCP层。为了保证报文传输的可靠性，会给每个包一个序号，同时序号也保证了传送到接收端的数据报文能被按照顺序接收，然后接收端对成功接收的报文发回一个响应的确认ACK，如果传送端在合理的时间RTT内没有收到确认，那么对应的数据就会被重传TCP在数据正确性和合法性上采用一个校验和函数来测定数据是否有错误，在发送和接收时都必须计算校验和，在保证可靠性上，对于窗口内未经确认的分组需要重传报文，在拥塞控制上，采用TCP拥塞控制算法。

//TCP数据被封装在一个IP数据报文中，IP数据报文结构如下图所示。

//图片

//TCP报文数据格式在没有选项的情况下，通常是20个字节，数据结构如下图所示

//图片

//（1）源端口号和目的端口号用于寻找发送端和接收端的应用进程，这个与UDP报文相同，这两个值加上IP首部中的源IP地址和目的IP地址唯一确定了一个TCP连接。

//（2）序列号字段用来标识从TCP发送端向TCP接收端发送的数据字节流，用于表示在这个报文段中的第一个数据字节，当建立一个新的连接时，握手报文中的SYN标志置1，这个握手报文中的序号字段为随机选择的初始序号ISN（Initial Sequence Number），当连接建立好以后发送方要发送的第一个字节序号为ISN+1。

//（3）确认号字段只有在ACK为1的时候才有用，确认号中包含发送确认的一方所期望接收到的下一个序号，确认号是在上一次成功接收到的数据字节序列号上加1，例如上次接收成功接收到对方发过来的数据序号为X，那么返回的确认号就应该为X+1

//（4）头部长度又称为首部长度，首部长度中给出了首部的长度，以4个字节为单位，这个字段有4bit，因此TCP最多有60字节的首部，如果没有任何的选项字段，正常的首部长度是20字节，TCP首部中还有6个标志位，这6个标志位如下表所示。

//标志位

//说明

//URG

//置1时表示紧急指针有效

//ACK

//置1时表示确认序号字段有效

//PSH

//置1表示接收方应该尽快将这个报文段交给应用层

//RST

//置1表示重建连接

//SYN

//用于发起连接

//FIN

//发送端完成发送任务，终止连接

//（5）窗口尺寸也就是窗口大小，其中填写相应的值以通知对方期望接收的字节数，窗口大小字段是TCP流量控制的关键字段，窗口大小是一个2个字节的字段，因此窗口大小最大为65535个字节。

//（6）16位校验和和UDP的校验和计算原理相同，这是一个强制性的字段，校验和覆盖整个TCP报文段。

//（7）紧急指针只有在URG置1时有效，是一个正偏移量，和序号字段中的值相加表示紧急数据最后一个字节的序号。

//tcp.c，tcp.h，tcp_in.c和tcp_out.c是LWIP中关于TCP协议的文件，TCP层中函数的关系如下图所示。

//图片

//常用的TCP协议的API函数如下表所示。

//函数类型

//API函数

//功能

//建立TCP连接

//tcp_new()

//创建一个TCP的PCB控制块

//tcp_bind()

//为TCP的PCB控制块绑定一个本地IP地址和端口号

//tcp_listen()

//开始TCP的PCB监听

//tcp_accept()

//控制块accept字段注册的回调函数，侦听到连接时被调用

//tcp_accepted()

//通知LWIP协议栈一个TCP连接被接受了

//tcp_conect()

//连接远端主机

//TCP数据发送

//tcp_write()

//构造一个报文并放在控制块的发送队列缓冲中

//tcp_sent()

//控制块sent字段注册的回调函数，数据发送成功后被回调

//tcp_output()

//将发送缓冲队列中的数据发送出去

//TCP数据接收

//tcp_recv()

//控制块recv字段注册的回调函数，当接收到新数据时被调用

//tcp_recved()

//当程序处理完数据后一定要调用这个函数，通知内核更新接收窗口

//数据轮询

//tcp_poll()

//控制块poll字段注册的回调函数，该函数周期性调用

//关闭和终止连接

//tcp_close()

//关闭TCP连接

//tcp_err()

//控制块err字段注册的回调函数，遇到错误时被调用

//tcp_abort()

//中断TCP连接

//26.2 应用编写
//26.2.1 tcp_client.c代码编写
//#include "tcp_client.h"
//#include "delay.h"
//#include "usart1.h"
//#include "lcd.h"
//#include "malloc.h"
//#include "string.h"
//#include "comm.h"
//#include "lwip/debug.h"
//#include "lwip/stats.h"
//#include "lwip/memp.h"
//#include "lwip/mem.h"
////TCP Client 测试全局状态标记变量
////bit7:0,没有数据要发送;1,有数据要发送
////bit6:0,没有收到数据;1,收到数据了
////bit5:0,没有连接上服务器;1,连接上服务器了
////bit4~0:保留
//u8 tcp_client_flag;   
////设置远端IP地址
//void tcp_client_set_remoteip()
//{
//  u8 *tbuf;
//  tbuf=mymalloc( SRAMIN, 100 ) ;                      //申请内存
//  if( tbuf==NULL )
//    return ;
//  //前三个IP保持和DHCP得到的IP一致
//  lwipdev.remoteip[ 0 ] = lwipdev.ip[ 0 ] ;
//  lwipdev.remoteip[ 1 ] = lwipdev.ip[ 1 ] ;
//  lwipdev.remoteip[ 2 ] = lwipdev.ip[ 2 ] ;
//  lwipdev.remoteip[ 3 ] = 113 ;
//  sprintf( ( char* )tbuf, "Remote IP:%d.%d.%d.", lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2] ) ;
//  LCD_ShowString( 30, 150, tbuf ) ;                    //远端IP
//  myfree( SRAMIN, tbuf ) ;
//}
////关闭与服务器的连接
//void tcp_client_connection_close( struct tcp_pcb *tpcb, struct tcp_client_struct *es )
//{
//  tcp_abort( tpcb ) ;                            //终止连接,删除pcb控制块
//  tcp_arg( tpcb, NULL ) ;
//  tcp_recv( tpcb, NULL ) ;
//  tcp_sent( tpcb, NULL ) ;
//  tcp_err( tpcb, NULL ) ;
//  tcp_poll( tpcb, NULL, 0 );
//  if( es )
//    mem_free( es ) ;
//  tcp_client_flag &= ~( 1<<5 ) ;                      //标记连接断开了
//}
//// tcp_recv函数的回调函数
//u8 tcp_client_recvbuf[ TCP_CLIENT_RX_BUFSIZE ] ;                //接收数据缓冲区
//err_t tcp_client_recv( void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err )
//{
//  u32 data_len=0 ;
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
//// tcp_err函数的回调函数
//void tcp_client_error( void *arg, err_t err )
//{

//}
////发送数据
//void tcp_client_senddata( struct tcp_pcb *tpcb, struct tcp_client_struct *es )
//{
//  struct pbuf *ptr ; 
//   err_t wr_err = ERR_OK ;
//  while( ( wr_err==ERR_OK )&&( es->p )&&( es->p->len<=tcp_sndbuf( tpcb ) ) )
//  {
//    ptr = es->p ;
//    wr_err = tcp_write( tpcb, ptr->payload, ptr->len, 1 ) ;          //数据加入到发送缓冲队列中
//    if( wr_err==ERR_OK )
//    {
//      es->p = ptr->next ;                      //指向下一个pbuf
//      //pbuf的ref加一
//      if( es->p )
//        pbuf_ref( es->p );
//      pbuf_free( ptr ) ;                        //释放ptr
//    }
//    else if( wr_err==ERR_MEM )
//      es->p = ptr ;
//    tcp_output( tpcb ) ;                        //发送缓冲队列中的数据发送
//  }
//}
//// tcp_sent的回调函数(从远端接收到ACK后发送数据)
//err_t tcp_client_sent( void *arg, struct tcp_pcb *tpcb, u16_t len )
//{
//  struct tcp_client_struct *es ;
//  LWIP_UNUSED_ARG( len ) ;
//  es = ( struct tcp_client_struct* )arg ;
//  if( es->p )
//    tcp_client_senddata( tpcb, es ) ;                    //发送数据
//  return ERR_OK ;
//}
//// tcp_poll的回调函数
//const u8 *tcp_client_sendbuf = "STM32F103 TCP Client send data\r\n" ;      //TCP服务器发送数据内容
//err_t tcp_client_poll( void *arg, struct tcp_pcb *tpcb )
//{
//  err_t ret_err ;
//  struct tcp_client_struct *es ; 
//  es = ( struct tcp_client_struct* )arg ;
//  //连接处于空闲可以发送数据
//  if( es!=NULL )
//  {
//    //判断是否有数据要发送
//    if( tcp_client_flag&( 1<<7 ) )
//    {
//      es->p = pbuf_alloc( PBUF_TRANSPORT, strlen( ( char* )tcp_client_sendbuf ), PBUF_POOL ) ; 
//      pbuf_take( es->p, ( char* )tcp_client_sendbuf, strlen( ( char* )tcp_client_sendbuf ) ) ; 
//      tcp_client_senddata( tpcb, es ) ;                  //将数据发送出去
//      tcp_client_flag &= ~( 1<<7 ) ;                  //清除数据发送标志
//      //释放内存
//      if( es->p )
//        pbuf_free( es->p ) ;
//    }
//    else if( es->state==ES_TCPCLIENT_CLOSING )
//       tcp_client_connection_close( tpcb, es ) ;              //关闭TCP连接
//    ret_err = ERR_OK ;
//  }
//  else
//  { 
//    tcp_abort( tpcb ) ;                          //终止连接,删除pcb控制块
//    ret_err = ERR_ABRT ;
//  }
//  return ret_err ;
//}
////连接建立后调用回调函数
//err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
//{
//  struct tcp_client_struct *es=NULL;  
//  if(err==ERR_OK)   
//  {
//    es = ( struct tcp_client_struct* )mem_malloc( sizeof( struct tcp_client_struct ) ) ; //申请内存
//    //内存申请成功
//    if( es )
//    {
//       es->state = ES_TCPCLIENT_CONNECTED ;            //状态为连接成功
//      es->pcb = tpcb ;
//      es->p = NULL ;
//      tcp_arg( tpcb, es ) ;                      //更新tpcb的callback_arg
//      tcp_recv( tpcb, tcp_client_recv ) ;                  //初始化tcp_recv回调功能
//      tcp_err( tpcb, tcp_client_error ) ;                  //初始化tcp_err()回调函数
//      tcp_sent( tpcb, tcp_client_sent ) ;                  //初始化tcp_sent回调功能
//      tcp_poll( tpcb, tcp_client_poll, 1 ) ;                //初始化tcp_poll回调功能
//       tcp_client_flag |= 1<<5 ;                    //标记连接到服务器了
//      err = ERR_OK ;
//    }
//    else
//    {
//      tcp_client_connection_close( tpcb, es ) ;              //关闭连接
//      err = ERR_MEM ;                        //返回内存分配错误
//    }
//  }
//  else
//    tcp_client_connection_close( tpcb, 0 ) ;                //关闭连接
//  return err ;
//}
////客户机测试
//void tcp_client_test()
//{
//   struct tcp_pcb *tcppcb ;                        //定义一个TCP服务器控制块
//  struct ip_addr rmtipaddr ;                        //远端ip地址
//  u8 *tbuf ;
//  u8 res=0 ;    
//  u8 t=0 ; 
//  u8 connflag=0 ;                            //连接标记
//  tcp_client_set_remoteip() ;                        //先选择IP
//  tbuf = mymalloc( SRAMIN, 200 ) ;                    //申请内存
//  //内存申请失败了,直接退出
//  if( tbuf==NULL )
//    return ;
//  sprintf( ( char* )tbuf, "Local IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 130, tbuf ) ;                    //服务器IP
//  sprintf( ( char* )tbuf, "Remote IP:%d.%d.%d.%d", lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2], lwipdev.remoteip[3] ) ;
//  LCD_ShowString( 30, 150, tbuf ) ;                    //远端IP
//  sprintf( ( char* )tbuf, "Remote Port:%d", TCP_CLIENT_PORT ) ;          //客户端端口号
//  LCD_ShowString( 30, 170, tbuf ) ;
//  LCD_ShowString( 30, 190, "STATUS:Disconnected" ) ;
//  tcppcb = tcp_new() ;                          //创建一个新的pcb
//  //创建成功
//  if( tcppcb )
//  {
//  IP4_ADDR( &rmtipaddr, lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2], lwipdev.remoteip[3] ) ;
//    tcp_connect( tcppcb, &rmtipaddr, TCP_CLIENT_PORT, tcp_client_connected ) ; 
//   }
//  else
//    res = 1 ;
//  while( res==0 )
//  {
//    //是否收到数据
//    if( tcp_client_flag&1<<6 )
//    {
//      LCD_ShowString( 30, 230, tcp_client_recvbuf ) ;            //显示接收到的数据
//      tcp_client_flag |= 1<<7 ;                    //标记要发送数据
//      tcp_client_flag &= ~( 1<<6 ) ;                  //标记数据已经被处理了
//    }
//    //是否连接上
//    if( tcp_client_flag&1<<5 )
//    {
//      if( connflag==0 )
//      { 
//        LCD_ShowString( 30, 190, "STATUS:Connected   " ) ;
//        LCD_ShowString( 30, 210, "Receive Data:" ) ;
//        connflag = 1 ;                        //标记连接了
//      }
//    }
//    else if( connflag )
//    {
//       LCD_ShowString( 30, 190, "STATUS:Disconnected" ) ;
//      connflag = 0 ;                          //标记连接断开了
//    } 
//    lwip_periodic_handle() ;
//    lwip_pkt_handle() ;
//    delay_ms( 2 ) ;
//    t ++ ;
//    if( t==200 )
//    {
//      //未连接上,则尝试重连
//      if( ( connflag==0 )&&( ( tcp_client_flag&1<<5 )==0 ) )
//      { 
//        tcp_client_connection_close( tcppcb, 0 ) ;            //关闭连接
//        tcppcb = tcp_new() ;                    //创建一个新的pcb
//        //创建成功
//        if( tcppcb )
//          tcp_connect( tcppcb, &rmtipaddr, TCP_CLIENT_PORT, tcp_client_connected ) ; 
//      }
//      t = 0 ;
//    }    
//  }
//  tcp_client_connection_close( tcppcb, 0 ) ;                  //关闭TCP Client连接
//  myfree( SRAMIN, tbuf ) ;
//}

//26.2.2 tcp_client.h代码编写
//#ifndef _TCP_CLIENT_H_
//#define _TCP_CLIENT_H_
//#include "sys.h"
//#include "lwip/tcp.h"
//#include "lwip/pbuf.h"
//#define TCP_CLIENT_RX_BUFSIZE  1500            //最大接收数据长度
//#define TCP_CLIENT_TX_BUFSIZE  200              //最大发送数据长度
//#define LWIP_SEND_DATA      0x80            //有数据发送
//#define  TCP_CLIENT_PORT    8087            //远端端口
////tcp服务器连接状态
//enum tcp_client_states
//{
//  ES_TCPCLIENT_NONE = 0,    //没有连接
//  ES_TCPCLIENT_CONNECTED,  //连接到服务器了 
//  ES_TCPCLIENT_CLOSING,    //关闭连接
//};
////LWIP回调函数使用的结构体
//struct tcp_client_struct
//{
//  u8 state;            //当前连接状
//  struct tcp_pcb *pcb;      //指向当前的pcb
//  struct pbuf *p;        //指向接收/或传输的pbuf
//};  
//void tcp_client_test( void ) ;                    //TCP Client测试函数
//#endif

//26.2.3 主函数代码编写
//#include "sys.h"
//#include "delay.h"
//#include "usart1.h"
//#include "tim.h"
//#include "lcd.h"
//#include "malloc.h"
//#include "dm9000.h"
//#include "lwip/netif.h"
//#include "comm.h"
//#include "lwipopts.h"
//#include "tcp_client.h"
//int main()
//{
//  u8 buf[ 30 ];
//   STM32_Clock_Init( 9 ) ;                        //系统时钟设置
//  SysTick_Init( 72 ) ;                          //延时初始化
//  USART1_Init( 72, 115200 ) ;                      //串口初始化为115200
//  LCD_Init() ;                            //初始化LCD
//  TIM3_Init( 1000, 719 ) ;                        //定时器3频率为100hz
//  my_mem_init( SRAMIN ) ;                      //初始化内部内存池
//  while( lwip_comm_init() ) ;                      //lwip初始化
//  //等待DHCP获取成功/超时溢出
//  while( ( lwipdev.dhcpstatus!=2 )&&( lwipdev.dhcpstatus!=0xFF ) )
//  {
//    lwip_periodic_handle() ;                    //LWIP内核需要定时处理的函数
//    lwip_pkt_handle() ;
//  }
//  POINT_COLOR=RED;
//  LCD_ShowString( 30, 110, "LWIP Init Successed" ) ;
//  //打印动态IP地址
//  if( lwipdev.dhcpstatus==2 )
//    sprintf( ( char* )buf, "DHCP IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  //打印静态IP地址
//  else
//    sprintf( ( char* )buf, "Static IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 130, buf ) ; 
//  //得到网速
//  if( ( DM9000_Get_SpeedAndDuplex()&0x02 )==0x02 )
//    LCD_ShowString( 30, 150, "Ethernet Speed:10M" ) ;
//  else
//    LCD_ShowString( 30, 150, "Ethernet Speed:100M" ) ;
//   while( 1 )
//  {
//    tcp_client_test() ;
//    lwip_periodic_handle() ;
//    lwip_pkt_handle() ;
//    delay_ms( 2 ) ;
//  }
//}


//STM32学习笔记27—有线网络通信实验4之TCP服务器
//原创 电子技术园地 滑小稽笔记 2021-04-01 00:00
//有关于TCP协议的知识在上一章已经有过描述，这里我们直接使用API来实现TCP服务器模式。

//27.1 实验例程
//27.1.1 tcp_server.c代码编写
//#include "tcp_server.h"
//#include "delay.h"
//#include "usart1.h"
//#include "lcd.h"
//#include "malloc.h"
//#include "string.h"
//#include "lwip/debug.h"
//#include "lwip/stats.h"
//#include "lwip/memp.h"
//#include "lwip/mem.h"
//#include "comm.h"
////TCP Server 测试全局状态标记变量
////bit7:0,没有数据要发送;1,有数据要发送
////bit6:0,没有收到数据;1,收到数据了.
////bit5:0,没有客户端连接上;1,有客户端连接上了.
////bit4~0:保留
//u8 tcp_server_flag;
////关闭tcp连接
//void tcp_server_connection_close( struct tcp_pcb *tpcb, struct tcp_server_struct *es )
//{
//  tcp_close( tpcb ) ;
//  tcp_arg( tpcb, NULL ) ;
//  tcp_sent( tpcb, NULL ) ;
//  tcp_recv( tpcb, NULL ) ;
//  tcp_err( tpcb, NULL ) ;
//  tcp_poll( tpcb, NULL, 0 ) ;
//  if( es )
//    mem_free( es ) ; 
//  tcp_server_flag &= ~( 1<<5 ) ;                      //标记连接断开了
//}
//// tcp_recv函数的回调函数
//u8 tcp_server_recvbuf[ TCP_SERVER_RX_BUFSIZE ] ;                //TCP Server接收数据缓冲区
//err_t tcp_server_recv( void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err )
//{
//  err_t ret_err ;
//  u32 data_len = 0 ;
//  struct pbuf *q ;
//    struct tcp_server_struct *es ;
//  LWIP_ASSERT( "arg != NULL", arg != NULL ) ;
//  es = ( struct tcp_server_struct* )arg ;
//  //从客户端接收到空数据
//  if( p==NULL )
//  {
//    es->state = ES_TCPSERVER_CLOSING ;                //需要关闭TCP连接了
//    es->p = p ; 
//    ret_err = ERR_OK ;
//  }
//  //从客户端接收到一个非空数据,但是由于某种原因err!=ERR_OK
//  else if( err!=ERR_OK )
//  {
//    if( p )
//      pbuf_free( p ) ;                        //释放接收pbuf
//    ret_err = err ;
//  }
//  //处于连接状态
//  else if( es->state==ES_TCPSERVER_ACCEPTED )
//  {
//    //当处于连接状态并且接收到的数据不为空时将其打印出来
//    if( p!=NULL )
//    {
//      memset( tcp_server_recvbuf, 0, TCP_SERVER_RX_BUFSIZE ) ;      //数据接收缓冲区清零
//      //遍历完整个pbuf链表
//      for( q=p; q!=NULL; q=q->next )
//      {
//        if( q->len>( TCP_SERVER_RX_BUFSIZE-data_len ) )
//          memcpy( tcp_server_recvbuf+data_len, q->payload, TCP_SERVER_RX_BUFSIZE-data_len ) ; 
//        else
//          memcpy(tcp_server_recvbuf+data_len, q->payload, q->len ) ;
//        data_len += q->len ;
//        //超出TCP客户端接收数组,跳出
//        if( data_len>TCP_SERVER_RX_BUFSIZE )
//          break ;
//      }
//      tcp_server_flag |= 1<<6 ;                    //标记接收到数据了
//      lwipdev.remoteip[ 0 ] = tpcb->remote_ip.addr&0xFF ;        //IADDR4
//      lwipdev.remoteip[ 1 ] = ( tpcb->remote_ip.addr>>8 )&0xFF ;    //IADDR3
//      lwipdev.remoteip[ 2 ] = ( tpcb->remote_ip.addr>>16 )&0xFF ;    //IADDR2
//      lwipdev.remoteip[ 3 ] = ( tpcb->remote_ip.addr>>24 )&0xFF ;    //IADDR1 
//       tcp_recved( tpcb, p->tot_len ) ;                  //用于获取接收数据
//      pbuf_free( p ) ;                        //释放内存
//      ret_err = ERR_OK ;
//    }
//  }
//  //服务器关闭了
//  else
//  {
//    tcp_recved( tpcb, p->tot_len  );                    //用于获取接收数据
//    es->p=  NULL ;
//    pbuf_free( p ) ;                          //释放内存
//    ret_err = ERR_OK ;
//  }
//  return ret_err ;
//}
//// tcp_err函数的回调函数
//void tcp_server_error( void *arg, err_t err )
//{  
//  LWIP_UNUSED_ARG( err ) ;
//  printf( "tcp error:%x\r\n", ( u32 )arg ) ;
//  //释放内存
//  if( arg!=NULL )
//    mem_free( arg ) ;
//}
////发送数据
//void tcp_server_senddata( struct tcp_pcb *tpcb, struct tcp_server_struct *es )
//{
//  struct pbuf *ptr ;
//  u16 plen ;
//  err_t wr_err = ERR_OK ;
//   while( ( wr_err==ERR_OK )&&( es->p )&&( es->p->len<=tcp_sndbuf( tpcb ) ) )
//   {
//    ptr = es->p ;
//    wr_err = tcp_write( tpcb, ptr->payload, ptr->len, 1 ) ;
//    if( wr_err==ERR_OK )
//    { 
//      plen = ptr->len ;
//      es->p = ptr->next ;                      //指向下一个pbuf
//      //pbuf的ref加一
//      if( es->p )
//        pbuf_ref( es->p ) ;
//      pbuf_free( ptr ) ;
//      tcp_recved( tpcb, plen ) ;                    //更新tcp窗口大小
//    }
//    else if( wr_err==ERR_MEM )
//      es->p = ptr ;
//   }
//}
//// tcp_poll的回调函数
//const u8 *tcp_server_sendbuf = "STM32F103 TCP Server send data\r\n" ;      //TCP服务器发送数据内容
//err_t tcp_server_poll( void *arg, struct tcp_pcb *tpcb )
//{
//  err_t ret_err;
//  struct tcp_server_struct *es ; 
//  es = ( struct tcp_server_struct* )arg ;
//  if( es!=NULL )
//  {
//    //判断是否有数据要发送
//    if( tcp_server_flag&( 1<<7 ) )
//    {
//      es->p = pbuf_alloc( PBUF_TRANSPORT, strlen( ( char* )tcp_server_sendbuf ), PBUF_POOL ) ; 
//      pbuf_take( es->p, ( char* )tcp_server_sendbuf, strlen( ( char* )tcp_server_sendbuf ) ) ;
//      tcp_server_senddata( tpcb, es ) ;                  //轮询的时候发送要发送的数据
//      tcp_server_flag &= ~( 1<<7 ) ;                  //清除数据发送标志位
//      if( es->p!=NULL )
//        pbuf_free( es->p ) ;                    //释放内存  
//    }
//    //关闭操作
//    else if( es->state==ES_TCPSERVER_CLOSING )
//      tcp_server_connection_close( tpcb, es ) ;              //关闭连接
//    ret_err = ERR_OK ;
//  }
//  else
//  {
//    tcp_abort( tpcb ) ;                          //终止连接,删除pcb控制块
//    ret_err = ERR_ABRT ; 
//  }
//  return ret_err ;
//}
//// tcp_sent的回调函数(远端收到ACK信号后发送数据)
//err_t tcp_server_sent( void *arg, struct tcp_pcb *tpcb, u16_t len )
//{
//  struct tcp_server_struct *es ;
//  LWIP_UNUSED_ARG( len ) ;
//  es = ( struct tcp_server_struct * )arg ;
//  if( es->p )
//    tcp_server_senddata( tpcb, es ) ;                    //发送数据
//  return ERR_OK ;
//}
////强制删除主动断开时的time wait
//extern void tcp_pcb_purge( struct tcp_pcb *pcb ) ;                //在 tcp.c里面
//extern struct tcp_pcb *tcp_active_pcbs ;                    //在 tcp.c里面
//extern struct tcp_pcb *tcp_tw_pcbs ;                      //在 tcp.c里面
//void tcp_server_remove_timewait()
//{
//  struct tcp_pcb *pcb, *pcb2 ; 
//  while( tcp_active_pcbs!=NULL )
//  {
//    lwip_periodic_handle() ;                      //继续轮询
//    lwip_pkt_handle() ;
//     delay_ms( 10 ) ;                          //等待tcp_active_pcbs为空  
//  }
//  pcb = tcp_tw_pcbs ;
//  //如果有等待状态的pcbs
//  while( pcb!=NULL )
//  {
//    tcp_pcb_purge( pcb ) ;
//    tcp_tw_pcbs = pcb->next ;
//    pcb2 = pcb ;
//    pcb = pcb->next ;
//    memp_free( MEMP_TCP_PCB, pcb2 ) ;
//  }
//}
//// tcp_accept的回调函数
//err_t tcp_server_accept( void *arg, struct tcp_pcb *newpcb, err_t err )
//{
//  err_t ret_err ;
//  struct tcp_server_struct *es ;
//   LWIP_UNUSED_ARG( arg ) ;
//  LWIP_UNUSED_ARG( err ) ;
//  tcp_setprio( newpcb, TCP_PRIO_MIN ) ;                                //设置新创建的pcb优先级
//  es = ( struct tcp_server_struct* )mem_malloc( sizeof( struct tcp_server_struct ) ) ;  //分配内存
//  //内存分配成功
//   if( es!=NULL )
//  {
//    es->state = ES_TCPSERVER_ACCEPTED ;                //接收连接
//    es->pcb = newpcb ;
//    es->p = NULL ;
//    tcp_arg( newpcb, es ) ;
//    tcp_recv( newpcb, tcp_server_recv ) ;                  //初始化tcp_recv的回调函数
//    tcp_err( newpcb, tcp_server_error ) ;                  //初始化tcp_err回调函数
//    tcp_poll( newpcb, tcp_server_poll, 1 ) ;                //初始化tcp_poll回调函数
//    tcp_sent( newpcb, tcp_server_sent ) ;                  //初始化发送回调函数
//    tcp_server_flag |= 1<<5 ;                      //标记有客户端连上了
//    lwipdev.remoteip[ 0 ] = newpcb->remote_ip.addr&0xFF ;        //IADDR4
//    lwipdev.remoteip[ 1 ] = ( newpcb->remote_ip.addr>>8 )&0xFF ;      //IADDR3
//    lwipdev.remoteip[ 2 ] = ( newpcb->remote_ip.addr>>16 )&0xFF ;      //IADDR2
//    lwipdev.remoteip[ 3 ] = ( newpcb->remote_ip.addr>>24 )&0xFF ;      //IADDR1
//    ret_err = ERR_OK ;
//  }
//  else
//    ret_err = ERR_MEM ;
//  return ret_err ;
//}
//// TCP Server 测试
//void tcp_server_test()
//{
//  err_t err ;  
//  struct tcp_pcb *tcppcbnew ;                        //定义一个TCP服务器控制块
//  struct tcp_pcb *tcppcbconn ;                      //定义一个TCP服务器控制块
//  u8 *tbuf ;
//  u8 res=0 ;
//  u8 connflag=0 ;                            //连接标记
//  tbuf = mymalloc( SRAMIN, 200 ) ;                    //申请内存
//  //内存申请失败了,直接退出
//  if( tbuf==NULL )
//    return ;
//  sprintf( ( char* )tbuf, "Server IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 130, tbuf ) ;                    //服务器IP
//  sprintf( ( char* )tbuf, "Server Port:%d", TCP_SERVER_PORT ) ;
//  LCD_ShowString( 30, 150, tbuf ) ;                    //服务器端口号
//  tcppcbnew = tcp_new() ;                        //创建一个新的pcb
//  //创建成功
//  if( tcppcbnew )
//  {
//    err = tcp_bind( tcppcbnew, IP_ADDR_ANY, TCP_SERVER_PORT ) ;      //将本地IP与指定端口号绑定
//    //绑定完成
//    if( err==ERR_OK )
//    {
//      tcppcbconn = tcp_listen( tcppcbnew ) ;              //设置tcppcb进入监听状态
//      tcp_accept( tcppcbconn, tcp_server_accept ) ;            //初始化tcp_accept的回调函数
//    }
//    else
//      res = 1 ;
//  }
//  else
//    res = 1 ;
//  while( res==0 )
//  {
//    //收到数据
//    if( tcp_server_flag&1<<6 )
//    {
//      tcp_server_flag |= 1<<7 ;                    //标记要发送数据
//      LCD_ShowString( 30, 210, tcp_server_recvbuf ) ;          //显示接收到的数据
//      tcp_server_flag &= ~( 1<<6 ) ;                  //标记数据已经被处理了
//    }
//    //是否连接上
//    if( tcp_server_flag&1<<5 )
//    {
//      if( connflag==0 )
//      { 
//      sprintf( ( char* )tbuf, "Client IP:%d.%d.%d.%d", lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2], lwipdev.remoteip[3] ) ;
//         LCD_ShowString( 30, 170, tbuf ) ;              //客户端IP
//        LCD_ShowString( 30, 190, "Receive Data:" ) ;          //提示消息
//        connflag = 1 ;                        //标记连接了
//      }
//    }
//    else if( connflag )
//      connflag = 0 ;                          //标记连接断开了
//    lwip_periodic_handle() ;
//    lwip_pkt_handle() ;
//    delay_ms( 2 ) ;
//  }   
//  tcp_server_connection_close( tcppcbnew, 0 ) ;                //关闭TCP Server连接
//  tcp_server_connection_close( tcppcbconn, 0 ) ;                //关闭TCP Server连接
//  tcp_server_remove_timewait() ; 
//  memset( tcppcbnew, 0, sizeof( struct tcp_pcb ) ) ;
//  memset( tcppcbconn, 0, sizeof( struct tcp_pcb ) ) ;
//  myfree( SRAMIN, tbuf ) ;
//}

//27.1.2 tcp_server.h代码编写
//#ifndef _TCP_SERVER_DEMO_H_
//#define _TCP_SERVER_DEMO_H_
//#include "sys.h"
//#include "lwip/tcp.h"
//#include "lwip/pbuf.h"
//#define TCP_SERVER_RX_BUFSIZE  2000          //定义tcp server最大接收数据长度
//#define TCP_SERVER_PORT      8088          //定义tcp server的端口
////tcp服务器连接状态
//enum tcp_server_states
//{
//  ES_TCPSERVER_NONE = 0,      //没有连接
//  ES_TCPSERVER_ACCEPTED,      //有客户端连接上了
//  ES_TCPSERVER_CLOSING,      //即将关闭连接
//};
////LWIP回调函数使用的结构体
//struct tcp_server_struct
//{
//  u8 state;              //当前连接状
//  struct tcp_pcb *pcb;        //指向当前的pcb
//  struct pbuf *p;          //指向接收/或传输的pbuf
//}; 
//void tcp_server_test( void ) ;                  //TCP Server测试函数
//#endif

//27.1.3 主函数代码编写
//#include "sys.h"
//#include "delay.h"
//#include "usart1.h"
//#include "tim.h"
//#include "lcd.h"
//#include "malloc.h"
//#include "dm9000.h"
//#include "lwip/netif.h"
//#include "comm.h"
//#include "lwipopts.h"
//#include "tcp_server.h"
//int main()
//{
//  u8 buf[ 30 ];
//   STM32_Clock_Init( 9 ) ;                                        //系统时钟设置
//  SysTick_Init( 72 ) ;                          //延时初始化
//  USART1_Init( 72, 115200 ) ;                      //串口初始化为115200
//  LCD_Init() ;                            //初始化LCD
//  TIM3_Init( 1000, 719 ) ;                        //定时器3频率为100hz
//  my_mem_init( SRAMIN ) ;                      //初始化内部内存池
//  while( lwip_comm_init() ) ;                      //lwip初始化
//  //等待DHCP获取成功/超时溢出
//  while( ( lwipdev.dhcpstatus!=2 )&&( lwipdev.dhcpstatus!=0xFF ) )
//  {
//    lwip_periodic_handle() ;                    //LWIP内核需要定时处理的函数
//    lwip_pkt_handle() ;
//  }
//  POINT_COLOR=RED;
//  LCD_ShowString( 30, 110, "LWIP Init Successed" ) ;
//  //打印动态IP地址
//  if( lwipdev.dhcpstatus==2 )
//    sprintf( ( char* )buf, "DHCP IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  //打印静态IP地址
//  else
//    sprintf( ( char* )buf, "Static IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 130, buf ) ; 
//  //得到网速
//  if( ( DM9000_Get_SpeedAndDuplex()&0x02 )==0x02 )
//    LCD_ShowString( 30, 150, "Ethernet Speed:10M" ) ;
//  else
//    LCD_ShowString( 30, 150, "Ethernet Speed:100M" ) ;
//   while( 1 )
//  {
//    tcp_server_test() ;                        //TCP服务器测试
//    lwip_periodic_handle() ;
//    lwip_pkt_handle() ;
//    delay_ms( 2 ) ;
//  }
//}

