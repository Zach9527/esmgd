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
uint8_t tcp_client_recvbuf[ TCP_CLIENT_RX_BUFSIZE ] ;                //�������ݻ�����

uint16_t  tcp_client_flag;
/*

Ŀ���ǰ�ͨ�����ڽ��յ������ݴ洢��RxBuffer��

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
//	 	/******������ԭ������*******************/
////		  udp_sendto(upcb, p, &destAddr, port); /* ���յ��������ٷ��ͳ�ȥ */	
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

Ȼ��ͨ���������յ������ݽ��д���
��ȡ�����жϽ��յ������ݵ���RxBuffer��
���Ҵ洢���Լ������ָ����
*/
//�鿴�������Ƿ��ܵ�����
//uint8_t udp_HasData(void)
//{
//	if(RxCount >= 4)
//		return RxCount;
//	return 0;
//}
////������������
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
//�����pingͨ�����⣬���ھ�Ҫ��ʼ����ô������յ������ݣ�
//��ԭ���ķ���������������
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
//����udp_send���������յ����ְ�p�����˻�ȥ����ת��udp_send����
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
//// ��udp_send�����У��ܿ�������װ�õ�udp_sendto()�����������ʵ�ʽ����շ��ĺ�����


///*
//tcp_client.h�����д

//*/

//#ifndef _TCP_CLIENT_H_
//#define _TCP_CLIENT_H_
//#include "sys.h"
//#include "lwip/tcp.h"
//#include "lwip/pbuf.h"
//#define TCP_CLIENT_RX_BUFSIZE  1500            //���������ݳ���
//#define TCP_CLIENT_TX_BUFSIZE  200              //��������ݳ���
//#define LWIP_SEND_DATA      0x80            //�����ݷ���
//#define  TCP_CLIENT_PORT    8087            //Զ�˶˿�
////tcp����������״̬
//enum tcp_client_states
//{
//  ES_TCPCLIENT_NONE = 0,    //û������
//  ES_TCPCLIENT_CONNECTED,  //���ӵ��������� 
//  ES_TCPCLIENT_CLOSING,    //�ر�����
//};
////LWIP�ص�����ʹ�õĽṹ��
//struct tcp_client_struct
//{
//  u8 state;            //��ǰ����״
//  struct tcp_pcb *pcb;      //ָ��ǰ��pcb
//  struct pbuf *p;        //ָ�����/�����pbuf
//};  
//void tcp_client_test( void ) ;                    //TCP Client���Ժ���
//#endif

///*

//Ӧ�ñ�д
//tcp_client.c�����д
//*/

////TCP Client ����ȫ��״̬��Ǳ���
////bit7:0,û������Ҫ����;1,������Ҫ����
////bit6:0,û���յ�����;1,�յ�������
////bit5:0,û�������Ϸ�����;1,�����Ϸ�������
////bit4~0:����
//u8 tcp_client_flag;   
////����Զ��IP��ַ
//void tcp_client_set_remoteip()
//{
//  u8 *tbuf;
//  tbuf=mymalloc( SRAMIN, 100 ) ;                      //�����ڴ�
//  if( tbuf==NULL )
//    return ;
//  //ǰ����IP���ֺ�DHCP�õ���IPһ��
//  lwipdev.remoteip[ 0 ] = lwipdev.ip[ 0 ] ;
//  lwipdev.remoteip[ 1 ] = lwipdev.ip[ 1 ] ;
//  lwipdev.remoteip[ 2 ] = lwipdev.ip[ 2 ] ;
//  lwipdev.remoteip[ 3 ] = 113 ;
//  sprintf( ( char* )tbuf, "Remote IP:%d.%d.%d.", lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2] ) ;
//  LCD_ShowString( 30, 150, tbuf ) ;                    //Զ��IP
//  myfree( SRAMIN, tbuf ) ;
//}
////�ر��������������
//void tcp_client_connection_close( struct tcp_pcb *tpcb, struct tcp_client_struct *es )
//{
//  tcp_abort( tpcb ) ;                            //��ֹ����,ɾ��pcb���ƿ�
//  tcp_arg( tpcb, NULL ) ;
//  tcp_recv( tpcb, NULL ) ;
//  tcp_sent( tpcb, NULL ) ;
//  tcp_err( tpcb, NULL ) ;
//  tcp_poll( tpcb, NULL, 0 );
//  if( es )
//    mem_free( es ) ;
//  tcp_client_flag &= ~( 1<<5 ) ;                      //������ӶϿ���
//}
// tcp_recv�����Ļص�����
err_t tcp_client_recv( void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err )
{
  uint32_t data_len=0 ;
  struct pbuf *q ;
  struct tcp_client_struct *es ;
  err_t ret_err ;
  LWIP_ASSERT( "arg != NULL", arg!=NULL ) ;
  es = ( struct tcp_client_struct* )arg ;
  //����ӷ��������յ��յ�����֡�͹ر�����
  if( p==NULL )
  {
    es->state = ES_TCPCLIENT_CLOSING ;                //��Ҫ�ر�TCP������
     es->p = p ;
    ret_err = ERR_OK ;
  }
  //�����յ�һ���ǿյ�����֡,����err!=ERR_OK
  else if( err!=ERR_OK )
  { 
    if( p )
      pbuf_free( p ) ;                        //�ͷŽ���pbuf
    ret_err = err ;
  }
  //����������״̬ʱ
  else if( es->state==ES_TCPCLIENT_CONNECTED )
  {
    //����������״̬���ҽ��յ������ݲ�Ϊ��ʱ
    if( p!=NULL )
    {
      memset( tcp_client_recvbuf, 0, TCP_CLIENT_RX_BUFSIZE ) ;      //���ݽ��ջ���������
      //����������pbuf����
      for( q=p; q!=NULL; q=q->next )
      {
        if( q->len>( TCP_CLIENT_RX_BUFSIZE-data_len ) )
          memcpy( tcp_client_recvbuf+data_len, q->payload, TCP_CLIENT_RX_BUFSIZE-data_len ) ; 
        else
          memcpy( tcp_client_recvbuf+data_len, q->payload, q->len ) ;
        data_len += q->len ;
        //����TCP�ͻ��˽�������,����
        if( data_len>TCP_CLIENT_RX_BUFSIZE )
          break ;
      }
      tcp_client_flag |= 1<<6 ;                    //��ǽ��յ�������
       tcp_recved( tpcb,p->tot_len );                  //���ڻ�ȡ��������
      pbuf_free( p ) ;                        //�ͷ��ڴ�
      ret_err = ERR_OK ;
    }
  }
  //���յ����ݵ��������Ѿ��ر�
  else
  { 
    tcp_recved( tpcb, p->tot_len ) ;                    //���ڻ�ȡ��������
    es->p = NULL ;
    pbuf_free( p ) ;                          //�ͷ��ڴ�
    ret_err = ERR_OK ;
  }
  return ret_err ;
}
//// tcp_err�����Ļص�����
//void tcp_client_error( void *arg, err_t err )
//{

//}
////��������
//void tcp_client_senddata( struct tcp_pcb *tpcb, struct tcp_client_struct *es )
//{
//  struct pbuf *ptr ; 
//   err_t wr_err = ERR_OK ;
//  while( ( wr_err==ERR_OK )&&( es->p )&&( es->p->len<=tcp_sndbuf( tpcb ) ) )
//  {
//    ptr = es->p ;
//    wr_err = tcp_write( tpcb, ptr->payload, ptr->len, 1 ) ;          //���ݼ��뵽���ͻ��������
//    if( wr_err==ERR_OK )
//    {
//      es->p = ptr->next ;                      //ָ����һ��pbuf
//      //pbuf��ref��һ
//      if( es->p )
//        pbuf_ref( es->p );
//      pbuf_free( ptr ) ;                        //�ͷ�ptr
//    }
//    else if( wr_err==ERR_MEM )
//      es->p = ptr ;
//    tcp_output( tpcb ) ;                        //���ͻ�������е����ݷ���
//  }
//}
//// tcp_sent�Ļص�����(��Զ�˽��յ�ACK��������)
//err_t tcp_client_sent( void *arg, struct tcp_pcb *tpcb, u16_t len )
//{
//  struct tcp_client_struct *es ;
//  LWIP_UNUSED_ARG( len ) ;
//  es = ( struct tcp_client_struct* )arg ;
//  if( es->p )
//    tcp_client_senddata( tpcb, es ) ;                    //��������
//  return ERR_OK ;
//}
//// tcp_poll�Ļص�����
//const u8 *tcp_client_sendbuf = "STM32F103 TCP Client send data\r\n" ;      //TCP������������������
//err_t tcp_client_poll( void *arg, struct tcp_pcb *tpcb )
//{
//  err_t ret_err ;
//  struct tcp_client_struct *es ; 
//  es = ( struct tcp_client_struct* )arg ;
//  //���Ӵ��ڿ��п��Է�������
//  if( es!=NULL )
//  {
//    //�ж��Ƿ�������Ҫ����
//    if( tcp_client_flag&( 1<<7 ) )
//    {
//      es->p = pbuf_alloc( PBUF_TRANSPORT, strlen( ( char* )tcp_client_sendbuf ), PBUF_POOL ) ; 
//      pbuf_take( es->p, ( char* )tcp_client_sendbuf, strlen( ( char* )tcp_client_sendbuf ) ) ; 
//      tcp_client_senddata( tpcb, es ) ;                  //�����ݷ��ͳ�ȥ
//      tcp_client_flag &= ~( 1<<7 ) ;                  //������ݷ��ͱ�־
//      //�ͷ��ڴ�
//      if( es->p )
//        pbuf_free( es->p ) ;
//    }
//    else if( es->state==ES_TCPCLIENT_CLOSING )
//       tcp_client_connection_close( tpcb, es ) ;              //�ر�TCP����
//    ret_err = ERR_OK ;
//  }
//  else
//  { 
//    tcp_abort( tpcb ) ;                          //��ֹ����,ɾ��pcb���ƿ�
//    ret_err = ERR_ABRT ;
//  }
//  return ret_err ;
//}
////���ӽ�������ûص�����
//err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
//{
//  struct tcp_client_struct *es=NULL;  
//  if(err==ERR_OK)   
//  {
//    es = ( struct tcp_client_struct* )mem_malloc( sizeof( struct tcp_client_struct ) ) ; //�����ڴ�
//    //�ڴ�����ɹ�
//    if( es )
//    {
//       es->state = ES_TCPCLIENT_CONNECTED ;            //״̬Ϊ���ӳɹ�
//      es->pcb = tpcb ;
//      es->p = NULL ;
//      tcp_arg( tpcb, es ) ;                      //����tpcb��callback_arg
//      tcp_recv( tpcb, tcp_client_recv ) ;                  //��ʼ��tcp_recv�ص�����
//      tcp_err( tpcb, tcp_client_error ) ;                  //��ʼ��tcp_err()�ص�����
//      tcp_sent( tpcb, tcp_client_sent ) ;                  //��ʼ��tcp_sent�ص�����
//      tcp_poll( tpcb, tcp_client_poll, 1 ) ;                //��ʼ��tcp_poll�ص�����
//       tcp_client_flag |= 1<<5 ;                    //������ӵ���������
//      err = ERR_OK ;
//    }
//    else
//    {
//      tcp_client_connection_close( tpcb, es ) ;              //�ر�����
//      err = ERR_MEM ;                        //�����ڴ�������
//    }
//  }
//  else
//    tcp_client_connection_close( tpcb, 0 ) ;                //�ر�����
//  return err ;
//}
////�ͻ�������
//void tcp_client_test()
//{
//   struct tcp_pcb *tcppcb ;                        //����һ��TCP���������ƿ�
//  struct ip_addr rmtipaddr ;                        //Զ��ip��ַ
//  u8 *tbuf ;
//  u8 res=0 ;    
//  u8 t=0 ; 
//  u8 connflag=0 ;                            //���ӱ��
//  tcp_client_set_remoteip() ;                        //��ѡ��IP
//  tbuf = mymalloc( SRAMIN, 200 ) ;                    //�����ڴ�
//  //�ڴ�����ʧ����,ֱ���˳�
//  if( tbuf==NULL )
//    return ;
//  sprintf( ( char* )tbuf, "Local IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 130, tbuf ) ;                    //������IP
//  sprintf( ( char* )tbuf, "Remote IP:%d.%d.%d.%d", lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2], lwipdev.remoteip[3] ) ;
//  LCD_ShowString( 30, 150, tbuf ) ;                    //Զ��IP
//  sprintf( ( char* )tbuf, "Remote Port:%d", TCP_CLIENT_PORT ) ;          //�ͻ��˶˿ں�
//  LCD_ShowString( 30, 170, tbuf ) ;
//  LCD_ShowString( 30, 190, "STATUS:Disconnected" ) ;
//  tcppcb = tcp_new() ;                          //����һ���µ�pcb
//  //�����ɹ�
//  if( tcppcb )
//  {
//  IP4_ADDR( &rmtipaddr, lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2], lwipdev.remoteip[3] ) ;
//    tcp_connect( tcppcb, &rmtipaddr, TCP_CLIENT_PORT, tcp_client_connected ) ; 
//   }
//  else
//    res = 1 ;
//  while( res==0 )
//  {
//    //�Ƿ��յ�����
//    if( tcp_client_flag&1<<6 )
//    {
//      LCD_ShowString( 30, 230, tcp_client_recvbuf ) ;            //��ʾ���յ�������
//      tcp_client_flag |= 1<<7 ;                    //���Ҫ��������
//      tcp_client_flag &= ~( 1<<6 ) ;                  //��������Ѿ���������
//    }
//    //�Ƿ�������
//    if( tcp_client_flag&1<<5 )
//    {
//      if( connflag==0 )
//      { 
//        LCD_ShowString( 30, 190, "STATUS:Connected   " ) ;
//        LCD_ShowString( 30, 210, "Receive Data:" ) ;
//        connflag = 1 ;                        //���������
//      }
//    }
//    else if( connflag )
//    {
//       LCD_ShowString( 30, 190, "STATUS:Disconnected" ) ;
//      connflag = 0 ;                          //������ӶϿ���
//    } 
//    lwip_periodic_handle() ;
//    lwip_pkt_handle() ;
//    delay_ms( 2 ) ;
//    t ++ ;
//    if( t==200 )
//    {
//      //δ������,��������
//      if( ( connflag==0 )&&( ( tcp_client_flag&1<<5 )==0 ) )
//      { 
//        tcp_client_connection_close( tcppcb, 0 ) ;            //�ر�����
//        tcppcb = tcp_new() ;                    //����һ���µ�pcb
//        //�����ɹ�
//        if( tcppcb )
//          tcp_connect( tcppcb, &rmtipaddr, TCP_CLIENT_PORT, tcp_client_connected ) ; 
//      }
//      t = 0 ;
//    }    
//  }
//  tcp_client_connection_close( tcppcb, 0 ) ;                  //�ر�TCP Client����
//  myfree( SRAMIN, tbuf ) ;
//}

//26.2.2 tcp_client.h�����д


///*
//�й���TCPЭ���֪ʶ����һ���Ѿ��й���������������ֱ��ʹ��API��ʵ��TCP������ģʽ��
//*/
//27.1 ʵ������
//27.1.1 tcp_server.c�����д
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
////TCP Server ����ȫ��״̬��Ǳ���
////bit7:0,û������Ҫ����;1,������Ҫ����
////bit6:0,û���յ�����;1,�յ�������.
////bit5:0,û�пͻ���������;1,�пͻ�����������.
////bit4~0:����
//u8 tcp_server_flag;
////�ر�tcp����
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
//  tcp_server_flag &= ~( 1<<5 ) ;                      //������ӶϿ���
//}
//// tcp_recv�����Ļص�����
//u8 tcp_server_recvbuf[ TCP_SERVER_RX_BUFSIZE ] ;                //TCP Server�������ݻ�����
//err_t tcp_server_recv( void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err )
//{
//  err_t ret_err ;
//  u32 data_len = 0 ;
//  struct pbuf *q ;
//    struct tcp_server_struct *es ;
//  LWIP_ASSERT( "arg != NULL", arg != NULL ) ;
//  es = ( struct tcp_server_struct* )arg ;
//  //�ӿͻ��˽��յ�������
//  if( p==NULL )
//  {
//    es->state = ES_TCPSERVER_CLOSING ;                //��Ҫ�ر�TCP������
//    es->p = p ; 
//    ret_err = ERR_OK ;
//  }
//  //�ӿͻ��˽��յ�һ���ǿ�����,��������ĳ��ԭ��err!=ERR_OK
//  else if( err!=ERR_OK )
//  {
//    if( p )
//      pbuf_free( p ) ;                        //�ͷŽ���pbuf
//    ret_err = err ;
//  }
//  //��������״̬
//  else if( es->state==ES_TCPSERVER_ACCEPTED )
//  {
//    //����������״̬���ҽ��յ������ݲ�Ϊ��ʱ�����ӡ����
//    if( p!=NULL )
//    {
//      memset( tcp_server_recvbuf, 0, TCP_SERVER_RX_BUFSIZE ) ;      //���ݽ��ջ���������
//      //����������pbuf����
//      for( q=p; q!=NULL; q=q->next )
//      {
//        if( q->len>( TCP_SERVER_RX_BUFSIZE-data_len ) )
//          memcpy( tcp_server_recvbuf+data_len, q->payload, TCP_SERVER_RX_BUFSIZE-data_len ) ; 
//        else
//          memcpy(tcp_server_recvbuf+data_len, q->payload, q->len ) ;
//        data_len += q->len ;
//        //����TCP�ͻ��˽�������,����
//        if( data_len>TCP_SERVER_RX_BUFSIZE )
//          break ;
//      }
//      tcp_server_flag |= 1<<6 ;                    //��ǽ��յ�������
//      lwipdev.remoteip[ 0 ] = tpcb->remote_ip.addr&0xFF ;        //IADDR4
//      lwipdev.remoteip[ 1 ] = ( tpcb->remote_ip.addr>>8 )&0xFF ;    //IADDR3
//      lwipdev.remoteip[ 2 ] = ( tpcb->remote_ip.addr>>16 )&0xFF ;    //IADDR2
//      lwipdev.remoteip[ 3 ] = ( tpcb->remote_ip.addr>>24 )&0xFF ;    //IADDR1 
//       tcp_recved( tpcb, p->tot_len ) ;                  //���ڻ�ȡ��������
//      pbuf_free( p ) ;                        //�ͷ��ڴ�
//      ret_err = ERR_OK ;
//    }
//  }
//  //�������ر���
//  else
//  {
//    tcp_recved( tpcb, p->tot_len  );                    //���ڻ�ȡ��������
//    es->p=  NULL ;
//    pbuf_free( p ) ;                          //�ͷ��ڴ�
//    ret_err = ERR_OK ;
//  }
//  return ret_err ;
//}
//// tcp_err�����Ļص�����
//void tcp_server_error( void *arg, err_t err )
//{  
//  LWIP_UNUSED_ARG( err ) ;
//  printf( "tcp error:%x\r\n", ( u32 )arg ) ;
//  //�ͷ��ڴ�
//  if( arg!=NULL )
//    mem_free( arg ) ;
//}
////��������
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
//      es->p = ptr->next ;                      //ָ����һ��pbuf
//      //pbuf��ref��һ
//      if( es->p )
//        pbuf_ref( es->p ) ;
//      pbuf_free( ptr ) ;
//      tcp_recved( tpcb, plen ) ;                    //����tcp���ڴ�С
//    }
//    else if( wr_err==ERR_MEM )
//      es->p = ptr ;
//   }
//}
//// tcp_poll�Ļص�����
//const u8 *tcp_server_sendbuf = "STM32F103 TCP Server send data\r\n" ;      //TCP������������������
//err_t tcp_server_poll( void *arg, struct tcp_pcb *tpcb )
//{
//  err_t ret_err;
//  struct tcp_server_struct *es ; 
//  es = ( struct tcp_server_struct* )arg ;
//  if( es!=NULL )
//  {
//    //�ж��Ƿ�������Ҫ����
//    if( tcp_server_flag&( 1<<7 ) )
//    {
//      es->p = pbuf_alloc( PBUF_TRANSPORT, strlen( ( char* )tcp_server_sendbuf ), PBUF_POOL ) ; 
//      pbuf_take( es->p, ( char* )tcp_server_sendbuf, strlen( ( char* )tcp_server_sendbuf ) ) ;
//      tcp_server_senddata( tpcb, es ) ;                  //��ѯ��ʱ����Ҫ���͵�����
//      tcp_server_flag &= ~( 1<<7 ) ;                  //������ݷ��ͱ�־λ
//      if( es->p!=NULL )
//        pbuf_free( es->p ) ;                    //�ͷ��ڴ�  
//    }
//    //�رղ���
//    else if( es->state==ES_TCPSERVER_CLOSING )
//      tcp_server_connection_close( tpcb, es ) ;              //�ر�����
//    ret_err = ERR_OK ;
//  }
//  else
//  {
//    tcp_abort( tpcb ) ;                          //��ֹ����,ɾ��pcb���ƿ�
//    ret_err = ERR_ABRT ; 
//  }
//  return ret_err ;
//}
//// tcp_sent�Ļص�����(Զ���յ�ACK�źź�������)
//err_t tcp_server_sent( void *arg, struct tcp_pcb *tpcb, u16_t len )
//{
//  struct tcp_server_struct *es ;
//  LWIP_UNUSED_ARG( len ) ;
//  es = ( struct tcp_server_struct * )arg ;
//  if( es->p )
//    tcp_server_senddata( tpcb, es ) ;                    //��������
//  return ERR_OK ;
//}
////ǿ��ɾ�������Ͽ�ʱ��time wait
//extern void tcp_pcb_purge( struct tcp_pcb *pcb ) ;                //�� tcp.c����
//extern struct tcp_pcb *tcp_active_pcbs ;                    //�� tcp.c����
//extern struct tcp_pcb *tcp_tw_pcbs ;                      //�� tcp.c����
//void tcp_server_remove_timewait()
//{
//  struct tcp_pcb *pcb, *pcb2 ; 
//  while( tcp_active_pcbs!=NULL )
//  {
//    lwip_periodic_handle() ;                      //������ѯ
//    lwip_pkt_handle() ;
//     delay_ms( 10 ) ;                          //�ȴ�tcp_active_pcbsΪ��  
//  }
//  pcb = tcp_tw_pcbs ;
//  //����еȴ�״̬��pcbs
//  while( pcb!=NULL )
//  {
//    tcp_pcb_purge( pcb ) ;
//    tcp_tw_pcbs = pcb->next ;
//    pcb2 = pcb ;
//    pcb = pcb->next ;
//    memp_free( MEMP_TCP_PCB, pcb2 ) ;
//  }
//}
//// tcp_accept�Ļص�����
//err_t tcp_server_accept( void *arg, struct tcp_pcb *newpcb, err_t err )
//{
//  err_t ret_err ;
//  struct tcp_server_struct *es ;
//   LWIP_UNUSED_ARG( arg ) ;
//  LWIP_UNUSED_ARG( err ) ;
//  tcp_setprio( newpcb, TCP_PRIO_MIN ) ;                                //�����´�����pcb���ȼ�
//  es = ( struct tcp_server_struct* )mem_malloc( sizeof( struct tcp_server_struct ) ) ;  //�����ڴ�
//  //�ڴ����ɹ�
//   if( es!=NULL )
//  {
//    es->state = ES_TCPSERVER_ACCEPTED ;                //��������
//    es->pcb = newpcb ;
//    es->p = NULL ;
//    tcp_arg( newpcb, es ) ;
//    tcp_recv( newpcb, tcp_server_recv ) ;                  //��ʼ��tcp_recv�Ļص�����
//    tcp_err( newpcb, tcp_server_error ) ;                  //��ʼ��tcp_err�ص�����
//    tcp_poll( newpcb, tcp_server_poll, 1 ) ;                //��ʼ��tcp_poll�ص�����
//    tcp_sent( newpcb, tcp_server_sent ) ;                  //��ʼ�����ͻص�����
//    tcp_server_flag |= 1<<5 ;                      //����пͻ���������
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
//// TCP Server ����
//void tcp_server_test()
//{
//  err_t err ;  
//  struct tcp_pcb *tcppcbnew ;                        //����һ��TCP���������ƿ�
//  struct tcp_pcb *tcppcbconn ;                      //����һ��TCP���������ƿ�
//  u8 *tbuf ;
//  u8 res=0 ;
//  u8 connflag=0 ;                            //���ӱ��
//  tbuf = mymalloc( SRAMIN, 200 ) ;                    //�����ڴ�
//  //�ڴ�����ʧ����,ֱ���˳�
//  if( tbuf==NULL )
//    return ;
//  sprintf( ( char* )tbuf, "Server IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 130, tbuf ) ;                    //������IP
//  sprintf( ( char* )tbuf, "Server Port:%d", TCP_SERVER_PORT ) ;
//  LCD_ShowString( 30, 150, tbuf ) ;                    //�������˿ں�
//  tcppcbnew = tcp_new() ;                        //����һ���µ�pcb
//  //�����ɹ�
//  if( tcppcbnew )
//  {
//    err = tcp_bind( tcppcbnew, IP_ADDR_ANY, TCP_SERVER_PORT ) ;      //������IP��ָ���˿ںŰ�
//    //�����
//    if( err==ERR_OK )
//    {
//      tcppcbconn = tcp_listen( tcppcbnew ) ;              //����tcppcb�������״̬
//      tcp_accept( tcppcbconn, tcp_server_accept ) ;            //��ʼ��tcp_accept�Ļص�����
//    }
//    else
//      res = 1 ;
//  }
//  else
//    res = 1 ;
//  while( res==0 )
//  {
//    //�յ�����
//    if( tcp_server_flag&1<<6 )
//    {
//      tcp_server_flag |= 1<<7 ;                    //���Ҫ��������
//      LCD_ShowString( 30, 210, tcp_server_recvbuf ) ;          //��ʾ���յ�������
//      tcp_server_flag &= ~( 1<<6 ) ;                  //��������Ѿ���������
//    }
//    //�Ƿ�������
//    if( tcp_server_flag&1<<5 )
//    {
//      if( connflag==0 )
//      { 
//      sprintf( ( char* )tbuf, "Client IP:%d.%d.%d.%d", lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2], lwipdev.remoteip[3] ) ;
//         LCD_ShowString( 30, 170, tbuf ) ;              //�ͻ���IP
//        LCD_ShowString( 30, 190, "Receive Data:" ) ;          //��ʾ��Ϣ
//        connflag = 1 ;                        //���������
//      }
//    }
//    else if( connflag )
//      connflag = 0 ;                          //������ӶϿ���
//    lwip_periodic_handle() ;
//    lwip_pkt_handle() ;
//    delay_ms( 2 ) ;
//  }   
//  tcp_server_connection_close( tcppcbnew, 0 ) ;                //�ر�TCP Server����
//  tcp_server_connection_close( tcppcbconn, 0 ) ;                //�ر�TCP Server����
//  tcp_server_remove_timewait() ; 
//  memset( tcppcbnew, 0, sizeof( struct tcp_pcb ) ) ;
//  memset( tcppcbconn, 0, sizeof( struct tcp_pcb ) ) ;
//  myfree( SRAMIN, tbuf ) ;
//}

//27.1.2 tcp_server.h�����д
//#ifndef _TCP_SERVER_DEMO_H_
//#define _TCP_SERVER_DEMO_H_
//#include "sys.h"
//#include "lwip/tcp.h"
//#include "lwip/pbuf.h"
//#define TCP_SERVER_RX_BUFSIZE  2000          //����tcp server���������ݳ���
//#define TCP_SERVER_PORT      8088          //����tcp server�Ķ˿�
////tcp����������״̬
//enum tcp_server_states
//{
//  ES_TCPSERVER_NONE = 0,      //û������
//  ES_TCPSERVER_ACCEPTED,      //�пͻ�����������
//  ES_TCPSERVER_CLOSING,      //�����ر�����
//};
////LWIP�ص�����ʹ�õĽṹ��
//struct tcp_server_struct
//{
//  u8 state;              //��ǰ����״
//  struct tcp_pcb *pcb;        //ָ��ǰ��pcb
//  struct pbuf *p;          //ָ�����/�����pbuf
//}; 
//void tcp_server_test( void ) ;                  //TCP Server���Ժ���
//#endif

//27.1.3 �����������д
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
//   STM32_Clock_Init( 9 ) ;                                        //ϵͳʱ������
//  SysTick_Init( 72 ) ;                          //��ʱ��ʼ��
//  USART1_Init( 72, 115200 ) ;                      //���ڳ�ʼ��Ϊ115200
//  LCD_Init() ;                            //��ʼ��LCD
//  TIM3_Init( 1000, 719 ) ;                        //��ʱ��3Ƶ��Ϊ100hz
//  my_mem_init( SRAMIN ) ;                      //��ʼ���ڲ��ڴ��
//  while( lwip_comm_init() ) ;                      //lwip��ʼ��
//  //�ȴ�DHCP��ȡ�ɹ�/��ʱ���
//  while( ( lwipdev.dhcpstatus!=2 )&&( lwipdev.dhcpstatus!=0xFF ) )
//  {
//    lwip_periodic_handle() ;                    //LWIP�ں���Ҫ��ʱ����ĺ���
//    lwip_pkt_handle() ;
//  }
//  POINT_COLOR=RED;
//  LCD_ShowString( 30, 110, "LWIP Init Successed" ) ;
//  //��ӡ��̬IP��ַ
//  if( lwipdev.dhcpstatus==2 )
//    sprintf( ( char* )buf, "DHCP IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  //��ӡ��̬IP��ַ
//  else
//    sprintf( ( char* )buf, "Static IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 130, buf ) ; 
//  //�õ�����
//  if( ( DM9000_Get_SpeedAndDuplex()&0x02 )==0x02 )
//    LCD_ShowString( 30, 150, "Ethernet Speed:10M" ) ;
//  else
//    LCD_ShowString( 30, 150, "Ethernet Speed:100M" ) ;
//   while( 1 )
//  {
//    tcp_server_test() ;                        //TCP����������
//    lwip_periodic_handle() ;
//    lwip_pkt_handle() ;
//    delay_ms( 2 ) ;
//  }
//}



//�й���TCPЭ���֪ʶ����һ���Ѿ��й���������������ֱ��ʹ��API��ʵ��TCP������ģʽ��

//27.1 ʵ������
//27.1.1 tcp_server.c�����д
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
////TCP Server ����ȫ��״̬��Ǳ���
////bit7:0,û������Ҫ����;1,������Ҫ����
////bit6:0,û���յ�����;1,�յ�������.
////bit5:0,û�пͻ���������;1,�пͻ�����������.
////bit4~0:����
//u8 tcp_server_flag;
////�ر�tcp����
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
//  tcp_server_flag &= ~( 1<<5 ) ;                      //������ӶϿ���
//}
//// tcp_recv�����Ļص�����
//u8 tcp_server_recvbuf[ TCP_SERVER_RX_BUFSIZE ] ;                //TCP Server�������ݻ�����
//err_t tcp_server_recv( void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err )
//{
//  err_t ret_err ;
//  u32 data_len = 0 ;
//  struct pbuf *q ;
//    struct tcp_server_struct *es ;
//  LWIP_ASSERT( "arg != NULL", arg != NULL ) ;
//  es = ( struct tcp_server_struct* )arg ;
//  //�ӿͻ��˽��յ�������
//  if( p==NULL )
//  {
//    es->state = ES_TCPSERVER_CLOSING ;                //��Ҫ�ر�TCP������
//    es->p = p ; 
//    ret_err = ERR_OK ;
//  }
//  //�ӿͻ��˽��յ�һ���ǿ�����,��������ĳ��ԭ��err!=ERR_OK
//  else if( err!=ERR_OK )
//  {
//    if( p )
//      pbuf_free( p ) ;                        //�ͷŽ���pbuf
//    ret_err = err ;
//  }
//  //��������״̬
//  else if( es->state==ES_TCPSERVER_ACCEPTED )
//  {
//    //����������״̬���ҽ��յ������ݲ�Ϊ��ʱ�����ӡ����
//    if( p!=NULL )
//    {
//      memset( tcp_server_recvbuf, 0, TCP_SERVER_RX_BUFSIZE ) ;      //���ݽ��ջ���������
//      //����������pbuf����
//      for( q=p; q!=NULL; q=q->next )
//      {
//        if( q->len>( TCP_SERVER_RX_BUFSIZE-data_len ) )
//          memcpy( tcp_server_recvbuf+data_len, q->payload, TCP_SERVER_RX_BUFSIZE-data_len ) ; 
//        else
//          memcpy(tcp_server_recvbuf+data_len, q->payload, q->len ) ;
//        data_len += q->len ;
//        //����TCP�ͻ��˽�������,����
//        if( data_len>TCP_SERVER_RX_BUFSIZE )
//          break ;
//      }
//      tcp_server_flag |= 1<<6 ;                    //��ǽ��յ�������
//      lwipdev.remoteip[ 0 ] = tpcb->remote_ip.addr&0xFF ;        //IADDR4
//      lwipdev.remoteip[ 1 ] = ( tpcb->remote_ip.addr>>8 )&0xFF ;    //IADDR3
//      lwipdev.remoteip[ 2 ] = ( tpcb->remote_ip.addr>>16 )&0xFF ;    //IADDR2
//      lwipdev.remoteip[ 3 ] = ( tpcb->remote_ip.addr>>24 )&0xFF ;    //IADDR1 
//       tcp_recved( tpcb, p->tot_len ) ;                  //���ڻ�ȡ��������
//      pbuf_free( p ) ;                        //�ͷ��ڴ�
//      ret_err = ERR_OK ;
//    }
//  }
//  //�������ر���
//  else
//  {
//    tcp_recved( tpcb, p->tot_len  );                    //���ڻ�ȡ��������
//    es->p=  NULL ;
//    pbuf_free( p ) ;                          //�ͷ��ڴ�
//    ret_err = ERR_OK ;
//  }
//  return ret_err ;
//}
//// tcp_err�����Ļص�����
//void tcp_server_error( void *arg, err_t err )
//{  
//  LWIP_UNUSED_ARG( err ) ;
//  printf( "tcp error:%x\r\n", ( u32 )arg ) ;
//  //�ͷ��ڴ�
//  if( arg!=NULL )
//    mem_free( arg ) ;
//}
////��������
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
//      es->p = ptr->next ;                      //ָ����һ��pbuf
//      //pbuf��ref��һ
//      if( es->p )
//        pbuf_ref( es->p ) ;
//      pbuf_free( ptr ) ;
//      tcp_recved( tpcb, plen ) ;                    //����tcp���ڴ�С
//    }
//    else if( wr_err==ERR_MEM )
//      es->p = ptr ;
//   }
//}
//// tcp_poll�Ļص�����
//const u8 *tcp_server_sendbuf = "STM32F103 TCP Server send data\r\n" ;      //TCP������������������
//err_t tcp_server_poll( void *arg, struct tcp_pcb *tpcb )
//{
//  err_t ret_err;
//  struct tcp_server_struct *es ; 
//  es = ( struct tcp_server_struct* )arg ;
//  if( es!=NULL )
//  {
//    //�ж��Ƿ�������Ҫ����
//    if( tcp_server_flag&( 1<<7 ) )
//    {
//      es->p = pbuf_alloc( PBUF_TRANSPORT, strlen( ( char* )tcp_server_sendbuf ), PBUF_POOL ) ; 
//      pbuf_take( es->p, ( char* )tcp_server_sendbuf, strlen( ( char* )tcp_server_sendbuf ) ) ;
//      tcp_server_senddata( tpcb, es ) ;                  //��ѯ��ʱ����Ҫ���͵�����
//      tcp_server_flag &= ~( 1<<7 ) ;                  //������ݷ��ͱ�־λ
//      if( es->p!=NULL )
//        pbuf_free( es->p ) ;                    //�ͷ��ڴ�  
//    }
//    //�رղ���
//    else if( es->state==ES_TCPSERVER_CLOSING )
//      tcp_server_connection_close( tpcb, es ) ;              //�ر�����
//    ret_err = ERR_OK ;
//  }
//  else
//  {
//    tcp_abort( tpcb ) ;                          //��ֹ����,ɾ��pcb���ƿ�
//    ret_err = ERR_ABRT ; 
//  }
//  return ret_err ;
//}
//// tcp_sent�Ļص�����(Զ���յ�ACK�źź�������)
//err_t tcp_server_sent( void *arg, struct tcp_pcb *tpcb, u16_t len )
//{
//  struct tcp_server_struct *es ;
//  LWIP_UNUSED_ARG( len ) ;
//  es = ( struct tcp_server_struct * )arg ;
//  if( es->p )
//    tcp_server_senddata( tpcb, es ) ;                    //��������
//  return ERR_OK ;
//}
////ǿ��ɾ�������Ͽ�ʱ��time wait
//extern void tcp_pcb_purge( struct tcp_pcb *pcb ) ;                //�� tcp.c����
//extern struct tcp_pcb *tcp_active_pcbs ;                    //�� tcp.c����
//extern struct tcp_pcb *tcp_tw_pcbs ;                      //�� tcp.c����
//void tcp_server_remove_timewait()
//{
//  struct tcp_pcb *pcb, *pcb2 ; 
//  while( tcp_active_pcbs!=NULL )
//  {
//    lwip_periodic_handle() ;                      //������ѯ
//    lwip_pkt_handle() ;
//     delay_ms( 10 ) ;                          //�ȴ�tcp_active_pcbsΪ��  
//  }
//  pcb = tcp_tw_pcbs ;
//  //����еȴ�״̬��pcbs
//  while( pcb!=NULL )
//  {
//    tcp_pcb_purge( pcb ) ;
//    tcp_tw_pcbs = pcb->next ;
//    pcb2 = pcb ;
//    pcb = pcb->next ;
//    memp_free( MEMP_TCP_PCB, pcb2 ) ;
//  }
//}
//// tcp_accept�Ļص�����
//err_t tcp_server_accept( void *arg, struct tcp_pcb *newpcb, err_t err )
//{
//  err_t ret_err ;
//  struct tcp_server_struct *es ;
//   LWIP_UNUSED_ARG( arg ) ;
//  LWIP_UNUSED_ARG( err ) ;
//  tcp_setprio( newpcb, TCP_PRIO_MIN ) ;                                //�����´�����pcb���ȼ�
//  es = ( struct tcp_server_struct* )mem_malloc( sizeof( struct tcp_server_struct ) ) ;  //�����ڴ�
//  //�ڴ����ɹ�
//   if( es!=NULL )
//  {
//    es->state = ES_TCPSERVER_ACCEPTED ;                //��������
//    es->pcb = newpcb ;
//    es->p = NULL ;
//    tcp_arg( newpcb, es ) ;
//    tcp_recv( newpcb, tcp_server_recv ) ;                  //��ʼ��tcp_recv�Ļص�����
//    tcp_err( newpcb, tcp_server_error ) ;                  //��ʼ��tcp_err�ص�����
//    tcp_poll( newpcb, tcp_server_poll, 1 ) ;                //��ʼ��tcp_poll�ص�����
//    tcp_sent( newpcb, tcp_server_sent ) ;                  //��ʼ�����ͻص�����
//    tcp_server_flag |= 1<<5 ;                      //����пͻ���������
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
//// TCP Server ����
//void tcp_server_test()
//{
//  err_t err ;  
//  struct tcp_pcb *tcppcbnew ;                        //����һ��TCP���������ƿ�
//  struct tcp_pcb *tcppcbconn ;                      //����һ��TCP���������ƿ�
//  u8 *tbuf ;
//  u8 res=0 ;
//  u8 connflag=0 ;                            //���ӱ��
//  tbuf = mymalloc( SRAMIN, 200 ) ;                    //�����ڴ�
//  //�ڴ�����ʧ����,ֱ���˳�
//  if( tbuf==NULL )
//    return ;
//  sprintf( ( char* )tbuf, "Server IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 130, tbuf ) ;                    //������IP
//  sprintf( ( char* )tbuf, "Server Port:%d", TCP_SERVER_PORT ) ;
//  LCD_ShowString( 30, 150, tbuf ) ;                    //�������˿ں�
//  tcppcbnew = tcp_new() ;                        //����һ���µ�pcb
//  //�����ɹ�
//  if( tcppcbnew )
//  {
//    err = tcp_bind( tcppcbnew, IP_ADDR_ANY, TCP_SERVER_PORT ) ;      //������IP��ָ���˿ںŰ�
//    //�����
//    if( err==ERR_OK )
//    {
//      tcppcbconn = tcp_listen( tcppcbnew ) ;              //����tcppcb�������״̬
//      tcp_accept( tcppcbconn, tcp_server_accept ) ;            //��ʼ��tcp_accept�Ļص�����
//    }
//    else
//      res = 1 ;
//  }
//  else
//    res = 1 ;
//  while( res==0 )
//  {
//    //�յ�����
//    if( tcp_server_flag&1<<6 )
//    {
//      tcp_server_flag |= 1<<7 ;                    //���Ҫ��������
//      LCD_ShowString( 30, 210, tcp_server_recvbuf ) ;          //��ʾ���յ�������
//      tcp_server_flag &= ~( 1<<6 ) ;                  //��������Ѿ���������
//    }
//    //�Ƿ�������
//    if( tcp_server_flag&1<<5 )
//    {
//      if( connflag==0 )
//      { 
//      sprintf( ( char* )tbuf, "Client IP:%d.%d.%d.%d", lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2], lwipdev.remoteip[3] ) ;
//         LCD_ShowString( 30, 170, tbuf ) ;              //�ͻ���IP
//        LCD_ShowString( 30, 190, "Receive Data:" ) ;          //��ʾ��Ϣ
//        connflag = 1 ;                        //���������
//      }
//    }
//    else if( connflag )
//      connflag = 0 ;                          //������ӶϿ���
//    lwip_periodic_handle() ;
//    lwip_pkt_handle() ;
//    delay_ms( 2 ) ;
//  }   
//  tcp_server_connection_close( tcppcbnew, 0 ) ;                //�ر�TCP Server����
//  tcp_server_connection_close( tcppcbconn, 0 ) ;                //�ر�TCP Server����
//  tcp_server_remove_timewait() ; 
//  memset( tcppcbnew, 0, sizeof( struct tcp_pcb ) ) ;
//  memset( tcppcbconn, 0, sizeof( struct tcp_pcb ) ) ;
//  myfree( SRAMIN, tbuf ) ;
//}

//27.1.2 tcp_server.h�����д
//#ifndef _TCP_SERVER_DEMO_H_
//#define _TCP_SERVER_DEMO_H_
//#include "sys.h"
//#include "lwip/tcp.h"
//#include "lwip/pbuf.h"
//#define TCP_SERVER_RX_BUFSIZE  2000          //����tcp server���������ݳ���
//#define TCP_SERVER_PORT      8088          //����tcp server�Ķ˿�
////tcp����������״̬
//enum tcp_server_states
//{
//  ES_TCPSERVER_NONE = 0,      //û������
//  ES_TCPSERVER_ACCEPTED,      //�пͻ�����������
//  ES_TCPSERVER_CLOSING,      //�����ر�����
//};
////LWIP�ص�����ʹ�õĽṹ��
//struct tcp_server_struct
//{
//  u8 state;              //��ǰ����״
//  struct tcp_pcb *pcb;        //ָ��ǰ��pcb
//  struct pbuf *p;          //ָ�����/�����pbuf
//}; 
//void tcp_server_test( void ) ;                  //TCP Server���Ժ���
//#endif

//27.1.3 �����������д
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
//   STM32_Clock_Init( 9 ) ;                                        //ϵͳʱ������
//  SysTick_Init( 72 ) ;                          //��ʱ��ʼ��
//  USART1_Init( 72, 115200 ) ;                      //���ڳ�ʼ��Ϊ115200
//  LCD_Init() ;                            //��ʼ��LCD
//  TIM3_Init( 1000, 719 ) ;                        //��ʱ��3Ƶ��Ϊ100hz
//  my_mem_init( SRAMIN ) ;                      //��ʼ���ڲ��ڴ��
//  while( lwip_comm_init() ) ;                      //lwip��ʼ��
//  //�ȴ�DHCP��ȡ�ɹ�/��ʱ���
//  while( ( lwipdev.dhcpstatus!=2 )&&( lwipdev.dhcpstatus!=0xFF ) )
//  {
//    lwip_periodic_handle() ;                    //LWIP�ں���Ҫ��ʱ����ĺ���
//    lwip_pkt_handle() ;
//  }
//  POINT_COLOR=RED;
//  LCD_ShowString( 30, 110, "LWIP Init Successed" ) ;
//  //��ӡ��̬IP��ַ
//  if( lwipdev.dhcpstatus==2 )
//    sprintf( ( char* )buf, "DHCP IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  //��ӡ��̬IP��ַ
//  else
//    sprintf( ( char* )buf, "Static IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 130, buf ) ; 
//  //�õ�����
//  if( ( DM9000_Get_SpeedAndDuplex()&0x02 )==0x02 )
//    LCD_ShowString( 30, 150, "Ethernet Speed:10M" ) ;
//  else
//    LCD_ShowString( 30, 150, "Ethernet Speed:100M" ) ;
//   while( 1 )
//  {
//    tcp_server_test() ;                        //TCP����������
//    lwip_periodic_handle() ;
//    lwip_pkt_handle() ;
//    delay_ms( 2 ) ;
//  }
//}


//ע����lwip/coreĿ¼�µ�sys.c�ļ���lwip/include/lwipĿ¼�µ�sys.h������Ϊlwip_sys.c��lwip_sys.h��������SYSTEMĿ¼�µ�sys.c�����������������

//24.5.4 archĿ¼��Դ���ļ����޸�
//��1��arch/cc.h�ļ����루���ļ���Ҫ���Э��ʹ�õ��������͵Ķ��壩

//#ifndef _CC_H_
//#define _CC_H_
//#include "cpu.h"
//#include "stdio.h"
////������ƽ̨�޹ص���������
//typedef unsigned   char    u8_t;                    //�޷���8λ����
//typedef signed     char    s8_t;                    //�з���8λ����
//typedef unsigned   short   u16_t;                    //�޷���16λ����
//typedef signed     short   s16_t;                    //�з���16λ����
//typedef unsigned   long    u32_t;                    //�޷���32λ����
//typedef signed     long    s32_t;                    //�з���32λ����
//typedef u32_t mem_ptr_t ;                        //�ڴ��ַ������
//typedef int sys_prot_t ;                          //�ٽ籣��������
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
////���ݲ�ͬ�ı���������һЩ����
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
////LWIP��printf����ʱʹ�õ�һЩ��������
//#define U16_F "4d"
//#define S16_F "4d"
//#define X16_F "4x"
//#define U32_F "8ld"
//#define S32_F "8ld"
//#define X32_F "8lx"
////�궨��
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

//��2��arch/cpu.h�ļ����루������CPU�Ĵ��С��ģʽ��

//#ifndef _CPU_H_
//#define _CPU_H_
//#define BYTE_ORDER LITTLE_ENDIAN                //С��ģʽ
//#endif

//��3��arch/perf.h�ļ����루����ϵͳ������ͳ�ƣ�

//#ifndef _PERF_H_
//#define _PERF_H_
//#define PERF_START                        //�ն���
//#define PERF_STOP(x)                        //�ն���
//#endif

//��4��arch/sys_arch.h�ļ����루Ϊ�������ϵͳ����ʹ�õĻ�ȡʱ��ĺ���������ΪLWIP�ṩʱ�ӣ�

//#ifndef _ARCH_SYS_ARCH_H_
//#define _ARCH_SYS_ARCH_H_
//#include "cc.h"
//u32_t sys_now( void ) ;
//#endif

//��5��arch/sys_arch.c�ļ�����

//#include "lwip/debug.h"
//#include "lwip/def.h"
//#include "lwip/lwip_sys.h"
//#include "lwip/mem.h"
//#include "tim.h"
////ΪLWIP�ṩ��ʱ
//extern uint32_t lwip_localtime;//lwip����ʱ�������,��λ:ms
//u32_t sys_now()
//{
//  return lwip_localtime ;
//}

//24.5.5 app/commĿ¼��Դ���ļ����޸�
//��1��app/comm.c�ļ�����

//#include "lwip/tcpip.h" 
//#include "malloc.h"
//#include "delay.h"
//#include "usart1.h"
//__lwip_dev lwipdev ;                          //lwip���ƽṹ�� 
//struct netif lwip_netif ;                          //����һ��ȫ�ֵ�����ӿ�
//extern u32 memp_get_memorysize( void ) ;                //��memp.c���涨��
//extern u8_t *memp_memory ;                      //��memp.c���涨��
//extern u8_t *ram_heap ;                        //��mem.c���涨��
//u32 TCPTimer=0 ;                            //TCP��ѯ��ʱ��
//u32 ARPTimer=0 ;                            //ARP��ѯ��ʱ��
//u32 lwip_localtime ;                          //lwip����ʱ�������,��λ:ms
//#if LWIP_DHCP
//u32 DHCPfineTimer=0 ;                        //DHCP��ϸ�����ʱ��
//u32 DHCPcoarseTimer=0 ;                        //DHCP�ֲڴ����ʱ��
//#endif
//u8 lwip_comm_mem_malloc()
//{
//  u32 mempsize ;
//  u32 ramheapsize ;
//  mempsize = memp_get_memorysize() ;                //�õ�memp_memory�����С
//  memp_memory = mymalloc( SRAMIN, mempsize ) ;          //Ϊmemp_memory�����ڴ�
////�õ�ram heap��С
//  ramheapsize = LWIP_MEM_ALIGN_SIZE( MEM_SIZE )+2*LWIP_MEM_ALIGN_SIZE( 4*3 )+MEM_ALIGNMENT ; 
//  ram_heap = mymalloc( SRAMIN, ramheapsize ) ;            //Ϊram_heap�����ڴ�
//  //������ʧ�ܵ�
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
//  //Ĭ��Զ��IPΪ:192.168.1.100
//  lwipx->remoteip[ 0 ] = 192 ;  
//  lwipx->remoteip[ 1 ] = 168 ;
//  lwipx->remoteip[ 2 ] = 1 ;
//  lwipx->remoteip[ 3 ] = 104 ;
//  //MAC��ַ����(�����ֽڹ̶�Ϊ:2.0.0,�����ֽ���STM32ΨһID)
//  lwipx->mac[ 0 ] = dm9000cfg.mac_addr[ 0 ] ;
//  lwipx->mac[ 1 ] = dm9000cfg.mac_addr[ 1 ] ;
//  lwipx->mac[ 2 ] = dm9000cfg.mac_addr[ 2 ] ;
//  lwipx->mac[ 3 ] = dm9000cfg.mac_addr[ 3 ] ;
//  lwipx->mac[ 4 ] = dm9000cfg.mac_addr[ 4 ] ;
//  lwipx->mac[ 5 ] = dm9000cfg.mac_addr[ 5 ] ; 
//  //Ĭ�ϱ���IPΪ:192.168.1.30
//  lwipx->ip[ 0 ] = 192 ;  
//  lwipx->ip[ 1 ] = 168 ;
//  lwipx->ip[ 2 ] = 1 ;
//  lwipx->ip[ 3 ] = 30 ;
//  //Ĭ����������:255.255.255.0
//  lwipx->netmask[ 0 ] = 255 ;  
//  lwipx->netmask[ 1 ] = 255 ;
//  lwipx->netmask[ 2 ] = 255 ;
//  lwipx->netmask[ 3 ] = 0 ;
//  //Ĭ������:192.168.1.1
//  lwipx->gateway[ 0 ] = 192 ;
//  lwipx->gateway[ 1 ] = 168 ;
//  lwipx->gateway[ 2 ] = 1 ;
//  lwipx->gateway[ 3 ] = 1 ;
//  lwipx->dhcpstatus = 0 ;      //û��DHCP
//}
//u8 lwip_comm_init()
//{
//  struct netif *Netif_Init_Flag ;      //����netif_add()����ʱ�ķ���ֵ,�����ж������ʼ���Ƿ�ɹ�
//  struct ip_addr ipaddr ;        //ip��ַ
//  struct ip_addr netmask ;      //��������
//  struct ip_addr gw ;          //Ĭ������
//  //�ڴ�����ʧ��
//  if( lwip_comm_mem_malloc() )
//    return 1 ;
//  //��ʼ��DM9000AEP
//  if( DM9000_Init() )
//    return 2 ;
//  lwip_init() ;            //��ʼ��LWIP�ں�
//  lwip_comm_default_ip_set( &lwipdev ) ; //����Ĭ��IP����Ϣ
////ʹ�ö�̬IP
//#if LWIP_DHCP
//  ipaddr.addr = 0 ;
//  netmask.addr = 0 ;
//  gw.addr = 0 ;
////ʹ�þ�̬IP
//#else
//  IP4_ADDR( &ipaddr, lwipdev.ip[ 0 ], lwipdev.ip[ 1 ], lwipdev.ip[ 2 ], lwipdev.ip[ 3 ] ) ;
//  IP4_ADDR( &netmask, lwipdev.netmask[ 0 ], lwipdev.netmask[1] , lwipdev.netmask[ 2 ], lwipdev.netmask[ 3 ] ) ;
//  IP4_ADDR( &gw, lwipdev.gateway[ 0 ], lwipdev.gateway[ 1 ], lwipdev.gateway[ 2 ], lwipdev.gateway[ 3 ] );
//#endif
//  //�������б������һ������
//  Netif_Init_Flag = netif_add( &lwip_netif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input ) ; 
////���ʹ��DHCP�Ļ�
//#if LWIP_DHCP
//  lwipdev.dhcpstatus = 0 ;        //DHCP���Ϊ0
//  dhcp_start( &lwip_netif ) ;        //����DHCP����
//#endif
//  //�������ʧ��
//  if( Netif_Init_Flag==NULL )
//    return 3 ;
//  //������ӳɹ���,����netifΪĬ��ֵ,���Ҵ�netif����
//  else
//  {
//    netif_set_default( &lwip_netif ) ;    //����netifΪĬ������
//    netif_set_up( &lwip_netif ) ;      //��netif����
//  }
//  return 0 ;                //����OK
//}
//void lwip_pkt_handle()
//{
//  ethernetif_input( &lwip_netif ) ;      //�����绺�����ж�ȡ���յ������ݰ������䷢�͸�LWIP����
//}
//void lwip_periodic_handle()
//{
//#if LWIP_TCP
//  //ÿ250ms����һ��tcp_tmr()����
//  if( lwip_localtime-TCPTimer>=TCP_TMR_INTERVAL )
//  {
//    TCPTimer =  lwip_localtime ;
//    tcp_tmr() ;
//  }
//#endif
//  //ARPÿ5s�����Ե���һ��
//  if( ( lwip_localtime-ARPTimer )>=ARP_TMR_INTERVAL )
//  {
//    ARPTimer = lwip_localtime ;
//    etharp_tmr() ;
//  }
////���ʹ��DHCP�Ļ�
//#if LWIP_DHCP
//  //ÿ500ms����һ��dhcp_fine_tmr()
//  if( lwip_localtime-DHCPfineTimer>=DHCP_FINE_TIMER_MSECS )
//  {
//    DHCPfineTimer = lwip_localtime ;
//    dhcp_fine_tmr() ;
//    if( ( lwipdev.dhcpstatus!=2 )&&( lwipdev.dhcpstatus!=0xFF ) )
//      lwip_dhcp_process_handle() ;                                //DHCP����
//  }
//  //ÿ60sִ��һ��DHCP�ֲڴ���
//  if( lwip_localtime-DHCPcoarseTimer>=DHCP_COARSE_TIMER_MSECS )
//  {
//    DHCPcoarseTimer = lwip_localtime ;
//    dhcp_coarse_tmr() ;
//  }
//#endif
//}
////���ʹ����DHCP
//#if LWIP_DHCP
//void lwip_dhcp_process_handle()
//{
//  u32 ip=0, netmask=0, gw=0 ;
//  switch( lwipdev.dhcpstatus )
//  {
//    //����DHCP
//    case 0:
//      dhcp_start( &lwip_netif ) ;
//      lwipdev.dhcpstatus = 1 ;            //�ȴ�ͨ��DHCP��ȡ���ĵ�ַ
//      break ;
//    //�ȴ���ȡ��IP��ַ
//    case 1:
//    {
//      ip = lwip_netif.ip_addr.addr ;          //��ȡ��IP��ַ
//      netmask = lwip_netif.netmask.addr ;        //��ȡ��������
//      gw = lwip_netif.gw.addr ;            //��ȡĬ������ 
//      //��ȷ��ȡ��IP��ַ��ʱ��
//      if( ip!=0 )
//      {
//        lwipdev.dhcpstatus = 2 ;          //DHCP�ɹ�
//        //������ͨ��DHCP��ȡ����IP��ַ
//        lwipdev.ip[ 3 ] = ( uint8_t )( ip>>24 ) ; 
//        lwipdev.ip[ 2 ] = ( uint8_t )( ip>>16 ) ;
//        lwipdev.ip[ 1 ] = ( uint8_t )( ip>>8 ) ;
//        lwipdev.ip[ 0 ] = ( uint8_t )( ip ) ;
//        //����ͨ��DHCP��ȡ�������������ַ
//        lwipdev.netmask[ 3 ] = ( uint8_t )( netmask>>24 ) ;
//        lwipdev.netmask[ 2 ] = ( uint8_t )( netmask>>16 ) ;
//        lwipdev.netmask[ 1 ] = ( uint8_t )( netmask>>8 ) ;
//        lwipdev.netmask[ 0 ] = ( uint8_t )( netmask ) ;
//        //������ͨ��DHCP��ȡ����Ĭ������
//        lwipdev.gateway[ 3 ] = ( uint8_t )( gw>>24 ) ;
//        lwipdev.gateway[ 2 ] = ( uint8_t )( gw>>16 ) ;
//        lwipdev.gateway[ 1 ] = ( uint8_t )( gw>>8 ) ;
//        lwipdev.gateway[ 0 ] = ( uint8_t )( gw ) ;
//      }
//      //ͨ��DHCP�����ȡIP��ַʧ��,�ҳ�������Դ���
//      else if( lwip_netif.dhcp->tries>LWIP_MAX_DHCP_TRIES )
//      {
//        lwipdev.dhcpstatus = 0xFF ;          //DHCP��ʱʧ��
//        //ʹ�þ�̬IP��ַ
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

//��2��app/comm.h�ļ�����

//#ifndef _COMM_H_
//#define _COMM_H_
//#include "dm9000.h"
//#define LWIP_MAX_DHCP_TRIES  4            //DHCP������������Դ���
////lwip���ƽṹ��
//typedef struct  
//{
//  u8 mac[ 6 ] ;                    //MAC��ַ
//  u8 remoteip[ 4 ] ;                  //Զ������IP��ַ 
//  u8 ip[ 4 ] ;                    //����IP��ַ
//  u8 netmask[ 4 ] ;                  //��������
//  u8 gateway[ 4 ] ;                  //Ĭ�����ص�IP��ַ
//  vu8 dhcpstatus ;                  //dhcp״̬
//                          //0,δ��ȡDHCP��ַ
//                          //1,����DHCP��ȡ״̬
//                          //2,�ɹ���ȡDHCP��ַ
//                          //0XFF,��ȡʧ��
//}__lwip_dev ;
//extern __lwip_dev lwipdev ;                //lwip���ƽṹ��
//void lwip_pkt_handle( void ) ;
//void lwip_periodic_handle( void ) ;
//void lwip_comm_default_ip_set( __lwip_dev *lwipx ) ;
//u8 lwip_comm_mem_malloc( void ) ;
//void lwip_comm_mem_free( void ) ;
//u8 lwip_comm_init( void ) ;
//void lwip_dhcp_process_handle( void ) ;
//#endif

//ע���������õ���δ������������ͣ���Ҫ��sys.h����Ӹ����͵Ķ���typedef volatile uint8_t vu8��

//��3��app/lwipopts�ļ�����

//#ifndef _LWIPOPTS_H_
//#define _LWIPOPTS_H_
//#define SYS_LIGHTWEIGHT_PROT  0
////NO_SYS==1:��ʹ�ò���ϵͳ
//#define NO_SYS          1      //��ʹ��UCOS����ϵͳ
//#define MEM_ALIGNMENT      4      //ʹ��4�ֽڶ���ģʽ
////heap�ڴ�Ĵ�С,�����Ӧ�����д������ݷ��͵Ļ����ֵ������ô�һ��
//#define MEM_SIZE        10*1024    //�ڴ�Ѵ�С
////memp�ṹ��pbuf����,���Ӧ�ô�ROM���߾�̬�洢�����ʹ�������ʱ,���ֵӦ�����ô�һ��
//#define MEMP_NUM_PBUF      10
//#define MEMP_NUM_UDP_PCB    6      //UDPЭ����ƿ�(PCB)����.ÿ���UDP"����"��Ҫһ��PCB
//#define MEMP_NUM_TCP_PCB    10      //ͬʱ���������TCP����
//#define MEMP_NUM_TCP_PCB_LISTEN  6    //�ܹ�������TCP��������
//#define MEMP_NUM_TCP_SEG    20      //���ͬʱ�ڶ����е�TCP������
//#define MEMP_NUM_SYS_TIMEOUT  5      //�ܹ�ͬʱ�����timeout����
////Pbufѡ��
////PBUF_POOL_SIZE:pbuf�ڴ�ظ���
//#define PBUF_POOL_SIZE    10
////PBUF_POOL_BUFSIZE:ÿ��pbuf�ڴ�ش�С
//#define PBUF_POOL_BUFSIZE  1500
////TCPѡ��
//#define LWIP_TCP      1          //Ϊ1��ʹ��TCP
//#define TCP_TTL        255        //����ʱ��
////��TCP�����ݶγ�������ʱ�Ŀ���λ,���豸���ڴ��С��ʱ�����ӦΪ0
//#define TCP_QUEUE_OOSEQ    0
////���TCP�ֶ�
//#define TCP_MSS        ( 1500-40 )    //TCP_MSS = (MTU - IP��ͷ��С - TCP��ͷ��С
////TCP���ͻ�������С(bytes)
//#define TCP_SND_BUF      ( 4*TCP_MSS )
////TCP_SND_QUEUELEN: TCP���ͻ�������С(pbuf).���ֵ��СΪ(2 * TCP_SND_BUF/TCP_MSS) 
//#define TCP_SND_QUEUELEN  ( 4* TCP_SND_BUF/TCP_MSS )
////TCP���ʹ���
//#define TCP_WND        ( 2*TCP_MSS )
////ICMPѡ��
//#define LWIP_ICMP      1        //ʹ��ICMPЭ��
////DHCPѡ��
////��ʹ��DHCPʱ��λӦ��Ϊ1,LwIP 0.5.1�汾��û��DHCP����
//#define LWIP_DHCP      1
////UDPѡ��
//#define LWIP_UDP      1        //ʹ��UDP����
//#define UDP_TTL        255        //UDP���ݰ�����ʱ��
////��̬ѡ��
//#define LWIP_STATS      0
//#define LWIP_PROVIDE_ERRNO  1
////SequentialAPIѡ��
////LWIP_NETCONN==1:ʹ��NETCON����(Ҫ��ʹ��api_lib.c)
//#define LWIP_NETCONN    0
////Socket APIѡ��
////LWIP_SOCKET==1:ʹ��Socket API(Ҫ��ʹ��sockets.c)
//#define LWIP_SOCKET      0
//#define LWIP_COMPAT_MUTEX  1
//#define LWIP_SO_RCVTIMEO  1        //ͨ��������Ա��������߳�
////Lwip����ѡ��
////#define LWIP_DEBUG  1            //����DEBUGѡ��
//#define ICMP_DEBUG    LWIP_DBG_OFF    //����/�ر�ICMPdebug
//#endif

//24.5.6 include/netif/ethernetif.h�ļ��޸�
//#ifndef _ETHERNETIF_H_
//#define _ETHERNETIF_H_
//#include "lwip/err.h"
//#include "lwip/netif.h"
////����������
//#define IFNAME0 'e'
//#define IFNAME1 'n'
//err_t ethernetif_init( struct netif *netif ) ;
//err_t ethernetif_input( struct netif *netif ) ;
//#endif

//24.5.7 netif/ethernetif.c�ļ��޸�
//#include "netif/ethernetif.h"
//#include "dm9000.h"
//#include "comm.h"
//#include "malloc.h"
//#include "netif/etharp.h"
//#include "string.h"
//static err_t low_level_init( struct netif *netif )
//{
//  netif->hwaddr_len = ETHARP_HWADDR_LEN ;        //����MAC��ַ����,Ϊ6���ֽ�
//  //��ʼ��MAC��ַ,�����������������豸MAC��ַ�ظ�
//  netif->hwaddr[ 0 ] = lwipdev.mac[ 0 ] ;
//  netif->hwaddr[ 1 ] = lwipdev.mac[ 1 ] ;
//  netif->hwaddr[ 2 ] = lwipdev.mac[ 2 ] ;
//  netif->hwaddr[ 3 ] = lwipdev.mac[ 3 ] ;
//  netif->hwaddr[ 4 ] = lwipdev.mac[ 4 ] ;
//  netif->hwaddr[ 5 ] = lwipdev.mac[ 5 ] ;
//  netif->mtu = 1500 ;                  //��������䵥Ԫ,����������㲥��ARP����
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
//  p = low_level_input( netif ) ;                    //����low_level_input������������
//  if( p==NULL )
//    return ERR_MEM ;
//  err = netif->input( p, netif );                                    //����netif�ṹ���е�input�ֶ�(һ������)���������ݰ�
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
//  netif->hostname = "lwip" ;                    //��ʼ������
//#endif
//  netif->name[ 0 ] = IFNAME0 ;                  //��ʼ������netif��name�ֶ�
//  netif->name[ 1 ] = IFNAME1 ;                  //���ļ��ⶨ�����ﲻ�ù��ľ���ֵ
//  netif->output = etharp_output ;                  //IP�㷢�����ݰ�����
//  netif->linkoutput = low_level_output ;              //ARPģ�鷢�����ݰ�����
//  low_level_init( netif ) ;                      //�ײ�Ӳ����ʼ������
//  return ERR_OK ;
//}

//24.5.8 �����ļ��޸�
//��1��ͷ�ļ��޸ģ���Ҫ�ǽ�lwip/sys.h�޸�Ϊlwip/lwip_sys.h����Ϊ��������ֲ��ʱ��lwip/core/sys.c��lwip/include/lwipĿ¼�µ�һ���ļ����ƴ�sys��Ϊ��lwip_sys�����Ե��³������õ�ͷ�ļ�Ҳ��Ҫ�޸ģ���Ҫ�޸ĵ�ͷ�ļ��У�timers.c��init.c��lwip_sys.c��mem.c��pbuf.c��memp.c��

//��2��memp.c�ļ��޸�

//���޸�memp_memory�Ķ��壬֮ǰ�Ķ�����λ��170�е���ô���д��롣

//static u8_t memp_memory[MEM_ALIGNMENT - 1

//#define LWIP_MEMPOOL(name,num,size,desc) + ( (num) * (MEMP_SIZE + MEMP_ALIGN_SIZE(size) ) )

//#include "lwip/memp_std.h"

//];

//���ǽ���δ������ε���������Ӷ���u8_t *memp_memory;

//�����memp_get_memorysize����

//��333�в������º�������

//u32_t memp_get_memorysize()

//{

//       u32_t length=0;

//       length=(

//                     MEM_ALIGNMENT-1 //ȫ�������� Ϊ����POOL������ڴ�ռ�

//                     //MEMP_SIZE��ʾ��Ҫ��ÿ��POOLͷ��Ԥ���Ŀռ�  MEMP_SIZE = 0

//                     #define LWIP_MEMPOOL(name,num,size,desc)+((num)*(MEMP_SIZE+MEMP_ALIGN_SIZE(size)))

//                     #include "lwip/memp_std.h"

//                     );

//       return length;

//}

//24.6 ��������д
//��������Ĳ��裬�����Ѿ��ɹ���ֲ��LWIP 1.4.1�汾����������ͨ����д����������ʼ��LWIP����LWIP��������

//��1����Ӷ�ʱ�������������������ͨ�ö�ʱ��3�����LWIP�Ķ�ʱ���ܡ�

//��tim.c�ļ���������´���

//#include "tim.h"
//extern u32 lwip_localtime;                    //lwip����ʱ�������,��λ:ms
//void TIM3_IRQHandler()
//{
//  //����ж�
//  if( TIM3->SR&0x0001 )
//    lwip_localtime +=10 ;                  //��10
//  TIM3->SR &= ~( 1<<0 ) ;                  //����жϱ�־λ
//}
//void TIM3_Init( u16 arr, u16 psc )
//{
//  RCC->APB1ENR |= 1<<1 ;                  //TIM3ʱ��ʹ��
//   TIM3->ARR = arr ;                    //�趨�������Զ���װֵ//�պ�1ms
//  TIM3->PSC = psc ;                    //Ԥ��Ƶ��7200,�õ�10Khz�ļ���ʱ��
//  TIM3->DIER |= 1<<0 ;                                        //��������ж�
//  TIM3->CR1 |= 0x01 ;                    //ʹ�ܶ�ʱ��3
//    NVIC_Init( 1, 3, TIM3_IRQn, 2 ) ;                //��2
//}

//��tim.h�ļ���������´���

//#ifndef _TIM_H_
//#define _TIM_H_
//#include "sys.h"
//void TIM3_Init( u16 arr, u16 psc ) ;                //��ʱ��3��ʼ��
//#endif

//��2��������������´���

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
//   STM32_Clock_Init( 9 ) ;                                        //ϵͳʱ������
//  SysTick_Init( 72 ) ;                          //��ʱ��ʼ��
//  USART1_Init( 72, 115200 ) ;                      //���ڳ�ʼ��Ϊ115200
//  LCD_Init() ;                            //��ʼ��LCD
//  TIM3_Init( 1000, 719 ) ;                        //��ʱ��3Ƶ��Ϊ100hz
//  my_mem_init( SRAMIN ) ;                      //��ʼ���ڲ��ڴ��
//  while( lwip_comm_init() ) ;                      //lwip��ʼ��
//  //�ȴ�DHCP��ȡ�ɹ�/��ʱ���
//  while( ( lwipdev.dhcpstatus!=2 )&&( lwipdev.dhcpstatus!=0xFF ) )
//  {
//    lwip_periodic_handle() ;                    //LWIP�ں���Ҫ��ʱ����ĺ���
//    lwip_pkt_handle() ;
//  }
//  POINT_COLOR=RED;
//  LCD_ShowString( 30, 110, "LWIP Init Successed" ) ;
//  //��ӡ��̬IP��ַ
//  if( lwipdev.dhcpstatus==2 )
//    sprintf( ( char* )buf, "DHCP IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  //��ӡ��̬IP��ַ
//  else
//    sprintf( ( char* )buf, "Static IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 130, buf ) ; 
//  //�õ�����
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

//UDPЭ�����
//UDPЭ����TCP/IPЭ��ջ�д����Э�飬��һ���򵥵��������ݱ���Э�飬�ڴ�����л���һ��TCPЭ�飬UDP���ṩ���ݰ����飬��װ���޷������ݰ��������򣬵����ķ��ͳ�ȥ֮���޷�֪���Ƿ�ȫ�������ĵ����������UDP������������Э�飬����������ԴС�������ٶȿ죬ͨ��������Ƶ����Ƶ����ͨ���ݴ����У�UDP���ݰ��ṹ����ͼ��ʾ��

//ͼƬ

//�˿ںű�ʾ���ͺͽ��ս��̣�UDPʹ�ö˿ں�Ϊ��ͬ��Ӧ�ñ������Ե����ݴ���ͨ����UDP��TCP���ǲ��ö˿ںŵ���ʽ��ͬһʱ�̶��Ӧ��ͬʱ���ͺͽ������ݣ������ݽ��շ���ͨ��Ŀ��˿ڽ������ݣ��е�����ֻ��ʹ��Ԥ��Ԥ����ע��ľ�̬�˿ڣ���һЩ�������ʹ��û�б�ע��Ķ�̬�˿ڣ�����UDP��ͷʹ�������ֽڴ�Ŷ˿ںţ����Զ˿ڵ���Ч��Χ0~65535��һ�㣬����49151�Ķ˿ںŶ�����̬�˿ڡ�

//���ݰ��ĳ���ָ���ǰ�����ͷ�����ݲ������ڵ����ֽ��������ڰ�ͷ�ĳ��ȹ̶����������������Ҫ���ڼ���ɱ䳤�ȵ����ݲ��֣����ݰ�����󳤶ȸ��ݲ�������ѡ��������˵��������ͷ���ڵ����ݱ�����󳤶�Ϊ65535�ֽڡ�

//UDPͨ����ͷ�е�У�������֤���ݵ������ԣ�У������������ݷ��ͷ�ͨ��������㷨����������ݵ����շ�֮����Ҫ���¼��㣬���ĳ����������������б��۸Ļ�ĳ��ԭ���𻵣���ô���ͷ��ͽ��շ���У��;ͻ᲻һ�£���ˣ�UDPЭ����м�ⱨ���Ƿ�����������

//udp.c��udp.h�������ļ����Ǹ���ʵ��UDP����Э����ļ�����UDP���Ĵ����йصĺ���֮��Ĺ�ϵ����ͼ��ʾ��

//ͼƬ

//LWIPЭ����API��̷�ʽ�ǻ��ڻص����Ƶģ������ǳ�ʼ��Ӧ�õ�ʱ�����Ϊ�ں��в�ͬ���¼�ע�������Ӧ�Ļص�����������Ӧ���¼���������Щ�ص������ͻᱻ���ã�udp.c�г��õ�API���ܺ������±���ʾ��

//API����

//��������

//udp_new

//�½�һ��UDP��PCB��

//udp_remove

//��һ��PCB���ƿ��������ɾ�������ͷ�������ƿ���ڴ�

//udp_bind

//ΪUDP��PCN���ƿ��һ������IP��ַ�Ͷ˿ں�

//udp_connect

//���ӵ�ָ��IP��ַ������ָ���˿���

//udp_disconnent

//�Ͽ����ӣ������ƿ�����Ϊ������״̬

//udp_send

//ͨ��һ��PCB���ƿ鷢������

//udp_recv

//��Ҫ����һ���ص������������ܵ����ݵ�ʱ�򱻵���

//25.2 Ӧ�ñ�д
//��LWIP/app/udp_demoĿ¼�´���udp_demo.c��udp_demo.h�ļ���

//25.2.1 udp_demo.c�����д
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
////UDP ����ȫ��״̬��Ǳ���
////bit6:���ݽ���״̬
////bit5:����״̬
//u8 udp_demo_flag;
////����Զ��IP��ַ
//void udp_demo_set_remoteip()
//{
//  u8 *tbuf ;
//  LCD_Clear( WHITE ) ;
//  POINT_COLOR = RED ;
//  tbuf = mymalloc( SRAMIN, 100 ) ;                                  //�����ڴ�
//  if( tbuf==NULL )
//    return ;
//  //ǰ����IP���ֺ�DHCP�õ���IPһ��
//  lwipdev.remoteip[ 0 ] = lwipdev.ip[ 0 ] ;
//  lwipdev.remoteip[ 1 ] = lwipdev.ip[ 1 ] ;
//  lwipdev.remoteip[ 2 ] = lwipdev.ip[ 2 ] ;
//  lwipdev.remoteip[ 3 ] = 113 ;
//  sprintf( ( char* )tbuf, "Remote IP:%d.%d.%d.", lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2] ) ;
//  LCD_ShowString( 30, 150, tbuf ) ;                //Զ��IP
//  myfree( SRAMIN, tbuf ) ;
//}
//// UDP���ջص�����
//u8 udp_demo_recvbuf[ UDP_DEMO_RX_BUFSIZE ] ;            //UDP�������ݻ�����
//void udp_demo_recv( void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip_addr *addr, u16_t port )
//{
//  u32 data_len=0 ;
//  struct pbuf *q ;
//  //���յ���Ϊ�յ�����ʱ
//  if( p!=NULL )
//  {
//    memset( udp_demo_recvbuf, 0, UDP_DEMO_RX_BUFSIZE ) ;    //���ݽ��ջ���������
//    //����������pbuf����
//    for( q=p; q!=NULL; q=q->next )
//    {
//      //��������
//      if( q->len>( UDP_DEMO_RX_BUFSIZE-data_len ) )
//        memcpy( udp_demo_recvbuf+data_len, q->payload, UDP_DEMO_RX_BUFSIZE-data_len ) ;
//      else
//        memcpy( udp_demo_recvbuf+data_len, q->payload, q->len ) ;
//      data_len += q->len ;
//      //����TCP�ͻ��˽�������,����
//      if( data_len>UDP_DEMO_RX_BUFSIZE )
//        break ;  
//    }
//    upcb->remote_ip = *addr ;                  //��¼Զ��������IP��ַ
//    upcb->remote_port = port ;                //��¼Զ�������Ķ˿ں�
//    lwipdev.remoteip[ 0 ] = upcb->remote_ip.addr&0xFF ;      //IADDR4
//    lwipdev.remoteip[ 1 ] = ( upcb->remote_ip.addr>>8 )&0xFF ;  //IADDR3
//    lwipdev.remoteip[ 2 ] = ( upcb->remote_ip.addr>>16 )&0xFF ;  //IADDR2
//    lwipdev.remoteip[ 3 ] = ( upcb->remote_ip.addr>>24 )&0xFF ;  //IADDR1 
//    udp_demo_flag |= 1<<6 ;                  //��ǽ��յ�������
//    pbuf_free( p ) ;                      //�ͷ��ڴ�
//  }
//  else
//  {
//    udp_disconnect( upcb ) ;
//    LCD_Clear( WHITE ) ;                    //����
//    udp_demo_flag &= ~( 1<<5 ) ;                //������ӶϿ�
//  }
//}
//// UDP��������������
//const u8 *tcp_demo_sendbuf="STM32F103 UDP send data\r\n";
//void udp_demo_senddata( struct udp_pcb *upcb )
//{
//  struct pbuf *ptr ;
//  ptr = pbuf_alloc( PBUF_TRANSPORT, strlen( ( char* )tcp_demo_sendbuf ), PBUF_POOL ) ;//�����ڴ�
//  if( ptr )
//  {
//    ptr->payload = ( void* )tcp_demo_sendbuf ;
//    udp_send( upcb, ptr ) ;                    //udp�������� 
//    pbuf_free( ptr ) ;                                        //�ͷ��ڴ�
//  }
//}
////�ر�UDP����
//void udp_demo_connection_close( struct udp_pcb *upcb )
//{
//  udp_disconnect( upcb ) ;
//  udp_remove( upcb ) ;                      //�Ͽ�UDP���� 
//  udp_demo_flag &= ~( 1<<5 ) ;                  //������ӶϿ�
//  LCD_Clear( WHITE ) ;                      //����
//}
//// UDP����
//void udp_demo_test()
//{
//   err_t err ;
//  struct udp_pcb *udppcb ;                    //����һ��TCP���������ƿ�
//  struct ip_addr rmtipaddr ;                                      //Զ��ip��ַ
//  u8 *tbuf ;
//  u8 res=0 ;
//  udp_demo_set_remoteip() ;                    //��ѡ��IP
//  LCD_Clear( WHITE ) ;                      //����
//  tbuf = mymalloc( SRAMIN, 200 ) ;                //�����ڴ�
//  //�ڴ�����ʧ����,ֱ���˳�
//  if( tbuf==NULL )
//    return ;
//  sprintf( ( char* )tbuf, "Local IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 150, tbuf ) ;                //������IP
//  sprintf( ( char* )tbuf, "Remote IP:%d.%d.%d.%d", lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2], lwipdev.remoteip[3] ) ;
//  LCD_ShowString( 30, 170, tbuf ) ;                //Զ��IP
//  sprintf( ( char* )tbuf, "Remote Port:%d", UDP_DEMO_PORT ) ;
//  LCD_ShowString( 30, 190, tbuf ) ;                //�ͻ��˶˿ں�
//  LCD_ShowString( 30, 210, "STATUS:Disconnected" ) ;
//  udppcb = udp_new() ;
//  //�����ɹ�
//  if( udppcb )
//  { 
//    IP4_ADDR( &rmtipaddr, lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2], lwipdev.remoteip[3] ) ;
//    err = udp_connect( udppcb, &rmtipaddr, UDP_DEMO_PORT ) ;  //UDP�ͻ������ӵ�ָ��IP��ַ�Ͷ˿�
//    if( err==ERR_OK )
//    {
//      err = udp_bind( udppcb, IP_ADDR_ANY, UDP_DEMO_PORT ) ; //�󶨱���IP��ַ��˿ں�
//      //�����
//      if( err==ERR_OK )
//      {
//        udp_recv( udppcb, udp_demo_recv, NULL ) ;      //ע����ջص�����
//        LCD_ShowString( 30, 210, "STATUS:Connected   " ) ;  //�����������
//        udp_demo_flag |= 1<<5 ;              //����Ѿ�������
//        LCD_ShowString( 30, 230, "Receive Data:" ) ;      //��ʾ��Ϣ
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
//    //�Ƿ��յ�����
//    if( udp_demo_flag&1<<6 )
//    {
//      LCD_ShowString( 30, 250, udp_demo_recvbuf ) ;      //��ʾ���յ�������
//      udp_demo_senddata( udppcb ) ;              //��������
//      udp_demo_flag &= ~( 1<<6 ) ;              //��������Ѿ���������
//    } 
//    lwip_periodic_handle() ;
//    lwip_pkt_handle() ;
//    delay_ms( 2 ) ;
//  }
//  udp_demo_connection_close( udppcb ) ;
//  myfree( SRAMIN, tbuf ) ;
//}

//25.2.2 udp_demo.h�����д
//#ifndef _UDP_DEMO_H_
//#define _UDP_DEMO_H_
//#include "sys.h"
//#define UDP_DEMO_RX_BUFSIZE  2000              //����udp���������ݳ��� 
//#define UDP_DEMO_PORT      8089              //����udp���ӵĶ˿� 
//void udp_demo_test( void ) ;                    //UDP����
//#endif

//25.2.3 �����������д
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
//   STM32_Clock_Init( 9 ) ;                        //ϵͳʱ������
//  SysTick_Init( 72 ) ;                          //��ʱ��ʼ��
//  USART1_Init( 72, 115200 ) ;                      //���ڳ�ʼ��Ϊ115200
//  LCD_Init() ;                            //��ʼ��LCD
//  TIM3_Init( 1000, 719 ) ;                        //��ʱ��3Ƶ��Ϊ100hz
//  my_mem_init( SRAMIN ) ;                      //��ʼ���ڲ��ڴ��
//  while( lwip_comm_init() ) ;                      //lwip��ʼ��
//  //�ȴ�DHCP��ȡ�ɹ�/��ʱ���
//  while( ( lwipdev.dhcpstatus!=2 )&&( lwipdev.dhcpstatus!=0xFF ) )
//  {
//    lwip_periodic_handle() ;                    //LWIP�ں���Ҫ��ʱ����ĺ���
//    lwip_pkt_handle() ;
//  }
//  POINT_COLOR=RED;
//  LCD_ShowString( 30, 110, "LWIP Init Successed" ) ;
//  //��ӡ��̬IP��ַ
//  if( lwipdev.dhcpstatus==2 )
//    sprintf( ( char* )buf, "DHCP IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  //��ӡ��̬IP��ַ
//  else
//    sprintf( ( char* )buf, "Static IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 130, buf ) ; 
//  //�õ�����
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

//TCPЭ�����
//TCP��һ���������ӵģ��ɿ��أ�����IP�Ĵ����Э�飬�������Ӿ���ζ������ʵ��TCP��Ӧ���ڽ������ݽ�����ʱ������Ƚ���һ��TCP���ӣ���Ӧ�ò���TCP�㷢�����ڴ���ģ���8λ�ֽڱ�ʾ����������TCP�Ȱ��������ָ���ʵ����ȵı��ĶΣ������εĳ���MSSͨ���ܼ�������ӵ������������·�������䵥ԪMTU���ƣ�֮��TCP�Ű����ݰ�����IP�㣬ͨ�����������ݴ��͸����ն˵�TCP�㡣Ϊ�˱�֤���Ĵ���Ŀɿ��ԣ����ÿ����һ����ţ�ͬʱ���Ҳ��֤�˴��͵����ն˵����ݱ����ܱ�����˳����գ�Ȼ����ն˶Գɹ����յı��ķ���һ����Ӧ��ȷ��ACK��������Ͷ��ں����ʱ��RTT��û���յ�ȷ�ϣ���ô��Ӧ�����ݾͻᱻ�ش�TCP��������ȷ�ԺͺϷ����ϲ���һ��У��ͺ������ⶨ�����Ƿ��д����ڷ��ͺͽ���ʱ���������У��ͣ��ڱ�֤�ɿ����ϣ����ڴ�����δ��ȷ�ϵķ�����Ҫ�ش����ģ���ӵ�������ϣ�����TCPӵ�������㷨��

//TCP���ݱ���װ��һ��IP���ݱ����У�IP���ݱ��Ľṹ����ͼ��ʾ��

//ͼƬ

//TCP�������ݸ�ʽ��û��ѡ�������£�ͨ����20���ֽڣ����ݽṹ����ͼ��ʾ

//ͼƬ

//��1��Դ�˿ںź�Ŀ�Ķ˿ں�����Ѱ�ҷ��Ͷ˺ͽ��ն˵�Ӧ�ý��̣������UDP������ͬ��������ֵ����IP�ײ��е�ԴIP��ַ��Ŀ��IP��ַΨһȷ����һ��TCP���ӡ�

//��2�����к��ֶ�������ʶ��TCP���Ͷ���TCP���ն˷��͵������ֽ��������ڱ�ʾ��������Ķ��еĵ�һ�������ֽڣ�������һ���µ�����ʱ�����ֱ����е�SYN��־��1��������ֱ����е�����ֶ�Ϊ���ѡ��ĳ�ʼ���ISN��Initial Sequence Number���������ӽ������Ժ��ͷ�Ҫ���͵ĵ�һ���ֽ����ΪISN+1��

//��3��ȷ�Ϻ��ֶ�ֻ����ACKΪ1��ʱ������ã�ȷ�Ϻ��а�������ȷ�ϵ�һ�����������յ�����һ����ţ�ȷ�Ϻ�������һ�γɹ����յ��������ֽ����к��ϼ�1�������ϴν��ճɹ����յ��Է����������������ΪX����ô���ص�ȷ�Ϻž�Ӧ��ΪX+1

//��4��ͷ�������ֳ�Ϊ�ײ����ȣ��ײ������и������ײ��ĳ��ȣ���4���ֽ�Ϊ��λ������ֶ���4bit�����TCP�����60�ֽڵ��ײ������û���κε�ѡ���ֶΣ��������ײ�������20�ֽڣ�TCP�ײ��л���6����־λ����6����־λ���±���ʾ��

//��־λ

//˵��

//URG

//��1ʱ��ʾ����ָ����Ч

//ACK

//��1ʱ��ʾȷ������ֶ���Ч

//PSH

//��1��ʾ���շ�Ӧ�þ��콫������Ķν���Ӧ�ò�

//RST

//��1��ʾ�ؽ�����

//SYN

//���ڷ�������

//FIN

//���Ͷ���ɷ���������ֹ����

//��5�����ڳߴ�Ҳ���Ǵ��ڴ�С��������д��Ӧ��ֵ��֪ͨ�Է��������յ��ֽ��������ڴ�С�ֶ���TCP�������ƵĹؼ��ֶΣ����ڴ�С��һ��2���ֽڵ��ֶΣ���˴��ڴ�С���Ϊ65535���ֽڡ�

//��6��16λУ��ͺ�UDP��У��ͼ���ԭ����ͬ������һ��ǿ���Ե��ֶΣ�У��͸�������TCP���ĶΡ�

//��7������ָ��ֻ����URG��1ʱ��Ч����һ����ƫ������������ֶ��е�ֵ��ӱ�ʾ�����������һ���ֽڵ���š�

//tcp.c��tcp.h��tcp_in.c��tcp_out.c��LWIP�й���TCPЭ����ļ���TCP���к����Ĺ�ϵ����ͼ��ʾ��

//ͼƬ

//���õ�TCPЭ���API�������±���ʾ��

//��������

//API����

//����

//����TCP����

//tcp_new()

//����һ��TCP��PCB���ƿ�

//tcp_bind()

//ΪTCP��PCB���ƿ��һ������IP��ַ�Ͷ˿ں�

//tcp_listen()

//��ʼTCP��PCB����

//tcp_accept()

//���ƿ�accept�ֶ�ע��Ļص�����������������ʱ������

//tcp_accepted()

//֪ͨLWIPЭ��ջһ��TCP���ӱ�������

//tcp_conect()

//����Զ������

//TCP���ݷ���

//tcp_write()

//����һ�����Ĳ����ڿ��ƿ�ķ��Ͷ��л�����

//tcp_sent()

//���ƿ�sent�ֶ�ע��Ļص����������ݷ��ͳɹ��󱻻ص�

//tcp_output()

//�����ͻ�������е����ݷ��ͳ�ȥ

//TCP���ݽ���

//tcp_recv()

//���ƿ�recv�ֶ�ע��Ļص������������յ�������ʱ������

//tcp_recved()

//�������������ݺ�һ��Ҫ�������������֪ͨ�ں˸��½��մ���

//������ѯ

//tcp_poll()

//���ƿ�poll�ֶ�ע��Ļص��������ú��������Ե���

//�رպ���ֹ����

//tcp_close()

//�ر�TCP����

//tcp_err()

//���ƿ�err�ֶ�ע��Ļص���������������ʱ������

//tcp_abort()

//�ж�TCP����

//26.2 Ӧ�ñ�д
//26.2.1 tcp_client.c�����д
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
////TCP Client ����ȫ��״̬��Ǳ���
////bit7:0,û������Ҫ����;1,������Ҫ����
////bit6:0,û���յ�����;1,�յ�������
////bit5:0,û�������Ϸ�����;1,�����Ϸ�������
////bit4~0:����
//u8 tcp_client_flag;   
////����Զ��IP��ַ
//void tcp_client_set_remoteip()
//{
//  u8 *tbuf;
//  tbuf=mymalloc( SRAMIN, 100 ) ;                      //�����ڴ�
//  if( tbuf==NULL )
//    return ;
//  //ǰ����IP���ֺ�DHCP�õ���IPһ��
//  lwipdev.remoteip[ 0 ] = lwipdev.ip[ 0 ] ;
//  lwipdev.remoteip[ 1 ] = lwipdev.ip[ 1 ] ;
//  lwipdev.remoteip[ 2 ] = lwipdev.ip[ 2 ] ;
//  lwipdev.remoteip[ 3 ] = 113 ;
//  sprintf( ( char* )tbuf, "Remote IP:%d.%d.%d.", lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2] ) ;
//  LCD_ShowString( 30, 150, tbuf ) ;                    //Զ��IP
//  myfree( SRAMIN, tbuf ) ;
//}
////�ر��������������
//void tcp_client_connection_close( struct tcp_pcb *tpcb, struct tcp_client_struct *es )
//{
//  tcp_abort( tpcb ) ;                            //��ֹ����,ɾ��pcb���ƿ�
//  tcp_arg( tpcb, NULL ) ;
//  tcp_recv( tpcb, NULL ) ;
//  tcp_sent( tpcb, NULL ) ;
//  tcp_err( tpcb, NULL ) ;
//  tcp_poll( tpcb, NULL, 0 );
//  if( es )
//    mem_free( es ) ;
//  tcp_client_flag &= ~( 1<<5 ) ;                      //������ӶϿ���
//}
//// tcp_recv�����Ļص�����
//u8 tcp_client_recvbuf[ TCP_CLIENT_RX_BUFSIZE ] ;                //�������ݻ�����
//err_t tcp_client_recv( void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err )
//{
//  u32 data_len=0 ;
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
//// tcp_err�����Ļص�����
//void tcp_client_error( void *arg, err_t err )
//{

//}
////��������
//void tcp_client_senddata( struct tcp_pcb *tpcb, struct tcp_client_struct *es )
//{
//  struct pbuf *ptr ; 
//   err_t wr_err = ERR_OK ;
//  while( ( wr_err==ERR_OK )&&( es->p )&&( es->p->len<=tcp_sndbuf( tpcb ) ) )
//  {
//    ptr = es->p ;
//    wr_err = tcp_write( tpcb, ptr->payload, ptr->len, 1 ) ;          //���ݼ��뵽���ͻ��������
//    if( wr_err==ERR_OK )
//    {
//      es->p = ptr->next ;                      //ָ����һ��pbuf
//      //pbuf��ref��һ
//      if( es->p )
//        pbuf_ref( es->p );
//      pbuf_free( ptr ) ;                        //�ͷ�ptr
//    }
//    else if( wr_err==ERR_MEM )
//      es->p = ptr ;
//    tcp_output( tpcb ) ;                        //���ͻ�������е����ݷ���
//  }
//}
//// tcp_sent�Ļص�����(��Զ�˽��յ�ACK��������)
//err_t tcp_client_sent( void *arg, struct tcp_pcb *tpcb, u16_t len )
//{
//  struct tcp_client_struct *es ;
//  LWIP_UNUSED_ARG( len ) ;
//  es = ( struct tcp_client_struct* )arg ;
//  if( es->p )
//    tcp_client_senddata( tpcb, es ) ;                    //��������
//  return ERR_OK ;
//}
//// tcp_poll�Ļص�����
//const u8 *tcp_client_sendbuf = "STM32F103 TCP Client send data\r\n" ;      //TCP������������������
//err_t tcp_client_poll( void *arg, struct tcp_pcb *tpcb )
//{
//  err_t ret_err ;
//  struct tcp_client_struct *es ; 
//  es = ( struct tcp_client_struct* )arg ;
//  //���Ӵ��ڿ��п��Է�������
//  if( es!=NULL )
//  {
//    //�ж��Ƿ�������Ҫ����
//    if( tcp_client_flag&( 1<<7 ) )
//    {
//      es->p = pbuf_alloc( PBUF_TRANSPORT, strlen( ( char* )tcp_client_sendbuf ), PBUF_POOL ) ; 
//      pbuf_take( es->p, ( char* )tcp_client_sendbuf, strlen( ( char* )tcp_client_sendbuf ) ) ; 
//      tcp_client_senddata( tpcb, es ) ;                  //�����ݷ��ͳ�ȥ
//      tcp_client_flag &= ~( 1<<7 ) ;                  //������ݷ��ͱ�־
//      //�ͷ��ڴ�
//      if( es->p )
//        pbuf_free( es->p ) ;
//    }
//    else if( es->state==ES_TCPCLIENT_CLOSING )
//       tcp_client_connection_close( tpcb, es ) ;              //�ر�TCP����
//    ret_err = ERR_OK ;
//  }
//  else
//  { 
//    tcp_abort( tpcb ) ;                          //��ֹ����,ɾ��pcb���ƿ�
//    ret_err = ERR_ABRT ;
//  }
//  return ret_err ;
//}
////���ӽ�������ûص�����
//err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
//{
//  struct tcp_client_struct *es=NULL;  
//  if(err==ERR_OK)   
//  {
//    es = ( struct tcp_client_struct* )mem_malloc( sizeof( struct tcp_client_struct ) ) ; //�����ڴ�
//    //�ڴ�����ɹ�
//    if( es )
//    {
//       es->state = ES_TCPCLIENT_CONNECTED ;            //״̬Ϊ���ӳɹ�
//      es->pcb = tpcb ;
//      es->p = NULL ;
//      tcp_arg( tpcb, es ) ;                      //����tpcb��callback_arg
//      tcp_recv( tpcb, tcp_client_recv ) ;                  //��ʼ��tcp_recv�ص�����
//      tcp_err( tpcb, tcp_client_error ) ;                  //��ʼ��tcp_err()�ص�����
//      tcp_sent( tpcb, tcp_client_sent ) ;                  //��ʼ��tcp_sent�ص�����
//      tcp_poll( tpcb, tcp_client_poll, 1 ) ;                //��ʼ��tcp_poll�ص�����
//       tcp_client_flag |= 1<<5 ;                    //������ӵ���������
//      err = ERR_OK ;
//    }
//    else
//    {
//      tcp_client_connection_close( tpcb, es ) ;              //�ر�����
//      err = ERR_MEM ;                        //�����ڴ�������
//    }
//  }
//  else
//    tcp_client_connection_close( tpcb, 0 ) ;                //�ر�����
//  return err ;
//}
////�ͻ�������
//void tcp_client_test()
//{
//   struct tcp_pcb *tcppcb ;                        //����һ��TCP���������ƿ�
//  struct ip_addr rmtipaddr ;                        //Զ��ip��ַ
//  u8 *tbuf ;
//  u8 res=0 ;    
//  u8 t=0 ; 
//  u8 connflag=0 ;                            //���ӱ��
//  tcp_client_set_remoteip() ;                        //��ѡ��IP
//  tbuf = mymalloc( SRAMIN, 200 ) ;                    //�����ڴ�
//  //�ڴ�����ʧ����,ֱ���˳�
//  if( tbuf==NULL )
//    return ;
//  sprintf( ( char* )tbuf, "Local IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 130, tbuf ) ;                    //������IP
//  sprintf( ( char* )tbuf, "Remote IP:%d.%d.%d.%d", lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2], lwipdev.remoteip[3] ) ;
//  LCD_ShowString( 30, 150, tbuf ) ;                    //Զ��IP
//  sprintf( ( char* )tbuf, "Remote Port:%d", TCP_CLIENT_PORT ) ;          //�ͻ��˶˿ں�
//  LCD_ShowString( 30, 170, tbuf ) ;
//  LCD_ShowString( 30, 190, "STATUS:Disconnected" ) ;
//  tcppcb = tcp_new() ;                          //����һ���µ�pcb
//  //�����ɹ�
//  if( tcppcb )
//  {
//  IP4_ADDR( &rmtipaddr, lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2], lwipdev.remoteip[3] ) ;
//    tcp_connect( tcppcb, &rmtipaddr, TCP_CLIENT_PORT, tcp_client_connected ) ; 
//   }
//  else
//    res = 1 ;
//  while( res==0 )
//  {
//    //�Ƿ��յ�����
//    if( tcp_client_flag&1<<6 )
//    {
//      LCD_ShowString( 30, 230, tcp_client_recvbuf ) ;            //��ʾ���յ�������
//      tcp_client_flag |= 1<<7 ;                    //���Ҫ��������
//      tcp_client_flag &= ~( 1<<6 ) ;                  //��������Ѿ���������
//    }
//    //�Ƿ�������
//    if( tcp_client_flag&1<<5 )
//    {
//      if( connflag==0 )
//      { 
//        LCD_ShowString( 30, 190, "STATUS:Connected   " ) ;
//        LCD_ShowString( 30, 210, "Receive Data:" ) ;
//        connflag = 1 ;                        //���������
//      }
//    }
//    else if( connflag )
//    {
//       LCD_ShowString( 30, 190, "STATUS:Disconnected" ) ;
//      connflag = 0 ;                          //������ӶϿ���
//    } 
//    lwip_periodic_handle() ;
//    lwip_pkt_handle() ;
//    delay_ms( 2 ) ;
//    t ++ ;
//    if( t==200 )
//    {
//      //δ������,��������
//      if( ( connflag==0 )&&( ( tcp_client_flag&1<<5 )==0 ) )
//      { 
//        tcp_client_connection_close( tcppcb, 0 ) ;            //�ر�����
//        tcppcb = tcp_new() ;                    //����һ���µ�pcb
//        //�����ɹ�
//        if( tcppcb )
//          tcp_connect( tcppcb, &rmtipaddr, TCP_CLIENT_PORT, tcp_client_connected ) ; 
//      }
//      t = 0 ;
//    }    
//  }
//  tcp_client_connection_close( tcppcb, 0 ) ;                  //�ر�TCP Client����
//  myfree( SRAMIN, tbuf ) ;
//}

//26.2.2 tcp_client.h�����д
//#ifndef _TCP_CLIENT_H_
//#define _TCP_CLIENT_H_
//#include "sys.h"
//#include "lwip/tcp.h"
//#include "lwip/pbuf.h"
//#define TCP_CLIENT_RX_BUFSIZE  1500            //���������ݳ���
//#define TCP_CLIENT_TX_BUFSIZE  200              //��������ݳ���
//#define LWIP_SEND_DATA      0x80            //�����ݷ���
//#define  TCP_CLIENT_PORT    8087            //Զ�˶˿�
////tcp����������״̬
//enum tcp_client_states
//{
//  ES_TCPCLIENT_NONE = 0,    //û������
//  ES_TCPCLIENT_CONNECTED,  //���ӵ��������� 
//  ES_TCPCLIENT_CLOSING,    //�ر�����
//};
////LWIP�ص�����ʹ�õĽṹ��
//struct tcp_client_struct
//{
//  u8 state;            //��ǰ����״
//  struct tcp_pcb *pcb;      //ָ��ǰ��pcb
//  struct pbuf *p;        //ָ�����/�����pbuf
//};  
//void tcp_client_test( void ) ;                    //TCP Client���Ժ���
//#endif

//26.2.3 �����������д
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
//   STM32_Clock_Init( 9 ) ;                        //ϵͳʱ������
//  SysTick_Init( 72 ) ;                          //��ʱ��ʼ��
//  USART1_Init( 72, 115200 ) ;                      //���ڳ�ʼ��Ϊ115200
//  LCD_Init() ;                            //��ʼ��LCD
//  TIM3_Init( 1000, 719 ) ;                        //��ʱ��3Ƶ��Ϊ100hz
//  my_mem_init( SRAMIN ) ;                      //��ʼ���ڲ��ڴ��
//  while( lwip_comm_init() ) ;                      //lwip��ʼ��
//  //�ȴ�DHCP��ȡ�ɹ�/��ʱ���
//  while( ( lwipdev.dhcpstatus!=2 )&&( lwipdev.dhcpstatus!=0xFF ) )
//  {
//    lwip_periodic_handle() ;                    //LWIP�ں���Ҫ��ʱ����ĺ���
//    lwip_pkt_handle() ;
//  }
//  POINT_COLOR=RED;
//  LCD_ShowString( 30, 110, "LWIP Init Successed" ) ;
//  //��ӡ��̬IP��ַ
//  if( lwipdev.dhcpstatus==2 )
//    sprintf( ( char* )buf, "DHCP IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  //��ӡ��̬IP��ַ
//  else
//    sprintf( ( char* )buf, "Static IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 130, buf ) ; 
//  //�õ�����
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


//STM32ѧϰ�ʼ�27����������ͨ��ʵ��4֮TCP������
//ԭ�� ���Ӽ���԰�� ��С���ʼ� 2021-04-01 00:00
//�й���TCPЭ���֪ʶ����һ���Ѿ��й���������������ֱ��ʹ��API��ʵ��TCP������ģʽ��

//27.1 ʵ������
//27.1.1 tcp_server.c�����д
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
////TCP Server ����ȫ��״̬��Ǳ���
////bit7:0,û������Ҫ����;1,������Ҫ����
////bit6:0,û���յ�����;1,�յ�������.
////bit5:0,û�пͻ���������;1,�пͻ�����������.
////bit4~0:����
//u8 tcp_server_flag;
////�ر�tcp����
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
//  tcp_server_flag &= ~( 1<<5 ) ;                      //������ӶϿ���
//}
//// tcp_recv�����Ļص�����
//u8 tcp_server_recvbuf[ TCP_SERVER_RX_BUFSIZE ] ;                //TCP Server�������ݻ�����
//err_t tcp_server_recv( void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err )
//{
//  err_t ret_err ;
//  u32 data_len = 0 ;
//  struct pbuf *q ;
//    struct tcp_server_struct *es ;
//  LWIP_ASSERT( "arg != NULL", arg != NULL ) ;
//  es = ( struct tcp_server_struct* )arg ;
//  //�ӿͻ��˽��յ�������
//  if( p==NULL )
//  {
//    es->state = ES_TCPSERVER_CLOSING ;                //��Ҫ�ر�TCP������
//    es->p = p ; 
//    ret_err = ERR_OK ;
//  }
//  //�ӿͻ��˽��յ�һ���ǿ�����,��������ĳ��ԭ��err!=ERR_OK
//  else if( err!=ERR_OK )
//  {
//    if( p )
//      pbuf_free( p ) ;                        //�ͷŽ���pbuf
//    ret_err = err ;
//  }
//  //��������״̬
//  else if( es->state==ES_TCPSERVER_ACCEPTED )
//  {
//    //����������״̬���ҽ��յ������ݲ�Ϊ��ʱ�����ӡ����
//    if( p!=NULL )
//    {
//      memset( tcp_server_recvbuf, 0, TCP_SERVER_RX_BUFSIZE ) ;      //���ݽ��ջ���������
//      //����������pbuf����
//      for( q=p; q!=NULL; q=q->next )
//      {
//        if( q->len>( TCP_SERVER_RX_BUFSIZE-data_len ) )
//          memcpy( tcp_server_recvbuf+data_len, q->payload, TCP_SERVER_RX_BUFSIZE-data_len ) ; 
//        else
//          memcpy(tcp_server_recvbuf+data_len, q->payload, q->len ) ;
//        data_len += q->len ;
//        //����TCP�ͻ��˽�������,����
//        if( data_len>TCP_SERVER_RX_BUFSIZE )
//          break ;
//      }
//      tcp_server_flag |= 1<<6 ;                    //��ǽ��յ�������
//      lwipdev.remoteip[ 0 ] = tpcb->remote_ip.addr&0xFF ;        //IADDR4
//      lwipdev.remoteip[ 1 ] = ( tpcb->remote_ip.addr>>8 )&0xFF ;    //IADDR3
//      lwipdev.remoteip[ 2 ] = ( tpcb->remote_ip.addr>>16 )&0xFF ;    //IADDR2
//      lwipdev.remoteip[ 3 ] = ( tpcb->remote_ip.addr>>24 )&0xFF ;    //IADDR1 
//       tcp_recved( tpcb, p->tot_len ) ;                  //���ڻ�ȡ��������
//      pbuf_free( p ) ;                        //�ͷ��ڴ�
//      ret_err = ERR_OK ;
//    }
//  }
//  //�������ر���
//  else
//  {
//    tcp_recved( tpcb, p->tot_len  );                    //���ڻ�ȡ��������
//    es->p=  NULL ;
//    pbuf_free( p ) ;                          //�ͷ��ڴ�
//    ret_err = ERR_OK ;
//  }
//  return ret_err ;
//}
//// tcp_err�����Ļص�����
//void tcp_server_error( void *arg, err_t err )
//{  
//  LWIP_UNUSED_ARG( err ) ;
//  printf( "tcp error:%x\r\n", ( u32 )arg ) ;
//  //�ͷ��ڴ�
//  if( arg!=NULL )
//    mem_free( arg ) ;
//}
////��������
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
//      es->p = ptr->next ;                      //ָ����һ��pbuf
//      //pbuf��ref��һ
//      if( es->p )
//        pbuf_ref( es->p ) ;
//      pbuf_free( ptr ) ;
//      tcp_recved( tpcb, plen ) ;                    //����tcp���ڴ�С
//    }
//    else if( wr_err==ERR_MEM )
//      es->p = ptr ;
//   }
//}
//// tcp_poll�Ļص�����
//const u8 *tcp_server_sendbuf = "STM32F103 TCP Server send data\r\n" ;      //TCP������������������
//err_t tcp_server_poll( void *arg, struct tcp_pcb *tpcb )
//{
//  err_t ret_err;
//  struct tcp_server_struct *es ; 
//  es = ( struct tcp_server_struct* )arg ;
//  if( es!=NULL )
//  {
//    //�ж��Ƿ�������Ҫ����
//    if( tcp_server_flag&( 1<<7 ) )
//    {
//      es->p = pbuf_alloc( PBUF_TRANSPORT, strlen( ( char* )tcp_server_sendbuf ), PBUF_POOL ) ; 
//      pbuf_take( es->p, ( char* )tcp_server_sendbuf, strlen( ( char* )tcp_server_sendbuf ) ) ;
//      tcp_server_senddata( tpcb, es ) ;                  //��ѯ��ʱ����Ҫ���͵�����
//      tcp_server_flag &= ~( 1<<7 ) ;                  //������ݷ��ͱ�־λ
//      if( es->p!=NULL )
//        pbuf_free( es->p ) ;                    //�ͷ��ڴ�  
//    }
//    //�رղ���
//    else if( es->state==ES_TCPSERVER_CLOSING )
//      tcp_server_connection_close( tpcb, es ) ;              //�ر�����
//    ret_err = ERR_OK ;
//  }
//  else
//  {
//    tcp_abort( tpcb ) ;                          //��ֹ����,ɾ��pcb���ƿ�
//    ret_err = ERR_ABRT ; 
//  }
//  return ret_err ;
//}
//// tcp_sent�Ļص�����(Զ���յ�ACK�źź�������)
//err_t tcp_server_sent( void *arg, struct tcp_pcb *tpcb, u16_t len )
//{
//  struct tcp_server_struct *es ;
//  LWIP_UNUSED_ARG( len ) ;
//  es = ( struct tcp_server_struct * )arg ;
//  if( es->p )
//    tcp_server_senddata( tpcb, es ) ;                    //��������
//  return ERR_OK ;
//}
////ǿ��ɾ�������Ͽ�ʱ��time wait
//extern void tcp_pcb_purge( struct tcp_pcb *pcb ) ;                //�� tcp.c����
//extern struct tcp_pcb *tcp_active_pcbs ;                    //�� tcp.c����
//extern struct tcp_pcb *tcp_tw_pcbs ;                      //�� tcp.c����
//void tcp_server_remove_timewait()
//{
//  struct tcp_pcb *pcb, *pcb2 ; 
//  while( tcp_active_pcbs!=NULL )
//  {
//    lwip_periodic_handle() ;                      //������ѯ
//    lwip_pkt_handle() ;
//     delay_ms( 10 ) ;                          //�ȴ�tcp_active_pcbsΪ��  
//  }
//  pcb = tcp_tw_pcbs ;
//  //����еȴ�״̬��pcbs
//  while( pcb!=NULL )
//  {
//    tcp_pcb_purge( pcb ) ;
//    tcp_tw_pcbs = pcb->next ;
//    pcb2 = pcb ;
//    pcb = pcb->next ;
//    memp_free( MEMP_TCP_PCB, pcb2 ) ;
//  }
//}
//// tcp_accept�Ļص�����
//err_t tcp_server_accept( void *arg, struct tcp_pcb *newpcb, err_t err )
//{
//  err_t ret_err ;
//  struct tcp_server_struct *es ;
//   LWIP_UNUSED_ARG( arg ) ;
//  LWIP_UNUSED_ARG( err ) ;
//  tcp_setprio( newpcb, TCP_PRIO_MIN ) ;                                //�����´�����pcb���ȼ�
//  es = ( struct tcp_server_struct* )mem_malloc( sizeof( struct tcp_server_struct ) ) ;  //�����ڴ�
//  //�ڴ����ɹ�
//   if( es!=NULL )
//  {
//    es->state = ES_TCPSERVER_ACCEPTED ;                //��������
//    es->pcb = newpcb ;
//    es->p = NULL ;
//    tcp_arg( newpcb, es ) ;
//    tcp_recv( newpcb, tcp_server_recv ) ;                  //��ʼ��tcp_recv�Ļص�����
//    tcp_err( newpcb, tcp_server_error ) ;                  //��ʼ��tcp_err�ص�����
//    tcp_poll( newpcb, tcp_server_poll, 1 ) ;                //��ʼ��tcp_poll�ص�����
//    tcp_sent( newpcb, tcp_server_sent ) ;                  //��ʼ�����ͻص�����
//    tcp_server_flag |= 1<<5 ;                      //����пͻ���������
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
//// TCP Server ����
//void tcp_server_test()
//{
//  err_t err ;  
//  struct tcp_pcb *tcppcbnew ;                        //����һ��TCP���������ƿ�
//  struct tcp_pcb *tcppcbconn ;                      //����һ��TCP���������ƿ�
//  u8 *tbuf ;
//  u8 res=0 ;
//  u8 connflag=0 ;                            //���ӱ��
//  tbuf = mymalloc( SRAMIN, 200 ) ;                    //�����ڴ�
//  //�ڴ�����ʧ����,ֱ���˳�
//  if( tbuf==NULL )
//    return ;
//  sprintf( ( char* )tbuf, "Server IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 130, tbuf ) ;                    //������IP
//  sprintf( ( char* )tbuf, "Server Port:%d", TCP_SERVER_PORT ) ;
//  LCD_ShowString( 30, 150, tbuf ) ;                    //�������˿ں�
//  tcppcbnew = tcp_new() ;                        //����һ���µ�pcb
//  //�����ɹ�
//  if( tcppcbnew )
//  {
//    err = tcp_bind( tcppcbnew, IP_ADDR_ANY, TCP_SERVER_PORT ) ;      //������IP��ָ���˿ںŰ�
//    //�����
//    if( err==ERR_OK )
//    {
//      tcppcbconn = tcp_listen( tcppcbnew ) ;              //����tcppcb�������״̬
//      tcp_accept( tcppcbconn, tcp_server_accept ) ;            //��ʼ��tcp_accept�Ļص�����
//    }
//    else
//      res = 1 ;
//  }
//  else
//    res = 1 ;
//  while( res==0 )
//  {
//    //�յ�����
//    if( tcp_server_flag&1<<6 )
//    {
//      tcp_server_flag |= 1<<7 ;                    //���Ҫ��������
//      LCD_ShowString( 30, 210, tcp_server_recvbuf ) ;          //��ʾ���յ�������
//      tcp_server_flag &= ~( 1<<6 ) ;                  //��������Ѿ���������
//    }
//    //�Ƿ�������
//    if( tcp_server_flag&1<<5 )
//    {
//      if( connflag==0 )
//      { 
//      sprintf( ( char* )tbuf, "Client IP:%d.%d.%d.%d", lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2], lwipdev.remoteip[3] ) ;
//         LCD_ShowString( 30, 170, tbuf ) ;              //�ͻ���IP
//        LCD_ShowString( 30, 190, "Receive Data:" ) ;          //��ʾ��Ϣ
//        connflag = 1 ;                        //���������
//      }
//    }
//    else if( connflag )
//      connflag = 0 ;                          //������ӶϿ���
//    lwip_periodic_handle() ;
//    lwip_pkt_handle() ;
//    delay_ms( 2 ) ;
//  }   
//  tcp_server_connection_close( tcppcbnew, 0 ) ;                //�ر�TCP Server����
//  tcp_server_connection_close( tcppcbconn, 0 ) ;                //�ر�TCP Server����
//  tcp_server_remove_timewait() ; 
//  memset( tcppcbnew, 0, sizeof( struct tcp_pcb ) ) ;
//  memset( tcppcbconn, 0, sizeof( struct tcp_pcb ) ) ;
//  myfree( SRAMIN, tbuf ) ;
//}

//27.1.2 tcp_server.h�����д
//#ifndef _TCP_SERVER_DEMO_H_
//#define _TCP_SERVER_DEMO_H_
//#include "sys.h"
//#include "lwip/tcp.h"
//#include "lwip/pbuf.h"
//#define TCP_SERVER_RX_BUFSIZE  2000          //����tcp server���������ݳ���
//#define TCP_SERVER_PORT      8088          //����tcp server�Ķ˿�
////tcp����������״̬
//enum tcp_server_states
//{
//  ES_TCPSERVER_NONE = 0,      //û������
//  ES_TCPSERVER_ACCEPTED,      //�пͻ�����������
//  ES_TCPSERVER_CLOSING,      //�����ر�����
//};
////LWIP�ص�����ʹ�õĽṹ��
//struct tcp_server_struct
//{
//  u8 state;              //��ǰ����״
//  struct tcp_pcb *pcb;        //ָ��ǰ��pcb
//  struct pbuf *p;          //ָ�����/�����pbuf
//}; 
//void tcp_server_test( void ) ;                  //TCP Server���Ժ���
//#endif

//27.1.3 �����������д
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
//   STM32_Clock_Init( 9 ) ;                                        //ϵͳʱ������
//  SysTick_Init( 72 ) ;                          //��ʱ��ʼ��
//  USART1_Init( 72, 115200 ) ;                      //���ڳ�ʼ��Ϊ115200
//  LCD_Init() ;                            //��ʼ��LCD
//  TIM3_Init( 1000, 719 ) ;                        //��ʱ��3Ƶ��Ϊ100hz
//  my_mem_init( SRAMIN ) ;                      //��ʼ���ڲ��ڴ��
//  while( lwip_comm_init() ) ;                      //lwip��ʼ��
//  //�ȴ�DHCP��ȡ�ɹ�/��ʱ���
//  while( ( lwipdev.dhcpstatus!=2 )&&( lwipdev.dhcpstatus!=0xFF ) )
//  {
//    lwip_periodic_handle() ;                    //LWIP�ں���Ҫ��ʱ����ĺ���
//    lwip_pkt_handle() ;
//  }
//  POINT_COLOR=RED;
//  LCD_ShowString( 30, 110, "LWIP Init Successed" ) ;
//  //��ӡ��̬IP��ַ
//  if( lwipdev.dhcpstatus==2 )
//    sprintf( ( char* )buf, "DHCP IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  //��ӡ��̬IP��ַ
//  else
//    sprintf( ( char* )buf, "Static IP:%d.%d.%d.%d", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3] ) ;
//  LCD_ShowString( 30, 130, buf ) ; 
//  //�õ�����
//  if( ( DM9000_Get_SpeedAndDuplex()&0x02 )==0x02 )
//    LCD_ShowString( 30, 150, "Ethernet Speed:10M" ) ;
//  else
//    LCD_ShowString( 30, 150, "Ethernet Speed:100M" ) ;
//   while( 1 )
//  {
//    tcp_server_test() ;                        //TCP����������
//    lwip_periodic_handle() ;
//    lwip_pkt_handle() ;
//    delay_ms( 2 ) ;
//  }
//}

