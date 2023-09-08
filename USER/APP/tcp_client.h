/*
tcp_client.h�����д

*/

#ifndef _TCP_CLIENT_H_
#define _TCP_CLIENT_H_


#include "sys.h"
#include "lwip/tcp.h"
#include "lwip/pbuf.h"
#define TCP_CLIENT_RX_BUFSIZE  1500            //���������ݳ���
#define TCP_CLIENT_TX_BUFSIZE  200              //��������ݳ���
#define LWIP_SEND_DATA      0x80            //�����ݷ���
#define  TCP_CLIENT_PORT    8087            //Զ�˶˿�
//tcp����������״̬
enum tcp_client_states
{
  ES_TCPCLIENT_NONE = 0,    //û������
  ES_TCPCLIENT_CONNECTED,  //���ӵ��������� 
  ES_TCPCLIENT_CLOSING,    //�ر�����
};
//LWIP�ص�����ʹ�õĽṹ��
struct tcp_client_struct
{
  uint8_t state;            //��ǰ����״
  struct tcp_pcb *pcb;      //ָ��ǰ��pcb
  struct pbuf *p;        //ָ�����/�����pbuf
};  
void tcp_client_test( void ) ;                    //TCP Client���Ժ���
#endif
