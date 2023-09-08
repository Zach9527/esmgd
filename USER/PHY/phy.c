#include "phy.h"

uint8_t sntData[MBTCP_SENTMAX];

extern osThreadId PCSPHYTASKHandle;
extern osThreadId BSMPHYTASKHandle;

int8_t MBTCPSend(server_info *info,uint8_t *pucFrame,uint16_t usLength)
{
    err_t err;
    //taskENTER_CRITICAL();
    uint16_t usTCPLength = usLength + MB_TCP_FUNC;
    sntData[0] = (info->tID >> 8) & 0xff;
    sntData[1] = info->tID & 0xff;
    sntData[MB_TCP_LEN] = (usLength >> 8) &0xff;
    sntData[MB_TCP_LEN + 1] = ( usLength ) & 0xFF;
    memcpy(sntData + MB_TCP_FUNC, pucFrame, usLength);
    if(info->g_lwip_connect_state == 1)
    {
        err = write(info->sock,sntData,usTCPLength);
        if(err < 0)
        {
            closesocket(info->sock);
            //taskEXIT_CRITICAL();
            return err;
        }
        info->tID++;
    }
    //taskEXIT_CRITICAL();
		return usTCPLength;
}

void MBTCPRecv(server_info *info,uint8_t *pucFrame,uint16_t usLength)
{

    uint16_t usPID,revLength;
    uint8_t* mbrev;
    usPID = pucFrame[MB_TCP_PID] << 8;
    usPID |= pucFrame[MB_TCP_PID + 1];

    if( usPID == MB_TCP_PROTOCOL_ID )
    {
        mbrev = pucFrame + MB_DEV_ADDR;
        revLength = usLength - MB_TCP_FUNC;
        switch(pucFrame[MB_DEV_ADDR])
        {
            case TCP_PCS_ID:
                PcsDatGet(mbrev,revLength);
                info->Reissued = 1;
                //osSignalSet(PCSPHYTASKHandle,0x01);
                break;
            case TCP_BSM_ID:
                BsmDatGet(mbrev,revLength);
                info->Reissued = 1;
                //osSignalSet(BSMPHYTASKHandle,0x01);
                break;
            default:
                info->Reissued = 0;
            break;
        }
    }
}

void NetEndianConver(uint8_t *dat,uint16_t length)
{
    int tmp,i;
    uint16_t len = length/4;
    int *converdat = (int*)dat;
    if((length % 4) != 0)
    {
        len = len+1;
    }
    for(i = 0;i < len; i++)
    {
        tmp = L2B32(converdat[i]);
        converdat[i] = tmp;
    }
}
