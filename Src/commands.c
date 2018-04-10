
#include "stm32f4xx_hal.h"
#include "state_handler.h"
#include "commands.h"

/*Define memory areas that are readable*/


enum {
	idle = 0,
	reqReceived,
	receiveAllBytes,
	executeCommand
};

unsigned char writeMemoryByAddress()
{
	static unsigned char addressSize, lengthSize;
	static unsigned short bytesToReceive;
	unsigned char stayInLoop = 0, retVal = 0, indeX;
	do
	{
		if(commandSubState == idle)
		{
			addressSize = RxFrameBuffer[buffIndex].Data[1] & 0xF;
			lengthSize = (RxFrameBuffer[buffIndex].Data[1] & 0xF0) >> 4;
			for(indeX = 0; indeX < lengthSize; indeX++)
			{
				bytesToReceive += (unsigned short)RxFrameBuffer[buffIndex].Data[2+addressSize+indeX] <<((lengthSize-1)-indeX);
			}
			if ((6-(addressSize+lengthSize)) >= bytesToReceive)
			{
				/*Whole command fits in one CAN frame*/
				commandSubState = executeCommand;
				stayInLoop = 1;
			}
			else
			{
				commandSubState = receiveAllBytes;
				bytesToReceive -= (6-(addressSize+lengthSize)) ;
			}
			/*Store now the positive response*/
			TxMsg.StdId = TxMsg.ExtId = RxFrameBuffer[buffIndex].ID+1;
			TxMsg.IDE = CAN_ID_EXT;
			TxMsg.RTR = CAN_RTR_DATA;
			TxMsg.DLC = RxFrameBuffer[buffIndex].DLC;
			TxMsg.Data[0] = RxFrameBuffer[buffIndex].Data[0] | 0x40;
			for (indeX = 1; indeX < TxMsg.DLC; indeX++)
			{
				TxMsg.Data[indeX] = RxFrameBuffer[buffIndex].Data[indeX];
			}
		}
		else if(commandSubState == receiveAllBytes)
		{
			if(bytesToReceive > 0)
			{
				if(RxFrameBuffer[buffIndex].DLC < bytesToReceive)
				{
					bytesToReceive -= RxFrameBuffer[buffIndex].DLC;
				}
				else
				{
					bytesToReceive = 0;
					commandSubState = executeCommand;
					stayInLoop = 1;
				}
			}
		}
		else if (commandSubState == executeCommand)
		{
			/* add the response (TxMsg) to the array of frames*/
			buffIndex++;
			RxFrameBuffer[buffIndex].ID = TxMsg.StdId;
			RxFrameBuffer[buffIndex].DLC = 1;
			RxFrameBuffer[buffIndex].Data[0] = TxMsg.Data[0];
			RxFrameBuffer[buffIndex].Dir = 1;
			/*Send positive response*/
			(void)HAL_CAN_Transmit(&hcan2, 1);
			stayInLoop = 0;
			retVal = 1; /*Command is complete, exit from state*/
			commandSubState = 0;
		}
	}while (stayInLoop == 1);
	return retVal;
}

unsigned char securityAccess()
{
	unsigned char retVal = 0, indeX;
	if((RxFrameBuffer[buffIndex].Data[1] % 2) == 1)
	{
		/*Build a test frame*/
		TxMsg.StdId = TxMsg.ExtId = RxFrameBuffer[buffIndex].ID+1;
		TxMsg.IDE = CAN_ID_EXT;
		TxMsg.RTR = CAN_RTR_DATA;
		TxMsg.DLC = 4;
		TxMsg.Data[0] = RxFrameBuffer[buffIndex].Data[0] | 0x40;
		TxMsg.Data[1] = RxFrameBuffer[buffIndex].Data[1];
		for(indeX = 2; indeX < 4; indeX++)
		{
			TxMsg.Data[indeX] = 0xAA;
		}
		(void)HAL_CAN_Transmit(&hcan2, 1);
		commandSubState = reqReceived;
		/* add the response (TxMsg) to the array of frames*/
		buffIndex++;
		RxFrameBuffer[buffIndex].ID = TxMsg.StdId;
		RxFrameBuffer[buffIndex].DLC = TxMsg.DLC;
		RxFrameBuffer[buffIndex].Data[0] = TxMsg.Data[0];
		RxFrameBuffer[buffIndex].Data[1] = TxMsg.Data[1];
		RxFrameBuffer[buffIndex].Data[2] = TxMsg.Data[2];
		RxFrameBuffer[buffIndex].Data[3] = TxMsg.Data[3];
		RxFrameBuffer[buffIndex].Dir = 1;
	}
	else
	{
		if(commandSubState == reqReceived)
		{
			/*Security access seed was requested before, now send OK to the key (THE KEY IS NOT CHECKED)*/
			TxMsg.StdId = TxMsg.ExtId = RxFrameBuffer[buffIndex].ID+1;
			TxMsg.IDE = CAN_ID_EXT;
			TxMsg.RTR = CAN_RTR_DATA;
			TxMsg.DLC = 2;
			TxMsg.Data[0] = RxFrameBuffer[buffIndex].Data[0] | 0x40;
			TxMsg.Data[1] = RxFrameBuffer[buffIndex].Data[1];
			/* add the response (TxMsg) to the array of frames*/
			buffIndex++;
			RxFrameBuffer[buffIndex].ID = TxMsg.StdId;
			RxFrameBuffer[buffIndex].DLC = TxMsg.DLC;
			RxFrameBuffer[buffIndex].Data[0] = TxMsg.Data[0];
			RxFrameBuffer[buffIndex].Data[1] = TxMsg.Data[1];
			RxFrameBuffer[buffIndex].Dir = 1;
		}
		else
		{
			/*No Security access seed was requested, so return NOK frame*/
			TxMsg.StdId = TxMsg.ExtId = RxFrameBuffer[buffIndex].ID+1;
			TxMsg.IDE = CAN_ID_EXT;
			TxMsg.RTR = CAN_RTR_DATA;
			TxMsg.DLC = 3;
			TxMsg.Data[0] = RxFrameBuffer[buffIndex].Data[0] | 0x20;
			TxMsg.Data[1] = RxFrameBuffer[buffIndex].Data[1];
			TxMsg.Data[2] = 0x22; /*Return error code CNC - ConditionsNotCorrect*/
			/* add the response (TxMsg) to the array of frames*/
			buffIndex++;
			RxFrameBuffer[buffIndex].ID = TxMsg.StdId;
			RxFrameBuffer[buffIndex].DLC = TxMsg.DLC;
			RxFrameBuffer[buffIndex].Data[0] = TxMsg.Data[0];
			RxFrameBuffer[buffIndex].Data[1] = TxMsg.Data[1];
			RxFrameBuffer[buffIndex].Data[2] = TxMsg.Data[2];
			RxFrameBuffer[buffIndex].Dir = 1;
		}
		(void)HAL_CAN_Transmit(&hcan2, 1);
		commandSubState = idle;
		retVal = 1;
	}
	return retVal;
}
