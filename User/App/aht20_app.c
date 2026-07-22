#include "aht20_app.h"

AHT20_StructureTypedef  Humiture;

void Aht20_Init(void)
{
  Uart_Printf(DEBUG_UART, "Aht_Init ......\r\n");
  
  if(AHT20_Init() != 0)
  {
    Humiture.alive = 0;
  }
  else
  {
    Humiture.alive = 1;
  }  
}

void Aht20_Task(void)
{
  if(Humiture.alive == 0) return;
  
  AHT20_Get_Value(&Humiture);
}

