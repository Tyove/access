#include "cnclude.h"
extern void Hardware_Init(void);

int main()
{
  //MUC initialize
  MCU_ Init();
  
  //else initialize
  Hardware_Init();
  
  while(ture)
  {
    //KernelPolling
    KernelPolling();
  }
}
