#include "main.h"
#include "watchdog.h"
#ifdef ADAFRUIT_FEATHER_M0
#include <wdt_samd21.h>
#endif
#ifdef PRO_MINI_RFM69
#include "avr_watchdog.h"
#endif


#ifdef PRO_MINI_RFM69
// AVR_Watchdog watchdog(4);
#endif


void watchdog_initialize(void)
{
    #ifdef PRO_MINI_RFM69
    //watchdog.set_timeout(30);
    #endif

    #ifdef  ADA_M0_RFM69
    
    #endif
      
}


void watchdog_clear_local(void)
{
    #ifdef PRO_MINI_RFM69
    //watchdog.clear();
    #endif
    #ifdef  ADA_M0_RFM69
      // Initialze WDT with a 2 sec. timeout
      wdt_init ( WDT_CONFIG_PER_16K );
    #endif
      
}
