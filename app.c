

static  void  AppTask1 (void *p_arg)
{
    OS_ERR      err;
    while (DEF_TRUE) {                                                    /*  Sense the temperature every 100 ms     */
      count1 += 1;
      if(count1 >= threshold){
        BSP_LED_Toggle(1);
        count1 = 0;
      }
//        OSTimeDlyHMSM(0, 0, 0, 100,   
//                      OS_OPT_TIME_HMSM_STRICT, 
//                      &err);
    }
}
        

static  void  AppTask2 (void *p_arg)
{
    OS_ERR      err;
    while (DEF_TRUE) {                                                    /*  Sense the temperature every 100 ms     */
      count2 += 1;
      if(count2 >= threshold){
        BSP_LED_Toggle(2);
        count2 = 0;
      }
//        OSTimeDlyHMSM(0, 0, 0, 100,
//                      OS_OPT_TIME_HMSM_STRICT, 
//                      &err);
    }
}

static  void  AppTask3 (void *p_arg)
{
    OS_ERR      err;
    while (DEF_TRUE) {                                                    /*  Sense the temperature every 100 ms     */
      count3 += 1;
      if(count3 >= threshold){
        BSP_LED_Toggle(3);
        count3 = 0;
      }
//        OSTimeDlyHMSM(0, 0, 0, 100,
//                      OS_OPT_TIME_HMSM_STRICT, 
//                      &err);
    }
}
