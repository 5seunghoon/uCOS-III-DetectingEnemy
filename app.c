/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                             (c) Copyright 2009; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                            EXAMPLE CODE
*
*                                     ST Microelectronics STM32
*                                              on the
*
*                                     Micrium uC-Eval-STM32F107
*                                        Evaluation Board
*
* Filename      : app.c
* Version       : V1.00
* Programmer(s) : JJL
                  EHS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include <includes.h>


/*
*********************************************************************************************************
*                                             LOCAL DEFINES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                            LOCAL VARIABLES
*********************************************************************************************************
*/

static OS_SEM AppSem;

static OS_TCB AppTaskStartTCB;
static OS_TCB AppTask1_TCB;
static OS_TCB AppTask2_TCB;
static CPU_STK AppTaskStartStk[APP_TASK_START_STK_SIZE];
static CPU_STK AppTask1_Stk[APP_TASK_START_STK_SIZE];
static CPU_STK AppTask2_Stk[APP_TASK_START_STK_SIZE];

/////////////////////


static OS_SEM infraredIncomingSem1;
static OS_SEM infraredIncomingSem2;
static OS_SEM infraredIncomingSem3;
static OS_SEM infraredIncomingSem4;

static OS_SEM infraredExitSem1;
static OS_SEM infraredExitSem2;
static OS_SEM infraredExitSem3;
static OS_SEM infraredExitSem4;

static OS_SEM buttonSem;

static OS_SEM fireDetectionSem;

static OS_SEM timerEndSem;


//static OS_MUTEX infraredIncomingSem1;
//static OS_MUTEX infraredIncomingSem2;
//static OS_MUTEX infraredIncomingSem3;
//static OS_MUTEX infraredIncomingSem4;
//
//static OS_MUTEX infraredExitSem1;
//static OS_MUTEX infraredExitSem2;
//static OS_MUTEX infraredExitSem3;
//static OS_MUTEX infraredExitSem4;
//
//static OS_MUTEX buttonSem;
//static OS_MUTEX fireDetectionSem;
//static OS_MUTEX timerEndSem;



static OS_TCB DectionPatrolTask_TCB;
static OS_TCB DectionMoveTask_TCB;
static OS_TCB WindowCloseTask_TCB;
static OS_TCB WindowOpenTask_TCB;
static OS_TCB AlertOnTask_TCB;
static OS_TCB AlertOffTask_TCB;
static OS_TCB TimerStartTask_TCB;
static OS_TCB TimerForceQuitTask_TCB;

static CPU_STK DectionPatrolTask_Stk[APP_TASK_START_STK_SIZE];
static CPU_STK DectionMoveTask_Stk[APP_TASK_START_STK_SIZE];
static CPU_STK WindowCloseTask_Stk[APP_TASK_START_STK_SIZE];
static CPU_STK WindowOpenTask_Stk[APP_TASK_START_STK_SIZE];
static CPU_STK AlertOnTask_Stk[APP_TASK_START_STK_SIZE];
static CPU_STK AlertOffTask_Stk[APP_TASK_START_STK_SIZE];
static CPU_STK TimerStartTask_Stk[APP_TASK_START_STK_SIZE];
static CPU_STK TimerForceQuitTask_Stk[APP_TASK_START_STK_SIZE];

//////////////////////////////////////////////

static int DectionMoveTask_PRIO = 4;
static int WindowCloseTask_PRIO = 6;
static int WindowOpenTask_PRIO = 5;
static int AlertOnTask_PRIO = 9;
static int AlertOffTask_PRIO = 3;
static int TimerStartTask_PRIO = 6;
static int TimerForceQuitTask_PRIO = 6;


////////////////////    


static vu32 ADCvalue[6];
static int record[6];

static int RxData=0;
static int initDegree = 0;
static int sencingDegree = 90;
static int noSencingDergreee = 0;

static TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

static u32 SystemCoreClock = 168000000;

static int posX = 0;
static int posY = 0;

static int degreeX = 0;
static int degreeY = 0;

OS_MEM UART_MemPool;
char UART_MemStorage[200][20];

char *RxDataPtr;
int RxDataCtr = 0;

OS_Q UART_Q;
OS_SEM MySem;

OS_TMR MyTmr;

static int infraredNum = 6;
static int isDetect = 0;

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static void AppTaskStart(void *p_arg);

static void AppTask1(void *p_arg);

static void AppTask2(void *p_arg);

static void RCC_Config(void);

static void GPIO_Config(void);

static void ADC_Config(void);

static void NVIC_Config(void);

static void DMA_Channel_Config(void);

static void PWM_Timer_Config(void);

static void USART_Config(void);

static void Data_Init(void);
    
static void suboDegreeSet(int motornum, int degree);
static int degreeToPWM(int degree);

static void subomotorInitDegree(void);

static void subomotorSencingDegree(void);

static void subomotorNoSencingDegree(void);

static void suboLightXY(int x, int y);          

static int coorToDegreeX(int x);

static int coorToDegreeY(int y);

//static void InterruptConfigure(void);

void USART_InterruptConfigure(void);
void USART1_IRQHandler(void);
void TmrCallback(void);
void TmrStopCallback(void);
/////////////////////////

static void DectionMoveTask(void *p_arg);
static void WindowCloseTask(void *p_arg);
static void WindowOpenTask(void *p_arg);
static void AlertOnTask(void *p_arg);
static void AlertOffTask(void *p_arg);
static void TimerStartTask(void *p_arg);
static void TimerForceQuitTask(void *p_arg);

////////////////////////
/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Arguments   : none
*
* Returns     : none
*********************************************************************************************************
*/


int main(void) {
    OS_ERR err;

    BSP_IntDisAll();                                            /* Disable all interrupts.                              */

    OSInit(&err);                                               /* Init uC/OS-III.                                      */

    RxData = 0;
    
    OSSemCreate(&infraredIncomingSem1,
                "infraredIncomingSem1",
                (OS_SEM_CTR) 0,
                &err);
    OSSemCreate(&infraredIncomingSem2,
                "infraredIncomingSem2",
                (OS_SEM_CTR) 0,
                &err);
    OSSemCreate(&infraredIncomingSem3,
                "infraredIncomingSem3",
                (OS_SEM_CTR) 0,
                &err);
    OSSemCreate(&infraredIncomingSem4,
                "infraredIncomingSem4",
                (OS_SEM_CTR) 0,
                &err);
    
    OSSemCreate(&infraredExitSem1,
                "infraredExitSem1",
                (OS_SEM_CTR) 0,
                &err);
    OSSemCreate(&infraredExitSem2,
                "infraredExitSem2",
                (OS_SEM_CTR) 0,
                &err);
    OSSemCreate(&infraredExitSem3,
                "infraredExitSem3",
                (OS_SEM_CTR) 0,
                &err);
    OSSemCreate(&infraredExitSem4,
                "infraredExitSem4",
                (OS_SEM_CTR) 0,
                &err);
    
    OSSemCreate(&buttonSem,
                "buttonSem",
                (OS_SEM_CTR) 0,
                &err);
    
    OSSemCreate(&fireDetectionSem,
                "fireDetectionSem",
                (OS_SEM_CTR) 0,
                &err);
        
    OSSemCreate(&timerEndSem,
                "timerEndSem",
                (OS_SEM_CTR) 0,
                &err);
    
 /*   
    OSMutexCreate(&infraredIncomingSem1,
                "infraredIncomingSem1",
                &err);
    OSMutexCreate(&infraredIncomingSem2,
                "infraredIncomingSem2",
                &err);
    OSMutexCreate(&infraredIncomingSem3,
                "infraredIncomingSem3",
                &err);
    OSMutexCreate(&infraredIncomingSem4,
                "infraredIncomingSem4",
                &err);
    
    OSMutexCreate(&infraredExitSem1,
                "infraredExitSem1",
                &err);
    OSMutexCreate(&infraredExitSem2,
                "infraredExitSem2",
                &err);
    OSMutexCreate(&infraredExitSem3,
                "infraredExitSem3",

                &err);
    OSMutexCreate(&infraredExitSem4,
                "infraredExitSem4",

                &err);
    
    OSMutexCreate(&buttonSem,
                "buttonSem",
                &err);
    
    OSMutexCreate(&fireDetectionSem,
                "fireDetectionSem",
                &err);
        
    OSMutexCreate(&timerEndSem,
                "timerEndSem",
                &err);
*/
    
    OSTaskCreate( 
            (OS_TCB * ) & AppTaskStartTCB,                /* Create the start task                                */
            (CPU_CHAR *) "App Task Start",
            (OS_TASK_PTR) AppTaskStart,
            (void *) 0,
            (OS_PRIO) APP_TASK_START_PRIO,
            (CPU_STK * ) & AppTaskStartStk[0],
            (CPU_STK_SIZE) APP_TASK_START_STK_SIZE / 10,
            (CPU_STK_SIZE) APP_TASK_START_STK_SIZE,
            (OS_MSG_QTY) 0,
            (OS_TICK) 0,
            (void *) 0,
            (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
            (OS_ERR * ) & err);
    
    OSMemCreate((OS_MEM        *)&UART_MemPool,
                (CPU_CHAR      *)"UART_MEM",
                (void          *)&UART_MemStorage[0][0],
                (OS_MEM_QTY     ) 200,
                (OS_MEM_SIZE    ) 20,
                (OS_ERR        *) &err);
    
    OSQCreate((OS_Q *)&UART_Q,
              (CPU_CHAR *)"UART_Q",
              (OS_MSG_QTY)20,
              (OS_ERR *)&err);
    
    OSSemCreate(&MySem,
                "My Sem",
                (OS_SEM_CTR)0,
                &err);

    OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */
}


/*
*********************************************************************************************************
*                                          STARTUP TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static void AppTaskStart(void *p_arg) {
    CPU_INT32U cpu_clk_freq;
    CPU_INT32U cnts;
    OS_ERR err;
    CPU_TS ts;

    (void) p_arg;

    OSSemCreate(&AppSem, "Test Sem", 0, &err);

    BSP_Init();                                                   /* Initialize BSP functions                         */
    CPU_Init();                                                   /* Initialize the uC/CPU services                   */

    RCC_Config();
    GPIO_Config();
    ADC_Config();
    NVIC_Config();
    DMA_Channel_Config();
    PWM_Timer_Config();
    USART_Config();
    //USART_InterruptConfigure();

    Data_Init();
    subomotorInitDegree();


    cpu_clk_freq = BSP_CPU_ClkFreq();                             /* Determine SysTick reference freq.                */
    cnts = cpu_clk_freq / (CPU_INT32U) OSCfg_TickRate_Hz;  /* Determine nbr SysTick increments                 */
    OS_CPU_SysTickInit(cnts);                                     /* Init uC/OS periodic time src (SysTick).          */

#if OS_CFG_STAT_TASK_EN > 0u
    OSStatTaskCPUUsageInit(&err);                                 /* Compute CPU capacity with no task running        */
#endif

    CPU_IntDisMeasMaxCurReset();

    
    OSTaskCreate((OS_TCB * ) & DectionMoveTask_TCB,
                 (CPU_CHAR *) "DectionMoveTask",
                 (OS_TASK_PTR) DectionMoveTask,
                 (void *) 0,
                 (OS_PRIO) DectionMoveTask_PRIO,
                 (CPU_STK * ) & DectionMoveTask_Stk[0],
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE / 10,
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE,
                 (OS_MSG_QTY) 0,
                 (OS_TICK) 0,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR * ) & err);        
    
    
    OSTaskCreate((OS_TCB * ) & WindowCloseTask_TCB,
                 (CPU_CHAR *) "WindowCloseTask",
                 (OS_TASK_PTR) WindowCloseTask,
                 (void *) 0,
                 (OS_PRIO) WindowCloseTask_PRIO,
                 (CPU_STK * ) & WindowCloseTask_Stk[0],
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE / 10,
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE,
                 (OS_MSG_QTY) 0,
                 (OS_TICK) 0,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR * ) & err);
    
    
    
    OSTaskCreate((OS_TCB * ) & WindowOpenTask_TCB,
                 (CPU_CHAR *) "WindowOpenTask",
                 (OS_TASK_PTR) WindowOpenTask,
                 (void *) 0,
                 (OS_PRIO) WindowOpenTask_PRIO,
                 (CPU_STK * ) & WindowOpenTask_Stk[0],
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE / 10,
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE,
                 (OS_MSG_QTY) 0,
                 (OS_TICK) 0,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR * ) & err);        
    
  
    OSTaskCreate((OS_TCB * ) & AlertOnTask_TCB,
                 (CPU_CHAR *) "AlertOnTask",
                 (OS_TASK_PTR) AlertOnTask,
                 (void *) 0,
                 (OS_PRIO) AlertOnTask_PRIO,
                 (CPU_STK * ) & AlertOnTask_Stk[0],
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE / 10,
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE,
                 (OS_MSG_QTY) 0,
                 (OS_TICK) 0,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR * ) & err);     
   

    OSTaskCreate((OS_TCB * ) & AlertOffTask_TCB,
                 (CPU_CHAR *) "AlertOffTask",
                 (OS_TASK_PTR) AlertOffTask,
                 (void *) 0,
                 (OS_PRIO) AlertOffTask_PRIO,
                 (CPU_STK * ) & AlertOffTask_Stk[0],
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE / 10,
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE,
                 (OS_MSG_QTY) 0,
                 (OS_TICK) 0,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR * ) & err);     
       
    
    
    OSTaskCreate((OS_TCB * ) & TimerStartTask_TCB,
                 (CPU_CHAR *) "TimerStartTask",
                 (OS_TASK_PTR) TimerStartTask,
                 (void *) 0,
                 (OS_PRIO) TimerStartTask_PRIO,
                 (CPU_STK * ) & TimerStartTask_Stk[0],
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE / 10,
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE,
                 (OS_MSG_QTY) 0,
                 (OS_TICK) 0,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR * ) & err);        
    
    
   OSTaskCreate((OS_TCB * ) & TimerForceQuitTask_TCB,
                 (CPU_CHAR *) "TimerForceQuitTask",
                 (OS_TASK_PTR) TimerForceQuitTask,
                 (void *) 0,
                 (OS_PRIO) TimerForceQuitTask_PRIO,
                 (CPU_STK * ) & TimerForceQuitTask_Stk[0],
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE / 10,
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE,
                 (OS_MSG_QTY) 0,
                 (OS_TICK) 0,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR * ) & err);          
    
    
    
    while (DEF_TRUE) {                                            /* Task body, always written as an infinite loop.   */
      if(ADCvalue[3] > 500) {
          OSSemPost(&fireDetectionSem,
          OS_OPT_POST_ALL,
          &err);
          
//          OSMutexPost(&fireDetectionSem,
//          OS_OPT_POST_ALL,
//          &err);
//          
      }
      
      if(ADCvalue[1] > 1200) {
          OSSemPost(&buttonSem,
          OS_OPT_POST_ALL,
          &err);
          
//          OSMutexPost(&buttonSem,
//          OS_OPT_POST_ALL,
//          &err);
      }
      
      if(isDetect == 0) {
          if(ADCvalue[0] > 2500 && ADCvalue[0] < 3400 ){//1번센서 감지
            OSSemPost(&infraredIncomingSem1,
            OS_OPT_POST_ALL,
            &err);
            
//            OSMutexPost(&infraredIncomingSem1,
//            OS_OPT_POST_ALL,
//            &err);

            isDetect = 1;
            OSTimeDlyHMSM(0, 0, 0, 500,
                       OS_OPT_TIME_HMSM_STRICT,
                       &err);
          } else if(ADCvalue[2] > 2500 && ADCvalue[2] < 3400){//2번센서 감지
            OSSemPost(&infraredIncomingSem2,
            OS_OPT_POST_ALL,
            &err);
            
//            OSMutexPost(&infraredIncomingSem2,
//            OS_OPT_POST_ALL,
//            &err);
            isDetect = 1;
            OSTimeDlyHMSM(0, 0, 0, 500,
                       OS_OPT_TIME_HMSM_STRICT,
                       &err);
          } else if(ADCvalue[4] > 2500 && ADCvalue[4] < 3400){//3번센서 감지
            OSSemPost(&infraredIncomingSem3,
            OS_OPT_POST_ALL,
            &err);
            
//            OSMutexPost(&infraredIncomingSem3,
//            OS_OPT_POST_ALL,
//            &err);
            isDetect = 1;
            OSTimeDlyHMSM(0, 0, 0, 500,
                       OS_OPT_TIME_HMSM_STRICT,
                       &err);
          } else if(ADCvalue[5] > 2500 && ADCvalue[5] < 3400){//4번센서 감지
            OSSemPost(&infraredIncomingSem4,
            OS_OPT_POST_ALL,
            &err);

//            OSMutexPost(&infraredIncomingSem4,
//            OS_OPT_POST_ALL,
//            &err);
            
            isDetect = 1;
            OSTimeDlyHMSM(0, 0, 0, 500,
                       OS_OPT_TIME_HMSM_STRICT,
                       &err);
        }
      } else if(isDetect == 1) {
        if(ADCvalue[0] > 2500 && ADCvalue[0] < 3400 ){//1번센서 감지
            OSSemPost(&infraredExitSem1,
            OS_OPT_POST_ALL,
            &err);

//            OSMutexPost(&infraredExitSem1,
//            OS_OPT_POST_ALL,
//            &err);
            
            OSTimeDlyHMSM(0, 0, 0, 500,
                       OS_OPT_TIME_HMSM_STRICT,
                       &err);
          } else if(ADCvalue[2] > 2500 && ADCvalue[2] < 3400){//2번센서 감지
            OSSemPost(&infraredExitSem2,
            OS_OPT_POST_ALL,
            &err);

//            OSMutexPost(&infraredExitSem2,
//            OS_OPT_POST_ALL,
//            &err);
            
            OSTimeDlyHMSM(0, 0, 0, 500,
                       OS_OPT_TIME_HMSM_STRICT,
                       &err);
          } else if(ADCvalue[4] > 2500 && ADCvalue[4] < 3400){//3번센서 감지
            OSSemPost(&infraredExitSem3,
            OS_OPT_POST_ALL,
            &err);

//            OSMutexPost(&infraredExitSem3,
//            OS_OPT_POST_ALL,
//            &err);
            
            OSTimeDlyHMSM(0, 0, 0, 500,
                       OS_OPT_TIME_HMSM_STRICT,
                       &err);
          } else if(ADCvalue[5] > 2500 && ADCvalue[5] < 3400){//4번센서 감지
            OSSemPost(&infraredExitSem4,
            OS_OPT_POST_ALL,
            &err);

//            OSMutexPost(&infraredExitSem4,
//            OS_OPT_POST_ALL,
//            &err);
            
            OSTimeDlyHMSM(0, 0, 0, 500,
                       OS_OPT_TIME_HMSM_STRICT,
                       &err);
        }
      }
        
       OSTimeDlyHMSM(0, 0, 0, 200,
                     OS_OPT_TIME_HMSM_STRICT,
                     &err);
        
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////

static void DectionMoveTask(void *p_arg){
    OS_ERR err;
    CPU_TS ts;
    p_arg = p_arg;
    BSP_LED_Off(0);
    char *coor;
    OS_MSG_SIZE size;
    int i;

    while (1) {
        coor = (char*)OSQPend((OS_Q *)&UART_Q,
               (OS_TICK )0,
               (OS_OPT )OS_OPT_PEND_BLOCKING,
               (OS_MSG_SIZE *)&size,
               (CPU_TS *)&ts,
               (OS_ERR *)&err);
        
        //BSP_LED_Toggle(2);
        
        posX = 0;
        posY = 0;
        
        for(i=0; coor[i] != ','; i++) {
              posX = posX * 10 + coor[i] - '0';
        }
        
        i++;
        
        for(; coor[i] != '\n'; i++) {
              posY = posY * 10 + coor[i] - '0';
        }
        
        degreeX = coorToDegreeX(posX);
        degreeY = coorToDegreeY(posY);
        
        suboLightXY(coorToDegreeX(posX),coorToDegreeY(posY));
        
        OSMemPut((OS_MEM   *)&UART_MemPool,
                     (void     *)coor,
                     (OS_ERR   *)&err);
        
    }
}
static void WindowCloseTask(void *p_arg){
    OS_ERR err;
    CPU_TS ts;
    OS_PEND_DATA infraredMultiTbl[2];

    infraredMultiTbl[0].PendObjPtr = (OS_PEND_OBJ*) &fireDetectionSem;
    infraredMultiTbl[1].PendObjPtr = (OS_PEND_OBJ*) &timerEndSem;
    
    while(1){ 
      
      OSPendMulti((OS_PEND_DATA*)       &infraredMultiTbl[0],
                  (OS_OBJ_QTY)          2,
                  (OS_TICK)             0,
                  (OS_OPT)              OS_OPT_PEND_BLOCKING,
                  (OS_ERR*)             &err );
      
      subomotorSencingDegree();
    }      
}
static void WindowOpenTask(void *p_arg){
    OS_ERR err;
    CPU_TS ts;
    while(1){
            OSSemPend(&buttonSem,
                      0,
                      OS_OPT_PEND_BLOCKING,
                      &ts,
                      &err);

//            OSMutexPend(&buttonSem,
//                      0,
//                      OS_OPT_PEND_BLOCKING,
//                      &ts,
//                      &err);
            subomotorNoSencingDegree();
            isDetect = 0;
    }
}
static void AlertOnTask(void *p_arg){
    OS_ERR err;
    CPU_TS ts;
    OS_PEND_DATA infraredMultiTbl[4];
         
    infraredMultiTbl[0].PendObjPtr = (OS_PEND_OBJ*) &infraredIncomingSem1;
    infraredMultiTbl[1].PendObjPtr = (OS_PEND_OBJ*) &infraredIncomingSem2;
    infraredMultiTbl[2].PendObjPtr = (OS_PEND_OBJ*) &infraredIncomingSem3;
    infraredMultiTbl[3].PendObjPtr = (OS_PEND_OBJ*) &infraredIncomingSem4;
    
    while(1){     
      OSPendMulti((OS_PEND_DATA*)       &infraredMultiTbl[0],
                  (OS_OBJ_QTY)          4,
                  (OS_TICK)             0,
                  (OS_OPT)              OS_OPT_PEND_BLOCKING,
                  (OS_ERR*)             &err );
      BSP_LED_On(4);
      BSP_IntEn(BSP_INT_ID_USART1);
      
    }
}
static void AlertOffTask(void *p_arg){
    OS_ERR err;
    CPU_TS ts;
    OS_PEND_DATA infraredMultiTbl[4];
      
    infraredMultiTbl[0].PendObjPtr = (OS_PEND_OBJ*) &infraredExitSem1;
    infraredMultiTbl[1].PendObjPtr = (OS_PEND_OBJ*) &infraredExitSem2;
    infraredMultiTbl[2].PendObjPtr = (OS_PEND_OBJ*) &infraredExitSem3;
    infraredMultiTbl[3].PendObjPtr = (OS_PEND_OBJ*) &infraredExitSem4;
    infraredMultiTbl[4].PendObjPtr = (OS_PEND_OBJ*) &buttonSem;
    
    
    while(1){     
      OSPendMulti((OS_PEND_DATA*)       &infraredMultiTbl[0],
                  (OS_OBJ_QTY)          5,
                  (OS_TICK)             0,
                  (OS_OPT)              OS_OPT_PEND_BLOCKING,
                  (OS_ERR*)             &err );
      BSP_LED_Off(4);
      BSP_IntDis(BSP_INT_ID_USART1);
      
    }
}
static void TimerStartTask(void *p_arg){
    OS_ERR err;
    CPU_TS ts;
    OS_PEND_DATA infraredMultiTbl[4];
         
    infraredMultiTbl[0].PendObjPtr = (OS_PEND_OBJ*) &infraredIncomingSem1;
    infraredMultiTbl[1].PendObjPtr = (OS_PEND_OBJ*) &infraredIncomingSem2;
    infraredMultiTbl[2].PendObjPtr = (OS_PEND_OBJ*) &infraredIncomingSem3;
    infraredMultiTbl[3].PendObjPtr = (OS_PEND_OBJ*) &infraredIncomingSem4;
    
    while(1){     
      OSPendMulti((OS_PEND_DATA*)       &infraredMultiTbl[0],
                  (OS_OBJ_QTY)          4,
                  (OS_TICK)             0,
                  (OS_OPT)              OS_OPT_PEND_BLOCKING,
                  (OS_ERR*)             &err );
      BSP_LED_On(4);
      BSP_IntEn(BSP_INT_ID_USART1);
      OSTmrCreate((OS_TMR*)&MyTmr,
                  (CPU_CHAR*)"MY TIMER",
                  (OS_TICK)50,
                  (OS_TICK)0,
                  (OS_OPT)OS_OPT_TMR_ONE_SHOT,
                  (OS_TMR_CALLBACK_PTR)TmrCallback,
                  (void *)0,
                  (OS_ERR*)&err);
      
      OSTmrStart ((OS_TMR *)&MyTmr,
                  (OS_ERR *)&err);  
    }
}
static void TimerForceQuitTask(void *p_arg){
    OS_ERR err;
    CPU_TS ts;
    OS_PEND_DATA infraredMultiTbl[5];
      
    infraredMultiTbl[0].PendObjPtr = (OS_PEND_OBJ*) &infraredExitSem1;
    infraredMultiTbl[1].PendObjPtr = (OS_PEND_OBJ*) &infraredExitSem2;
    infraredMultiTbl[2].PendObjPtr = (OS_PEND_OBJ*) &infraredExitSem3;
    infraredMultiTbl[3].PendObjPtr = (OS_PEND_OBJ*) &infraredExitSem4;
    infraredMultiTbl[4].PendObjPtr = (OS_PEND_OBJ*) &buttonSem;
    
    
    while(1){
      BSP_LED_Toggle(3);
      OSPendMulti((OS_PEND_DATA*)       &infraredMultiTbl[0],
                  (OS_OBJ_QTY)          5,
                  (OS_TICK)             0,
                  (OS_OPT)              OS_OPT_PEND_BLOCKING,
                  (OS_ERR*)             &err );
      BSP_LED_Toggle(3);
      OSTmrStop((OS_TMR*)&MyTmr,
                (OS_OPT)OS_OPT_TMR_ONE_SHOT,
                (void*)TmrStopCallback,
                (OS_ERR*)&err);
      isDetect = 0;
    }
}



//////////////////////////////////////////////////////////////////////////////////////////////
/*
static void AppTask2(void *p_arg) {
    OS_ERR err;
    CPU_TS ts;
    p_arg = p_arg;
    BSP_LED_Off(0);
    char *coor;
    OS_MSG_SIZE size;
    int i;

    while (1) {
        coor = (char*)OSQPend((OS_Q *)&UART_Q,
               (OS_TICK )0,
               (OS_OPT )OS_OPT_PEND_BLOCKING,
               (OS_MSG_SIZE *)&size,
               (CPU_TS *)&ts,
               (OS_ERR *)&err);
        
        BSP_LED_Toggle(2);
        
        posX = 0;
        posY = 0;
        
        for(i=0; coor[i] != ','; i++) {
              posX = posX * 10 + coor[i] - '0';
        }
        
        i++;
        
        for(; coor[i] != '\n'; i++) {
              posY = posY * 10 + coor[i] - '0';
        }
        
        degreeX = coorToDegreeX(posX);
        degreeY = coorToDegreeY(posY);
        
        suboLightXY(coorToDegreeX(posX),coorToDegreeY(posY));
        
        OSMemPut((OS_MEM   *)&UART_MemPool,
                     (void     *)coor,
                     (OS_ERR   *)&err);
        
    }
}
*/

/*
static void AppTask2(void *p_arg) {
    OS_ERR err;
    CPU_TS ts;
    p_arg = p_arg;
    BSP_LED_Off(0);
    char *coor;
    OS_MSG_SIZE size;
    int i;

    while (1) {
      
      OSTmrCreate((OS_TMR*)&MyTmr,
                  (CPU_CHAR*)"MY TIMER",
                  (OS_TICK)50,
                  (OS_TICK)0,
                  (OS_OPT)OS_OPT_TMR_ONE_SHOT,
                  (OS_TMR_CALLBACK_PTR)TmrCallback,
                  (void *)0,
                  (OS_ERR*)&err);
      
      OSTmrStart ((OS_TMR *)&MyTmr,
                  (OS_ERR *)&err);
      OSSemPend(&MySem,
                0,
                OS_OPT_PEND_BLOCKING,
                &ts,
                &err);
      
      //BSP_LED_Toggle(2);
    }
}
*/

void Data_Init(void) {
  int i = 0;
  for(i = 0 ; i < infraredNum ; i ++){
    ADCvalue[i] = 0;
    record[i] = 0;
  }
}

int degreeToPWM(int degree){
   degree -= 15;
   return (2400 - 544) / 360 * degree + 700;
}

void suboDegreeSet(int motornum, int degree) {
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = degreeToPWM(degree);
    
    switch(motornum){
    case 1:
      TIM_OC1Init(TIM3, &TIM_OCInitStructure);
      break;
    case 2:
      TIM_OC3Init(TIM3, &TIM_OCInitStructure);
      break;
    case 3:
      TIM_OC4Init(TIM3, &TIM_OCInitStructure);
      break;
    }

}
void subomotorInitDegree(void){
  suboDegreeSet(1, initDegree);
}

void subomotorSencingDegree(void){
  suboDegreeSet(1, sencingDegree);
}

void subomotorNoSencingDegree(void){
  suboDegreeSet(1, noSencingDergreee);
}

void suboLightXY(int x, int y){
  suboDegreeSet(2, x);
  suboDegreeSet(3, y);
}
void RCC_Config(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    /* TIM3 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
}

void USART_Config(void) {
    GPIO_InitTypeDef Putty_TX;
    GPIO_InitTypeDef Putty_RX;

    Putty_TX.GPIO_Mode  	= GPIO_Mode_AF_PP;
    Putty_TX.GPIO_Pin  	 	= GPIO_Pin_9;
    Putty_TX.GPIO_Speed 	= GPIO_Speed_50MHz;

    Putty_RX.GPIO_Mode  	= GPIO_Mode_IN_FLOATING;
    Putty_RX.GPIO_Pin   	= GPIO_Pin_10;
    Putty_RX.GPIO_Speed 	= GPIO_Speed_50MHz;

    GPIO_Init(GPIOA, &Putty_TX);
    GPIO_Init(GPIOA, &Putty_RX);
    
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength 			= USART_WordLength_8b;
    USART_InitStructure.USART_StopBits 				= USART_StopBits_1;
    USART_InitStructure.USART_Parity 				= USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl 	        = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode 				= USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    BSP_IntVectSet(BSP_INT_ID_USART1, USART1_IRQHandler);
    BSP_IntPrioSet(BSP_INT_ID_USART1,1);  
    //BSP_IntEn(BSP_INT_ID_USART1);  
    
    USART_Cmd(USART1, ENABLE);
}

void USART_InterruptConfigure(){
	EXTI_InitTypeDef Putty_IR;
	NVIC_InitTypeDef Putty_IRQ;

	Putty_IR.EXTI_Line     							= EXTI_Line10;
	Putty_IR.EXTI_LineCmd  							= ENABLE;
	Putty_IR.EXTI_Mode     							= EXTI_Mode_Interrupt;
	Putty_IR.EXTI_Trigger  							= EXTI_Trigger_Rising_Falling;

	Putty_IRQ.NVIC_IRQChannel                   	= BSP_INT_ID_USART1;
	Putty_IRQ.NVIC_IRQChannelPreemptionPriority 	= 0;
	Putty_IRQ.NVIC_IRQChannelSubPriority        	= 0;
	Putty_IRQ.NVIC_IRQChannelCmd                	= ENABLE;

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource10);

	EXTI_Init(&Putty_IR);
	NVIC_Init(&Putty_IRQ);
}

void GPIO_Config(void) {
    GPIO_InitTypeDef GPIO_init;
    GPIO_init.GPIO_Pin = GPIO_Pin_5;
    GPIO_init.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_init);

    GPIO_init.GPIO_Pin = GPIO_Pin_6;
    GPIO_init.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_init);

    GPIO_init.GPIO_Pin = GPIO_Pin_7;
    GPIO_init.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_init);

    GPIO_init.GPIO_Pin = GPIO_Pin_1;
    GPIO_init.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_init);

    GPIO_init.GPIO_Pin = GPIO_Pin_1;
    GPIO_init.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_init);

    GPIO_init.GPIO_Pin = GPIO_Pin_1;
    GPIO_init.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_init);
}


void NVIC_Config(void) {
    NVIC_InitTypeDef NVIC_init;
    NVIC_init.NVIC_IRQChannel = ENABLE;
    NVIC_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_init.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_init.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_init);
}


void ADC_Config(void) {

    ADC_InitTypeDef ADC_init;

    // ADC1 parameter
    ADC_init.ADC_ContinuousConvMode = ENABLE;
    ADC_init.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_init.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_init.ADC_Mode = ADC_Mode_Independent;
    ADC_init.ADC_NbrOfChannel = 6;
    ADC_init.ADC_ScanConvMode = ENABLE;
    ADC_Init(ADC1, &ADC_init);

    // ADC Channel Config
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 2, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 3, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 4, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 5, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 6, ADC_SampleTime_239Cycles5);

    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    // ADC Start
    ADC_ResetCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void DMA_Channel_Config(void) {
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32) & ADC1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32) ADCvalue;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = infraredNum;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel1, ENABLE);    
}

void PWM_Timer_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    u16 PrescalerValue;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB 
      | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

    /*GPIOB Configuration: TIM3 channel1, 2, 3 and 4 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
    //GPIO_PinRemapConfig(GPIO_FullRemap_TIM4, ENABLE);


    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    //TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    PrescalerValue = (u16) (SystemCoreClock / 1000000) - 1;
    TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
    PrescalerValue = (u16) (SystemCoreClock / 1000000) - 1;
    TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

}

void USART1_IRQHandler(){

        OS_ERR err;
        int i = 0;
        
        
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
          BSP_LED_Toggle(1);
          RxData = USART_ReceiveData(USART1);
          if(RxDataCtr == 0) {
            RxDataPtr = OSMemGet(&UART_MemPool, &err);
            *RxDataPtr++ = RxData;
            RxDataCtr = 1;
          }
         
        
          if (RxData == '\n') {
            *RxDataPtr++ = RxData;
            RxDataCtr++;
            
            RxDataPtr = RxDataPtr - (RxDataCtr - 1);
                      
            OSQPost((OS_Q       *)&UART_Q,
                    (void       *)RxDataPtr,
                    (OS_MSG_SIZE )RxDataCtr,
                    (OS_OPT      )OS_OPT_POST_LIFO,
                    (OS_ERR     *)&err);
       
            
            RxDataPtr = NULL;
            RxDataCtr = 0;
          } else {
            *RxDataPtr++ = RxData;
            RxDataCtr++;
          }
        }
}

static int coorToDegreeX(int x) {
  return -1 * ( x * 90 / 639  - 45) ;
}


static int coorToDegreeY(int y) {
  return y *  50 / 479  + 40;
}

void TmrCallback(void) {
  OS_ERR err;
  
  BSP_LED_Toggle(2);
  
  OSSemPost(&timerEndSem,
          OS_OPT_POST_ALL,
          &err);

//  OSMutexPost(&timerEndSem,
//          OS_OPT_POST_ALL,
//          &err);
  
  isDetect = 2;
}


void TmrStopCallback(void) {
  isDetect = 0;
}




  