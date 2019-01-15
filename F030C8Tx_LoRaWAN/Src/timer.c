/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Timer objects and scheduling management

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "board.h"
#include "rtc-board.h"


/*!
 * This flag is used to make sure we have looped through the main several time to avoid race issues
 */
volatile uint8_t HasLoopedThroughMain = 0;

/*!
 * Timers list head pointer
 */
static TimerEvent_t *TimerListHead = NULL;

/*!
 * \brief Adds or replace the head timer of the list.
 *
 * \remark The list is automatically sorted. The list head always contains the
 *         next timer to expire.
 *
 * \param [IN]  obj Timer object to be become the new head
 * \param [IN]  remainingTime Remaining time of the previous head to be replaced
 */
static void TimerInsertNewHeadTimer( TimerEvent_t *obj, uint32_t remainingTime );

/*!
 * \brief Adds a timer to the list.
 *
 * \remark The list is automatically sorted. The list head always contains the
 *         next timer to expire.
 *
 * \param [IN]  obj Timer object to be added to the list
 * \param [IN]  remainingTime Remaining time of the running head after which the object may be added
 */
static void TimerInsertTimer( TimerEvent_t *obj, uint32_t remainingTime );

/*!
 * \brief Sets a timeout with the duration "timestamp"
 *
 * \param [IN] timestamp Delay duration
 */
static void TimerSetTimeout( TimerEvent_t *obj );

/*!
 * \brief Check if the Object to be added is not already in the list
 *
 * \param [IN] timestamp Delay duration
 * \retval true (the object is already in the list) or false
 */
static bool TimerExists( TimerEvent_t *obj );

/*!
 * \brief Read the timer value of the currently running timer
 *
 * \retval value current timer value
 */
TimerTime_t TimerGetValue( void );

void TimerInit( TimerEvent_t *obj, void ( *callback )( void ) )
{
    obj->Timestamp = 0;
    obj->ReloadValue = 0;
    obj->IsRunning = false;
    obj->Callback = callback;
    obj->Next = NULL;
}

void TimerStart( TimerEvent_t *obj )
{
	
    uint32_t elapsedTime = 0;
    uint32_t remainingTime = 0;  //ʣ��ʱ��=����ͷ��ʱ��ʱ���-���ϴ������¼�֮���ѹ���ʱ���

    BoardDisableIrq( );

    if( ( obj == NULL ) || ( TimerExists( obj ) == true ) )
    {
        BoardEnableIrq( );
        return;
    }

    obj->Timestamp = obj->ReloadValue;
    obj->IsRunning = false;

    if( TimerListHead == NULL )
    {
        TimerInsertNewHeadTimer( obj, obj->Timestamp );
    }
    else
    {
        if( TimerListHead->IsRunning == true )
        {
            elapsedTime = TimerGetValue( );
            if( elapsedTime > TimerListHead->Timestamp )
            {
                elapsedTime = TimerListHead->Timestamp; // security but should never occur
            }
            remainingTime = TimerListHead->Timestamp - elapsedTime;
        }
        else
        {
            remainingTime = TimerListHead->Timestamp;
        }

        if( obj->Timestamp < remainingTime )
        {
            TimerInsertNewHeadTimer( obj, remainingTime );
        }
        else
        {
             TimerInsertTimer( obj, remainingTime );
        }
    }
    BoardEnableIrq( );
}

static void TimerInsertTimer( TimerEvent_t *obj, uint32_t remainingTime )
{
    uint32_t aggregatedTimestamp = 0;      // hold the sum of timestamps
    uint32_t aggregatedTimestampNext = 0;  // hold the sum of timestamps up to the next event

    TimerEvent_t* prev = TimerListHead;       //��ʱ������ͷָ��
    TimerEvent_t* cur = TimerListHead->Next;  //��ͷ�ڵ��βָ��ָ��ڶ�����ʱ��

    if( cur == NULL )                         //���û����һ������,��βָ������
    { // obj comes just after the head
        obj->Timestamp -= remainingTime;      //�������붨ʱ��ʱ�������ȥʣ��ʱ��
        prev->Next = obj;
        obj->Next = NULL;                     //��βָ������
    }
    else                                      //��ʱ���������������϶�ʱ������Ҫ����ʣ��ʱ��ĶԱȡ���ʱ�������ǰ���ʣ��ʱ�䳤�������
    {
        aggregatedTimestamp = remainingTime;
        aggregatedTimestampNext = remainingTime + cur->Timestamp;

        while( prev != NULL )
        {
					  //ͨ��ѭ���Ա����������е�ÿ����ʱ�������ʱ���������ʱ����ȼ������붨ʱ��ʱ���С��λ�ã�
            //Ȼ�󽫶�ʱ�����뵽��λ��ǰ�棬�������²���λ��֮��ÿ����ʱ����ʱ�����
            if( aggregatedTimestampNext > obj->Timestamp )   //�ҵ����붨ʱ����λ��
            {
                obj->Timestamp -= aggregatedTimestamp;       //����õ�obj���������prev�����ʱ���
                if( cur != NULL )
                {
                    cur->Timestamp -= obj->Timestamp;       //����õ�cur���������obj�����ʱ���
                }
                prev->Next = obj;                           //����ָ�룬���붨ʱ����prev�������һ��λ��      
                obj->Next = cur;                            //����ָ�룬ʹcur������obj�������һλ��     
                break;                                      //������ɣ��˳�
            }
            else                                            //ͨ���ı�ǰһ��ָ��͵�ǰָ���λ�ã������²��붨ʱ����λ��
            {
                prev = cur;
                cur = cur->Next;
                if( cur == NULL )
                { // obj comes at the end of the list
                    aggregatedTimestamp = aggregatedTimestampNext;
                    obj->Timestamp -= aggregatedTimestamp;              //����õ�obj���������prev�����ʱ���
                    prev->Next = obj;                                   //����ָ�룬���붨ʱ����prev�������һ��λ��            
                    obj->Next = NULL;                                   //����ָ�룬obj�������һλ��Ϊ��(����β��)
                    break;                                              //������ɣ��˳�
                }
                else
                {
                    aggregatedTimestamp = aggregatedTimestampNext;                      //������prev�����ʱ���
                    aggregatedTimestampNext = aggregatedTimestampNext + cur->Timestamp; //������cur�����ʱ���
                }
            }
        }
    }
}

static void TimerInsertNewHeadTimer( TimerEvent_t *obj, uint32_t remainingTime )
{//���ﴫ���remainingTimeΪ1000
    TimerEvent_t* cur = TimerListHead;

    if( cur != NULL )
    {
        cur->Timestamp = remainingTime - obj->Timestamp;
        cur->IsRunning = false;
    }

    obj->Next = cur;                        //�²��붨ʱ������һ������ָ��ǰͷָ��
    obj->IsRunning = true;                  //��ʱ������״̬
    TimerListHead = obj;                    //�ı�ͷָ�룬ָ���²���Ķ�ʱ������
    TimerSetTimeout( TimerListHead );       //���ö�ʱ����ʱʱ��
}

void TimerIrqHandler( void )
{
    uint32_t elapsedTime = 0;

    // Early out when TimerListHead is null to prevent null pointer
	  //��TimerListHeadΪ��ʱ��ǰ�˳��Է�ֹ��ָ��
    if ( TimerListHead == NULL )
    {
			  //DebugPrintf("TimerListHead == NULL\r\n");
        return;
    }

    elapsedTime = TimerGetValue( );  //�������һ�������жϵ����ڵ�ʱ��λms
		
    if( elapsedTime >= TimerListHead->Timestamp )  //���������һ���жϵ�ʱ���Ѿ������˱�ͷ�¼������õ�ʱ���򽫱�ͷ�¼���ʱ����0,Ϊ֮��Ĵ�����׼��
    {
				//DebugPrintf("��ǰʱ�䳬�������õ�ʱ��\r\n");
				//DebugPrintf("elapsedTime is %f\r\n",elapsedTime);
				//DebugPrintf("TimerListHead->Timestamp is %f\r\n",TimerListHead->Timestamp);
        TimerListHead->Timestamp = 0;                //���¼�ʱ�������
    }
    else
    {
			  //DebugPrintf("��ǰʱ��С���¼�ʱ��\r\n");
        TimerListHead->Timestamp -= elapsedTime;   //TimerListHead->Timestamp=TimerListHead->Timestamp-elapsedTime
			  //DebugPrintf("TimerListHead->Timestamp is %f\r\nelapsedTime is %f\r\n",TimerListHead->Timestamp,elapsedTime);
    }                     //�����û���Ļ���elapsedTime<TimerListHead->Timestamp�ͽ����õ�ʱ���ȥ��ǰ��ʱ��(�ѹ���ʱ��)

    TimerListHead->IsRunning = false;
    while( ( TimerListHead != NULL ) && ( TimerListHead->Timestamp == 0 ) )  //�����㳬ʱ����ʱִ�лص�����
    {
			  //DebugPrintf("���㳬ʱ����,��ʼִ�лص�����\r\n");
        TimerEvent_t* elapsedTimer = TimerListHead; //����ͷλ���ó�
        TimerListHead = TimerListHead->Next;
        if( elapsedTimer->Callback != NULL )
        {
					  //DebugPrintf("�ص���������,ִ��\r\n");
            elapsedTimer->Callback( );      //ִ�лص�����
        }
				else
				{
					//DebugPrintf("�ص�����������\r\n");
				}
    }

    // start the next TimerListHead if it exists

    if( TimerListHead != NULL )
    {
			  //DebugPrintf("��һ���¼�δִ�л������һ���¼�\r\n");
        if( TimerListHead->IsRunning != true )
        {
            TimerListHead->IsRunning = true;
            TimerSetTimeout( TimerListHead );
        }
    }
}
//ֹͣ�Ķ�ʱ���պ�������ͷ�Ķ��󡣲����Ƕ�ʱ�������е�����������õ�ʣ��ʱ�䣬������ͷ����ָ��ָ����һ����������������ͷ����
//���Ƕ�ʱ������ֹͣ���������ֱ�Ӱ�����ͷָ��ָ����һ������
//ֹͣ�Ķ�ʱ���������ڲ�����λ�á���Ҫ��������ѭ��������Ҫֹͣ�Ķ�ʱ�������ҵ�֮�󣬵������ڶ���ָ��ָ��λ�ã����ɰ�ָ����ʱ����������ɾ����
//���ǰѶ�ʱ����������ɾ��,û��ɾ����ʱ������
//ָֹͣ���¼��Ķ�ʱ��
void TimerStop( TimerEvent_t *obj )
{
    BoardDisableIrq( );

    uint32_t elapsedTime = 0;
    uint32_t remainingTime = 0;

    TimerEvent_t* prev = TimerListHead;
    TimerEvent_t* cur = TimerListHead;

    // List is empty or the Obj to stop does not exist
    if( ( TimerListHead == NULL ) || ( obj == NULL ) )
    {
        BoardEnableIrq( );
        return;
    }

    if( TimerListHead == obj ) // Stop the Head   //ֹͣ�Ķ�ʱ������������ͷλ��
    {
        if( TimerListHead->IsRunning == true ) // The head is already running  //����ͷ��ʱ������������״̬
        {
            elapsedTime = TimerGetValue( );     //��ȡ�ѹ�ʱ���
            if( elapsedTime > obj->Timestamp )  //�����ȥ��ʱ������¼��Ķ�ʱ(��ʱ)
            {
                elapsedTime = obj->Timestamp;   //���¼�ʱ��ڵ�������¼��Ķ�ʱ���Ӵ�,�˾����������Ľ��¼��Ķ�ʱ�������������޳�
            }

            remainingTime = obj->Timestamp - elapsedTime;   //���û�г�ʱ,�����ʣ��ʱ��
            if( TimerListHead->Next != NULL )               //��ʱ�����������������϶���
            {
                TimerListHead->IsRunning = false;
                TimerListHead = TimerListHead->Next;        //����һ�¼��Ž�ͷ�ڵ���
                TimerListHead->Timestamp += remainingTime;  //ͷ�ڵ�Ķ�ʱʱ�����Ҫ����ʣ���ʱ��
							                                              //��Ϊ��������RTC����Ƕ�ʱ������,�¼����¼�֮�䲢��ƽ�ж����Թ̶���ʱ�������
							                                              //��ֹͣĳ�м��¼�ʱΪ�˲�Ӱ������¼�,����Ҫ��ֹͣ�¼���ʱ�䱣��
                TimerListHead->IsRunning = true;
                TimerSetTimeout( TimerListHead );           //������ʱ���������һ���¼���ʱ��
            }
            else
            {
                TimerListHead = NULL;                       //��ʱ������ֻ��һ������ֱ�ӽ���ͷ����
            }
        }
        else // Stop the head before it is started          //����ͷ��ʱ��������ֹͣ״̬
        {
            if( TimerListHead->Next != NULL )               //������һ���¼�
            {
                remainingTime = obj->Timestamp;
                TimerListHead = TimerListHead->Next;
                TimerListHead->Timestamp += remainingTime;
            }
            else
            {
                TimerListHead = NULL;                       
            }
        }
    }
    else // Stop an object within the list                  //ֹͣ�Ķ�ʱ��������������
    {
        remainingTime = obj->Timestamp;
//ѭ��������Ҫֹͣ�Ķ�ʱ������
        while( cur != NULL )
        {
            if( cur == obj )
            {//���������ָ��
                if( cur->Next != NULL )
                {
                    cur = cur->Next;
                    prev->Next = cur;
                    cur->Timestamp += remainingTime;
                }
                else
                {
                    cur = NULL;
                    prev->Next = cur;
                }
                break;
            }
            else
            {
                prev = cur;
                cur = cur->Next;
            }
        }
    }
    BoardEnableIrq( );
}

static bool TimerExists( TimerEvent_t *obj )
{
    TimerEvent_t* cur = TimerListHead;

    while( cur != NULL )
    {
        if( cur == obj )
        {
            return true;
        }
        cur = cur->Next;
    }
    return false;
}

void TimerReset( TimerEvent_t *obj )
{
    TimerStop( obj );
    TimerStart( obj );
}

void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{
    TimerStop( obj );
    obj->Timestamp = value;
    obj->ReloadValue = value;
}


/****************************************************************************************
�����ĸ��������ڲ�RTCʱ��ʱ�������ת�������з�װ�������ö�ʱ��ʹ�á�
��������ת����ʱ���
****************************************************************************************/
TimerTime_t TimerGetValue( void )
{
	//////��ʱ����ȡ�������һ�����ӻ���֮�������ڵ�ʱ���ֵ ��λms
    return RtcGetElapsedAlarmTime( );
}
//��ʱ�����ڲ�RTC��ȡ��ǰʱ�����ʵ����ϵͳ�������е�ʱ�䡣
//TimerGetCurrentTime()ʵ����ͨ����ȡ��ǰRTCʱ�ӵ���������ת����ʱ��������ʱ��ֵ�Ǿ���ϵͳ���������е���ǰ��ֵ
TimerTime_t TimerGetCurrentTime( void )
{
    return RtcGetTimerValue( );
}
//��ʱ����ȡ����savedTime��ʱ�����ֵ
//TimerGetElapsedTime( TimerTime_t savedTime )�ǻ�ȡ��ǰ���봫��ȥ���ѹ�ʱ���savedTime��ʱ�����ֵs
TimerTime_t TimerGetElapsedTime( TimerTime_t savedTime )
{
    return RtcComputeElapsedTime( savedTime );
}
//��ʱ����ȡ�����¼���ʱ�䣬��ǰʱ�����eventInFuture��ʱ�����ֵ
//��ȡ��ǰʱ�����δ��ʱ��eventInFuture��ʱ�����ֵ
TimerTime_t TimerGetFutureTime( TimerTime_t eventInFuture )
{
    return RtcComputeFutureEventTime( eventInFuture );
}

/****************************************************************************************/




static void TimerSetTimeout( TimerEvent_t *obj )
{
    HasLoopedThroughMain = 0;
    obj->Timestamp = RtcGetAdjustedTimeoutValue( obj->Timestamp ); //��ʱ����ȥ��ʱ�仹������1000
		//����ϵͳ������McuWakeUpTime���������ߣ�
		//	������趨��ʱ����ʱʱ���ʱ��
		//	Ҫ�������趨��ʱ�䳤���Ƿ񳬹�һ��McuWakeUpTime���ڡ�
		//	�������һ��McuWakeUpTime���ڵĻ���Ҫ���趨�ĳ�ʱʱ����е�����
		//	ʹ֮����һ��(���������)�ı��ֻ�����������ɶ�ʱ������
		//	����RtcGetAdjustedTimeoutValue()��������McuWakeUpTime������ʱʱ�䡣
    RtcSetTimeout( obj->Timestamp );  //����Ҫ�趨�ĳ�ʱʱ��(��ʱ����1000)���õ�RTC�У������������ж�
}
extern uint8_t isRxWindowOpen;
void TimerLowPowerHandler( void )
{
    if( ( TimerListHead != NULL ) && ( TimerListHead->IsRunning == true ) )
    {
        if( HasLoopedThroughMain < 5 )
        {
            HasLoopedThroughMain++;
        }
        else
        {
            HasLoopedThroughMain = 0;
            if( GetBoardPowerSource( ) == BATTERY_POWER )
            {
#if !defined( USE_DEBUGGER ) 
							if(isRxWindowOpen==0)
							{
								GpioWrite( &ENrf, 1 );
							}
								GpioWrite( &ENSensor, 1 );
								SleepIOConfig();            
                RtcEnterLowPowerStopMode( );
#endif
            }
        }
    }
}
