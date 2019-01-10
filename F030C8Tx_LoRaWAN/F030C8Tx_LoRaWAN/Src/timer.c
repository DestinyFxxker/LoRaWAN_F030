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
    uint32_t elapsedTime = 0;    //过去时间
    uint32_t remainingTime = 0;  //剩余时间

    BoardDisableIrq( );
	  DebugPrintf("GO to BoardDisableIrq\r\n"); //调试用

    if( ( obj == NULL ) || ( TimerExists( obj ) == true ) )
    {
        BoardEnableIrq( );
        return;
    }

    obj->Timestamp = obj->ReloadValue;
    obj->IsRunning = false;

    if( TimerListHead == NULL )
    {
			  DebugPrintf("TimerListHead == NULL\r\n"); //调试用
        TimerInsertNewHeadTimer( obj, obj->Timestamp );
			  DebugPrintf("Go to TimerInsertNewHeadTimer\r\n"); //调试用
    }
    else
    {
			  DebugPrintf("TimerListHead != NULL\r\n"); //调试用
        if( TimerListHead->IsRunning == true )
        {
					  DebugPrintf("TimerListHead->IsRunning == true\r\n"); //调试用
            elapsedTime = TimerGetValue( );//获取距离上一次设置闹钟的时间
					  DebugPrintf("Get elapsedTime ok\r\n"); //调试用
            if( elapsedTime > TimerListHead->Timestamp )
            {
                elapsedTime = TimerListHead->Timestamp; // security but should never occur
            }
            remainingTime = TimerListHead->Timestamp - elapsedTime;//remainingTime表示剩余的头节点中的事件剩余的定时事件
						DebugPrintf("Get remainingTime ok\r\n"); //调试用
        }
        else
        {
					  DebugPrintf("TimerListHead->IsRunning == false\r\n"); //调试用
            remainingTime = TimerListHead->Timestamp;
        }

        if( obj->Timestamp < remainingTime )
        {
				    DebugPrintf("obj->Timestamp < remainingTime\r\n"); //调试用
            TimerInsertNewHeadTimer( obj, remainingTime );
						DebugPrintf("Go to TimerInsertNewHeadTimer\r\n"); //调试用
        }
        else
        {
					   DebugPrintf("obj->Timestamp > remainingTime\r\n"); //调试用
             TimerInsertTimer( obj, remainingTime );
        }
    }
    BoardEnableIrq( );
		DebugPrintf("Go to BoardEnableIrq( )\r\n");//调试用
}

static void TimerInsertTimer( TimerEvent_t *obj, uint32_t remainingTime )
{
    uint32_t aggregatedTimestamp = 0;      // hold the sum of timestamps
    uint32_t aggregatedTimestampNext = 0;  // hold the sum of timestamps up to the next event

    TimerEvent_t* prev = TimerListHead;
    TimerEvent_t* cur = TimerListHead->Next;
		DebugPrintf("TimerListHead->Next\r\n"); //调试用

    if( cur == NULL )
    { 
			  DebugPrintf("cur == NULL\r\n"); //调试用
			// obj comes just after the head
        obj->Timestamp -= remainingTime;
        prev->Next = obj;
        obj->Next = NULL;
    }
    else
    {
			  DebugPrintf("cur != NULL\r\n"); //调试用
        aggregatedTimestamp = remainingTime;
        aggregatedTimestampNext = remainingTime + cur->Timestamp;

        while( prev != NULL )
        {
					  DebugPrintf("prev != NULL\r\n"); //调试用
            if( aggregatedTimestampNext > obj->Timestamp )
            {
							  DebugPrintf("aggregatedTimestampNext > obj->Timestamp\r\n"); //调试用
                obj->Timestamp -= aggregatedTimestamp;
                if( cur != NULL )
                {
									  DebugPrintf("prev != NULL\r\n"); //调试用
                    cur->Timestamp -= obj->Timestamp;
                }
                prev->Next = obj;
                obj->Next = cur;
                break;
            }
            else
            {
							  DebugPrintf("aggregatedTimestampNext !> obj->Timestamp\r\n"); //调试用
                prev = cur;
                cur = cur->Next;
                if( cur == NULL )
                { 
									  DebugPrintf("cur == NULL\r\n"); //调试用
									// obj comes at the end of the list
                    aggregatedTimestamp = aggregatedTimestampNext;
                    obj->Timestamp -= aggregatedTimestamp;
                    prev->Next = obj;
                    obj->Next = NULL;
                    break;
                }
                else
                {
									  DebugPrintf("cur != NULL\r\n"); //调试用
                    aggregatedTimestamp = aggregatedTimestampNext;
                    aggregatedTimestampNext = aggregatedTimestampNext + cur->Timestamp;
                }
            }
        }
    }
}

static void TimerInsertNewHeadTimer( TimerEvent_t *obj, uint32_t remainingTime )
{
    TimerEvent_t* cur = TimerListHead;
		DebugPrintf("TimerEvent_t* cur = TimerListHead \r\n"); //调试用
    if( cur != NULL )//表头不为空，将新的定时器插入之前，将原先表头的定时器时间减去新定时器的定时时间，确保原先的定时器任务定时正常
    {
			  DebugPrintf("cur != NULL\r\n"); //调试用
        cur->Timestamp = remainingTime - obj->Timestamp;
        cur->IsRunning = false;
    }

    obj->Next = cur;
    obj->IsRunning = true;
    TimerListHead = obj;
		DebugPrintf("\r\n\r\n[obj->Next = cur]\r\n[obj->IsRunning = true]\r\n[TimerListHead = obj]\r\n\r\n"); //调试用
    TimerSetTimeout( TimerListHead );//设置超时，等时间到的时候，会发生RTC报警
		DebugPrintf("TimerSetTimeout\r\n"); //调试用
}

void TimerIrqHandler( void )
{
    uint32_t elapsedTime = 0;

    // Early out when TimerListHead is null to prevent null pointer
    if ( TimerListHead == NULL )
    {
        return;
    }

    elapsedTime = TimerGetValue( );

    if( elapsedTime >= TimerListHead->Timestamp )
    {
        TimerListHead->Timestamp = 0;
    }
    else
    {
        TimerListHead->Timestamp -= elapsedTime;
    }

    TimerListHead->IsRunning = false;

    while( ( TimerListHead != NULL ) && ( TimerListHead->Timestamp == 0 ) )
    {
        TimerEvent_t* elapsedTimer = TimerListHead;
        TimerListHead = TimerListHead->Next;

        if( elapsedTimer->Callback != NULL )
        {
            elapsedTimer->Callback( );
        }
    }

    // start the next TimerListHead if it exists
    if( TimerListHead != NULL )
    {
        if( TimerListHead->IsRunning != true )
        {
            TimerListHead->IsRunning = true;
            TimerSetTimeout( TimerListHead );
        }
    }
}

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

    if( TimerListHead == obj ) // Stop the Head
    {
        if( TimerListHead->IsRunning == true ) // The head is already running
        {
            elapsedTime = TimerGetValue( );
            if( elapsedTime > obj->Timestamp )
            {
                elapsedTime = obj->Timestamp;
            }

            remainingTime = obj->Timestamp - elapsedTime;

            if( TimerListHead->Next != NULL )
            {
                TimerListHead->IsRunning = false;
                TimerListHead = TimerListHead->Next;
                TimerListHead->Timestamp += remainingTime;
                TimerListHead->IsRunning = true;
                TimerSetTimeout( TimerListHead );
            }
            else
            {
                TimerListHead = NULL;
            }
        }
        else // Stop the head before it is started
        {
            if( TimerListHead->Next != NULL )
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
    else // Stop an object within the list
    {
        remainingTime = obj->Timestamp;

        while( cur != NULL )
        {
            if( cur == obj )
            {
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

TimerTime_t TimerGetValue( void )
{
    return RtcGetElapsedAlarmTime( );
}

TimerTime_t TimerGetCurrentTime( void )
{
    return RtcGetTimerValue( );
}

TimerTime_t TimerGetElapsedTime( TimerTime_t savedTime )
{
    return RtcComputeElapsedTime( savedTime );
}

TimerTime_t TimerGetFutureTime( TimerTime_t eventInFuture )
{
    return RtcComputeFutureEventTime( eventInFuture );
}

static void TimerSetTimeout( TimerEvent_t *obj )
{
    HasLoopedThroughMain = 0;
    obj->Timestamp = RtcGetAdjustedTimeoutValue( obj->Timestamp );
    RtcSetTimeout( obj->Timestamp );
}

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
                RtcEnterLowPowerStopMode( );
#endif
            }
        }
    }
}
