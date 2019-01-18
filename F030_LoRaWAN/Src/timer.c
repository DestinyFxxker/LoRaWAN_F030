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
    uint32_t remainingTime = 0;  //剩余时间=链表头定时器时间戳-自上次闹钟事件之后已过的时间戳

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

    TimerEvent_t* prev = TimerListHead;       //定时器链表头指针
    TimerEvent_t* cur = TimerListHead->Next;  //将头节点的尾指针指向第二个定时器

    if( cur == NULL )                         //如果没有下一个对象,将尾指针悬空
    { // obj comes just after the head
        obj->Timestamp -= remainingTime;      //调整插入定时器时间戳，减去剩余时间
        prev->Next = obj;
        obj->Next = NULL;                     //将尾指针悬空
    }
    else                                      //定时器链表有两个以上定时器对象，要进行剩余时间的对比。定时器链表是按照剩余时间长短排序的
    {
        aggregatedTimestamp = remainingTime;
        aggregatedTimestampNext = remainingTime + cur->Timestamp;

        while( prev != NULL )
        {
					  //通过循环对比链表中现有的每个定时器对象的时间戳，查找时间戳比即将插入定时器时间戳小的位置，
            //然后将定时器插入到该位置前面，并调整新插入位置之后每个定时器的时间戳。
            if( aggregatedTimestampNext > obj->Timestamp )   //找到插入定时器的位置
            {
                obj->Timestamp -= aggregatedTimestamp;       //计算得到obj对象相对于prev对象的时间戳
                if( cur != NULL )
                {
                    cur->Timestamp -= obj->Timestamp;       //计算得到cur对象相对于obj对象的时间戳
                }
                prev->Next = obj;                           //调整指针，插入定时器到prev对象的下一个位置      
                obj->Next = cur;                            //调整指针，使cur对象处于obj对象的下一位置     
                break;                                      //插入完成，退出
            }
            else                                            //通过改变前一个指针和当前指针的位置，搜索新插入定时器的位置
            {
                prev = cur;
                cur = cur->Next;
                if( cur == NULL )
                { // obj comes at the end of the list
                    aggregatedTimestamp = aggregatedTimestampNext;
                    obj->Timestamp -= aggregatedTimestamp;              //计算得到obj对象相对于prev对象的时间戳
                    prev->Next = obj;                                   //调整指针，插入定时器到prev对象的下一个位置            
                    obj->Next = NULL;                                   //调整指针，obj对象的下一位置为空(链表尾部)
                    break;                                              //插入完成，退出
                }
                else
                {
                    aggregatedTimestamp = aggregatedTimestampNext;                      //调整到prev对象的时间戳
                    aggregatedTimestampNext = aggregatedTimestampNext + cur->Timestamp; //调整到cur对象的时间戳
                }
            }
        }
    }
}

static void TimerInsertNewHeadTimer( TimerEvent_t *obj, uint32_t remainingTime )
{//这里传入的remainingTime为1000
    TimerEvent_t* cur = TimerListHead;

    if( cur != NULL )
    {
        cur->Timestamp = remainingTime - obj->Timestamp;
        cur->IsRunning = false;
    }

    obj->Next = cur;                        //新插入定时器的下一个对象指向当前头指针
    obj->IsRunning = true;                  //定时器运行状态
    TimerListHead = obj;                    //改变头指针，指向新插入的定时器对象
    TimerSetTimeout( TimerListHead );       //设置定时器超时时间
}

void TimerIrqHandler( void )
{
    uint32_t elapsedTime = 0;

    // Early out when TimerListHead is null to prevent null pointer
	  //当TimerListHead为空时提前退出以防止空指针
    if ( TimerListHead == NULL )
    {
			  //DebugPrintf("TimerListHead == NULL\r\n");
        return;
    }

    elapsedTime = TimerGetValue( );  //返回最近一次闹钟中断到现在的时间差单位ms
		
    if( elapsedTime >= TimerListHead->Timestamp )  //如果距离上一次中断的时间已经超过了表头事件中设置的时间则将表头事件的时间置0,为之后的触发做准备
    {
				//DebugPrintf("当前时间超过了设置的时间\r\n");
				//DebugPrintf("elapsedTime is %f\r\n",elapsedTime);
				//DebugPrintf("TimerListHead->Timestamp is %f\r\n",TimerListHead->Timestamp);
        TimerListHead->Timestamp = 0;                //将事件时间戳清零
    }
    else
    {
			  //DebugPrintf("当前时间小于事件时间\r\n");
        TimerListHead->Timestamp -= elapsedTime;   //TimerListHead->Timestamp=TimerListHead->Timestamp-elapsedTime
			  //DebugPrintf("TimerListHead->Timestamp is %f\r\nelapsedTime is %f\r\n",TimerListHead->Timestamp,elapsedTime);
    }                     //如果还没到的话则elapsedTime<TimerListHead->Timestamp就将设置的时间减去当前的时间(已过的时间)

    TimerListHead->IsRunning = false;
    while( ( TimerListHead != NULL ) && ( TimerListHead->Timestamp == 0 ) )  //当满足超时条件时执行回调函数
    {
			  //DebugPrintf("满足超时条件,开始执行回调函数\r\n");
        TimerEvent_t* elapsedTimer = TimerListHead; //将表头位置让出
        TimerListHead = TimerListHead->Next;
        if( elapsedTimer->Callback != NULL )
        {
					  //DebugPrintf("回调函数存在,执行\r\n");
            elapsedTimer->Callback( );      //执行回调函数
        }
				else
				{
					//DebugPrintf("回调函数不存在\r\n");
				}
    }

    // start the next TimerListHead if it exists

    if( TimerListHead != NULL )
    {
			  //DebugPrintf("上一个事件未执行或存在下一个事件\r\n");
        if( TimerListHead->IsRunning != true )
        {
            TimerListHead->IsRunning = true;
            TimerSetTimeout( TimerListHead );
        }
    }
}
//停止的定时器刚好是链表头的对象。并且是定时器在运行的情况，则计算得到剩余时间，把链表头对象指针指向下一个对象，启动新链表头对象
//若是定时器处于停止的情况，则直接把链表头指针指向下一个对象。
//停止的定时器在链表内部其它位置。需要在链表中循环查找需要停止的定时器对象，找到之后，调整相邻对象指针指向位置，即可把指定定时器从链表中删除。
//仅是把定时器从链表中删除,没有删除定时器对象
//停止指定事件的定时器
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

    if( TimerListHead == obj ) // Stop the Head   //停止的定时器对象处于链表头位置
    {
        if( TimerListHead->IsRunning == true ) // The head is already running  //链表头定时器对象处于运行状态
        {
            elapsedTime = TimerGetValue( );     //获取已过时间戳
            if( elapsedTime > obj->Timestamp )  //如果过去的时间大于事件的定时(超时)
            {
                elapsedTime = obj->Timestamp;   //将事件时间节点调整到事件的定时闹钟处,此举在于完整的将事件的定时从整个链表中剔除
            }

            remainingTime = obj->Timestamp - elapsedTime;   //如果没有超时,则计算剩余时间
            if( TimerListHead->Next != NULL )               //定时器链表有两个及以上对象
            {
                TimerListHead->IsRunning = false;
                TimerListHead = TimerListHead->Next;        //将下一事件放进头节点中
                TimerListHead->Timestamp += remainingTime;  //头节点的定时时间戳需要加上剩余的时间
							                                              //因为此链表是RTC链表非定时器链表,事件与事件之间并非平行而是以固定的时间段相连
							                                              //当停止某中间事件时为了不影响其后事件,则需要将停止事件的时间保留
                TimerListHead->IsRunning = true;
                TimerSetTimeout( TimerListHead );           //启动定时器链表的下一个事件定时器
            }
            else
            {
                TimerListHead = NULL;                       //定时器链表只有一个对象直接将表头悬空
            }
        }
        else // Stop the head before it is started          //链表头定时器对象处于停止状态
        {
            if( TimerListHead->Next != NULL )               //存在下一个事件
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
    else // Stop an object within the list                  //停止的定时器对象在链表内
    {
        remainingTime = obj->Timestamp;
//循环查找需要停止的定时器对象
        while( cur != NULL )
        {
            if( cur == obj )
            {//调整链表的指针
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
以下四个函数对内部RTC时钟时间进行了转换，进行封装后以适用定时器使用。
将脉冲数转换成时间戳
****************************************************************************************/
TimerTime_t TimerGetValue( void )
{
	//////定时器获取距离最近一次闹钟唤醒之后与现在的时间差值 单位ms
    return RtcGetElapsedAlarmTime( );
}
//定时器从内部RTC获取当前时间戳，实际是系统启动运行的时间。
//TimerGetCurrentTime()实际是通过获取当前RTC时钟的脉冲数，转换成时间戳，这个时间值是距离系统自启动运行到当前的值
TimerTime_t TimerGetCurrentTime( void )
{
    return RtcGetTimerValue( );
}
//定时器获取距离savedTime的时间戳差值
//TimerGetElapsedTime( TimerTime_t savedTime )是获取当前距离传进去的已过时间戳savedTime的时间戳差值s
TimerTime_t TimerGetElapsedTime( TimerTime_t savedTime )
{
    return RtcComputeElapsedTime( savedTime );
}
//定时器获取将来事件的时间，当前时间距离eventInFuture的时间戳差值
//获取当前时间距离未来时间eventInFuture的时间戳差值
TimerTime_t TimerGetFutureTime( TimerTime_t eventInFuture )
{
    return RtcComputeFutureEventTime( eventInFuture );
}

/****************************************************************************************/




static void TimerSetTimeout( TimerEvent_t *obj )
{
    HasLoopedThroughMain = 0;
    obj->Timestamp = RtcGetAdjustedTimeoutValue( obj->Timestamp ); //此时传进去的时间还是数字1000
		//由于系统在运行McuWakeUpTime后会进入休眠，
		//	因此在设定定时器超时时间的时候，
		//	要考虑所设定的时间长度是否超过一个McuWakeUpTime周期。
		//	如果超过一个McuWakeUpTime周期的话，要对设定的超时时间进行调整，
		//	使之在下一个(或则更后面)的保持唤醒周期内完成定时器任务。
		//	调用RtcGetAdjustedTimeoutValue()函数根据McuWakeUpTime调整超时时间。
    RtcSetTimeout( obj->Timestamp );  //把需要设定的超时时间(此时还是1000)设置到RTC中，并启动闹钟中断
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
