// generic imports
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <math.h>

// nrf
#include "app_scheduler.h"
// #include "app_timer.h"
#include "bsp_thread.h"
#include "nrf_log_ctrl.h"
#include "nrf_log.h"
#include "nrf_log_default_backends.h"

// gpio
// #include "drv_gpio.h"

// timer
#include "nrf.h"
#include "nrf_drv_timer.h"
#include "bsp.h"
#include "app_error.h"

// saadc
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"

// mqtt
#include "mqttsn_client.h"

// thread
#include "thread_utils.h"

#include <openthread/thread.h>
#include <openthread/thread_ftd.h>
#include <openthread/dataset_ftd.h>
#include <openthread/config.h>
#include <openthread/cli.h>
#include <openthread/diag.h>
#include <openthread/tasklet.h>
#include <openthread/platform/logging.h>

#define COMPARE_COUNTERTIME  (3UL) /**< Get Compare event COMPARE_TIME seconds after the counter starts from 0. */
#define SCHED_QUEUE_SIZE      32                              /**< Maximum number of events in the scheduler queue. */
#define SCHED_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum app_scheduler event size. */

// mqtt-sn

#define DEFAULT_CHILD_TIMEOUT    40                                         /**< Thread child timeout [s]. */
#define DEFAULT_POLL_PERIOD      1000                                       /**< Thread Sleepy End Device polling period when MQTT-SN Asleep. [ms] */
#define SHORT_POLL_PERIOD        100                                        /**< Thread Sleepy End Device polling period when MQTT-SN Awake. [ms] */
#define SEARCH_GATEWAY_TIMEOUT   5                                          /**< MQTT-SN Gateway discovery procedure timeout in [s]. */                                   

static mqttsn_client_t      m_client;                                       /**< An MQTT-SN client instance. */
static mqttsn_remote_t      m_gateway_addr;                                 /**< A gateway address. */
static uint8_t              m_gateway_id;                                   /**< A gateway ID. */
static mqttsn_connect_opt_t m_connect_opt;                                  /**< Connect options for the MQTT-SN client. */
static uint16_t             m_msg_id           = 0;                         /**< Message ID thrown with MQTTSN_EVENT_TIMEOUT. */
static char                 m_client_id[]      = "current-sensor-01";      /**< The MQTT-SN Client's ID. */
static char                 m_topic_name[]     = "gb-0134/mains"; /**< Name of the topic corresponding to subscriber's BSP_LED_2. */
static mqttsn_topic_t       m_topic            =                            /**< Topic corresponding to subscriber's BSP_LED_2. */
{
    .p_topic_name = (unsigned char *)m_topic_name,
    .topic_id     = 0,
};

// end mqtt sn
// saadc

#define SAMPLES_IN_BUFFER 1000

static char buffer[200] = {};

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(2);
static nrf_saadc_value_t     m_buffer[SAMPLES_IN_BUFFER];

// end saadc
static void find_gateway();
static bool has_valid_role();
static void wake_up(void);
static void set_radio_on(void);
static void set_radio_off(void);

/***************************************************************************************************
 * @section State
 **************************************************************************************************/

static void thread_state_changed_callback(uint32_t flags, void * p_context)
{

}

uint32_t get_rtc_counter(void)
{
    return NRF_RTC2->COUNTER;
}

/***************************************************************************************************
 * @section Initialization
 **************************************************************************************************/

/**@brief Function for initializing the LEDs.
 */
static void leds_init(void)
{
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(get_rtc_counter, 32768);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing the Thread Board Support Package.
 */
static void thread_bsp_init(void)
{
    uint32_t err_code;

    err_code = otPlatRadioSetTransmitPower(thread_ot_instance_get(), 0);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Thread Stack.
 */
static void thread_instance_init(void)
{
    thread_configuration_t thread_configuration =
    {
        .radio_mode            = THREAD_RADIO_MODE_RX_OFF_WHEN_IDLE,
        .autocommissioning     = true,
        .poll_period           = DEFAULT_POLL_PERIOD,
        .default_child_timeout = DEFAULT_CHILD_TIMEOUT,
    };

    thread_init(&thread_configuration);
    // thread_cli_init();
    thread_state_changed_callback_set(thread_state_changed_callback);
}

void setNetworkConfiguration(otInstance *aInstance)
{
    otOperationalDataset aDataset;

    memset(&aDataset, 0, sizeof(otOperationalDataset));
   
    aDataset.mActiveTimestamp                      = 1;
    aDataset.mComponents.mIsActiveTimestampPresent = true;
     
    aDataset.mChannel                      = 15;
    aDataset.mComponents.mIsChannelPresent = true;
    
    aDataset.mPanId                      = (otPanId)0x1234;
    aDataset.mComponents.mIsPanIdPresent = true;

    uint8_t MeshLocalPrefix[OT_PSKC_MAX_SIZE] = {0xfd, 0x11, 0x11, 0x11, 0x22, 0x22, 0x00, 0x00};
    memcpy(aDataset.mMeshLocalPrefix.m8, MeshLocalPrefix, sizeof(aDataset.mMeshLocalPrefix));
    aDataset.mComponents.mIsMeshLocalPrefixPresent = true;

    uint8_t key[OT_MASTER_KEY_SIZE] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
    memcpy(aDataset.mMasterKey.m8, key, sizeof(aDataset.mMasterKey));
    aDataset.mComponents.mIsMasterKeyPresent = true;

#if OPENTHREAD_FTD
    otDatasetSetActive(aInstance, &aDataset);
#else
    OT_UNUSED_VARIABLE(aInstance);
#endif
}

/***************************************************************************************************
 * @section MQTT-SN
 **************************************************************************************************/


static bool has_valid_role() 
{
    if (otThreadGetDeviceRole(thread_ot_instance_get()) < OT_DEVICE_ROLE_CHILD )
    {
        return false;
    }
    return true;
}

static void find_gateway() 
{
    if (!has_valid_role()) 
    {
        return;
    }
    wake_up();
    set_radio_on();

    uint32_t err_code = mqttsn_client_search_gateway(&m_client, SEARCH_GATEWAY_TIMEOUT);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("SEARCH GATEWAY message could not be sent. Error: 0x%x\r\n", err_code);
    }
}

static void connect_mqttsn()
{
    if (!has_valid_role()) 
    {
        return;
    }
    wake_up();
    set_radio_on();

    NRF_LOG_DEBUG("Sent CONNECT");
    uint32_t err_code = mqttsn_client_connect(&m_client, &m_gateway_addr, m_gateway_id, &m_connect_opt);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("CONNECT message could not be sent. Error: 0x%x\r\n", err_code);
    }
}

/**@brief Puts MQTT-SN client in sleep mode.
 *
 * @details This function changes Thread Sleepy End Device polling period to default.
 */
static void sleep(void)
{
    NRF_LOG_DEBUG("Sleep");
    otError error;

    error = otLinkSetPollPeriod(thread_ot_instance_get(), DEFAULT_POLL_PERIOD);
    ASSERT(error == OT_ERROR_NONE);
}

/**@brief Puts MQTT-SN client in active mode.
 *
 * @details This function changes Thread Sleepy End Device polling period to short.
 */
static void wake_up(void)
{
    NRF_LOG_DEBUG("Wake up");
    otError error;

    error = otLinkSetPollPeriod(thread_ot_instance_get(), SHORT_POLL_PERIOD);
    ASSERT(error == OT_ERROR_NONE);
}

/**@brief Initializes MQTT-SN client's connection options.
 */
static void connect_opt_init(void)
{
    m_connect_opt.alive_duration = MQTTSN_DEFAULT_ALIVE_DURATION,
    m_connect_opt.clean_session  = MQTTSN_DEFAULT_CLEAN_SESSION_FLAG,
    m_connect_opt.will_flag      = MQTTSN_DEFAULT_WILL_FLAG,
    m_connect_opt.client_id_len  = strlen(m_client_id),

    memcpy(m_connect_opt.p_client_id, (unsigned char *)m_client_id, m_connect_opt.client_id_len);
}

/**@brief Processes GWINFO message from a gateway.
 *
 * @details This function initializes MQTT-SN Client's connect options and launches the connect procedure.
 *
 * @param[in]    p_event  Pointer to MQTT-SN event.
 */
static void gateway_info_callback(mqttsn_event_t * p_event)
{
    m_gateway_addr  = *(p_event->event_data.connected.p_gateway_addr);
    m_gateway_id    = p_event->event_data.connected.gateway_id;
}

/**@brief Processes CONNACK message from a gateway.
 *
 * @details This function launches the topic registration procedure if necessary.
 */
static void connected_callback(void)
{
    NRF_LOG_DEBUG("connected_callback");
    set_radio_off();

    uint32_t err_code = mqttsn_client_topic_register(&m_client,
                                                     m_topic.p_topic_name,
                                                     strlen(m_topic_name),
                                                     &m_msg_id);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("REGISTER message could not be sent. Error code: 0x%x\r\n", err_code);
    }
}

/**@brief Processes DISCONNECT message from a gateway. */
static void disconnected_callback(void)
{
    NRF_LOG_DEBUG("disconnected_callback");
    sleep();
}

/**@brief Processes REGACK message from a gateway.
 *
 * @details This function puts the client in sleep mode.
 *
 * @param[in] p_event Pointer to MQTT-SN event.
 */
static void regack_callback(mqttsn_event_t * p_event)
{
    m_topic.topic_id = p_event->event_data.registered.packet.topic.topic_id;
    NRF_LOG_DEBUG("MQTT-SN event: Topic has been registered with ID: %d.\r\n",
                 p_event->event_data.registered.packet.topic.topic_id);

    sleep();
}

/**@brief Processes PUBACK message from a gateway.
 *
 * @details This function puts the client in sleep mode.
 */
static void puback_callback(void)
{
    NRF_LOG_DEBUG("puback_callback");
    sleep();
}

/**@brief Processes DISCONNECT message being a response to sleep request.
 *
 * @details This function puts the client in sleep mode.
 */
static void sleep_callback(void)
{
    NRF_LOG_DEBUG("sleep_callback");
    sleep();
}

/**@brief Processes callback from keep-alive timer timeout.
 *
 * @details This function puts the client in active mode.
 */
static void wakeup_callback(void)
{
    NRF_LOG_DEBUG("wakeup_callback");
    wake_up();
}

/**@brief Processes retransmission limit reached event. */
static void timeout_callback(mqttsn_event_t * p_event)
{
    NRF_LOG_DEBUG("MQTT-SN event: Timed-out message: %d. Message ID: %d.\r\n",
                  p_event->event_data.error.msg_type,
                  p_event->event_data.error.msg_id);

}

/**@brief Processes results of gateway discovery procedure. */
static void searchgw_timeout_callback(mqttsn_event_t * p_event)
{
    NRF_LOG_DEBUG("MQTT-SN event: Gateway discovery result: 0x%x.\r\n", p_event->event_data.discovery);
    sleep();

}

/**@brief Function for handling MQTT-SN events. */
void mqttsn_evt_handler(mqttsn_client_t * p_client, mqttsn_event_t * p_event)
{
    switch (p_event->event_id)
    {
        case MQTTSN_EVENT_GATEWAY_FOUND:
            NRF_LOG_DEBUG("MQTT-SN event: Client has found an active gateway.");
            gateway_info_callback(p_event);
            connect_mqttsn();
            break;

        case MQTTSN_EVENT_CONNECTED:
            NRF_LOG_DEBUG("MQTT-SN event: Client connected.");
            connected_callback();
            break;

        case MQTTSN_EVENT_DISCONNECTED:
            NRF_LOG_DEBUG("MQTT-SN event: Client disconnected.");
            disconnected_callback();
            break;

        case MQTTSN_EVENT_REGISTERED:
            NRF_LOG_DEBUG("MQTT-SN event: Client registered topic.");
            regack_callback(p_event);
            break;

        case MQTTSN_EVENT_PUBLISHED:
            NRF_LOG_DEBUG("MQTT-SN event: Client has successfully published content.");
            puback_callback();
            break;

        case MQTTSN_EVENT_SLEEP_PERMIT:
            NRF_LOG_DEBUG("MQTT-SN event: Client permitted to sleep.");
            sleep_callback();
            break;

        case MQTTSN_EVENT_SLEEP_STOP:
            NRF_LOG_DEBUG("MQTT-SN event: Client wakes up.");
            wakeup_callback();
            break;

        case MQTTSN_EVENT_TIMEOUT:
            NRF_LOG_DEBUG("MQTT-SN event: Retransmission retries limit has been reached.");
            timeout_callback(p_event);
            break;

        case MQTTSN_EVENT_SEARCHGW_TIMEOUT:
            NRF_LOG_DEBUG("MQTT-SN event: Gateway discovery procedure has finished.");
            searchgw_timeout_callback(p_event);
            break;

        default:
            break;
    }
}

/**@brief Function for initializing the MQTTSN client.
 */
static void mqttsn_init(void)
{
    uint32_t err_code = mqttsn_client_init(&m_client,
                                           MQTTSN_DEFAULT_CLIENT_PORT,
                                           mqttsn_evt_handler,
                                           thread_ot_instance_get());
    APP_ERROR_CHECK(err_code);

    connect_opt_init();
}

static void set_radio_on(void) 
{
    otError error;
    otLinkModeConfig mode;
    memset(&mode, 0, sizeof(mode));

    mode.mRxOnWhenIdle       = true;
    mode.mSecureDataRequests = true;

    error = otThreadSetLinkMode(thread_ot_instance_get(), mode);
    ASSERT(error == OT_ERROR_NONE);
}

static void set_radio_off(void) 
{
    otError error;
    otLinkModeConfig mode;
    memset(&mode, 0, sizeof(mode));

    mode.mRxOnWhenIdle       = false;
    mode.mSecureDataRequests = true;

    error = otThreadSetLinkMode(thread_ot_instance_get(), mode);
    ASSERT(error == OT_ERROR_NONE);
}

/***************************************************************************************************
 * @section SAADC
 **************************************************************************************************/
void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
    nrf_drv_saadc_sample();
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    float offset = 1.6653044224;
    float acc = 0;
    float voltage_rms = 240.0;

    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        for (int i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            float sense_voltage = p_event->data.done.p_buffer[i] * 0.0034655819 - offset;
            float current = (sense_voltage / 150.0) * 2000.0;
            acc += pow(current, 2);
        }

        float current_rms = sqrt(acc / SAMPLES_IN_BUFFER) - 0.045;
        float apparent_power = voltage_rms * current_rms;

        sprintf(buffer, "iRMS: %f\nApparent Power: %f", current_rms, apparent_power);
        NRF_LOG_ERROR("%s", buffer);

        if (mqttsn_client_state_get(&m_client) == MQTTSN_CLIENT_CONNECTED)
        {
            sprintf(buffer, "{ \"i_rms\" : %f, \"power_va\" : %f }", 
                current_rms,
                apparent_power
            );

            // NRF_LOG_ERROR("Publishing ADC state %s", buffer);

            err_code = mqttsn_client_publish(&m_client, m_topic.topic_id, (uint8_t*)&buffer, strlen(buffer), &m_msg_id);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("PUBLISH message could not be sent. Error code: 0x%x\r\n", err_code);
            }
            sleep();
        }
        else if (has_valid_role())
        {
            find_gateway();
        }
    }
}


void saadc_init(void)
{
    ret_code_t err_code;
    uint32_t time_ms = 10;
    uint32_t time_ticks;
    
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t channel_config_a0 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    err_code = nrf_drv_saadc_channel_init(0, &channel_config_a0);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer, SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_ms_to_ticks(&m_timer, time_ms);

    nrf_drv_timer_extended_compare(
        &m_timer, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrf_drv_timer_enable(&m_timer);
}

void otPlatLog(otLogLevel aLogLevel, otLogRegion aLogRegion, const char *aFormat, ...)
{
    OT_UNUSED_VARIABLE(aLogLevel);
    OT_UNUSED_VARIABLE(aLogRegion);
    OT_UNUSED_VARIABLE(aFormat);

    // va_list ap;
    // va_start(ap, aFormat);
    // otCliPlatLogv(aLogLevel, aLogRegion, aFormat, ap);
    // va_end(ap);
}

/***************************************************************************************************
 * @section Main
 **************************************************************************************************/

int main(int argc, char *argv[])
{
    log_init();
    // scheduler_init();
    // timer_init();
    leds_init();

    thread_instance_init();
    thread_bsp_init();

    saadc_init();

    otInstance *instance = thread_ot_instance_get();

    setNetworkConfiguration(instance);

    /* Start the Thread network interface (CLI cmd > ifconfig up) */
    otIp6SetEnabled(instance, true);

    /* Start the Thread stack (CLI cmd > thread start) */
    otThreadSetEnabled(instance, true);

    mqttsn_init();

    while (true)
    {
        thread_process();
        // app_sched_execute();

        if (NRF_LOG_PROCESS() == false)
        {
            thread_sleep();
        }
    }
}

/**
 *@}
 **/

