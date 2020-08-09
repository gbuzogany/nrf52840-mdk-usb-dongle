#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_gpiote.h"
#include "nrf_log_ctrl.h"
#include "nrf_log.h"
#include "nrf_log_default_backends.h"

// ble
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "nrf_pwr_mgmt.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
// end ble

#include "boards.h"
#include "app_error.h"
#include <stdint.h>
#include <stdbool.h>

#define COMPARE_COUNTERTIME  (3UL) /**< Get Compare event COMPARE_TIME seconds after the counter starts from 0. */
#define PIN_IN 19
#define PIN_MOTOR 2

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2);

// Bluetooth definitions

#define DEVICE_NAME                     "Potager Grand-père"                    /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION                18000                                   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);      
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);          

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

static void nus_data_handler(ble_nus_evt_t * p_evt);

// Valve logic

int last_read = 0;

int time_8hz = 0;
int time_sec = 0;
int time_min = 0;

int open_duration_min = 60;
int close_duration_min = 1380;

bool valve_closed = true;
bool valve_should_close = true;

bool motor_on = false;

/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on TICK and COMPARE0 match.
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (valve_closed == true && valve_should_close == true) {
        time_8hz++;
        if (time_8hz >= 8) {
            time_8hz = 0;
            time_sec++;
        }
        if (time_sec >= 60) {
            NRF_LOG_INFO("closed for %d seconds", time_sec);
            time_sec = 0;
            time_min++;
        }
        if (time_min >= close_duration_min) {
            time_min = 0;
            valve_should_close = false;
        }
    }
    
    if (valve_closed == false && valve_should_close == false) {
        time_8hz++;
        if (time_8hz >= 8) {
            time_8hz = 0;
            time_sec++;
        }
        if (time_sec >= 60) {
            NRF_LOG_INFO("open for %d seconds", time_sec);
            time_sec = 0;
            time_min++;
        }
        if (time_min >= open_duration_min) {
            time_min = 0;
            valve_should_close = true;
        }
    }
}

/** @brief Function configuring gpio for pin toggling.
 */
static void leds_config(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void rtc_config(void)
{
    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 4095;
    err_code = nrfx_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //Enable tick event & interrupt
    nrfx_rtc_tick_enable(&rtc,true);

    //Set compare channel to trigger interrupt after COMPARE_COUNTERTIME seconds
    err_code = nrfx_rtc_cc_set(&rtc, 0 ,COMPARE_COUNTERTIME * 8,true);
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrfx_rtc_enable(&rtc);
}

/**@brief Function for initializing the nrf log module.
 */

uint32_t get_rtc_counter(void)
{
    return NRF_RTC2->COUNTER;
}

//////////////////////////////////////////
// BLUETOOTH STUFF

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}

static void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_DEBUG("BLE_ADV_EVT_FAST");
            break;
        case BLE_ADV_EVT_IDLE:
            NRF_LOG_DEBUG("BLE_ADV_EVT_IDLE");
            break;
        default:
            break;
    }
}


static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    NRF_LOG_INFO("ble_advertising_start: %d", err_code);
    APP_ERROR_CHECK(err_code);
}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            NRF_LOG_INFO("Open duration: %d min", open_duration_min);
            NRF_LOG_INFO("Close duration: %d min", close_duration_min);
            // bsp_board_led_on(CONNECTED_LED);
            // bsp_board_led_off(ADVERTISING_LED);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // bsp_board_led_off(CONNECTED_LED);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            // advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}

static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

int parse_int_from_command(ble_nus_evt_rx_data_t const* p_data) 
{
    int val = 0;
    int mul = 1;
    bool found_num = false;
    for (int i=p_data->length-1; i > 0; --i) {
        char curr = p_data->p_data[i];
        if (found_num == false) {
            if (curr >= '0' && curr <= '9') {
                found_num = true;
            }
        }
        if (found_num == true) {
            if (curr >= '0' && curr <= '9') {
                val += (curr - '0') * mul;
                mul *= 10;
            }
            else {
                // end of the number
                break;
            }
        }
    }
    return val;
}

void parse_open_command(ble_nus_evt_rx_data_t const* rx_data) 
{
    time_min = 0;
    time_sec = 0;
    valve_should_close = false;
}

void parse_close_command(ble_nus_evt_rx_data_t const* rx_data) 
{
    time_min = 0;
    time_sec = 0;
    valve_should_close = true;
}

void parse_open_time_command(ble_nus_evt_rx_data_t const* rx_data) 
{
    int val = parse_int_from_command(rx_data);
    NRF_LOG_INFO("updated open duration to %d min", val);
    open_duration_min = val;
}

void parse_close_time_command(ble_nus_evt_rx_data_t const* rx_data) 
{
    int val = parse_int_from_command(rx_data);
    NRF_LOG_INFO("updated close duration to %d min", val);
    close_duration_min = val;
}

void parse_uart_command(ble_nus_evt_rx_data_t const* rx_data) 
{
    NRF_LOG_INFO("Message length: %d", rx_data->length);

    if (rx_data->length > 0) {
        if (rx_data->length > 2) {
            if (rx_data->p_data[0] == 'o' && rx_data->p_data[1] == 't') 
            {
                parse_open_time_command(rx_data);
            }
            else if (rx_data->p_data[0] == 'c' && rx_data->p_data[1] == 't') 
            {
                parse_close_time_command(rx_data);
            }
        }
        if (rx_data->length == 1) {
            if (rx_data->p_data[0] == 'o') 
            {
                parse_open_command(rx_data);
            }
            else if (rx_data->p_data[0] == 'c') 
            {
                parse_close_command(rx_data);
            }
        }
    }
}

static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        // uint32_t err_code;

        NRF_LOG_INFO("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_INFO(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        parse_uart_command(&p_evt->params.rx_data);
    }

}

////////////////////////////////
// Bluetooth END
////////////////////////////////

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(get_rtc_counter);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

void gpio_init() 
{
    nrf_gpio_cfg_input(PIN_IN, NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_cfg_output(PIN_MOTOR);
    // nrf_gpio_cfg_output(BSP_LED_1);
}

/**
 * @brief Function for application main entry.
 */

void update_motor() {
    if (valve_closed == true && valve_should_close == false) 
    {
        if (motor_on == false) {
            NRF_LOG_INFO("Opening valve...");
            // nrf_gpio_pin_write(BSP_LED_1, 0);
            nrf_gpio_pin_write(PIN_MOTOR, 1);
            motor_on = true;
        }
    }
	else if (valve_closed == false && valve_should_close == true) 
    {
        if (motor_on == false) {
            NRF_LOG_INFO("Closing valve...");
            // nrf_gpio_pin_write(BSP_LED_1, 0);
            nrf_gpio_pin_write(PIN_MOTOR, 1);
            motor_on = true;
        }
    }
	else 
    {
        if (motor_on == true) {
            NRF_LOG_INFO("Stopping motor.");
            // nrf_gpio_pin_write(BSP_LED_1, 1);
            nrf_gpio_pin_write(PIN_MOTOR, 0);
            motor_on = false;
        }
    }
}

static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

static void idle_state_handle(void)
{
    int val = nrf_gpio_pin_read(PIN_IN);
    if (val != last_read) {
        last_read = val;
        valve_closed = val;
        NRF_LOG_INFO("changed");
    }
    update_motor();
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

int main(void)
{
    log_init();
    gpio_init();
    leds_config();
    timers_init();
    power_management_init();
    NRF_LOG_INFO("power_management_init");
    ble_stack_init();
    NRF_LOG_INFO("ble_stack_init");
    gap_params_init();
    NRF_LOG_INFO("gap_params_init");
    gatt_init();
    NRF_LOG_INFO("gatt_init");
    services_init();
    NRF_LOG_INFO("services_init");
    advertising_init();
    NRF_LOG_INFO("advertising_init");
    conn_params_init();
    NRF_LOG_INFO("conn_params_init");
    rtc_config();
    NRF_LOG_INFO("rtc_config");

    advertising_start();
    NRF_LOG_INFO("advertising_start");

    valve_closed = !nrf_gpio_pin_read(PIN_IN);

    while (true)
    {
        idle_state_handle();
    }
}


/**  @} */
