// posix headers
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <errno.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <assert.h>
#include <netdb.h>
#include <fcntl.h>
#include <unistd.h>

// esp32 freeRTOS headers
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "nvs_flash.h"

// sensors headers
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"
#include "sht4x_i2c.h"
#include "tsl2591.h"
#include "lis2mdl_reg.h"
#include "lsm6dsox_reg.h"
#include "dispcolor.h"


/* HW config macros */
#define SDA 13 // SDA PIN
#define SCL 14 // SCL PIN
#define I2C_FREQ 100000
#define PWM_FREQ 1000
#define GPIO_PWM_DISPLAY 4
#define GPIO_HOME_PAGE 25
#define GPIO_SCROLL_DOWN 26
#define GPIO_ENTER 27
#define GPIO_BUT_BIT_MASK ((1ULL << GPIO_HOME_PAGE) | (1ULL << GPIO_SCROLL_DOWN) | (1ULL << GPIO_ENTER)) // Bitmask for buttons gpio

// WIFI macros
#define WIFI_SUCCESS 1
#define WIFI_FAILURE 2
#define MAX_FAILURES 10
#define WIFI_BUFFSIZE 1024 // WIFI message buffer size

#define MAX_DEVICES 5 // max num  uwb devices to display
#define N_TASKS 7
#define MSEC(x) ((x) / portTICK_PERIOD_MS) // to convert x ticks to milliseconds in freeRTOS

// mutex to control task to put to sleep
SemaphoreHandle_t xMutex;

// enum of possibe buttons
enum button_t{
    go_home,
    scroll_down,
    enter
};
typedef enum button_t button_t;

// possible screen guis
enum screen_t{
    home_page,
    compass_screen,
    gyro_accellerometer_screen,
    uwb_page
};
typedef enum screen_t screen_t;

/* constant variables (safer than macros + used by many tasks) */
// I2C port in use
static const i2c_port_t I2C_PORT = I2C_NUM_0;
// settings for pwm timer (for display brightness)
// is a global constant cause the tasks needs to read info about the channel
static const ledc_timer_config_t pwm_tim_cfg = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_12_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = PWM_FREQ
};
// pwm channel in use
static const ledc_channel_t PWM_CHANNEL = LEDC_CHANNEL_0;

// WIFI typdefs
typedef struct sockaddr sockaddr;
typedef struct sockaddr_in sockaddr_in;
typedef struct addr_in addr_in;
// event group to contain status information
static EventGroupHandle_t wifi_event_group;
// retry tracker to reconnect to access point
static int s_retry_num = 0;

/* shared/needed variables for the tasks */
// tsl22591 device handler
static tsl2591_t tsl2591;

// button cmds
static QueueHandle_t gpio_but_queue = NULL;

// battery variable TO BE KILLED
static volatile uint8_t battery;

// temp humidity queue
static QueueHandle_t temp_humid_queue = NULL;
// temperature and humidity struct (together becouse sensor reads both in the same call)
typedef struct aa{
    float temp;
    float humid;
} temp_humid_measure_t;

// Brightness queue
static QueueHandle_t brightness_queue = NULL;

// common struct to store values from magnetometer, accellerometer and gyroscope
// everyone returns 3 values, one per axis
typedef struct {
    float x_axis;
    float y_axis;
    float z_axis;
} measurement_3_axis_t;

// magnetometer struct to store 3 axis values
typedef measurement_3_axis_t magnetometer_measure_t;
static QueueHandle_t magnetometer_queue = NULL;

// gyroscope struct to store 3 axis values
typedef measurement_3_axis_t gyro_measure_t;
static QueueHandle_t gyro_queue = NULL;

// accellerometer struct to store 3 axis values
typedef measurement_3_axis_t accelerometer_measure_t;
static QueueHandle_t accellerometer_queue = NULL;

// struct to store UWB devices, represented as an array of points
// with coordinates relative to this device, which is at (0, 0)
typedef struct {
    int8_t size;
    float x_coord[MAX_DEVICES];
    float y_coord[MAX_DEVICES];
} uwb_devices_list_t;
static QueueHandle_t uwb_devices_queue = NULL;

/* stm32 aux functions */
// since stm32 sensors all use the same protocol we just create a template with variable addresses
int32_t stm32_generic_write_reg(stmdev_ctx_t* ctx, uint8_t addr, uint8_t reg, uint8_t* data, uint16_t len){
    int err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_PORT, cmd, MSEC(1000));
    i2c_cmd_link_delete(cmd);
    return err;
}

int32_t stm32_generic_read_reg(stmdev_ctx_t* ctx, uint8_t addr, uint8_t reg, uint8_t* data, uint16_t len){
    int err = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_PORT, cmd, MSEC(1000));
    i2c_cmd_link_delete(cmd);
    return err;
}

// override of weak functions
int32_t lis2mdl_write_reg(stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data, uint16_t len){
    reg |= 0x80U; // assume always multiple write cmd
    return stm32_generic_write_reg(ctx, LIS2MDL_I2C_ADD, reg, data, len);
}

int32_t lis2mdl_read_reg(stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data, uint16_t len){
    reg |= 0x80U; // assume always multiple read cmd
    return stm32_generic_read_reg(ctx, LIS2MDL_I2C_ADD, reg, data, len);
}

// aux fun to read magnetoneter data
int32_t lis2mdl_read_magnetometers(magnetometer_measure_t* output){
    uint8_t arr[6] = {0};
    int32_t ret = lis2mdl_read_reg(NULL, LIS2MDL_OUTX_L_REG, &arr[0], 6);

    output->x_axis = lis2mdl_from_lsb_to_mgauss((arr[1] << 8) | arr[0]);
    output->y_axis = lis2mdl_from_lsb_to_mgauss((arr[3] << 8) | arr[2]);
    output->z_axis = lis2mdl_from_lsb_to_mgauss((arr[5] << 8) | arr[4]);

    return ret;
}

int32_t lsm6dsox_write_reg(stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data, uint16_t len){
    return stm32_generic_write_reg(ctx, LSM6DSOX_I2C_ADD_L, reg, data, len);
}

int32_t lsm6dsox_read_reg(stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data, uint16_t len){
    return stm32_generic_read_reg(ctx, LSM6DSOX_I2C_ADD_L, reg, data, len);
}

// aux fun to read accellerometer data
int32_t lsm6dsox_read_acceloremeter(accelerometer_measure_t* output){
    int16_t val[3] = {0};
    int ret = lsm6dsox_acceleration_raw_get(NULL, val);

    output->x_axis = lsm6dsox_from_fs4_to_mg(val[0]);
    output->y_axis = lsm6dsox_from_fs4_to_mg(val[1]);
    output->z_axis = lsm6dsox_from_fs4_to_mg(val[2]);

    // ESP_LOGI("LOG:", "ret: %d, accel : %f", ret, output->z);

    return ret;
}
// aux fun to read gyro data
int32_t lsm6dsox_read_gyro(gyro_measure_t* output){
    int16_t val[3] = {0};
    int ret = lsm6dsox_angular_rate_raw_get(NULL, val);

    output->x_axis = lsm6dsox_from_fs500_to_mdps(val[0]);
    output->y_axis = lsm6dsox_from_fs500_to_mdps(val[1]);
    output->z_axis = lsm6dsox_from_fs500_to_mdps(val[2]);

    // ESP_LOGI("LOG:", "ret: %d, rot : %f", ret, output->z);

    return ret;
}

/* tsl2591 functions wrappers to power ON/OFF the device */
// Enables the chip, so it's ready to take readings
esp_err_t tsl2591_enable(tsl2591_t* dev){
    esp_err_t errno;
    // power ON
    do{ errno = tsl2591_set_power_status(dev, TSL2591_POWER_ON); } while (errno == ESP_ERR_TIMEOUT);
    if (errno)
        return errno;
    // set als on
    do{ errno = tsl2591_set_als_status(&tsl2591, TSL2591_ALS_ON); } while (errno == ESP_ERR_TIMEOUT);
    return errno;
}
// Disables the chip, so it's in power down mode
esp_err_t tsl2591_disable(tsl2591_t* dev){
    esp_err_t errno;
    do{ errno = tsl2591_set_power_status(dev, TSL2591_POWER_OFF); } while (errno == ESP_ERR_TIMEOUT);
    return errno;
}

/* setup routines */
static inline void setup_i2c(void){
    i2c_config_t i2c_mconf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA, // select SDA GPIO
        .sda_pullup_en = 1,
        .scl_io_num = SCL, // select SCL GPIO
        .scl_pullup_en = 1,
        .master.clk_speed = I2C_FREQ, // select frequency
        .clk_flags = 0,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &i2c_mconf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));
}

static inline void setup_lis2mdl(void){
    uint8_t rst;
    /* Restore default configuration */
    ESP_ERROR_CHECK(lis2mdl_reset_set(NULL, PROPERTY_ENABLE));
    do { lis2mdl_reset_get(NULL, &rst); } while (rst);

    /* Enable Block Data Update */
    ESP_ERROR_CHECK(lis2mdl_block_data_update_set(NULL, PROPERTY_ENABLE));
    /* Set Output Data Rate */
    ESP_ERROR_CHECK(lis2mdl_data_rate_set(NULL, LIS2MDL_ODR_10Hz));
    /* Set / Reset sensor mode */
    ESP_ERROR_CHECK(lis2mdl_set_rst_mode_set(NULL, LIS2MDL_SENS_OFF_CANC_EVERY_ODR));
    /* Enable temperature compensation */
    ESP_ERROR_CHECK(lis2mdl_offset_temp_comp_set(NULL, PROPERTY_ENABLE));
    /*  */
    ESP_ERROR_CHECK(lis2mdl_operating_mode_set(NULL, LIS2MDL_POWER_DOWN));
    /* Set device in continuous mode */
    // ESP_ERROR_CHECK(lis2mdl_operating_mode_set(NULL, LIS2MDL_CONTINUOUS_MODE));
}

static inline void setup_lsm6dsox(void){
    uint8_t rst;
    /* Restore default configuration */
    ESP_ERROR_CHECK(lsm6dsox_reset_set(NULL, PROPERTY_ENABLE));
    do { lsm6dsox_reset_get(NULL, &rst); } while (rst);

    /* Disable I3C interface */
    ESP_ERROR_CHECK(lsm6dsox_i3c_disable_set(NULL, LSM6DSOX_I3C_DISABLE));
    /* Enable Block Data Update */
    ESP_ERROR_CHECK(lsm6dsox_block_data_update_set(NULL, PROPERTY_ENABLE));
    /* Set Output Data Rate */
    ESP_ERROR_CHECK(lsm6dsox_xl_data_rate_set(NULL, LSM6DSOX_XL_ODR_OFF));
    ESP_ERROR_CHECK(lsm6dsox_gy_data_rate_set(NULL, LSM6DSOX_GY_ODR_OFF));
    // ESP_ERROR_CHECK(lsm6dsox_xl_data_rate_set(NULL, LSM6DSOX_XL_ODR_12Hz5));
    // ESP_ERROR_CHECK(lsm6dsox_gy_data_rate_set(NULL, LSM6DSOX_GY_ODR_12Hz5));
    /* Set full scale */
    ESP_ERROR_CHECK(lsm6dsox_xl_full_scale_set(NULL, LSM6DSOX_4g));
    ESP_ERROR_CHECK(lsm6dsox_gy_full_scale_set(NULL, LSM6DSOX_500dps));
    /* Enable timestamp */
    ESP_ERROR_CHECK(lsm6dsox_timestamp_set(NULL, PROPERTY_ENABLE));
    /*
    * Configure filtering chain(No aux interface)
    * Accelerometer - LPF1 + LPF2 path
    */
    ESP_ERROR_CHECK(lsm6dsox_xl_hp_path_on_out_set(NULL, LSM6DSOX_LP_ODR_DIV_100));
    ESP_ERROR_CHECK(lsm6dsox_xl_filter_lp2_set(NULL, PROPERTY_ENABLE));
}

static inline void setup_sht4x(void){
    sensirion_i2c_hal_port_init(I2C_PORT);
    sht4x_init(SHT40_I2C_ADDR_44);
    ESP_ERROR_CHECK(sht4x_soft_reset());
}

static inline void setup_tsl2591(void){
    ESP_ERROR_CHECK(tsl2591_init_desc(&tsl2591, I2C_PORT, SDA, SCL, I2C_FREQ));
    ESP_ERROR_CHECK(tsl2591_init(&tsl2591));
    // ESP_ERROR_CHECK(tsl2591_set_power_status(&tsl2591, TSL2591_POWER_ON));
    // ESP_ERROR_CHECK(tsl2591_set_als_status(&tsl2591, TSL2591_ALS_ON));
    ESP_ERROR_CHECK(tsl2591_set_gain(&tsl2591, TSL2591_GAIN_MEDIUM));
    ESP_ERROR_CHECK(tsl2591_set_integration_time(&tsl2591, TSL2591_INTEGRATION_200MS));
    // by default the chip is in power down mode
    // ESP_ERROR_CHECK(tsl2591_set_power_status(dev, TSL2591_POWER_OFF));
}

static inline void setup_but_gpio(void){
    // initialize the buttons gpio config structure.
    gpio_config_t gpio_button_cfg = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = GPIO_BUT_BIT_MASK,
        .pull_up_en = 1,
        .pull_down_en = 0
    };
    // configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_config(&gpio_button_cfg));
}

static inline void setup_pwm(void){
    ESP_ERROR_CHECK(ledc_timer_config(&pwm_tim_cfg));
    ledc_channel_config_t pwm_chan_cfg = {
        .gpio_num = GPIO_PWM_DISPLAY,
        .speed_mode = pwm_tim_cfg.speed_mode,
        .channel = PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE, // no need for interrupt
        .timer_sel = pwm_tim_cfg.timer_num,
        .duty = pwm_tim_cfg.duty_resolution / 2
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_chan_cfg));
}

// event handler for wifi events
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
    int32_t event_id, void* event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START){
		// ESP_LOGI("Wifi", "Connecting to AP...");
		esp_wifi_connect();
        return;
	}

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED){
		if (s_retry_num < MAX_FAILURES){
			// ESP_LOGI("Wifi", "Reconnecting to AP...");
			esp_wifi_connect();
			++s_retry_num;
            return;
		}
        xEventGroupSetBits(wifi_event_group, WIFI_FAILURE);
	}
}

// event handler for ip events
static void ip_event_handler(void* arg, esp_event_base_t event_base,
    int32_t event_id, void* event_data){
	if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP){

        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("Wifi", "STA IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_SUCCESS);
    }
}

// connect to wifi and return the result
esp_err_t connect_wifi(){
	int status = WIFI_FAILURE;

	/** INITIALIZE ALL THE THINGS **/
	//initialize the esp network interface
	ESP_ERROR_CHECK(esp_netif_init());

	//initialize default esp event loop
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	//create wifi station in the wifi driver
	esp_netif_create_default_wifi_sta();

	//setup wifi station with the default wifi configuration
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /** EVENT LOOP CRAZINESS **/
	wifi_event_group = xEventGroupCreate();

    esp_event_handler_instance_t wifi_handler_event_instance;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        &wifi_handler_event_instance));

    esp_event_handler_instance_t got_ip_event_instance;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &ip_event_handler,
        NULL,
        &got_ip_event_instance));

    /** START THE WIFI DRIVER **/
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "Vodafone-C00180028",
            .password = "3ffJKL3gENzxTpxK",
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    // set the wifi controller to be a station
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // set the wifi config
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );

    // start the wifi driver
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI("Wifi", "STA initialization complete");

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
            WIFI_SUCCESS | WIFI_FAILURE,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_SUCCESS) {
        ESP_LOGI("Wifi", "Connected to ap");
        status = WIFI_SUCCESS;
    } else if (bits & WIFI_FAILURE) {
        ESP_LOGI("Wifi", "Failed to connect to ap");
        status = WIFI_FAILURE;
    } else {
        ESP_LOGE("Wifi", "UNEXPECTED EVENT");
        status = WIFI_FAILURE;
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, got_ip_event_instance));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_handler_event_instance));
    vEventGroupDelete(wifi_event_group);

    return status;
}

void setup_wifi(){

    esp_err_t status = WIFI_FAILURE;

	// initialize storage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // connect to wireless AP
	status = connect_wifi();
	if (WIFI_SUCCESS != status){
		ESP_LOGI("Wifi", "Failed to associate to AP, dying...");
		return;
    }

}


/* tasks */

// buttons interrupt handler
// debouncing + possible way of handling of simultaneous button presses
// TODO see if it's the intended behaviour
static void IRAM_ATTR gpio_isr_handler(void* gpio_pin){
    static TickType_t last_time = 0;

    TickType_t now = xTaskGetTickCountFromISR();
    if (pdTICKS_TO_MS(now - last_time) <= 150)
        return;
    last_time = now;
    button_t but = (button_t)gpio_pin;
    xQueueSendToBackFromISR(gpio_but_queue, &but, NULL);
}

void temp_humid_task(void* pvParams){
    while (4){

        // this sensor operates only if you write to him otherwise is idle
        int32_t temp, humid;
        sht4x_measure_high_precision(&temp, &humid);
        temp_humid_measure_t res = {
            .temp = temp / 1000., // millideg to deg
            .humid = humid / 1000. // milli percent to percent
        };

        xQueueOverwrite(temp_humid_queue, &res);

        // ESP_LOGI("LOGI", "New humid: %f", res.humid);

        vTaskDelay(MSEC(5000));
    }
}

// battery status task
void battery_task(void* pvParams){
    while (4){
        battery = rand() % 100;
        // ESP_LOGI("LOGI", "New battery: %hhu", battery);

        vTaskDelay(MSEC(10000));
    }

}

void uwb_task(void* pvParams){
    while (4){
        uwb_devices_list_t dev_list = {0};

        // read from uwb..

        // for(int i=0; i<MAX_DEVICES; ++i){
        //     dev_list.x_coord[dev_list.size] = (rand()%2000) - 1000; // 10m
        //     dev_list.y_coord[dev_list.size] = (rand()%2000) - 1000;
        //     dev_list.size++;
        // }

        dev_list.x_coord[0] = 250;
        dev_list.y_coord[0] = 0;

        dev_list.x_coord[1] = 0;
        dev_list.y_coord[1] = 250;

        dev_list.x_coord[2] = 1000;
        dev_list.y_coord[2] = 750;

        dev_list.x_coord[3] = -490;
        dev_list.y_coord[3] = 0;

        dev_list.size = 4;

        xQueueOverwrite(uwb_devices_queue, &dev_list);

        vTaskDelay(MSEC(4000));
    };
}

// TODO enable / disable sensor?
void brightness_task(void* pvParams){
    uint32_t duty, old_duty = 0;

    while (4){
        float val;
        esp_err_t errno;

        ESP_ERROR_CHECK(tsl2591_enable(&tsl2591));
        tsl2591_integration_time_t integration_time;
        do{ errno = tsl2591_get_integration_time(&tsl2591, &integration_time); } while(errno == ESP_ERR_TIMEOUT);
        ESP_ERROR_CHECK(errno);
        vTaskDelay((integration_time + 1) * 120);

        do{ errno = tsl2591_get_lux(&tsl2591, &val); } while(errno == ESP_ERR_TIMEOUT);
        ESP_ERROR_CHECK(errno);

        xQueueOverwrite(brightness_queue, &val);

        val /= 100;
        val = 100 * (val + 1);
        val = val / 2000 * (1 << pwm_tim_cfg.duty_resolution);
        if (val >= 1 << pwm_tim_cfg.duty_resolution)
            val = (1 << pwm_tim_cfg.duty_resolution) - 1;

        // limit val insinde [200, 3000] and map into [100, 8192]
        // val = (val < 200) ? 200 : (val > 3000) ? 3000 : val;
        // val = ((1 << 12)-100)/(3000.-200) * val + 100;

        // ESP_LOGI("LOG", "BRIGHTNESS: %f\n", val);

        duty = (uint32_t)val;
        if (duty != old_duty){
            ledc_set_duty(pwm_tim_cfg.speed_mode, PWM_CHANNEL, duty);
            ledc_update_duty(pwm_tim_cfg.speed_mode, PWM_CHANNEL);
        }

        old_duty = duty;

        ESP_ERROR_CHECK(tsl2591_disable(&tsl2591));
        vTaskDelay(MSEC(1000));
    }
}

// if these functions timeout they don't poll cause of the mutex
void magnetometer_task(void* pvParams){
    vTaskSuspend(NULL);
    magnetometer_measure_t res;

    while (4){
        if (xSemaphoreTake(xMutex, 0) == pdTRUE){
            lis2mdl_read_magnetometers(&res);

            xQueueOverwrite(magnetometer_queue, &res);
            xSemaphoreGive(xMutex);
        }
        vTaskDelay(MSEC(150));
    }
}

void acc_gyro_task(void* pvParams){
    vTaskSuspend(NULL);
    gyro_measure_t res_g = {0};
    accelerometer_measure_t res_a = {0};

    while (4){
        if (xSemaphoreTake(xMutex, 0) == pdTRUE){
            lsm6dsox_read_gyro(&res_g);
            lsm6dsox_read_acceloremeter(&res_a);

            xQueueOverwrite(accellerometer_queue, &res_a);
            xQueueOverwrite(gyro_queue, &res_g);
            xSemaphoreGive(xMutex);
        }
        vTaskDelay(MSEC(150));
    }
}

void wifi_client_task(void* pvParams){
    int sock;
    char msg[WIFI_BUFFSIZE] = {0};
    struct addrinfo *gai_res;

    const char* server_addr = "192.168.1.12";
    const char* port = "50001";

    temp_humid_measure_t temp_hum = {0};
    float brightness = 0;
    magnetometer_measure_t magn = {0};
    gyro_measure_t gyro = {0};
    accelerometer_measure_t accel = {0};
    uwb_devices_list_t uwb = {0};

    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        ESP_ERROR_CHECK(sock);

    ESP_ERROR_CHECK(getaddrinfo(server_addr, port, NULL, &gai_res));

    ESP_ERROR_CHECK(connect(sock, gai_res->ai_addr, gai_res->ai_addrlen));

    while (4){

        xQueuePeek(temp_humid_queue, &temp_hum, 0);
        xQueuePeek(brightness_queue, &brightness, 0);
        xQueuePeek(magnetometer_queue, &magn, 0);
        xQueuePeek(gyro_queue, &gyro, 0);
        xQueuePeek(accellerometer_queue, &accel, 0);
        xQueuePeek(uwb_devices_queue, &uwb, 0);

        // JSON message
        int sprintf_wb = sprintf(
            msg,
            "{"
            "\"temperature\": %3.1f,\n"
            "\"humidity\": %3.1f,\n"
            "\"brightness\": %3.1f,\n"
            "\"magnetometer\": [%3.1f, %3.1f, %3.1f],\n"
            "\"gyroscope\": [%3.1f, %3.1f, %3.1f],\n"
            "\"accellerometer\": [%3.1f, %3.1f, %3.1f],\n"
            "\"uwb_devices\": [",
            temp_hum.temp,
            temp_hum.humid,
            brightness,
            magn.x_axis,
            magn.y_axis,
            magn.z_axis,
            gyro.x_axis,
            gyro.y_axis,
            gyro.z_axis,
            accel.x_axis,
            accel.y_axis,
            accel.z_axis
        );

        for (int i = 0; i < uwb.size; ++i){
            sprintf_wb += sprintf(
                msg + sprintf_wb,
                "{\"x\": %3.1f, \"y\": %3.1f}",
                uwb.x_coord[i],
                uwb.y_coord[i]
            );
            if (i < uwb.size - 1)
                sprintf_wb += sprintf(
                msg + sprintf_wb,
                ","
            );
        }

        sprintf(msg + sprintf_wb, "]}");

        int wb = write(sock, msg, strlen(msg));
        if (wb < 0)
            ESP_ERROR_CHECK(9);

        memset(msg, 0, WIFI_BUFFSIZE);
        read(sock, msg, WIFI_BUFFSIZE-1);
        ESP_LOGI("WIFI", "Server responded '%s'\n-----\n", msg);

        vTaskDelay(MSEC(2000));
    }
}

// DRAW FUNCTIONS
void draw_battery_and_background(){
    dispcolor_FillCircle(120, 120, 120, battery > 75 ? GREEN : battery > 20 ? YELLOW : RED);
    dispcolor_FillCircle(120, 120, 110, BLACK);
}

void draw_home_page(void* pvscreen_idx){
    static const char* opt[] = {
        "Home page",
        "Compass",
        "Gyroscope",
        "Compass and UWB"
    };

    static const char* ptr = "> ";
    uint8_t screen_idx = *(uint8_t*)pvscreen_idx;

    static temp_humid_measure_t temp_humid_res = {0};
    xQueuePeek(temp_humid_queue, &temp_humid_res, 0);

    static float brightness_res = 0;
    xQueuePeek(brightness_queue, &brightness_res, 0);

    int i = screen_idx;
    if (screen_idx == 0)
        i++;
    else if (screen_idx == sizeof(opt) / sizeof(char*) - 1)
        i--;
    i--;

    draw_battery_and_background();

    for (int k = 0; k < 3; ++k, ++i)
        dispcolor_printf(45, 45 + 20 * k, 1, WHITE, "%s%s", (i != screen_idx) ? "" : ptr, opt[i]);


    dispcolor_printf(45, 45 + 20 * 3 + 35, 1, WHITE, "Temperature: %3.1fC", temp_humid_res.temp);
    dispcolor_printf(45, 45 + 20 * 3 + 55, 1, WHITE, "Humidity: %3.1f%%", temp_humid_res.humid);
    dispcolor_printf(45, 45 + 20 * 3 + 75, 1, WHITE, "Brightness: %3.1f", brightness_res);
}

void draw_compass_page(void* pvdeg){
    static magnetometer_measure_t data = {0};
    xQueuePeek(magnetometer_queue, &data, 0);

    float rad = atan2f(data.y_axis, data.x_axis) + M_PI;
    float deg = rad * (180 / M_PI);

    // ESP_LOGI("DEG","%f", deg);

    // mapping 0deg (Nord) and 90deg (East) to the correct direction on the screen

    /* since the origin is in the top-left corner, the angles must be specified
    in clockwise direction, here I convert them from counter-clockwisw direction*/
    deg = 360. - deg;
    rad = 2 * M_PI - rad;
    // needed to map north(0 deg) to the correct direction on the screen
    deg -= 90;
    rad -= M_PI / 2;

    if (deg < 0){
        deg += 360;
        rad += 2 * M_PI;
    }

    const int radius = 80;

    int line_1_dest_x = radius * cos(rad);
    int line_1_dest_y = radius * sin(rad);

    int line_2_dest_x = line_1_dest_x;
    int line_2_dest_y = line_1_dest_y;

    // default (West)
    int line_1_start_x = 119;
    int line_1_start_y = 119;
    int line_2_start_x = 119;
    int line_2_start_y = 120;


    // nord is 270 deg on the screen
    if (deg >= 235. || deg < 315.){
        line_1_start_x = 119;
        line_1_start_y = 119;
        line_2_start_x = 120;
        line_2_start_y = 119;
    }

    // east
    else if (deg >= 315. && deg < 45.){
        line_1_start_x = 120;
        line_1_start_y = 120;
        line_2_start_x = 120;
        line_2_start_y = 119;
    }

    // south
    else if (deg >= 45. && deg < 135.){
        line_1_start_x = 119;
        line_1_start_y = 120;
        line_2_start_x = 120;
        line_2_start_y = 120;
    }

    draw_battery_and_background();

    dispcolor_printf(115, 15, 1, WHITE, "N");
    dispcolor_printf(115, 210, 1, WHITE, "S");
    dispcolor_printf(15, 112, 1, WHITE, "W");
    dispcolor_printf(215, 112, 1, WHITE, "E");


    dispcolor_DrawRectangle(119, 119, 120, 120, RED);

    dispcolor_DrawLine(line_1_start_x, line_1_start_y, line_1_start_x + line_1_dest_x ,line_1_start_y + line_1_dest_y, RED);
    dispcolor_DrawLine(line_2_start_x, line_2_start_y, line_2_start_x + line_2_dest_x, line_1_start_y + line_2_dest_y, RED);
}

void draw_gyro_accellerometer_page(void* pvarg){
    static accelerometer_measure_t accellerometer_res = {0};
    xQueuePeek(accellerometer_queue, &accellerometer_res, 0);

    static gyro_measure_t gyro_res = {0};
    xQueuePeek(gyro_queue, &gyro_res, 0);

    draw_battery_and_background();

    dispcolor_printf(45, 45, 1, WHITE, "Accel x: %3.1f mG", accellerometer_res.x_axis);
    dispcolor_printf(45, 45 + 20, 1, WHITE, "Accel y: %3.1f mG", accellerometer_res.y_axis);
    dispcolor_printf(45, 45 + 20 * 2, 1, WHITE, "Accel z: %3.1f mG", accellerometer_res.z_axis);

    dispcolor_printf(45, 45 + 20 * 3, 1, WHITE, "Ang vel x: %3.1f mdps", gyro_res.x_axis);
    dispcolor_printf(45, 45 + 20 * 4, 1, WHITE, "Ang vel y: %3.1f mdps", gyro_res.y_axis);
    dispcolor_printf(45, 45 + 20 * 5, 1, WHITE, "Ang vel z: %3.1f mdps", gyro_res.z_axis);
}

void draw_uwb_page(void* pvdeg){

    static uwb_devices_list_t dev;

    xQueuePeek(uwb_devices_queue, &dev, 0);

    draw_compass_page(pvdeg);

    const float radius = 80;
    const float max_distance_from_me = 500.;
    // ESP_LOGI("DEVS", "%d", dev.size);

    float cm_to_pixel_x;
    float cm_to_pixel_y;

    // dispcolor_DrawCircle(120, 120, radius, YELLOW, 0);


    for (int i = 0; i < dev.size; ++i){

        cm_to_pixel_x = dev.x_coord[i];
        cm_to_pixel_y = dev.y_coord[i];

        uint16_t color = MAGENTA;
        // restrict to 5m radius from this device
        if (fabs(dev.x_coord[i]) < max_distance_from_me
            && fabs(dev.y_coord[i]) < max_distance_from_me){

            cm_to_pixel_x = cm_to_pixel_x / max_distance_from_me * radius;
            cm_to_pixel_y = cm_to_pixel_y / max_distance_from_me * radius;
            color = WHITE;
        }

        else{
            float hyp = sqrt(dev.x_coord[i] * dev.x_coord[i] + dev.y_coord[i] * dev.y_coord[i]);

            cm_to_pixel_x = cm_to_pixel_x / hyp * radius;
            cm_to_pixel_y = cm_to_pixel_y / hyp * radius;
        }
        // ESP_LOGI("POINT", "%f - %f - %s", cm_to_pixel_x + 120,cm_to_pixel_y+ 120, i==1 ? "magenta" : "white");
        dispcolor_FillCircle(cm_to_pixel_x + 120, 120 - cm_to_pixel_y, 3, color);
    }
}

void app_main(void){
    // i2c setup
    setup_i2c();
    // setup temperature/humidity sensors
    setup_sht4x();
    // setup magnetometer
    setup_lis2mdl();
    // brightness sensor setup
    setup_tsl2591();
    // gyroscope
    setup_lsm6dsox();
    // wifi
    setup_wifi();
    // initialize the buttons gpio config structure.
    setup_but_gpio();
    // initialize pwm signal
    setup_pwm();
    // wait for config
    vTaskDelay(MSEC(500));

    // mutex creation
    xMutex = xSemaphoreCreateMutex();

    // create a queue to handle gpio event from isr
    gpio_but_queue = xQueueCreate(10, sizeof(button_t));
    temp_humid_queue = xQueueCreate(1, sizeof(temp_humid_measure_t));
    brightness_queue = xQueueCreate(1, sizeof(float));
    magnetometer_queue = xQueueCreate(1, sizeof(magnetometer_measure_t));
    gyro_queue = xQueueCreate(1, sizeof(gyro_measure_t));
    accellerometer_queue = xQueueCreate(1, sizeof(accelerometer_measure_t));
    uwb_devices_queue = xQueueCreate(1, sizeof(uwb_devices_list_t));
    // install gpio isr service
    gpio_install_isr_service(0);
    // hook isr handler for specific gpio pin
    // NB the void* address is to be read as an integer to get the right button enum
    gpio_isr_handler_add(GPIO_HOME_PAGE, gpio_isr_handler, (void*)go_home);
    gpio_isr_handler_add(GPIO_SCROLL_DOWN, gpio_isr_handler, (void*)scroll_down);
    gpio_isr_handler_add(GPIO_ENTER, gpio_isr_handler, (void*)enter);

    // setup periodic tasks

    TaskHandle_t task[N_TASKS];
    xTaskCreate(brightness_task, "lux", 2048, NULL, 1, &task[0]);
    xTaskCreate(temp_humid_task, "temp_humid", 2048, NULL, 1, &task[1]);
    xTaskCreate(battery_task, "battery", 2048, NULL, 1, &task[2]);
    xTaskCreate(magnetometer_task, "magnetometer", 2048, NULL, 1, &task[3]);
    xTaskCreate(acc_gyro_task, "acc_gyro", 2048, NULL, 1, &task[4]);
    xTaskCreate(uwb_task, "uwb", 2048, NULL, 1, &task[5]);
    xTaskCreate(wifi_client_task, "wifi", 8192, NULL, 1, &task[6]);

    for (uint32_t i = 0; i < N_TASKS; ++i)
        configASSERT(task[i]);

    // the main becomes the screen/task manager
    // Screen handler
    screen_t current_screen = home_page;
    button_t but;
    int8_t screen_idx = 0;
    esp_err_t errno;

    static const screen_t screen_array[] = {home_page, compass_screen, gyro_accellerometer_screen, uwb_page};
    static void(*const draw[])(void*) = {draw_home_page, draw_compass_page, draw_gyro_accellerometer_page, draw_uwb_page};

    dispcolor_Init(240, 240);
    draw_home_page(&screen_idx);
    while (4) {
        if (xQueueReceive(gpio_but_queue, &but, 0)){
            switch(but){
                case go_home:
                    if (current_screen == gyro_accellerometer_screen){
                        do {} while(xSemaphoreTake(xMutex, 0) == pdFALSE);
                        vTaskSuspend(task[4]);
                        xSemaphoreGive(xMutex);
                        // these waits are to avoid a weird behaviour (bug of esp-idf?) with the I2C mutex,
                        // the system panics if you take it too fast after another thread (???)
                        do { errno = lsm6dsox_xl_data_rate_set(NULL, LSM6DSOX_XL_ODR_OFF); } while(errno == ESP_ERR_TIMEOUT);
                        do { errno = lsm6dsox_gy_data_rate_set(NULL, LSM6DSOX_GY_ODR_OFF); } while(errno == ESP_ERR_TIMEOUT);
                        vTaskDelay(MSEC(10));
                    }
                    if (current_screen == compass_screen || current_screen == uwb_page){
                        do {} while(xSemaphoreTake(xMutex, 0) == pdFALSE);
                        vTaskSuspend(task[3]);
                        xSemaphoreGive(xMutex);
                        do { errno = lis2mdl_operating_mode_set(NULL, LIS2MDL_POWER_DOWN); } while(errno == ESP_ERR_TIMEOUT);
                        vTaskDelay(MSEC(10));
                    }
                    current_screen = home_page;
                    break;
                case scroll_down:
                    if (current_screen == home_page)
                        screen_idx = (screen_idx + 1) % (sizeof(screen_array) / sizeof(screen_t));
                    break;
                case enter:
                    current_screen = screen_array[screen_idx];
                    if (current_screen == gyro_accellerometer_screen){
                        do { errno = lsm6dsox_xl_data_rate_set(NULL, LSM6DSOX_XL_ODR_12Hz5); } while(errno == ESP_ERR_TIMEOUT);
                        do { errno = lsm6dsox_gy_data_rate_set(NULL, LSM6DSOX_GY_ODR_12Hz5); } while(errno == ESP_ERR_TIMEOUT);
                        vTaskDelay(MSEC(10));
                        do {} while(xSemaphoreTake(xMutex, 0) == pdFALSE);
                        vTaskResume(task[4]);
                        xSemaphoreGive(xMutex);
                    }
                    if (current_screen == compass_screen || current_screen == uwb_page){
                        do { errno = lis2mdl_operating_mode_set(NULL, LIS2MDL_CONTINUOUS_MODE); } while(errno == ESP_ERR_TIMEOUT);
                        vTaskDelay(MSEC(10));
                        do {} while(xSemaphoreTake(xMutex, 0) == pdFALSE);
                        vTaskResume(task[3]);
                        xSemaphoreGive(xMutex);
                    }
                    break;
            }
        }

        draw[current_screen](&screen_idx);
        dispcolor_Update();
        vTaskDelay(MSEC(150));
    }
}
