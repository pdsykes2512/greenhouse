#define SSID "Fallowfield"
#define PSK "mock.united.marathons"
#define MySQL
#define mysql_url "http://192.168.11.101/adddata.php"
#define SYSTEM_HOSTNAME "Greenhouse"
#define FLASH_PWD "p0UmrR7k6xrp0wzfGNEP"
#define TELNET_DEBUG
#define on      1
#define off     0
#define open    1
#define closed  0

#define PUMP_RELAY      D6

// Plant 1
#define PLANT1_ACTIVE       on
#define TEMP1_SENSOR        0
#define SOIL1_PIN           A0
#define VALVE1_RELAY        D5
#define PLANT1_SOIL_DRY     40
#define PLANT1_SOIL_WET     95
#define PLANT1_WATER_ON     10
#define PLANT1_WATER_OFF    (6 * 60 * 60)

// Plant 2
#define PLANT2_ACTIVE       on
#define TEMP2_SENSOR        1
#define SOIL2_PIN           A1
#define VALVE2_RELAY        D4
#define PLANT2_SOIL_DRY     40
#define PLANT2_SOIL_WET     95
#define PLANT2_WATER_ON     20
#define PLANT2_WATER_OFF    (6 * 60 * 60)

// Plant 3
#define PLANT3_ACTIVE       on
#define TEMP3_SENSOR        2
#define SOIL3_PIN           A2
#define VALVE3_RELAY        D3
#define PLANT3_SOIL_DRY     40
#define PLANT3_SOIL_WET     95
#define PLANT3_WATER_ON     10
#define PLANT3_WATER_OFF    (6 * 60 * 60)

// Plant 4
#define PLANT4_ACTIVE       on
#define TEMP4_SENSOR        3
#define SOIL4_PIN           A3
#define VALVE4_RELAY        D2
#define PLANT4_SOIL_DRY     40
#define PLANT4_SOIL_WET     95
#define PLANT4_WATER_ON     20
#define PLANT4_WATER_OFF    (6 * 60 * 60)

// Watering variables
#define WATER1_LEVEL_SENSOR A6

#define OLED_ADDRESS 0x3C // initialize with the I2C addr 0x3C Typically eBay OLED's
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels