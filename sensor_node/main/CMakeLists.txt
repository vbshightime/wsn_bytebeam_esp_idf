idf_component_register(
    INCLUDE_DIRS 
        "."
        "sht"
    SRCS 
        "app_main.c"
        "sht/sht31.c"
        "espnow_example.h"    
    PRIV_REQUIRES
        "esp_wifi"	
        "driver"
        "nvs_flash"
        "app_update"
        )
