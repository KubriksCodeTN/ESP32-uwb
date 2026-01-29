idf_component_register(SRCS "FP1.c"
                    INCLUDE_DIRS 
                        "." 
                        "../components/gc9a01" 
                        "../components/dispcolor"
                        "../components/lsm6dsox" 
                        "../components/lis2mdl"
                        "../components/sht4x"
                        "../components/tsl2591"
                    REQUIRES driver
                    REQUIRES esp_wifi
                    REQUIRES nvs_flash)

