idf_component_register(SRCS "decode_image.c"
                            "pretty_effect.c"
                            "gatts_server.c"
                            "spi_master_main.c"
                            "main.c"
                            "i2c_main.c"
                            "vl53l0x.c"
                    INCLUDE_DIRS "."
                    EMBED_FILES image.jpg)