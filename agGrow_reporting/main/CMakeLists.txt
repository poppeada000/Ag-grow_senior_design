idf_component_register(SRCS "decode_image.c"
                            "pretty_effect.c"
                            "spi_master_main.c"
                            "gatts_server.c"
                            "main.c"
                    INCLUDE_DIRS "."
                    EMBED_FILES image.jpg)