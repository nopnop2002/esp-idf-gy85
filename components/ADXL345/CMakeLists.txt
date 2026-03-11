idf_component_register(
	SRCS "ADXL345.cpp"
	INCLUDE_DIRS "."
	PRIV_REQUIRES driver esp_driver_i2c esp_timer
	REQUIRES I2Cdev
)
