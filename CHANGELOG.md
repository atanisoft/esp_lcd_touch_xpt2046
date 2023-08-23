# Change log for esp_lcd_touch_xpt2046

## v1.0.2 - New features

* Set minimum version for esp_lcd_touch to 1.0.4.
* Add touch interrupt callback functionality.
* Add abilitty to read voltage level from vBat pin.
* Enable ADC during read of X/Y coords (Z already was enabled).
* Add IDF v5.1 esp_lcd_panel_io_spi_config_t updates.
* Fixing interrupt gpio pin reference during initialization.

## v1.0.1 - Bug fix release

* Fixing typo in configuration options.
* Removing usage of temporary array of coordinates.
* Adjustments to ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG for IDF v5.0-rc1.
* Add option to return raw coordinate data instead of converting to screen coordinates.

## v1.0.0 - Initial release

This release is the first revision of this component.