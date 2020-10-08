###############################################################################
# Select libraries needed to be installed
# 
# If INSTALL_LIBRARIES_* is y, the library will be compiled and installed to
# your system.
# If INSTALL_LIBRARIES_* is not "y", the library will be ignored.
#
# NOTE:
#    Blank spaces behind letter "y" in lines which begin with INSTALL_LIBRARIES_*
# will cause some exceptions unexpectedly. In other words, Letter "y" MUST be
# the last one in those lines.
###############################################################################

#
# User can replace "y" with "n" to ignore the corresponding library.
# Actually, commenting the line which begins with INSTALL_LIBRARIES_* by
# inserting "#" at the head will also get the same result.
# NOTE:
#     User MUST select one INSTALL_LIBRARIES_* as "y" at least.
#
INSTALL_LIBRARIES_itead_GSM-GPRS-GPS    := n
INSTALL_LIBRARIES_itead_Nextion      := y
INSTALL_LIBRARIES_itead_LiquidCrystal   := n
INSTALL_LIBRARIES_itead_SSD1306         := n
INSTALL_LIBRARIES_itead_GFX             := n
INSTALL_LIBRARIES_itead_TinyGPS         := n
INSTALL_LIBRARIES_itead_PN532_SPI       := n
INSTALL_LIBRARIES_itead_nRF24L01-lite   := n
INSTALL_LIBRARIES_itead_NRF24L01P		:= n
INSTALL_LIBRARIES_itead_SHT1x			:= n
INSTALL_LIBRARIES_itead_IoTgo			:= n