.. _nrf52_audio_opus_sgtl5000:

nrf52_audio_opus_sgtl5000
###########

Overview
********

The audio recorder record audio from SGTL5000 on I2S interface, encode PCM buffer with opus codec, and stream the opus frame on radio (simple custom Frequency hopping protocol using Long range radio).

The audio player receive opus encoded frame, decode it and play the PCM decoded buffer on SGTL5000 I2S interface.



Hardware Requirements
************
rf52840 DK board
Audio Adaptor Board for Teensy 3.0 - 3.6 : https://www.pjrc.com/store/teensy3_audio.html


Building and Running
********************

Get the application source :
git clone https://github.com/ubicore/nrf52_audio_opus_sgtl5000.git

Get the zephyr fork :
git clone https://github.com/zephyrproject-rtos/zephyr.git
checkout nrf52_audio_opus_sgtl5000


To compile the Audio player
.. zephyr-app-commands::
   :zephyr-app: nrf52_audio_opus_sgtl5000
   :board: nrf52840_pca10056
   :conf: "prj.conf overlay-cpustats.conf overlay-audio.conf overlay-oled.conf overlay-sgtl5000.conf"

To compile the Audio recorder
.. zephyr-app-commands::
   :zephyr-app: nrf52_audio_opus_sgtl5000
   :board: nrf52840_pca10056
   :conf: "prj.conf overlay-cpustats.conf overlay-audio.conf overlay-oled.conf overlay-sgtl5000.conf overlay-audio_recorder.conf"


