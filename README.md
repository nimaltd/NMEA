# GPS NMEA library
<br />
I hope use it and enjoy.
<br />
I use Stm32f103vc and Keil Compiler and Stm32CubeMX wizard.
 <br />
Please Do This ...
<br />
<br />
1) Config your usart and enable RX interrupt on CubeMX.
<br />
2) Select "General peripheral Initalizion as a pair of '.c/.h' file per peripheral" on project settings.
<br />
3) Config your GpsConfig.h file.
<br />
4) Add Gps_RxCallBack() on usart interrupt routin. 
<br />
5) call  Gps_Init() in your app.
<br />
6) Put Gps_Process() in Loop.

