# AndruinoApp
Andruino App is a wired and wireless home sensors for iPhone.

This repository contains the source code to be uploaded on Arduino to be used with the iPhone AndruinoApp

Arduino (Hardware+firmware) + AndruinoApp (downloadable from AppStore)

site web:www.andruino.it

Description:
buying a low cost Arduino open-source electronics prototyping platform, you can control all what you want in your home simply connecting Arduino board to ethernet plug.

Andruino is the first App in which you can drag&drop the sensors/controls over your real home map.

You can connect whatever analog/digital sensors on hardware board and Andruino App communicates with them, displaying in real time the room temperatures (C°/K°), the lights/plugs power consumption (Watt), the kitchen humidity (%), opened doors, and any other informations that comes from your home.
You can switch ON/OFF relays, control PWM outputs, and create pulses.

Andruino is 100% customizable:<br>
• Maps: home map can be uploaded from camera roll. Size and zoom can be modified.<br>
• Ambients: different Arduino board numbers can be managed with Andruino Ambients.<br>
• Wireless: supports ZigBee wireless modules<br>
• Sensor configuration: measure unit, mathematical formula, number of decimals can be managed according to the sensor requirements. Each sensor/relay can be full configurable (both analog/digital/pwm). No sensor/controls limit number (wired and wireless).<br>
• Input events: Low/high limits can trigger visual alarm, push notifications and output activations.<br>
• Arduino can send real time Push Notifications when an event is triggered and/or a limit is reached. You can control your sensors also when AndruinoApp is closed. A notification with all the sensor details will rise on your iPhone (Push notifications are sent also when Andruino-App is closed)<br>
• The digital outputs can be triggered when a limit is reached. This option can be used to activate a relay when a temperature is reached. The activation can be triggered by any analog/digital/variable resource.<br>
• Graphs: real time sensor monitoring. Each sensor can be plotted on a graph. Average/Max/Min value are calculated (graphs are updated only with the App opened) <br>
• TIMERS: each output pin can be configured with a timer to switch on at desired time (hour, minute, repeat day) and with a specific duration (0 to 18 hours). The time is synchronized every time AndruinoApp is connected and the clock is done using simply using the Arduino internal oscillator (no RTC is required). <br>
• Output temporization: all Arduino outputs PIN can be temporized (from 1 sec to 18 hours) <br>
• Predefined sensors: DHT22, Dallas DS18B20, current sensors etc, have predefined settings with detailed informations used to simplify the electrical connection and configuration.<br>
• iCloud support: AndruinoApp setting are under iCloud, so you can share the same setting using different devices.<br>
• Data communication: HTTP JSON data-interchange format used to communicate over ethernet. Network access restricted with username and password.<br>
• Supported Arduino boards: Arduino UNO, Arduino Mega 2560, Arduino YUN, and compatible boards. Official Arduino Wifi shield is supported but the connection is too slow.<br>
• Arduino firmware is open source, you can download it from Andruino website support (firmware page). You can add new features, improve it as you want.<br>
